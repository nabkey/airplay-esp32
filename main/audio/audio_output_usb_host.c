// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * USB host output — AirPlay through an external USB sound card
 *
 * The ESP32 is the USB host and the sound card (or USB DAC) is the device:
 * the opposite direction from audio_output_usb.c, where the board enumerates
 * as a microphone on a computer.  Uses the espressif/usb_host_uac managed
 * component, which owns the isochronous OUT endpoint and its own ring.
 *
 * Unlike the UAC device backend this is a push model: the playback task
 * blocks in uac_host_device_write() once the driver's ring is full, exactly
 * as the I2S backend blocks on DMA space.  That keeps the ring at a steady
 * occupancy, which is what audio_output_get_hardware_latency_us() reports.
 *
 * Cards come and go at runtime.  The driver's callbacks only post events to
 * a queue; opening, starting and closing a card happens on the control task,
 * because the driver does not allow those calls from its own callback.  While
 * no card is attached the playback task keeps pulling from the source at the
 * output rate and drops the samples, so the timeline advances as if the
 * audio had played and a card plugged in mid-stream joins in sync.
 */

#include "audio_output.h"

#include "audio_receiver.h"
#include "audio_resample.h"
#include "led.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "rtsp_server.h"
#include "usb/uac_host.h"
#include "usb/usb_host.h"
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#define TAG           "audio_usbh"
#define FRAME_SAMPLES 352

/* Preferred card rate; 44.1 kHz is accepted as a fallback, in which case
 * AirPlay plays without resampling. */
#define PREFERRED_RATE CONFIG_OUTPUT_SAMPLE_RATE_HZ
#define FALLBACK_RATE  44100

/* Max output frames after resampling one input frame */
#define MAX_RESAMPLE_FRAMES \
  ((size_t)((FRAME_SAMPLES + 2) * ((double)PREFERRED_RATE / 44100) + 16))

#if CONFIG_FREERTOS_UNICORE
#define PLAYBACK_CORE 0
#else
#define PLAYBACK_CORE 1
#endif

/* The USB host library and class driver are event plumbing, not audio: they
 * sit below the playback task and the source tasks on the other core. */
#define USB_LIB_TASK_PRIORITY 6
#define USB_CTL_TASK_PRIORITY 5
#define USB_TASK_CORE         0

#define BYTES_PER_FRAME 4 /* stereo 16-bit */

/* Driver-side ring.  The playback task keeps it full, so its size is the
 * steady-state output latency: ~20 ms is deep enough to ride out a source
 * burst and shallow enough not to matter to the timing engine. */
#define DRIVER_RING_MS 20
#define DRIVER_RING_BYTES(rate) \
  ((uint32_t)(rate) * DRIVER_RING_MS / 1000 * BYTES_PER_FRAME)

/* Longest a write may block before the task re-checks the card state. */
#define WRITE_TIMEOUT_MS 50

typedef enum {
  EVT_DRIVER_CONNECTED,
  EVT_DEVICE_DISCONNECTED,
} usbh_evt_type_t;

typedef struct {
  usbh_evt_type_t type;
  uint8_t addr;
  uint8_t iface_num;
  uac_host_device_handle_t handle;
} usbh_evt_t;

static QueueHandle_t s_evt_queue;

/* Held by the playback task across a write and by the control task across
 * open/close, so a card is never closed under an in-flight write. */
static SemaphoreHandle_t s_dev_lock;
static uac_host_device_handle_t s_dev;
static uint32_t s_dev_rate;

static volatile bool flush_requested = false;
static volatile int source_rate = 44100;
static volatile bool resample_reinit_needed = false;
static volatile uint32_t s_output_rate = PREFERRED_RATE;
static volatile uint32_t s_underruns = 0;

/* ── Driver callbacks (driver task context: post and return) ────────── */

static void uac_device_cb(uac_host_device_handle_t handle,
                          const uac_host_device_event_t event, void *arg) {
  (void)arg;
  if (event == UAC_HOST_DRIVER_EVENT_DISCONNECTED) {
    usbh_evt_t evt = {.type = EVT_DEVICE_DISCONNECTED, .handle = handle};
    xQueueSend(s_evt_queue, &evt, 0);
  } else if (event == UAC_HOST_DEVICE_EVENT_TRANSFER_ERROR) {
    ESP_LOGW(TAG, "Transfer error");
  }
}

static void uac_driver_cb(uint8_t addr, uint8_t iface_num,
                          const uac_host_driver_event_t event, void *arg) {
  (void)arg;
  /* Only playback interfaces are interesting; a headset's microphone
   * (RX_CONNECTED) is left unopened. */
  if (event == UAC_HOST_DRIVER_EVENT_TX_CONNECTED) {
    usbh_evt_t evt = {
        .type = EVT_DRIVER_CONNECTED, .addr = addr, .iface_num = iface_num};
    xQueueSend(s_evt_queue, &evt, 0);
  }
}

/* ── Card setup ────────────────────────────────────────────────────── */

static bool alt_supports_rate(const uac_host_dev_alt_param_t *alt,
                              uint32_t rate) {
  if (alt->sample_freq_type == 0) {
    return rate >= alt->sample_freq_lower && rate <= alt->sample_freq_upper;
  }
  for (int i = 0; i < alt->sample_freq_type && i < UAC_FREQ_NUM_MAX; i++) {
    if (alt->sample_freq[i] == rate) {
      return true;
    }
  }
  return false;
}

/* Pick the card's rate: the preferred rate if any 16-bit stereo alternate
 * setting offers it, else the fallback.  Returns 0 if neither is offered. */
static uint32_t pick_rate(uac_host_device_handle_t handle, uint8_t alt_count) {
  const uint32_t rates[] = {PREFERRED_RATE, FALLBACK_RATE};
  for (size_t r = 0; r < sizeof(rates) / sizeof(rates[0]); r++) {
    for (uint8_t alt = 1; alt <= alt_count; alt++) {
      uac_host_dev_alt_param_t param;
      if (uac_host_get_device_alt_param(handle, alt, &param) != ESP_OK) {
        continue;
      }
      if (param.channels == 2 && param.bit_resolution == 16 &&
          alt_supports_rate(&param, rates[r])) {
        return rates[r];
      }
    }
  }
  return 0;
}

static void open_card(uint8_t addr, uint8_t iface_num) {
  if (s_dev) {
    ESP_LOGW(TAG, "Ignoring second sound card (addr %u)", addr);
    return;
  }

  uac_host_device_handle_t handle = NULL;
  const uac_host_device_config_t dev_cfg = {
      .addr = addr,
      .iface_num = iface_num,
      .buffer_size = DRIVER_RING_BYTES(PREFERRED_RATE),
      .buffer_threshold = 0,
      .callback = uac_device_cb,
      .callback_arg = NULL,
  };
  esp_err_t err = uac_host_device_open(&dev_cfg, &handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Open failed: %s", esp_err_to_name(err));
    return;
  }

  uac_host_dev_info_t info;
  err = uac_host_get_device_info(handle, &info);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Device info failed: %s", esp_err_to_name(err));
    uac_host_device_close(handle);
    return;
  }
  ESP_LOGI(TAG, "Sound card %04x:%04x \"%ls\" connected", info.VID, info.PID,
           info.iProduct);

  uint32_t rate = pick_rate(handle, info.iface_alt_num);
  if (rate == 0) {
    ESP_LOGE(TAG, "Card offers no 16-bit stereo stream at %d or %d Hz",
             PREFERRED_RATE, FALLBACK_RATE);
    uac_host_printf_device_param(handle);
    uac_host_device_close(handle);
    return;
  }

  const uac_host_stream_config_t stream_cfg = {
      .channels = 2,
      .bit_resolution = 16,
      .sample_freq = rate,
      .flags = 0,
  };
  err = uac_host_device_start(handle, &stream_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Stream start failed: %s", esp_err_to_name(err));
    uac_host_device_close(handle);
    return;
  }

  xSemaphoreTake(s_dev_lock, portMAX_DELAY);
  s_dev = handle;
  s_dev_rate = rate;
  xSemaphoreGive(s_dev_lock);

  if (rate != s_output_rate) {
    s_output_rate = rate;
    resample_reinit_needed = true;
  }
  ESP_LOGI(TAG, "Streaming to card at %" PRIu32 " Hz", rate);
}

static void close_card(uac_host_device_handle_t handle) {
  xSemaphoreTake(s_dev_lock, portMAX_DELAY);
  if (handle == s_dev) {
    s_dev = NULL;
    s_dev_rate = 0;
  }
  xSemaphoreGive(s_dev_lock);
  uac_host_device_close(handle);
  ESP_LOGI(TAG, "Sound card disconnected");
}

/* ── Event tasks ───────────────────────────────────────────────────── */

static void usb_lib_task(void *arg) {
  TaskHandle_t ctl_task = (TaskHandle_t)arg;
  const usb_host_config_t host_cfg = {
      .skip_phy_setup = false,
      .intr_flags = ESP_INTR_FLAG_LOWMED,
  };
  esp_err_t err = usb_host_install(&host_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "USB host install failed: %s", esp_err_to_name(err));
    vTaskDelete(NULL);
    return;
  }
  xTaskNotifyGive(ctl_task);

  while (true) {
    uint32_t flags = 0;
    usb_host_lib_handle_events(portMAX_DELAY, &flags);
  }
}

static void usb_ctl_task(void *arg) {
  (void)arg;
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  const uac_host_driver_config_t uac_cfg = {
      .create_background_task = true,
      .task_priority = USB_LIB_TASK_PRIORITY,
      .stack_size = 4096,
      .core_id = USB_TASK_CORE,
      .callback = uac_driver_cb,
      .callback_arg = NULL,
  };
  esp_err_t err = uac_host_install(&uac_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "UAC host install failed: %s", esp_err_to_name(err));
    vTaskDelete(NULL);
    return;
  }
  ESP_LOGI(TAG, "Waiting for a USB sound card");

  usbh_evt_t evt;
  while (true) {
    if (xQueueReceive(s_evt_queue, &evt, portMAX_DELAY) != pdTRUE) {
      continue;
    }
    switch (evt.type) {
    case EVT_DRIVER_CONNECTED:
      open_card(evt.addr, evt.iface_num);
      break;
    case EVT_DEVICE_DISCONNECTED:
      close_card(evt.handle);
      break;
    }
  }
}

/* ── Volume ────────────────────────────────────────────────────────── */

static void apply_volume(int16_t *buf, size_t n) {
#ifndef CONFIG_DAC_CONTROLS_VOLUME
  // Same exponential ramp as the other backends: an abrupt gain step is an
  // audible click, so approach the target once per stereo frame.
  static int32_t cur_q15 = -1;
  int32_t target = airplay_get_volume_q15();
  if (cur_q15 < 0) {
    cur_q15 = target;
  }
  for (size_t i = 0; i < n; i++) {
    if ((i & 1) == 0 && cur_q15 != target) {
      int32_t diff = target - cur_q15;
      int32_t step = diff / 256;
      if (step == 0) {
        step = diff > 0 ? 1 : -1;
      }
      cur_q15 += step;
    }
    buf[i] = (int16_t)(((int32_t)buf[i] * cur_q15) >> 15);
  }
#endif
}

/* ── Playback task ─────────────────────────────────────────────────── */

/* Hand one block to the card.  Returns false if there is no card to take
 * it, in which case the caller paces itself instead. */
static bool write_card(int16_t *buf, size_t frames) {
  bool written = false;
  xSemaphoreTake(s_dev_lock, portMAX_DELAY);
  if (s_dev) {
    esp_err_t err = uac_host_device_write(s_dev, (uint8_t *)buf,
                                          (uint32_t)(frames * BYTES_PER_FRAME),
                                          pdMS_TO_TICKS(WRITE_TIMEOUT_MS));
    if (err == ESP_OK) {
      written = true;
    } else if (err == ESP_FAIL) {
      /* Ring stayed full for the whole timeout: the card stopped pulling. */
      s_underruns++;
    }
  }
  xSemaphoreGive(s_dev_lock);
  return written;
}

static void playback_task(void *arg) {
  (void)arg;
  int16_t *pcm = malloc((size_t)(FRAME_SAMPLES + 1) * 2 * sizeof(int16_t));
  int16_t *silence = calloc((size_t)FRAME_SAMPLES * 2, sizeof(int16_t));
  int16_t *resample_buf = malloc(MAX_RESAMPLE_FRAMES * 2 * sizeof(int16_t));
  if (!pcm || !silence || !resample_buf) {
    ESP_LOGE(TAG, "Failed to allocate buffers");
    free(pcm);
    free(silence);
    free(resample_buf);
    vTaskDelete(NULL);
    return;
  }

  TickType_t last_wake = xTaskGetTickCount();
  while (true) {
    if (resample_reinit_needed) {
      resample_reinit_needed = false;
      audio_resample_init((uint32_t)source_rate, s_output_rate, 2);
    }
    if (flush_requested) {
      flush_requested = false;
      audio_resample_reset();
      /* The driver ring cannot be drained from here; at ~20 ms it plays
       * out faster than a seek can be heard. */
    }

    int16_t *play_buf = silence;
    size_t play_samples = FRAME_SAMPLES;
    size_t samples = audio_output_read_source(pcm, FRAME_SAMPLES + 1);
    if (samples > 0) {
      play_buf = pcm;
      play_samples = samples;
      if (audio_resample_is_active()) {
        play_samples = audio_resample_process(pcm, samples, resample_buf,
                                              MAX_RESAMPLE_FRAMES);
        play_buf = resample_buf;
      }
      apply_volume(play_buf, play_samples * 2);
    }
    led_audio_feed(play_buf, play_samples);

    if (write_card(play_buf, play_samples)) {
      last_wake = xTaskGetTickCount();
      continue;
    }

    /* No card: consume at the output rate so the timeline keeps moving. */
    TickType_t period =
        pdMS_TO_TICKS((uint32_t)(play_samples * 1000U / s_output_rate));
    vTaskDelayUntil(&last_wake, period > 0 ? period : 1);
  }
}

/* ── Public API ────────────────────────────────────────────────────── */

esp_err_t audio_output_init(void) {
  ESP_LOGI(TAG, "Initialising USB host audio output (rate=%d)", PREFERRED_RATE);

  s_evt_queue = xQueueCreate(8, sizeof(usbh_evt_t));
  s_dev_lock = xSemaphoreCreateMutex();
  if (!s_evt_queue || !s_dev_lock) {
    ESP_LOGE(TAG, "Failed to create queue or lock");
    return ESP_ERR_NO_MEM;
  }

  TaskHandle_t ctl_task = NULL;
  if (xTaskCreatePinnedToCore(usb_ctl_task, "usbh_ctl", 4096, NULL,
                              USB_CTL_TASK_PRIORITY, &ctl_task,
                              USB_TASK_CORE) != pdPASS ||
      xTaskCreatePinnedToCore(usb_lib_task, "usbh_lib", 4096, ctl_task,
                              USB_LIB_TASK_PRIORITY, NULL,
                              USB_TASK_CORE) != pdPASS) {
    ESP_LOGE(TAG, "Failed to create USB host tasks");
    return ESP_ERR_NO_MEM;
  }

  audio_resample_init(44100, PREFERRED_RATE, 2);
  return ESP_OK;
}

void audio_output_start(void) {
  xTaskCreatePinnedToCore(playback_task, "usbh_play", 4096, NULL,
                          AUDIO_PLAYBACK_TASK_PRIORITY, NULL, PLAYBACK_CORE);
}

void audio_output_flush(void) {
  flush_requested = true;
}

void audio_output_set_source_rate(int rate) {
  if (rate > 0 && rate != source_rate) {
    source_rate = rate;
    resample_reinit_needed = true;
  }
}

uint32_t audio_output_get_hardware_latency_us(void) {
  // The playback task keeps the driver ring full, so a new write is heard
  // after one ring's worth of audio.  The card's own buffering on top of
  // that is not reported by UAC 1.0 and is not modelled here.
  return DRIVER_RING_MS * 1000U;
}

uint32_t audio_output_get_underruns(void) {
  return s_underruns;
}

/* The UAC host driver exposes no transfer-completion cursor and there is no
 * channel routing, so the weak defaults in audio_output_common.c cover the
 * rest of the API. */
