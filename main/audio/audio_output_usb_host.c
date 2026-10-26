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
 * occupancy.
 *
 * Queue depth is measured, not modelled.  The driver posts TX_DONE once its
 * ring is at or below the threshold after it refills a finished isochronous
 * transfer; opening with the threshold one byte under the ring size makes
 * that every completion, which serves as the I2S backend's DMA-complete
 * interrupt does.  Frames written minus frames the bus has finished sending
 * is the depth of the ring plus the transfers in flight, interpolated
 * between completions at the output rate.
 *
 * The card follows the source's sample rate.  AirPlay arrives at 44.1 kHz
 * and Sendspin usually at 48 kHz; when the source changes rate the stream is
 * restarted at the new one if the card offers it, so neither is resampled.
 * Only a rate the card lacks goes through the resampler, to 48 kHz or
 * failing that 44.1 kHz.  The restart costs a few tens of milliseconds of
 * silence, and only when the rate actually changes.
 *
 * The pipeline is 16-bit throughout.  Cards that only take 24- or 32-bit
 * samples get the same audio widened on the way out: UAC 1.0 samples are
 * MSB-justified in their subframe, so the 16-bit sample goes in the top two
 * bytes and the rest are zero.
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
#include "esp_timer.h"
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

/* Card rates to resample to when the card does not offer the source's
 * own, in order of preference. */
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

#define CHANNELS     2
#define MAX_SUBFRAME 4 /* bytes per sample on the wire, 32-bit worst case */

/* Driver-side ring.  The playback task keeps it full, so its size is the
 * steady-state output latency: ~20 ms is deep enough to ride out a source
 * burst and shallow enough not to matter to the timing engine.
 *
 * One byte over whole frames: everything in and out of the ring is whole
 * frames, so it never fills to the last byte, and TX_DONE (sent while the
 * ring is under the threshold of size - 1) fires on every completion.  A
 * frame-exact ring loses the event whenever the blocked playback task
 * refills it to full before the driver checks, about one in 20. */
#define DRIVER_RING_MS 20
#define DRIVER_RING_BYTES(rate, subframe) \
  ((uint32_t)(rate) * DRIVER_RING_MS / 1000 * CHANNELS * (subframe) + 1U)

/* Longest a write may block before the task re-checks the card state. */
#define WRITE_TIMEOUT_MS 50

/* Isochronous transfers the driver keeps queued past its ring, each one
 * packet per 1 ms full-speed frame. */
#define URB_COUNT   CONFIG_UAC_NUM_ISOC_URBS
#define URB_PACKETS CONFIG_UAC_NUM_PACKETS_PER_URB
#define URB_US      (URB_PACKETS * 1000U)

typedef enum {
  EVT_DRIVER_CONNECTED,
  EVT_DEVICE_DISCONNECTED,
  EVT_SOURCE_RATE, /* the source changed rate; the card may follow */
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
static uint8_t s_dev_subframe;

static volatile bool flush_requested = false;
static volatile int source_rate = 44100;
static volatile bool resample_reinit_needed = false;
static volatile uint32_t s_output_rate = PREFERRED_RATE;
static volatile uint32_t s_underruns = 0;

/* Queue-depth accounting, guarded by s_queue_lock: the playback task adds
 * what it writes, the driver task what the bus has sent, and the timing
 * engine reads both. */
static portMUX_TYPE s_queue_lock = portMUX_INITIALIZER_UNLOCKED;
static uint64_t s_frames_written;
static uint64_t s_frames_sent;
static int64_t s_sent_us;        /* esp_timer at the last completion */
static uint32_t s_urb_frac;      /* fractional frames, as the driver keeps */
static uint32_t s_card_delay_us; /* the card's own bDelay */
static TaskHandle_t s_play_task;

static void queue_reset(uint32_t card_delay_us) {
  portENTER_CRITICAL(&s_queue_lock);
  s_frames_written = 0;
  s_frames_sent = 0;
  s_sent_us = 0;
  s_urb_frac = 0;
  s_card_delay_us = card_delay_us;
  portEXIT_CRITICAL(&s_queue_lock);
}

/* Frames in one finished transfer.  At 44.1 kHz packets alternate between
 * 44 and 45 frames; tracking the remainder the same way the driver does
 * keeps the count within a frame of what it actually sent. */
static uint32_t urb_frames(uint32_t rate) {
  uint32_t frames = 0;
  for (int i = 0; i < URB_PACKETS; i++) {
    frames += rate / 1000;
    s_urb_frac += rate % 1000;
    if (s_urb_frac >= 1000) {
      frames++;
      s_urb_frac -= 1000;
    }
  }
  return frames;
}

/* ── Driver callbacks (driver task context: post and return) ────────── */

static void uac_device_cb(uac_host_device_handle_t handle,
                          const uac_host_device_event_t event, void *arg) {
  (void)arg;
  if (event == UAC_HOST_DEVICE_EVENT_TX_DONE) {
    /* uac_host_device_write() also posts this, from the playback task, when
     * it restarts transfers that ran dry.  That is not a completion. */
    if (xTaskGetCurrentTaskHandle() == s_play_task) {
      return;
    }
    const int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_queue_lock);
    s_frames_sent += urb_frames(s_output_rate);
    s_sent_us = now;
    portEXIT_CRITICAL(&s_queue_lock);
  } else if (event == UAC_HOST_DRIVER_EVENT_DISCONNECTED) {
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

/* The alternate setting uac_host_device_start() will claim for this format,
 * found the same way it does: later discrete-rate matches replace earlier
 * ones and a matching range ends the search.  Its subframe size decides the
 * byte layout, so it has to be the one the driver picks. */
static bool find_alt(uac_host_device_handle_t handle, uint8_t alt_count,
                     uint8_t bits, uint32_t rate,
                     uac_host_dev_alt_param_t *out) {
  bool found = false;
  for (uint8_t alt = 1; alt <= alt_count; alt++) {
    uac_host_dev_alt_param_t param;
    if (uac_host_get_device_alt_param(handle, alt, &param) != ESP_OK ||
        param.channels != CHANNELS || param.bit_resolution != bits ||
        !alt_supports_rate(&param, rate)) {
      continue;
    }
    *out = param;
    found = true;
    if (param.sample_freq_type == 0) {
      break;
    }
  }
  return found;
}

typedef struct {
  uint32_t rate;
  uint8_t bits;
  uint8_t subframe;
} card_format_t;

/* The card's format at one rate: the narrowest depth it offers there, since
 * anything wider only pads zeros. */
static bool pick_depth(uac_host_device_handle_t handle, uint8_t alt_count,
                       uint32_t rate, card_format_t *fmt) {
  const uint8_t depths[] = {16, 24, 32};
  for (size_t d = 0; d < sizeof(depths) / sizeof(depths[0]); d++) {
    uac_host_dev_alt_param_t param;
    if (!find_alt(handle, alt_count, depths[d], rate, &param)) {
      continue;
    }
    if (param.subframe_size * 8 < depths[d] || param.subframe_size < 2 ||
        param.subframe_size > MAX_SUBFRAME) {
      continue;
    }
    fmt->rate = rate;
    fmt->bits = depths[d];
    fmt->subframe = param.subframe_size;
    return true;
  }
  return false;
}

/* Rates to try for a source at `want`: its own first, then the resampling
 * targets.  Returns how many were written, duplicates dropped. */
static size_t rate_candidates(uint32_t want, uint32_t out[3]) {
  const uint32_t all[] = {want, PREFERRED_RATE, FALLBACK_RATE};
  size_t n = 0;
  for (size_t i = 0; i < sizeof(all) / sizeof(all[0]); i++) {
    bool dup = false;
    for (size_t j = 0; j < n; j++) {
      dup |= out[j] == all[i];
    }
    if (!dup && all[i] > 0) {
      out[n++] = all[i];
    }
  }
  return n;
}

/* The format start_card() would choose for a source at `want`. */
static bool choose_format(uac_host_device_handle_t handle, uint8_t alt_count,
                          uint32_t want, card_format_t *fmt) {
  uint32_t rates[3];
  const size_t n = rate_candidates(want, rates);
  for (size_t i = 0; i < n; i++) {
    if (pick_depth(handle, alt_count, rates[i], fmt)) {
      return true;
    }
  }
  return false;
}

static esp_err_t open_iface(uint8_t addr, uint8_t iface_num,
                            uint32_t buffer_size,
                            uac_host_device_handle_t *handle) {
  const uac_host_device_config_t dev_cfg = {
      .addr = addr,
      .iface_num = iface_num,
      .buffer_size = buffer_size,
      /* One byte short of full: TX_DONE on every transfer completion
       * (see DRIVER_RING_BYTES). */
      .buffer_threshold = buffer_size - 1,
      .callback = uac_device_cb,
      .callback_arg = NULL,
  };
  return uac_host_device_open(&dev_cfg, handle);
}

/* What the class driver does not expose: the endpoint's synchronisation
 * type and the card's bDelay, read from the configuration descriptor through
 * a short-lived client of our own.  Both come from the first streaming
 * alternate setting; cards do not vary them between settings. */
typedef struct {
  int sync_type; /* USB_BM_ATTRIBUTES_SYNC_* >> 2, or -1 if not found */
  uint8_t delay_ms;
} card_timing_t;

static void client_event_cb(const usb_host_client_event_msg_t *msg, void *arg) {
  (void)msg;
  (void)arg;
}

static card_timing_t read_card_timing(uint8_t addr, uint8_t iface_num) {
  card_timing_t timing = {.sync_type = -1, .delay_ms = 0};
  const usb_host_client_config_t client_cfg = {
      .is_synchronous = false,
      .max_num_event_msg = 3,
      .async = {.client_event_callback = client_event_cb, .callback_arg = NULL},
  };
  usb_host_client_handle_t client = NULL;
  if (usb_host_client_register(&client_cfg, &client) != ESP_OK) {
    return timing;
  }
  usb_device_handle_t dev = NULL;
  const usb_config_desc_t *cfg = NULL;
  if (usb_host_device_open(client, addr, &dev) == ESP_OK &&
      usb_host_get_active_config_descriptor(dev, &cfg) == ESP_OK) {
    const uint8_t *p = (const uint8_t *)cfg;
    const size_t total = cfg->wTotalLength;
    bool in_stream = false;
    for (size_t off = 0; off + 2 <= total && p[off] >= 2; off += p[off]) {
      const uint8_t len = p[off];
      const uint8_t type = p[off + 1];
      if (off + len > total) {
        break;
      }
      if (type == USB_B_DESCRIPTOR_TYPE_INTERFACE && len >= 4) {
        if (in_stream) {
          break; /* first streaming alternate setting is enough */
        }
        in_stream = p[off + 2] == iface_num && p[off + 3] != 0;
      } else if (in_stream && type == 0x24 && len >= 5 && p[off + 2] == 0x01) {
        timing.delay_ms = p[off + 4]; /* CS AS_GENERAL: bDelay */
      } else if (in_stream && type == USB_B_DESCRIPTOR_TYPE_ENDPOINT &&
                 len >= 4 && (p[off + 2] & 0x80) == 0 &&
                 (p[off + 3] & USB_BM_ATTRIBUTES_XFERTYPE_MASK) ==
                     USB_BM_ATTRIBUTES_XFER_ISOC) {
        timing.sync_type = (p[off + 3] & USB_BM_ATTRIBUTES_SYNCTYPE_MASK) >> 2;
      }
    }
  }
  if (dev) {
    usb_host_device_close(client, dev);
  }
  /* Drain anything posted to us while open, then leave.  Bounded: the
   * queue holds at most max_num_event_msg entries. */
  for (int i = 0; i < client_cfg.max_num_event_msg + 1; i++) {
    if (usb_host_client_handle_events(client, 0) != ESP_OK) {
      break;
    }
  }
  usb_host_client_deregister(client);
  return timing;
}

/* The attached card, owned by the control task.  s_dev is its open stream
 * and is NULL while the stream is being restarted. */
static bool s_card_present;
static uint8_t s_card_addr;
static uint8_t s_card_iface;
static uint32_t s_card_delay;
static card_format_t s_card_fmt;

/* Open the card's playback interface and start it at the best format for a
 * source at `want`, trying the next candidate rate if a start fails.  The
 * ring is sized at open, so a format that needs another size reopens. */
/* Volume is applied in software, so the card's own gain only sets the
 * ceiling.  Run it at unity (0 dB), or its maximum if that is below unity,
 * and unmuted: cards power up at an arbitrary level, and the host they last
 * met may have left them quiet or muted.  The driver does not expose the
 * range, so go to the maximum and read back what that is.  Cards without
 * the controls keep their own. */
static void card_gain_unity(uac_host_device_handle_t handle) {
  esp_err_t err = uac_host_device_set_volume(handle, 100);
  int16_t max_db = 0; /* 1/256 dB */
  if (err == ESP_OK) {
    err = uac_host_device_get_volume_db(handle, &max_db);
  }
  if (err == ESP_OK && max_db > 0) {
    err = uac_host_device_set_volume_db(handle, 0);
  }
  if (err != ESP_OK) {
    ESP_LOGD(TAG, "Card volume not set: %s", esp_err_to_name(err));
  }
  err = uac_host_device_set_mute(handle, false);
  if (err != ESP_OK) {
    ESP_LOGD(TAG, "Card mute not set: %s", esp_err_to_name(err));
  }
}

/* Append a USB string descriptor as ASCII, space-separated and trimmed.
 * The ROM printf has no %ls. */
static void append_usb_str(char *dst, size_t size, const wchar_t *src) {
  size_t n = strlen(dst);
  while (*src == L' ') {
    src++;
  }
  if (*src && n && n + 1 < size) {
    dst[n++] = ' ';
  }
  for (; *src && n + 1 < size; src++) {
    dst[n++] = (*src >= 0x20 && *src < 0x7f) ? (char)*src : '?';
  }
  while (n && dst[n - 1] == ' ') {
    n--;
  }
  dst[n] = '\0';
}

static bool start_card(uint32_t want, bool announce) {
  uint32_t ring_bytes = DRIVER_RING_BYTES(want, 2);
  uac_host_device_handle_t handle = NULL;
  esp_err_t err = open_iface(s_card_addr, s_card_iface, ring_bytes, &handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Open failed: %s", esp_err_to_name(err));
    return false;
  }

  uac_host_dev_info_t info;
  err = uac_host_get_device_info(handle, &info);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Device info failed: %s", esp_err_to_name(err));
    uac_host_device_close(handle);
    return false;
  }
  if (announce) {
    char name[2 * UAC_STR_DESC_MAX_LENGTH] = "";
    append_usb_str(name, sizeof(name), info.iManufacturer);
    append_usb_str(name, sizeof(name), info.iProduct);
    ESP_LOGI(TAG, "Sound card %04x:%04x \"%s\" connected", info.VID, info.PID,
             name);
  }

  uint32_t rates[3];
  const size_t n = rate_candidates(want, rates);
  for (size_t i = 0; i < n; i++) {
    card_format_t fmt;
    if (!pick_depth(handle, info.iface_alt_num, rates[i], &fmt)) {
      continue;
    }
    if (DRIVER_RING_BYTES(fmt.rate, fmt.subframe) != ring_bytes) {
      ring_bytes = DRIVER_RING_BYTES(fmt.rate, fmt.subframe);
      uac_host_device_close(handle);
      handle = NULL;
      err = open_iface(s_card_addr, s_card_iface, ring_bytes, &handle);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "Reopen failed: %s", esp_err_to_name(err));
        return false;
      }
    }

    const uac_host_stream_config_t stream_cfg = {
        .channels = CHANNELS,
        .bit_resolution = fmt.bits,
        .sample_freq = fmt.rate,
        .flags = 0,
    };
    err = uac_host_device_start(handle, &stream_cfg);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "Start at %" PRIu32 " Hz failed: %s", fmt.rate,
               esp_err_to_name(err));
      continue;
    }
    card_gain_unity(handle);

    /* The rate must be in place before the first completion counts
     * frames. */
    if (fmt.rate != s_output_rate) {
      s_output_rate = fmt.rate;
      resample_reinit_needed = true;
    }
    s_card_fmt = fmt;

    xSemaphoreTake(s_dev_lock, portMAX_DELAY);
    queue_reset(s_card_delay);
    s_dev = handle;
    s_dev_subframe = fmt.subframe;
    xSemaphoreGive(s_dev_lock);
    ESP_LOGI(TAG, "Streaming to card at %" PRIu32 " Hz, %u-bit in %u bytes%s",
             fmt.rate, fmt.bits, fmt.subframe,
             fmt.rate == want ? "" : " (resampling)");
    return true;
  }

  ESP_LOGE(TAG,
           "Card offers no usable 16/24/32-bit stereo stream at %" PRIu32
           ", %d or %d Hz",
           want, PREFERRED_RATE, FALLBACK_RATE);
  uac_host_printf_device_param(handle);
  uac_host_device_close(handle);
  return false;
}

/* Detach the open stream from the playback task and close it.  The caller
 * decides whether the card itself is gone. */
static void stop_card(void) {
  xSemaphoreTake(s_dev_lock, portMAX_DELAY);
  uac_host_device_handle_t handle = s_dev;
  s_dev = NULL;
  s_dev_subframe = 0;
  queue_reset(0);
  xSemaphoreGive(s_dev_lock);
  if (handle) {
    uac_host_device_close(handle);
  }
}

static void open_card(uint8_t addr, uint8_t iface_num) {
  if (s_card_present) {
    ESP_LOGW(TAG, "Ignoring second sound card (addr %u)", addr);
    return;
  }

  const card_timing_t timing = read_card_timing(addr, iface_num);
  static const char *const sync_names[] = {"none", "asynchronous", "adaptive",
                                           "synchronous"};
  ESP_LOGI(TAG, "Endpoint sync: %s, card delay: %u ms",
           timing.sync_type < 0 ? "unknown" : sync_names[timing.sync_type],
           timing.delay_ms);
  if (timing.sync_type == 1) {
    /* The driver ignores the feedback endpoint, so packets are sized from
     * our clock while the card's DAC runs on its own crystal. */
    ESP_LOGW(TAG, "Asynchronous card: its clock is not slaved to ours, so "
                  "expect slow drift and an occasional glitch");
  }

  s_card_addr = addr;
  s_card_iface = iface_num;
  s_card_delay = (uint32_t)timing.delay_ms * 1000U;
  s_card_present = start_card((uint32_t)source_rate, true);
}

/* The source changed rate: restart the card at it if that changes the
 * format the card would run, and leave it alone otherwise. */
static void follow_source_rate(void) {
  if (!s_card_present || !s_dev) {
    return;
  }
  const uint32_t want = (uint32_t)source_rate;
  uac_host_dev_info_t info;
  card_format_t fmt;
  if (uac_host_get_device_info(s_dev, &info) != ESP_OK ||
      !choose_format(s_dev, info.iface_alt_num, want, &fmt) ||
      (fmt.rate == s_card_fmt.rate && fmt.bits == s_card_fmt.bits)) {
    return;
  }
  ESP_LOGI(TAG, "Source now %" PRIu32 " Hz, restarting card", want);
  stop_card();
  s_card_present = start_card(want, false);
}

static void close_card(uac_host_device_handle_t handle) {
  /* Only the open stream is ours to close.  Any other handle is one this
   * task already closed, during a restart or after rejecting the card. */
  if (handle != s_dev) {
    return;
  }
  stop_card();
  s_card_present = false;
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
    case EVT_SOURCE_RATE:
      follow_source_rate();
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

/* Widen interleaved 16-bit samples into subframes of `subframe` bytes:
 * little-endian, MSB-justified, low bytes zero. */
static void widen(const int16_t *in, size_t samples, uint8_t subframe,
                  uint8_t *out) {
  for (size_t i = 0; i < samples; i++) {
    uint16_t s = (uint16_t)in[i];
    for (uint8_t b = 0; b < subframe - 2; b++) {
      *out++ = 0;
    }
    *out++ = (uint8_t)(s & 0xFF);
    *out++ = (uint8_t)(s >> 8);
  }
}

/* Hand one block to the card.  Returns false if there is no card to take
 * it, in which case the caller paces itself instead. */
static bool write_card(int16_t *buf, size_t frames, uint8_t *wide_buf) {
  bool written = false;
  xSemaphoreTake(s_dev_lock, portMAX_DELAY);
  if (s_dev) {
    uint8_t *data = (uint8_t *)buf;
    size_t samples = frames * CHANNELS;
    if (s_dev_subframe != sizeof(int16_t)) {
      widen(buf, samples, s_dev_subframe, wide_buf);
      data = wide_buf;
    }
    esp_err_t err =
        uac_host_device_write(s_dev, data, (uint32_t)(samples * s_dev_subframe),
                              pdMS_TO_TICKS(WRITE_TIMEOUT_MS));
    if (err == ESP_OK) {
      written = true;
      portENTER_CRITICAL(&s_queue_lock);
      s_frames_written += frames;
      portEXIT_CRITICAL(&s_queue_lock);
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
  s_play_task = xTaskGetCurrentTaskHandle();
  int16_t *pcm = malloc((size_t)(FRAME_SAMPLES + 1) * 2 * sizeof(int16_t));
  int16_t *silence = calloc((size_t)FRAME_SAMPLES * 2, sizeof(int16_t));
  int16_t *resample_buf = malloc(MAX_RESAMPLE_FRAMES * 2 * sizeof(int16_t));
  uint8_t *wide_buf = malloc(MAX_RESAMPLE_FRAMES * CHANNELS * MAX_SUBFRAME);
  if (!pcm || !silence || !resample_buf || !wide_buf) {
    ESP_LOGE(TAG, "Failed to allocate buffers");
    free(pcm);
    free(silence);
    free(resample_buf);
    free(wide_buf);
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

    if (write_card(play_buf, play_samples, wide_buf)) {
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
    if (s_evt_queue) {
      /* The control task reads source_rate itself, so a lost event only
       * delays following until the next change. */
      usbh_evt_t evt = {.type = EVT_SOURCE_RATE};
      xQueueSend(s_evt_queue, &evt, 0);
    }
  }
}

uint32_t audio_output_get_hardware_latency_us(void) {
  // Fallback for when no depth can be measured: a full ring, plus the
  // transfers in flight, which on average are half a transfer short of all
  // queued.  The card's bDelay is added once it is known.
  return DRIVER_RING_MS * 1000U + URB_COUNT * URB_US - URB_US / 2 +
         s_card_delay_us;
}

bool audio_output_get_pipeline_us(int64_t *now_us, uint32_t *pipeline_us) {
  portENTER_CRITICAL(&s_queue_lock);
  const uint64_t written = s_frames_written;
  const uint64_t sent = s_frames_sent;
  const int64_t sent_us = s_sent_us;
  const uint32_t card_delay_us = s_card_delay_us;
  portEXIT_CRITICAL(&s_queue_lock);
  const int64_t sampled_us = esp_timer_get_time();
  const uint32_t rate = s_output_rate;

  // Nothing has completed yet, or no card: let the caller use the model.
  if (sent_us == 0 || rate == 0) {
    return false;
  }

  uint64_t queued = written > sent ? written - sent : 0;
  // Completions step the count a whole transfer at a time, so charge off
  // what the bus has sent since the last one.  Past one transfer the next
  // completion is already due; extrapolating further would invent drain.
  if (sampled_us > sent_us) {
    uint64_t drained = ((uint64_t)(sampled_us - sent_us) * rate) / 1000000ULL;
    const uint64_t urb_max = (uint64_t)rate * URB_PACKETS / 1000U + 1U;
    if (drained > urb_max) {
      drained = urb_max;
    }
    queued = drained < queued ? queued - drained : 0U;
  }

  if (now_us) {
    *now_us = sampled_us;
  }
  if (pipeline_us) {
    *pipeline_us = (uint32_t)((queued * 1000000ULL) / rate) + card_delay_us;
  }
  return true;
}

uint32_t audio_output_get_underruns(void) {
  return s_underruns;
}

/* There is no channel routing, so the weak defaults in audio_output_common.c
 * cover the rest of the API. */
