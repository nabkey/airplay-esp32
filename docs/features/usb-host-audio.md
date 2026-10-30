# USB sound card output

Boards with a USB OTG port can play AirPlay audio through a **USB sound card or USB DAC**
plugged into that port. The board is the USB host and the card is the device, which is the
opposite of [USB audio (UAC)](usb-audio.md), where the board plugs into a computer.

!!! warning "Experimental"

    This output is new and has only been tested on the ESP32-S3, with the cards listed
    [below](#tested-cards). Please report the card and board you used, working or not.

!!! warning "Needs USB OTG and a host-role port"

    Only the ESP32-S2, S3 and P4 have a USB OTG peripheral. The S2 and P4 builds compile but
    have not been tried on hardware. The board must supply 5 V on VBUS to power the card;
    many dev boards only do this through a USB-OTG adapter or a dedicated host port.

## Building

Select **Audio Output → Audio output type → USB host (external sound card)** in
menuconfig, or layer `config/sdkconfig.defaults.usbhost` on top of a board's defaults:

```bash
idf.py -DSDKCONFIG_DEFAULTS="config/sdkconfig.defaults;config/sdkconfig.defaults.esp32s3;config/sdkconfig.defaults.usbhost" build
```

It replaces the I2S output, so it cannot be combined with an I2S DAC or with the USB
speaker mode, which needs the same port.

## Supported cards

- **Stereo PCM at 16, 24 or 32 bits.** The narrowest depth the card offers is used; wider
  ones carry the same 16-bit audio padded with zeros.
- **The card runs at the source's own rate when it can:** 44.1 kHz for AirPlay, and usually
  48 kHz for Sendspin, so neither is resampled. When the source changes rate the stream
  restarts at the new one, which costs a moment of silence. A rate the card does not offer
  is resampled to 48 kHz, or 44.1 kHz if the card lacks 48 kHz too. A card that offers
  neither is logged with its full list of formats and left unused.
- **Output packets up to 600 bytes.** The S3's USB FIFOs are small, so the build gives
  isochronous OUT most of them (`CONFIG_USB_HOST_HW_BUFFER_BIAS_PERIODIC_OUT`). That covers
  stereo up to 48 kHz at any depth, and 96 kHz at 16 bits. A card declaring more is refused
  with `EP MPS (...) exceeds supported limit`.
- The card needs its own DAC chip. Passive USB-C to 3.5 mm adapters, which only pass analog
  audio from a phone, do not work.
- **UAC 1.0 only, on every chip.** Espressif's USB host audio driver rejects UAC 2.0
  devices, so even the high-speed ESP32-P4 cannot use them yet. Most inexpensive USB sound
  cards and dongles are UAC 1.0, and many UAC 2.0 DACs fall back to UAC 1.0 when they are
  connected at full speed (the S2 and S3 only run at full speed) or offer a UAC 1.0 mode.

## Wiring

On a dev board with two USB-C ports, the card goes in the one wired to the chip's own USB
pins (often labelled **USB** or **OTG**), not the one behind a USB-to-UART bridge (**COM**
or **UART**). Power for the card has to reach that port's VBUS:

- On the **YD-ESP32-S3**, bridge the **USB-OTG** solder pads on the back. The 5 V from a
  supply or computer on the COM port then also feeds the OTG port, so the card is powered
  while the COM port keeps the UART console for logs and flashing.
- With those pads bridged, never plug a computer or charger into the OTG port: two 5 V
  supplies end up tied together.

## Tested cards

All on a YD-ESP32-S3 (N16R8).

| Card | USB ID | Formats | Result |
| --- | --- | --- | --- |
| Generic USB Audio | `001f:0b21` | 48 kHz, 16-bit, adaptive | Works; AirPlay resampled to 48 kHz. 24-minute soak with no gaps. |
| C-Media USB Headphone Set | `0d8c:000c` | 44.1 and 48 kHz, 16-bit, adaptive | Works at 44.1 kHz, no resampling. Its 100% volume step overshoots its range, see below. |
| Turtle Beach USB Audio | `10f5:0211` | 48 kHz, 16-bit, adaptive | Works; AirPlay resampled to 48 kHz. |

## How it works

- The card can be plugged in and out while the board runs. Playback continues in the
  background while no card is attached, so a card plugged in mid-song joins in sync.
- AirPlay volume is applied in software, as on the S/PDIF output. The card's own volume is
  set to 0 dB (or its maximum, if lower) and unmuted when streaming starts. Some cards,
  such as the C-Media above, read back almost silent when asked for 100%, because the
  driver rounds past the top of their range; the board then uses the step below and logs
  `Card gain at 100% reads ... using 99%`.
- The channel setting (stereo, left, right or mono) works as on the I2S output, from the
  web page or the button, and is kept across reboots.
- About 30 ms of audio is queued in the USB driver. Its depth is measured from transfer
  completions, the way the I2S output measures its DMA, so multiroom sync uses the real
  figure rather than an estimate. The card's own delay is added when it reports one
  (`bDelay`); delay inside the card that it does not report is not covered.
- The boot log shows the card's endpoint sync type. **Adaptive** and **synchronous** cards
  follow the board's clock. **Asynchronous** cards run on their own clock, and since the
  driver does not use their feedback endpoint, expect slow drift and an occasional glitch.

!!! note "No USB console"

    Claiming the OTG port disconnects the USB-Serial-JTAG console. Use the UART console, and
    hold BOOT while resetting the board to reflash it.
