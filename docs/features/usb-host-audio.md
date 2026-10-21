# USB sound card output

Boards with a USB OTG port can play AirPlay audio through a **USB sound card or USB DAC**
plugged into that port. The board is the USB host and the card is the device, which is the
opposite of [USB audio (UAC)](usb-audio.md), where the board plugs into a computer.

!!! warning "Experimental"

    This output is new and has been tested on few cards. Please report the card you used,
    working or not.

!!! warning "Needs USB OTG and a host-role port"

    Only the ESP32-S2, S3 and P4 have a USB OTG peripheral. The board must supply 5 V on
    VBUS to power the card; many dev boards only do this through a USB-OTG adapter or a
    dedicated host port.

## Building

Select **Audio Output → Audio output type → USB host (external sound card)** in
menuconfig, or layer `config/sdkconfig.defaults.usbhost` on top of a board's defaults:

```bash
idf.py -DSDKCONFIG_DEFAULTS="config/sdkconfig.defaults;config/sdkconfig.defaults.esp32s3;config/sdkconfig.defaults.usbhost" build
```

It replaces the I2S output, so it cannot be combined with an I2S DAC or with the USB
speaker mode, which needs the same port.

## Supported cards

- **Stereo PCM at 16, 24 or 32 bits**, at 48 kHz or 44.1 kHz as a fallback. The narrowest
  depth the card offers is used; wider ones carry the same 16-bit audio padded with zeros.
  A card that offers none of these is logged with its full list of formats and left unused.
- The card needs its own DAC chip. Passive USB-C to 3.5 mm adapters, which only pass analog
  audio from a phone, do not work.
- **UAC 1.0 only, on every chip.** Espressif's USB host audio driver rejects UAC 2.0
  devices, so even the high-speed ESP32-P4 cannot use them yet. Most inexpensive USB sound
  cards and dongles are UAC 1.0, and many UAC 2.0 DACs fall back to UAC 1.0 when they are
  connected at full speed (the S2 and S3 only run at full speed) or offer a UAC 1.0 mode.

## How it works

- The card can be plugged in and out while the board runs. Playback continues in the
  background while no card is attached, so a card plugged in mid-song joins in sync.
- AirPlay volume is applied in software, as on the S/PDIF output. The card's own volume is
  left where it was.
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
