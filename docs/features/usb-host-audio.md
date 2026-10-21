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

- **16-bit stereo PCM** at 48 kHz, or 44.1 kHz as a fallback. A card that offers neither is
  logged with its full list of formats and left unused.
- **ESP32-S2 and S3**: the OTG port is full-speed only, so only **UAC 1.0** cards work. Most
  inexpensive USB sound cards and dongles are UAC 1.0.
- **ESP32-P4**: high-speed, so UAC 2.0 cards work as well.

## How it works

- The card can be plugged in and out while the board runs. Playback continues in the
  background while no card is attached, so a card plugged in mid-song joins in sync.
- AirPlay volume is applied in software, as on the S/PDIF output. The card's own volume is
  left where it was.
- About 20 ms of audio is buffered in the USB driver, which is reported to the timing engine
  so multiroom sync is kept.

!!! note "No USB console"

    Claiming the OTG port disconnects the USB-Serial-JTAG console. Use the UART console, and
    hold BOOT while resetting the board to reflash it.
