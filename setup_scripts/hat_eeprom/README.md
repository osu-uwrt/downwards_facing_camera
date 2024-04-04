# External Camera HAT EEPROM

This folder contains the required configuration data for the EEPROM on the External Camera HAT.
You typically do not need to worry about this folder unless you are building a new External Camera HAT.
Then you will need to use Raspberry Pi's `eeptools` utility to generate and flash this configuration.

Additionally, if you need to change the default overlays when the HAT is installed, you will need to modify the
contents of this folder and reprogram the EEPROM on the HAT. This can be done by shorting the write protect
test point to the ground testpoint directly next to it (see the schematics for more info).

Note that this needs to use the legacy v1 HAT format as this loads custom device tree overlays not present by
default in the raspberry pi firmware.

## Generating the EEPROM Image

If you need to change the EEPROM image for any reason, you can do so by modifying the settings and overlay files.
Before compiling, be sure you have either installed [eeptools](https://github.com/raspberrypi/utils) or built
it and set the `EEPMAKE_PATH` to point to the program.
You can run the `./compile.sh` script in this folder to regenerate the eeprom image.

## Uploading the EEPROM Image

To upload, use the tutorial in the README of the eeptools directory of the Raspberry Pi [utils repo](https://github.com/raspberrypi/utils).
You should find the eeprom image named `ext-cam-hat.eep` in this directory. During the flash process, you must
hold the write protect testpoint to the ground testpoint throughout the entire flash process. If not, you will
receive a time out error while attempting to flash.
