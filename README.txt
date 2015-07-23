About
=====

This is a generic Linux kernel driver for the Silead GSLx68y
series of touch screen controllers.
It is currently designed to work with ACPI platforms, but
support for DeviceTree/OpenFirmware is also in the works.

The code was adapted from the platform specific driver here:
https://github.com/jabjoe/sunxi-gslx680

The driver is currently incomplete and will only send ABS_X/Y
events to the input subsystem. It seems some versions of the
controller (or its firmware) do not support finger tracking,
or additional initialisation code is necessary to enable it.

The display resolution is currently hardcoded as 1024x768.
There seems to be a hardware register that contains the panel
data, but due to the lack of a complete data sheet, the
location of this register is currently unknown.


Firmware Instructions
=====================

The device requires firmware/configuration data to work.
Example firmware for some devices is included.

You may either extract the firmware from an Android or Windows
driver for this panel or use one of the supplied images.

The Android driver can be found under this path (or similar):
/system/vendor/modules/gslx680.ko
Copy this to a SD card or use a GNU/Linux chroot to scp it over, or use
adb pull.
On your build machine, on the command line, use the script
'firmware/fw_extractor' to extract the firmware to its own file.

./firmware/fw_extractor my_android_gslx680.ko my_tablets.fw

The Android driver may well contain multiple firmwares to support
different hardware configurations with the same driver. The extractor
will spit those out as seperate files. We currently have no way of
knowing which is right for your device. You will have to try each.

If you have a Windows driver instead, it seems some vendors will
include a header file with the firmware. You can convert this header
file into a firmware image using the tscfg2fw script in the
firmware/ directory.

Shortly, a new firmware format will be introduced. You can use the
fwtool script in firmware/ to convert "classic" firmware images
into the new format and set device parameters. This format is
designed to provide information to the driver that cannot obtain
anywhere else. It is particularly useful on platforms that do
not contain touch panel and firmware parameters in the DSDT or DT.

A description of the file format is included at the end of
firmware/Firmware/Silead.pm.

The driver does not support this new firmware format, however.


Build Instructions
==================

If you don't need to cross compile, just make sure you have headers
for your running Linux kernel installed, then type

make

This will produce gslx680_ts_acpi.ko

If you need to cross compile, pass appropriate KDIR, ARCH and
CROSS_COMPILE variables to the make command. For example:

make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- KDIR=../linux-arm


Install Instructions
====================

The driver is currently incomplete, but it will generate multitouch
events.

Load the driver with

insmod ./gslx680_ts_acpi.ko

Running dmesg should produce some output if the device was matched
by the driver. You should also see a message from the input
subsystem that a new input device was added.

You may then observe the output from evtest. X.org touchscreen input
should work too, but you will likely notice that the touch
coordinates are very much off.

You will need to calibrate the X.org touchscreen driver to make it
work correctly. Install xinput_calibrate through your package
manager and start it from a terminal. Some desktop environments
may offer their own touchscreen calibrator, which you can also use.

xinput_calbrator will present a series of points on the screen,
which you should touch. It will then print a configuration block
and instructions on where to store it. After doing so and restarting
X, you should have a working touchscreen.
