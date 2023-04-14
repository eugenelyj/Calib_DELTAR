# Recording program for VL53L5CX
Based on the official VL53L5CX Linux driver and test applications for linux and android platforms.

## Introduction
The proposed implementation is customized to run on a raspberry pi v3, but can be adapted to run on any linux embedded platform,
as far as the VL53L5CX device is connected through I2C
Two options are offered to the user
- 1. compile and run this driver with a kernel module responsible for handling i2c bus and the interruption. This is the kernel mode
- 2. compile and run this driver in a full user mode, where the i2c commnication is handled with the /dev/i2c-1 file descriptor. This is the user mode

Option 1 supports the interruption line of the VL53L5CX but needs a kernel module to be compiled and inserted.
Option 2 may be more suitable for simple application, but needs the /dev/i2c-1 to be available which may not be the case on some secured platforms

## How to run a test application on raspberry pi
    Note that the following instructions were tested on raspberrypi 3.

### Install the raspberry pi kernel source headers (kernel mode only)
    refer to raspberrypi official documentation to download the headers matching your kernel version
    $ sudo apt-get install raspberrypi-kernel-headers

### update /boot/config.txt file (kernel mode only)
    $ sudo nano /boot/config.txt
    --> add or uncomment the following lines at the end of the /boot/config.txt
    dtparam=i2c_arm=on
    dtparam=i2c1=on
    dtparam=i2c1_baudrate=1000000
    dtoverlay=stmvl53l5cx
### compile the device tree blob (kernel mode only)
    $ cd vl53l5cx-uld-driver/kernel
    $ make dtb
    $ sudo reboot
### compile the test examples, the platform adaptation layer and the uld driver
    $ cd vl53l5cx-driver/user
    $ make
### compile the kernel module (kernel mode only)
    $ cd vl53l5cx-uld-driver/kernel
    $ make clean
    $ make
    $ sudo make insert
### run the recording program
    $ cd vl53l5cx-uld-driver/user
    $ ./bin/long_record or ./bin/short_record
