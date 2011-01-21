#!/bin/sh
#make CONFIG_DEBUG_SECTION_MISMATCH=y ARCH=arm CROSS_COMPILE=/mnt/video/OE/tmp/sysroots/i686-linux/usr/armv7a/bin/arm-oe-linux-gnueabi- zImage
make CONFIG_DEBUG_SECTION_MISMATCH=y zImage 1> zImage.log
