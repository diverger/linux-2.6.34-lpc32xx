#!/bin/sh

echo Set enviorement

cd /home/diverger/work/projects/lpc32x0/kernel/linux-2.6.34-lpc32xx

export ARCH=arm
export CROSS_COMPILE=arm-linux-
export PATH=/home/diverger/work/projects/eldk42/usr/bin:/home/diverger/work/projects/eldk42/bin:$PATH

echo Launch menuconfig

make ARCH=arm CROSS_COMPILE=arm-linux- smart3250_defconfig menuconfig