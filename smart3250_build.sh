#!/bin/bash

echo Set enviorement

cd /home/diverger/work/projects/lpc32x0/kernel/inux-2.6.34-lpc32xx

export ARCH=arm
export CROSS_COMPILE=arm-linux-
export PATH=/home/diverger/work/projects/eldk42/usr/bin:/home/diverger/work/projects/eldk42/bin:$PATH

echo Build ther kernel

make ARCH=arm CROSS_COMPILE=arm-linux- smart3250_defconfig uImage