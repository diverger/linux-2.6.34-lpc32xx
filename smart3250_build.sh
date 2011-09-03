#!/bin/sh

echo Set enviorement

cd /home/diverger/work/projects/lpc32x0/kernel/linux-2.6.34-lpc32xx

export ARCH=arm
export CROSS_COMPILE=arm-linux-
export PATH=/home/diverger/work/projects/eldk42/usr/bin:/home/diverger/work/projects/eldk42/bin:$PATH

echo Build ther kernel

make ARCH=arm CROSS_COMPILE=arm-linux- smart3250_defconfig
make ARCH=arm CROSS_COMPILE=arm-linux- uImage
make ARCH=arm CROSS_COMPILE=arm-linux- modules
make ARCH=arm CROSS_COMPILE=arm-linux- INSTALL_MOD_PATH=/home/diverger/work/projects/eldk42/arm modules_install
make ARCH=arm CROSS_COMPILE=arm-linux- INSTALL_MOD_PATH=/home/diverger/work/projects/eldk42/armVFP modules_install