#
# Makefile for the linux kernel.
#

obj-y	:= timer.o irq.o common.o serial.o clock.o
obj-y	+=  dma.o gpiolib.o pm.o suspend.o
obj-$(CONFIG_MACH_PHY3250)	+= phy3250.o
obj-$(CONFIG_MACH_EA3250)	+= ea3250.o
obj-$(CONFIG_MACH_FDI3250)	+= fdi3250.o

