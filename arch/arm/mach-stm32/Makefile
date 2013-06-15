#
# Makefile for the STM32 platform files
#

obj-$(CONFIG_MACH_STM32)	+= stm32_platform.o timer.o clock.o \
				   iomux.o reboot.o exti.o
obj-$(CONFIG_GPIOLIB)		+= gpio.o
obj-$(CONFIG_STM32_DMA)		+= dmainit.o dmac.o
obj-$(CONFIG_SERIAL_STM32)	+= uart.o
obj-$(CONFIG_STM32_ETHER)	+= eth.o
obj-$(CONFIG_SPI_STM32)		+= spi.o
obj-$(CONFIG_I2C_STM32)		+= i2c.o
obj-$(CONFIG_MTD_PHYSMAP)	+= flash.o
obj-$(CONFIG_MMC_ARMMMCI)	+= sdcard.o
obj-$(CONFIG_RTC_DRV_STM32F2)	+= rtc.o
