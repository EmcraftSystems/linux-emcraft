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
obj-$(CONFIG_I2C_STM32F7)	+= i2c.o
obj-$(CONFIG_I2C_GPIO)		+= i2c-gpio.o
obj-$(CONFIG_MTD_PHYSMAP)	+= flash.o
obj-$(CONFIG_MTD_STM32F4_MAP)	+= flash.o
obj-$(CONFIG_MMC_ARMMMCI)	+= sdcard.o
obj-$(CONFIG_RTC_DRV_STM32F2)	+= rtc.o
obj-$(CONFIG_STM32_USB_OTG_FS)	+= stm32_usb_fs.o
obj-$(CONFIG_STM32_USB_OTG_HS)	+= stm32_usb_hs.o
obj-$(CONFIG_STM32_FB)		+= fb.o
obj-$(CONFIG_PM)		+= suspend.o pm.o
obj-$(CONFIG_CAN_STM32)		+= can.o
