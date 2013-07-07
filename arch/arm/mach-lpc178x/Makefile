#
# Makefile for the NXP LPC178x/7x platform files
#

obj-$(CONFIG_MACH_LPC178X)	+= lpc178x_platform.o timer.o clock.o \
				   iomux.o reboot.o power.o dma.o
obj-$(CONFIG_SERIAL_8250)	+= uart.o
obj-$(CONFIG_LPC178X_ETHER)	+= eth.o
obj-$(CONFIG_MTD_PHYSMAP)	+= flash.o
obj-$(CONFIG_USB_OHCI_HCD)	+= ohci.o
obj-$(CONFIG_MMC_ARMMMCI)	+= sdcard.o
obj-$(CONFIG_SPI_PL022)		+= spi.o
obj-$(CONFIG_I2C_LPC2K)		+= i2c.o
obj-$(CONFIG_I2C_GPIO)		+= i2c-gpio.o
obj-$(CONFIG_FB_ARMCLCD)	+= fb.o
obj-$(CONFIG_LEDS_PCA9532)	+= ea-lpc1788-pca9532.o
obj-$(CONFIG_GPIOLIB)		+= gpio.o
obj-$(CONFIG_RTC_DRV_LPC178X)	+= rtc.o
obj-$(CONFIG_LPC2K_WATCHDOG)	+= wdt.o
