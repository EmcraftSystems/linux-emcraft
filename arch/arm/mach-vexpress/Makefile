#
# Makefile for the linux kernel.
#

obj-y					:= core.o clock.o
obj-$(CONFIG_MACH_VEXPRESS)		+= vexpress_pca9.o
obj-$(CONFIG_SMP)			+= platsmp.o headsmp.o
obj-$(CONFIG_HOTPLUG_CPU)		+= hotplug.o
obj-$(CONFIG_LOCAL_TIMERS)		+= localtimer.o
