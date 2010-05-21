#
# Makefile for the linux kernel.
#

obj-y					:= core.o clock.o
obj-$(CONFIG_MACH_REALVIEW_EB)		+= realview_eb.o
obj-$(CONFIG_MACH_REALVIEW_PB11MP)	+= realview_pb11mp.o
obj-$(CONFIG_MACH_REALVIEW_PB1176)	+= realview_pb1176.o
obj-$(CONFIG_MACH_REALVIEW_PBA8)	+= realview_pba8.o
obj-$(CONFIG_MACH_REALVIEW_PBX)		+= realview_pbx.o
obj-$(CONFIG_SMP)			+= platsmp.o headsmp.o
obj-$(CONFIG_HOTPLUG_CPU)		+= hotplug.o
ifeq ($(CONFIG_PCI),y)
obj-$(CONFIG_MACH_REALVIEW_PB11MP)	+= pcix.o
obj-$(CONFIG_MACH_REALVIEW_PBA8)	+= pcix.o
obj-$(CONFIG_MACH_REALVIEW_PBX)		+= pcix.o
endif
obj-$(CONFIG_LOCAL_TIMERS)		+= localtimer.o
obj-$(CONFIG_CPU_FREQ_REALVIEW)		+= iec.o
