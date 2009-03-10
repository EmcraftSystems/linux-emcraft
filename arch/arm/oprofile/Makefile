obj-$(CONFIG_OPROFILE) += oprofile.o

DRIVER_OBJS = $(addprefix ../../../drivers/oprofile/, \
		oprof.o cpu_buffer.o buffer_sync.o \
		event_buffer.o oprofile_files.o \
		oprofilefs.o oprofile_stats.o \
		timer_int.o )

oprofile-y				:= $(DRIVER_OBJS) common.o backtrace.o op_arm11.o op_v7.o op_model_v6-7.o
oprofile-$(CONFIG_CACHE_L2X0)		+= op_l2x0.o
oprofile-$(CONFIG_SMP)			+= op_scu.o
oprofile-$(CONFIG_CPU_XSCALE)		+= op_model_xscale.o
