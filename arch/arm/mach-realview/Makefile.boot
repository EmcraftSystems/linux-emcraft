   zreladdr-y	:= $(shell perl -e "printf('0x%08X',$(CONFIG_REALVIEW_PHYS_OFFSET)+0x00008000)")
params_phys-y	:= $(shell perl -e "printf('0x%08X',$(CONFIG_REALVIEW_PHYS_OFFSET)+0x00000100)")
initrd_phys-y	:= $(shell perl -e "printf('0x%08X',$(CONFIG_REALVIEW_PHYS_OFFSET)+0x00800000)")

