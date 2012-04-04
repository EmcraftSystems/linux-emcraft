__ZRELADDR	:= $(shell /bin/bash -c 'printf "0x%08x" \
		     $$[$(CONFIG_DRAM_BASE) + 0x008000]')
__PARAMS_PHYS	:= $(shell /bin/bash -c 'printf "0x%08x" \
		     $$[$(CONFIG_DRAM_BASE) + 0x000100]')
__INITRD_PHYS	:= $(shell /bin/bash -c 'printf "0x%08x" \
		     $$[$(CONFIG_DRAM_BASE) + 0x100000]')

   zreladdr-y	+= $(__ZRELADDR)
params_phys-y	:= $(__PARAMS_PHYS)
initrd_phys-y	:= $(__INITRD_PHYS)
