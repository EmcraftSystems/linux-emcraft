/* ld script to make ARM Linux kernel
 * taken from the i386 version by Russell King
 * Written by Martin Mares <mj@atrey.karlin.mff.cuni.cz>
 */

#include <asm-generic/vmlinux.lds.h>
#include <asm/thread_info.h>
#include <asm/memory.h>
#include <asm/page.h>
	
OUTPUT_ARCH(arm)
ENTRY(stext)

#ifndef __ARMEB__
jiffies = jiffies_64;
#else
jiffies = jiffies_64 + 4;
#endif

SECTIONS
{

#ifdef CONFIG_XIP_KERNEL
	. = XIP_VIRT_ADDR(CONFIG_XIP_PHYS_ADDR);
#else
#if defined(PHYS_ALIAS_OFFSET)
	. = PHYS_ALIAS_OFFSET + TEXT_OFFSET;
#else
	. = PAGE_OFFSET + TEXT_OFFSET;
#endif
#endif

	.init : {			/* Init code and data		*/
		_stext = .;
		_sinittext = .;
			HEAD_TEXT
#if !defined(CONFIG_M2S_CACHE)
			INIT_TEXT
#endif
		_einittext = .;
#if !defined(CONFIG_INITRAMFS_IS_LARGE)
		/*
		 * In case the kernel links in a built-in initramfs
		 * and that initramfs is large enough, there is a symbol
		 * in .proc.info.init that makes a relative reference
		 * (in an assembly-level file) to a symbol in the kernel text.
		 * The initramfs being large, that relative reference 
		 * doesn't work because the linker is not able to fit
		 * in the relative offset into the 24 bit offset field.
		 * The solution (well, part of it) is to move .proc.info.init
		 * closer to the kernel text section.
		 */
		__proc_info_begin = .;
			*(.proc.info.init)
		__proc_info_end = .;
#endif
		__arch_info_begin = .;
			*(.arch.info.init)
		__arch_info_end = .;
		__tagtable_begin = .;
			*(.taglist.init)
		__tagtable_end = .;

#if !defined(CONFIG_M2S_CACHE)
		INIT_SETUP(16)
#endif

		__early_begin = .;
			*(.early_param.init)
		__early_end = .;

#if !defined(CONFIG_M2S_CACHE)
		INIT_CALLS
		CON_INITCALL
		SECURITY_INITCALL
#endif
		INIT_RAM_FS

#ifndef CONFIG_XIP_KERNEL
		__init_begin = _stext;
		INIT_DATA
#endif
	} 

	PERCPU(PAGE_SIZE)

#ifndef CONFIG_XIP_KERNEL
	. = ALIGN(PAGE_SIZE);
	__init_end = .;
#endif

	/*
	 * unwind exit sections must be discarded before the rest of the
	 * unwind sections get included.
	 */
	/DISCARD/ : {
		*(.ARM.exidx.exit.text)
		*(.ARM.extab.exit.text)
#ifndef CONFIG_HOTPLUG_CPU
		*(.ARM.exidx.cpuexit.text)
		*(.ARM.extab.cpuexit.text)
#endif
#ifndef CONFIG_HOTPLUG
		*(.ARM.exidx.devexit.text)
		*(.ARM.extab.devexit.text)
#endif
#ifndef CONFIG_MMU
		*(.fixup)
		*(__ex_table)
#endif
	}

/*
 * On Microsemi SmartFusion2, kernel code is linked into
 * an alias address region, which is cacheable. Anything that
 * is not code MUST reside in the non-cached address region.
 */
#if defined(CONFIG_M2S_CACHE)
	_non_cached_start = .;
	.cached m2s_phys_to_cached(_non_cached_start) : {
		_cached_init = .;

		INIT_TEXT
		INIT_SETUP(16)
		INIT_CALLS
		CON_INITCALL
		SECURITY_INITCALL

		*(.text)
		SCHED_TEXT
		LOCK_TEXT
		KPROBES_TEXT

		_cached_end = .;
	}
	. = _non_cached_start + SIZEOF(.cached);
#endif

/*
 * In case we run parts of the kernel in embedded Flash,
 * relocate critical kernel code to that fast Flash.
 */
#ifdef CONFIG_KERNEL_IN_ENVM
	_envm_loc = .;


	.envm  ENVM_PHYS_OFFSET + CONFIG_KERNEL_IN_ENVM_OFFSET * 1024 : {
		_envm_start = .;

#if CONFIG_KERNEL_IN_ENVM_SIZE>0
		__exception_text_start = .;
		*(.exception.text*)
		*(.exception.rodata*)
		__exception_text_end = .;
		SCHED_TEXT
		LOCK_TEXT
		KPROBES_TEXT

		usr/built-in.o(.text)
		usr/built-in.o(.rodata*)
		init/built-in.o(.text)
		init/built-in.o(.rodata*)
		mm/built-in.o(.text)
		mm/built-in.o(.rodata*)
		arch/arm/mm/built-in.o(.text)
		arch/arm/mm/built-in.o(.rodata*)
		arch/arm/common/built-in.o(.text)
		arch/arm/common/built-in.o(.rodata*)

		arch/arm/lib/lib.a(.text)
		arch/arm/lib/lib.a(.rodata*)
		arch/arm/lib/built-in.o(.text)
		arch/arm/lib/built-in.o(.rodata*)
		lib/built-in.o(.text)
		lib/built-in.o(.rodata*)

#if CONFIG_KERNEL_IN_ENVM_SIZE>128
		kernel/built-in.o(.text)
		kernel/built-in.o(.rodata*)
		ipc/built-in.o(.text)
		ipc/built-in.o(.rodata*)
		block/built-in.o(.text)
		block/built-in.o(.rodata*)
		security/built-in.o(.text)
		security/built-in.o(.rodata*)
		crypto/built-in.o(.text)
		crypto/built-in.o(.rodata*)
		firmware/built-in.o(.text)
		firmware/built-in.o(.rodata*)

#if CONFIG_KERNEL_IN_ENVM_SIZE>384
		drivers/built-in.o(.text)
		drivers/built-in.o(.rodata*)
		fs/built-in.o(.text)
		fs/built-in.o(.rodata*)
#if 0
		net/built-in.o(.text)
		net/built-in.o(.rodata*)
#endif

#endif /* CONFIG_KERNEL_IN_ENVM_SIZE>384 */
#endif /* CONFIG_KERNEL_IN_ENVM_SIZE>128 */
#endif /* CONFIG_KERNEL_IN_ENVM_SIZE>0 */

		_envm_end = .;
	} 

	. = _envm_loc;
#endif /* CONFIG_KERNEL_IN_ENVM */

	.text : {			/* Real text segment		*/
		_text = .;		/* Text and read-only data	*/
			__exception_text_start = .;
			*(.exception.text)
			__exception_text_end = .;
#if defined(CONFIG_INITRAMFS_IS_LARGE)
			__proc_info_begin = .;
				*(.proc.info.init)
			__proc_info_end = .;
#endif
			TEXT_TEXT
			SCHED_TEXT
			LOCK_TEXT
			KPROBES_TEXT
#ifdef CONFIG_MMU
			*(.fixup)
#endif
			*(.gnu.warning)
			*(.rodata)
			*(.rodata.*)
			*(.glue_7)
			*(.glue_7t)
		*(.got)			/* Global offset table		*/
	}

	RO_DATA(PAGE_SIZE)

	_etext = .;			/* End of text and rodata section */

#ifdef CONFIG_ARM_UNWIND
	/*
	 * Stack unwinding tables
	 */
	. = ALIGN(8);
	.ARM.unwind_idx : {
		__start_unwind_idx = .;
		*(.ARM.exidx*)
		__stop_unwind_idx = .;
	}
	.ARM.unwind_tab : {
		__start_unwind_tab = .;
		*(.ARM.extab*)
		__stop_unwind_tab = .;
	}
#endif

#ifdef CONFIG_XIP_KERNEL
	__data_loc = ALIGN(4);		/* location in binary */
	. = PAGE_OFFSET + TEXT_OFFSET;
#else
	. = ALIGN(THREAD_SIZE);
	__data_loc = .;
#endif

	.data : AT(__data_loc) {
		_data = .;		/* address in memory */
		_sdata = .;

		/*
		 * first, the init task union, aligned
		 * to an 8192 byte boundary.
		 */
		INIT_TASK_DATA(THREAD_SIZE)

#ifdef CONFIG_XIP_KERNEL
		. = ALIGN(PAGE_SIZE);
		__init_begin = .;
		INIT_DATA
		. = ALIGN(PAGE_SIZE);
		__init_end = .;
#endif

		NOSAVE_DATA
		CACHELINE_ALIGNED_DATA(32)

		/*
		 * The exception fixup table (might need resorting at runtime)
		 */
		. = ALIGN(32);
		__start___ex_table = .;
#ifdef CONFIG_MMU
		*(__ex_table)
#endif
		__stop___ex_table = .;

		/*
		 * and the usual data section
		 */
		DATA_DATA
		CONSTRUCTORS

		_edata = .;
	}
	_edata_loc = __data_loc + SIZEOF(.data);

#ifdef CONFIG_HAVE_TCM
        /*
	 * We align everything to a page boundary so we can
	 * free it after init has commenced and TCM contents have
	 * been copied to its destination.
	 */
	.tcm_start : {
		. = ALIGN(PAGE_SIZE);
		__tcm_start = .;
		__itcm_start = .;
	}

	/*
	 * Link these to the ITCM RAM
	 * Put VMA to the TCM address and LMA to the common RAM
	 * and we'll upload the contents from RAM to TCM and free
	 * the used RAM after that.
	 */
	.text_itcm ITCM_OFFSET : AT(__itcm_start)
	{
		__sitcm_text = .;
		*(.tcm.text)
		*(.tcm.rodata)
		. = ALIGN(4);
		__eitcm_text = .;
	}

	/*
	 * Reset the dot pointer, this is needed to create the
	 * relative __dtcm_start below (to be used as extern in code).
	 */
	. = ADDR(.tcm_start) + SIZEOF(.tcm_start) + SIZEOF(.text_itcm);

	.dtcm_start : {
		__dtcm_start = .;
	}

	/* TODO: add remainder of ITCM as well, that can be used for data! */
	.data_dtcm DTCM_OFFSET : AT(__dtcm_start)
	{
		. = ALIGN(4);
		__sdtcm_data = .;
		*(.tcm.data)
		. = ALIGN(4);
		__edtcm_data = .;
	}

	/* Reset the dot pointer or the linker gets confused */
	. = ADDR(.dtcm_start) + SIZEOF(.data_dtcm);

	/* End marker for freeing TCM copy in linked object */
	.tcm_end : AT(ADDR(.dtcm_start) + SIZEOF(.data_dtcm)){
		. = ALIGN(PAGE_SIZE);
		__tcm_end = .;
	}
#endif

	BSS_SECTION(0, 0, 0)
	_end = .;

	STABS_DEBUG
	.comment 0 : { *(.comment) }

	/* Default discards */
	DISCARDS
}

/*
 * These must never be empty
 * If you have to comment these two assert statements out, your
 * binutils is too old (for other reasons as well)
 */
ASSERT((__proc_info_end - __proc_info_begin), "missing CPU support")
ASSERT((__arch_info_end - __arch_info_begin), "no machine record defined")
