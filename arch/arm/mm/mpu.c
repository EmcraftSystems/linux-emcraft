/*
 *  linux/arch/arm/mm/mpu.c
 *
 *  Copyright (C) 2011,2012 Vladimir Khusainov, Emcraft Systems
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* 
 * This module provides MPU-based process protection for ARMv7.
 * Both process-to-kernel and process-to-process protection is supported.
 * An optional "stack redzone" feature is provided.
 * ...
 * This code has been tested on the Cortex-M3 core of:
 * - Microsemi's SmartFusion cSOC
 * - STmicro's STMF32F2
 * - NXP's LPC1788.
 * Minimally, it should work as is on any Cortex-M3/M4 core.
 * ...
 * Some changes to the MPU h/w handling code may be needed (i.e. I
 * don't know if changes are needed or not; I haven't studied that), in case
 * some ARMv7 processor has an MPU that is different from that of
 * Cortex-M3 (which is 8 regions; no separation of instruction data;
 * pages of various size; protection must be size-aligned).
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/swap.h>
#include <linux/file.h>
#include <linux/highmem.h>
#include <linux/pagemap.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/tracehook.h>
#include <linux/blkdev.h>
#include <linux/backing-dev.h>
#include <linux/mount.h>
#include <linux/personality.h>
#include <linux/security.h>
#include <linux/syscalls.h>
#include <linux/io.h>

#include <asm/uaccess.h>
#include <asm/tlb.h>
#include <asm/tlbflush.h>
#include <asm/mmu_context.h>
#include <asm/mmu.h>
#include <asm/setup.h>

/*
 * Enable / disable debug print-outs
 */
#if 0
#define DEBUG
#endif

/* 
 * Description of the MPU programming interfaces.
 * Specifically, we describe that part of NVIC that concerns
 * the MPU (Memory Fault) configuration and processing
 * as well as exception handlers configuration.
 */
#define NVIC_REGS_BASE	0xE000E004

struct nvic_regs {
	unsigned int	some_regs1[840];	/* e000e004 */
	unsigned int	system_handler_csr;	/* e000ed24 */
	unsigned int	fault_status;		/* e000ed28 */
	unsigned int	some_regs2[2];
	unsigned int	mfar;			/* e000ed34 */
	unsigned int	some_regs3[22];
	unsigned int	mpu_type;		/* e000ed90 */
	unsigned int	mpu_control;
	unsigned int	reg_number;
	unsigned int	reg_base;
	unsigned int	reg_attr;
};
#define NVIC		((struct nvic_regs *)(NVIC_REGS_BASE))

/*
 * For some Cortex-M processors (LPC1788 being an example, the MPU
 * will be already enabled and a first MPU protection region will be
 * taken when mpu_hw_init() is called. This is needed to allow
 * accesses to addresses at and above 0xA0000000, where external 
 * RAM resides for such processors. In other words, unless a bootloader
 * (such as U-Boot) performs the above-described MPU enabling and
 * initialization, the region above 0xA0000000 will have an attribute
 * XN (eXecute Never) and Cortex-M3 will not be able to run code
 * form external RAM, on such processors. 
 * ...
 * To account for such processors, we define a module-wide variable that
 * will be an index of the first MPU protection region available for
 * use in this module. 
 */
static int mpu_hw_reg_indx = 0;

/*
 * Set up an MPU protection region 
 */
static inline void mpu_region_write(int i, unsigned int b, unsigned int a)
{
	writel(b | (1<<4) | i, &NVIC->reg_base);
	writel(a, &NVIC->reg_attr);
}

/*
 * Read an MPU protection region 
 */
static void mpu_region_read(int i, unsigned int *b, unsigned int *a)
{
	writel(i, &NVIC->reg_number); 
	*b = readl(&NVIC->reg_base);
	*a = readl(&NVIC->reg_attr);
}

/*
 * Set up the MPU and related hardware interfaces.
 */
static void mpu_hw_on(void)
{
	unsigned int b, a;

#define MEMFAULTENA		(1<<16)
#define MPU_CONTROL_ENABLE	(1<<0)
#define MPU_CONTROL_PRIVDEFENA	(1<<2)
#define MPU_ATTR_AP_MSK		(7<<24)
#define MPU_ATTR_AP_PRVL_RW	(1<<24)

	/*
	 * As long as software does not attempt to expressly change
	 * the priority of MemManage, it remains at the highest priority
	 * and no other exception handlers will preempt the MemManage
	 * exception handler (with exception of Reset, NMI and HardFault). 
	 */

	/*
	 * Enable Memory Fault (MemManage) system handler
	 */
	writel(readl(&NVIC->system_handler_csr) | MEMFAULTENA,
		&NVIC->system_handler_csr);
	
	/*
 	 * Check if the MPU already on. This will be the case
 	 * for those processors that have external RAM at 0xA0000000
 	 * or above. For such processors, the MPU will have 
 	 * a single region defined, enabling R|W|X accesses
 	 * to the entire 4GB address space.
 	 * We must always preserve this mapping!
 	 * ...
 	 * There is yet another complication here. Not only we must
 	 * preserve that mapping we also must make sure that it is
 	 * valid only for privildged accesses and is disabled for
 	 * user-space applications.
 	 */
	if (readl(&NVIC->mpu_control) & MPU_CONTROL_ENABLE) {
		/*
		 * Make sure the mapping is valid only for the kernel
		 */
		mpu_region_read(0, &b, &a);
		a &= ~MPU_ATTR_AP_MSK;
		a |= MPU_ATTR_AP_PRVL_RW;
		mpu_region_write(0, b, a);

		/*
		 * Make sure the mapping is never affected
		 */
		mpu_hw_reg_indx = 1;
	}

	/*
	 * Turn the MPU on, enable the default map for
	 * privilidged accesses
	 */
	writel(MPU_CONTROL_PRIVDEFENA | MPU_CONTROL_ENABLE,
		&NVIC->mpu_control);
}

/*
 * Address region data structure. This represents a "mappable region".
 * There will always be a region for the main RAM but other regions
 * are possible as well, in case a configuration desires to allow mappings
 * into Flash, internal RAM, control registers, etc.
 */
typedef struct {
	unsigned long			bot;		/* Region base */
	unsigned long			top;		/* Region top */
} mpu_addr_region_t;

/*
 * Array of address regions
 */
#define MPU_ADDR_REGION_TBLSZ		16
static mpu_addr_region_t mpu_addr_region_tbl[MPU_ADDR_REGION_TBLSZ];

/*
 * Next entry to fill in in the array of address regions
 */
static int mpu_addr_region_next = 0;

/*
 * Per-process MPU context data structure
 * It is referenced, as an opaque pointer, 
 * from mm->context (in case CONFIG_MPU is on).
 */
typedef struct  {
	struct {
		unsigned int		base;		/* Region base */
		unsigned int 		attr;		/* Region attr */
	}				mpu_regs[8];	/* MPU regions copy */
	unsigned int			indx;		/* Next reg to use */
	unsigned int			barrier;	/* Locked regs bar */
#ifdef CONFIG_MPU_STACK_REDZONE
	unsigned long			redzone_bot;	/* Stack redzone btm */
	unsigned long			redzone_top;	/* Stack redzone top */
#endif
	unsigned int			page_tbl[0];	/* Page tables pool */
} mpu_context_t;

/*
 * Short-hand to a process' MPU context
 */
#define mpu_context_p(mm)	((mpu_context_t *) (mm)->context.p)

/*
 * These variables below are visible module-wide and provide geometry
 * of the "page tables" for the defined MPU address regions.
 * Each such page table is a bit-wise mask for the pages of the corresponing
 * MPU address region. The bit tells if a page is mapped (1) or unmapped (0).
 * We do not distinguish between READ / WRITE / EXEC mappings. 
 * This is because page mappings are implemented using 32K MPU protection
 * regions, with the 8 sub-regions of a region used to define mappings
 * for individul pages (an alternative would be to map using 4K MPU
 * protection regions with more frequent page misses and constant swap-outs).
 * With this decision though, there is a single mapping for a 32K 
 * region (for each there is just one specific set of the permissions 
 * defined - no separate permissions for the sub-regions). A 32K region
 * in general case spans more than one "virtual memory" mappings with
 * different permissions (i.e. text and data). Hence unified permissions
 * (always all of READ and WRITE and EXEC) for anything that gets mapped.
 * This is a bit a violation for the POSIX interfaces (e.g. a region
 * that is mmapp()-ed READ-ONLY will be possible to WRITE), however given
 * the accent on making sure that a process can't trash memory of
 * the kernel or other processes, is still an acceptable trade-off, since
 * the above goal is met.
 * ...
 * The variables below are:
 * - size of the entire pool for the page tables. Each region draws memory
 *   for its page table from that pool;
 * - number of full pages required for the pool;
 * - array of offsets to the base of a page table for each region.
 * We calculate this geometry just once so as not to repeat these
 * calcualtions any time the MPU context is created for a new process. 
 */
static unsigned long mpu_page_tbl_len = 0;
static unsigned long mpu_page_tbl_order;
static unsigned long mpu_page_tbl_off[MPU_ADDR_REGION_TBLSZ];

/*
 * Define an MPU address region. 
 */
static void mpu_addr_region_define(
	unsigned long b, unsigned long t)
{
	mpu_addr_region_t *r;

	/*
	 * Check that there is no over-allocation. If there is,
	 * just ignore that new region.
	 */
	if (mpu_addr_region_next == MPU_ADDR_REGION_TBLSZ) {
		printk("%s: unable to define MPU address region: [%lx,%lx]\n",
		       __func__, b, t);
		goto Done;
	}

	/*
	 * Store the regions settings.
	 */
	r = &mpu_addr_region_tbl[mpu_addr_region_next++];
	r->bot = b;
	r->top = t;

Done:	;
}

/*
 * Calculate geometry for all MPU address regions that have been defined
 */
static void mpu_addr_region_geometry(void)
{
	unsigned long s;
	int i;
	mpu_addr_region_t *r;

	for (i = 0, s = 0; i < mpu_addr_region_next; i++) {
		r = &mpu_addr_region_tbl[i];
		mpu_page_tbl_off[i] = s;
		s += sizeof(unsigned int) * 
		     ((((r->top - r->bot) >> PAGE_SHIFT) + 31) / 32);
#if defined(DEBUG)
		printk("%s: %d: b=0x%lx,t=0x%lx,sz=%ldK,s=%ld\n",
			__func__, i, r->bot, r->top,
			(r->top - r->bot)/1024, s);
#endif
	}

	/*
 	 * Now, calculate the size of the entire page tables pool
 	 * The pool is allocated as a number of 4K pages.
 	 */
	mpu_page_tbl_len = sizeof(mpu_context_t) + s;
	mpu_page_tbl_order =  max(1, get_order(mpu_page_tbl_len));
#if defined(DEBUG)
	printk("%s: tbl=%ld,ord=%ld\n",
		__func__, mpu_page_tbl_len, mpu_page_tbl_order);
#endif

}

/*
 * Find an MPU address region an address belongs to
 */
static inline mpu_addr_region_t * mpu_addr_region_find(unsigned long a)
{
	int i;
	int f;
	mpu_addr_region_t *r;

	for (i = 0, f = 0, r = 0; i < mpu_addr_region_next && !f; i++) {
		r = &mpu_addr_region_tbl[i];
		f = (r->bot <= a) && (a < r->top);
	}

	return f ? r : (mpu_addr_region_t *) 0;
}

/*
 * Debug service: print active MPU address regions
 */
#if defined(DEBUG)
static void mpu_addr_region_tbl_print(void)
{
	mpu_addr_region_t *r;
	int i;

	for (i = 0; i < mpu_addr_region_next; i++) {
		r = &mpu_addr_region_tbl[i];
		printk("%s: %d: %lx,%lx\n",
		      __func__, i, r->bot, r->top);
	}
}
#endif 

/*
 * Initialize the MPU context for a new process
 */
static inline void mpu_context_init(struct mm_struct *mm)
{
	unsigned long r;
	mpu_context_t *p;

	/*
 	 * If this is called for a first time (i.e. first user process),
 	 * calculate the page table geometry 
 	 */
	if (mpu_page_tbl_len == 0) {
		mpu_addr_region_geometry();
	}

	/*
	 * Allocate an MPU context structure for the process and
	 * zero it out. 
	 */
	r = __get_free_pages(GFP_KERNEL, mpu_page_tbl_order);
	if (! r) {
		printk("%s: no free_pages; stopping the kernel\n", __func__);
		for (;;);
	}
	memset((void *)r, 0, mpu_page_tbl_len);

	/*
	 * Store a pointer to the MPU context in mm->context.
	 */
	mm->context.p = (void *)r;

	/*
	 * Get access to the MPU context with an appropriate data pointer
	 */
	p = mpu_context_p(mm);

	/*
	 * Set up the next region to use
	 */
	p->indx = mpu_hw_reg_indx;
}

/*
 * Free the MPU context for a process
 */
static inline void mpu_context_free(struct mm_struct *mm)
{
	free_pages((unsigned long) mm->context.p, mpu_page_tbl_order);
}

/*
 * Set specified permissions for a page in the appropriate page table
 */
static inline void mpu_context_map_page(
	struct mm_struct *mm, unsigned int a, unsigned int f)
{
	mpu_context_t *p;
	mpu_addr_region_t *r;
	unsigned int *mask;
	unsigned int page;
	unsigned int idx;
	unsigned int bit;
	int i;	

	/*
	 * Check if there is even an MPU address region for this address
	 */
	r = mpu_addr_region_find(a);
	if (! r) {
		goto Done;
	}

	/*
 	 * Get access to: context; region index; page table
 	 */
	p = mpu_context_p(mm);
	i = r - mpu_addr_region_tbl;
	mask = p->page_tbl + mpu_page_tbl_off[i];

	/*
 	 * Calculate: relative page; page index; bit
 	 */
	page = (a - r->bot) >> PAGE_SHIFT;
	idx = page >> 5;
	bit = 1 << (page & 31);

	/*
 	 * Change mappings for the page.
 	 * Note that we do not distinguish between RD/WR/EXEC,
 	 * any permission results in a valid mapping.
 	 */
	if (f & (VM_READ | VM_WRITE | VM_EXEC)) mask[idx] |= bit;
	else					mask[idx] &= ~bit; 

Done:	;
}

/*
 * Is a specified page validly mapped within a specified address region?
 */
static inline int mpu_context_addr_valid_for_region(
	struct mm_struct *mm, mpu_addr_region_t *r,
	unsigned int a, unsigned int f)
{
	mpu_context_t *p = mpu_context_p(mm);
	unsigned int *mask;
	unsigned int page;
	unsigned int idx;
	unsigned int bit;
	int i;	
	int b = 0;

	/*
 	 * Get access to: region index; page table
 	 */
	i = r - mpu_addr_region_tbl;
	mask = p->page_tbl + mpu_page_tbl_off[i];

	/*
 	 * Calculate: relative page; page index; bit
 	 */
	page = (a - r->bot) >> PAGE_SHIFT;
	idx = page >> 5;
	bit = 1 << (page & 31);

	/*
 	 * Get mappings for the page.
 	 * Note that we do not distinguish between RD/WR/EXEC,
 	 * any permission results in a valid mapping.
 	 */
	b = (mask[idx] & bit) ? 1 : 0; 

	return b;
}

/*
 * Is a specified address validly mapped for the context?
 */
static inline int mpu_context_addr_valid(
	struct mm_struct *mm, unsigned int a, unsigned int f)
{
#ifdef CONFIG_MPU_STACK_REDZONE
	mpu_context_t *p = mpu_context_p(mm);
#endif
	mpu_addr_region_t *r;
	int b = 0;

	/*
 	 * Check if the address is not in the stack redzone
 	 */
#ifdef CONFIG_MPU_STACK_REDZONE
	if ((p->redzone_bot <= a) && (a < p->redzone_top)) {
		goto Done;
	}	
#endif

	/*
	 * Check if there is even an MPU address region for this address
	 */
	r = mpu_addr_region_find(a);
	if (! r) {
		goto Done;
	}
	
	b = mpu_context_addr_valid_for_region(mm, r, a, f);

Done:	return b;
}

/*
 * Fill in a next available MPU protection region with a specified
 * mapping. If needed, this swaps out an existing mapping using 
 * a simple round-robin algorithm, which affects only MPU regions
 * that have the priority above the barrier (i.e. all regions
 * below barrier are locked).
 */
static inline void mpu_page_map(
	struct mm_struct *mm, unsigned long a, unsigned int s,
	unsigned int sz, unsigned int acs, unsigned int srd, unsigned int xn)
{
	mpu_context_t *p = mpu_context_p(mm);

	/*
 	 * This is next MPU region for this process
 	 */
	unsigned int i = p->indx;

	/*
 	 * Prepare region settings
 	 */
	unsigned int b = a & ~(s - 1);
	unsigned int t = 0x1 | sz<<1 | acs<<24 | srd<<8 | xn<<28;

	/*
 	 * Copy them to the context so we can restore them on a switch.
 	 */
	p->mpu_regs[i].base = b;
	p->mpu_regs[i].attr = t;

	/*
 	 * Set up an MPU region in the hardware.
 	 */
	mpu_region_write(i, b, t);

	/*
 	 * Set the context for a next region allocation.
 	 */
	p->indx = i == 7 ? p->barrier : i + 1;
}

/*
 * Set a "priority barrier" for the protection region for 
 * the specified barrier. Further MPU mappings will round-robin
 * above that barrier but will not touch any mappings below it.
 */
static inline void mpu_page_barrier(
	struct mm_struct *mm, unsigned int barrier)
{
	mpu_context_p(mm)->barrier = barrier;
}

/*
 * Set up the MPU protections region from the context
 * of the specified process (in preparation to re-starting it).
 */
static inline void mpu_page_copyall(struct mm_struct *mm)
{
	int i;
	mpu_context_t *p = mpu_context_p(mm);

	for (i = mpu_hw_reg_indx; i < 8; i ++) {
		mpu_region_write(i, p->mpu_regs[i].base, p->mpu_regs[i].attr);
	}	
}

/*
 * Non-standard pages used by this module
 */
#define PAGE32K_SIZE	(32 * 1024)
#define PAGE32K_MASK	(~(PAGE32K_SIZE - 1))
#define PAGE256B_SIZE	256
#define PAGE256B_MASK	(~(PAGE256B_SIZE - 1))

/*
 * Calculate settings for sub-regions in a 32K page
 */
static inline int mpu_page_srd(
	struct mm_struct *mm, unsigned long a, unsigned long f)
{
	mpu_addr_region_t *r;
	unsigned long pb;
	unsigned long pt;
	unsigned long p;
	unsigned int srd;
	unsigned int x;
	int s;

	/*
	 * Check if there is even an MPU address region for this address
	 */
	r = mpu_addr_region_find(a);
	if (! r) {
		goto Done;
	}

	/*
 	 * These are the boundaries for the 32K region
 	 */
	pb = a & PAGE32K_MASK;
	pt = pb + PAGE32K_SIZE;

	/*
 	 * Only those 4K pages that are mapped in the context
 	 * get mapped as sub-regions.
 	 */
	for (srd = 0xFF, s = 0, p = pb + PAGE_SIZE-1;
	     p < pt; p += PAGE_SIZE, s ++) {
		x = mpu_context_addr_valid_for_region(mm, r, p, f);
		srd &= ~(x << s);
	}

Done:	return srd;
}

/* 
 * Map in a 32K protection region. Access is ANY (RD/WR/EXEC).
 * Sub-regions define mappings (or not) for the surounding pages.
 */
static inline void mpu_page_map_32k(
	struct mm_struct *mm, unsigned long a, unsigned int f)
{
	unsigned int srd = mpu_page_srd(mm, a, f);

	mpu_page_map(mm, a, PAGE32K_SIZE, 0xE, 0x3, srd, 0);
}

/* 
 * Map in a 256K protection region. Access is NONE for user mode. 
 */
static inline void mpu_page_map_256b(
	struct mm_struct *mm, unsigned long a, unsigned int f)
{
	mpu_page_map(mm, a, PAGE256B_SIZE, 0x7, 0x1, 0, 1);
}

/*
 * Debug service: print current MPU regions
 */
#if defined(DEBUG)
static void mpu_page_printall(const char * s, struct mm_struct *mm)
{
	unsigned int b;	
	unsigned int a;	
	int i;
	mpu_context_t *p = mpu_context_p(mm);

	for (i = 0; i < 8; i ++) {
		mpu_region_read(i, &b, &a);
		printk("%s: reg=%d: cb=%08x,ca=%08x,mb=%08x,ma=%08x\n", s, i,
			p->mpu_regs[i].base, p->mpu_regs[i].attr, b, a);
	}	
}
#endif /* DEBUG */

/*
 * Section below is a set of services exported by this module.
 */

/*
 * Define a mappable region
 */
void mpu_region_define(unsigned long b, unsigned long t)
{
	mpu_addr_region_define(b, t);
}

/* 
 * Enable the MPU hardware as well as initalize related data structures
 * This is called from the architecture-specific initilization code.
 */
void mpu_init(void)
{
	/*
	 * Figure out the RAM geometry. This code implements a very
	 * simplistic approach; we assume there is a single bank of RAM.
	 */
	unsigned int ram_bot = bank_phys_start(& meminfo.bank[0]);
	unsigned int ram_top = ram_bot + bank_phys_size(& meminfo.bank[0]);
	mpu_region_define(ram_bot, ram_top);

	/*
	 * Turn the MPU and related h/w intefaces on.
	 */
	mpu_hw_on();
}

/*
 * Store new permissions for a particular page in the MPU context.
 * This is called by the architecture-neutral code in do_mmap.
 */
void protect_page(struct mm_struct *mm, unsigned long addr,
		  unsigned long flags)
{
	mpu_context_map_page(mm, addr, flags);
#if 0 && defined(DEBUG)
	printk("%s: addr=0x%lx\n", __func__, addr);
#endif
}

/*
 * Update the MPU settings after permissions have changed for a mm area
 * This is called by the architecture-neutral code related to do_mmap.
 */
void update_protections(struct mm_struct *mm)
{
}

#ifdef CONFIG_MPU_USER_ACCESS
/*
 * The MPU version of addr_ok used by uaccess.h to check
 * validiry of a user-space address
 */
int mpu_addr_ok(const void * addr)
{
	return mpu_context_addr_valid(current->mm, (unsigned long) addr,
		  		      VM_READ | VM_WRITE);
}
EXPORT_SYMBOL(mpu_addr_ok);
#endif

/*
 * Create the MPU context for a new process.
 * This is called from init_new_context.
 */
int mpu_init_new_context(struct task_struct *tsk, struct mm_struct *mm)
{
	mpu_context_init(mm);

	return 0;
}

/*
 * Free up the the MPU context for a process.
 * This is called from destroy_context.
 */
void mpu_destroy_context(struct mm_struct *mm)
{
	mpu_context_free(mm);
}

/*
 * MPU-specific part of switch_mm
 */
void mpu_switch_mm(struct mm_struct *prev, struct mm_struct *next)
{
	/*
 	 * Set up the MPU protection regions with the values
 	 * that were used by the new process last time it 
 	 * was called. This will be all-disabled for a first call
 	 * because the initial table in the context is all zeroes.
 	 */
	mpu_page_copyall(next);
}

/*
 * Barrier for the number of 16K pages allocated to the stack
 */
#define MPU_STACK_BAR	4

/*
 * MPU-specific part of start_thread.
 * This process is about to go to the user mode.
 */
void mpu_start_thread(struct pt_regs * regs)
{
	struct mm_struct *mm = current->mm;
	unsigned long t;
	unsigned long b;
	unsigned long p;
	int i;

#if defined(DEBUG)
	printk("%s: control=%x,indx=%d\n",
		__func__, readl(&NVIC->mpu_control), mpu_hw_reg_indx);
	printk("%s,%d: end_stack=%lx,start_stack=%lx,len_stack=%ld\n",
	        __func__, __LINE__,
               mm->context.end_brk, mm->start_stack,
               mm->start_stack - mm->context.end_brk);
	mpu_addr_region_tbl_print();
#endif

	/*
 	 * Set up mappings for the stack.
 	 */
	for (t = mm->start_stack;
	     mpu_context_addr_valid(mm, t, (VM_READ | VM_WRITE));
	     t += PAGE_SIZE);
	t &= PAGE_MASK;
	b = mm->context.end_brk;

#if defined(DEBUG)
	printk("%s,%d: b=%lx,t=%lx\n", __func__, __LINE__, b, t);
#endif

	/* 
 	 * Disable interrupts since it is important to leave
 	 * the MPU hardware in a consistent state
 	 */
	local_irq_disable();

	/*
 	 * Map in the stack pages
 	 */
	for (i = mpu_hw_reg_indx, p = b & PAGE32K_MASK;
             p <= t && i < MPU_STACK_BAR;
             i ++, p += PAGE32K_SIZE) {
		mpu_page_map_32k(mm, p, (VM_READ | VM_WRITE));
#if defined(DEBUG)
		printk("%s:%d: p=%lx\n", __func__, __LINE__, p);
#endif
	}

	/*
 	 * If the user has configured a stack "red zone",
 	 * disable a 256 bytes region at the bottom of the stack.
 	 */
#ifdef CONFIG_MPU_STACK_REDZONE
	p = (b + PAGE256B_SIZE - 1) & PAGE256B_MASK;
	mpu_page_map_256b(mm, p, 0);

	/* 
	 * Save the redzone geometry in the context
	 */
	mpu_context_p(mm)->redzone_bot = p;
	mpu_context_p(mm)->redzone_top = p + PAGE256B_SIZE;
#endif

	/*
 	 * Enable interrupts
 	 */
	local_irq_enable();

	/*
 	 * Stack is large; we are not able to fit it in.
 	 */
	if (i == MPU_STACK_BAR) {
		printk("MPU: stack too large for '%s';", current->comm);
		printk("; killing process\n");

		/*
 	 	 * Send a SIGSEGV to the process
 	 	 */
		send_sig(SIGSEGV, current, 0);
		goto Done;
	}

	/*
 	 * Stack is mapped. Set a barrier in the MPU context
 	 * so that the round-robin page mapping doesn't touch
 	 * the mappings for the stack.
 	 */
	mpu_page_barrier(mm, i);

#ifdef DEBUG
	mpu_page_printall(__func__, mm);
#endif

Done:	;
}

/*
 * Memory Fault (memmanage) exception handler.
 * This is called from the architecture-specific low-level
 * exception handler. It is assumed that this code doesn't
 * get pre-emtped. 
 */
asmlinkage void __exception do_memmanage(struct pt_regs *regs)
{
	unsigned int fault_status;
	unsigned long mfar;
	unsigned long pc;
	unsigned long addr = 0;
	unsigned long flags;
	struct mm_struct *mm = current->mm;

	/*
 	 * Read the fault status register from the MPU hardware
 	 */
	fault_status = readl(&NVIC->fault_status);

	/* 
 	 * Calculate a candidate offending address
 	 */
	mfar = readl(&NVIC->mfar);
	pc = instruction_pointer(regs);

	/*
	 * Clean up the memory fault status register for a next exception
	 */
	writel(fault_status, &NVIC->fault_status);

	/*
	 * Did we get a valid address in the memory fault address register?
	 * If so, this is a data access failure (can't tell read or write).
	 */
	if ((fault_status & 0xF0) == 0x80) {
		addr = mfar;
		flags = VM_READ | VM_WRITE;
	}
	/* 
	 * If there is no valid address, did we get an instuction access
	 * failure? If so, this is a code access failure.
	 */
	else if (fault_status & 0x1) {
		addr = pc;
		flags = VM_EXEC;
	}

	/* 
	 * Some error bit set in the memory fault address register.
	 * This must be MUNSTKERR due to a stacking error, which
	 * implies that we have gone beyond the low stack boundary
	 * (or somehow SP got incorrect).
	 * There is no recovery from that since no registers
	 * were saved on the stack on this exception, that is,
	 * we have no PC saved to return to user mode.
	 */
	else {
		goto Kill;
	}

	/* 
	 * We have a potentially benign exception. See if there is a valid
	 * mapping for the offending address and if so, "map" it in.
	 * The only exception is the stack redzone (if configured).
	 * Note: This code doesn't esnure XN (i.e. code protection),
	 * although it would be easy to do so. All pages that are mapped
	 * are READ/WRITE/EXEC.
	 */
	if (mpu_context_addr_valid(mm, addr, flags)) {
		mpu_page_map_32k(mm, addr, flags);
	}
	/*
	 * No mapping for the offending address. Kill the process.
	 */
	else {
		goto Kill;
	}

	/*
 	 * All is done successfully. Go to the end.
 	 */
	goto Done;

Kill:
	/*
 	 * This is a real MPU fault because the procees did a bad access
 	 */
#ifdef CONFIG_DEBUG_USER
	/*
	 * One must pass user_debug=8 to the kernel to make this print
	 */
	if (user_debug & UDBG_SEGV) {
		printk("\n\n%s: unhandled MPU fault (0x%02x) at 0x%08lx",
		       current->comm, fault_status, addr);
		printk(" [pc=0x%08lx,sp=0x%08lx]\n\n", pc, regs->ARM_sp);
	}
#endif

	/*
 	 * Send a SIGSEGV to the process
 	 */
	send_sig(SIGSEGV, current, 0);

Done:	;
}
