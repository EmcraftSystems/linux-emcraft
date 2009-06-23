#include <linux/linkage.h>
#include <asm/assembler.h>
/*
 * Function: v7m_early_abort
 */
	.align	5
ENTRY(v7m_early_abort)
	mov	pc, lr
ENDPROC(v7m_early_abort)
