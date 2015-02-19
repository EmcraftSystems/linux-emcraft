#ifndef _STM32F4_FB_H
#define _STM32F4_FB_H

#include <linux/types.h>

/*
 * These are the fields of control descriptor for every layer
 */
struct stm32f4_layer_desc {
        u32 layer_num;
        u32 width;
        u32 height;
        u32 posx;
        u32 posy;
        u32 addr;
        u32 en;
} __attribute__ ((packed));

#endif /* _STM32F4_FB_H */
