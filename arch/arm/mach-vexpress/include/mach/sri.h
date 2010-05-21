#ifndef ASMARM_ARCH_SRI_H
#define ASMARM_ARCH_SRI_H

#define SRI_TIMEOUT 250000

typedef enum
{
        SRI_CFG_OSC = 1,
        SRI_CFG_VOLT,
        SRI_CFG_AMP,
        SRI_CFG_TEMP,
        SRI_CFG_RESET,
        SRI_CFG_SYSCON,
        SRI_CFG_MUXFPGA,
        SRI_CFG_SHUTDOWN,
        SRI_CFG_REBOOT,
        SRI_CFG_BKUPDR
}
_sri_function_t;

typedef enum
{
        SRI_CFG_READ,
        SRI_CFG_WRITE
}
_sri_direction_t;

typedef struct
{
        _sri_function_t  function;
        _sri_direction_t direction;
        unsigned int     board;
        unsigned int     card;
        unsigned int     device;
        unsigned int     data;
}
_sri_info_t;

extern unsigned int vexpress_sri_transfer( _sri_info_t * sri_info);

#endif
