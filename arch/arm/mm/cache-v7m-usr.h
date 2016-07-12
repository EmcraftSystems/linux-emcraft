#ifndef _CACHE_V7M_USR_H_
#define _CACHE_V7M_USR_H_

#include <linux/ioctl.h>

/*
 * Descriptor of the region
 */
struct cache_v7m_reg {
	unsigned long	start;	/* Start address of the region	*/
	unsigned long	len;	/* Length of the region		*/
};

/*
 * Device major number
 */
#define V7M_CACHE_MAJOR		240

/*
 * Operations to apply to the region
 */
#define CACHE_V7M_IOC_CLEAN	_IOW(V7M_CACHE_MAJOR, 1, struct cache_v7m_reg *)
#define CACHE_V7M_IOC_INV	_IOW(V7M_CACHE_MAJOR, 2, struct cache_v7m_reg *)
#define CACHE_V7M_IOC_FLUSH	_IOW(V7M_CACHE_MAJOR, 3, struct cache_v7m_reg *)

#endif /* _CACHE_V7M_USR_H_ */
