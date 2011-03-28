/*
 * Low-level services to access the eNVM of SmartFusion
 *
 * (C) Copyright 2011
 * Vladimir Khusainov, Emcraft Systems, vlad@emcraft.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#ifndef __MSS_ENVM_H__
#define __MSS_ENVM_H__

/*
 * Initialize the eNVM interface
 */
extern void mss_envm_init(void);

/*
 * Size of eNVM.
 * TO-DO: this needs to be derived from a SmartFusion chip variant.
 */
static inline unsigned long mss_envm_size(void)
{
	return (1024 * 256);
}

/*
 * Read a data buffer from eNVM
 */
extern int mss_envm_read(unsigned int offset, void * buf, unsigned int size);

/*
 * Write a data buffer to eNVM
 */
extern int mss_envm_write(unsigned int offset, void * buf, unsigned int size);

#endif /* __MSS_ENVM_H__ */
