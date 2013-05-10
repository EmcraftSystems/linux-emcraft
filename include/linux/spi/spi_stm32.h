/*
 * Copyright (C) 2013
 * Vladimir Khusainov, Emcraft Systems, vlad@emcraft.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _SPI_STM32_H_
#define _SPI_STM32_H_

/*
 * Slave-specific parameters that the SPI controller driver cares about
 */
struct spi_stm32_slv {
	int				cs_port;	/* CS GPIO port */
	int				cs_pin;		/* CS GPIO pin */
	int				timeout;	/* Timeout in secs */
};

#endif	/*_SPI_STM32_H_ */
