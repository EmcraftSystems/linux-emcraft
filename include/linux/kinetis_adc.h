/*
 * "ioctl()" interface to the ADC driver for Freescale Kinetis MCUs
 *
 * (C) Copyright 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
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

#ifndef _KINETIS_ADC_H_
#define _KINETIS_ADC_H_

/* Enable an ADC module */
#define IOCTL_KINETISADC_ENABLE_ADC	_IO(KINETIS_ADC_MINOR,1)
/* Disable an ADC module */
#define IOCTL_KINETISADC_DISABLE_ADC	_IO(KINETIS_ADC_MINOR,2)
/* Make a single measurement on a specific ADC channel */
#define IOCTL_KINETISADC_MEASURE	_IOR(KINETIS_ADC_MINOR,3,u16)

struct kinetis_adc_request {
	/* ADC module: 0..3 - ADC0, ADC1, ADC2, ADC3 */
	unsigned int adc_module;
	/* Bits 4:0 = ADC channel (SC1n[ADCH]); bit 5 = mux sel ("a" or "b") */
	unsigned int channel_id;
	/* Measurement result */
	u16 result;
};

#endif /* _KINETIS_ADC_H_ */
