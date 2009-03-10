/* 
   Copyright (C) 2004-2007 ARM Limited.

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License version 2
   as published by the Free Software Foundation.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

   As a special exception, if other files instantiate templates or use macros
   or inline functions from this file, or you compile this file and link it
   with other works to produce a work based on this file, this file does not
   by itself cause the resulting work to be covered by the GNU General Public
   License. However the source code for this file must still be made available
   in accordance with section (3) of the GNU General Public License.

   This exception does not invalidate any other reasons why a work based on
   this file might be covered by the GNU General Public License.
*/

#ifndef RVIDCC_INTERFACE_H
#define RVIDCC_INTERFACE_H

/*
 * These are the functions provided by the generic section that the OS-dependent
 * section is allowed to call
 */

#define RVIDCC_ERR_CORE_NOT_SUPPORTED 1

/* Initialise DCC comms driver.
 *
 * Note that this triggers the request to the RVI for a MAC address when using
 * virtual Ethernet.  Returns 0 on success, RVIDCC_ERR_* on failure
 */
extern int rvidcc_init(void);

/* Poll for DCC communications.
 *
 * If interrupts are not being used then this should be called periodically,
 * otherwise this is the interrupt handler for DCC communications.
 */
extern void rvidcc_poll(void);

#ifndef RVIDCC_RAW
/* Process received DCC data.
 *
 * This function is only relevant if using virtual Ethernet. This is a
 * 'bottom half' routine and processes the data captured in rvidcc_poll(). If
 * using interrupts, this routine should be scheduled to be called at the next
 * available opportunity once the interrupt handler has completed. If not
 * using interrupts then this can be called immediately on notification of the
 * arrival of incoming data.
 */
extern void rvidcc_process(void);
#endif

/* Handle low level reading of dcc information
 *
 * This should be called from the interrupt handler for the dcc read interrupt
 * to read data from the DCC register.  It is also called by rvidcc_poll.
 */
extern void rvidcc_read(void);

/* Handle low level writing of dcc information
 *
 * This should be called from the interrupt handler for the dcc write interrupt
 * to write data to the DCC register.  It is also called by rvidcc_poll.
 */
extern void rvidcc_write(void);

#ifdef RVIDCC_OUTBUF_DATA
/* Get the number of bytes left to be sent out to the ICE */
extern size_t rvidcc_outbuf_data(void);
#endif

/* Get the number of bytes available for reading from TTY.
 *
 * Zero is returned if no data is available.
 */
extern size_t rvidcc_serial_can_read(void);

/* Get the number of bytes that can be written to the TTY.
 *
 * Zero is returned if no data can be written.
 */
extern size_t rvidcc_serial_can_write(void);

/* Read and consume incoming TTY data.
 *
 * This is the primary TTY read routine.
 */
extern size_t rvidcc_read_serial(void *ptr, size_t len);

/* Write and queue up outgoing TTY data.
 *
 * This is the primary TTY write routine.
 */
extern size_t rvidcc_write_serial(const void *ptr, size_t len);

/* Send an IP packet out to the virtual network.
 *
 * The data provided must be a valid IP packet and must not contain an
 * Ethernet header. The data provided is all copied, if there is space on the
 * queue, othewise none of it is and the packet should be treated as still
 * pending. The return value is the number of bytes copied.
 */
extern int rvidcc_transmit_ip_packet(void *packet, size_t length);


#ifdef RVIDCC_SCAN_INPUT_FOR
/* Scan pending incoming TTY data for a character.
 *
 * This is primarily provided for detecting ^C (interrupt) on pending incoming
 * TTY data, e.g. to allow a gdb stub to be re-activated when a program is
 * running. It should be called from 'bottom half' processing, or from a
 * regular poll. The scan starts at the latest character received and works
 * backwards for a maximum of 'span' characters.
 */
extern int rvidcc_scan_input_for(char c, size_t span);
#endif


/*
 * Callbacks
 * These are called from the generic section (in rvidcc.c) and should be provided
 * by the OS-dependent section.
 */


/* DCC incoming data notification callback.
 *
 * This is called by rvidcc_poll() whenever new data is received on the DCC
 * channel. It indicates rvidcc_process() should be called at the next
 * available opportunity (and also rvidcc_scaninput_for() if required). If
 * using interrupts, schedule a 'bottom half' or tasklet to run. If not using
 * interrupts, rvidcc_process() can be called directly from here.
 */
extern void rvidcc_cb_notify(void);

#ifndef RVIDCC_RAW

/* Network IP packet received callback.
 *
 * This is called from rvidcc_process() whenever an IP packet is received. Note:
 * this only contains IP data without any Ethernet headers. The data must be
 * copied and passed on to the OS appropriately.
 */
extern void rvidcc_cb_ip_packet_received(void *packet, size_t length);

/* Pseudo ARP callback.
 *
 * This is called from rvidcc_process() whenever the ICE needs to determine if
 * it should direct IP packets for the given IP address to this processor.
 * Return non-zero if this is the case, or zero otherwise.  The parameter
 * passed is a pointer to 4-byte array which is the binary IP address in
 * network (big-endian) order.
 */
extern int rvidcc_cb_has_addr(unsigned char *ip_binary);

/* Set Ethernet MAC address callback.
 *
 * This is called from rvidcc_process() whenever in response to _init_dcc()'s
 * request to the ICE for a (globally unique) MAC address. This happens very
 * soon after initialisation, and if a MAC is required, then the virtual
 * Ethernet driver should not be considered fully up and running until this
 * has been received. The parameter is a pointer to a 6-byte array which is
 * the MAC address.
 */
extern void rvidcc_cb_set_mac_address(unsigned char *mac);

#endif


#endif // RVIDCC_INTERFACE_H
