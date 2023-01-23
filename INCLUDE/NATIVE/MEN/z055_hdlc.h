/***********************  I n c l u d e  -  F i l e  ***********************/
/*!
 *        \file  z055_hdlc.h
 *
 *      \author  Christian.Schuster@men.de
 *
 *       \brief  Header file for Z055_HDLC linux driver
 *               containing driver specific IOCTL codes,
 *               structures and defines used for getting status from and
 *               configuring driver and hardware
 *
 *    \switches  none
 *---------------------------------------------------------------------------
 * Copyright 2004-2020, MEN Mikro Elektronik GmbH
 ****************************************************************************/

 /*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _Z055_DRV_H_
#define _Z055_DRV_H_

#define HDLC_MAX_FRAME_SIZE	2046

//#define HDLC_FLAG_RXC_BRG			0x0200
//#define HDLC_FLAG_RXC_TXCPIN		0x0400
//#define HDLC_FLAG_RXC_RXCPIN		0x0800
//#define HDLC_FLAG_TXC_DPLL		0x1000
//#define HDLC_FLAG_TXC_BRG			0x2000
#define HDLC_FLAG_TXIDLE_FLAGS		0x0000
#define HDLC_FLAG_TXIDLE_ONES		0x0100
//#define HDLC_FLAG_TXIDLE_ZEROS	0x0200
#define HDLC_FLAG_TXIDLE_MARK		HDLC_FLAG_TXIDLE_ONES
#define HDLC_FLAG_ADDRCMP			0x0800
#define HDLC_FLAG_TXC_TRANSP		0x1000
#define HDLC_FLAG_TXC_DTRPIN		0x2000
#define HDLC_FLAG_TXC_RTSPIN		0x0000
#define HDLC_FLAG_HDLC_LOOPBACK	0x4000
#define HDLC_FLAG_FOUR_START_FLAGS	0x8000

#define HDLC_CRC_16			0x0000
#define HDLC_CRC_CCITT		0x0001
#define HDLC_CRC_INV		0x0010
#define HDLC_CRC_PRESET		0x0020
#define HDLC_CRC_RETURN_ERR	0x8000

#define RX_OK					0
#define RX_CRC_ERROR			1

#define HDLC_ENCODING_NRZ				0
#define HDLC_ENCODING_NRZI			1
#define HDLC_ENCODING_MANCHESTER		2
#define HDLC_ENCODING_MANCHESTER_NRZI		3
#define HDLC_ENCODING_NRZ_S			4

//#define HDLC_PREAMBLE_LENGTH_8BITS	0
//#define HDLC_PREAMBLE_LENGTH_16BITS	1
//#define HDLC_PREAMBLE_LENGTH_32BITS	2
//#define HDLC_PREAMBLE_LENGTH_64BITS	3

#define HDLC_PREAMBLE_PATTERN_NONE	0
//#define HDLC_PREAMBLE_PATTERN_ZEROS	1
//#define HDLC_PREAMBLE_PATTERN_FLAGS	2
//#define HDLC_PREAMBLE_PATTERN_10	3
//#define HDLC_PREAMBLE_PATTERN_01	4
//#define HDLC_PREAMBLE_PATTERN_ONES	5

#define Z055_MODE_HDLC		2

#define Z055_BUS_TYPE_PCI	1

#define HDLC_FULL_DUPLEX	0
#define HDLC_HALF_DUPLEX	1

typedef struct _Z055_PARAMS
{
	/* Common */

	unsigned long	mode;		/* Transmission mode (for now only HDLC) */

	/* HDLC Only */

	unsigned long	flags;
	unsigned char	encoding;	/* NRZ, NRZI, etc. */
	unsigned long	baud_rate;		/* baud rate */
	unsigned char	comp_addr;		/* receive compare address*/
	unsigned char	comp_broadc;	/* receive broadcast compare address,
									 * only when address compare is enabled */
	unsigned char	addr_mask;		/* receive compare address mask */
	unsigned char	broadc_mask;	/* receive compare broadcast address mask */
	unsigned char	crc_mode;	/* CRC16, CRC-CCITT, inverted,
								 * preset, return error frame */
	unsigned char	preamble;	/* number of preambles to send */
								/* preamble currently not supported */
	unsigned char	half_duplex;	/* half-duplex */
} Z055_PARAMS, *PZ055_PARAMS;

#define Z055_MAX_SERIAL_NUMBER 30

/*
** device diagnostics status
*/

#define DiagStatus_OK				0
#define DiagStatus_AddressFailure		1
#define DiagStatus_AddressConflict		2
#define DiagStatus_IrqFailure			3
#define DiagStatus_IrqConflict			4
#define DiagStatus_DmaFailure			5
#define DiagStatus_DmaConflict			6
#define DiagStatus_PciAdapterNotFound		7
#define DiagStatus_CantAssignPciResources	8
#define DiagStatus_CantAssignPciMemAddr		9
#define DiagStatus_CantAssignPciIoAddr		10
#define DiagStatus_CantAssignPciIrq		11
#define DiagStatus_MemoryError			12

#define SerialSignal_TXD				0x02     /* Transmit Data */
#define SerialSignal_RXD				0x08     /* Receive Data */
#define SerialSignal_CTS				0x10     /* Clear to Send */
#define SerialSignal_RTS				0x20     /* Request to Send */
#define SerialSignal_DSR				0x40     /* Data Set Ready */
#define SerialSignal_DTR				0x80     /* Data Terminal Ready */


/*
 * Counters of the input lines (CTS, DSR, DCD) interrupts
*/
struct Z055_ICOUNT {
	__u32	cts, dsr;
	__u32	txok;
	__u32	txbovr;
	__u32	txtimeout;
	__u32	rxinv;
	__u32	rxabort;
	__u32	rxcrc;
	__u32	rxrcst;
	__u32	rxbover;
	__u32	rxlong;
	__u32	rxok;
};

#define DEBUG_LEVEL_DATA	0x01
#define DEBUG_LEVEL_ERROR 	0x02
#define DEBUG_LEVEL_INFO  	0x04
#define DEBUG_LEVEL_BH    	0x08
#define DEBUG_LEVEL_ISR		0x10

/*
** Event bit flags for use with z055WaitEvent
*/

#define Z055EVENT_DSR	0x0001
//#define z055Event_DsrActive	0x0001
//#define z055Event_DsrInactive	0x0002
//#define z055Event_Dsr		0x0003
#define Z055EVENT_CTS	0x0004
//#define z055Event_CtsActive	0x0004
//#define z055Event_CtsInactive	0x0008
//#define z055Event_Cts		0x000c

/* Private IOCTL codes:
 *
 * Z055_IOCSPARAMS	set Z055_PARAMS structure values
 * Z055_IOCGPARAMS	get current Z055_PARAMS structure values
 * Z055_IOCSTXIDLE	set current transmit idle mode
 * Z055_IOCGTXIDLE	get current transmit idle mode
 * Z055_IOCTXENABLE	enable or disable transmitter
 * Z055_IOCRXENABLE	enable or disable receiver
 * Z055_IOCTXABORT	abort transmitting frame (HDLC)
 * Z055_IOCGSTATS	return current statistics
 * Z055_IOCWAITEVENT	wait for specified event to occur
 */
#define Z055_MAGIC_IOC		't'
#define Z055_IOCSPARAMS		_IOW(Z055_MAGIC_IOC,0,struct _Z055_PARAMS)
#define Z055_IOCGPARAMS		_IOR(Z055_MAGIC_IOC,1,struct _Z055_PARAMS)
#define Z055_IOCSTXIDLE		_IO(Z055_MAGIC_IOC,2)
#define Z055_IOCGTXIDLE		_IO(Z055_MAGIC_IOC,3)
#define Z055_IOCTXENABLE	_IO(Z055_MAGIC_IOC,4)
#define Z055_IOCRXENABLE	_IO(Z055_MAGIC_IOC,5)
#define Z055_IOCTXABORT		_IO(Z055_MAGIC_IOC,6)
#define Z055_IOCGSTATS		_IO(Z055_MAGIC_IOC,7)
#define Z055_IOCWAITEVENT	_IOWR(Z055_MAGIC_IOC,8,int)

#endif /* _Z55_DRV_H_ */
