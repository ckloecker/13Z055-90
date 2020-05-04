/***********************  I n c l u d e  -  F i l e  ***********************/
/*!
 *        \file  z055_hdlc_int.h
 *
 *      \author  Christian.Schuster@men.de
 *
 *       \brief  Internal header file for Z055_HDLC linux driver
 *               containing driver internal defines, structures
 *
 *    \switches  none
 *

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

#ifndef _Z055_HDLC_INT_H_
#define _Z055_HDLC_INT_H_

#include <MEN/z055_hdlc.h>
#include <MEN/men_typs.h>
#include <MEN/maccess.h>
#define Z055_HDLC_MAGIC 0x5401

/*
 * Hardware Ressources
 */
#define SYSTEM_CLOCK_FREQUENCY	33333333
#define TX_SWITCH_BUFFERS		2

#define Z055_ADDR_SIZE	0x2000		/**< mem size to request for device
									  *  0x1000 for registers
									  *  0x0800 RxBuf (2kBytes max)
									  *  0x0800 TxBuf (2kBytes max)
									  */


#define BOOLEAN int
#define TRUE 1
#define FALSE 0

#if LINUX_VERSION_CODE < VERSION(3,7,0)
#define tty_cflags(tty) ((tty)->termios->c_cflag)
#else
#define tty_cflags(tty) ((tty)->termios.c_cflag)
#endif

/* The queue of BH actions to be performed */
#define BH_RECEIVE  1
#define BH_TRANSMIT 2
#define BH_STATUS   4

#define RELEVANT_IFLAG(iflag) (iflag & (IGNBRK|BRKINT|IGNPAR|PARMRK|INPCK))

struct	_input_signal_events {
	int	dsr;
	int	cts;
};

/* transmit holding buffer definitions*/
#define Z055_MAX_BUFFERS 10
#define BUFFER_STATUS_READY 0x0001

struct RXTX_BUFFER_S {
	u8 *buffer;				/**< data buffer */
	u16 ccount;				/**< character count of data in buffer*/
	u16 status;				/**< flags with status of buffer */
	struct RXTX_BUFFER_S *next;	/**< link to next buffer in list */
};
struct RXTX_BUFFER_QUEUE_S {
	int 	num_buffers;				/* number of buffers allocated */
	int 	buffers_used;				/* number of buffers waiting */
	struct RXTX_BUFFER_S *get_buffer;	/* next buffer to be loaded */
	struct RXTX_BUFFER_S *put_buffer;	/* next buffer to be emptied */
	u8		*buffer_start;				/* start of buffer (address allocated) */
};

/*
 * Device instance data structure
 */

struct Z055_STRUCT {
	void	*if_ptr;	/* General purpose pointer (used by SPPP) */
	int						magic;
	int						flags;

	int						line;
	int						hw_version;

	struct Z055_ICOUNT	icount;

	struct termios			normal_termios;
#if LINUX_VERSION_CODE >= VERSION(2,6,27)
	struct tty_port port;
#else
	struct tty_struct		*tty;
	int									count;		/* count of opens */
	unsigned short			close_delay;
	unsigned short			closing_wait;	/* time to wait before closing */
	int									blocked_open;	/* # of blocked opens */
	wait_queue_head_t		open_wait;
	wait_queue_head_t		close_wait;
#endif

	int						timeout;
	int						x_char;		/* xon/xoff character */

	u16						read_status_mask;
	u16						ignore_status_mask;
	unsigned char 			*xmit_buf;
	int						xmit_head;
	int						xmit_tail;
	int						xmit_cnt;

	wait_queue_head_t		status_event_wait_q;
	wait_queue_head_t		event_wait_q;
	struct timer_list		tx_timer;	/* HDLC transmit timeout timer */
	struct Z055_STRUCT		*next_device;	/* device list link */

	spinlock_t irq_spinlock;		/* spinlock for synchronizing with ISR */
	struct work_struct task;		/* task structure for scheduling bh */

	u32 			EventMask;			/* event trigger mask */
	u32 			RecordedEvents;		/* pending events */

	u32 			max_frame_size;		/* as set by device config */

	u32				pending_bh;

	int 			bh_running;			/* Protection from multiple */
	int 			isr_overflow;
	int 			bh_requested;

	int 			cts_chkcount;		/* too many IRQs if a signal */
	int 			dsr_chkcount;		/* is floating */

	struct RXTX_BUFFER_QUEUE_S rx_buffer_q;

	int rx_enabled;
	int rx_overflow;
	int rx_rcc_underrun;

	int tx_enabled;
	int tx_active;
	int tx_busy;			/* status of switch buffer */
	u32 idle_mode;

	u16 cmr_value;
	u16 hcr_value;

	char device_name[25];		/* device instance name */

	unsigned int bus_type;		/* expansion bus type (PCI) */
	unsigned char bus;			/* expansion bus number (zero based) */
	unsigned char function;		/* PCI device number */

	U_INT32_OR_64 phys_base;		/* base address of adapter */
	unsigned int phys_addr_size;/* size of the I/O address range */
	int addr_requested;			/* nonzero if I/O address requested */

	unsigned char *ma_base;		/* mapped base address */
	unsigned int ma_offs;		/* offset from ma_base to register start*/

	unsigned int irq;			/* interrupt level */
	unsigned long irq_flags;
	int irq_requested;			/* nonzero if IRQ requested */

	unsigned int dma_level;		/* DMA channel */
	int dma_requested;			/* nonzero if dma channel requested */

	u16 mbre_bit;
	u16 loopback_bits;
	u16 z055_hw_idle_mode;

	Z055_PARAMS params;				/* communications parameters */

	unsigned char serial_signals;	/* current serial signal states */

	int irq_occurred;			/* for diagnostics use */
	unsigned int init_error;	/* Initialization startup error 	(DIAGS)	*/
	int	fDiagnosticsmode;		/* Driver in Diagnostic mode?		(DIAGS)	*/

	u32 misc_ctrl_value;
	BOOLEAN drop_rts_on_tx_done;

	BOOLEAN loopmode_insert_requested;
	BOOLEAN	loopmode_send_done_requested;

	struct	_input_signal_events	input_signal_events;
};

/*
 * define operators on tty values to decouple driver code from
 * changing value locations of different kernel versions
 */

#if LINUX_VERSION_CODE >= VERSION(2,6,27)
#define Z055_STRUCT_set_tty(info, a)          ((info)->port.tty = (a))
#define Z055_STRUCT_get_tty(info)             ((info)->port.tty)
#define Z055_STRUCT_ref_count(info)           ((info)->port.count)
#define Z055_STRUCT_set_ref_count(info, a)    ((info)->port.count = (a))
#define Z055_STRUCT_inc_ref_count(info)       ((info)->port.count++)
#define Z055_STRUCT_dec_ref_count(info)       ((info)->port.count--)
#define Z055_STRUCT_flags(info)               ((info)->port.flags)
#define Z055_STRUCT_set_flags(info, a)        ((info)->port.flags |= (a))
#define Z055_STRUCT_clear_flags(info, a)      ((info)->port.flags &= ~(a))
#define Z055_STRUCT_open_wait_q(info)         ((info)->port.open_wait)
#if LINUX_VERSION_CODE < VERSION(4,4,0) && !defined(RHEL_7_3_514)
#define Z055_STRUCT_close_wait_q(info)               ((info)->port.close_wait)
#endif
#if LINUX_VERSION_CODE < VERSION(3,9,0)
#define Z055_STRUCT_set_low_latency(info, a)  ((info)->port.tty->low_latency = (a))
#else
#define Z055_STRUCT_set_low_latency(info, a)  ((info)->port.low_latency = (a))
#endif
#define Z055_STRUCT_blocked_open(info)        ((info)->port.blocked_open)
#define Z055_STRUCT_inc_blocked_open(info)    ((info)->port.blocked_open++)
#define Z055_STRUCT_dec_blocked_open(info)    ((info)->port.blocked_open--)
#define Z055_STRUCT_get_close_delay(info)     ((info)->port.close_delay)
#define Z055_STRUCT_set_close_delay(info, a)  ((info)->port.close_delay = (a))
#define Z055_STRUCT_get_closing_wait(info)    ((info)->port.closing_wait)
#define Z055_STRUCT_set_closing_wait(info, a) ((info)->port.closing_wait = (a))
#else
#define Z055_STRUCT_set_tty(info, a)          ((info)->tty = (a))
#define Z055_STRUCT_get_tty(info)             ((info)->tty)
#define Z055_STRUCT_ref_count(info)           ((info)->count)
#define Z055_STRUCT_set_ref_count(info, a)    ((info)->count = (a))
#define Z055_STRUCT_inc_ref_count(info)       ((info)->count++)
#define Z055_STRUCT_dec_ref_count(info)       ((info)->count--)
#define Z055_STRUCT_flags(info)               ((info)->flags)
#define Z055_STRUCT_set_flags(info, a)        ((info)->flags |= (a))
#define Z055_STRUCT_clear_flags(info, a)      ((info)->flags &= ~(a))
#define Z055_STRUCT_open_wait_q(info)         ((info)->open_wait)
#define Z055_STRUCT_close_wait_q(info)        ((info)->close_wait)
#define Z055_STRUCT_set_low_latency(info, a)  ((info)->tty->low_latency = (a))
#define Z055_STRUCT_blocked_open(info)        ((info)->blocked_open)
#define Z055_STRUCT_inc_blocked_open(info)    ((info)->blocked_open++)
#define Z055_STRUCT_dec_blocked_open(info)    ((info)->blocked_open--)
#define Z055_STRUCT_get_close_delay(info)     ((info)->close_delay)
#define Z055_STRUCT_set_close_delay(info, a)  ((info)->close_delay = (a))
#define Z055_STRUCT_get_closing_wait(info)    ((info)->closing_wait)
#define Z055_STRUCT_set_closing_wait(info, a) ((info)->closing_wait = (a))
#endif

#if LINUX_VERSION_CODE < VERSION(3,7,0)
#define tty_cflags(tty) ((tty)->termios->c_cflag)
#else
#define tty_cflags(tty) ((tty)->termios.c_cflag)
#endif

/*
 * These macros define the offsets used in calculating the
 * I/O address of the specified Setup registers.
 */

#define Z055_HCR		0x0000	/* HDLC Control Register (HCR) */
#define Z055_CDR		0x0002	/* Coding Register (CDR) */
#define Z055_ACR		0x0004	/* Address Compare Register (ACR) */
#define Z055_AMR		0x0006	/* Address Mask Register (AMR) */
#define Z055_BCL		0x0008	/* Baud Constant Low Word (BCL) */
#define Z055_BCH		0x000A	/* Baud Constant High Word (BCH) */
#define Z055_BCR		0x000C	/* Baudrate Generator Control Register (BCR) */
#define Z055_IER		0x0010	/* Interrupt Enable Register (IRER) */
#define Z055_IRQR		0x0012	/* Interrupt Request Register (IRQR) */
#define Z055_HSCR		0x0014	/* Handshake Control Register (HSCR) */
#define Z055_HSSR		0x0016	/* Handshake Status Register (HSSR) */
#define Z055_RES1		0x0018..0x001F	/* Reserved */
#define Z055_TXSZR		0x0020	/* Tx Frame Size Register (TXSZR) */
#define Z055_RXSZR		0x0022	/* Rx Frame Size Register (RXSZR) */
#define Z055_RES2		0x003C-0x0FFF	/* Reserved */
#define Z055_RXBUF		0x1000	/* Receive buffer (read only) */
#define Z055_TXBUF		0x1800	/* Transmit buffer (write only) */
#define Z055_BUF_SIZE		0x0400	/* Buffer Size, default: 1 kB used (0xFFF) */
/*
 * MACRO DEFINITIONS FOR HDLC CTRL REG
 */

#define Z055_HCR_SWRST		0x8000 	/**< Software Reset */
										/**<0: No software reset is activated \n
										 *  1: Software activated,
										 *     reset by design */
#define Z055_HCR_LOOPB		0x4000	/**< Loopback mode */
										/**<0: No loopback mode enabled\n
										 *  1: Loopback mode enabled.*/
#define Z055_HCR_CRCPRS		0x2000	/**< CRC preset value */
#define Z055_HCR_CRCCCITT	0x1000	/**< Used CRC polynom */
										/**<0: CRC-16 is used
										 *     (not yet supported by HW)\n
										 *  1: CRC-CCITT is used */
#define Z055_HCR_CRCINV		0x0800	/**< CRC inverse */
										/**<0: CRC is sent not inverted\n
										 *  1: CRC is sent inverted */

#define Z055_HCR_TXIDLE		0x0040	/**< Transmitter state during idle */
										/**<0: Tx sends flags during idle\n
										 *  1: Tx sends marking 1 during idle*/

#define Z055_HCR_SENDBRK		0x0020	/**< Send Abort */
										/**<0: No abort is send\n
										 *  1: Force transmitter to send
										 *     abort frame */

#define Z055_HCR_TXEN		0x0010	/**< Transmitter enable */
										/**<0: Tx is disabled and can be started
										 *     when transmit data is written\n
										 *  1: Transmitter is enabled and starts
										 *     to send */
#define Z055_HCR_ADDRCMPEN	0x0002	/**< Address compare enable */
										/**<0: Address comparison with
										 *     ACR and AMR disalbed\n
										 *  1: Address comparison with
										 *     ACR and AMR enabled */

#define Z055_HCR_RXEN		0x0001	/**< Receiver enable */
										/**<0: Receiver is disabled\n
									 *  1: Receiver is enabled */

/* macros used in driver to set modes */
#define Z055_IDLE_MASK		0x0040	/**< used to clear bits setting the idle mode */
#define Z055_IDLE_FLAG		0x01	/**< Send flags when idling */
#define Z055_IDLE_MARK1		0x02	/**< Send mark '1' when idling */
/*
 * MACRO DEFINITIONS FOR CODING REGISTER
 */

/** Setting of the Encoder and Decoder*/
#define Z055_CDR_NRZ 		0x0000	/**< Set Rx/Tx to NRZ coded */
#define Z055_CDR_NRZI 		0x0001	/**< Set Rx/Tx to NRZI (NRZ-M) coded */
#define Z055_CDR_MAN		0x0002	/**< Set Rx/Tx to Manchester coded */
#define Z055_CDR_NRZI_MAN	0x0003	/**< Set Rx/Tx to NRZIManch. coded */
#define Z055_CDR_NRZS		0x0004	/**< Set Rx/Tx to NRZI (NRZ-S) coded */


/*
 * MACRO DEFINITIONS FOR ADDRESS COMPARE REGISTER
 */

#define Z055_ACR_BRC_MASK		0xFF00	/**< bradcast address mask */
#define Z055_ACR_DEV_MASK 		0x00FF	/**< device address mask */


/*
 * MACRO DEFINITIONS FOR BAUDRATE GENERATOR CONTROL REGISTER
 */

#define Z055_BCR_BRGEN		0x0001	/**< Baudrate Generator enable */
										/**<0: Baudrate Generator disabled\n
										 *  1: Baudrate Generator enalbed*/
#define Z055_BCR_CLKTRANSEN	0x0002	/**< Clock transmission via interface */
										/**<0: Tx clock not transmitted via HDLC
										 *     interface\n
										 *  1: Tx clock transmitted via HDLC
										 *     interface (see BCR[2])*/
#define Z055_BCR_CLKPIN		0x0004	/**< Transmit Clock via PIN RTS/DTR */
										/**<0: Transmit clock via RTS when
										 *     enabled\n
										 *  1: Transmit clock via DTR when
										 *     enabled */

/*
 * MACRO DEFINITIONS FOR INTERRUPT ENABLE REGISTER
 */
#define Z055_IER_GIRQEN		0x8000	/**< Global interrupt enable */
										/**<0: Interrupts disabled,
										 *     no interrupt will be generated
										 *     (status can be read in IRQ)\n
										 *  1: Global interrupt enabled,
										 *     interrupts will be generated */
#define Z055_IER_HSSEN		0x4000	/**< Handshake status interrupt enable */
										/**<0: Interrupts disabled,
										 *     no interrupt will be generated
										 *     (status can be read in IRQ)\n
										 *  1: Interrupt enabled,
										 *     interrupt will be generated when
										 *     handshake input lines change */
#define Z055_IER_TXBOVREN	0x0200	/**< Transmit buffer overrun interrupt
										 *   enable */
										/**<0: Interrupt disabled\n
										 *  1: Interrupt enabled;
										 *     interrupt will be generated when
										 *     transmitter is started again
										 *     before last frame has been sent
										 *     completely */
#define Z055_IER_TXBEPYEN	0x0100	/**< Transmitter buffer empty interrupt
										 *   enable */
										/**<0: Interrupts disabled\n
										 *  1: Interrupt enabled;
										 *     interrupt will be generated when
										 *     transmitter buffer becomes empty
										 *     after sending a complete frame.*/
#define Z055_IER_RXINVEN		0x0020	/**< Receiver invalid interrupt enable*/
										/**<0: Interrupts disabled,
										 *     no invalid HDLC structures will
										 *     be displayed\n
										 *  1: Interrupt enabled;
										 *     interrupt will be generated when
										 *     HDLC structure is invalid. */
#define Z055_IER_RXABRTEN	0x0010	/**< Receiver abort detection interrupt
										 *     enable */
										/**<0: Interrupts disabled,
										 *     no receiver abort detection will
										 *     be displayed\n
										 *  1: Interrupt enabled;
										 *     interrupt will be generated when
										 *     receiver detects abort frame. */
#define Z055_IER_RXFCSEEN	0x0008	/**< Receiver frame check sequence
										 *   interrupt enable */
										/**<0: Interrupts disabled,
										 *     no CRC error will be displayed\n
										 *  1: Interrupt enabled;
										 *     interrupt will be generated
										 *     when corrupt CRC is detected. */
#define Z055_IER_RXRCSTEN	0x0004	/**< Receiver reception start interrupt
										 *   enable */
										/**<0: Interrupts disabled,
										 *     no start of reception will be
										 *     displayed\n
										 *  1: Interrupt enabled;
										 *     interrupt will be generated when
										 *     HDLC address is correct and
										 *     Controller starts with
										 *     reception. */
#define Z055_IER_RXBOVREN	0x0002	/**< Receiver buffer overrun interrupt
										 *   enable */
										/**<0: Interrupts disabled,
										 *     no receiver buffer overrun will
										 *     be displayed\n
										 *  1: Interrupt enabled;
										 *     interrupt will be generated when
										 *     following HDLC frame is received
										 *     before last frame has been read
										 *     by SW. */
#define Z055_IER_RXBFLEN		0x0001	/**< Receiver buffer full interrupt
										 *   enable */
										/**<0: Interrupts disabled,
										 *     no receiver buffer full will be
										 *     displayed\n
										 *  1: Interrupt enabled;
										 *     interrupt will be generated when
										 *     complete HDLC frame has been
										 *     received and can be read by SW.*/

/*
 * MACRO DEFINITIONS FOR INTERRUPT REQUEST REGISTER
 */
#define Z055_IRQR_HSS			0x4000	/**< Handshake status interrupt*/
										/**<0: No handshake status interrupt
										 *     pending\n
										 *  1: Handshake status interrupt
										 *     pending. Handshake input lines
										 *     have changed. See HSSR(1..0)*/
#define Z055_IRQR_TXBOVR		0x0200	/**< Transmit buffer overrun interrupt*/
										/**<0: No transmit buffer overrun
										 *     interrupt pending\n
										 *  1: Transmitter buffer overrun
										 *     interrupt pending. Transmitter
										 *     has been started again before
										 *     last frame has been sent
										 *     completely*/
#define Z055_IRQR_TXBEPY		0x0100	/**< Transmitter buffer empty interrupt*/
										/**<0: No transmit buffer empty
										 *     interrupt pending\n
										 *  1: Transmit buffer empty interrupt
										 *     pending; transmitter buffer has
										 *     becomes empty after sending a
										 *     complete frame.*/
#define Z055_IRQR_RXINV		0x0020	/**< Receiver invalid frame interrupt */
										/**<0: No receiver invalid frame
										 *     interrupt pending\n
										 *  1: Receiver invalid frame interrupt
										 *     pending; HDLC structure is
										 *     invalid.*/
#define Z055_IRQR_RXABRT		0x0010	/**< Receiver abort detected interrupt*/
										/**<0: No abort detection interrupt
										 *     pending\n
										 *  1: Abort detection interrupt pending;
										 *     receiver has detected an abort
										 *     frame*/
#define Z055_IRQR_RXFCSE		0x0008	/**< Receiver frame check sequence
										 *     interrupt */
										/**<0: No CRC interrupt pending\n
										 *  1: CRC interrupt pending;
										 *     corrupt CRC is detected.*/
#define Z055_IRQR_RXRCST		0x0004	/**< Receiver reception start interrupt */
										/**<0: No receiver reception start
										 *     interrupt pending\n
										 *  1: Receiver reception start
										 *     interrupt pending;
										 *     matching HDLC address detected,
										 *     Controller starts with reception.
										 */
#define Z055_IRQR_RXBOVR		0x0002	/**< Receiver buffer overrun interrupt*/
										/**<0: No receiver buffer overrun
										 *     interrupt pending\n
										 *  1: Receiver buffer overrun interrupt
										 *     pending; following HDLC frame
										 *     has been received before last
										 *     frame has been read by SW.*/
#define Z055_IRQR_RXBFL		0x0001	/**< Receiver buffer full interrupt */
										/**<0: No receiver buffer full interrupt
										 *     pending\n
										 *  1: Receiver buffer full interrupt
										 *     pending; complete HDLC frame has
										 *     been received and can be read by
										 *     SW.*/

/*
 * MACRO DEFINITIONS FOR HANDSHAKE CONTROL REGISTER
 */
#define Z055_HSCR_RTS	0x0001		/**< RTS control */
										/**<0: RTS is set to 0\n
										 *  1: RTS is set to 1*/
#define Z055_HSCR_DTR	0x0002		/**< DTR control */
										/**<0: DTR is set to 0\n
										 *  1: DTR is set to 1*/
#define Z055_HSCR_HDX	0x0004		/**< Half duplex */
										/**<0: Interface configured in full
										 *     duplex\n
										 *  1: Interface configured in half
										 *     duplex; HW-flow control will be
										 *     generated by controller */

/*
 * MACRO DEFINITIONS FOR HANDSHAKE STATUS REGISTER
 */
#define Z055_HSSR_CTS	0x0001		/**< CTS status */
#define Z055_HSSR_DSR	0x0002		/**< DSR status */



/*
 * MACRO definitions for HW access
 */
/* sanity check */
#ifdef MAC_MEM_MAPPED
#	ifdef MAC_IO_MAPPED
#		error "Do not define MAC_MEM_MAPPED and MAC_IO_MAPPED together"
#	endif /* MAC_IO_MAPPED */
#endif /* MAC_MEM_MAPPED */
#ifndef MAC_MEM_MAPPED
#	ifndef MAC_IO_MAPPED
#		define MAC_MEM_MAPPED	/* use MAC_MEM_MAPPED by default */
//#		error "Define MAC_MEM_MAPPED or MAC_IO_MAPPED must be defined"
#	endif /* MAC_IO_MAPPED */
#endif /* MAC_MEM_MAPPED */

/************************************
 *            PPC                   *
 ***********************************/
#ifdef _PPC_IO_H /* handle PPCs differently */

typedef unsigned long MACCESS;         /* access pointer */
typedef volatile unsigned char  __vu8;
typedef volatile unsigned short __vu16;
typedef volatile unsigned  __vu32;

#ifdef MAC_IO_MAPPED
# define _MAC_OFF_	(_IO_BASE)
#else
# define _MAC_OFF_	0
#endif


#define MREAD_D8(ma,offs)		\
            in_8((__vu8*)((MACCESS)(ma)+(offs)+_MAC_OFF_))
#define MREAD_D16(ma,offs)		\
            in_le16((__vu16*)((MACCESS)(ma)+(offs)+_MAC_OFF_))
#define MREAD_D32(ma,offs)		\
            in_le32((__vu32*)((MACCESS)(ma)+(offs)+_MAC_OFF_))

#define MWRITE_D8(ma,offs,val)	\
            out_8((__vu8*)((MACCESS)(ma)+(offs)+_MAC_OFF_),val)
#define MWRITE_D16(ma,offs,val) \
            out_le16((__vu16*)((MACCESS)(ma)+(offs)+_MAC_OFF_),val)
#define MWRITE_D32(ma,offs,val) \
            out_le32((__vu32*)((MACCESS)(ma)+(offs)+_MAC_OFF_),val)

#define MBLOCK_READ_D32(ma,offs,size,dst) \
        { int sz=size>>2;           \
          u32 *mem=(u32 *)dst; \
          unsigned long hw = (MACCESS)(ma)+(offs)+_MAC_OFF_; \
          while(sz--){ \
			  *mem++ = in_be32( (__vu32*)hw );\
              hw += 4;	\
          }             \
        }

#define MBLOCK_WRITE_D32(ma,offs,size,src) \
        { int sz=size>>2;           \
          u32 *mem=(u32 *)src; \
          unsigned long hw = (MACCESS)(ma)+(offs)+_MAC_OFF_; \
          while(sz--){ \
              out_be32((__vu32*)hw,*mem);\
              mem++; \
              hw += 4;	\
          }             \
        }
#else
/************************************
 *         not PPC                  *
 ***********************************/
#endif

#define Z055_INREG(info, reg) 		MREAD_D16(info->ma_base + info->ma_offs, reg)
#define Z055_OUTREG(info, reg, val)	MWRITE_D16(info->ma_base + info->ma_offs, reg, val)
#define Z055_INRX(info, offs, len, buf) \
				MBLOCK_READ_D32(info->ma_base + info->ma_offs + Z055_RXBUF, \
								offs, len, buf)
#define Z055_OUTTX(info, offs, len, buf) \
				MBLOCK_WRITE_D32(info->ma_base + info->ma_offs + Z055_TXBUF, \
								offs, len, buf)



#endif  /* _Z055_HDLC_INT_H_ */

