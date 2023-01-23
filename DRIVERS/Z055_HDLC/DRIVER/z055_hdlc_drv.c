/*********************  P r o g r a m  -  M o d u l e ***********************/
/*!
 *        \file  z055_hdlc_drv.c
 *      Project: Z055_HDLC Linux native driver with IP/PPP stack integration
 *
 *      \author: christian.schuster@men.de
 *
 *       \brief  Linux native device driver for MEN Z055_HDLC FPGA core
 *
 *               Derived from synclink.c written by Paul Fulghum
*
 *               This driver is intended for use in synchronous HDLC mode.
 *
 *               When operating in synchronous mode, each call to z055_write()
 *               contains exactly one complete HDLC frame.
 *
 *               Synchronous receive data is reported as complete frames.
 *               To accomplish this, the TTY flip buffer is bypassed (too
 *               small to hold largest frame and may fragment frames) and
 *               the line discipline receive entry point is called directly.
 *
 *     Required:  -
 *     Switches:  -
 *
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

#define VERSION(ver,rel,seq) (((ver)<<16) | ((rel)<<8) | (seq))
#if defined(__i386__)
#  define BREAKPOINT() asm("   int $3");
#else
#  define BREAKPOINT() { }
#endif

#define Z055_MAX_DEVICES    10


#include <linux/version.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/ptrace.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/slab.h>

#include <linux/netdevice.h>

#include <linux/vmalloc.h>
#include <linux/init.h>
#include <asm/serial.h>

#include <linux/delay.h>
#include <linux/ioctl.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/bitops.h>
#include <asm/types.h>
#include <linux/termios.h>
#include <linux/workqueue.h>

#include <MEN/men_chameleon.h>
#include "z055_hdlc_int.h"

#define GET_USER(error,value,addr) error = get_user(value,addr)
#define COPY_FROM_USER(error,dest,src,size) error = copy_from_user(dest,src,size) ? -EFAULT : 0
#define PUT_USER(error,value,addr) error = put_user(value,addr)
#define COPY_TO_USER(error,dest,src,size) error = copy_to_user(dest,src,size) ? -EFAULT : 0

#if LINUX_VERSION_CODE < VERSION(4,10,0)
#include <asm/uaccess.h>
#else
#include <linux/uaccess.h>
#endif

#include <generated/autoconf.h>

#define RCLRVALUE 0xffff

Z055_PARAMS G_default_params    = {
	Z055_MODE_HDLC,                 /* unsigned long mode */
	HDLC_FLAG_TXIDLE_FLAGS,         /* unsigned long flags; */
	HDLC_ENCODING_MANCHESTER_NRZI,  /* unsigned char encoding; */
	256000,                         /* unsigned long baud_rate */
	0xff,                           /* unsigned char comp_addr */
	0xff,                           /* unsigned char comp_broadc */
	0xff,                           /* unsigned char addr_mask */
	0xff,                           /* unsigned char broadc_mask */
	HDLC_CRC_CCITT,                 /* unsigned char crc_mode */
	HDLC_PREAMBLE_PATTERN_NONE,     /* unsigned char preamble */
	HDLC_FULL_DUPLEX                /* unsigned char half_duplex */
};

void z055_hw_DisableMasterIrqBit( struct Z055_STRUCT *info );
void z055_hw_EnableMasterIrqBit( struct Z055_STRUCT *info    );
void z055_hw_EnableInterrupts( struct Z055_STRUCT *info, u16 IrqMask );
void z055_hw_DisableInterrupts( struct Z055_STRUCT *info, u16 IrqMask );
void z055_hw_ClearIrqPendingBits( struct Z055_STRUCT *info, u16 IrqMask );

#define z055_hw_EnableInterrupts( a, b ) \
	Z055_OUTREG( (a), Z055_IER, \
				 (Z055_INREG( (a), Z055_IER ) | (b) | Z055_IER_GIRQEN ) )

#define z055_hw_DisableInterrupts( a, b ) \
	Z055_OUTREG( (a), Z055_IER, \
				(Z055_INREG( (a), Z055_IER) & ~(b)) )

#define z055_hw_EnableMasterIrqBit(a) \
	Z055_OUTREG( (a), Z055_IER, (Z055_INREG( (a), Z055_IER ) | Z055_IER_GIRQEN) )

#define z055_hw_DisableMasterIrqBit(a) \
	Z055_OUTREG( (a), Z055_IER, (Z055_INREG( (a), Z055_IER ) & ~Z055_IER_GIRQEN))

#define z055_hw_ClearIrqPendingBits( a, b ) \
	Z055_OUTREG( (a), Z055_IRQR, (b) )

/* Transmit status Bits in Transmit Control status Register (TCSR) */
/* and Transmit Interrupt Control Register (TICR) (except BIT2, BIT0) */


#define DISABLE_UNCONDITIONAL    0
#define ENABLE_UNCONDITIONAL     1
#define z055_hw_EnableTransmitter(a,b) \
	Z055_OUTREG( (a), Z055_HCR, ((Z055_INREG((a), Z055_HCR) & 0xffef) | (b) << 4) )
#define z055_hw_EnableReceiver(a,b) \
	Z055_OUTREG( (a), Z055_HCR, ((Z055_INREG((a), Z055_HCR) & 0xfffe) | (b)) )

static void z055_hw_start_receiver( struct Z055_STRUCT *info );
static void z055_hw_stop_receiver( struct Z055_STRUCT *info );

static void z055_hw_start_transmitter( struct Z055_STRUCT *info );
static void z055_hw_stop_transmitter( struct Z055_STRUCT *info );
static void z055_hw_set_txidle( struct Z055_STRUCT *info );
void z055_hw_load_txfifo( struct Z055_STRUCT *info );

static void z055_hw_enable_brgen( struct Z055_STRUCT *info, u32 DataRate   );
static void z055_hw_enable_loopback( struct Z055_STRUCT *info, int enable );

static void z055_hw_get_serial_signals( struct Z055_STRUCT *info );
static void z055_hw_set_serial_signals( struct Z055_STRUCT *info );

static void z055_hw_reset( struct Z055_STRUCT *info );

static void z055_hw_set_sync_mode( struct Z055_STRUCT *info );
static void z055_hw_set_sdlc_mode( struct Z055_STRUCT *info );

void z055_hw_loopback_frame( struct Z055_STRUCT *info );

#if LINUX_VERSION_CODE < VERSION(4,15,0)
static void z055_tx_timeout(unsigned long context);
#else
static void z055_tx_timeout(struct timer_list *t);
#endif

static int z055_ioctl_common(struct Z055_STRUCT *info, unsigned int cmd, unsigned long arg);
static int z055_tty_install(struct tty_driver *driver, struct tty_struct *tty);

static unsigned int z055_hw_get_modem_info(struct Z055_STRUCT * info);

/*
 * Defines a BUS descriptor value for the PCI adapter
 * local bus address ranges.
 */

#define BUS_DESCRIPTOR( WrHold, WrDly, RdDly, Nwdd, Nwad, Nxda, Nrdd, Nrad ) \
(0x00400020 + \
((WrHold) << 30) + \
((WrDly)  << 28) + \
((RdDly)  << 26) + \
((Nwdd)   << 20) + \
((Nwad)   << 15) + \
((Nxda)   << 13) + \
((Nrdd)   << 11) + \
((Nrad)   <<  6) )

static void z055_trace_block(struct Z055_STRUCT *info,const char* data, int count, int xmit);

/*
 * Adapter diagnostic routines
 */
static BOOLEAN z055_register_test( struct Z055_STRUCT *info );
static int z055_adapter_test( struct Z055_STRUCT *info );

/*
 * device and resource management routines
 */
static int z055_claim_resources(struct Z055_STRUCT *info);
static void z055_release_resources(struct Z055_STRUCT *info);
static void z055_add_device(struct Z055_STRUCT *info);
struct Z055_STRUCT* z055_allocate_device(void);

/*
 * buffer manipulation functions.
 */
static void z055_reset_rx_buffers( struct Z055_STRUCT *info );
static void z055_load_tx_buffer( struct Z055_STRUCT *info, const char *Buffer, unsigned int BufferSize);

static int z055_alloc_rtxbuffer_memory( struct Z055_STRUCT *info,
								 struct RXTX_BUFFER_QUEUE_S *queue);
static void z055_free_rtxbuffer_memory( struct Z055_STRUCT *info,
								 struct RXTX_BUFFER_QUEUE_S *queue);

/*
 * Bottom half interrupt handlers
 */
static void z055_bh_handler(struct work_struct *work);
static void z055_bh_receive(struct Z055_STRUCT *info);
static void z055_bh_transmit(struct Z055_STRUCT *info);
static void z055_bh_status(struct Z055_STRUCT *info);

/*
 * Interrupt handler routines and dispatch table.
 */
static void z055_isr_transmit( struct Z055_STRUCT *info );
static void z055_isr_receive_data( struct Z055_STRUCT *info );
static void z055_isr_receive_status( struct Z055_STRUCT *info );
static void z055_isr_misc( struct Z055_STRUCT *info );

/*
 * ioctl call handlers
 */
static int tiocmget(struct tty_struct *tty);
static int tiocmset(struct tty_struct *tty, unsigned int set, unsigned int clear);
static int z055_get_stats(struct Z055_STRUCT *info,
						  struct Z055_ICOUNT *user_icount);
static int z055_get_params(struct Z055_STRUCT *info,
						   Z055_PARAMS *user_params);
static int z055_set_params(struct Z055_STRUCT   * info,
						   Z055_PARAMS *new_params);
static int z055_get_txidle(struct Z055_STRUCT *info, int*idle_mode);
static int z055_set_txidle(struct Z055_STRUCT *info, int idle_mode);
static int z055_txenable(struct Z055_STRUCT *info, int enable);
static int z055_txabort(struct Z055_STRUCT *info);
static int z055_rxenable(struct Z055_STRUCT *info, int enable);
static int z055_wait_event(struct Z055_STRUCT *info, int *mask);

#define jiffies_from_ms(a) ((((a) * HZ)/1000)+1)

/*
 * Global linked list of Z55 devices
 */
struct Z055_STRUCT *G_z055_device_list;
static int G_z055_device_count;

/*
 * Set this param to non-zero to load eax with the
 * .text section address and breakpoint on module load.
 * This is useful for use with gdb and add-symbol-file command.
 */
static int break_on_load;

/*
 * Driver major number, defaults to zero to get auto
 * assigned major number. May be forced as module parameter.
 */
static int ttymajor;

/*
 * Array of user specified options for ISA adapters.
 */
static int debug_level;
static int maxframe[Z055_MAX_DEVICES];
static int num_rxbufs[Z055_MAX_DEVICES];

static const char IdentString[]=MENT_XSTR(MAK_REVISION);

module_param(break_on_load,int,0);
module_param(ttymajor,int,0);
module_param(debug_level,int,0);
module_param_array(maxframe, int, NULL, 0);
module_param_array(num_rxbufs, int, NULL, 0);

MODULE_PARM_DESC(break_on_load, "flag, set breakpoint on module load");
MODULE_PARM_DESC(ttymajor,  "tty driver major number");
MODULE_PARM_DESC(debug_level,   "drivers debug level");
MODULE_PARM_DESC(maxframe,  "maximum framesize to allocate buffer for");
MODULE_PARM_DESC(num_rxbufs, "number of Rx Buffers to   allocate");

static char *G_driver_name = "Z055 HDLC driver";


static int z055_init_one    (CHAMELEON_UNIT_T   *chu);
static int z055_remove_one  (CHAMELEON_UNIT_T   *chu);

MODULE_LICENSE("GPL");
MODULE_VERSION(MENT_XSTR(MAK_REVISION));

static u16 G_modCodeArr[] = { CHAMELEON_16Z055_HDLC, CHAMELEON_MODCODE_END };

static CHAMELEON_DRIVER_T G_driver = {
	.name       =       "z055_hdlc",
	.modCodeArr =       G_modCodeArr,
	.probe      =       z055_init_one,
	.remove     =       z055_remove_one
};

static struct tty_driver *G_serial_driver;

/* number of characters left in xmit buffer before we ask for more */
#define WAKEUP_CHARS 256


static void z055_change_params(struct Z055_STRUCT   *info);
static void z055_wait_until_sent(struct tty_struct *tty,    int timeout);


#ifndef MIN
#define MIN(a,b)    ((a) < (b) ? (a) : (b))
#endif

/*
 * 1st function defined in .text section. Calling this function in
 * init_module() followed by a breakpoint allows a remote debugger
 * (gdb) to get the .text address for the add-symbol-file command.
 * This allows remote debugging of dynamically loadable modules.
 */
static void* z055_get_text_ptr(void);
static void* z055_get_text_ptr() {return z055_get_text_ptr;}

/*
 * G_tmp_buf is used as a temporary buffer by z055_write. We need to
 * lock it in case the COPY_FROM_USER blocks while swapping in a page,
 * and some other program tries to do a serial write at the same time.
 * Since the lock will only come under contention when the system is
 * swapping and available memory is low, it makes sense to share one
 * buffer across all the serial ioports, since it significantly saves
 * memory if large numbers of serial ports are open.
 */
static unsigned char *G_tmp_buf;

static inline int z055_paranoia_check(struct Z055_STRUCT *info,
				      char *name,
				      const char *routine)
{
#ifdef Z055_PARANOIA_CHECK
	static const char *badmagic =
		"Warning: bad magic number for Z055_STRUCT (%s) in %s\n";
	static const char *badinfo =
		"Warning: null Z055_STRUCT for (%s) in %s\n";

	if (!info) {
		printk(badinfo, name, routine);
		return 1;
	}Z055_MAGIC) {
		printk(badmagic, name, routine);
		return 1;
	}
#endif
	return 0;
}

/* z055_stop()      throttle (stop) transmitter
 *
 * Arguments:       tty pointer to tty info structure
 * Return Value:    None
 */
static void z055_stop(struct    tty_struct *tty)
{
	struct Z055_STRUCT *info    = (struct Z055_STRUCT *)tty->driver_data;
	unsigned long flags;

	if (z055_paranoia_check(info, tty->name,    "z055_stop"))
		return;

	if ( debug_level & DEBUG_LEVEL_INFO )
		printk("%s(%d): %s\n", __FUNCTION__, __LINE__, info->device_name);

	spin_lock_irqsave(&info->irq_spinlock,flags);
	if (info->tx_enabled)
		z055_hw_stop_transmitter(info);
	spin_unlock_irqrestore(&info->irq_spinlock,flags);

}   /* end of z055_stop() */

/* z055_start()     release (start) transmitter
 *
 * Arguments:       tty pointer to tty info structure
 * Return Value:    None
 */
static void z055_start(struct tty_struct    *tty)
{
	struct Z055_STRUCT *info    = (struct Z055_STRUCT *)tty->driver_data;
	unsigned long flags;

	if (z055_paranoia_check(info, tty->name,    "z055_start"))
		return;

	if ( debug_level & DEBUG_LEVEL_INFO )
		printk("z055_start(%s)\n",info->device_name);

	spin_lock_irqsave(&info->irq_spinlock,flags);
	if (!info->tx_enabled)
		z055_hw_start_transmitter(info);
	spin_unlock_irqrestore(&info->irq_spinlock,flags);

}   /* end of z055_start() */

/*
 * Bottom half work queue access functions
 */

/* z055_bh_action() Return next bottom half action to perform.
 * Return Value:    BH action code or 0 if nothing to do.
 */
int z055_bh_action(struct Z055_STRUCT   *info)
{
	unsigned long flags;
	int rc = 0;

	spin_lock_irqsave(&info->irq_spinlock,flags);

	if (info->pending_bh & BH_RECEIVE) {
		info->pending_bh &= ~BH_RECEIVE;
		rc = BH_RECEIVE;
	} else if (info->pending_bh & BH_TRANSMIT) {
		info->pending_bh &= ~BH_TRANSMIT;
		rc = BH_TRANSMIT;
	} else if (info->pending_bh & BH_STATUS) {
		info->pending_bh &= ~BH_STATUS;
		rc = BH_STATUS;
	}

	if (!rc) {
		/* Mark BH routine as complete */
		info->bh_running   = 0;
		info->bh_requested = 0;
	}

	spin_unlock_irqrestore(&info->irq_spinlock,flags);

	return rc;
}

/*
 *  Perform bottom half processing of work items queued by ISR.
 */
static void z055_bh_handler(struct work_struct *work)
{
	struct Z055_STRUCT *info = container_of(work, struct Z055_STRUCT, task);
	int action;

	if (!info)
		return;

	if ( debug_level & DEBUG_LEVEL_BH )
		printk( "%s(%d): %s entry\n", __FUNCTION__, __LINE__, info->device_name);

	info->bh_running = 1;

	while((action = z055_bh_action(info)) !=    0) {

		/* Process work item */
		if ( debug_level & DEBUG_LEVEL_BH )
			printk( "%s(%d): work item action=%d\n",
					__FUNCTION__, __LINE__, action);

		switch (action) {

		case BH_RECEIVE:
			z055_bh_receive(info);
			break;
		case BH_TRANSMIT:
			z055_bh_transmit(info);
			break;
		case BH_STATUS:
			z055_bh_status(info);
			break;
		default:
			/* unknown work item ID */
			printk("Unknown work item ID=%08X!\n", action);
			break;
		}
	}

	if ( debug_level & DEBUG_LEVEL_BH )
		printk( "%s(%d): %s exit\n", __FUNCTION__, __LINE__, info->device_name);
}

static void z055_bh_receive(struct Z055_STRUCT *info)
{
	struct tty_struct *tty = Z055_STRUCT_get_tty(info);
	unsigned long flags;
	struct tty_ldisc *ld;

	if ( debug_level & DEBUG_LEVEL_BH )
		printk( "%s(%d): %s\n", __FUNCTION__, __LINE__, info->device_name);

	/* pass frame(s) */
	while( info->rx_buffer_q.buffers_used ) {
		if ( debug_level & DEBUG_LEVEL_DATA )
			z055_trace_block( info, info->rx_buffer_q.put_buffer->buffer,
								MIN(info->rx_buffer_q.put_buffer->ccount, PAGE_SIZE),0);

		if (tty) {
			ld = tty_ldisc_ref(tty);
			if (ld) {
				if (ld->ops->receive_buf) {
					/* Call the line discipline receive callback directly. */
					struct RXTX_BUFFER_S *put_buf = info->rx_buffer_q.put_buffer;
					if ( debug_level & DEBUG_LEVEL_BH )
						printk( "%s(%d): %s frame put from buffer addr:0x%08x;"
							" count:%d; used_bufs:0x%d\n",
							__FUNCTION__, __LINE__, info->device_name,
							put_buf->buffer, put_buf->ccount,
							info->rx_buffer_q.buffers_used);
					ld->ops->receive_buf( tty,
							      info->rx_buffer_q.put_buffer->buffer,
							      NULL,
							      info->rx_buffer_q.put_buffer->ccount );
				}
				tty_ldisc_deref(ld);
			}
		}
		spin_lock_irqsave(&info->irq_spinlock,flags);
		info->rx_buffer_q.put_buffer = info->rx_buffer_q.put_buffer->next;
		info->rx_buffer_q.buffers_used--;
		spin_unlock_irqrestore(&info->irq_spinlock,flags);
	}
}

static void z055_bh_transmit(struct Z055_STRUCT *info)
{
	struct tty_struct *tty = Z055_STRUCT_get_tty(info);

	if ( debug_level & DEBUG_LEVEL_BH )
		printk( "%s(%d): entry on %s\n",
			__FUNCTION__, __LINE__, info->device_name);

	if (tty) {
		if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
			tty->ldisc->ops->write_wakeup) {
			if ( debug_level & DEBUG_LEVEL_BH )
				printk( "%s(%d):calling ldisc.write_wakeup on %s\n",
					__FUNCTION__, __LINE__, info->device_name);
			(tty->ldisc->ops->write_wakeup)(tty);
		}
		wake_up_interruptible(&tty->write_wait);
	}
}

static void z055_bh_status(struct Z055_STRUCT *info)
{
	if ( debug_level & DEBUG_LEVEL_BH )
		printk( "%s(%d): entry on %s\n",
				__FUNCTION__, __LINE__, info->device_name);

	info->dsr_chkcount = 0;
	info->cts_chkcount = 0;
}

/* z055_isr_receive_status()
 *
 *  Service a receive status interrupt. The type of status
 *  interrupt is indicated by the state of the RCSR.
 *  This is only used for HDLC mode.
 *
 * Arguments:       info    pointer to device instance data
 * Return Value:    None
 */
static void z055_isr_receive_status( struct    Z055_STRUCT *info )
{
	u16 status = Z055_INREG(    info, Z055_IRQR );

	if ( debug_level & DEBUG_LEVEL_ISR )
		printk( "%s(%d): status=%04X\n",
				__FUNCTION__, __LINE__, status);

	if ((status & Z055_IRQR_RXINV)  |
		(status & Z055_IRQR_RXABRT) |
		(status & Z055_IRQR_RXBOVR) ) {
		if (status & Z055_IRQR_RXINV)
			info->icount.rxinv++;
		if (status & Z055_IRQR_RXABRT)
			info->icount.rxabort++;
		if (status & Z055_IRQR_RXBOVR)
			info->icount.rxbover++;
		wake_up_interruptible(&info->event_wait_q);
	}

	z055_hw_ClearIrqPendingBits( info,  Z055_IRQR_RXINV +
										Z055_IRQR_RXABRT +
										Z055_IRQR_RXBOVR );
}   /* end of z055_isr_receive_status() */

/* z055_isr_transmit()
 *
 *  Service a transmit status interrupt
 *  HDLC mode :end of transmit frame and Tx Buffer overrun
 *  Async mode:all data is sent
 *  transmit status is indicated by bits in the TCSR.
 *
 * Arguments:       info           pointer to device instance data
 * Return Value:    None
 */
static void z055_isr_transmit( struct Z055_STRUCT *info )
{
	struct tty_struct *tty = Z055_STRUCT_get_tty(info);
	u16 status = Z055_INREG( info, Z055_IRQR );

	if ( debug_level & DEBUG_LEVEL_ISR )
		printk( "%s(%d): status=%04X\n",
				__FUNCTION__, __LINE__, status);

	z055_hw_ClearIrqPendingBits( info, Z055_IRQR_TXBOVR + Z055_IRQR_TXBEPY );

	if ( status & Z055_IRQR_TXBEPY )
		info->icount.txok++;
	else if ( status & Z055_IRQR_TXBOVR )
		info->icount.txbovr++;
	else
		info->icount.txbovr++;

	info->tx_active = 0;
	info->xmit_cnt = info->xmit_head = info->xmit_tail = 0;
	del_timer(&info->tx_timer);

	if ( info->drop_rts_on_tx_done ) {
		z055_hw_get_serial_signals( info );
		if ( info->serial_signals & SerialSignal_RTS ) {
			info->serial_signals &= ~SerialSignal_RTS;
			z055_hw_set_serial_signals( info );
		}
		info->drop_rts_on_tx_done = 0;
	}
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,14,0)
	if (tty && (tty->stopped || tty->hw_stopped)) {
#else
	if (tty && (tty->flow.stopped || tty->hw_stopped)) {
#endif
		z055_hw_stop_transmitter(info);
		return;
	}
	info->pending_bh |= BH_TRANSMIT;

}   /* end of z055_isr_transmit()    */


/* z055_isr_receive_data()
 *
 *  Service a receive data interrupt. This occurs
 *  when operating in asynchronous interrupt transfer mode.
 *  The receive data FIFO is flushed to the receive data buffers.
 *
 * Arguments:       info   pointer to device instance data
 * Return Value:    None
 */
static void z055_isr_receive_data( struct Z055_STRUCT *info )
{
	int cpCount, rxCount = Z055_INREG( info, Z055_RXSZR );
	u16 status   = Z055_INREG( info, Z055_IRQR ),
		z055_ier = Z055_INREG( info, Z055_IER );

	if ( debug_level & DEBUG_LEVEL_ISR )
		printk( "%s(%d):\n", __FUNCTION__, __LINE__);

	/* handle RXRCST interrupt ? */
	if( !(status & Z055_IRQR_RXBFL) && (status & Z055_IRQR_RXRCST)  ) {
		if( (z055_ier & Z055_IER_RXRCSTEN) ) {
			info->icount.rxrcst++;
			z055_hw_ClearIrqPendingBits( info,  Z055_IRQR_RXRCST );
			return;
		} else {
			return;
		}
	}

//	rxCount -= (info->params.crc_mode == HDLC_CRC_CCITT ) ? 2 : 2;
	rxCount -= 2;

	/* get Rx byte count */
	cpCount = (rxCount & 0x3) ? (rxCount & ~0x3 ) + 4 : rxCount;

	if( cpCount > info->max_frame_size ) {
		info->icount.rxlong++;
		printk( KERN_ERR "%s(%d): %s frame too big, frame discarded\n"
				"        char_count: 0x%04x; max_frame_size: 0x%04x\n",
				__FUNCTION__, __LINE__, info->device_name,
				cpCount, info->max_frame_size);
		goto isr_end;
	}

	/* check status of receive frame */
	if ( status & Z055_IRQR_RXFCSE ) {
		info->icount.rxcrc++;
		cpCount = 0;
	} else
		info->icount.rxok++;

	/* read the received frame */
	if ( debug_level & DEBUG_LEVEL_ISR )
		printk("%s(%d): %s status=%04X rxCount=%d cpCount=%d\n",
			__FUNCTION__, __LINE__, info->device_name,status,rxCount,cpCount);

	if ( cpCount ) {
		struct RXTX_BUFFER_S *get_buf = info->rx_buffer_q.get_buffer;
		/* copy adapters rx buffer to rx buffer queue */
		if ( debug_level & DEBUG_LEVEL_ISR )
			printk("%s(%d): %s receive frame: o.k.\n",
				__FUNCTION__, __LINE__, info->device_name);

		if( info->rx_buffer_q.buffers_used >= info->rx_buffer_q.num_buffers  ) {
			printk( KERN_ERR "%s(%d): %s Rx queue full, frame discarded\n",
					__FUNCTION__, __LINE__, info->device_name);
			goto isr_end;
		}

		/* get the frame from the adapters rx buffer */
		/* always copy full 32bit words. */

		Z055_INRX(info, 0, cpCount, get_buf->buffer);

		get_buf->ccount = rxCount;
		info->rx_buffer_q.get_buffer = get_buf->next;
		info->rx_buffer_q.buffers_used++;

		if ( debug_level & DEBUG_LEVEL_ISR )
			printk( "%s(%d): %s put to buf addr:0x%08x; count:%d used_bufs:%d\n",
					__FUNCTION__, __LINE__, info->device_name,
					get_buf->buffer, get_buf->ccount,
					info->rx_buffer_q.buffers_used);

		info->pending_bh |= BH_RECEIVE;
	} else {
		struct RXTX_BUFFER_S *get_buf = info->rx_buffer_q.get_buffer;

		get_buf->ccount = 0;
		info->rx_buffer_q.get_buffer = get_buf->next;
		info->rx_buffer_q.buffers_used++;

		if ( debug_level & DEBUG_LEVEL_ISR )
			printk( "%s(%d): %s empty frame put to buf addr:0x%08x;"
					" used_bufs:%d\n",
					__FUNCTION__, __LINE__, info->device_name,
					get_buf->buffer, info->rx_buffer_q.buffers_used);

		info->pending_bh |= BH_RECEIVE;
	}

isr_end:
	z055_hw_ClearIrqPendingBits( info,  Z055_IRQR_RXBFL +
										Z055_IRQR_RXRCST +
										Z055_IRQR_RXFCSE);

} /* end of z055_isr_receive_data() */

/* z055_isr_misc()
 *
 *  Service a miscellaneos interrupt source. (Handshake, ...)
 *
 * Arguments:       info        pointer to device extension (instance data)
 * Return Value:    None
 */
static void z055_isr_misc( struct Z055_STRUCT *info )
{
	u16 status = Z055_INREG(info, Z055_IRQR);

	if ( debug_level & DEBUG_LEVEL_ISR )
		printk( "%s(%d): status=%04X\n",
				__FUNCTION__, __LINE__, status);

	if ((status & Z055_IRQR_HSS)) {
		u16 hss = Z055_INREG(info, Z055_HSSR);
		if( hss & Z055_HSSR_CTS ) {
			info->input_signal_events.cts |= Z055EVENT_CTS;
			info->icount.cts++;		/* actually only when latched !!! tbd !!! */
		}
		if( hss & Z055_HSSR_DSR ) {
			info->input_signal_events.dsr |= Z055EVENT_DSR;
			info->icount.dsr++;		/* actually only when latched !!! tbd !!! */
		}
	}

	z055_hw_ClearIrqPendingBits( info, Z055_IRQR_HSS );

}   /* end of z055_isr_misc() */


/* z055_interrupt()
 *
 *  Interrupt service routine entry point.
 *
 * Arguments:
 *
 *  irq         interrupt number that caused interrupt
 *  dev_id      device ID supplied during interrupt registration
 *  regs        interrupted processor context
 *
 * Return Value: None
 */
static irqreturn_t z055_interrupt(int dummy, void *dev_id)
{

	struct Z055_STRUCT *    info;
	u16 z055_irqr, z055_ier;
	info = (struct Z055_STRUCT *)dev_id;
	if (!info)
		return IRQ_NONE;

	spin_lock(&info->irq_spinlock);

	z055_ier = Z055_INREG(info, Z055_IER);

	for(;;) {
		/* Read the interrupt vectors from hardware. */
		z055_irqr = Z055_INREG(info, Z055_IRQR);

		if ( debug_level & DEBUG_LEVEL_ISR )
			printk( "%s(%d): %s IER=0x%04X    IRQR=0x%04X\n",
					__FUNCTION__, __LINE__, info->device_name,
					z055_ier, z055_irqr);

		if ( !z055_irqr )
				break;

		/* Dispatch interrupt vector */
		if( ((z055_irqr & Z055_IRQR_RXBFL) && (z055_ier & Z055_IER_RXBFLEN)) ||
			((z055_irqr & Z055_IRQR_RXFCSE) && (z055_ier & Z055_IER_RXFCSEEN))||
			((z055_irqr & Z055_IRQR_RXRCST) && (z055_ier & Z055_IER_RXRCSTEN)) )
			 z055_isr_receive_data(info);
		else if( ((z055_irqr & Z055_IRQR_RXINV)  && (z055_ier & Z055_IER_RXINVEN)) ||
			 	 ((z055_irqr & Z055_IRQR_RXABRT) && (z055_ier & Z055_IER_RXABRTEN))||
			 	 ((z055_irqr & Z055_IRQR_RXBOVR) && (z055_ier & Z055_IER_RXBOVREN)) )
			z055_isr_receive_status(info);
		else if( ((z055_irqr & Z055_IRQR_TXBOVR) && (z055_ier & Z055_IER_TXBOVREN))||
			 	 ((z055_irqr & Z055_IRQR_TXBEPY) && (z055_ier & Z055_IER_TXBEPYEN)) )
			z055_isr_transmit(info);
		else if( ((z055_irqr & Z055_IRQR_HSS) && (z055_ier & Z055_IER_HSSEN)) )
			z055_isr_misc(info);
		else
			break; /* no enabled interrupt is triggered */

	}

	/* Request bottom half processing if there's something
	 * for it to do and the bh is not already running
	 */
	if ( info->pending_bh && !info->bh_running && !info->bh_requested ) {
		if ( debug_level & DEBUG_LEVEL_ISR )
			printk( "%s(%d): %s queueing bh task.\n",
					__FUNCTION__, __LINE__, info->device_name);
		schedule_work(&info->task);
		info->bh_requested = 1;
	}

	spin_unlock(&info->irq_spinlock);

	return IRQ_HANDLED;
}   /* end of z055_interrupt() */



/* startup()
 *
 *  Initialize and start device.
 *
 * Arguments:       info    pointer to device instance data
 * Return Value:    0 if success, otherwise error code
 */
static int startup(struct Z055_STRUCT * info)
{
	struct tty_struct *tty = Z055_STRUCT_get_tty(info);
	int retval = 0;

	if ( debug_level & DEBUG_LEVEL_INFO )
		printk( "%s(%d): %s\n",
				__FUNCTION__, __LINE__, info->device_name);

	if (tty_port_initialized(&info->port))
		return 0;

	if (!info->xmit_buf) {
		/* allocate a page of memory for a transmit buffer */
		info->xmit_buf = (unsigned char *)get_zeroed_page(GFP_KERNEL);
		if (!info->xmit_buf) {
			printk( KERN_ERR "%s(%d):%s can't allocate transmit buffer\n",
					__FUNCTION__, __LINE__, info->device_name);
			return -ENOMEM;
		}
	}

	info->pending_bh = 0;

	memset(&info->icount, 0, sizeof(info->icount));

#if LINUX_VERSION_CODE < VERSION(4,15,0)
	init_timer(&info->tx_timer);
	info->tx_timer.data = (unsigned long)info;
	info->tx_timer.function = z055_tx_timeout;
#else
	timer_setup(&info->tx_timer, z055_tx_timeout, 0);
#endif


	/* Allocate and claim adapter resources */
	retval = z055_claim_resources(info);

	/* perform existence check and diagnostics */
	if ( !retval )
		retval = z055_adapter_test(info);

	if ( retval ) {
		if (capable(CAP_SYS_ADMIN) && tty)
			set_bit(TTY_IO_ERROR, &tty->flags);
		z055_release_resources(info);
		return retval;
	}

	/* program hardware for current parameters */
	z055_change_params(info);

	if (tty)
		clear_bit(TTY_IO_ERROR, &tty->flags);

	tty_port_set_initialized(&info->port, 1);

	return 0;

}   /* end of startup() */

/* shutdown()
 *
 * Called by z055_close() and z055_hangup() to shutdown hardware
 *
 * Arguments:       info    pointer to device instance data
 * Return Value:    None
 */
static void shutdown(struct Z055_STRUCT * info)
{
	unsigned long flags;
	struct tty_struct *tty = Z055_STRUCT_get_tty(info);

	if (!tty_port_initialized(&info->port))
		return;

	if (debug_level & DEBUG_LEVEL_INFO)
		printk( "%s(%d): %s\n", __FUNCTION__, __LINE__, info->device_name );

	/* clear status wait queue because status changes */
	/* can't happen after shutting down the hardware */
	wake_up_interruptible(&info->status_event_wait_q);
	wake_up_interruptible(&info->event_wait_q);

	del_timer(&info->tx_timer);

	if (info->xmit_buf) {
		free_page((unsigned long) info->xmit_buf);
		info->xmit_buf = 0;
	}

	spin_lock_irqsave(&info->irq_spinlock,flags);
	z055_hw_DisableMasterIrqBit(info);
	z055_hw_stop_receiver(info);
	z055_hw_stop_transmitter(info);
	z055_hw_DisableInterrupts(info, Z055_IER_RXINVEN   +
									Z055_IER_RXABRTEN +
									Z055_IER_RXFCSEEN +
									Z055_IER_RXRCSTEN +
									Z055_IER_RXBOVREN +
									Z055_IER_RXBFLEN +
									Z055_IER_TXBOVREN +
									Z055_IER_TXBEPYEN  );

	if (!tty || tty_cflags(tty) & HUPCL) {
		info->serial_signals &= ~(SerialSignal_DTR + SerialSignal_RTS);
		z055_hw_set_serial_signals(info);
	}

	spin_unlock_irqrestore(&info->irq_spinlock,flags);

	z055_release_resources(info);

	if (tty)
		set_bit(TTY_IO_ERROR, &tty->flags);

	tty_port_set_initialized(&info->port, 0);

}   /* end of shutdown() */

static void z055_program_hw(struct Z055_STRUCT *info)
{
	unsigned long flags;
	struct tty_struct *tty = Z055_STRUCT_get_tty(info);
	if (debug_level & DEBUG_LEVEL_INFO)
		printk( "%s(%d):%s\n", __FUNCTION__, __LINE__, info->device_name );

	spin_lock_irqsave(&info->irq_spinlock,flags);

	z055_hw_stop_receiver(info);
	z055_hw_stop_transmitter(info);
	info->xmit_cnt = info->xmit_head = info->xmit_tail = 0;

	if ( info->params.mode == Z055_MODE_HDLC )
		z055_hw_set_sync_mode(info);

	z055_hw_set_serial_signals(info);

	info->cts_chkcount = 0;
	info->dsr_chkcount = 0;

	z055_hw_EnableInterrupts(info, Z055_IER_HSSEN );

	z055_hw_get_serial_signals(info);

	if ( tty && (tty_cflags(tty) & CREAD) )
		z055_hw_start_receiver(info);

	spin_unlock_irqrestore(&info->irq_spinlock,flags);
}

/* Reconfigure adapter based on new parameters
 */
static void z055_change_params(struct Z055_STRUCT *info)
{
	unsigned cflag;
	struct tty_struct *tty = Z055_STRUCT_get_tty(info);

	if (!tty)
		return;

	if (debug_level & DEBUG_LEVEL_INFO)
		printk( "%s(%d):%s\n", __FUNCTION__, __LINE__, info->device_name );

	cflag = tty_cflags(tty);

	/* if B0 rate (hangup) specified then negate DTR and RTS */
	/* otherwise assert DTR and RTS */
	if (cflag & CBAUD)
		info->serial_signals |= SerialSignal_RTS + SerialSignal_DTR;
	else
		info->serial_signals &= ~(SerialSignal_RTS + SerialSignal_DTR);


	/* process tty input control flags */

	info->read_status_mask = Z055_IRQR_RXBOVR;
	if (I_INPCK(tty))
		info->read_status_mask |= Z055_IRQR_RXINV |
									Z055_IRQR_RXFCSE;

	if (I_BRKINT(tty) || I_PARMRK(tty))
		info->read_status_mask |= Z055_IRQR_RXABRT;

	if (I_IGNPAR(tty))
		info->ignore_status_mask |= Z055_IRQR_RXINV |
									Z055_IRQR_RXFCSE;

	if (I_IGNBRK(tty)) {
		info->ignore_status_mask |= Z055_IRQR_RXABRT;
		/* If ignoring parity and break indicators, ignore
		 * overruns too.  (For real raw support).
		 */
		if (I_IGNPAR(tty))
			info->ignore_status_mask |= Z055_IRQR_RXBOVR;
	}

	z055_program_hw(info);

}   /* end of z055_change_params() */

/* z055_put_char()
 *
 *  Add a character to the transmit buffer.
 *
 * Arguments:       tty pointer to tty information structure
 *          ch  character to add to transmit buffer
 *
 * Return Value:    None
 */
static int z055_put_char(struct tty_struct *tty, unsigned char ch)
{
	struct Z055_STRUCT *info = (struct Z055_STRUCT *)tty->driver_data;
	int ret = 1;

	printk( "%s(%d): function not supported for device %s, ch=%d\n",
			__FUNCTION__, __LINE__, info->device_name, ch);

	return ret;
}   /* end of z055_put_char() */

/* z055_flush_chars()
 *
 * 	Enable transmitter so remaining characters in the
 * 	transmit buffer are sent.
 *
 * Arguments:       tty    pointer to tty information structure
 * Return Value:    None
 */
static void z055_flush_chars(struct tty_struct *tty)
{
	struct Z055_STRUCT *info = (struct Z055_STRUCT *)tty->driver_data;
	printk( "%s(%d): function not supported for device %s\n",
			__FUNCTION__, __LINE__, info->device_name);
}   /* end of z055_flush_chars() */

/* z055_write()
 *
 *  Send a block of data
 *
 * Arguments:
 *
 *  tty         pointer to tty information structure
 *  from_user   flag: 1 = from user process
 *  buf         pointer to buffer containing send data
 *  count       size of send data in bytes
 *
 * Return Value:    number of characters written
 */
static int z055_write(struct tty_struct *tty,
		 const unsigned char *buf, int count)
{
	int ret = 0;
	struct Z055_STRUCT *info = (struct Z055_STRUCT *)tty->driver_data;
	unsigned long flags;

	if ( debug_level & DEBUG_LEVEL_INFO )
		printk( "%s(%d): %s count=%d\n",
				__FUNCTION__, __LINE__, info->device_name, count);

	if (z055_paranoia_check(info, tty->name,    "z055_write"))
		goto cleanup;

	if (!tty || !info->xmit_buf || !G_tmp_buf)
		goto cleanup;
/*
	if ( debug_level & DEBUG_LEVEL_INFO ) {
		printk("Got Data\n");
		z055_trace_block(info, buf, 48, 1);

	}
*/
	if ( info->params.mode == Z055_MODE_HDLC )    {
		/* operating in synchronous (frame oriented) mode */

		if (info->tx_active) {
			ret = 0;
			if ( debug_level & DEBUG_LEVEL_INFO )
				printk( "%s(%d): %s tx_active in MODE_HDLC\n",
						__FUNCTION__, __LINE__, info->device_name);
			goto cleanup;
		}

		if ( debug_level & DEBUG_LEVEL_INFO )
			printk( "%s(%d): %s sync transmit accepted\n",
					__FUNCTION__, __LINE__, info->device_name);
		ret = count;
		info->xmit_cnt = count;

		z055_load_tx_buffer(info,buf,count);
			ret = count;
	} else
		printk( KERN_ERR"%s(%d): %s modes others than HDLC are not supported!\n",
				__FUNCTION__, __LINE__, info->device_name, ret);

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,14,0)
	if (info->xmit_cnt && !tty->stopped && !tty->hw_stopped) {
#else
	if (info->xmit_cnt && !tty->flow.stopped && !tty->hw_stopped) {
#endif
		spin_lock_irqsave(&info->irq_spinlock,flags);
		if (!info->tx_active)
			z055_hw_start_transmitter(info);
		spin_unlock_irqrestore(&info->irq_spinlock,flags);
	}
cleanup:
	if ( debug_level & DEBUG_LEVEL_INFO )
		printk( "%s(%d): %s returning=%d\n",
			__FUNCTION__, __LINE__, info->device_name, ret);

	return ret;
}   /* end of z055_write() */

/* z055_write_room()
 *
 *  Return the count of free bytes in transmit buffer
 *
 * Arguments:       tty pointer to tty info structure
 * Return Value:    None
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,14,0)
static int z055_write_room(struct tty_struct *tty)
#else
static unsigned int z055_write_room(struct tty_struct *tty)
#endif
{
	struct Z055_STRUCT *info    = (struct Z055_STRUCT *)tty->driver_data;

	if (z055_paranoia_check(info, tty->name,    "z055_write_room"))
		return 0;

	if ( debug_level & DEBUG_LEVEL_INFO )
		printk( "%s(%d): %s\n", __FUNCTION__, __LINE__, info->device_name);

	if ( info->params.mode == Z055_MODE_HDLC  )    {
		/* operating in synchronous (frame oriented) mode */
		if ( info->tx_active )
			return 0;
		else
			return HDLC_MAX_FRAME_SIZE;
	} else
		return 0;

}   /* end of z055_write_room() */

/* z055_chars_in_buffer()
 *
 *  Return the count of bytes in transmit buffer
 *
 * Arguments:       tty pointer to tty info structure
 * Return Value:    None
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,14,0)
static int z055_chars_in_buffer(struct tty_struct *tty)
#else
static unsigned int z055_chars_in_buffer(struct tty_struct *tty)
#endif
{
	struct Z055_STRUCT *info    = (struct Z055_STRUCT *)tty->driver_data;

	if (debug_level & DEBUG_LEVEL_INFO)
		printk( "%s(%d): %s\n", __FUNCTION__, __LINE__, info->device_name );

	if (z055_paranoia_check(info, tty->name,    "z055_chars_in_buffer"))
		return 0;

	if (debug_level & DEBUG_LEVEL_INFO)
		printk( "%s(%d): %s = %d\n",
				__FUNCTION__, __LINE__, info->device_name, info->xmit_cnt );

	if ( info->params.mode == Z055_MODE_HDLC )    {
		/* operating in synchronous (frame oriented) mode */
		if ( info->tx_active )
			return info->max_frame_size;
		else
			return 0;
	} else
		return 0;
}   /* end of z055_chars_in_buffer()    */

/* z055_flush_buffer()
 *
 *  Discard all data in the send buffer nothing to do for z055 device
 *
 * Arguments:       tty pointer to tty info structure
 * Return Value:    None
 */
static void z055_flush_buffer(struct tty_struct *tty)
{
}   /* end of z055_flush_buffer() */

/* z055_send_xchar()
 *
 *  Send a high-priority XON/XOFF character
 *
 * Arguments:       tty pointer to tty info structure
 *          ch  character to send
 * Return Value:    None
 */
static void z055_send_xchar(struct tty_struct *tty, char    ch)
{
	struct Z055_STRUCT *info    = (struct Z055_STRUCT *)tty->driver_data;
	printk( "*** ERROR %s(%d): function not supported for device: %s, ch %d\n",
			 	__FUNCTION__, __LINE__, info->device_name, ch );

}   /* end of z055_send_xchar() */

/* z055_throttle()
 *
 *  Signal remote device to throttle send data (our receive data)
 *
 * Arguments:       tty pointer to tty info structure
 * Return Value:    None
 */
static void z055_throttle(struct tty_struct * tty)
{
	struct Z055_STRUCT *info    = (struct Z055_STRUCT *)tty->driver_data;
	printk( "*** ERROR %s(%d): function not supported for device: %s\n",
			 	__FUNCTION__, __LINE__, info->device_name );

}   /* end of z055_throttle() */

/* z055_unthrottle()
 *
 *  Signal remote device to stop throttling send data (our receive data)
 *
 * Arguments:       tty pointer to tty info structure
 * Return Value:    None
 */
static void z055_unthrottle(struct tty_struct * tty)
{
	struct Z055_STRUCT *info    = (struct Z055_STRUCT *)tty->driver_data;
	printk( "*** ERROR %s(%d): function not supported for device: %s\n",
			 	__FUNCTION__, __LINE__, info->device_name );

}   /* end of z055_unthrottle() */

/* z055_get_stats()
 *
 *  get the current serial parameters information
 *
 * Arguments:   info        pointer to device instance data
 *      user_icount pointer to buffer to hold returned stats
 *
 * Return Value:    0 if success, otherwise error code
 */
static int z055_get_stats(struct    Z055_STRUCT * info,
							 struct Z055_ICOUNT *user_icount)
{
	int err;

	if (debug_level & DEBUG_LEVEL_INFO)
		printk( "%s(%d): %s\n", __FUNCTION__, __LINE__, info->device_name);

	COPY_TO_USER(err,user_icount, &info->icount, sizeof(struct Z055_ICOUNT));
	if (err) {
		if ( debug_level & DEBUG_LEVEL_INFO )
			printk( "%s(%d): %s user buffer copy failed\n",
					__FUNCTION__, __LINE__, info->device_name);
		return -EFAULT;
	}

	return 0;

}   /* end of z055_get_stats() */

/* z055_get_params()
 *
 *  get the current serial parameters information
 *
 * Arguments:   info        pointer to device instance data
 *      user_params pointer to buffer to hold returned params
 *
 * Return Value:    0 if success, otherwise error code
 */
static int z055_get_params(struct Z055_STRUCT   *info,  Z055_PARAMS *user_params)
{
	int err;
	if (debug_level & DEBUG_LEVEL_INFO)
		printk("%s(%d): %s\n", __FUNCTION__, __LINE__, info->device_name);

	COPY_TO_USER(err,user_params, &info->params, sizeof(Z055_PARAMS));
	if (err) {
		if ( debug_level & DEBUG_LEVEL_INFO )
			printk( "%s(%d): %s user buffer copy failed\n",
					__FUNCTION__, __LINE__, info->device_name);
		return -EFAULT;
	}

	return 0;

}   /* end of z055_get_params() */

/* z055_set_params()
 *
 *  set the serial parameters
 *
 * Arguments:
 *
 *  info        pointer to device instance data
 *  new_params  user buffer containing new serial params
 *
 * Return Value:    0 if success, otherwise error code
 */
static int z055_set_params(struct Z055_STRUCT   * info, Z055_PARAMS *new_params)
{
	unsigned long flags;
	Z055_PARAMS tmp_params;
	int err;

	if (debug_level & DEBUG_LEVEL_INFO)
		printk( "%s(%d): %s\n", __FUNCTION__, __LINE__,	info->device_name );

	COPY_FROM_USER(err, &tmp_params, new_params, sizeof(Z055_PARAMS));
	if (err) {
		if ( debug_level & DEBUG_LEVEL_INFO )
			printk( "%s(%d): %s user buffer copy failed\n",
					__FUNCTION__, __LINE__, info->device_name);
		return -EFAULT;
	}

	spin_lock_irqsave(&info->irq_spinlock,flags);
	memcpy(&info->params,&tmp_params,sizeof(Z055_PARAMS));
	spin_unlock_irqrestore(&info->irq_spinlock,flags);

	if (debug_level & DEBUG_LEVEL_INFO)
		printk( "%s(%d): %s: baud_rate = %d\n",
				__FUNCTION__, __LINE__,	info->device_name,
				info->params.baud_rate);
	z055_change_params(info);

	return 0;

}   /* end of z055_set_params() */

/* z055_get_txidle()
 *
 *  get the current transmit idle mode
 *
 * Arguments:   info        pointer to device instance data
 *      idle_mode   pointer to buffer to hold returned idle mode
 *
 * Return Value:    0 if success, otherwise error code
 */
static int z055_get_txidle(struct Z055_STRUCT   * info, int*idle_mode)
{
	int err;

	if (debug_level & DEBUG_LEVEL_INFO)
		printk( "%s(%d): %s idle = %d\n",
			 	__FUNCTION__, __LINE__, info->device_name, info->idle_mode);

	COPY_TO_USER(err,idle_mode, &info->idle_mode, sizeof(int));
	if (err) {
		if ( debug_level & DEBUG_LEVEL_INFO )
			printk( "%s(%d): %s user buffer copy failed\n",
					__FUNCTION__, __LINE__, info->device_name);
		return -EFAULT;
	}

	return 0;

}   /* end of z055_get_txidle() */

/* z055_set_txidle()    service ioctl to set transmit idle mode
 *
 * Arguments: info       pointer to device instance data
 *            idle_mode  new idle mode
 *
 * Return Value:    0 if success, otherwise error code
 */
static int z055_set_txidle(struct Z055_STRUCT *info, int idle_mode)
{
	unsigned long flags;

	if (debug_level & DEBUG_LEVEL_INFO)
		printk( "%s(%d): %s idle_mode = %d\n",
				__FUNCTION__, __LINE__, info->device_name, idle_mode );

	spin_lock_irqsave(&info->irq_spinlock,flags);
	info->idle_mode = idle_mode;
	z055_hw_set_txidle( info );
	spin_unlock_irqrestore(&info->irq_spinlock,flags);
	return 0;

}   /* end of z055_set_txidle() */

/* z055_txenable()
 *
 *  enable or disable the transmitter
 *
 * Arguments:
 *
 *  info        pointer to device instance data
 *  enable      1 = enable, 0 = disable
 *
 * Return Value:    0 if success, otherwise error code
 */
static int z055_txenable(struct Z055_STRUCT *info, int enable)
{
	unsigned long flags;

	if (debug_level & DEBUG_LEVEL_INFO)
		printk( "%s(%d): %s en = %d\n",
				__FUNCTION__, __LINE__, info->device_name, enable);

	spin_lock_irqsave(&info->irq_spinlock,flags);
	if ( enable ) {
		if ( !info->tx_enabled ) {
			z055_hw_start_transmitter(info);
		}
	} else {
		if ( info->tx_enabled )
			z055_hw_stop_transmitter(info);
	}
	spin_unlock_irqrestore(&info->irq_spinlock,flags);
	return 0;

}   /* end of z055_txenable() */

/* z055_txabort()   abort send HDLC frame
 *
 * Arguments:       info        pointer to device instance data
 * Return Value:    0 if success, otherwise error code
 */
static int z055_txabort(struct Z055_STRUCT *info)
{
	unsigned long flags;

	if (debug_level & DEBUG_LEVEL_INFO)
		printk( "%s(%d): %s\n", __FUNCTION__, __LINE__, info->device_name);

	spin_lock_irqsave(&info->irq_spinlock,flags);
	if ( info->tx_active && info->params.mode == Z055_MODE_HDLC )
	{
		Z055_OUTREG(info, Z055_HCR, Z055_INREG(info,Z055_HCR) |Z055_HCR_SENDBRK);
	}
	spin_unlock_irqrestore(&info->irq_spinlock,flags);
	return 0;

}   /* end of z055_txabort()    */

/* z055_rxenable()  enable or disable the receiver
 *
 * Arguments:       info        pointer to device instance data
 *          enable      1 = enable, 0 = disable
 * Return Value:    0 if success, otherwise error code
 */
static int z055_rxenable(struct Z055_STRUCT *   info, int enable)
{
	unsigned long flags;

	if (debug_level & DEBUG_LEVEL_INFO)
		printk( "%s(%d): %s en = %d\n",
				__FUNCTION__,__LINE__, info->device_name, enable);

	spin_lock_irqsave(&info->irq_spinlock,flags);
	if ( enable ) {
		if ( !info->rx_enabled )
			z055_hw_start_receiver(info);
	} else {
		if ( info->rx_enabled )
			z055_hw_stop_receiver(info);
	}
	spin_unlock_irqrestore(&info->irq_spinlock,flags);
	return 0;

}   /* end of z055_rxenable() */

/* z055_wait_event()        wait for specified event to occur
 *
 * Arguments:       info    pointer to device instance data
 *                  mask    pointer to bitmask of events to wait for
 * Return Value:    0   if successful and bit mask updated with
 *                      of events triggerred,
 *                      otherwise error code
 */
static int z055_wait_event(struct Z055_STRUCT *info, int *mask_ptr)
{
	unsigned long flags;
	int s;
	int rc=0;
	struct Z055_ICOUNT cprev, cnow;
	int events;
	int mask;
	struct  _input_signal_events oldsigs, newsigs;
	DECLARE_WAITQUEUE(wait, current);

	COPY_FROM_USER(rc,&mask, mask_ptr, sizeof(int));
	if (rc) {
		return  -EFAULT;
	}

	if (debug_level & DEBUG_LEVEL_INFO)
		printk( "%s(%d): %s mask = %d\n",
				__FUNCTION__, __LINE__, info->device_name, mask);

	spin_lock_irqsave(&info->irq_spinlock,flags);

	/* return immediately if state matches requested events */
	z055_hw_get_serial_signals(info);
	s = info->serial_signals;
	events = mask &
		( ((s & SerialSignal_DSR) ? Z055EVENT_DSR:0) +
		  ((s & SerialSignal_CTS) ? Z055EVENT_CTS:0) );
	if (events) {
		spin_unlock_irqrestore(&info->irq_spinlock,flags);
		goto exit;
	}

	/* save current irq counts */
	cprev = info->icount;
	oldsigs = info->input_signal_events;

	set_current_state(TASK_INTERRUPTIBLE);
	add_wait_queue(&info->event_wait_q, &wait);

	spin_unlock_irqrestore(&info->irq_spinlock,flags);


	for(;;) {
		schedule();
		if (signal_pending(current)) {
			rc = -ERESTARTSYS;
			break;
		}

		/* get current irq counts */
		spin_lock_irqsave(&info->irq_spinlock,flags);
		cnow = info->icount;
		newsigs = info->input_signal_events;
		set_current_state(TASK_INTERRUPTIBLE);
		spin_unlock_irqrestore(&info->irq_spinlock,flags);

		/* if no change, wait aborted for some reason */
		if (newsigs.dsr   == oldsigs.dsr   &&
			newsigs.cts   == oldsigs.cts ) {
			rc = -EIO;
			break;
		}

		events = mask &
			( (newsigs.dsr != oldsigs.dsr ? Z055EVENT_DSR:0) +
			  (newsigs.cts != oldsigs.cts ? Z055EVENT_CTS:0) );
		if (events)
			break;

		cprev = cnow;
		oldsigs = newsigs;
	}

	remove_wait_queue(&info->event_wait_q, &wait);
	set_current_state(TASK_RUNNING);

exit:
	if ( rc == 0 )
		PUT_USER(rc, events, mask_ptr);

	return rc;
}   /* end of z055_wait_event() */

static int modem_input_wait(struct Z055_STRUCT *info,int    arg)
{
	unsigned long flags;
	int rc;
	struct Z055_ICOUNT cprev, cnow;
	DECLARE_WAITQUEUE(wait, current);

	/* save current irq counts */
	spin_lock_irqsave(&info->irq_spinlock,flags);
	cprev = info->icount;
	add_wait_queue(&info->status_event_wait_q, &wait);
	set_current_state(TASK_INTERRUPTIBLE);
	spin_unlock_irqrestore(&info->irq_spinlock,flags);

	for(;;) {
		schedule();
		if (signal_pending(current)) {
			rc = -ERESTARTSYS;
			break;
		}

		/* get new irq counts */
		spin_lock_irqsave(&info->irq_spinlock,flags);
		cnow = info->icount;
		set_current_state(TASK_INTERRUPTIBLE);
		spin_unlock_irqrestore(&info->irq_spinlock,flags);

		/* if no change, wait aborted for some reason */
		if (cnow.dsr == cprev.dsr && cnow.cts == cprev.cts) {
			rc = -EIO;
			break;
		}

		/* check for change in caller specified modem input */
		if ((arg & TIOCM_DSR && cnow.dsr != cprev.dsr) ||
			(arg & TIOCM_CTS && cnow.cts != cprev.cts)) {
			rc = 0;
			break;
		}

		cprev = cnow;
	}
	remove_wait_queue(&info->status_event_wait_q, &wait);
	set_current_state(TASK_RUNNING);
	return rc;
} /* end of modem_input_wait */

/* z055_hw_get_modem_info()
 *
 *  Read the state of the serial control and
 *  status signals and return to caller.
 *
 * Arguments:       info    pointer to device instance data
 *          value   pointer to int to hold returned info
 *
 * Return Value:    0 if success, otherwise error code
 */
static unsigned int z055_hw_get_modem_info(struct Z055_STRUCT * info)
{
	unsigned int result = 0;
	unsigned long flags;

	spin_lock_irqsave(&info->irq_spinlock,flags);
	z055_hw_get_serial_signals(info);
	spin_unlock_irqrestore(&info->irq_spinlock,flags);

	result = ((info->serial_signals & SerialSignal_RTS) ? TIOCM_RTS:0) +
		((info->serial_signals & SerialSignal_DTR) ? TIOCM_DTR:0) +
		((info->serial_signals & SerialSignal_DSR) ? TIOCM_DSR:0) +
		((info->serial_signals & SerialSignal_CTS) ? TIOCM_CTS:0);

	if (debug_level & DEBUG_LEVEL_INFO)
		printk( "%s(%d): %s value=%08X\n",
			 	__FUNCTION__, __LINE__, info->device_name, result );

	return result;
}   /* end of get_modem_info() */

/* z055_hw_set_modem_info()
 *
 *  Set the state of the modem control signals (DTR/RTS)
 *
 * Arguments:
 *
 *  info    pointer to device instance data
 *  cmd signal command: TIOCMBIS = set bit TIOCMBIC = clear bit
 *      TIOCMSET = set/clear signal values
 *  set     bit mask for signals to set
 *  clear   bitmask for signals to reset
 *
 * Return Value:    0 if success, otherwise error code
 */
static int z055_hw_set_modem_info(struct Z055_STRUCT * info,
									 unsigned int set,
									 unsigned int clear)
{
	unsigned long flags;

	if (debug_level & DEBUG_LEVEL_INFO)
		printk( "%s(%d):%s (set = %x, clear = %x)\n",
				__FUNCTION__, __LINE__, info->device_name, set, clear);

	if (set & TIOCM_RTS)
		info->serial_signals |= SerialSignal_RTS;
	if (set & TIOCM_DTR)
		info->serial_signals |= SerialSignal_DTR;
	if (clear & TIOCM_RTS)
		info->serial_signals &= ~SerialSignal_RTS;
	if (clear & TIOCM_DTR)
		info->serial_signals &= ~SerialSignal_DTR;

	spin_lock_irqsave(&info->irq_spinlock,flags);
	z055_hw_set_serial_signals(info);
	spin_unlock_irqrestore(&info->irq_spinlock,flags);

	return 0;
} /* z055_hw_set_modem_info() */

/* return the state of the serial control and status signals
 */
static int tiocmget(struct tty_struct *tty)
{
	return(z055_hw_get_modem_info((struct Z055_STRUCT *)tty->driver_data));
}

/* set modem control signals (DTR/RTS)
 */
static int tiocmset(struct tty_struct *tty, unsigned int set, unsigned int clear)
{
	return z055_hw_set_modem_info((struct Z055_STRUCT *)tty->driver_data,
									 set, clear);
}

/* z055_break()     Set or clear transmit break condition
 *
 * Arguments:       tty     pointer to tty instance data
 *          break_state -1=set break condition, 0=clear
 * Return Value:    None
 */
static int  z055_break(struct tty_struct *tty, int break_state)
{
	struct Z055_STRUCT *info = (struct Z055_STRUCT *)tty->driver_data;
	unsigned long flags;

	if (debug_level & DEBUG_LEVEL_INFO)
		printk( "%s(%d): %s break = %d\n",
			 	__FUNCTION__, __LINE__, info->device_name, break_state);

	if (z055_paranoia_check(info, tty->name,    "z055_break"))
		return -ENODEV;

	spin_lock_irqsave(&info->irq_spinlock,flags);
	if (break_state == -1)
		Z055_OUTREG( info, Z055_HCR,
					 Z055_INREG( info, Z055_HCR ) | Z055_HCR_SENDBRK );
	else
		Z055_OUTREG( info, Z055_HCR,
					 Z055_INREG( info, Z055_HCR ) & ~Z055_HCR_SENDBRK );
	spin_unlock_irqrestore(&info->irq_spinlock,flags);
	return 0;
}   /* end of z055_break() */

/* z055_ioctl() Service an IOCTL request
 *
 * Arguments:
 *
 *  tty pointer to tty instance data
 *  file    pointer to associated file object for device
 *  cmd IOCTL command code
 *  arg command argument/context
 *
 * Return Value:    0 if success, otherwise error code
 */
static int z055_ioctl(struct tty_struct *tty, unsigned int cmd, unsigned long arg)
{
	struct Z055_STRUCT *info = tty->driver_data;

	if (debug_level & DEBUG_LEVEL_INFO)
		printk( "%s(%d): %s cmd=%08X\n",
				__FUNCTION__, __LINE__, info->device_name, cmd );

	if (z055_paranoia_check(info, tty->name,    "z055_ioctl"))
		return -ENODEV;

	if ((cmd != TIOCGSERIAL) && (cmd != TIOCSSERIAL) &&
		(cmd != TIOCMIWAIT) && (cmd != TIOCGICOUNT)) {
		if (tty->flags & (1 << TTY_IO_ERROR))
			return -EIO;
	}

	return z055_ioctl_common(info, cmd, arg);
}

static int z055_ioctl_common(struct Z055_STRUCT *info,
					  unsigned int cmd,
					  unsigned long arg)
{
	int error;
	struct Z055_ICOUNT cnow;    /* kernel counter temps */
	struct serial_icounter_struct *p_cuser; /* user space */
	unsigned long flags;

	if (debug_level & DEBUG_LEVEL_INFO)
		printk( "%s(%d): %s cmd=%08X\n",
				__FUNCTION__, __LINE__, info->device_name, cmd );

	switch (cmd) {
		case Z055_IOCGPARAMS:
			return z055_get_params(info,(Z055_PARAMS *)arg);
		case Z055_IOCSPARAMS:
			return z055_set_params(info,(Z055_PARAMS *)arg);
		case Z055_IOCGTXIDLE:
			return z055_get_txidle(info,(int*)arg);
		case Z055_IOCSTXIDLE:
			return z055_set_txidle(info,(int)arg);
		case Z055_IOCTXENABLE:
			return z055_txenable(info,(int)arg);
		case Z055_IOCRXENABLE:
			return z055_rxenable(info,(int)arg);
		case Z055_IOCTXABORT:
			return z055_txabort(info);
		case Z055_IOCGSTATS:
			return z055_get_stats(info,(struct Z055_ICOUNT*)arg);
		case Z055_IOCWAITEVENT:
			return z055_wait_event(info,(int*)arg);
		case TIOCMIWAIT:
			return modem_input_wait(info,(int)arg);

		/*
		 * Get counter of input serial line interrupts (DCD,RI,DSR,CTS)
		 * Return: write counters to the user passed counter struct
		 * NB: both 1->0 and 0->1 transitions are counted
		 */
		case TIOCGICOUNT:
			spin_lock_irqsave(&info->irq_spinlock,flags);
			cnow = info->icount;
			spin_unlock_irqrestore(&info->irq_spinlock,flags);
			p_cuser = (struct serial_icounter_struct *) arg;
			PUT_USER(error,cnow.cts, &p_cuser->cts);
			if (error) return error;
			PUT_USER(error,cnow.dsr, &p_cuser->dsr);
			if (error) return error;
			/* no async mode supported => no more values to pass */
			return 0;
		default:
			return -ENOIOCTLCMD;
	}
	return 0;
}


static long z055_compat_ioctl(struct tty_struct *tty,
			 unsigned int cmd, unsigned long arg)
{
	int rc = -ENOIOCTLCMD;
	return rc;
}

/* z055_set_termios()
 *
 *  Set new termios settings
 *
 * Arguments:
 *
 *  tty     pointer to tty structure
 *  termios     pointer to buffer to hold returned old termios
 *
 * Return Value:        None
 */
static void z055_set_termios(struct tty_struct *tty, struct ktermios *old_termios)
{
	struct Z055_STRUCT *info    = (struct Z055_STRUCT *)tty->driver_data;
	unsigned long flags;

	if (debug_level & DEBUG_LEVEL_INFO)
		printk( "%s(%d): %s",
				__FUNCTION__,__LINE__, tty->driver->name);

	/* just return if nothing has changed */
	if ((tty_cflags(tty) == old_termios->c_cflag)
		&& (RELEVANT_IFLAG(tty_cflags(tty))
		== RELEVANT_IFLAG(old_termios->c_iflag)))
		return;

	z055_change_params(info);

	/* Handle transition to B0 status */
	if (old_termios->c_cflag & CBAUD &&
		!(tty_cflags(tty) & CBAUD)) {
		info->serial_signals &= ~(SerialSignal_RTS + SerialSignal_DTR);
		spin_lock_irqsave(&info->irq_spinlock,flags);
		z055_hw_set_serial_signals(info);
		spin_unlock_irqrestore(&info->irq_spinlock,flags);
	}

	/* Handle transition away from B0 status */
	if (!(old_termios->c_cflag & CBAUD) &&
		tty_cflags(tty) & CBAUD) {
		info->serial_signals |= SerialSignal_DTR;
		if (!(tty_cflags(tty) & CRTSCTS) ||
			!test_bit(TTY_THROTTLED, &tty->flags)) {
			info->serial_signals |= SerialSignal_RTS;
		}
		spin_lock_irqsave(&info->irq_spinlock,flags);
		z055_hw_set_serial_signals(info);
		spin_unlock_irqrestore(&info->irq_spinlock,flags);
	}

	/* Handle turning off CRTSCTS */
	if (old_termios->c_cflag & CRTSCTS &&
		!(tty_cflags(tty) & CRTSCTS)) {
		tty->hw_stopped = 0;
		z055_start(tty);
	}

}   /* end of z055_set_termios()    */

/* z055_close()
 *
 *  Called when port is closed. Wait for remaining data to be
 *  sent. Disable port and free resources.
 *
 * Arguments:
 *
 *  tty pointer to open tty structure
 *  filp    pointer to open file object
 *
 * Return Value:    None
 */
static void z055_close(struct tty_struct    *tty, struct file * filp)
{
	struct Z055_STRUCT *    info = (struct Z055_STRUCT *)tty->driver_data;

	if (z055_paranoia_check(info, tty->name,    "z055_close"))
		return;

	if (debug_level & DEBUG_LEVEL_INFO)
		printk( "%s(%d): %s entry, count=%d\n",
				__FUNCTION__, __LINE__, info->device_name, Z055_STRUCT_ref_count(info));

	if (!Z055_STRUCT_ref_count(info))
		return;

	if (tty_hung_up_p(filp))
		goto cleanup;

	if ((tty->count == 1) && (Z055_STRUCT_ref_count(info) != 1)) {
		/*
		 * tty->count is 1 and the tty structure will be freed.
		 * info->count should be one in this case.
		 * if it's not, correct it so that the port is shutdown.
		 */
		printk("z055_close: bad refcount; tty->count    is 1, "
			   "info->count is %d\n", Z055_STRUCT_ref_count(info));
		Z055_STRUCT_set_ref_count(info, 1);
	}

	Z055_STRUCT_dec_ref_count(info);

	/* if at least one open remaining, leave hardware active */
	if (Z055_STRUCT_ref_count(info))
		goto cleanup;

	/* set tty->closing to notify line discipline to
	 * only process XON/XOFF characters. Only the N_TTY
	 * discipline appears to use this (ppp does not).
	 */
	tty->closing = 1;

	/* wait for transmit data to clear all layers */

	if (Z055_STRUCT_get_closing_wait(info) != ASYNC_CLOSING_WAIT_NONE) {
		if (debug_level & DEBUG_LEVEL_INFO)
			printk( "%s(%d): %s calling tty_wait_until_sent\n",
					__FUNCTION__, __LINE__, info->device_name );
		tty_wait_until_sent(tty, Z055_STRUCT_get_closing_wait(info));
	}

	if (tty_port_initialized(&info->port))
		z055_wait_until_sent(tty, info->timeout);

	if (tty->driver->ops->flush_buffer)
		tty->driver->ops->flush_buffer(tty);

	if (tty->ldisc->ops->flush_buffer)
		tty->ldisc->ops->flush_buffer(tty);

	shutdown(info);

	tty->closing = 0;
	Z055_STRUCT_set_tty(info, NULL);

	if (Z055_STRUCT_blocked_open(info)) {
		if (Z055_STRUCT_get_close_delay(info)) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(Z055_STRUCT_get_close_delay(info));
		}
		wake_up_interruptible(&Z055_STRUCT_open_wait_q(info));
	}

	tty_port_set_active(&info->port, 0);

cleanup:
	if (debug_level & DEBUG_LEVEL_INFO)
		printk( "%s(%d): %s exit, count=%d\n", __FUNCTION__, __LINE__,
				tty->driver->name,
				Z055_STRUCT_ref_count(info));

}   /* end of z055_close() */

/* z055_wait_until_sent()
 *
 *  Wait until the transmitter is empty.
 *
 * Arguments:
 *
 *  tty         pointer to tty info structure
 *  timeout     time to wait for send completion
 *
 * Return Value:    None
 */
static void z055_wait_until_sent(struct tty_struct *tty, int timeout)
{
	struct Z055_STRUCT *info = (struct Z055_STRUCT *)tty->driver_data;
	unsigned long orig_jiffies, char_time;

	if (!info )
		return;

	if (debug_level & DEBUG_LEVEL_INFO)
		printk( "%s(%d): %s entry\n",
			 	__FUNCTION__, __LINE__, info->device_name );

	if (z055_paranoia_check(info, tty->name,    "z055_wait_until_sent"))
		return;

	if (!tty_port_initialized(&info->port))
		goto exit;

	orig_jiffies = jiffies;

	/* Set check interval to 1/5 of estimated time to
	 * send a character, and make it at least 1. The check
	 * interval should also be less than the timeout.
	 * Note: use tight timings here to satisfy the NIST-PCTS.
	 */

	if ( info->params.baud_rate ) {
			char_time = info->timeout/(32 * 5);
		if (!char_time)
			char_time++;
	} else
		char_time = 1;

	if (timeout)
		char_time = MIN(char_time, timeout);

	if ( info->params.mode == Z055_MODE_HDLC )    {
		while ( info->tx_active ) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(char_time);
			if (signal_pending(current))
				break;
			if (timeout && time_after(jiffies, orig_jiffies + timeout))
				break;
		}
	}

exit:
	if (debug_level & DEBUG_LEVEL_INFO)
		printk("%s(%d): %s exit\n", __FUNCTION__, __LINE__, info->device_name );

}   /* end of z055_wait_until_sent()    */

/* z055_hangup()
 *
 *  Called by tty_hangup() when a hangup is signaled.
 *  This is the same as to closing all open files for the port.
 *
 * Arguments:       tty pointer to associated tty object
 * Return Value:    None
 */
static void z055_hangup(struct tty_struct *tty)
{
	struct Z055_STRUCT *    info = (struct Z055_STRUCT *)tty->driver_data;

	if (debug_level & DEBUG_LEVEL_INFO)
		printk("%s(%d): %s\n", __FUNCTION__, __LINE__, info->device_name );

	if (z055_paranoia_check(info, tty->name,    "z055_hangup"))
		return;

	z055_flush_buffer(tty);
	shutdown(info);

	Z055_STRUCT_set_ref_count(info, 0);
	tty_port_set_active(&info->port, 0);
	Z055_STRUCT_set_tty(info, NULL);

	wake_up_interruptible(&Z055_STRUCT_open_wait_q(info));

}   /* end of z055_hangup() */

/* block_til_ready()
 *
 *  Block the current process until the specified port
 *  is ready to be opened.
 *
 * Arguments:
 *
 *  tty     pointer to tty info structure
 *  filp        pointer to open file object
 *  info        pointer to device instance data
 *
 * Return Value:    0 if success, otherwise error code
 */
static int block_til_ready(struct tty_struct *tty, struct file * filp,
			   struct Z055_STRUCT *info)
{
	DECLARE_WAITQUEUE(wait, current);
	int     retval;
	int     do_clocal = 0;
#ifndef RHEL_7_3_514
	int extra_count = 0;
#endif
	unsigned long   flags;

	if (debug_level & DEBUG_LEVEL_INFO)
		printk("%s(%d): on %s\n",
			 __FUNCTION__, __LINE__,
			tty->driver->name
		 );
	if (filp->f_flags & O_NONBLOCK || tty->flags & (1 << TTY_IO_ERROR)){
		/* nonblock mode is set or port is not enabled */
		tty_port_set_active(&info->port, 1);
		return 0;
	}

	if (tty_cflags(tty) & CLOCAL)
		do_clocal = 1;

	/* Wait for carrier detect and the line to become
	 * free (i.e., not in use by the callout).  While we are in
	 * this loop, info->count is dropped by one, so that
	 * z055_close() knows when to free things.  We restore it upon
	 * exit, either normal or abnormal.
	 */

	retval = 0;
	add_wait_queue(&Z055_STRUCT_open_wait_q(info), &wait);

	if (debug_level & DEBUG_LEVEL_INFO)
		printk("%s(%d): before block on %s count=%d\n",
				__FUNCTION__, __LINE__,
				tty->driver->name,
				Z055_STRUCT_ref_count(info) );

	spin_lock_irqsave(&info->irq_spinlock, flags);
#ifndef RHEL_7_3_514
	if (!tty_hung_up_p(filp)) {
		extra_count = 1;
		Z055_STRUCT_dec_ref_count(info);
	}
#else
	Z055_STRUCT_dec_ref_count(info);
#endif
	spin_unlock_irqrestore(&info->irq_spinlock, flags);
	Z055_STRUCT_inc_blocked_open(info);

	while (1) {
		if ((tty_cflags(tty) & CBAUD)) {
			spin_lock_irqsave(&info->irq_spinlock,flags);
			info->serial_signals |= SerialSignal_RTS + SerialSignal_DTR;
			z055_hw_set_serial_signals(info);
			spin_unlock_irqrestore(&info->irq_spinlock,flags);
		}

		set_current_state(TASK_INTERRUPTIBLE);

		if (tty_hung_up_p(filp) || !tty_port_initialized(&info->port)){
			retval = (Z055_STRUCT_flags(info) & ASYNC_HUP_NOTIFY) ?
					-EAGAIN : -ERESTARTSYS;
			break;
		}

		spin_lock_irqsave(&info->irq_spinlock,flags);
		z055_hw_get_serial_signals(info);
		spin_unlock_irqrestore(&info->irq_spinlock,flags);

		if (do_clocal)
			break;

		if (signal_pending(current)) {
			retval = -ERESTARTSYS;
			break;
		}

		if (debug_level & DEBUG_LEVEL_INFO)
			printk( "%s(%d): blocking on %s count=%d\n",
					__FUNCTION__, __LINE__, tty->driver->name, Z055_STRUCT_ref_count(info) );

		schedule();
	}

	set_current_state(TASK_RUNNING);
	remove_wait_queue(&Z055_STRUCT_open_wait_q(info), &wait);

#ifndef RHEL_7_3_514
	if (extra_count)
		Z055_STRUCT_inc_ref_count(info);
#else
	if (!tty_hung_up_p(filp))
		Z055_STRUCT_inc_ref_count(info);
#endif

	Z055_STRUCT_dec_blocked_open(info);

	if (debug_level & DEBUG_LEVEL_INFO)
		printk( "%s(%d): after blocking on %s count=%d\n",
				__FUNCTION__, __LINE__,
				tty->driver->name,
				Z055_STRUCT_ref_count(info) );

	if (!retval)
		tty_port_set_active(&info->port, 1);

	return retval;

}   /* end of block_til_ready() */


static int z055_tty_install(struct tty_driver *driver, struct tty_struct *tty)
{
	struct Z055_STRUCT *info;
	int line;
	/* find device instance with matching line number */
	info = G_z055_device_list;

	/* verify range of specified line number */
	line = tty->index;

	while(info && info->line != line) {
		info = info->next_device;
	}

	if (!info) {
		printk( "%s(%d):%s: tty line #%d not found\n",
				__FUNCTION__, __LINE__,G_driver_name, line);
		return -ENODEV;
	}

	if (info->init_error) {
		printk( "%s(%d): %s init error=%d\n",
				__FUNCTION__, __LINE__, info->device_name, info->init_error);
		return -ENODEV;
	}

	tty->driver_data = info;

	printk( "tty_port_install\n");
	return tty_port_install(&info->port, driver, tty);
}

/* z055_open()
 *
 *  Called when a port is opened.  Init and enable port.
 *  Perform serial-specific initialization for the tty structure.
 *
 * Arguments:       tty pointer to tty info structure
 *          filp    associated file pointer
 *
 * Return Value:    0 if success, otherwise error code
 */
static int z055_open(struct tty_struct *tty, struct file * filp)
{
	struct Z055_STRUCT  *info;
	int             retval, line;
	unsigned long   page;

	/* verify range of specified line number */
	line = tty->index;
	if ((line < 0) || (line >= G_z055_device_count))    {
		printk( "%s(%d): invalid line #%d.\n",
				__FUNCTION__, __LINE__, line);
		return -ENODEV;
	}

	/* find the info structure for the specified line */
	info = G_z055_device_list;
	while(info && info->line != line) {
		info = info->next_device;
	}

	if (z055_paranoia_check(info, tty->name,    "z055_open"))
		return -ENODEV;

	tty->driver_data = info;
	Z055_STRUCT_set_tty(info, tty);

	if (debug_level & DEBUG_LEVEL_INFO)
		printk( "%s(%d): %s, old ref count = %d\n",
				__FUNCTION__, __LINE__,
				tty->driver->name,
				Z055_STRUCT_ref_count(info));

	if (!G_tmp_buf) {
		page = get_zeroed_page(GFP_KERNEL);
		if (!page) {
			retval = -ENOMEM;
			goto cleanup;
		}
		if (G_tmp_buf)
			free_page(page);
		else
			G_tmp_buf = (unsigned char *) page;
	}

	Z055_STRUCT_inc_ref_count(info);

	if (Z055_STRUCT_ref_count(info) == 1) {
		/* 1st open on this device, init hardware */
		retval = startup(info);
		if (retval < 0)
			goto cleanup;
	}

	retval = block_til_ready(tty, filp, info);
	if (retval) {
		if (debug_level & DEBUG_LEVEL_INFO)
			printk( "%s(%d):block_til_ready(%s) returned %d\n",
				 	__FUNCTION__, __LINE__, info->device_name, retval);
		goto cleanup;
	}

	if (debug_level & DEBUG_LEVEL_INFO)
		printk( "%s(%d): %s success\n",
			 	__FUNCTION__, __LINE__, info->device_name);
	retval = 0;

cleanup:
	if (retval) {
		if (tty->count == 1)
			Z055_STRUCT_set_tty(info, NULL); /* tty layer will release tty struct */
		if(Z055_STRUCT_ref_count(info))
			Z055_STRUCT_dec_ref_count(info);
	}

	return retval;

}   /* end of z055_open() */

/*
 * /proc fs routines....
 */
static inline void line_info(struct seq_file *m, struct Z055_STRUCT *info)
{
	char stat_buf[30];
	unsigned long flags;

	if (debug_level & DEBUG_LEVEL_INFO)
		printk( "%s(%d): %s\n",
			 	__FUNCTION__, __LINE__, info->device_name);

	if (info->bus_type == Z055_BUS_TYPE_PCI)    {
		if( !info->ma_base ) {
			seq_printf( m, "%s: Device not opened yet ==> "
								"no ressources claimed\n",
						   info->device_name);
		}
		seq_printf( m, "%s: PCI ma_base:%04X irq:%d\n",
					info->device_name, info->ma_base + info->ma_offs, info->irq);
	}

	/* output current serial signal states */
	spin_lock_irqsave(&info->irq_spinlock,flags);
	z055_hw_get_serial_signals(info);
	spin_unlock_irqrestore(&info->irq_spinlock,flags);

	stat_buf[0] = 0;
	stat_buf[1] = 0;
	if (info->serial_signals & SerialSignal_RTS)
		strcat(stat_buf, "|RTS");
	if (info->serial_signals & SerialSignal_CTS)
		strcat(stat_buf, "|CTS");
	if (info->serial_signals & SerialSignal_DTR)
		strcat(stat_buf, "|DTR");
	if (info->serial_signals & SerialSignal_DSR)
		strcat(stat_buf, "|DSR");

	if (info->params.mode == Z055_MODE_HDLC ) {
		seq_printf( m, " HDLC txok:%d rxok:%d",
				info->icount.txok, info->icount.rxok);
		if (info->icount.rxrcst)
			seq_printf( m, " rxrcst:%d", info->icount.rxrcst);
		if (info->icount.rxlong)
			seq_printf( m, " rxlong:%d", info->icount.rxlong);
		if (info->icount.rxbover)
			seq_printf( m, " rxbover:%d", info->icount.rxbover);
		if (info->icount.rxcrc)
			seq_printf( m, " rxcrc:%d", info->icount.rxcrc);
		if (info->icount.rxcrc)
			seq_printf( m, " rxabort:%d", info->icount.rxabort);
	}
	/* Append serial signal status to end */
	seq_printf( m, " %s\n", stat_buf+1);

	spin_lock_irqsave(&info->irq_spinlock,flags);
	{
		u16 hcr		= Z055_INREG( info, Z055_HCR );
		u16 cdr		= Z055_INREG( info, Z055_CDR );
		u16 acr		= Z055_INREG( info, Z055_ACR );
		u16 amr		= Z055_INREG( info, Z055_AMR );
		u16 bcl		= Z055_INREG( info, Z055_BCL );
		u16 bch		= Z055_INREG( info, Z055_BCH );
		u16 bcr		= Z055_INREG( info, Z055_BCR );
		u16 ier		= Z055_INREG( info, Z055_IER );
		u16 irqr	= Z055_INREG( info, Z055_IRQR );
		u16 hscr 	= Z055_INREG( info, Z055_HSCR );
		u16 hssr	= Z055_INREG( info, Z055_HSSR );
		u16 txszr	= Z055_INREG( info, Z055_TXSZR );
		u16 rxszr	= Z055_INREG( info, Z055_RXSZR );
		seq_printf( m, "hcr  =%04X cdr =%04X acr  =%04X amr=%04X\n"
								 "bcl  =%04X bch =%04X bcr  =%04X\n"
								 "ier  =%04X irqr=%04X hscr =%04X hssr=%04X\n"
								 "txszr= %04X          rxszr=%04X\n",
						hcr, cdr, acr, amr, bcl, bch, bcr,
						ier, irqr, hscr, hssr, txszr, rxszr );
	}
	seq_printf( m, "txactive=%d bh_req=%d  bh_run=%d   pending_bh=%x\n",
					info->tx_active,info->bh_requested,info->bh_running,
					info->pending_bh);

	spin_unlock_irqrestore(&info->irq_spinlock,flags);

}   /* end of line_info() */

/* Called to print information about devices
 */
static int z055_proc_show(struct seq_file *m, void *v)
{
	struct Z055_STRUCT *info;

	seq_puts(m, "z055_hdlc driver\n");

	info = G_z055_device_list;
	while( info ) {
		line_info(m, info);
		info = info->next_device;
	}
	return 0;
}

#if LINUX_VERSION_CODE < VERSION(4,18,0)
static int z055_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, z055_proc_show, NULL);
}

static const struct file_operations z055_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= z055_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif // linux version < 4.18

/*
 * z055_alloc_rtxbuffer_memory()
 *
 *  Allocate a buffer large enough to hold max_frame_size. This buffer
 *  is used to pass an assembled frame to the line discipline.
 *
 * Arguments:
 *
 *  info        pointer to device instance data
 *
 * Return Value:    0 if success, otherwise -ENOMEM
 */
static int z055_alloc_rtxbuffer_memory( struct Z055_STRUCT *info,
								 struct RXTX_BUFFER_QUEUE_S *queue)
{
	struct RXTX_BUFFER_S *qEnt;
	u8 *datap;
	u32 i;
	if ( debug_level & DEBUG_LEVEL_INFO )
		printk( "%s(%d): %s allocating %d rx/tx buffers of 0x%x bytes\n",
				__FUNCTION__, __LINE__,
				info->device_name, queue->num_buffers, info->max_frame_size);

	/* allocate new queue */
	/*  _________________________________________________________________
	 * |1|2|3|.|.|.|.| | |  1  |  2  |  3  |  .  |  .  |  .  |     |     |
	 * | queue structure |              queue frame data                 |
	 * |_________________|_______________________________________________| */
	queue->buffer_start = kmalloc( queue->num_buffers *
							(info->max_frame_size+sizeof(struct RXTX_BUFFER_S)),
						 	GFP_KERNEL );
	if ( queue->buffer_start == NULL )
		return -ENOMEM;

	/* init queue entries to 0, data buffers not initialized */
	memset(queue->buffer_start, 0,
		   queue->num_buffers*(sizeof(struct RXTX_BUFFER_S)));

	qEnt = queue->get_buffer = queue->put_buffer = (struct RXTX_BUFFER_S*)queue->buffer_start;
	/* queue data entries start at end of queue structure entries */
	datap = (u8 *)(qEnt + queue->num_buffers*sizeof(struct RXTX_BUFFER_S));

	/* init queue */
	for(i = 1; i < queue->num_buffers; i++){
		queue->get_buffer->status = 0x0000;
		queue->get_buffer->buffer = datap;
		queue->get_buffer->next = ++qEnt;
		queue->get_buffer = queue->get_buffer->next;
		datap += info->max_frame_size;
	}
	/* init last, make queue wrap around; queue empty ==> last points to first*/
	queue->get_buffer->status = 0x0000;
	queue->get_buffer->buffer = datap;
	queue->get_buffer->next = queue->put_buffer;
	queue->get_buffer = queue->get_buffer->next;

	if ( debug_level & DEBUG_LEVEL_INFO )
		for(i = 0; i < queue->num_buffers; i++){
			printk( "%s(%d): buf %d; addr=0x%08x; next=0x%08x; data=0x%08x;\n",
        	   		__FUNCTION__, __LINE__, i,
           			queue->get_buffer, queue->get_buffer->next,
           			queue->get_buffer->buffer);
			queue->get_buffer = queue->get_buffer->next;
		}
	return 0;

}   /* end of z055_alloc_rtxbuffer_memory() */

/*
 * z055_free_rtxbuffer_memory()
 *
 *
 * Arguments:
 *
 *  info        pointer to device instance data
 *
 * Return Value:    None
 */
static void z055_free_rtxbuffer_memory( struct Z055_STRUCT *info,
								 struct RXTX_BUFFER_QUEUE_S *queue)
{
	if ( queue->buffer_start )
		kfree(queue->buffer_start);

	queue->buffer_start = NULL;

}   /* end of z055_free_rtxbuffer_memory() */

static int z055_claim_resources(struct Z055_STRUCT *info)
{
#ifdef MAC_MEM_MAPPED
	if ( debug_level & DEBUG_LEVEL_INFO )
		printk( "%s(%d): %s: memory mapped, phys_base: 0x%08x size: 0x%08x\n",
				__FUNCTION__, __LINE__, info->device_name,
				info->phys_base, info->phys_addr_size);

	if (request_mem_region(info->phys_base,info->phys_addr_size,"men_z055") == NULL) {
		printk( "%s(%d): mem addr conflict device %s Addr=%p, size=%p\n",
				__FUNCTION__, __LINE__, info->device_name, info->phys_base,info->phys_addr_size);
		goto errout;
	}
	info->addr_requested = 1;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,5,0)
	info->ma_base = ioremap(info->phys_base, info->phys_addr_size);
#else
	info->ma_base = ioremap_nocache(info->phys_base, info->phys_addr_size);
#endif

	if (!info->ma_base) {
		printk( "%s(%d): Can't map shared memory on device %s MemAddr=%08X\n",
				__FUNCTION__, __LINE__, info->device_name, info->phys_base );
		goto errout;
	}

	if ( debug_level & DEBUG_LEVEL_INFO )
		printk( "%s(%d): %s: memory mapped to ma_base: 0x%08x ma_offs: 0x%08x\n",
				__FUNCTION__, __LINE__, info->device_name,
				info->ma_base, info->ma_offs);

#else
/* Access I/O Mapped space */
	if ( debug_level & DEBUG_LEVEL_INFO )
		printk( "%s(%d): %s: IO-mapped, phys_base: 0x%08x size: 0x%08x\n",
				__FUNCTION__, __LINE__, info->device_name,
				info->phys_base, info->phys_addr_size);

	if (request_region(info->phys_base,info->phys_addr_size,"men_z055") == NULL) {
		printk( "%s(%d): I/O address conflict on device %s Addr=%08X\n",
				__FUNCTION__, __LINE__, info->device_name, info->phys_base);
		return -ENODEV;
	}
	ma_base = info->phys_base;
	info->addr_requested    = 1;

#endif
	if ( request_irq(info->irq,z055_interrupt,info->irq_flags,
		info->device_name, info ) < 0 ) {
		printk( "%s(%d): Can't request interrupt on device %s IRQ=%d\n",
				__FUNCTION__, __LINE__, info->device_name, info->irq );
		goto errout;
	}
	info->irq_requested = 1;

	if ( z055_alloc_rtxbuffer_memory( info, &info->rx_buffer_q ) ) {
		printk( "%s(%d): Can't allocate buffers for device %s\n",
				__FUNCTION__, __LINE__, info->device_name, info->irq );
		goto errout;
	}

	return 0;
errout:
	z055_release_resources(info);
	return -ENODEV;

}   /* end of z055_claim_resources()    */

static void z055_release_resources(struct Z055_STRUCT *info)
{
	if ( debug_level & DEBUG_LEVEL_INFO )
		printk( "%s(%d): %s entry\n", __FUNCTION__, __LINE__, info->device_name);

	if ( info->irq_requested ) {
		free_irq(info->irq, info);
		info->irq_requested = 0;
	}
	z055_free_rtxbuffer_memory( info, &info->rx_buffer_q );

	if ( info->addr_requested ) {
#ifdef MAC_MEM_MAPPED
		release_mem_region(info->phys_base,info->phys_addr_size);
		if (info->ma_base){
			iounmap(info->ma_base);
			info->ma_base = 0;
		}
#else
		release_region(info->phys_base,info->phys_addr_size);
#endif
		info->addr_requested    = 0;
	}

	if ( debug_level & DEBUG_LEVEL_INFO )
		printk( "%s(%d): %s exit\n",
				__FUNCTION__, __LINE__, info->device_name );

}   /* end of z055_release_resources() */

/* z055_add_device()
 *
 *  Add the specified device instance data structure to the
 *  global linked list of devices and increment the device count.
 *
 * Arguments:       info    pointer to device instance data
 * Return Value:    None
 */
static void z055_add_device( struct    Z055_STRUCT *info )
{
	info->next_device = NULL;
	info->line = G_z055_device_count;
	sprintf(info->device_name,"ttyTH%d",info->line);

	/* get parameters passed to driver */
	if (info->line < Z055_MAX_DEVICES) {
		if (maxframe[info->line]) {
			info->max_frame_size = maxframe[info->line];
			/* buffers have to be passed 32 bit aligned to hw */
			if( info->max_frame_size & 0x3 )
				info->max_frame_size = (info->max_frame_size & ~0x3 ) + 4;
		}

		if (num_rxbufs[info->line]) {
			info->rx_buffer_q.num_buffers = num_rxbufs[info->line];
			if (info->rx_buffer_q.num_buffers < 1)
				info->rx_buffer_q.num_buffers = 1;
			else if (info->rx_buffer_q.num_buffers > Z055_MAX_BUFFERS)
				info->rx_buffer_q.num_buffers = Z055_MAX_BUFFERS;
		}
	}

	G_z055_device_count++;

	if ( !G_z055_device_list    )
		G_z055_device_list =    info;
	else {
		struct Z055_STRUCT *current_dev = G_z055_device_list;
		while( current_dev->next_device )
			current_dev = current_dev->next_device;
		current_dev->next_device = info;
	}

	if ( info->max_frame_size < 0x200 )
		info->max_frame_size = 0x200;
	else if ( info->max_frame_size > HDLC_MAX_FRAME_SIZE )
		info->max_frame_size = HDLC_MAX_FRAME_SIZE;

	if ( info->bus_type == Z055_BUS_TYPE_PCI    ) {
			printk( "z055 HDLC %s: phys_base=%04X IRQ=%d MaxFrameSize=%u\n",
			info->device_name, info->phys_base, info->irq,
			info->max_frame_size );
	}

}   /* end of z055_add_device() */

/* z055_allocate_device()
 *
 *  Allocate and initialize a device instance structure
 *
 * Arguments:       none
 * Return Value:    pointer to Z055_STRUCT if success, otherwise    NULL
 */
struct Z055_STRUCT* z055_allocate_device()
{
	struct Z055_STRUCT *info;

	info = kmalloc(sizeof(struct Z055_STRUCT),GFP_KERNEL);

	if (!info) {
		printk("Error can't allocate device instance data\n");
	} else {
		memset(info, 0, sizeof(struct Z055_STRUCT));
		info->magic = Z055_HDLC_MAGIC;
		INIT_WORK(&info->task, z055_bh_handler);
		info->max_frame_size = HDLC_MAX_FRAME_SIZE;
		Z055_STRUCT_set_close_delay(info, 5*HZ/10);
		Z055_STRUCT_set_closing_wait(info,30*HZ);

		tty_port_init(&info->port);
		init_waitqueue_head(&info->status_event_wait_q);
		init_waitqueue_head(&info->event_wait_q);
		spin_lock_init(&info->irq_spinlock);
		memcpy(&info->params,&G_default_params,sizeof(Z055_PARAMS));
		info->idle_mode = Z055_IDLE_FLAG;
		info->rx_buffer_q.num_buffers = 5;
		info->ma_base = 0;
	}

	return info;

}   /* end of z055_allocate_device()*/

static struct tty_operations ops = {
	.install = z055_tty_install,
	.tiocmget = tiocmget,
	.tiocmset = tiocmset,
	.open = z055_open,
	.close = z055_close,
	.write = z055_write,
	.put_char = z055_put_char,
	.flush_chars = z055_flush_chars,
	.write_room = z055_write_room,
	.chars_in_buffer = z055_chars_in_buffer,
	.flush_buffer = z055_flush_buffer,
	.ioctl = z055_ioctl,
	.compat_ioctl = z055_compat_ioctl,
	.throttle = z055_throttle,
	.unthrottle = z055_unthrottle,
	.send_xchar = z055_send_xchar,
	.break_ctl = z055_break,
	.wait_until_sent = z055_wait_until_sent,
	.set_termios = z055_set_termios,
	.stop = z055_stop,
	.start = z055_start,
	.hangup = z055_hangup,
#if LINUX_VERSION_CODE < VERSION(4,18,0)
	.proc_fops = &z055_proc_fops,
#else
	.proc_show = z055_proc_show,
#endif
};

/*
 * perform tty device initialization
 */
int z055_init_tty(void);
int z055_init_tty()
{
	if ( debug_level & DEBUG_LEVEL_INFO )
		printk( "%s(%d)\n", __FUNCTION__, __LINE__   );

	G_serial_driver = tty_alloc_driver(G_z055_device_count, 0);
	if (!G_serial_driver)
		return -ENOMEM;
	G_serial_driver->owner = THIS_MODULE;

	G_serial_driver->driver_name = "men_lx_z055";
	G_serial_driver->name = "ttyTH";
	G_serial_driver->major = ttymajor;
	G_serial_driver->minor_start = 64;
	G_serial_driver->type = TTY_DRIVER_TYPE_SERIAL;
	G_serial_driver->subtype = SERIAL_TYPE_NORMAL;
	G_serial_driver->init_termios = tty_std_termios;
	G_serial_driver->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	G_serial_driver->init_termios.c_ispeed = 9600;
	G_serial_driver->init_termios.c_ospeed = 9600;
	G_serial_driver->flags = TTY_DRIVER_REAL_RAW;
	tty_set_operations(G_serial_driver, &ops);
	if (tty_register_driver(G_serial_driver) < 0)
		printk( "%s(%d): Couldn't register serial driver\n",
				__FUNCTION__, __LINE__);

	printk("%s %s, tty major#%d\n",
		G_driver_name, IdentString,
		G_serial_driver->major);

	return 0;
} /* end of z055_init_tty */


/* z055_init()
 *
 *  Driver initialization entry point.
 *
 * Arguments:   None
 * Return Value:    0 if success, otherwise error code
 */
int __init z055_init(void)
{
	int rc;

	if (debug_level & DEBUG_LEVEL_INFO)
		printk( "%s(%d)\n", __FUNCTION__, __LINE__ );

	printk("%s %s\n", G_driver_name, IdentString);

	if( !men_chameleon_register_driver( &G_driver ) )
		return -ENODEV;
	if ( !G_z055_device_list    ) {
		printk( "%s(%d): No Chameleon devices with z055 modules found.\n",
				__FUNCTION__, __LINE__);
		return -ENODEV;
	}
	if ((rc = z055_init_tty()))
		return rc;

	return 0;
}

static int __init z055_hdlc_init(void)
{
/* Uncomment this to kernel debug module.
 * z055_get_text_ptr() leaves the .text address in eax
 * which can be used with add-symbol-file with gdb.
 */
	if (break_on_load) {
		z055_get_text_ptr();
		BREAKPOINT();
	}

	return z055_init();
}

static void __exit z055_hdlc_exit(void)
{
	int rc;
	struct Z055_STRUCT *info;
	struct Z055_STRUCT *tmp;

	printk( "%s(%d): Unloading %s: %s\n",
			__FUNCTION__, __LINE__, G_driver_name,  IdentString);

	printk( "%s(%d): %s %s\n",
			__FUNCTION__, __LINE__,
			G_serial_driver->driver_name, G_serial_driver->name);

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,14,0)
	if ((rc = tty_unregister_driver(G_serial_driver)))
		printk( "%s(%d): failed to unregister tty driver err=%d\n",
				__FUNCTION__, __LINE__, rc);
#else
	tty_unregister_driver(G_serial_driver);
#endif

	printk("%s(%d)\n", __FUNCTION__, __LINE__);
	tty_driver_kref_put(G_serial_driver);
	printk("%s(%d)\n", __FUNCTION__, __LINE__);

	info = G_z055_device_list;
	while(info) {
		z055_release_resources(info);
		tmp = info;
		info = info->next_device;
		kfree(tmp);
	}
	printk("%s(%d)\n", __FUNCTION__, __LINE__);

	if (G_tmp_buf) {
		free_page((unsigned long) G_tmp_buf);
		G_tmp_buf = NULL;
	}
	printk("%s(%d)\n", __FUNCTION__, __LINE__);

	men_chameleon_unregister_driver( &G_driver );
	printk("%s(%d)\n", __FUNCTION__, __LINE__);


}

module_init(z055_hdlc_init);
module_exit(z055_hdlc_exit);


/* z055_hw_set_sdlc_mode()
 *
 *    Set up the adapter for SDLC communications.
 *
 * Arguments:       info    pointer to device instance data
 * Return Value:    NONE
 */
static void z055_hw_set_sdlc_mode( struct Z055_STRUCT *info )
{
	u16 RegValue;

	if ( debug_level & DEBUG_LEVEL_INFO )
		printk( "%s(%d): %s\n",
				__FUNCTION__, __LINE__, info->device_name);

	/* Coding Register (CDR) */
	switch ( info->params.encoding ) {
	case HDLC_ENCODING_NRZI:            RegValue = Z055_CDR_NRZI;       break;
	case HDLC_ENCODING_MANCHESTER:      RegValue = Z055_CDR_MAN;        break;
	case HDLC_ENCODING_MANCHESTER_NRZI: RegValue = Z055_CDR_NRZI_MAN;   break;
	case HDLC_ENCODING_NRZ_S:           RegValue = Z055_CDR_NRZS;       break;
	default:                            RegValue = Z055_CDR_NRZ;        break;
	}
	Z055_OUTREG( info, Z055_CDR, RegValue );

	/* HDLC Control Register (HCR) */
	RegValue = 0;
	if ( info->params.flags & HDLC_FLAG_ADDRCMP )
	{
		/* set up receive address filtering */
		Z055_OUTREG( info, Z055_ACR, (info->params.comp_broadc << 8) +
									 info->params.comp_addr);
		Z055_OUTREG( info, Z055_AMR, (info->params.addr_mask << 8) +
									 info->params.broadc_mask);
		RegValue |= Z055_HCR_ADDRCMPEN;
	}
	if ( info->params.flags & HDLC_FLAG_FOUR_START_FLAGS )
	{
		RegValue |= Z055_HCR_U4STRTFLGS;
	} else {
		RegValue &= ~Z055_HCR_U4STRTFLGS;
	}
	if ( info->params.crc_mode & HDLC_CRC_PRESET )
	{
		RegValue |= Z055_HCR_CRCPRS;
	}
	if ( info->params.crc_mode & HDLC_CRC_INV )
	{
		RegValue |= Z055_HCR_CRCINV;
	}
	if ( info->params.crc_mode & HDLC_CRC_CCITT )
	{
		RegValue |= Z055_HCR_CRCCCITT;
	}
	Z055_OUTREG( info, Z055_HCR, RegValue );


	/* Interrupt Enable Register (IER) */
	z055_hw_EnableInterrupts( info, Z055_IER_TXBOVREN +
									Z055_IER_TXBEPYEN +
									Z055_IER_RXABRTEN +
									Z055_IER_RXBFLEN   );

	/* Unlatch all Rx status bits and clear Rx status IRQ Pending */
	z055_hw_ClearIrqPendingBits( info,  Z055_IRQR_RXINV +
										Z055_IRQR_RXABRT +
										Z055_IRQR_RXFCSE +
										Z055_IRQR_RXRCST +
										Z055_IRQR_RXBOVR +
										Z055_IRQR_RXBFL +
										Z055_IRQR_TXBOVR +
										Z055_IRQR_TXBEPY );


	z055_hw_set_txidle( info );


	/* Baud rate Generator Control Register (BCR) */

	RegValue = 0;

	if ( info->params.flags & HDLC_FLAG_TXC_TRANSP ) {
		RegValue |= Z055_BCR_CLKTRANSEN;    /* Send Tx Clock on Interface */
		if( info->params.flags  & HDLC_FLAG_TXC_RTSPIN )
			RegValue |= Z055_BCR_CLKPIN;    /* Send Tx Clock on DTR pin */
			/* else send on DTR pin */
	}
	Z055_OUTREG( info, Z055_BCR, RegValue );

	/* enable Master Interrupt Enable bit (MIE) */
	z055_hw_EnableMasterIrqBit( info );

	z055_hw_stop_transmitter(info);
	z055_hw_stop_receiver(info);
}   /* end of z055_hw_set_sdlc_mode() */

/* z055_hw_enable_loopback()
 *
 * Set the z055 for internal loopback mode.
 * The TxCLK and RxCLK signals are generated from the BRG and
 * the TxD is looped back to the RxD internally.
 *
 * Arguments:       info    pointer to device instance data
 *                  enable  1 = enable loopback, 0 = disable
 * Return Value:    None
 */
static void z055_hw_enable_loopback(struct Z055_STRUCT *info, int enable)
{

	if ( debug_level & DEBUG_LEVEL_INFO )
		printk( "%s(%d): %s (en=%d)\n",
				__FUNCTION__, __LINE__, info->device_name, enable);

	if (enable) {
		/* Write 16-bit Time Constant for BRG0 */
		/* use baud_rate if available, otherwise use 9600 for diagnostics */
		if (info->params.baud_rate) {
			z055_hw_enable_brgen(info, info->params.baud_rate);
		} else
			z055_hw_enable_brgen(info, 9600);

		/* HDLC Control Register (HCR) */
		Z055_OUTREG(info, Z055_HCR, Z055_INREG(info, Z055_HCR) | Z055_HCR_LOOPB);

		/* set Internal Data loopback mode */
		info->loopback_bits = 0x300;
	} else {
		/* enable external TXD output */
		/* HDLC Control Register (HCR) */
		Z055_OUTREG(info, Z055_HCR, Z055_INREG(info,Z055_HCR) & ~Z055_HCR_LOOPB);
		/* clear Internal Data loopback mode */
		info->loopback_bits = 0;
	}
}   /* end of z055_hw_enable_loopback() */

/* z055_hw_enable_brgen()
 *
 * Enable the BR Generator clock output at the specified frequency.
 *
 * Arguments:
 *
 *  info        pointer to device extension
 *  data_rate   data rate of clock in bits per second
 *
 * Return Value:    None
 */
static void z055_hw_enable_brgen( struct Z055_STRUCT *info, u32 data_rate )
{
	u32 XtalSpeed;
	u32 Tc;

	if ( debug_level & DEBUG_LEVEL_INFO )
		printk( "%s(%d): %s data_rate = %d\n",
				__FUNCTION__, __LINE__, info->device_name, data_rate);

	if ( data_rate ) {
		XtalSpeed = SYSTEM_CLOCK_FREQUENCY;


		/* Tc = (Xtal/Speed) - 1 */
		/* If twice the remainder of (Xtal/Speed) is greater than Speed */
		/* then rounding up gives a more precise time constant. Instead */
		/* of rounding up and then subtracting 1 we just don't subtract */
		/* the one in this case. */

		Tc = (u32)(XtalSpeed/data_rate);
		if ( !(((XtalSpeed % data_rate)  * 2) / data_rate) )
			Tc--;

		/* Write 2x 16-bit  Time Constant for BRG */
		Z055_OUTREG( info, Z055_BCL, (u16)( Tc & 0x0000FFFF) );
		Z055_OUTREG( info, Z055_BCH, (u16)((Tc & 0xFFFF0000) >> 16) );

		/* Baud Rate Generator Control Register (BCR) */
		Z055_OUTREG(info, Z055_BCR, Z055_INREG(info, Z055_BCR) | Z055_BCR_BRGEN);

	} else {
		/* data rate == 0 so turn off BRG */
		Z055_OUTREG(info, Z055_BCR, Z055_INREG(info,Z055_BCR) & ~Z055_BCR_BRGEN);
	}

}   /* end of z055_hw_enable_brgen()    */

/* z055_hw_stop_receiver()
 *
 *  Disable receiver
 *
 * Arguments:       info    pointer to device instance data
 * Return Value:    None
 */
static void z055_hw_stop_receiver( struct Z055_STRUCT *info )
{
	if (debug_level & DEBUG_LEVEL_ISR)
		printk("%s(%d): %s\n", __FUNCTION__, __LINE__, info->device_name );

	z055_hw_ClearIrqPendingBits( info,  Z055_IRQR_RXINV +
										Z055_IRQR_RXABRT +
										Z055_IRQR_RXFCSE +
										Z055_IRQR_RXRCST +
										Z055_IRQR_RXBOVR +
										Z055_IRQR_RXBFL );
	z055_hw_DisableInterrupts( info, Z055_IER_RXINVEN   +
									 Z055_IER_RXABRTEN +
									 Z055_IER_RXFCSEEN +
									 Z055_IER_RXRCSTEN +
									 Z055_IER_RXBOVREN +
									 Z055_IER_RXBFLEN   );
	z055_hw_EnableReceiver( info, DISABLE_UNCONDITIONAL );

	info->rx_enabled = 0;
	info->rx_overflow = 0;
	info->rx_rcc_underrun = 0;

}   /* end of z055_hw_stop_receiver() */

/* z055_hw_start_receiver()
 *
 *  Enable the receiver
 *
 * Arguments:       info    pointer to device instance data
 * Return Value:    None
 */
static void z055_hw_start_receiver( struct Z055_STRUCT *info    )
{
	if (debug_level & DEBUG_LEVEL_ISR)
		printk( "%s(%d): %s\n",
				__FUNCTION__, __LINE__, info->device_name );

	z055_reset_rx_buffers( info );
	z055_hw_stop_receiver( info );

	if ( info->params.mode == Z055_MODE_HDLC    ) {

		z055_hw_ClearIrqPendingBits( info,  Z055_IRQR_RXINV  +
											Z055_IRQR_RXABRT +
											Z055_IRQR_RXFCSE +
											Z055_IRQR_RXRCST +
											Z055_IRQR_RXBOVR +
											Z055_IRQR_RXBFL );
		z055_hw_EnableInterrupts( info, Z055_IER_RXINVEN  +
										Z055_IER_RXABRTEN +
										Z055_IER_RXFCSEEN +
//										Z055_IER_RXRCSTEN +
										Z055_IER_RXBOVREN +
										Z055_IER_RXBFLEN );

		z055_hw_EnableReceiver( info, ENABLE_UNCONDITIONAL );
	}

	info->rx_enabled = 1;

}   /* end of z055_hw_start_receiver() */

/* z055_hw_start_transmitter()
 *
 *  Enable the transmitter and send a transmit frame if
 *  one is loaded in the DMA buffers.
 *
 * Arguments:       info    pointer to device instance data
 * Return Value:    None
 */
static void z055_hw_start_transmitter( struct Z055_STRUCT *info )
{
	if (debug_level & DEBUG_LEVEL_ISR)
		printk( "%s(%d): %s\n",
				__FUNCTION__, __LINE__, info->device_name );

	if ( info->xmit_cnt ) {

		/* If auto RTS enabled and RTS is inactive, then assert */
		/* RTS and set a flag indicating that the driver should */
		/* negate RTS when the transmission completes. */

		info->drop_rts_on_tx_done = 0;

		z055_hw_ClearIrqPendingBits( info,  Z055_IRQR_TXBOVR +
											Z055_IRQR_TXBEPY );
		z055_hw_EnableInterrupts( info, Z055_IER_TXBOVREN +
										Z055_IER_TXBEPYEN );

		z055_hw_EnableTransmitter(info,ENABLE_UNCONDITIONAL);

		info->tx_active = 1;

		info->tx_timer.expires = jiffies + jiffies_from_ms(5000);
		add_timer(&info->tx_timer);

	}
}   /* end of z055_hw_start_transmitter() */

/* z055_hw_stop_transmitter()
 *
 *  Stops the transmitter and DMA
 *
 * Arguments:       info    pointer to device isntance data
 * Return Value:    None
 */
static void z055_hw_stop_transmitter( struct Z055_STRUCT   *info )
{
	if (debug_level & DEBUG_LEVEL_ISR)
		printk( "%s(%d): %s\n",
				__FUNCTION__, __LINE__,  info->device_name );

	del_timer(&info->tx_timer);

	z055_hw_ClearIrqPendingBits( info,  Z055_IRQR_TXBOVR +
										Z055_IRQR_TXBEPY );
	z055_hw_DisableInterrupts( info, Z055_IER_TXBOVREN +
									 Z055_IER_TXBEPYEN );

	z055_hw_EnableTransmitter( info, DISABLE_UNCONDITIONAL );

	info->tx_enabled = 0;
	info->tx_active  = 0;

}   /* end of z055_hw_stop_transmitter()    */

/* z055_hw_reset()
 *
 *  Reset the adapter to a known state and prepare it for further use.
 *
 * Arguments:       info    pointer to device instance data
 * Return Value:    None
*/
static void z055_hw_reset( struct Z055_STRUCT *info )
{
	if ( info->bus_type == Z055_BUS_TYPE_PCI    ) {
		int i;
		u32 readval;

		/* Set BIT31 of HDLC Control Register, is reset by FPGA */

		Z055_OUTREG( info, Z055_HCR, Z055_HCR_SWRST );

		/*
		 * wait a few ns to be safe
		 */
		for(i=0;i<10;i++)
			readval = Z055_INREG( info, Z055_ACR );

	}

	info->mbre_bit = 0;
	info->loopback_bits = 0;
	info->z055_hw_idle_mode = HDLC_FLAG_TXIDLE_FLAGS;
	info->hcr_value = 0;
	info->cmr_value = 0;

	/* set some registers here */

}   /* end of z055_hw_reset() */

/* z055_hw_set_sync_mode()  Programs the Z055 for SDLC communications.
 *
 * Arguments:       info    pointer to adapter info structure
 * Return Value:    None
 */
static void z055_hw_set_sync_mode( struct Z055_STRUCT *info )
{
	z055_hw_set_sdlc_mode( info );

	z055_hw_enable_brgen(info, info->params.baud_rate);

	if (Z055_STRUCT_flags(info) & HDLC_FLAG_HDLC_LOOPBACK)
		z055_hw_enable_loopback(info,1);

}   /* end of z055_set_sync_mode() */

/* z055_hw_set_txidle() Set the HDLC idle mode for the transmitter.
 *
 * Arguments:       info    pointer to device instance data
 * Return Value:    None
 */
static void z055_hw_set_txidle(    struct Z055_STRUCT *info    )
{
	u16 hw_idle_mode =  Z055_HCR_TXIDLE;

	/* Map API idle mode to z055 register bits */

	switch( info->idle_mode ){
	case HDLC_FLAG_TXIDLE_FLAGS:
		hw_idle_mode =  0;
		break;
	case HDLC_FLAG_TXIDLE_MARK:
		hw_idle_mode =  Z055_HCR_TXIDLE;
		break;
	}

	Z055_OUTREG( info, Z055_HCR,
				 (Z055_INREG(info, Z055_HCR) & ~Z055_HCR_TXIDLE) | hw_idle_mode);
}   /* end of z055_hw_set_txidle() */

/* z055_hw_get_serial_signals()
 *
 *  Query the adapter for the state of the V24 status (input) signals.
 *
 * Arguments:       info    pointer to device instance data
 * Return Value:    None
 */
static void z055_hw_get_serial_signals( struct Z055_STRUCT *info )
{
	u16 status;

	/* clear all serial signals except DTR and RTS */
	info->serial_signals &= SerialSignal_DTR + SerialSignal_RTS;

	/* Read the Handshake Status register (Z055_HSSR) to get */
	/* the V24 status signals. */

	status = Z055_INREG( info, Z055_HSSR );

	/* set serial signal bits to reflect Z055_HSSR */

	if ( status & Z055_HSSR_CTS )
		info->serial_signals |= SerialSignal_CTS;

	if ( status & Z055_HSSR_DSR )
		info->serial_signals |= SerialSignal_DSR;

}   /* end of z055_hw_get_serial_signals() */

/* z055_hw_set_serial_signals()
 *
 *  Set the state of DTR and RTS based on contents of
 *  serial_signals member of device extension.
 *
 * Arguments:       info    pointer to device instance data
 * Return Value:    None
 */
static void z055_hw_set_serial_signals(    struct Z055_STRUCT *info    )
{
	u16 Control;
	unsigned char V24Out = info->serial_signals;

	/* get the current value of the Handshake Control register (HSCR) */

	Control = Z055_INREG( info, Z055_HSCR );

	if ( V24Out & SerialSignal_RTS )
		Control |= Z055_HSCR_RTS;
	else
		Control &= ~(Z055_HSCR_RTS);

	if ( V24Out & SerialSignal_DTR )
		Control |= Z055_HSCR_DTR;
	else
		Control &= ~(Z055_HSCR_DTR);

	if ( info->params.half_duplex == HDLC_HALF_DUPLEX )
		Control |= Z055_HSCR_HDX;
	else
		Control &= ~(Z055_HSCR_HDX);

	Z055_OUTREG( info, Z055_HSCR, Control );

}   /* end of z055_hw_set_serial_signals() */

/*
 * z055_reset_rx_buffers()
 *
 *  Set the count for all receive buffers to 0
 *  and set the current buffer to the first buffer. This effectively
 *  makes all buffers free and discards any data in buffers.
 *
 * Arguments:       info    pointer to device instance data
 * Return Value:    None
 */
static void z055_reset_rx_buffers( struct Z055_STRUCT *info )
{
	unsigned int i;

	for ( i = 0; i < info->rx_buffer_q.num_buffers; i++ ) {
		info->rx_buffer_q.get_buffer->ccount = 0;
		info->rx_buffer_q.get_buffer->status = 0;
		info->rx_buffer_q.get_buffer = info->rx_buffer_q.get_buffer->next;
	}

	info->rx_buffer_q.buffers_used = 0;
	info->rx_buffer_q.get_buffer = info->rx_buffer_q.put_buffer;

}   /* end of z055_reset_rx_buffers() */

/* z055_load_tx_buffer()
 *
 *  Load the adapters transmit buffer with the specified data.
 *
 * Arguments:
 *
 *  info        pointer to device extension
 *  Buffer      pointer to buffer containing frame to load
 *  BufferSize  size in bytes of frame in Buffer
 *
 * Return Value:    None
 */
static void z055_load_tx_buffer(struct Z055_STRUCT *info,
						 const char *buffer,
	 					 unsigned int bufferSize)
{
	unsigned short copyCount = 0;
	if ( debug_level & DEBUG_LEVEL_DATA ){
		printk( "%s(%d): %s: size=0x%04x;\n",
				__FUNCTION__, __LINE__,  info->device_name , bufferSize);
		z055_trace_block(info, buffer, MIN(bufferSize+2,info->max_frame_size), 1);
	}

	/* begin loading the frame in the adapters tx buffer */
	/* Copy frame data from source buffer to the adapters buffers. */
	/* always copy full 32bit words. */
 	if( bufferSize & 0x3 )
		copyCount = ( bufferSize & ~0x3 ) + 4;
	else
		copyCount = bufferSize;

	Z055_OUTTX(info, 0, copyCount, buffer);

	/* set TXSZR register */
	Z055_OUTREG( info, Z055_TXSZR, bufferSize );

}   /* end of z055_load_tx_buffer() */

/*
 * z055_register_test()
 *
 *  Performs a register test of the 16C32.
 *
 * Arguments:       info    pointer to device instance data
 * Return Value:        TRUE if test passed, otherwise FALSE
 */
static BOOLEAN z055_register_test( struct Z055_STRUCT *info )
{
	static unsigned short BitPatterns[] =
		{0x0000, 0xffff, 0xaaaa, 0x5555, 0x1234, 0x6969, 0x9696, 0x0f0f, 0x0000};
	static unsigned int Patterncount = sizeof(BitPatterns)/sizeof(unsigned short);
	unsigned int i;
	BOOLEAN rc = TRUE;
	unsigned long flags;

	spin_lock_irqsave(&info->irq_spinlock,flags);
	z055_hw_reset(info);

	/* Verify the reset state of some registers. */

	if ( (Z055_INREG( info, Z055_HCR ) != 0) ||
		 (Z055_INREG( info, Z055_IER ) != 0) ||
		 (Z055_INREG( info, Z055_ACR ) != 0) ||
		 (Z055_INREG( info, Z055_AMR ) != 0)   ){
		rc = FALSE;
	}

	if ( rc == TRUE ){
		/* Write bit patterns to various registers but do it out of */
		/* sync, then read back and verify values. */

		for ( i = 0 ; i < Patterncount ; i++ ) {
			Z055_OUTREG(    info, Z055_BCL,     BitPatterns[i] );
			Z055_OUTREG(    info, Z055_BCH,     BitPatterns[(i+1)%Patterncount] );
			Z055_OUTREG(    info, Z055_TXSZR,   BitPatterns[(i+2)%Patterncount] );

			if ( (Z055_INREG( info, Z055_BCL )   != BitPatterns[i]) ||
				 (Z055_INREG( info, Z055_BCH )   != BitPatterns[(i+1)%Patterncount]) ||
				 (Z055_INREG( info, Z055_TXSZR ) != BitPatterns[(i+2)%Patterncount])   ){
				rc = FALSE;
				break;
			}
		}
	}

	z055_hw_reset(info);
	spin_unlock_irqrestore(&info->irq_spinlock,flags);

	return rc;
}   /* end of z055_register_test() */

/* z055_adapter_test()
 *
 *  Perform the register test.
 *
 * Arguments:       info    pointer to device instance data
 * Return Value:    0 if success, otherwise -ENODEV
 */
static int z055_adapter_test( struct Z055_STRUCT   *info )
{
	if ( debug_level & DEBUG_LEVEL_INFO )
		printk( "%s(%d):Testing device %s\n",
				__FUNCTION__, __LINE__, info->device_name );


	if ( !z055_register_test( info )    ) {
		info->init_error = DiagStatus_AddressFailure;
		printk( "%s(%d): Register test failure for device %s Addr=0x%08X\n",
				__FUNCTION__, __LINE__, info->device_name,
				info->ma_base + info->ma_offs );
		return -ENODEV;
	}

	if ( debug_level & DEBUG_LEVEL_INFO )
		printk( "%s(%d):device %s passed diagnostics\n",
				__FUNCTION__, __LINE__, info->device_name );

	return 0;

}   /* end of z055_adapter_test() */

static void z055_trace_block(struct Z055_STRUCT *info, const char* data, int count, int xmit)
{
	int i;
	int linecount;
	if (xmit)
		printk("%s tx data:\n",info->device_name);
	else
		printk("%s rx data:\n",info->device_name);

	while(count) {
		if (count > 16)
			linecount = 16;
		else
			linecount = count;

		for(i=0;i<linecount;i++)
			printk("%02X ",(unsigned char)data[i]);
		for(;i<17;i++)
		printk("   ");
		for(i=0;i<linecount;i++) {
			if (data[i]>=040 && data[i]<=0176)
				printk("%c",data[i]);
			else
				printk(".");
		}
		printk("\n");

		data  += linecount;
		count -= linecount;
	}
}   /* end of z055_trace_block()    */

/* z055_tx_timeout()
 *
 *  called when HDLC frame times out
 *  update stats and do tx completion processing
 *
 * Arguments:   context     pointer to device instance data
 * Return Value:    None
 */

#if LINUX_VERSION_CODE < VERSION(4,15,0)
static void z055_tx_timeout(unsigned long context)
{
	struct Z055_STRUCT *info = (struct Z055_STRUCT*)context;
#else
static void z055_tx_timeout(struct timer_list *t)
{
	struct Z055_STRUCT *info = from_timer(info, t, tx_timer);
#endif
	unsigned long flags;

	if ( debug_level & DEBUG_LEVEL_INFO )
		printk( "%s(%d): %s\n", __FUNCTION__, __LINE__, info->device_name);
	if(info->tx_active &&
	   (info->params.mode == Z055_MODE_HDLC) ) {
		info->icount.txtimeout++;
	}
	spin_lock_irqsave(&info->irq_spinlock,flags);
	info->tx_active = 0;
	info->xmit_cnt = info->xmit_head = info->xmit_tail = 0;

	spin_unlock_irqrestore(&info->irq_spinlock,flags);

	z055_bh_transmit(info);

}   /* end of z055_tx_timeout() */

static int z055_init_one (CHAMELEON_UNIT_T *chu)
{
	struct Z055_STRUCT *info;
	int ioMapped = 0;
	if (debug_level & DEBUG_LEVEL_INFO)
		printk( "%s(%d)\n", __FUNCTION__, __LINE__ );

	if (!(info = z055_allocate_device())) {
		printk("can't allocate device instance data.\n");
		return -EIO;
	}

	chu->driver_data = info;

	/* Copy user configuration info to device instance data */

	info->phys_base = (U_INT32_OR_64)(chu->phys);
	info->irq = chu->pdev->irq;

	info->bus_type = Z055_BUS_TYPE_PCI;
	info->phys_addr_size = Z055_ADDR_SIZE;
	info->irq_flags = IRQF_SHARED;

	/*--- are we io-mapped ? ---*/
	ioMapped = pci_resource_flags( chu->pdev, chu->bar ) & IORESOURCE_IO;

#ifdef MAC_MEM_MAPPED
	if( ioMapped ) {
		printk( "*** ERROR *** BAR is IO mapped but driver is compiled"
				" for mem mapped access\n");
		return -ENODEV;
	}
	/* Because veremap only works on page boundaries we must map
	 * a larger area than is actually implemented for the LCR
	 * memory range. We map a full page starting at the page boundary.
	 */
	info->ma_offs = info->phys_base & (PAGE_SIZE-1);
	info->phys_base &= ~(PAGE_SIZE-1);
	info->phys_addr_size = ((Z055_ADDR_SIZE / PAGE_SIZE)) * PAGE_SIZE;

	/* in case the size is not multiple PAGE_SIZE get sufficient space */
	if( 0 != (Z055_ADDR_SIZE % PAGE_SIZE) ) {
		info->phys_addr_size += PAGE_SIZE;
	}

#else
	if( !ioMapped ) {
		printk( "*** ERROR *** BAR is mem mapped but driver is compiled"
				" for IO mapped access\n");
		return -ENODEV;
	}
	info->ma_offs = 0;
#endif

	z055_add_device(info);

	return 0;
}

static int z055_remove_one (CHAMELEON_UNIT_T *chu)
{
	return 0;
}

