/****************************************************************************
 ************                                                    ************
 ************                 Z055_HDLC_UTIL                     ************
 ************                                                    ************
 ****************************************************************************/
/*!
 *         \file z055_hdlc_util.c
 *
 *       \author Christian.Schuster@men.de
 *
 *       \brief  Tool for setting up Z055_HDLC hardware and driver
 *               The program parses the command line options and issues the
 *               necessary IOCTL call(s) to the driver.
 *
 *     \switches (none)
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


#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <errno.h>
#include <stdio.h>
#include <memory.h>

#include <MEN/men_typs.h>
#include "MEN/z055_hdlc.h"

/*
 * Handler function prototypes for individual options
 */

int set_hdlc(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int set_loopback(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int clr_loopback(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int display_stats(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int set_txctransp(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int clr_txctransp(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int set_txcrtspin(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int set_txcdtrpin(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int set_crc16(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int set_crcccitt(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int set_crcinv(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int clr_crcinv(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int set_crcpreset(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int set_idleflag(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int set_idlemark(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int set_nrz(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int set_nrzi(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int set_nrz_s(int argc, char* argv[], char* devname, Z055_PARAMS* params);
/* int set_fm0(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int set_fm1(int argc, char* argv[], char* devname, Z055_PARAMS* params);*/
int set_manch(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int set_manchnrzi(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int set_addrcomp(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int clr_addrcomp(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int set_compaddr(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int set_compbroadc(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int set_addrmask(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int set_broadcmask(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int set_baudrate(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int set_four_start_flags(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int clr_four_start_flags(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int set_half_duplex(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int clr_half_duplex(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int set_quiet(int argc, char* argv[], char* devname, Z055_PARAMS* params);

typedef int (*CMD_SET_FUNC)(int argc, char *argv[], char* devname, Z055_PARAMS* params);
typedef struct _CMD_TABLE_ENTRY {
	char* cmd;
	CMD_SET_FUNC func;
} CMD_TABLE_ENTRY;

CMD_TABLE_ENTRY cmd_table[] = {
	{"stats",display_stats},
	{"baudrate",set_baudrate},
	{"hdlc", set_hdlc},

	{"loopback",set_loopback},
	{"+loopback",set_loopback},
	{"-loopback",clr_loopback},

	{"txctransp",set_txctransp},
	{"+txctransp",set_txctransp},
	{"-txctransp",clr_txctransp},
	{"txcrtspin",set_txcrtspin},
	{"txcdtrpin",set_txcdtrpin},

	{"crc16",set_crc16},
	{"crcccitt",set_crcccitt},
	{"crcinv",set_crcinv},
	{"+crcinv",set_crcinv},
	{"-crcinv",clr_crcinv},
	{"crcpreset",set_crcpreset},

	{"idlemark",set_idlemark},
	{"idleflag",set_idleflag},
	{"nrz",set_nrz},
	{"nrzi",set_nrzi},
	{"nrz-s",set_nrz_s},
/*	{"fm0",set_fm0},
	{"fm1",set_fm1},
*/
	{"manch",set_manch},
	{"manch+nrzi",set_manchnrzi},

	{"addrcomp",set_addrcomp},
	{"+addrcomp",set_addrcomp},
	{"-addrcomp",clr_addrcomp},
	{"compaddr",set_compaddr},
	{"compbroadc",set_compbroadc},
	{"addrmask",set_addrmask},
	{"broadcmask",set_broadcmask},

	{"+fourflags",set_four_start_flags},
	{"-fourflags",clr_four_start_flags},

	{"+halfduplex",set_half_duplex},
	{"-halfduplex",clr_half_duplex},

	{"quiet",set_quiet},
};

#define CMD_TABLE_COUNT (sizeof(cmd_table)/sizeof(CMD_TABLE_ENTRY))

int parse_cmdline(int argc, char* argv[], char* devname, Z055_PARAMS* params);
int get_params(char *devname, Z055_PARAMS* params);
int apply_params(char *devname, Z055_PARAMS* params);
void display_params(char *devname, Z055_PARAMS* params);
void display_params_uart(Z055_PARAMS* params);
void display_params_hdlc(Z055_PARAMS* params);
void display_params_comm(Z055_PARAMS* params);
void display_usage(char *progName);
int quiet=0;

/* main()
 *
 * 	program entry point
 *
 * Arguments:
 *
 * 	argc	count of command line arguments
 * 	argv	array of pointers to command line arguments
*
 * Return Value:
 *
 * 	0 if success, otherwise error code
 */
int main(int argc, char* argv[])
{
	char devname[100];
	Z055_PARAMS params;
	int rc;
	char *progName;

	progName = argv[0];
	if ( argc == 1 ||
		 !strcmp(argv[1],"--help") ||
		 !strcmp(argv[1],"-?") ||
		 !strcmp(argv[1],"-h") ) {
		display_usage(progName);
	exit(0);
	}

	sprintf(devname,argv[1]);

	rc = get_params(devname,&params);
	if (rc<0)
		exit(rc);

	/* skip program- and device name */
	argc -= 2;
	argv += 2;

	rc = parse_cmdline(argc,argv,devname,&params);

	if (!quiet && !rc) {
		display_params(devname,&params);
	}

	if (!rc && argc) {
		rc = apply_params(devname,&params);
	} else
		printf( "Nothing set, returned premature (rc = %x, argc = %d)\n",
				rc, argc);

	return rc;

}	/* end of main() */

/* parse_cmdline()
 *
 * 	parse command line arguments into a device name
 * 	and a device parameters structure
 *
 * Arguments:
 *
 * 	argc		number of command line arguments
 * 	argv		array of pointers to command line arguments
 * 	devname		buffer to hold parsed device name
 * 	params		buffer to hold parsed device parameters structure
 *
 * Return Value:
 *
 *	0 if success, otherwise error code
 */
int parse_cmdline(int argc, char* argv[], char* devname,
	Z055_PARAMS* params)
{
	int rc=0;
	int i;

	while(argc) {
		for(i=0;i<CMD_TABLE_COUNT;i++) {
			if (!strcmp(cmd_table[i].cmd,*argv)) {
				rc =(*cmd_table[i].func)(argc,argv,devname,params);
				if (rc<0) {
					printf( "returned with error(0x%x) at argv= %s\n",
							-rc, argv[0]);
					return rc;
				}
				break;
			}
		}
		if (i==CMD_TABLE_COUNT) {
			printf("\nInvalid option %s\n",*argv);
			exit(-EINVAL);
		}
		argc -= rc;
		argv += rc;
	}

	return 0;

}	/* end of parse_cmdline() */

/* display_usage()
 *
 * 	output program usage to stdout
 *
 * Arguments:
 *
 * 	progName	name of this program
 *
 * Return Value:	None
 */
void display_usage(char *progName)
{
	printf( "\n%s, command line utility to view and alter\n"
		"device parameters for a Z055 hdlc fpga module.\n"
		"usage: %s devicename [options]\n"

		"options with [+/-] may be prepended with a\n"
		"+ or - character to enable or disable the option\n"
		"\n-== COMMON OPTIONS ==- \n"
		"stats             display device statistics\n"
		"baudrate <const>  set baud rate generator constant to const\n"
		"hdlc              set mode to bit synchronous HDLC (default)\n"
		"[+/-]halfduplex   en-/disable half duplex mode\n"
		"[+/-]loopback     set/clear internal loopback mode\n"
		"[+/-]txctransp    en-/disable transportation of Tx clock on interface\n"
		"txcrtspin         set Tx clock to be sent on RTS pin\n"
		"txcdtrpin         set Tx clock to be sent on DTR pin\n"
		"crc16             append/check CRC-16 on HDLC frames\n"
		"crcccitt          append/check CRC-CCITT on HDLC frames\n"
		"[+/-]crcinv       en-/disable inversion of crc word before transmit/check\n"
		"crcpreset <0/1>   preset value for crc calculation (0x00/0xFF)\n"
		"idlemark          send \"ones\" between frames\n"
		"idleflag          send \"flags\" between frames\n"
		"[+/-]fourflags    en-/disable sending of four start flags\n"
		"nrz               set NRZ encoding algorithm\n"
		"nrzi              set NRZI/NRZ-M encoding algorithm\n"
		"nrz-s             set NRZ-S encoding algorithm\n"
#if 0
		"fm0               set FM1 encoding algorithm\n"
		"fm1               set FM0 encoding algorithm\n"
#endif
		"manch             set Manchester encoding algorithm\n"
		"manch+nrzi        set Manchester and NRZI/NRZ-M combined encoding algorithm\n"
		"[+/-]addrcomp     enable/disable address search mode\n"
		"compaddr <addr>   address for HDLC frame compare, hex value (0xFF)\n"
		"compbroadc <addr> address for HDLC broadcast frame compare, hex value (0xFF)\n"
		"addrmask <addr>   mask for HDLC frame address compare, hex value (0xFF)\n"
		"broadcmask <addr> mask for HDLC broadcast frame address compare, hex value (0xFF)\n"
		"quiet             set quiet mode, do not display any information\n",
		 progName, progName);


}	/* end of display_usage() */

/* display_params()
 *
 * 	display the specified device parameters to stdout
 *
 * Arguments:
 *
* 	devname		device name
 * 	params		pointer to parameters structure
 *
 * Return Value:
 *
 * 	None
 */
void display_params(char *devname, Z055_PARAMS* params)
{
	char *str,*str2;
	//static char unknown_str[] = "unknown";
	//static char enabled_str[] = "enabled";
	//static char disabled_str[] = "disabled";

	printf("*** %s Settings ***\n", devname);

	printf("    baudrate = %d\n", params->baud_rate);

	switch(params->mode) {
	case Z055_MODE_HDLC: 	str ="HDLC\n"; break;
	default: 				str = "unknown"; break;
	}
	printf("    Tx/Rx mode = %s", str);

	if(	params->flags & HDLC_FLAG_TXC_TRANSP ) {
		str = "";
		if( params->flags & HDLC_FLAG_TXC_DTRPIN )
			str2 = " on DTR pin";
		else
			str2 = " on RTS pin";
	} else {
		str = " not";
		str2 = "";
	}
	printf( "    Tx clock is%s sent%s\n", str, str2);

	printf( "    send %s between frames\n",
			(params->flags & HDLC_FLAG_TXIDLE_MARK) ? "ones" : "flags" );

	if( params->flags & HDLC_FLAG_FOUR_START_FLAGS )
		printf("    sending four start flags\n");
	else
		printf("    sending one start flag\n");

	if( params->flags & HDLC_FLAG_ADDRCMP )
		printf( "    HDLC frame address and broadcast frame address are compared\n"
				"         frame compare address           = 0x%02x\n"
				"         broadcast frame compare address = 0x%02x\n"
				"         frame compare mask              = 0x%02x\n"
				"         broadcast frame compare mask    = 0x%02x\n",
				params->comp_addr,
				params->comp_broadc,
				params->addr_mask,
				params->broadc_mask);
	else
		printf("    HDLC frame addresses are not validated\n");

	switch(params->encoding) {
	case HDLC_ENCODING_NRZ:				str = "NRZ"; break;
	case HDLC_ENCODING_NRZI:			str = "NRZI"; break;
#if 0
	case HDLC_ENCODING_FM1:				str = "FM1"; break;
	case HDLC_ENCODING_FM0:				str = "FM0"; break;
#endif
	case HDLC_ENCODING_MANCHESTER:		str = "Manchester"; break;
	case HDLC_ENCODING_MANCHESTER_NRZI:	str = "Manchester plus NRZI"; break;
	case HDLC_ENCODING_NRZ_S:		str = "NRZ-S"; break;
	default:							str = "unknown";
	}
	printf("    encoding = %s\n",str);

	if( params->crc_mode & HDLC_CRC_CCITT )
		str = "CRC-CCITT";
	else
		str = "CRC-16";
	if( params->crc_mode & HDLC_CRC_INV )
		str2 = "";
	else
		str2 = "not ";
	printf("    CRC is computed based on %s polynome, preset = %d, %sinverted\n",
			str,
			params->crc_mode & HDLC_CRC_PRESET ? 1 : 0,
			str2);

	fflush(0);

}	/* end of display_params() */


/* get_params()
 *
 * 	get the current device parameters for the specified device
 *
 * Arguments:
 *
 * 	devname		device name
 * 	params		pointer to parameters structure
 *
 * Return Value:
 *
 * 	0 if success, otherwise error code
 */
int get_params(char *devname, Z055_PARAMS* params)
{
	int fd,rc;
	static char funcname[] = "get_params()";

	/* open the specified device */
	fd = open(devname,O_NONBLOCK,0);
	if (fd < 0) {
		printf("%s(%d):%s open on device %s failed with err=%d %s\n",
			__FILE__,__LINE__,funcname,devname,errno, strerror(errno));
		return fd;
	}

	/* make ioctl call to get current parameters */
	rc = ioctl(fd, Z055_IOCGPARAMS, params);
	if (rc < 0) {
		printf("%s(%d):%s ioctl(Z055_IOCGPARAMS) on device %s"
			" failed with err=%d %s\n",
			__FILE__,__LINE__,funcname,devname,errno, strerror(errno));
		return rc;
	}


	close(fd);
	return 0;

}	/* end of get_params() */

/* apply_params()
 *
 * 	apply a device parameters structure to the specified device
 *
 * Arguments:
 *
 * 	devname		device name
 * 	params		pointer to parameters structure
 *
 * Return Value:
 *
 * 	0 if success, otherwise error code
 */
int apply_params(char *devname, Z055_PARAMS* params)
{
	int fd,rc;
	struct ifreq ifr;
	int sock;
	int devnum;

	/* open the specified device */
	fd = open(devname,O_NONBLOCK,0);
	if (fd < 0) {
		printf("%s(%d):%s open on device %s failed with err=%d %s\n",
			__FILE__,__LINE__,__FUNCTION__,devname,errno, strerror(errno));
		return fd;
	}

	/* make ioctl call to set current parameters */
	rc = ioctl(fd,Z055_IOCSPARAMS,params);
	if (rc < 0) {
		printf("%s(%d):%s ioctl(Z055_IOCGPARAMS) on device %s"
			" failed with err=%d %s\n",
			__FILE__,__LINE__,__FUNCTION__,devname,errno, strerror(errno));
		return rc;
	}

//	sleep(50);
	close(fd);

	if ((sock = socket(PF_INET,SOCK_DGRAM,0)) < 0) {
		perror("Cant open socket\n");
		return 0;
	}
	do {
		if (sscanf(devname,"/dev/ttyTH%d",&devnum) == 1 ) {
			sprintf(ifr.ifr_ifrn.ifrn_name, "z055%d", devnum);
			break;
		} else {
			perror("Cant parse device number\n");
			return 0;
		}
	} while ( 0 );

	return 0;

}	/* end of apply_params() */

/* /\* */
/*  * Handler functions for individual options */
/*  *\/ */
int display_stats(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	int fd,rc;
	static char funcname[] = "display_stats()";
	struct Z055_ICOUNT icount;

	/* open the specified device */
	fd = open(devname,0,0);
	if (fd < 0) {
		printf("%s(%d):%s open on device %s failed with err=%d %s\n",
			__FILE__,__LINE__,funcname,devname,errno, strerror(errno));
		return 1;
	}

	/* make ioctl call to get current parameters */
	rc = ioctl(fd,Z055_IOCGSTATS,&icount);
	if (rc < 0) {
		printf("%s(%d):%s ioctl(Z055_IOCGSTATS) on device %s"
			" failed with err=%d %s\n",
			__FILE__, __LINE__, funcname, devname, errno, strerror(errno));
		return 1;
	}

	/* close device */
	close(fd);

	printf( "\n*** %s Statistics ***\n"
		"irqs: DSR:%d CTS:%d\n"
		"hdlc/raw frame stats:\n"
		"    txok=%d txbovr=%d txtimeout=%d\n"
		"    rxok=%d rxlong=%d rxrcst=%d rxabort=%d rxbover=%d rxcrc=%d\n\n",
		devname,icount.dsr,icount.cts, icount.txok, icount.txbovr,
		icount.txtimeout, icount.rxok, icount.rxlong, icount.rxrcst,
		icount.rxabort, icount.rxbover, icount.rxcrc);

	return 1;
}	/* end of display_stats() */

int set_hdlc(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->mode = Z055_MODE_HDLC;
	return 1;
}	/* end of set_hdlc() */

int set_loopback(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->flags |= HDLC_FLAG_HDLC_LOOPBACK;
	return 1;
}	/* end of set_loopback() */

int clr_loopback(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->flags &= ~HDLC_FLAG_HDLC_LOOPBACK;
	return 1;
}	/* end of clr_loopback() */

#if 0
int set_TxClkBRGen(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->clockMode = (params->clockMode & ~M75_CLM_TXCLK_CLR) |
						M75_CLM_TXCLK_BRGEN;
	return 1;
}	/* end of set_TxClkBRGen() */

int set_TxClkTRxC(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->clockMode = (params->clockMode & ~M75_CLM_TXCLK_CLR) |
						M75_CLM_TXCLK_TRXC;
	return 1;
}	/* end of set_TxClkTRxC() */

int set_TxClkRTxC(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->clockMode = (params->clockMode & ~M75_CLM_TXCLK_CLR) |
						M75_CLM_TXCLK_RTXC;
	return 1;
}	/* end of set_TxClkRTxC() */

int set_TxClkDPLL(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->clockMode = (params->clockMode & ~M75_CLM_TXCLK_CLR) |
						M75_CLM_TXCLK_DPLL;
	return 1;
}	/* end of set_TxClkDPLL() */

int set_RxClkBRGen(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->clockMode = (params->clockMode & ~M75_CLM_RXCLK_CLR) |
						M75_CLM_RXCLK_BRGEN;
	return 1;
}	/* end of set_RxClkBRGen() */

int set_RxClkTRxC(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->clockMode = (params->clockMode & ~M75_CLM_RXCLK_CLR) |
						M75_CLM_RXCLK_TRXC;
	return 1;
}	/* end of set_RxClkTRxC() */

int set_RxClkRTxC(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->clockMode = (params->clockMode & ~M75_CLM_RXCLK_CLR) |
						M75_CLM_RXCLK_RTXC;
	return 1;
}	/* end of set_RxClkRTxC() */

int set_RxClkDPLL(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->clockMode = (params->clockMode & ~M75_CLM_RXCLK_CLR) |
						M75_CLM_RXCLK_DPLL;
	return 1;
}	/* end of set_RxClkDPLL() */

int set_TRxC_XTAL(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->clockMode = (params->clockMode & ~M75_CLM_TRXC_CLR) |
						M75_CLM_TRXC_XTAL;
	return 1;
}	/* end of set_TRxC_XTAL() */
int set_TRxC_TxClk(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->clockMode = (params->clockMode & ~M75_CLM_TRXC_CLR) |
						M75_CLM_TRXC_TXCLK;
	return 1;
}	/* end of set_TRxC_TxClk() */
int set_TRxC_BRGen(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->clockMode = (params->clockMode & ~M75_CLM_TRXC_CLR) |
						M75_CLM_TRXC_BRGEN;
	return 1;
}	/* end of set_TRxC_BRGen() */
int set_TRxC_DPLL(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->clockMode = (params->clockMode & ~M75_CLM_TRXC_CLR) |
						M75_CLM_TRXC_DPLL;
	return 1;
}	/* end of set_TRxC_DPLL() */
#endif

int set_txctransp(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->flags |= HDLC_FLAG_TXC_TRANSP;
	return 1;
}	/* end of set_txctransp() */

int clr_txctransp(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->flags &= ~HDLC_FLAG_TXC_TRANSP;
	return 1;
}	/* end of clr_txctransp() */
int set_txcrtspin(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->flags &= ~HDLC_FLAG_TXC_DTRPIN;
	params->flags |= HDLC_FLAG_TXC_TRANSP;
	return 1;
}	/* end of set_txcrtspin() */
int set_txcdtrpin(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->flags |= HDLC_FLAG_TXC_DTRPIN;
	params->flags |= HDLC_FLAG_TXC_TRANSP;
	return 1;
}	/* end of set_txcdtrpin() */


int set_crc16(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->crc_mode &= ~HDLC_CRC_CCITT;
	return 1;
}	/* end of set_crc16() */

int set_crcccitt(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->crc_mode |= HDLC_CRC_CCITT;
	return 1;
}	/* end of set_crcccitt() */

int set_crcinv(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->crc_mode |= HDLC_CRC_INV;
	return 1;
}	/* end of set_crcinv() */

int clr_crcinv(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->crc_mode &= ~HDLC_CRC_INV;
	return 1;
}	/* end of clr_crcinv() */

int set_crcpreset(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	int crc_preset = 0;
	if (argc<2 || !sscanf(argv[1],"%d",&crc_preset)) {
		printf("\ncrcpreset option requires preset value (0/1)\n");
		return -EINVAL;
	}
	if( (crc_preset != 1) && (crc_preset != 0) )
		return -EINVAL;
	if( crc_preset )
		params->crc_mode |= HDLC_CRC_PRESET;
	else
		params->crc_mode &= ~HDLC_CRC_PRESET;
	return 2;
}	/* end of set_crcpreset() */

int set_idlemark(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->flags |= HDLC_FLAG_TXIDLE_MARK;
	return 1;
}	/* end of set_idlemark() */
int set_idleflag(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->flags &= ~HDLC_FLAG_TXIDLE_MARK;
	return 1;
}	/* end of set_idleflag() */

int set_nrz(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->encoding = HDLC_ENCODING_NRZ;
	return 1;
}	/* end of set_nrz() */

int set_nrzi(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->encoding = HDLC_ENCODING_NRZI;
	return 1;
}	/* end of set_nrzi() */

int set_nrz_s(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->encoding = HDLC_ENCODING_NRZ_S;
	return 1;
}	/* end of set_nrz_s() */

#if 0
int set_fm0(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->encoding = HDLC_ENCODING_FM0;
	return 1;
}	/* end of set_fm0() */

int set_fm1(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->encoding = HDLC_ENCODING_FM1;
	return 1;
}	/* end of set_fm1() */
#endif

int set_manch(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->encoding = HDLC_ENCODING_MANCHESTER;
	return 1;
}	/* end of set_manch() */

int set_manchnrzi(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->encoding = HDLC_ENCODING_MANCHESTER_NRZI;
	return 1;
}	/* end of set_manchnrzi() */

int set_addrcomp(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->flags |= HDLC_FLAG_ADDRCMP;
	return 1;
}	/* end of set_addrcomp() */

int clr_addrcomp(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->flags &= ~HDLC_FLAG_ADDRCMP;
	return 1;
}	/* end of clr_addrcomp() */

int set_compaddr(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	unsigned int cmpaddr = 0;

	if (argc<2 || !sscanf(argv[1],"%x",&cmpaddr)) {
		printf("\ncompaddr option requires compare address as hex value\n");
		return -EINVAL;
	}
	params->comp_addr = (unsigned char)cmpaddr;
	params->flags |= HDLC_FLAG_ADDRCMP; /* address search mode gets enabled*/
	return 2;
}	/* end of set_compaddr() */

int set_compbroadc(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	unsigned int cmpbroadc = 0;

	if (argc<2 || !sscanf(argv[1],"%x",&cmpbroadc)) {
		printf("\ncompbroadc option requires compare address as hex value\n");
		return -EINVAL;
	}
	params->comp_broadc = (unsigned char)cmpbroadc;
	params->flags |= HDLC_FLAG_ADDRCMP; /* address search mode gets enabled*/
	return 2;
}	/* end of set_compbroadc() */

int set_addrmask(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	unsigned int addrmask = 0;

	if (argc<2 || !sscanf(argv[1],"%x",&addrmask)) {
		printf("\naddrmask option requires compare address mask as hex value\n");
		return -EINVAL;
	}
	params->addr_mask = (unsigned char)addrmask;
	return 2;
}	/* end of set_addrmask() */

int set_broadcmask(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	unsigned int broadcmask = 0;

	if (argc<2 || !sscanf(argv[1],"%x",&broadcmask)) {
		printf("\nbroadcmask option requires compare address mask as hex value\n");
		return -EINVAL;
	}
	params->broadc_mask = (unsigned char)broadcmask;
	return 2;
}	/* end of set_broadcmask() */

int set_baudrate(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	int brate;

	if (argc<2 || !sscanf(argv[1],"%d",&brate)) {
		printf("\baudrate option requires decimal value\n");
		return -EINVAL;
	}

	params->baud_rate = brate;
	return 2;
}	/* end of set_baudrate() */

int set_four_start_flags(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->flags |= HDLC_FLAG_FOUR_START_FLAGS;
	return 1;
}	/* end of set_four_start_flags() */

int clr_four_start_flags(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->flags &= ~HDLC_FLAG_FOUR_START_FLAGS;
	return 1;
}	/* end of clr_four_start_flags() */

int set_half_duplex(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->half_duplex = HDLC_HALF_DUPLEX;
	return 1;
}	/* end of set_half_duplex() */

int clr_half_duplex(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	params->half_duplex = HDLC_FULL_DUPLEX;
	return 1;
}	/* end of clr_half_duplex() */

int set_quiet(int argc, char* argv[], char* devname, Z055_PARAMS* params)
{
	quiet=1;
	return 1;
}	/* end of set_quiet() */
