#!/bin/bash

DRIVERLOADTOOL=/opt/menlinux/DRIVERS/Z055_HDLC/load-drivers.sh

# Sample script to activate PPP connection using a Z055_HDLC adapter
# and the pppd program.
#
#
# This script does not use PAP or CHAP authorization. Refer to
# the pppd documentation for details on authorization.
#
# This script does not use chat to autodial or setup a connection.
# pppd raises DTR and waits for DCD active to start connection.

# Set to yes for synchronous HDLC mode, otherwise async PPP is used

SYNC=yes

# This is the network device name created by pppd.
# If you have only one ppp connection leave this at ppp0.
# For additional connections use ppp1, ppp2 etc.

DEVICE0=ppp0
DEVICE1=ppp1

# MODEMPORT is the device name of the Z055_HDLC device.
#
# adapter and port numbers start at 0
#
# see readme.txt for more information of Z055_HDLC adapter
# device naming conventions.

MODEMPORT0=/dev/ttyTH0
MODEMPORT1=/dev/ttyTH1

# If this port needs to generate an output clock on the
# AUXCLK signal, set GENCLOCK to the data rate in bits per
# second. If data clocks are provided by an external
# device, then set GENCLOCK to zero (default).
#
# Generating a clock is usually only necessary when connecting
# two ports back to back through a NULL modem (cross over cable).
# In this case, one side of the connection generates the clock
# which must be routed to the RxC and TxC input signals on
# both sides of the connection.

GENCLOCK=0

# Asynchronous line speed in bits per second.
# Ignored for synchronous HDLC mode.

LINESPEED=230400

# Output debug information to the system log.
# Set to yes to diagnose problems. Set to no for
# production environment.

DEBUG=yes

# Set this to yes to have pppd create a default route
# in the routing table that uses the ppp network interface.

DEFROUTE=yes

# Monitor the modem status signals. (yes/no)

HARDFLOWCTL=no

# Misc pppd options. Refer to pppd documentation for
# a list of options.

PPPOPTIONS="passive"

# Remote and local IP address. Usually only the
# local IP address is specified.

REMIP0=
IPADDR0=192.1.1.195
REMIP1=
IPADDR1=192.1.1.196

# Maximum Receive/Transmit Unit size in bytes.
# Leave blank unless you know what they do,
# will default to 1500 (DEFMRU)

MRU=
MTU=

if [ "$1" != daemon ] ; then
  # disconnect stdin, out, err to disassociate from controlling tty
  # so that no HUPs will get through.
  $0 daemon $*& </dev/null >/dev/null 2>/dev/null
  exit 0
fi
shift

[ -x /usr/sbin/pppd ] || {
  echo "/usr/sbin/pppd does not exist or is not executable"
  exit 1
}

#-----------------------------------------------------------
# load necessary drivers using the sample driver load script
#-----------------------------------------------------------

DRIVERLOADED=$(lsmod | grep -c men_lx_z055)

if [ "${DRIVERLOADED}" = "0" ] ; then
  echo "Loading necessary drivers for $MODEMPORT..."
  ${DRIVERLOADTOOL} || exit 1
else
  echo "Driver men_lx_z055 is loaded!"
fi

[ -c $MODEMPORT0 ] || {
  echo "Z055_HDLC device $MODEMPORT0 does not exist."
  echo "Be sure that Z055_HDLC device driver is loaded"
  echo "and proper device instance is specified."
  exit 1
}

[ -c $MODEMPORT1 ] || {
  echo "Z055_HDLC device $MODEMPORT1 does not exist."
  echo "Be sure that Z055_HDLC device driver is loaded"
  echo "and proper device instance is specified."
  exit 1
}

opts0="lock noauth"
if [ "${HARDFLOWCTL}" = yes ] ; then
  opts0="$opts0 modem crtscts"
else
  opts0="$opts0 local"
fi
if [ "${DEFROUTE}" = yes ] ; then
  opts0="$opts0 defaultroute"
fi
if [ -n "${MRU}" ] ; then
  echo "set MRU to $MRU"
  opts0="$opts0 mru ${MRU}"
fi
if [ -n "${MTU}" ] ; then
  echo "set MTU to $MTU"
  opts0="$opts0 mtu ${MTU}"
fi
if [ -n "${IPADDR0}${REMIP0}" ] ; then
  # if either IP address is set, the following will work.
  opts0="$opts0 ${IPADDR0}:${REMIP0}"
fi
if [ "${DEBUG}" = yes ] ; then
  opts0="$opts0 debug"
  chatdbg="-v"
fi

opts1="lock noauth"
if [ "${HARDFLOWCTL}" = yes ] ; then
  opts1="$opts1 modem crtscts"
else
  opts1="$opts1 local"
fi
if [ "${DEFROUTE}" = yes ] ; then
  opts1="$opts1 defaultroute"
fi
if [ -n "${MRU}" ] ; then
  echo "set MRU to $MRU"
  opts1="$opts1 mru ${MRU}"
fi
if [ -n "${MTU}" ] ; then
  echo "set MTU to $MTU"
  opts1="$opts1 mtu ${MTU}"
fi
if [ -n "${IPADDR1}${REMIP1}" ] ; then
  # if either IP address is set, the following will work.
  opts1="$opts1 ${IPADDR1}:${REMIP1}"
fi
if [ "${DEBUG}" = yes ] ; then
  opts1="$opts1 debug"
  chatdbg="-v"
fi

#-----------------------------------------------
# Use z055_hdlc_util to configure the adapter
#-----------------------------------------------
#
if ! command -v z055_hdlc_util ; then
  echo "Can't find z055_hdlc_util program."
  echo "Verify program is built and installed"
  exit 1
fi
  Z055UTIL=$(command -v z055_hdlc_util)

if [ "$SYNC" = "yes" ]; then
  PORTOPTIONS="hdlc manch+nrzi -loopback crcpreset 1 crcinv"

  # add PPPD sync option
  opts0="$opts0 sync"
  # add PPPD sync option
  opts1="$opts1 sync"
else
  echo "Modes others than synchronous, frame oriented are not supported by this driver"
  exit 1
fi

if [ ! "$LINESPEED" = "0" ]; then
  echo "Setting linespeed with z055_hdlc_util to $LINESPEED"
  PORTOPTIONS="$PORTOPTIONS baudrate $LINESPEED"
fi

#-------------------------------------------------------
# Default behavior for TTY ports is to activate DTR when
# the port is opened and deactivate DTR when the port is
# closed. This causes a problem when using NULL modem
# connections to connect two systems back-to-back for
# testing. Use 'stty' to clear this HUPCL (hang up on
# close) behavior before using 'z055_hdlc_util' to
# configure the port.
#-------------------------------------------------------
stty --file=$MODEMPORT0 -hupcl
stty --file=$MODEMPORT1 -hupcl

echo "z055_hdlc_util cmd 0:"
echo "$Z055UTIL $MODEMPORT0 $PORTOPTIONS"
$Z055UTIL $MODEMPORT0 $PORTOPTIONS

echo "z055_hdlc_util cmd 1:"
echo "$Z055UTIL $MODEMPORT1 $PORTOPTIONS"
$Z055UTIL $MODEMPORT1 $PORTOPTIONS

#-------------------------------------------
# start pppd program with configured options
#-------------------------------------------
echo ""
echo "pppd cmd 0"
echo "/usr/sbin/pppd -detach $opts0 $MODEMPORT0 ipparam $DEVICE0 ${PPPOPTIONS}"
echo ""
echo "pppd cmd 1"
echo "/usr/sbin/pppd -detach $opts1 $MODEMPORT1 ipparam $DEVICE1 ${PPPOPTIONS}"
echo ""

/usr/sbin/pppd -detach $opts0 $MODEMPORT0 ipparam $DEVICE0 ${PPPOPTIONS} &
PPPD0_PID=$!
echo "PPPD0_PID: $PPPD0_PID"
/usr/sbin/pppd -detach $opts1 $MODEMPORT1 ipparam $DEVICE1 ${PPPOPTIONS}

kill $PPPD0_PID
#-------------------------------------------
# reset default HUPCL behavior on port after
# ppp session terminates.
#-------------------------------------------
stty --file=$MODEMPORT0 hupcl
stty --file=$MODEMPORT1 hupcl
