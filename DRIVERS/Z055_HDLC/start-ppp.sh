#!/bin/sh
PATH=/sbin:/usr/sbin:/bin:/usr/bin

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

DEVICE=ppp0

# MODEMPORT is the device name of the Z055_HDLC device.
#
# adapter and port numbers start at 0
#
# see readme.txt for more information of Z055_HDLC adapter
# device naming conventions.

MODEMPORT=/dev/ttyTH0

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

REMIP=
IPADDR=192.1.1.195

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

echo "Loading necessary drivers for $MODEMPORT..."
./load-drivers.sh  || exit 1

[ -c $MODEMPORT ] || {
  echo -e "Z055_HDLC device $MODEMPORT does not exist.\n" \
          "Be sure that Z055_HDLC device driver is loaded" \
          "and proper device instance is specified.\n"
  exit 1
}

opts="lock noauth"
if [ "${HARDFLOWCTL}" = yes ] ; then
  opts="$opts modem crtscts"
else
  opts="$opts local"
fi
if [ "${DEFROUTE}" = yes ] ; then
  opts="$opts defaultroute"
fi
if [ -n "${MRU}" ] ; then
	echo "set MRU to $MRU"
  opts="$opts mru ${MRU}"
fi
if [ -n "${MTU}" ] ; then
	echo "set MTU to $MTU"
  opts="$opts mtu ${MTU}"
fi
if [ -n "${IPADDR}${REMIP}" ] ; then
  # if either IP address is set, the following will work.
  opts="$opts ${IPADDR}:${REMIP}"
fi
if [ "${DEBUG}" = yes ] ; then
  opts="$opts debug"
  chatdbg="-v"
fi

#-----------------------------------------------
# Use z055_hdlc_util to configure the adapter
#-----------------------------------------------
#
if [ -f z055_hdlc_util ] ; then
    echo "Can't find z055_hdlc_util program."
    echo "Verify program is built and installed in $PATH."
    exit 1
fi

if [ "$SYNC" = "yes" ]; then
    PORTOPTIONS="hdlc manch+nrzi -loopback crcpreset 1 crcinv"

    # add PPPD sync option
    opts="$opts sync"
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
stty --file=$MODEMPORT -hupcl

z055_hdlc_util $MODEMPORT $PORTOPTIONS

#-------------------------------------------
# start pppd program with configured options
#-------------------------------------------

/usr/sbin/pppd -detach $opts $MODEMPORT \
 				ipparam $DEVICE ${PPPOPTIONS}

#-------------------------------------------
# reset default HUPCL behavior on port after
# ppp session terminates.
#-------------------------------------------
stty --file=$MODEMPORT hupcl
