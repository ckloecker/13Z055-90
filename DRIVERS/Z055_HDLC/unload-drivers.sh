#!/bin/sh
# script to unload the Z055 HDLC device driver and
# HDLC line discipline driver
#
# $Id: unload-drivers.sh,v 1.1 2005/02/15 14:36:23 cs Exp $
# originally cloned from load-drivers.sh
# from the Microgate Synclink driver package

# locate the necessary module utilities
# these locations may be distribution specific

RMMOD=/sbin/rmmod
if [ ! -f $RMMOD ]; then
    RMMOD=rmmod
fi

LSMOD=/bin/lsmod
if [ ! -f $LSMOD ]; then
    LSMOD=lsmod
fi

( $LSMOD | grep -i "men_z055_hdlc" >/dev/null) 	&& ${RMMOD} men_z055_hdlc
( $LSMOD | grep -i "n_hdlc"	>/dev/null)	&& ${RMMOD} n_hdlc



