#!/bin/sh

# Sample script to deactivate PPP connection using a MEN Z055 adapter
# and the pppd program.
#
# $Id: stop-ppp.sh,v 1.1 2005/02/15 14:36:20 cs Exp $

# This is the network device name created by pppd.
# If you have only one ppp connection leave this at ppp0.
# For additional connections use ppp1, ppp2 etc.

DEVICE=ppp0

PIDFILE="/var/run/$DEVICE.pid"

[ -f $PIDFILE ] || {
    echo -e "The pppd process ID file for device $DEVICE \n" \
            "does not exist. Either pppd is not running or \n" \
            "the wrong device was specified."
}

PID=`cat $PIDFILE`

if [ -n $PID ]; then
    echo "Process ID of pppd for device $DEVICE is $PID"
    echo "Sending hangup signal to process $PID..."
    /bin/kill -HUP $PID
else
    echo "$PIDFILE is empty, the pppd process ID cant be identified"
fi




