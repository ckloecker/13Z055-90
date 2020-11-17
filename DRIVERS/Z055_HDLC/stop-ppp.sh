#!/bin/sh

# Sample script to deactivate PPP connection using a MEN Z055 adapter
# and the pppd program.
#

# This is the network device name created by pppd.
# If you have only one ppp connection leave this at ppp0.
# For additional connections use ppp1, ppp2 etc.

DEVICE=ppp0

PIDFILE="/var/run/$DEVICE.pid"

[ -f $PIDFILE ] || {
    echo "The pppd process ID file for device $DEVICE"
    echo "does not exist. Either pppd is not running or"
    echo "the wrong device was specified."
}

PID=$(cat $PIDFILE)

if [ -n "$PID" ]; then
    echo "Process ID of pppd for device $DEVICE is $PID"
    echo "Sending hangup signal to process $PID..."
    /bin/kill -HUP "$PID"
else
    echo "$PIDFILE is empty, the pppd process ID cant be identified"
fi




