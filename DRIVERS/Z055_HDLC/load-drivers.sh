#!/bin/sh

# Sample script to load the Z055 HDLC device driver
#
#
# Set the device options and select which supporting drivers to load
# by modifying the script variables below.
#
# originally cloned from load-drivers.sh
# from the Microgate Synclink driver package

#--------------------------------------------------------------------------
SCRIPT_ARGS=

# locate the necessary module utilities
# these locations may be distribution specific

MODPROBE=/sbin/modprobe
if [ ! -f $MODPROBE ]; then
    MODPROBE=modprobe
fi

LSMOD=/bin/lsmod
if [ ! -f $LSMOD ]; then
    LSMOD=lsmod
fi

#--------------------------------------------------------------------------
# This section sets variables specifying arguments to load to the
# drivers as they are loaded
#--------------------------------------------------------------------------

# Z055 HDLC driver options (see README.TXT for details)
#
# Debug level is global for all device instances in a driver.
# Debug levels other than 0 (disabled) should only be used for diagnosing
# problems because the debugging output will fill the system log and
# degrade performance.
# The following debug levels are or-combined:
#      DEBUG_LEVEL_DATA     0x01
#      DEBUG_LEVEL_ERROR    0x02
#      DEBUG_LEVEL_INFO     0x04
#      DEBUG_LEVEL_BH       0x08
#      DEBUG_LEVEL_ISR      0x10
#
# Options other than debug level require an entry for each installed
# adapter seperated by commas. For example, if two adapters are installed
# then the MAX_FRAME_SIZE options might be set to "2044,1024"

Z055_HDLC_DEBUG_LEVEL="0x0"

Z055_HDLC_MAX_FRAME_SIZE="2044,2044"

Z055_HDLC_RXBUFS="5,5"

#--------------------------------------------------------------------------
# This section builds command line args for the drivers, based on the
# above configuration items
#--------------------------------------------------------------------------

# adapter driver module options

Z055_HDLC_OPTIONS=""
if [ -f /etc/z055_hdlc.conf ] ; then
    . /etc/z055_hdlc.conf
    Z055_HDLC_OPTIONS="$SCRIPT_ARGS $Z055_HDLC_OPTIONS"
fi
if [ $Z055_HDLC_DEBUG_LEVEL ] ; then
    Z055_HDLC_OPTIONS="${Z055_HDLC_OPTIONS} debug_level=${Z055_HDLC_DEBUG_LEVEL}"
fi
if [ $Z055_HDLC_MAX_FRAME_SIZE ] ; then
    Z055_HDLC_OPTIONS="${Z055_HDLC_OPTIONS} maxframe=${Z055_HDLC_MAX_FRAME_SIZE}"
fi
if [ $Z055_HDLC_RXBUFS ]; then
    Z055_HDLC_OPTIONS="${Z055_HDLC_OPTIONS} num_rxbufs=${Z055_HDLC_RXBUFS}"
fi


#--------------------------------------------------------------------------
# common routine to load the specified adapter
# driver and create the device nodes required
#
# Args: name prefix [adapters] [ports] [group] [mode]
#
# where:
#	name		driver module name (required)
#	prefix		device name prefix (required)
#	adapters	number of adapters (optional)
#				default=2
#	ports		number of ports (optional)
#				default=0
#	group		group owner for device nodes (optional)
#				default=root
#	mode		access permissions for device nodes (optional)
#				default=666 (rw for all)
#
#--------------------------------------------------------------------------
load_and_mknode() {
    DRIVER=$1
    DEVICE_PREFIX=$2
    NUM_ADAPTERS=$3
    NUM_PORTS=$4
    DEVICE_GROUP=$5
    DEVICE_MODE=$6

    if [ -z $DRIVER ] ; then
        echo "Driver name not specified\n"
        return 1;
    fi
    if [ -z $DEVICE_PREFIX ] ; then
        echo "Device name prefix name not specified\n"
        return 1;
    fi

    #
    # set defaults if some arguments were not specified
    #
    if [ -z $DEVICE_GROUP ]; then
        DEVICE_GROUP=root
    fi
    if [ -z $DEVICE_MODE ]; then
        DEVICE_MODE=666
    fi
    if [ -z $NUM_ADAPTERS ]; then
        NUM_ADAPTERS=2
    fi
    if [ -z $NUM_PORTS ]; then
       NUM_PORTS=0
    fi

    #
    # load the driver module specified by
    # the first arg
    #
    if ( ! ( $LSMOD | grep -i "$DRIVER" > /dev/null ) ) then
        $MODPROBE $DRIVER $Z055_HDLC_OPTIONS || {
    	    echo "Can't load $DRIVER driver."
	    return 1
        }
    fi

    #
    # Create a list of device names for our adapters
    #
    DEVNAMES=
    adapter=0;
    while [ $adapter -lt $NUM_ADAPTERS ] ; do
        if [ $NUM_PORTS -ne 0 ]; then
            port=0
            while [ $port -lt $NUM_PORTS ] ; do
                DEVNAMES="${DEVNAMES} /dev/${DEVICE_PREFIX}${adapter}p${port}"
                port=$((port+1))
            done
        else
            DEVNAMES="${DEVNAMES} /dev/${DEVICE_PREFIX}${adapter}"
        fi
        adapter=$((adapter+1))
    done

    # Create device special files using the dynamically
    # assigned device major number. Device minor numbers
    # start at 64. Remove any existing device nodes
    # and create new ones based on the drivers assigned
    # major value.
    #
    echo "Removing existing device nodes ..."
    rm -f ${DEVNAMES}

    echo "Creating new device nodes ..."

    ttymajor=`cat /proc/devices | awk "\\$2==\"$DEVICE_PREFIX\" {print \\$1}"`
    ttyminor=64

    for device in ${DEVNAMES} ; do
        mknod ${device} c $ttymajor $ttyminor
        ttyminor=$(($ttyminor + 1))
    done

    # give appropriate group/permissions
    chgrp ${DEVICE_GROUP} ${DEVNAMES}
    chmod ${DEVICE_MODE}  ${DEVNAMES}

}




#--------------------------------------------------------------------------
# load drivers
#--------------------------------------------------------------------------

load_and_mknode men_lx_z055 ttyTH 2

exit 0;


