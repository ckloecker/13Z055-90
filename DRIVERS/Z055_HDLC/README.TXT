****************************************
*** MEN Z055 HDLC Adapters for Linux ***
****************************************

This document describes the MEN family of serial adapters and software support for
the Linux operating system. The information is organized into the follow sections:

Software Map
Building the Z055 Drivers
Building the Utilities
Device Driver Overview
Loading and Unloading the Driver
   Z055 HDLC Adapter Driver Load Options
   Max Frame Sizes:
   Device major and minor numbers
   Unloading the device driver
   Interrupt Sharing
   Z055 HDLC Device Naming Conventions
z055_hdlc_util Configuration Utility

More information is available in the following files:

README.PPP            Setting up a PPP network connection (using pppd program)


********************
*** Applications ***
********************

* Synchronous PPP connections

Synchronous PPP
---------------

   The point to point protocol (PPP) provides a network connection between
   two devices connected by a point to point media such as a serial
   connection. High speed applications typically use synchronous (HDLC) serial
   communication which is not supported by the standard COM ports.

   The Z055 HDLC adapter provides a high speed synchronous connection that
   can be used by the existing Linux PPP software. Linux PPP currently
   supports a variety of network protocols including IP, IPX, and
   Appletalk. The Linux PPP implementation uses a device driver ppp.o and
   a user program pppd. The Z055 HDLC driver can be used by the ppp
   software in synchronous HDLC modes.

   For more information on using the Z055 HDLC adapter for PPP read the
   README.PPP file supplied with the Z055 HDLC software.

********************
*** Software Map ***
********************

The Z055 HDLC adapter includes the following software:

* Device Drivers
    - z055_hdlc_drv.c    Z055 HDLC PCI adapter
    - z055_hdlc.h        common header file for drivers and applications
    - z055_hdlc_int.h    internal header file
    - driver.mak         makefile to use in the MEN MDIS build environment
* Z055 HDLC configuration utility (z055_hdlc_util.c)
* sample shell scripts
    - load-drivers.sh    load the device driver and line discipline
    - unload-drivers.sh  unload the device driver and line discipline
    - start-ppp.sh       start a PPP connection
    - stop-ppp.sh        stop a PPP connection

All Z055 HDLC software includes source code released under the GNU General
Public License (GPL).



***************************************
*** Building the Z055 HDLC  Drivers ***
***************************************

Building the Z055 HDLC drivers requires the Linux kernel source,
usually located in the directory /usr/src/linux. The kernel source
is available on most Linux distributions and from ftp.kernel.org.

The kernel source must have an existing configuration, located
in the file /usr/src/linux/.config and must have build dependencies
located in files named .depend located throughout the kernel source.
If the configuration file is missing, a configuration can be
generated using the 'make menuconfig' command in the kernel
source directory. The dependency files are created, after generating
a configuration file, using the 'make dep' command in the kernel
source directory. Refer to the kernel documentation for more details.


    Note on Driver and Kernel Versions
    ----------------------------------

    currently only Kernel version 2.4.xx is supported

The Z055 HDLC drivers can be built using 2 methods:

1. in the MDIS driver directory, outside of the kernel source tree,
   using the MDIS build environment (ELinOS) and the driver.mak file supplied.
2. with standard kernel build procedures in the kernel source tree.


Building Drivers Outside of the Kernel Source Tree
--------------------------------------------------

This is the easiest method of building the Z055 HDLC drivers,
and works even if the drivers are not part of the kernel source.
This method *only* builds the drivers as modules. Building
drivers into the kernel requires building inside of the
kernel source tree as described in the next section.

In the $(ELINOS_PROJECT)/src/mdis directory add the driver.mak file
to the key ALL_NATIVE_DRIVERS
(e.g. ALL_NATIVE_DRIVERS = DRIVERS/Z055_HDLC/driver.mak)

then build / compile the project from the $(ELINOS_PROJECT) directory type
# make boot

This builds the men_z055_hdlc driver for use with
the kernel located in the $(ELINOS_PROJECT)/linux directory, copies the
binaries to the lib/modules/x.y.z directory and runs the depmod program
to calculate module dependencies.

After installation, the drivers should be ready for use.


Building Drivers Inside of the Kernel Source Tree
-------------------------------------------------

/* to be done */

******************************
*** Building the Utilities ***
******************************

The programs can be built in the Z055_HDLC
distribution directory by performing the following steps

In the $(ELINOS_PROJECT)/src/mdis directory add the driver.mak file
to the key ALL_NATIVE_TOOLS
(e.g. ALL_NATIVE_TOOLS = DRIVERS/Z055_HDLC/Z055_HDLC_UTIL/program.mak)

then build / compile the project from the $(ELINOS_PROJECT) directory type
# make boot


******************************
*** Device Driver Overview ***
******************************

header file  : z055_hdlc.h

source files          : z055_hdlc_drv.c
internal header file  : z055_hdlc_int.h
binary file           : men_z055_hdlc.o

The synchronous PPP communications is provided by
the freely available 'pppd' package.


****************************************
*** Loading and Unloading the Driver ***
****************************************

The driver is a loadable module that can be dynamically started, stopped,
and reconfigured. The sample scripts load-drivers.sh and unload-drivers.sh
demonstrate the steps necessary to load and unload the drivers as
described below.

The load-drivers.sh script file will load the required driver.


See the section entitled 'Z055 HDLC Device Naming Conventions' below for
more information regarding device names.

The unload-drivers.sh script file will remove all loaded drivers which are
not active.

The device driver is loaded and unloaded using the standard module
utilities modprobe (load module and and dependent modules), insmod (insert
module) and rmmod (remove module). Refer to the man pages for these
utilities for more information.

Z055 HDLC Adapter Driver Load Options
------------------------------------

   The following options are used with insmod to control the Z055 HDLC
   device drivers:

   maxframe=N[,N...]       specify the maximum frame size for each device
                           instance.

                           If this option is not specified the default
                           maximum frame size of 2046 is used for the
                           device instance. For each instance whose value
                           is not specified or is specified as 0, the
                           default will be used.

   debug_level=N           specify the debug output level for the driver.
                           debugging output is sent to the kernel logger
                           which usually forwards the output to the system
                           logger syslogd. All output levels ored:

                           0x00 = debugging output disabled
                           0x01 = output raw send and receive data
                           0x02 = output error messages
                           0x04 = output informational messages
                           0x08 = output bottom half processing messages
                           0x10 = output interrupt processing messages

   ttymajor=N              Specify the tty driver major device number.
                           Usually this number is dynamically assigned
                           when the driver is loaded. Use this parameter
                           to force the device number to a particular
                           value.

   num_rxbufs=N[,N...]     Specify the number of receive buffers for
                           use on the adapter. The default is 5 if not
                           specified. This parameter is used to buffer
                           the data before beeing handed over to the
                           application. Usually 5 buffers are sufficient.


   Note: All Z055 HDLC PCI adapters are automatically detected and
   configured. No insmod options are required for using PCI adapters.

      An example of the insmod command may look like:

         insmod men_z055_hdlc.o

   Any PCI adapters in the computer will be automatically detected.


* Max Frame Sizes:

   Each device instance can have a different maximum frame size associated
   with it. For example, assume you have two Z055 device adapters:

      insmod men_z055_hdlc.o maxframe= 0,1024

   would result in :

      ttyTH0 having a the default max frame size
      ttyTH1 having a max frame size of 1024

* Device major and minor numbers

   Linux internally identifies devices by the device major and minor
   numbers. The major number identifies a particular driver. The minor
   number identifies a device instance managed by the driver. By default,
   the Z055 HDLC major number is dynamically assigned when the driver is
   loaded. The ttymajor option can be used to force a specific device
   major number when loading the Z055 HDLC driver. Z055 HDLC device minor
   numbers start at 64.

   After loading the Z055 HDLC driver, the device major number can be
   determined with the command: cat /proc/devices. The Z055 HDLC devices
   will be listed under the 'Character devices' section:

      Adapter                          List As
      -------                          -------
      Z055 HDLC Adapter                ttyTH

   A device name, such as /dev/ttyTH0 is mapped to the device major
   and minor numbers using the make special file program 'mknod'.
   Because the Z055 HDLC driver dynamically allocates the device major
   number, these links must be updated every time the driver is loaded.
   This can be done with the 'load-drivers.sh' shell script.
   This script queries the system for the device numbers and calls
   mknod to create the device files.

   Alternatively mknod can be used to create the device files once, and
   the ttymajor option can be used with insmod to force the use of a
   particular device major number. Care must be taken to avoid a conflict
   with an existing major device number or the Z055 HDLC driver will fail
   to load.

* Unloading the device driver

   When no applications are using the Z055 HDLC device driver, the driver
   may be unloaded with the following command:

      rmmod men_z055_hdlc

   If the driver is still in use when rmmod is executed, then a message is
   displayed indicating that the device is busy, and the driver remains
   loaded.

* Interrupt Sharing

   The Z055 HDLC PCI adapters can share interrupts with other devices. The
   Z055 HDLC device driver has been tested in a shared interrupt
   configuration. Drivers for some devices may not be 'well behaved' and
   may cause problems when sharing interrupts. If you encounter problems
   when sharing interrupts, try changing BIOS settings to assign a
   seperate interrupt to the Z055 HDLC adapter. If the BIOS does not
   support interrupt assignment, test the Z055 HDLC adapter with other
   devices removed or disabled.

* Z055 HDLC Device Naming Conventions

   Z055 HDLC Adapter:   /dev/ttyTHx

      Where 'x' is the adapter instance, the first Z055 HDLC adapter will
      be known as '/dev/ttyTH0', the next adapter will be '/dev/ttyTH1',
      etc.

********************************************
*** z055_hdlc_util Configuration Utility ***
********************************************

source file : z055_hdlc_util.c
binary file : z055_hdlc_util

The command line utility z055_hdlc_util allows the user to set the Z055 HDLC mode
(HDLC) and serial options. This utility can be called from shell scripts.

For usage and options of the shell script please type z055_hdlc_util -h

The z055_hdlc_util.c source files is provided to demonstrate how to
programmatically control these parameters for custom applications.



