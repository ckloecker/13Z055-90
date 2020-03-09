#**************************  M a k e f i l e ********************************
#
#         Author: christian.schuster@men.de
#          $Date: 2005/02/15 14:36:26 $
#      $Revision: 1.1 $
#
#    Description: makefile descriptor for Toronto linux native driver
#
#---------------------------------[ History ]---------------------------------
#
# $Log: driver.mak,v $
# Revision 1.1  2005/02/15 14:36:26  cs
# Initial Revision
#
# Revision 1.1  2005/02/15 14:15:26  cs
# Initial Revision
#
#
#
#-----------------------------------------------------------------------------
#   (c) Copyright 2004 by MEN mikro elektronik GmbH, Nuernberg, Germany
#*****************************************************************************
MAK_NAME=z055_hdlc

MAK_SWITCH=$(SW_PREFIX)MAC_MEM_MAPPED\

MAK_LIBS=	\

MAK_INCL=$(MEN_INC_DIR)/../../NATIVE/MEN/z055_hdlc.h     \
		 $(MEN_MOD_DIR)/z055_hdlc_int.h \

MAK_INP1=z055_hdlc_drv$(INP_SUFFIX)

MAK_INP=$(MAK_INP1)
