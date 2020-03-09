#**************************  M a k e f i l e ********************************
#
#         Author: cs
#          $Date: 2005/02/15 14:36:28 $
#      $Revision: 1.1 $
#
#    Description: Makefile definitions for the z055_hdlc_util program
#
#---------------------------------[ History ]---------------------------------
#
#   $Log: program.mak,v $
#   Revision 1.1  2005/02/15 14:36:28  cs
#   Initial Revision
#
#   Revision 1.1  2005/02/15 14:15:29  cs
#   Initial Revision
#
#
#
#-----------------------------------------------------------------------------
#   (c) Copyright 2004 by MEN mikro elektronik GmbH, Nuernberg, Germany
#*****************************************************************************

MAK_NAME=z055_hdlc_util

MAK_LIBS= \


MAK_INCL=$(MEN_INC_DIR)/men_typs.h \
		 $(MEN_INC_DIR)/../../NATIVE/MEN/z055_hdlc.h \

MAK_INP1=z055_hdlc_util$(INP_SUFFIX)

MAK_INP=$(MAK_INP1)
