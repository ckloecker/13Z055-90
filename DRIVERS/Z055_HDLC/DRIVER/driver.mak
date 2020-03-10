#**************************  M a k e f i l e ********************************
#
#         Author: christian.schuster@men.de
#
#    Description: makefile descriptor for Toronto linux native driver
#
#
#-----------------------------------------------------------------------------
# Copyright 2004-2020, MEN Mikro Elektronik GmbH, Nuernberg, Germany
#*****************************************************************************
MAK_NAME=z055_hdlc
# the next line is updated during the MDIS installation
STAMPED_REVISION="49bf2e6-dirty_2020-03-09"

DEF_REVISION=MAK_REVISION=$(STAMPED_REVISION)

MAK_SWITCH=$(SW_PREFIX)MAC_MEM_MAPPED\
		$(SW_PREFIX)$(DEF_REVISION)

MAK_LIBS=	\

MAK_INCL=$(MEN_INC_DIR)/../../NATIVE/MEN/z055_hdlc.h     \
		 $(MEN_MOD_DIR)/z055_hdlc_int.h \

MAK_INP1=z055_hdlc_drv$(INP_SUFFIX)

MAK_INP=$(MAK_INP1)
