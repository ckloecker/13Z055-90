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
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
MAK_NAME=lx_z055
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
