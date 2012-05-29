# Copyright (C) 2012 t-lo <thilo@thilo-fromm.de>
# Copyright (C) 2012 Richard Hughes <richard@hughsie.com>
#
# Licensed under the GNU General Public License Version 2
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#
#
# Helper Makefile to check for, fetch, and install Microchip's toolchain.
#
# The toolchain package, "MPLAB® C18 Lite Compiler for PIC18 MCUs", is
# referenced at this page :
#
# 	<http://www.microchip.com/pagehandler/en-us/family/mplabx/>
#

include makefiles/common-microchip.mk

MICROCHIP_TOOLCHAIN_ROOT = ${MICROCHIP_ROOT}/mplabc18/v3.40

TOOLCHAIN_URL = http://www.microchip.com/mplabc18-linux-installer

TOOLCHAIN_INSTALLER   = ${DOWNLOAD_DIR}/mplabc18-v3.40-linux-full-installer.run
TOOLCHAIN_UNINSTALLER = ${MICROCHIP_TOOLCHAIN_ROOT}/UninstallMPLABC18v3.40

CC = ${MICROCHIP_TOOLCHAIN_ROOT}/bin/mcc18
AS = ${MICROCHIP_TOOLCHAIN_ROOT}/mpasm/MPASMWIN
LD = ${MICROCHIP_TOOLCHAIN_ROOT}/bin/mplink
AR = ${MICROCHIP_TOOLCHAIN_ROOT}/bin/mplib

CC_DEVICE = 18f46j50
LD_DEVICE = 18F46J50

linker_script = 18F46J50.lkr

# include toolchain headers and app lib includes into the build and set the device
CFLAGS  += -I$(MICROCHIP_TOOLCHAIN_ROOT)/h -I${MICROCHIP_APP_LIB_ROOT}/Microchip/Include -p${CC_DEVICE}

# include toolchain libraries into the build and set the device
LDFLAGS += ${linker_script} -p ${LD_DEVICE} -l ${MICROCHIP_TOOLCHAIN_ROOT}/lib -w -z__MPLAB_BUILD=1 -u_CRUNTIME 

.PHONY: check-toolchain 		\
	sudo-install-toolchain 		\
	sudo-uninstall-toolchain	\
	clean-toolchain 

clean-toolchain:
	rm -rf ${DOWNLOAD_DIR}/

check-toolchain:
	@${CC} -v    >/dev/null 2>&1 &&								\
	 ${LD} -v    >/dev/null 2>&1 || { echo "";						\
		echo "####  You are lacking the Microchip toolchain.";				\
		echo "####  This toolchain is required to build the ColorHug firmware."; 	\
		echo "####  To fix it please run:"; 						\
		echo "";									\
		echo "		make sudo-install-toolchain"; 					\
		echo ""; false; }

${TOOLCHAIN_INSTALLER}:
	mkdir -p ${DOWNLOAD_DIR}
	@echo ""
	@echo "####  Downloading the toolchain installer from ${TOOLCHAIN_URL}."
	@echo ""
	wget -O $@ ${TOOLCHAIN_URL}
	chmod 755 $@
	

sudo-install-toolchain: ${TOOLCHAIN_INSTALLER}
	@sudo=""											 \
	test "`whoami`" = "root" || { echo "";								 \
		 echo  "####   Unfortunately, Microchip's toolchain installer requires root privileges.";\
		 echo  "####   I will attempt to run the installer with 'sudo' for you.";		 \
		 echo  "####   If this does not work please re-run this make command as root.";		 \
		 echo "";								  	 	 \
		 sudo echo "Sudo successful. Now installing, please be patient...";		  	 \
		 sudo="sudo"; };									 \
	$$sudo ${TOOLCHAIN_INSTALLER} --mode unattended &&						 \
		cd ${MICROCHIP_TOOLCHAIN_ROOT}/lib/ &&							 \
		sudo ln -fs p18F46J50.lib  p18f46j50.lib &&						 \
		echo "Done."

sudo-uninstall-toolchain: ${TOOLCHAIN_UNINSTALLER}
	@sudo=""											\
	test "`whoami`" = "root" || { echo "";								\
		 echo  "####   Unfortunately, Microchip's toolchain un-installer requires root.";	\
		 echo  "####   I will attempt to run it with 'sudo' for you.";				\
		 echo  "####   If this does not work please re-run this make command as root.";		\
		 echo "";								  	 	\
		 sudo echo "Sudo successful. Now un-installing, please be patient...";		  	\
		 sudo="sudo"; };									\
	$$sudo ${TOOLCHAIN_UNINSTALLER} --mode unattended &&						\
		echo "Done."


