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
# Helper Makefile to check for, fetch, and install Microchip's app library. 
#
#
# The libraries package, "Microchip Application Libraries", is referenced here:
#
#	<http://www.microchip.com/stellent/idcplg?IdcService=SS_GET_PAGE&nodeId=2680&dDocName=en547784>
#

include makefiles/common-microchip.mk

MICROCHIP_APP_LIB_ROOT 	 = ${MICROCHIP_ROOT}/libs-v2012-04-03

APP_LIB_URL   = http://ww1.microchip.com/downloads/en/softwarelibrary/microchip-application-libraries-v2012-04-03-linux-installer.run

APP_LIB_INSTALLER   = ${DOWNLOAD_DIR}/microchip-application-libraries-v2012-04-03-linux-installer.run
APP_LIB_UNINSTALLER = ${MICROCHIP_APP_LIB_ROOT}/Uninstall\ Microchip\ Application\ Libraries\ v2012-04-03

.PHONY: check-app-lib 		\
	sudo-install-app-lib	\
	sudo-uninstall-app-lib	\
	clean-app-lib

clean-app-lib:
	rm -rf ${DOWNLOAD_DIR}/

check-app-lib:
	@test -d ${MICROCHIP_APP_LIB_ROOT}/Microchip/Include || { echo "";			\
		echo "####  You are lacking the Microchip application libraries.";		\
		echo "####  These libraries are required to build the ColorHug firmware."; 	\
		echo "####  To fix it please run:"; 						\
		echo "";									\
		echo "		make sudo-install-app-lib"; 					\
		echo ""; false; }


${APP_LIB_INSTALLER}:
	mkdir -p ${DOWNLOAD_DIR}
	@echo ""
	@echo "####  Downloading the application libraries installer from ${APP_LIB_URL}."
	@echo ""
	wget -O $@ ${APP_LIB_URL}
	chmod 755 $@


sudo-install-app-lib: ${APP_LIB_INSTALLER}
	@sudo=""											  \
	test "`whoami`" = "root" || { echo "";								  \
		 echo  "####   Unfortunately, Microchip's application libraries installer requires";	  \
		 echo  "####   root privileges. I will attempt to run the installer with 'sudo' for";	  \
		 echo  "####   you. If this does not work please re-run this make command as root."; 	  \
		 echo "";									  	  \
		 sudo echo "Sudo successful. Now installing, please be patient...";		  	  \
		 sudo="sudo"; };									  \
	$$sudo ${APP_LIB_INSTALLER} --mode unattended --prefix "${MICROCHIP_APP_LIB_ROOT}" >/dev/null &&  \
		echo "Done."


sudo-uninstall-app-lib: ${APP_LIB_UNINSTALLER}
	@sudo=""											\
	test "`whoami`" = "root" || { echo "";								\
		 echo  "####   Unfortunately, Microchip's app library un-installer requires root.";	\
		 echo  "####   I will attempt to run it with 'sudo' for you.";				\
		 echo  "####   If this does not work please re-run this make command as root.";		\
		 echo "";								  	 	\
		 sudo echo "Sudo successful. Now un-installing, please be patient...";		  	\
		 sudo="sudo"; };									\
	$$sudo ${APP_LIB_UNINSTALLER} --mode unattended &&						\
		echo "Done."

