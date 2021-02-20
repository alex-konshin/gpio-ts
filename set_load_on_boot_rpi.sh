#!/bin/sh

# This script deploys gpio-ts.ko into system and updates system configuration 
# to load the driver during system boot.
# The script has to be run as root or via sudo.

RUNDIR=`pwd`
BUILD_HOME=`dirname $0`
BUILD_HOME=`cd $BUILD_HOME; pwd`

# Comma-separated list of GPIOs.
# No blanks are allowed.
LIST_OF_GPIOS=27

if [ ! -f ${BUILD_HOME}/gpio-ts.ko ]; then
  echo "You must build gpio-ts.ko before running this script."
  exit 1
fi

if [ ! -d /etc/modprobe.d ]; then
  echo "Directory /etc/modprobe.d does not exist. Either this script does not support this platform or it should be updated."
  exit 1
fi

if [ ! -d /etc/modules-load.d ]; then
  echo "Directory /etc/modules-load.d does not exist. Either this script does not support this platform or it should be updated."
  exit 1
fi

echo "Copying \"${BUILD_HOME}/gpio-ts.ko\" to system directory \"/lib/modules/$(uname -r)/kernel/drivers/gpio/\"..."
cp -f "${BUILD_HOME}/gpio-ts.ko" "/lib/modules/$(uname -r)/kernel/drivers/gpio/gpio-ts.ko"
status=$?
if [ "$status" -ne 0 ]; then
  echo "Failed to copy the driver to \"/lib/modules/$(uname -r)/kernel/drivers/gpio/gpio-ts.ko\".\nDo you run the script as root?"
  exit $status
fi

echo "Running depmod..."
depmod
status=$?
if [ "$status" -ne 0 ]; then
  echo "Error code ${status} from depmod."
  exit $status
fi

echo "Configuring automatic load on system boot..."
echo "options gpio_ts gpios=${LIST_OF_GPIOS}\n" > /etc/modprobe.d/gpio_ts.conf
echo "gpio_ts\n" > /etc/modules-load.d/gpio_ts.conf

echo "Done."
exit 0
