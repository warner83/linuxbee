#! /bin/sh

XBEE_DRIVER_PATH=$(dirname $0)

XBEE_ADDRESS="10.0.0.0"
XBEE_NETMASK="255.255.255.0"

insmod ${XBEE_DRIVER_PATH}/n_xbee.ko &&
${XBEE_DRIVER_PATH}/ldisc_daemon /dev/ttyACM0 115200 &&
ifconfig xbee0 ${XBEE_ADDRESS} netmask ${XBEE_NETMASK} up


