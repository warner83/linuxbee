#! /bin/sh

XBEE_DRIVER_PATH=$(dirname $0)

XBEE_ADDRESS="10.0.0.0"
XBEE_NETMASK="255.255.255.0"

insmod ${XBEE_DRIVER_PATH}/n_xbee.ko &&
ifconfig xbee0 ${XBEE_ADDRESS} netmask ${XBEE_NETMASK} up
killall ldisc_daemon  # Very strange I've to start kill and then start again otherwise recv does not work, cruel!
sleep 1
${XBEE_DRIVER_PATH}/ldisc_daemon /dev/ttyACM0 115200
