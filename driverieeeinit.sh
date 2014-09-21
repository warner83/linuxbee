#! /bin/sh

XBEE_DRIVER_PATH=$(dirname $0)

XBEE_ADDRESS="10.0.0.0"
XBEE_NETMASK="255.255.255.0"

modprobe ieee802154
modprobe 6lowpan
insmod ${XBEE_DRIVER_PATH}/ieee802154_xbee.ko &&
ip link add link hardwpan0 name lowpan0 type lowpan
ip link set lowpan0 address de:ad:be:ef:ca:fe:ca:fe
ip link set lowpan0 up
#ifconfig hardwpan0 ${XBEE_ADDRESS} netmask ${XBEE_NETMASK} up
killall ldisc_daemon  # Very strange I've to start kill and then start again otherwise recv does not work, cruel!
sleep 1
${XBEE_DRIVER_PATH}/ldisc_daemon /dev/ttyACM0 115200
