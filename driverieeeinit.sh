#! /bin/sh

XBEE_DRIVER_PATH=$(dirname $0)

XBEE_ADDRESS="10.0.0.0"
XBEE_NETMASK="255.255.255.0"

insmod ./6lowpan/6lowpan.ko

insmod ./ieee80214/ieee802154.ko

#modprobe 6lowpan
#modprobe ieee802154

insmod ${XBEE_DRIVER_PATH}/ieee802154_xbee.ko &&
ip link set hardwpan0 address de:ad:be:af:ca:fe:ba:be
ifconfig hardwpan0 up
ip link add link hardwpan0 name lowpan0 type lowpan
ip link set lowpan0 address a0:0:0:0:0:0:0:1
ip link set lowpan0 up
ip addr add fe80::0000:0:0:1/64 dev lowpan0
#ifconfig hardwpan0 ${XBEE_ADDRESS} netmask ${XBEE_NETMASK} up
killall ldisc_daemon  # Very strange I've to start kill and then start again otherwise recv does not work, cruel!
sleep 1
${XBEE_DRIVER_PATH}/ldisc_daemon /dev/ttyACM0 115200
#izattach /dev/ttyACM0
