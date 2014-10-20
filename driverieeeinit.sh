#! /bin/sh

XBEE_DRIVER_PATH=$(dirname $0)

rmmod btusb 
rmmod bluetooth 
rmmod 6lowpan_iphc 

insmod ./ieee802154/ieee802154.ko

insmod ./ieee802154/6lowpan_iphc.ko

insmod ./ieee802154/6lowpan.ko

#modprobe 6lowpan
#modprobe ieee802154

insmod ${XBEE_DRIVER_PATH}/ieee802154_xbee.ko &&
ifconfig hardwpan0 down
ip link set hardwpan0 address 00:13:a2:00:40:b4:c5:65
ifconfig hardwpan0 up
ip link add link hardwpan0 name lowpan0 type lowpan
ip link set lowpan0 address 00:13:a2:00:40:b4:c5:65
ip link set lowpan0 up

#ip addr add fe80::0000:0:0:1/64 dev lowpan0
ip addr add fe80:0000:0000:0000:c30c:0000:0000:0001 dev lowpan0

#ip -6 addr add aaaa::1/64 dev lowpan0
ip -6 addr add aaaa::c30c:0:0:1/64 dev lowpan0

killall ldisc_daemon #Just in case... 
sleep 1
${XBEE_DRIVER_PATH}/ldisc_daemon /dev/ttyACM0 115200
#izattach /dev/ttyACM0 # this is the alternative if the tool is installed

