#! /bin/sh

ifconfig lowpan0 down
ifconfig hardwpan0 down

pkill -f ldisc_daemon
rmmod ieee802154_xbee
rmmod 6lowpan
rmmod ieee802154
rmmod 6lowpan_iphc

echo "Xbee driver stopped"

