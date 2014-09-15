#! /bin/sh

ifconfig xbee0 down

pkill -f ldisc_daemon
rmmod n_xbee

echo "Xbee driver stopped"

