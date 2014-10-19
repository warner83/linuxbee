linuxbee
========

Linux Module For Xbee s1 cards, for the 3.X kernels.
Compatible with 6lowpan :)

Fork of the original driver from https://github.com/robbles/xbee-driver

So far it works only on the kernel 3.10, tested with Ubuntu 14.04LTS. 

USAGE:
- Compile it
- Load with driverieeeinit.sh
- Unload with driverieeeshutdown.sh

Status:
- Tested on Ubuntu 14.04 using two Xbee s1
- 6lowpan implemented in this version of the linux kernel does not guarantee much interoperability with other devices (read http://sourceforge.net/p/linux-zigbee/mailman/message/30476463/)

Future development:
- Port the whole thing to the last kernel with updated 6lowpan version
- Implement others command through AT_COMMANDs



