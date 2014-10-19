obj-m := ieee802154_xbee.o

KERNELDIR := /usr/src/linux-source-3.13.0

#KERNELDIR := /home/carlo/Downloads/linux-3.16.3/

PWD := $(shell pwd)

CC := /usr/bin/gcc 

default: driver ldisc_daemon

clean:
	rm -f ldisc_daemon *.ko *.o *.mod.c Module.symvers modules.order

driver:
	@echo "Building driver"
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
	
ldisc_daemon:
	@echo "Building line discipline daemon"
	$(CC) -o ldisc_daemon ldisc_daemon.c

