/*
 * Sample driver for HardMAC IEEE 802.15.4 devices
 *
 * Copyright (C) 2009 Siemens AG
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Written by:
 * Dmitry Eremin-Solenikov <dmitry.baryshkov@siemens.com>
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/if_arp.h>

#include <net/af_ieee802154.h>
#include <net/ieee802154_netdev.h>
#include <net/ieee802154.h>
#include <net/nl802154.h>
#include <net/wpan-phy.h>

#include <linux/module.h>
#include <linux/init.h>
#include <linux/moduleparam.h>

#include <linux/sched.h>
#include <linux/slab.h> /* kmalloc() */
#include <linux/errno.h>  /* error codes */
#include <linux/types.h>  /* size_t */
#include <linux/interrupt.h> /* mark_bh */
#include <linux/tty.h>


#include <linux/in.h>
#include <linux/netdevice.h>   /* struct device, and other headers */
#include <linux/etherdevice.h> /* eth_type_trans */
#include <linux/ip.h>          /* struct iphdr */
#include <linux/tcp.h>         /* struct tcphdr */
#include <linux/skbuff.h>
#include <linux/udp.h>
#include <linux/ip.h>
#include <net/checksum.h>

static int escape_into(char *dest, const void *srcarray, int len);


#include "n_xbee.h"

static struct wpan_phy *fake_to_phy(const struct net_device *dev)
{
	struct xbee_priv *priv = netdev_priv(dev);
	return priv->phy;
}

/**
 * fake_get_phy - Return a phy corresponding to this device.
 * @dev: The network device for which to return the wan-phy object
 *
 * This function returns a wpan-phy object corresponding to the passed
 * network device. Reference counter for wpan-phy object is incremented,
 * so when the wpan-phy isn't necessary, you should drop the reference
 * via @wpan_phy_put() call.
 */
static struct wpan_phy *fake_get_phy(const struct net_device *dev)
{
	struct wpan_phy *phy = fake_to_phy(dev);
	return to_phy(get_device(&phy->dev));
}

/**
 * fake_get_pan_id - Retrieve the PAN ID of the device.
 * @dev: The network device to retrieve the PAN of.
 *
 * Return the ID of the PAN from the PIB.
 */
static u16 fake_get_pan_id(const struct net_device *dev)
{
	BUG_ON(dev->type != ARPHRD_IEEE802154);

	return 0xabcd;
}

/**
 * fake_get_short_addr - Retrieve the short address of the device.
 * @dev: The network device to retrieve the short address of.
 *
 * Returns the IEEE 802.15.4 short-form address cached for this
 * device. If the device has not yet had a short address assigned
 * then this should return 0xFFFF to indicate a lack of association.
 */
static u16 fake_get_short_addr(const struct net_device *dev)
{
	BUG_ON(dev->type != ARPHRD_IEEE802154);

	return 0x1;
}

/**
 * fake_get_dsn - Retrieve the DSN of the device.
 * @dev: The network device to retrieve the DSN for.
 *
 * Returns the IEEE 802.15.4 DSN for the network device.
 * The DSN is the sequence number which will be added to each
 * packet or MAC command frame by the MAC during transmission.
 *
 * DSN means 'Data Sequence Number'.
 *
 * Note: This is in section 7.2.1.2 of the IEEE 802.15.4-2006
 *       document.
 */
static u8 fake_get_dsn(const struct net_device *dev)
{
	BUG_ON(dev->type != ARPHRD_IEEE802154);

	return 0x00; /* DSN are implemented in HW, so return just 0 */
}

/**
 * fake_assoc_req - Make an association request to the HW.
 * @dev: The network device which we are associating to a network.
 * @addr: The coordinator with which we wish to associate.
 * @channel: The channel on which to associate.
 * @cap: The capability information field to use in the association.
 *
 * Start an association with a coordinator. The coordinator's address
 * and PAN ID can be found in @addr.
 *
 * Note: This is in section 7.3.1 and 7.5.3.1 of the IEEE
 *       802.15.4-2006 document.
 */
static int fake_assoc_req(struct net_device *dev,
		struct ieee802154_addr *addr, u8 channel, u8 page, u8 cap)
{
	struct wpan_phy *phy = fake_to_phy(dev);

	mutex_lock(&phy->pib_lock);
	phy->current_channel = channel;
	phy->current_page = page;
	mutex_unlock(&phy->pib_lock);

	printk(KERN_ALERT "Association request confirmed!\n");

	/* We simply emulate it here */
	return ieee802154_nl_assoc_confirm(dev, fake_get_short_addr(dev),
			IEEE802154_SUCCESS);
}

/**
 * fake_assoc_resp - Send an association response to a device.
 * @dev: The network device on which to send the response.
 * @addr: The address of the device to respond to.
 * @short_addr: The assigned short address for the device (if any).
 * @status: The result of the association request.
 *
 * Queue the association response of the coordinator to another
 * device's attempt to associate with the network which we
 * coordinate. This is then added to the indirect-send queue to be
 * transmitted to the end device when it polls for data.
 *
 * Note: This is in section 7.3.2 and 7.5.3.1 of the IEEE
 *       802.15.4-2006 document.
 */
static int fake_assoc_resp(struct net_device *dev,
		struct ieee802154_addr *addr, u16 short_addr, u8 status)
{
	return 0;
}

/**
 * fake_disassoc_req - Disassociate a device from a network.
 * @dev: The network device on which we're disassociating a device.
 * @addr: The device to disassociate from the network.
 * @reason: The reason to give to the device for being disassociated.
 *
 * This sends a disassociation notification to the device being
 * disassociated from the network.
 *
 * Note: This is in section 7.5.3.2 of the IEEE 802.15.4-2006
 *       document, with the reason described in 7.3.3.2.
 */
static int fake_disassoc_req(struct net_device *dev,
		struct ieee802154_addr *addr, u8 reason)
{
	return ieee802154_nl_disassoc_confirm(dev, IEEE802154_SUCCESS);
}

/**
 * fake_start_req - Start an IEEE 802.15.4 PAN.
 * @dev: The network device on which to start the PAN.
 * @addr: The coordinator address to use when starting the PAN.
 * @channel: The channel on which to start the PAN.
 * @bcn_ord: Beacon order.
 * @sf_ord: Superframe order.
 * @pan_coord: Whether or not we are the PAN coordinator or just
 *             requesting a realignment perhaps?
 * @blx: Battery Life Extension feature bitfield.
 * @coord_realign: Something to realign something else.
 *
 * If pan_coord is non-zero then this starts a network with the
 * provided parameters, otherwise it attempts a coordinator
 * realignment of the stated network instead.
 *
 * Note: This is in section 7.5.2.3 of the IEEE 802.15.4-2006
 * document, with 7.3.8 describing coordinator realignment.
 */
static int fake_start_req(struct net_device *dev, struct ieee802154_addr *addr,
				u8 channel, u8 page,
				u8 bcn_ord, u8 sf_ord, u8 pan_coord, u8 blx,
				u8 coord_realign)
{
	struct wpan_phy *phy = fake_to_phy(dev);

	mutex_lock(&phy->pib_lock);
	phy->current_channel = channel;
	phy->current_page = page;
	mutex_unlock(&phy->pib_lock);

	/* We don't emulate beacons here at all, so START should fail */
	ieee802154_nl_start_confirm(dev, IEEE802154_INVALID_PARAMETER);
	return 0;
}

/**
 * fake_scan_req - Start a channel scan.
 * @dev: The network device on which to perform a channel scan.
 * @type: The type of scan to perform.
 * @channels: The channel bitmask to scan.
 * @duration: How long to spend on each channel.
 *
 * This starts either a passive (energy) scan or an active (PAN) scan
 * on the channels indicated in the @channels bitmask. The duration of
 * the scan is measured in terms of superframe duration. Specifically,
 * the scan will spend aBaseSuperFrameDuration * ((2^n) + 1) on each
 * channel.
 *
 * Note: This is in section 7.5.2.1 of the IEEE 802.15.4-2006 document.
 */
static int fake_scan_req(struct net_device *dev, u8 type, u32 channels,
		u8 page, u8 duration)
{
	u8 edl[27] = {};
	return ieee802154_nl_scan_confirm(dev, IEEE802154_SUCCESS, type,
			channels, page,
			type == IEEE802154_MAC_SCAN_ED ? edl : NULL);
}

static struct ieee802154_mlme_ops fake_mlme = {
	.assoc_req = fake_assoc_req,
	.assoc_resp = fake_assoc_resp,
	.disassoc_req = fake_disassoc_req,
	.start_req = fake_start_req,
	.scan_req = fake_scan_req,

	.get_phy = fake_get_phy,

	.get_pan_id = fake_get_pan_id,
	.get_short_addr = fake_get_short_addr,
	.get_dsn = fake_get_dsn,
};

static int ieee802154_fake_open(struct net_device *dev)
{
	printk(KERN_ALERT "[NET open called ]\n");
	
	
	/* request_region(), request_irq(), ....  (like fops->open) */

	/* 
	* Assign the hardware address of the board: use "\0XBEEx", where
	* x is 0 or 1. The first byte is '\0' to avoid being a multicast
	* address (the first byte of multicast addrs is odd).
	*/
    	//TODO: This should be set to the actual Xbee address somehow, even though it's never used
	//memcpy(dev->dev_addr, "\0XBEE111", 8);
	//memcpy(dev->perm_addr, "\0XBEE111", 8);

	netif_start_queue(dev);
	return 0;
}

static int ieee802154_fake_close(struct net_device *dev)
{
	printk(KERN_ALERT "[NET release called ]\n");

	netif_stop_queue(dev);
	return 0;
}

/*
 * Adds in the checksum and sends the packet off to the serial driver.
 */
static void xbee_hw_tx(char *frame, int len, struct net_device *dev)
{

	int i;
	unsigned char checksum;
	int actual;
	
	printk(KERN_ALERT "[NET hardware_tx called, %d bytes ]\n", len);
	
	//Add start delimiter
	*frame = 0x7E;
	
	//Checksum
	checksum = 0;
	for(i=3; i<(len-1); i++) {
		if(frame[i] == 0x7D) {
			i++;
			checksum += (unsigned char) frame[i] ^ 0x20;
		} else {
			checksum += ((unsigned char)frame[i]);
		}
	}
	checksum = 0xFF - checksum;
	if (checksum == 0x7E || checksum == 0x7D || checksum == 0x11 || checksum == 0x13) {
		frame[len-1] = 0x7D;
		frame[len] = checksum ^ 0x20;
		len++;
	} else {
		frame[len-1] = checksum;
	}
	
	//Send the data to serial driver
	if(main_tty != NULL) {
			
		//printk(KERN_ALERT "Setting DO_WRITE_WAKEUP\n");
		//main_tty->flags |= (1 << TTY_DO_WRITE_WAKEUP);
		
		set_bit(TTY_DO_WRITE_WAKEUP, &main_tty->flags);

		printk(KERN_ALERT "Writing the data to tty...\n");

		print_hex_dump(KERN_ALERT, "", DUMP_PREFIX_OFFSET, 16, 1, frame, len, 0);
	
		actual = main_tty->driver->ops->write(main_tty, frame, len);
	 	
		if(actual != len) { 
			printk(KERN_ALERT "Write failed!\n");
		}
	} else {
		printk(KERN_ALERT "No tty is attached!\n");
	}
	
	//printk(KERN_ALERT "Freeing the frame memory\n");
	kfree(frame);
}


static netdev_tx_t ieee802154_fake_xmit(struct sk_buff *skb,
					      struct net_device *dev)
{
	struct ieee802154_mac_cb* cb;

	int framelen, datalen;
	char *frame;
	
	struct xbee_priv *priv = netdev_priv(dev);
	
	struct xbee_tx_header header = {
		.length = 0x00,
		.api_id = 0x00,
		.frame_id = 0x02,
		.options = 0x04
	};

        dev->stats.tx_packets++;
        dev->stats.tx_bytes += skb->len;

		
	if(main_tty == NULL) {
		printk(KERN_ALERT "No tty attached!\nMake sure ldisc_daemon is running\n");
		return 0;
	}
		
	printk(KERN_ALERT "[NET tx called, %d bytes in sk_buff, pkt type %hu, protocol %hu]\n", skb->len, skb->pkt_type, skb->protocol);
   
	cb = mac_cb(skb);	
 
	//header.address = cpu_to_be64(0x000000000000FFFF); // So far broadcast

	memcpy(&header.address, &(cb->da.hwaddr), 8);
	
	printk(KERN_ALERT "DST addr %u\n", cb->da.hwaddr[0]);

	header.length = cpu_to_be16(skb->len + 10);
    	datalen = skb->len;

	// Allocate buffer to hold entire serial frame, including start byte and checksum
	frame = kmalloc(sizeof(struct xbee_tx_header) + (2*datalen) + 2, GFP_KERNEL);

	/* Assemble the frame */
    
	/* Escaping the XBee header */
    	printk(KERN_ALERT "Adding XBee header- length is %lu\n", sizeof(header) + 1);
	framelen = escape_into((frame + 1), &header, sizeof(header));
    
    	printk(KERN_ALERT "Adding data - length is %u\n", datalen);
	framelen += escape_into((frame + framelen + 1), (skb->data), datalen);

	framelen++; // Escape char at the beginning
	
	/* save the timestamp */
	dev->trans_start = jiffies; 
	
	//Stop the interface from sending more data, get the spinlock, and send it!
	netif_stop_queue(dev);
	
	spin_lock(&priv->lock);
	
	xbee_hw_tx(frame, framelen, dev);
	
	//Free the skbuff, we've copied all the data out
	dev_kfree_skb(skb);

	spin_unlock(&priv->lock);	

	return NETDEV_TX_OK;
}



/*
 * Copies bytes from one array into another, escaping them if necessary.
 * Returns the size of the new array, including character escapes.
 * The destination array MUST be at least twice the size of the source
 */
static int escape_into(char *dest, const void *srcarray, int len) {
	int i, j=0;
	
	const char *src = srcarray;
	
	for(i=0; i<len; i++) {
		char unesc = *(src + i);
		// We support API mode 1 (escape characters is for API 2, if we want to enable mode 2 we need to implement escape management at RX)
		if( unesc == 0x7D || unesc == 0x7E || unesc == 0x11 || unesc == 0x13) {
			dest[j] = 0x7D;
			dest[j+1] = unesc ^ 0x20;
			j+=2;
		} else {
			dest[j] = unesc;
			j++;
		}	
	}
	return j;
}


static int ieee802154_fake_ioctl(struct net_device *dev, struct ifreq *ifr,
		int cmd)
{
	struct sockaddr_ieee802154 *sa =
		(struct sockaddr_ieee802154 *)&ifr->ifr_addr;
	u16 pan_id, short_addr;

	switch (cmd) {
	case SIOCGIFADDR:
		/* FIXME: fixed here, get from device IRL */
		pan_id = fake_get_pan_id(dev);
		short_addr = fake_get_short_addr(dev);
		if (pan_id == IEEE802154_PANID_BROADCAST ||
		    short_addr == IEEE802154_ADDR_BROADCAST)
			return -EADDRNOTAVAIL;

		sa->family = AF_IEEE802154;
		sa->addr.addr_type = IEEE802154_ADDR_SHORT;
		sa->addr.pan_id = pan_id;
		sa->addr.short_addr = short_addr;
		return 0;
	}
	return -ENOIOCTLCMD;
}

static int ieee802154_fake_mac_addr(struct net_device *dev, void *p)
{
	return -EBUSY; /* HW address is built into the device */
}

static const struct net_device_ops fake_ops = {
	.ndo_open		= ieee802154_fake_open,
	.ndo_stop		= ieee802154_fake_close,
	.ndo_start_xmit		= ieee802154_fake_xmit,
	.ndo_do_ioctl		= ieee802154_fake_ioctl,
	.ndo_set_mac_address	= ieee802154_fake_mac_addr,
};

static void ieee802154_fake_destruct(struct net_device *dev)
{
	struct wpan_phy *phy = fake_to_phy(dev);

	wpan_phy_unregister(phy);
	free_netdev(dev);
	wpan_phy_free(phy);
}

static void ieee802154_fake_setup(struct net_device *dev)
{
	struct xbee_priv *priv;
	
    	/*
        	* Then, initialize the priv field. This encloses the statistics
        	* and a few private fields.
    	*/
    	priv = netdev_priv(dev);
    	memset(priv, 0, sizeof(struct xbee_priv));
    	spin_lock_init(&priv->lock);
	
	dev->addr_len		= IEEE802154_ADDR_LEN;
	memset(dev->broadcast, 0xff, IEEE802154_ADDR_LEN);
	dev->features		= NETIF_F_HW_CSUM;
	dev->needed_tailroom	= 2; /* FCS */
	dev->mtu		= 127;
	dev->tx_queue_len	= 10;
	dev->type		= ARPHRD_IEEE802154;
	dev->flags		= IFF_NOARP | IFF_BROADCAST;
	dev->watchdog_timeo	= 0;
	dev->destructor		= ieee802154_fake_destruct;
	
}


static int ieee802154fake_probe(struct platform_device *pdev)
{
	struct net_device *dev;
	struct xbee_priv *priv;
	struct wpan_phy *phy = wpan_phy_alloc(0);
	int err;

	if (!phy)
		return -ENOMEM;

	dev = alloc_netdev(sizeof(struct xbee_priv), "hardwpan%d", ieee802154_fake_setup);

	if (!dev) {
		wpan_phy_free(phy);
		return -ENOMEM;
	}

	memcpy(dev->dev_addr, "\xba\xbe\xca\xfe\xde\xad\xbe\xef",
			dev->addr_len);

	/*
	 * For now we'd like to emulate 2.4 GHz-only device,
	 * both O-QPSK and CSS
	 */
	/* 2.4 GHz O-QPSK 802.15.4-2003 */
	phy->channels_supported[0] |= 0x7FFF800;
	/* 2.4 GHz CSS 802.15.4a-2007 */
	phy->channels_supported[3] |= 0x3fff;

	phy->transmit_power = 0xbf;

	dev->netdev_ops = &fake_ops;
	dev->ml_priv = &fake_mlme;

	priv = netdev_priv(dev);
	priv->phy = phy;

	wpan_phy_set_dev(phy, &pdev->dev);
	SET_NETDEV_DEV(dev, &phy->dev);

	platform_set_drvdata(pdev, dev);

	priv->rbuff = kmalloc(XBEE_MAXFRAME, GFP_KERNEL);
        priv->rcount = 0;
        priv->frame_status = UNFRAMED;
        priv->frame_len = 0;
	priv->escaped = 0;

	xbee_dev = dev; // This should be removed in a future!

	err = wpan_phy_register(phy);
	if (err)
		goto out;

	err = register_netdev(dev);
	if (err < 0)
		goto out;

	dev_info(&pdev->dev, "Added ieee802154 HardMAC hardware\n");
	return 0;

out:
	unregister_netdev(dev);
	return err;
}

static int ieee802154fake_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct xbee_priv *priv = netdev_priv(xbee_dev);

	kfree(priv->rbuff);

	unregister_netdev(dev);
	return 0;
}

static struct platform_device *ieee802154fake_dev;

static struct platform_driver ieee802154fake_driver = {
	.probe = ieee802154fake_probe,
	.remove = ieee802154fake_remove,
	.driver = {
			.name = "ieee802154xbee",
			.owner = THIS_MODULE,
	},
};


/*
***************LDISC Functions*************
*/


/*
* The following routines are called from above (user space/tty).
*/
static int n_xbee_open(struct tty_struct *tty) {


	// Merge the two structures!!! THIS IS A NULL POINTER	
	//struct xbee_priv *priv = netdev_priv(xbee_dev);
	
	//printk(KERN_ALERT "OPEN CALLED\n");
	
	// Don't allow more than one tty to be attached
	if( main_tty != NULL )
		return -EPERM;
	
	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	if (tty->ops->stop)
		tty->ops->stop(tty);
	
	main_tty = tty;
	printk(KERN_ALERT "Attached to a tty!\n\n");
	
        tty_driver_flush_buffer(tty);
        tty_ldisc_flush(tty);

	// For kernels < 3.10 it needs to be set
	tty->receive_room = 4096;
	
	/*priv->rbuff = kmalloc(XBEE_MAXFRAME, GFP_KERNEL);
	priv->rcount = 0;
	priv->frame_status = UNFRAMED;
	priv->frame_len = 0;*/
	
	return 0;
}

static void n_xbee_close(struct tty_struct *tty) {
	
	//struct xbee_priv *priv = netdev_priv(xbee_dev);
	
	printk(KERN_ALERT "CLOSE CALLED\n");

        //tty_driver_flush_buffer(tty);
	//tty_ldisc_flush(tty);
	
	module_put(THIS_MODULE);
	
	main_tty = NULL;

	//kfree(priv->rbuff);
	
}

static void n_xbee_flush_buffer(struct tty_struct *tty) {
	printk(KERN_ALERT "FLUSH_BUFFER CALLED\n");
        tty_driver_flush_buffer(tty);
}

static ssize_t	n_xbee_chars_in_buffer(struct tty_struct *tty) {
	printk(KERN_ALERT "CHARS_IN_BUFFER CALLED\n");

	return 0;
}

static int n_xbee_ioctl(struct tty_struct * tty, struct file * file, unsigned int cmd, unsigned long arg) {
	printk(KERN_ALERT "IOCTL CALLED\n");

	return -EPERM;
}


static unsigned int n_xbee_poll(struct tty_struct *tty, struct file *file, struct poll_table_struct *poll) {
	printk(KERN_ALERT "POLL CALLED\n");

	return 0;
}
	
/*
* The following routines are called from below (serial port driver).
*/




static char checksum_validate(unsigned char *buffer, int len, unsigned char checksum) {
	int i = 0;
	while(len--) {
		checksum += buffer[i++];
	}
	if(checksum == 0xFF)
		return true;
	printk(KERN_ALERT "[XBEE] CSUM should be 0xFF, is 0x%x\n", checksum);
	return false;
}

/*
 * Encapsulate a packet with UDP/IP and hand off to upper networking layers
 */
void xbee_rx(struct net_device *dev, unsigned char *data, int len, unsigned char* addr) {

	struct xbee_priv *priv = netdev_priv(dev);
	struct sk_buff *skb;
	struct ieee802154_mac_cb* cb;

	//struct iphdr *ih;
    	int packet_stat;

	skb = dev_alloc_skb(len /*+ sizeof(struct udphdr) + sizeof(struct iphdr)*/);
	
	if (!skb) {
		if (printk_ratelimit())
			printk(KERN_NOTICE "[NET] xbee rx: low on mem - packet dropped\n");
		priv->stats.rx_dropped++;
		return;
	}
	//skb_reserve (skb, sizeof(struct iphdr) + sizeof(struct udphdr));
	
	// Put all the data into a new socket buffer
	memcpy(skb_put(skb, len), data, len);
	
    	// Add an IP header and pretend it's a broadcast packet
    	/*ih = (struct iphdr*)skb_push(skb, sizeof(struct iphdr));
	ih->version = 4;
	ih->ihl = 5;
	ih->tos = 0;
	ih->tot_len = htons(len + sizeof(struct iphdr));
	ih->id = 555;
	ih->frag_off = 0;
	ih->ttl = 64;
	ih->protocol = 17;
	ih->check = 0x0000;
	ih->saddr = 0x00000000L;
	ih->daddr = 0xFFFFFFFFL;
	skb_reset_network_header(skb);
	
	ih->check = ip_fast_csum((unsigned char *)ih, ih->ihl);
	*/	

	skb->dev = dev;

        //skb->protocol = htons(ETH_P_IPV6);
        //skb->pkt_type = PACKET_HOST;
	
	skb->protocol = htons(ETH_P_IEEE802154);
        skb_reset_mac_header(skb);

	cb = mac_cb(skb);

        cb->sa.addr_type = IEEE802154_ADDR_LONG;
        cb->sa.pan_id = fake_get_pan_id(dev);

	//const unsigned short int fakeadd = 0x00000000L;

        memcpy(&(cb->sa.hwaddr), addr, 8);

	skb->ip_summed = CHECKSUM_UNNECESSARY; // don't check it (does this make any difference?)
	
	packet_stat = netif_rx(skb);
	
	if(packet_stat == NET_RX_SUCCESS) {
		printk(KERN_ALERT "[NET] Packet received successfully\n");
	} else if(packet_stat == NET_RX_DROP) {
		printk(KERN_ALERT "[NET] Packet was dropped!\n");
	}
	
	priv->stats.rx_packets++;
	priv->stats.rx_bytes += len;

}





/*
 * Receive a packet from device and send data to the right place
 */
void xbee_receive_packet(struct net_device *dev, unsigned char *data, int len)
{
	
	struct xbee_priv *priv = netdev_priv(dev);

	printk(KERN_ALERT "[XBEE] receive_packet called ]\n");

	switch(*data) {
	
		case IEEE802154_RECEIVE_PACKET:
			printk(KERN_ALERT "[XBEE] 802154 packet received \n");
			xbee_rx(dev, (data + 11), len - 11, data + 1 );
                        goto out;
	
		case ZIGBEE_RECEIVE_PACKET:
			printk(KERN_ALERT "[XBEE] Zigbee Receive Packet Frame received\n");
			if(len<13) {
				printk(KERN_ALERT "[XBEE] Packet too short to be a Receive Packet Frame!\n");
				goto out;
			}

			xbee_rx(dev, (data + 12), len - 12, data + 1 );
			goto out;

		case ZIGBEE_TRANSMIT_STATUS:
			//printk(KERN_ALERT "[XBEE] Zigbee Transmit Status Frame received\n");
			if(len != 7) {
				printk(KERN_ALERT "[XBEE] Packet is wrong length for a Transmit Status Packet!\n");
				goto out;
			}

			if(data[5] == TRANSMIT_SUCCESS) {
				printk(KERN_ALERT "[XBEE] Packet delivery reported as success!\n");
                //TODO: report which packet by reading message (and notify super-paranoid drivers?)
			} else {
                if(data[5] == TRANSMIT_INVALID) {
                    printk(KERN_ALERT "[XBEE] Transmit failed: invalid endpoint\n");
                } else if(data[5] == TRANSMIT_ADDRNOTFOUND) {
                    printk(KERN_ALERT "[XBEE] Transmit failed: address not found\n");
                } else {
                    printk(KERN_ALERT "[XBEE] Packet delivery failed with status %d\n", data[5]);
                }
			}

			goto out;

		case NODE_IDENTIFICATION_INDICATOR:
			printk(KERN_ALERT "[XBEE] NODE_IDENTIFICATION_INDICATOR Frame received\n");
			break;
		case MODEM_STATUS:
			printk(KERN_ALERT "[XBEE] Zigbee Modem Status Frame received\n");
			break;
		case AT_COMMAND:
			printk(KERN_ALERT "[XBEE] Zigbee AT_COMMAND Frame received\n");
			break;
		case AT_COMMAND_QUEUE_PARAMETER_VALUE:
			printk(KERN_ALERT "[XBEE] AT_COMMAND_QUEUE_PARAMETER_VALUE Frame received\n");
			break;
		case AT_COMMAND_RESPONSE:
			printk(KERN_ALERT "[XBEE] AT_COMMAND_RESPONSE Frame received\n");
			break;
		case REMOTE_COMMAND_REQUEST:
			printk(KERN_ALERT "[XBEE] REMOTE_COMMAND_REQUEST Frame received\n");
			break;
		case REMOTE_COMMAND_RESPONSE:
			printk(KERN_ALERT "[XBEE] REMOTE_COMMAND_RESPONSE Frame received\n");
			break;
		case ZIGBEE_TRANSMIT_REQUEST:
			printk(KERN_ALERT "[XBEE] ZIGBEE_TRANSMIT_REQUEST Frame received\n");
			break;
		case EXPLICIT_ADDRESSING_COMMAND_FRAME:
			printk(KERN_ALERT "[XBEE] EXPLICIT_ADDRESSING_COMMAND_FRAME received\n");
			break;
		case ZIGBEE_EXPLICIT_RX_INDICATOR:
			printk(KERN_ALERT "[XBEE] ZIGBEE_EXPLICIT_RX_INDICATOR Frame received\n");
			break;
		case ZIGBEE_IO_DATA_SAMPLE_RX_INDICATOR:
			printk(KERN_ALERT "[XBEE] ZIGBEE_IO_DATA_SAMPLE_RX_INDICATOR Frame received\n");
			break;
		case XBEE_SENSOR_READ_INDICATOR:
			printk(KERN_ALERT "[XBEE] XBEE_SENSOR_READ_INDICATOR Frame received\n");
			break;
		default:
		printk(KERN_ALERT "[XBEE] UNKNOWN Frame Received : errors %lu\n", priv->stats.rx_dropped++);
	}
	

	out:
	  return;
}



/*
 * Called from the serial driver when new received data is ready
 */
static void	n_xbee_receive_buf(struct tty_struct *tty, const unsigned char *cp, char *fp, int count) {
	
	unsigned char temp;
	struct xbee_priv *priv = netdev_priv(xbee_dev);
	unsigned char checksum;
	
	//printk(KERN_ALERT "[XBEE] RECEIVE_BUF CALLED  %d chars received\n", count);

	while(count--) {
		
		temp = *cp++;

		
		//Start a new frame if 0x7E received, even if we seem to be in the middle of one
		if(temp == 0x7E) {
			printk(KERN_ALERT "[XBEE] INIT FRAME : %02X\n", temp );
			priv->frame_status = FRAME_LEN_HIGH;
			priv->frame_len = 0;
			priv->rcount = 0;
			continue;
		}

		//Escape characters after 0x7D byte
		if(temp == 0x7D) {
			priv->escaped=1;
			continue;
		}

		// The last one was an escaped char
		if(priv->escaped){
			temp = temp ^ 0x20;
			printk(KERN_ALERT "[XBEE] ESCAPED : %02X\n", temp );
			priv->escaped=0;
		}

	
		switch(priv->frame_status) {

            		//FIXME: All the Xbee packets seem to have two extra bytes on the end. why...?
			case UNFRAMED:
				//printk(KERN_ALERT "Unframed data received: 0x%x\n", temp);
				break;
				
			case FRAME_LEN_HIGH:
				printk(KERN_ALERT "[XBEE] LEN HIGH : %02X\n", temp );
				priv->frame_len = (temp) << 8;
				priv->frame_status = FRAME_LEN_LOW;
				break;
				
			case FRAME_LEN_LOW:
				printk(KERN_ALERT "[XBEE] LEN LOW : %02X\n", temp );
				priv->frame_len |= (temp);
				priv->frame_status = FRAME_DATA;
				priv->rcount = 0;
				break;
				
			case FRAME_DATA:
				//Add to the frame buffer
				priv->rbuff[priv->rcount++] = (temp);
				
				//Is the frame done?
				if(priv->rcount == (priv->frame_len )) {
					priv->frame_status = FRAME_CHECKSUM;
				} 
				//Is the frame FUBAR?
				if(priv->rcount > XBEE_MAXFRAME || priv->rcount > priv->frame_len || priv->frame_len > XBEE_MAXFRAME) {
					priv->rcount = 0;
					priv->frame_status = UNFRAMED;
					printk(KERN_ALERT "[XBEE] A frame got pwned!! rcount %lu max %lu frame_len %lu : %lu errors\n",  priv->rcount, XBEE_MAXFRAME, priv->frame_len,priv->stats.rx_dropped++);
				} 
				break;
				
			case FRAME_CHECKSUM:
				checksum = temp;
				
				printk(KERN_ALERT "[XBEE] Frame checksum : %02X\n", checksum);
				print_hex_dump(KERN_ALERT, "", DUMP_PREFIX_OFFSET, 16, 1, priv->rbuff, priv->frame_len, 0);
		
				if(checksum_validate(priv->rbuff, priv->frame_len , checksum)) {
					
					//Hand off to xbee_receive_packet for networking/packet ID
					xbee_receive_packet(xbee_dev, priv->rbuff, priv->frame_len );
				} else {
					printk(KERN_ALERT "[XBEE] CHECKSUM FAIL : %lu errors\n", priv->stats.rx_dropped++);
				}
				priv->frame_status = UNFRAMED;
				priv->frame_len = 0;
				priv->rcount = 0;

				break;
				
			default:
				printk(KERN_ALERT "[XBEE] WARNING - Undefined Frame Status!!\n");
				priv->frame_status = UNFRAMED;
		}
		
		
	}
	
}

static void	n_xbee_write_wakeup(struct tty_struct *tty) {

	printk(KERN_ALERT "[XBEE] write wakeup\n");

	//main_tty->flags &= ~(1 << TTY_DO_WRITE_WAKEUP);

	clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
	
	netif_wake_queue(xbee_dev);
}


/*
 * Read on the tty.
 * Unused
 */
static ssize_t
n_xbee_read(struct tty_struct *tty, struct file *file,
                 unsigned char __user *buf, size_t count)
{
        return -EAGAIN;
}

/*
 * Write on the tty.
 * Unused
 */
static ssize_t
n_xbee_write(struct tty_struct *tty, struct file *file,
                  const unsigned char *buf, size_t count)
{
        return -EAGAIN;
}

struct tty_ldisc_ops n_xbee_ldisc = {
    .owner           = THIS_MODULE,
	.magic           = TTY_LDISC_MAGIC,
	.name            = "n_xbee",
	.open            = n_xbee_open, /* TTY */
	.close           = n_xbee_close, /* TTY */
	.flush_buffer    = n_xbee_flush_buffer, /* TTY */
	.chars_in_buffer = n_xbee_chars_in_buffer, /* TTY */
	.read            = n_xbee_read, /* TTY */
	.write           = n_xbee_write, /* TTY */
	.ioctl           = n_xbee_ioctl, /* TTY */
	.set_termios     = NULL, /* FIXME no support for termios setting yet */
	.poll            = n_xbee_poll,
	.receive_buf     = n_xbee_receive_buf, /* Serial driver */
	.write_wakeup    = n_xbee_write_wakeup /* Serial driver */
};


/*
*************** End of LDISC Functions*************
*/



void xbee_cleanup(void)
{
	
	if (xbee_dev) {
		unregister_netdev(xbee_dev);
		
		free_netdev(xbee_dev);
	}
	
	return;
}


int xbee_init_module(void)
{
	int result, ret=0;

	//Register the line discipline
    // TODO: actually use a well-known discipline instead of over-riding
    // a relatively pointless one (N_SLCAN?)
	result = tty_register_ldisc(N_XBEE, &n_xbee_ldisc);
	if(result) {
		printk(KERN_ALERT "Registering the line discipline failed with error %d\n", result);
		return result;
	}
	
	/* Allocate the devices */
	//xbee_dev = alloc_netdev(sizeof(struct xbee_priv), "xbee%d",
	//			     xbee_init);
	
	
	if (xbee_dev == NULL)
		goto out;

	ret = -ENODEV;
	
	//if ((result = register_netdev(xbee_dev)))
	//	printk("xbee: error %i registering device \"%s\"\n", result, xbee_dev->name);
	//else
	//	ret = 0;
	
	
out:
	if (ret) 
		xbee_cleanup();
	return ret;
}


static __init int fake_init(void)
{
	ieee802154fake_dev = platform_device_register_simple(
			"ieee802154xbee", -1, NULL, 0);
	xbee_init_module();

	return platform_driver_register(&ieee802154fake_driver);
}

static __exit void fake_exit(void)
{
	tty_unregister_ldisc(N_XBEE);
	platform_driver_unregister(&ieee802154fake_driver);
	platform_device_unregister(ieee802154fake_dev);
}

module_init(fake_init);
module_exit(fake_exit);
MODULE_LICENSE("GPL");

MODULE_ALIAS_LDISC(N_XBEE);
