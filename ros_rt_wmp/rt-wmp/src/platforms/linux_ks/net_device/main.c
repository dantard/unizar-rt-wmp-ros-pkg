/*-------------------------------------------------------------------------
 *--------------------------- RT-WMP IP INTERFACE -------------------------
 *-------------------------------------------------------------------------
 *
 * File: main.c
 * Authors: Rubén Durán
 *          Danilo Tardioli
 *-------------------------------------------------------------------------
 *  Copyright (C) 2011, Universidad de Zaragoza, SPAIN
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *-----------------------------------------------------------------------*/
#include <linux/kernel.h>
#include <asm/delay.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/kthread.h>
#include <linux/inetdevice.h>    // To get the IP address
#include <linux/ip.h>            // struct iphdr
#include <linux/tcp.h>           // struct tcphdr
#include <linux/udp.h>           // struct udphdr
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>

#include "ioctl.h"
#include "conf.h"
#include "../include/ioctl_interface.h"
#include "../include/cross_space.h"
#include "core/interface/wmp_interface.h"
#include "core/include/frames.h"
#include "core/include/queues.h"

#define IP_TRAFFIC_PORT 0
#define IP_TRAFFIC_PRIORITY 0

/* Interfaces */
static struct net_device *interface;
static dev_t first; // Global variable for the first device number
static struct cdev c_dev; // Global variable for the character device structure
static struct class *cl; // Global variable for the device class

/* Mutex */
DEFINE_MUTEX(chr_dev_mtx);

/* Private data for the interface */
typedef struct {
	struct task_struct *rx_thread, *queue_thread;
	tpConfig conf;
	__be32 net;
} priv_data;

/* Net Interface operations */
static int interface_open(struct net_device *netdev);
static int interface_close(struct net_device *netdev);
static int interface_tx(struct sk_buff *skb, struct net_device *netdev);
static int interface_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd);
static void interface_tx_timeout(struct net_device *dev);

/* From 2.6.30 on, all operations are placed inside a common structure */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30)
static const struct net_device_ops interfaz_netdev_ops = { .ndo_open =
		interface_open, .ndo_stop = interface_close, .ndo_start_xmit =
		interface_tx, .ndo_do_ioctl = interface_ioctl, .ndo_tx_timeout =
		interface_tx_timeout, };
#endif

/* Char device operations */
static int device_open(struct inode *inode, struct file *file);
static int device_release(struct inode *inode, struct file *file);
static int ch_open(struct inode *inode, struct file *file);
static ssize_t ch_read(struct file *filp, char *buffer, size_t length,
		loff_t * offset);
static ssize_t ch_write(struct file *filp, const char *buff, size_t len, loff_t * off);

/* Char device operations definition*/
static struct file_operations fops = { .open = device_open, .release =
		device_release, .read = ch_read, .write = ch_write, .open = ch_open, .owner = THIS_MODULE,
};

/**
 *  Creates a skb with the data in 'm' and calls netif_receive_skb()
 */
int rx_msg(char * p, unsigned int size) {
	struct sk_buff *skb;

	/* Create the skb */
	skb = dev_alloc_skb(ETH_HLEN + size + NET_IP_ALIGN);
	if (skb == NULL) {
		printk(KERN_ERR "Error: Can't allocate skb (rx_msg)\n");
		interface->stats.rx_dropped++;
		return -ENOMEM;
	}

	/* Data and info from the Msg to the skb */
	skb_reserve(skb, NET_IP_ALIGN);

	skb_put(skb, ETH_HLEN + size);

	/* MAC */
	((struct ethhdr *) skb->data)->h_proto = htons(ETH_P_IP);
	memcpy(((struct ethhdr *) skb->data)->h_dest, interface->dev_addr, ETH_ALEN);

	/* IP (What came through RT-WMP) */
	memcpy(skb->data + ETH_HLEN, p, size);

	/* Other skb properties */
	skb->dev = interface;
	skb->protocol = eth_type_trans(skb, interface);
	skb->ip_summed = CHECKSUM_UNNECESSARY;
	memset(skb->cb, 0, sizeof(skb->cb));

	/* RX stats */
	interface->stats.rx_packets++;
	interface->stats.rx_bytes += skb->len;

	/* To the linux network layer */
	netif_receive_skb(skb);

	return 0;
}

/* THREADS */

/**
 *  This thread starts the transmission queue periodically in case it has been
 *  stopped by the flow control
 */
int queue_thread_func(void *data) {
	while (!kthread_should_stop()) {
		msleep(50);
		netif_wake_queue(interface);
		msleep(50);
	}
	do_exit(0);
}

void ussleep(unsigned int usecs) {
	unsigned long timeout = ((usecs * (msecs_to_jiffies(1) + 1)) / (int)1000);
	while (timeout) {
		timeout = schedule_timeout_uninterruptible(timeout);
	}
}

/**
 *  Thread for the reception of normal and broadcast messages (queue number 1)
 */
int rx_thread_func(void *data) {
	signed char priority;
	char * p;
	int ret, id;
	unsigned int size;
	unsigned char src;
	while (!kthread_should_stop()) {
		if (wmpGetNumOfElementsInRXQueue(IP_TRAFFIC_PORT)<5){
			ussleep(1000);
		}
		id = wmpPopDataTimeout(IP_TRAFFIC_PORT, &p, &size, &src, &priority, 1000);
		if (id != -1) {
			ret = rx_msg(p, size);
			wmpPopDataDone(id);
		} else {
			wmpPopDataDone(id);
			ussleep(1000);
		}
	}
	do_exit(0);
}


/* Char device */

static int device_open(struct inode *inode, struct file *file) {
	return 0;
}

static int device_release(struct inode *inode, struct file *file) {
	return 0;
}

static int ch_open(struct inode *inode, struct file *file){
	return 0;
}

static ssize_t ch_read(struct file *filp, char *buffer, size_t length,
		loff_t * offset) {
	int id;
	cross_space_data_t * hdr = (cross_space_data_t *) buffer;
	if (length == 1) {
		id = wmpWaitData(hdr->port, hdr->timeout);
		if (id >= 0) {
			hdr->size = wmp_queue_rx_get_elem_size(id);
			hdr->priority = wmp_queue_rx_get_elem_priority(id);
			hdr->src = wmp_queue_rx_get_elem_source(id);
			hdr->id = id;
			return 1;
		}else{
			return 0;
		}
	} else{
		int id = hdr->id;
		if (id >= 0){
			int size = hdr->size;
			char * p = wmp_queue_rx_get_elem_data(id);
			if (p!=0){
				memcpy(buffer, p, size);
			}
			wmp_queue_rx_set_elem_done(id);
		}
		return 1;
	}
}

static ssize_t ch_write(struct file *filp, const char *buff, size_t len, loff_t * off){
	static cross_space_data_t hdr;
	if (len == 1){
		mutex_lock(&chr_dev_mtx);
		hdr = (*((cross_space_data_t *) buff));
	}else{
		wmpPushData(hdr.port, (char *) buff, hdr.size, hdr.dest, hdr.priority);
		mutex_unlock(&chr_dev_mtx);
	}
	return len;
}

static int chdev_init(void) {
	printk(KERN_INFO "RT-WMP char device registered");
	if (alloc_chrdev_region(&first, 0, 1, "Shweta") < 0) {
		return -1;
	}
	if ((cl = class_create(THIS_MODULE, "chardrv")) == NULL) {
		unregister_chrdev_region(first, 1);
		return -1;
	}
	if (device_create(cl, NULL, first, NULL, "rt-wmp") == NULL) {
		class_destroy(cl);
		unregister_chrdev_region(first, 1);
		return -1;
	}
	cdev_init(&c_dev, &fops);
	if (cdev_add(&c_dev, first, 1) == -1) {
		device_destroy(cl, first);
		class_destroy(cl);
		unregister_chrdev_region(first, 1);
		return -1;
	}
	return 0;
}

static void  chdev_finish(void) /* Destructor */
{
  cdev_del(&c_dev);
  device_destroy(cl, first);
  class_destroy(cl);
  unregister_chrdev_region(first, 1);
  printk(KERN_INFO "RT-WMP char device unregistered");
}

static int __net_init load_interface(void) {
	int err = -ENOMEM;
	priv_data *priv;
	/* The interface is created and configured */
	interface = alloc_netdev(sizeof(priv_data), "wmp%d", ether_setup);
	if ( interface == NULL ) {
		printk(KERN_ERR "Error while allocating space for the interface\n");
		return err;
	}

	priv = netdev_priv(interface);

	/* Interface operations */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30)
	interface->open = interface_open;
	interface->stop = interface_close;
	interface->hard_start_xmit = interface_tx;
	interface->do_ioctl = interface_ioctl;
	interface->ndo_tx_timeout = interface_tx_timeout,
#else
	interface->netdev_ops = &interfaz_netdev_ops;
#endif

	/* Interface properties */
	interface->tx_queue_len = 5;
	interface->flags |= IFF_NOARP;
	interface->flags &= ~IFF_MULTICAST;

	//XXX: dangerous point
	interface->mtu = 1500 - sizeof(Token_Hdr) - sizeof(Message) - 60;

	/* The interface is registered */
	err = register_netdev(interface);
	if (err) {
		printk(KERN_ERR "Error %i while registering the interface \"%s\"\n", err, interface->name);
		free_netdev(interface);
		return err;
	}

	if (chdev_init()!=0){
		printk(KERN_ERR "Error while registering the RT-WMP char device \n");
		return err;
	}

	printk( KERN_INFO "Interface \"%s\" loaded...\n", interface->name);
	return 0;
}

static void __net_exit unload_interface(void) {

	if (interface) {
		unregister_netdev(interface);
		free_netdev(interface);
	}

	chdev_finish();
	printk(KERN_INFO "Interface unloaded...\n");
}

static int __devinit interface_open(struct net_device *netdev) {
	unsigned char node_id, n_nodes;
	priv_data *priv;
	priv = netdev_priv(netdev);

	/* MAC addr */
	random_ether_addr(netdev->dev_addr);

	/* Node ID and number of nodes on the RT-WMP network from the IP addr */
	priv->net = inet_select_addr(netdev, 0, 0);

	/* IP already in the network byte order => The same in every system */
	node_id = ((unsigned char *) &priv->net )[3]-1;
	n_nodes = ((unsigned char *) &priv->net )[2];

	/* Host number to 0 => Network address */
	((unsigned char *) &priv->net)[3] = 0;

	/* Make sure the number of nodes and the ID are coherent */
	if (n_nodes < 2) {
		printk(KERN_ERR "The minimum number of nodes is 2\n");
		return -EPERM;
	}
	if (node_id >= n_nodes) {
		printk(KERN_ERR "The ID must be between 0 and %d (IP between 1 y %d)\n",n_nodes-1,n_nodes);
		return -EPERM;
	}

	/* RT-WMP setup */
	if(!wmpSetup(node_id, n_nodes)) {
		printk(KERN_ERR "RT-WMP initialization error.\n");
		return -EAGAIN;
	}

	wmpSetTaskMinimumSeparation(IP_TRAFFIC_PRIORITY, 250);
	wmpForceBurst(IP_TRAFFIC_PORT);

	/* Read the configuration for the traffic */
	readConfig(&priv->conf);
	//wmpForceTopology("chain",0);

	/* Run RT-WMP */
	wmpRunBG();

	/* Launch the RX thread */
	priv->rx_thread = kthread_run(rx_thread_func,NULL,"rx_thread");
	if (IS_ERR(priv->rx_thread)) {
		printk(KERN_ERR "Error while starting the RX thread\n");
		wmpExit();
		return -ENOMEM;
	}

	/* Launch the queue control thread */
	priv->queue_thread = kthread_run(queue_thread_func ,NULL,"queue_thread");
	if (IS_ERR(priv->queue_thread)) {
		printk(KERN_ERR "Error while starting the queue control thread\n");
		kthread_stop(priv->rx_thread);
		wmpExit();
		return -ENOMEM;
	}

	/* Start the tx queue */
	netif_start_queue(netdev);

	return 0;
}

static int __devexit interface_close (struct net_device *netdev) {
	priv_data *priv;
	priv = netdev_priv(netdev);

	/* Stop threads */
	kthread_stop(priv->rx_thread);
	kthread_stop(priv->queue_thread);

	/* Stop tx queue */
	netif_stop_queue(netdev);

	/* Close RT-WMP */
	wmpExit();

	return 0;
}

/**
 *  Returns the type of the message pointed by m
 *  Some types will only be returned if the proper plugin is being used
 *  If there is a matching configuration, the priority of the message will be
 *  set to the one in the configuration.
 */
static tpTraffic getMsgType( struct net_device *netdev, char * data, unsigned int dest, signed char * priority) {
	struct iphdr *iph = (struct iphdr *) data;
	struct tcphdr *tcph;
	struct udphdr *udph;
	u16 p_src, p_dst;
	signed char prio;
	tpTraffic traffic;
	priv_data *priv;
	priv = netdev_priv(netdev);

	/* Broadcast in the local network */
	if ((dest != 1) && (((unsigned char *) &iph->daddr)[3] == 255)) {
		return BROADCAST;
	}

	switch (iph->protocol) {
	case IPPROTO_ICMP:
		if (getConfICMP(&priv->conf, &prio, &traffic)) {
			*priority = prio;
			if (traffic == QoS) {
				return QoS;
			}
		}
		return OTHER;

	case IPPROTO_TCP:
		tcph = (struct tcphdr *) (iph + 1);
		p_src = ntohs(tcph->source);
		p_dst = ntohs(tcph->dest);

		/* If there is a configuration, use it */
		if (getPortConf(&priv->conf, TCP, FROM, p_src, &prio, &traffic)
				|| getPortConf(&priv->conf, TCP, TO, p_dst, &prio, &traffic)) {
			*priority = prio;
			if (traffic == QoS) {
				return QoS;
			}
		}
		break;

	case IPPROTO_UDP:
		udph = (struct udphdr *) (iph + 1);
		p_src = ntohs(udph->source);
		p_dst = ntohs(udph->dest);

		/* If there is a configuration, use it */
		if (getPortConf(&priv->conf, UDP, FROM, p_src, &prio, &traffic)
				|| getPortConf(&priv->conf, UDP, TO, p_dst, &prio, &traffic)) {
			*priority = prio;
			if (traffic == QoS) {
				return QoS;
			}
		}
		break;
	default:
		return OTHER;
	}

	return OTHER;
}


/**
 *  Sends the message pointed by m, taking into account its type to choose the
 *  function to use for the transmission
 */

static int send_msg(struct net_device *netdev, unsigned int port, char  * p, unsigned int size, unsigned int  dest, signed char priority) {

	switch (getMsgType(netdev, p, dest, &priority)) {
	case QoS:
		/* Set the deadline and change the port before sending the message */
		return wmpPushData(port,p,size,dest,priority);
	case BROADCAST:
		/* Set all the nodes in the network as destinations and send the message */
		return wmpPushData(port,p,size,dest,priority);
	case OTHER:
		/* Just send the message */
		return wmpPushData(port,p,size,dest,priority);
	case DROP:
		break;
	}

	return 0;
}

static int interface_tx(struct sk_buff *skb, struct net_device *netdev) {
	static Msg msg;
	struct iphdr *iph;
	__be32 network;
	priv_data *priv = netdev_priv(netdev);
	int room;
	unsigned int size, dest, port;
	signed char priority;
	/* Make sure the protocol is IP (0x0800) */
	if (((struct ethhdr *) skb->data)->h_proto == htons(ETH_P_IP)) {

		/* Destination */
		iph = ip_hdr(skb);
		network = iph->daddr;
		((unsigned char *) &network)[3] = 0;

		if (network != priv->net) {
			/* The IP is not local to the network -> GATEWAY */
			dest = 1;
		} else if (((unsigned char *) &iph->daddr)[3] > wmpGetNumOfNodes()
				&& (((unsigned char *) &iph->daddr)[3] != 255)) {
			/* Wrong node ID => DROP */
			dev_kfree_skb_any(skb);
			return NETDEV_TX_OK;
		} else {
			/* Destination inside the network */
			dest = (1 << (((unsigned char *) &iph->daddr)[3] - 1));
		}

		//XXX: to add broadcast

		/* Length and Data */
		size = ntohs(iph->tot_len);

		/* Free the skb */
		dev_kfree_skb_any(skb);

		/* Transmission stats */
		netdev->stats.tx_packets++;
		netdev->stats.tx_bytes += msg.len;
		netdev->trans_start = jiffies;

		/* Flow control */
		room =  wmp_queue_tx_get_room();
		if ((room < 5)) {
			netif_stop_queue(netdev);
		}

		/* Set port and priority */
		port = IP_TRAFFIC_PORT;
		priority = IP_TRAFFIC_PRIORITY;

		/* Try to send the message and update the error stats if something went wrong */
		if (!send_msg(netdev, port, (char *) iph, size, dest, priority)) {
			netdev->stats.tx_errors++;
		}
	} else {
		/* Not an IP packet ==> DROP */
		dev_kfree_skb_any(skb);
	}
	return NETDEV_TX_OK;
}

/**
 *  This function is used to provide direct access to the RT-WMP protocol though
 *  IOCTL calls
 */
static int interface_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd) {
	int ret;
	priv_data *priv;
	priv = netdev_priv(dev);

	/* Cannot use IOCTL if the interface is not UP */
	if (!(dev->flags & IFF_UP)) {
		return -EPERM;
	}

	ret = 0;
	switch (cmd) {

	case SIO_NODEINFO:
		ret = ioctl_node_info((tpNodeInfo *) ifr->ifr_data);
		break;

	case SIO_GETLATESTLQM:
		ioctl_lqm(ifr->ifr_data);
		break;

	case SIO_NETWORKCONNECTED:
		ret = ioctl_network_connected((tpNetworkConnectedInfo *) ifr->ifr_data);
		break;

	case SIO_QUEUEACTIONS:
		ret = ioctl_queue_actions((tpQueueActionInfo *) ifr->ifr_data);
		break;

	case SIO_QUEUEELEMSINFO:
		ret = ioctl_queue_elems_info((tpGetQueueElemsInfo *) ifr->ifr_data);
		break;

	case SIO_RTWMPSETGET:
		ret = ioctl_setget((tpRTWMPSetGetInfo *) ifr->ifr_data);
		break;

	default:
		return -EINVAL;
	}
	return ret;
}

static void interface_tx_timeout(struct net_device *dev) {
	dev->stats.tx_errors++;
	netif_wake_queue(dev);
}


module_init( load_interface);
module_exit( unload_interface);

MODULE_AUTHOR("Rubén Durán and Danilo Tardioli");
MODULE_DESCRIPTION("IP interface for RT-WMP");
MODULE_LICENSE("GPL");
MODULE_VERSION("");
