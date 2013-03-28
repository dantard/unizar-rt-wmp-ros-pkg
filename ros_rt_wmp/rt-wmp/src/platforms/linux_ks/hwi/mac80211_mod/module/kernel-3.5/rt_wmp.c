#include "rt_wmp.h"

#define WMP_PROTO 0x6969
//#define MAC80211

/* Queue for RT-WMP packet reception */
static struct net_device * net_dev;
struct ieee80211_sub_if_data *sdata;
static int initialized = 0, promisc = 0;

static struct rx_queue {
   struct sk_buff *skb;
   struct semaphore rx_sem;
   spinlock_t spin;
} rx_rtwmp_queue;

static unsigned char ethernet_frame[2500];
static struct ethhdr *ethernet_header;
static unsigned char dst_mac[ETH_ALEN] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
static unsigned char src_mac[ETH_ALEN] = { 0xea, 0x1e, 0x1e, 0xe1, 0x00, 0x00 };



/* Initialization of the reception semaphore and queue for RT-WMP */
static void init_rx_queue(struct rx_queue *q) {
   sema_init(&q->rx_sem, 0);
   spin_lock_init(&q->spin);
   q->skb=NULL;
}

/* Free the remaining SKBs in the reception queue */
static void purge_rx_queue(struct rx_queue *q) {
   if(q->skb!=NULL)
      dev_kfree_skb_any(q->skb);
}

static void enqueue_rx_queue(struct rx_queue *q, struct sk_buff *skb) {
   unsigned long flags;
   spin_lock_irqsave(&q->spin, flags);
   if(q->skb == NULL){  // No hay elemento
      q->skb = skb;
      up(&q->rx_sem);
   } else {
      dev_kfree_skb_any(q->skb);
      q->skb = skb;
   }
   spin_unlock_irqrestore(&q->spin, flags);
}

static struct sk_buff *dequeue_rx_queue(struct rx_queue *q, long timeout) {
   unsigned long flags;
   struct sk_buff *skb = NULL;
   if (timeout == 0) {
      if (0==down_interruptible(&q->rx_sem)) {
         spin_lock_irqsave(&q->spin, flags);
         skb = q->skb;
         q->skb = NULL;
         while(!down_trylock(&q->rx_sem));
         spin_unlock_irqrestore(&q->spin, flags);
      }
   } else if (0==down_timeout(&q->rx_sem, msecs_to_jiffies(timeout))) {
      spin_lock_irqsave(&q->spin, flags);
      skb = q->skb;
      q->skb = NULL;
      while(!down_trylock(&q->rx_sem)){
    	  printk(KERN_ERR "Spinning\n");
      }
      spin_unlock_irqrestore(&q->spin, flags);
   }

   return skb;
}

int rt_wmp_is_active(){
	return initialized;
}

int rt_wmp_netif_receive_skb(struct sk_buff *skb, int signal){
	int proto;

//	printk(KERN_ERR "DEV_MINE:%p\n", net_dev);


	if (!initialized){
#ifdef MAC80211
		dev_kfree_skb(skb);
#else
		netif_rx(skb);
#endif
	}else{

#ifdef MAC80211
		unsigned short * protocol;
		struct ieee80211_radiotap_header * rth = (struct ieee80211_radiotap_header *) skb->data;
//		struct ieee80211_hdr * hdr =  (struct ieee80211_hdr *) (skb->data + rth->it_len);
		protocol = (unsigned short *) (skb->data + rth->it_len + sizeof(struct ieee80211_hdr));
		proto = *protocol;
#else
		proto = skb->protocol;
		skb->data[0] = 98 + (signed char) signal;
#endif

		if (promisc){
			enqueue_rx_queue(&rx_rtwmp_queue, skb);
		}else{
			if (proto == 0x6969) {
				enqueue_rx_queue(&rx_rtwmp_queue, skb);
			}
		}
	}

	return 1;
}

int ieee80211_rtwmp_timed_receive_raw(struct sk_buff **skb, unsigned int ms){
	struct sk_buff * k = dequeue_rx_queue(&rx_rtwmp_queue, ms);
	*skb=k;
	return (k!=NULL);
}

void ieee80211_rtwmp_finish(void){
	purge_rx_queue(&rx_rtwmp_queue);
	initialized = 0;
	printk(KERN_INFO "MAC80211-RT-WMP closed\n");
}


int ieee80211_rtwmp_send(char * f, int size) {
	int ret;
	struct sk_buff *skb;

	skb = alloc_skb(size + ETH_HLEN, GFP_ATOMIC);

	if (skb == NULL) {
		printk(KERN_ERR "Error: Unable to allocate skb (llsend) \n");
		return -ENOBUFS;
	}

	/* Copy the message into the ethernet frame */
	memcpy(ethernet_frame + ETH_HLEN, f, size);

	/* Copy the ethernet frame into the skb */
	memcpy(skb_put(skb, size + ETH_HLEN), ethernet_frame, size + ETH_HLEN);

	skb->protocol = WMP_PROTO;
	skb->dev = net_dev;

	ret = net_dev->netdev_ops->ndo_start_xmit(skb, net_dev);
	if (ret != 0) {
		kfree_skb(skb);
		return 0;
	}
	return 1;
}

#ifdef MAC80211
static void parse_radiotap(struct ieee80211_radiotap_header * buf, int * rate, char * rssi, char * noise) {
	int pkt_rate_100kHz = 0, antenna = 0, pwr = 0;
	char rssi_dbm = 0, noise_dbm = 0;
	struct ieee80211_radiotap_iterator iterator;
	int ret = ieee80211_radiotap_iterator_init(&iterator, buf, buf->it_len,0);

	while (!ret) {

		ret = ieee80211_radiotap_iterator_next(&iterator);

		if (ret)
			continue;

		/* see if this argument is something we can use */

		switch (iterator.this_arg_index) {
		/*
		 * You must take care when dereferencing iterator.this_arg
		 * for multibyte types... the pointer is not aligned.  Use
		 * get_unaligned((type *)iterator.this_arg) to dereference
		 * iterator.this_arg for type "type" safely on all arches.
		 */
		case IEEE80211_RADIOTAP_RATE:
			/* radiotap "rate" u8 is in
			 * 500kbps units, eg, 0x02=1Mbps
			 */
			pkt_rate_100kHz = (*iterator.this_arg) * 5;
			break;
		case IEEE80211_RADIOTAP_DBM_ANTSIGNAL:
			rssi_dbm = (*iterator.this_arg);
			break;
		case IEEE80211_RADIOTAP_ANTENNA:
			/* radiotap uses 0 for 1st ant */
			antenna = (*iterator.this_arg);
			break;

		case IEEE80211_RADIOTAP_DBM_TX_POWER:
			pwr = *iterator.this_arg;
			break;
		case IEEE80211_RADIOTAP_DB_ANTNOISE:
			noise_dbm = *iterator.this_arg;
			break;
		default:
			break;
		}
	} /* while more rt headers */
	*noise = noise_dbm;
	*rssi = rssi_dbm;
	*rate = pkt_rate_100kHz;
}
#endif

int ieee80211_rtwmp_timed_receive(char * buff, rxInfo * ret, int ms){
	unsigned int pad = 0;
	struct sk_buff * skb = dequeue_rx_queue(&rx_rtwmp_queue, ms);
	if (skb!=NULL){

#ifdef MAC80211
		int rate = 0, llc_size = 2;
		char rssi, noise;
		struct ieee80211_radiotap_header * rth = (struct ieee80211_radiotap_header *) skb->data;
		pad = rth->it_len + sizeof(struct ieee80211_hdr) + llc_size;
		parse_radiotap(rth, &rate, &rssi, &noise);
		ret->proto = (*((unsigned short *) (skb->data + rth->it_len + sizeof(struct ieee80211_hdr))));
		ret->rate = rate;
		ret->rssi = rssi;
		ret->noise = noise;
		ret->has_lq = 1;
#else
		ret->has_lq = 0 ;
		ret->proto = skb->protocol;
#endif


		ret->size = skb->len-pad;
		ret->error = 0;
		memcpy(buff, skb->data + pad, ret->size);
//		printk(KERN_INFO "Dequeuo %d size:%d \n",ret->proto, ret->size);

		return 1;
	}else{
		ret->error = 1;
		return 0;
	}
}

int ieee80211_rtwmp_init(char * net_dev_name){
	int found = 0;
	net_dev = first_net_device(&init_net);
	while (net_dev) {
		net_dev = next_net_device(net_dev);
		if (strstr(net_dev->name, net_dev_name) != NULL) {
			found = 1;
			break;
		}
	}
	if (!found){
		printk(KERN_INFO "Error: unable to find net_device %s\n",net_dev_name);
		return 0;
	}

	/* Build the ethernet header used for tx */
	ethernet_header = (struct ethhdr *) ethernet_frame;
	memcpy(ethernet_header->h_source, src_mac, ETH_ALEN);
	memcpy(ethernet_header->h_dest, dst_mac, ETH_ALEN);
	ethernet_header->h_proto = htons(WMP_PROTO);

	init_rx_queue(&rx_rtwmp_queue);

	printk(KERN_INFO "Found net_device %s",net_dev->name);
	printk(KERN_INFO "MAC80211-RT-WMP initialized successfully\n");
	initialized = 1;
	return 1;

}



void ieee80211_rtwmp_set_promisc(int _promisc){
	promisc = _promisc;
}


EXPORT_SYMBOL(ieee80211_rtwmp_init);
EXPORT_SYMBOL(ieee80211_rtwmp_timed_receive);
EXPORT_SYMBOL(ieee80211_rtwmp_timed_receive_raw);
EXPORT_SYMBOL(ieee80211_rtwmp_finish);
EXPORT_SYMBOL(ieee80211_rtwmp_send);
EXPORT_SYMBOL(ieee80211_rtwmp_set_promisc);
