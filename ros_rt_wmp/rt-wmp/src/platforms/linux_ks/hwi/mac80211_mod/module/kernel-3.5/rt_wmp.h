#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/rcupdate.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/rcupdate.h>
#include <linux/types.h>

#define MAC80211

#ifdef MAC80211
#include <linux/nl80211.h>
#include <net/mac80211.h>
#include <net/cfg80211.h>
#include <net/ieee80211_radiotap.h>
#include <linux/export.h>

#endif

//#include "ieee80211_i.h"
typedef struct {
	int rate;
	int size;
	int proto;
	char error;
	char has_lq;
	char rssi;
	char noise;
} rxInfo ;


int rt_wmp_netif_receive_skb(struct sk_buff *skb, int signal);
int rt_wmp_is_active(void);
