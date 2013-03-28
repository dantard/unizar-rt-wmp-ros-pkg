/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *
 *
 *
 *  File: ./src/platforms/linux_ks/hwi/ath5k_raw/ll_com.c
 *  Authors: Rubén Durán
 *           Danilo Tardioli
 *  ----------------------------------------------------------------------
 *  Copyright (C) 2000-2011, Universidad de Zaragoza, SPAIN
 *
 *  Contact Addresses: Danilo Tardioli                   dantard@unizar.es
 *
 *  RT-WMP is free software; you can  redistribute it and/or  modify it
 *  under the terms of the GNU General Public License  as published by the
 *  Free Software Foundation;  either  version 2, or (at  your option) any
 *  later version.
 *
 *  RT-WMP  is distributed  in the  hope  that  it will be   useful, but
 *  WITHOUT  ANY  WARRANTY;     without  even the   implied   warranty  of
 *  MERCHANTABILITY  or  FITNESS FOR A  PARTICULAR PURPOSE.    See the GNU
 *  General Public License for more details.
 *
 *  You should have received  a  copy of  the  GNU General Public  License
 *  distributed with RT-WMP;  see file COPYING.   If not,  write to the
 *  Free Software  Foundation,  59 Temple Place  -  Suite 330,  Boston, MA
 *  02111-1307, USA.
 *
 *  As a  special exception, if you  link this  unit  with other  files to
 *  produce an   executable,   this unit  does  not  by  itself cause  the
 *  resulting executable to be covered by the  GNU General Public License.
 *  This exception does  not however invalidate  any other reasons why the
 *  executable file might be covered by the GNU Public License.
 *
 *----------------------------------------------------------------------*/

#include "config/compiler.h"
#include "core/include/definitions.h"
#include "core/interface/wmp_interface.h"
#include "core/include/wmp_misc.h"
#include "core/include/ml_com.h"


#include <linux/if.h>
#include <linux/if_packet.h>
#include <linux/if_ether.h>
#include <linux/if_ether.h>
#include "module/ath5k_interface.h"

#include <linux/netdevice.h>
#include <linux/etherdevice.h>

#define WMP_PROTO 0x6969 /* TODO: Mover a un sitio apropiado */

/* Interface default configuration */
#define DEFAULT_FREQ		5200
#define DEFAULT_RATE		RATE_6M
#define DEFAULT_TXPOWER_DBM 15
#define DEFAULT_ANTENNA_MODE AR5K_ANTMODE_DEFAULT

#define ATH5K_RATE_CODE_1M	0x1B
#define ATH5K_RATE_CODE_2M	0x1A
#define ATH5K_RATE_CODE_5_5M	0x19
#define ATH5K_RATE_CODE_11M	0x18
/* A and G */
#define ATH5K_RATE_CODE_6M	0x0B
#define ATH5K_RATE_CODE_9M	0x0F
#define ATH5K_RATE_CODE_12M	0x0A
#define ATH5K_RATE_CODE_18M	0x0E
#define ATH5K_RATE_CODE_24M	0x09
#define ATH5K_RATE_CODE_36M	0x0D
#define ATH5K_RATE_CODE_48M	0x08
#define ATH5K_RATE_CODE_54M	0x0C

static double ath5k_rate[] = { 0, 0, 0, 0, 0, 0, 0, 0, 48, 24, 12, 6, 54, 36,
      18, 9, 0, 0, 0, 0, 0, 0, 0, 0, 11, 5.5, 2, 1, 0, 0, 0, 0, 0 };

/* Interface ath5k_raw used for tx and rx */
static struct net_device *interfaz;

static char param[256], val[256];

static short freq;
static short rate;
static unsigned char txpower_dbm;
static int antenna_mode;
static int ath5k_if_was_up;

/* Cached frame used for tx */
static unsigned char ethernet_frame[2500];

void print_packet_hex(char *msg, unsigned char *packet, int len) {
   unsigned char *p = packet;

   printk(KERN_INFO "---------Packet---Starts----\n");
   printk(KERN_INFO "%s\n", msg);
   while (len--) {
      printk(KERN_INFO "%.2x ", *p);
      p++;
   }
   printk(KERN_INFO "\n--------Packet---Ends-----\n\n");
}

int readllcfg(void) {
   struct file *f;
   char line[256];
   int exists;


   /* Set default values */
   freq = DEFAULT_FREQ;
   rate = DEFAULT_RATE;
   txpower_dbm = DEFAULT_TXPOWER_DBM;
   antenna_mode = DEFAULT_ANTENNA_MODE;

   f = filp_open("/etc/rt-wmp/linux_ks-ath5k.ll", O_RDONLY, 0);

   if (!IS_ERR(f)) {
      WMP_MSG(stderr, "Reading Low Level Configuration file (/etc/rt-wmp/linux_ks-ath5k.ll)... \n");
      while (fgets(line,256,f) != NULL) {
         if (line[0]<65 || line[0]>90){
            continue;
         }
         sscanf(line,"%s %s",param,val);
         exists = 0;
         /*if (strcmp(param, "DEVICE") == 0){  //Doesn't make sense for linux_ks
            strcpy(dev, val);
            exists = 1;
         }
         else*/ if (strcmp(param, "FREQ") == 0){
            freq = atoi(val);
            exists = 1;
         }
         else if (strcmp(param, "RATE") == 0){
            rate = atoi(val);
            exists = 1;
         }
         else if (strcmp(param, "TXPOWER_DBM") == 0){
            txpower_dbm = atoi(val);
            exists = 1;
         }
         else if (strcmp(param, "ANTENNA_MODE") == 0) {
            if (strcmp(val, "AR5K_ANTMODE_DEFAULT") == 0){
               antenna_mode = AR5K_ANTMODE_DEFAULT;
               exists = 1;
            }
            else if (strcmp(val, "AR5K_ANTMODE_FIXED_A") == 0){
               antenna_mode = AR5K_ANTMODE_FIXED_A;
               exists = 1;
            }
            else if (strcmp(val, "AR5K_ANTMODE_FIXED_B") == 0){
               antenna_mode = AR5K_ANTMODE_FIXED_B;
               exists = 1;
            }
            else if (strcmp(val, "AR5K_ANTMODE_SINGLE_AP") == 0){
               antenna_mode = AR5K_ANTMODE_SINGLE_AP;
               exists = 1;
            }
            else if (strcmp(val, "AR5K_ANTMODE_SECTOR_AP") == 0){
               antenna_mode = AR5K_ANTMODE_SECTOR_AP;
               exists = 1;
            }
            else if (strcmp(val, "AR5K_ANTMODE_SECTOR_STA") == 0){
               antenna_mode = AR5K_ANTMODE_SECTOR_STA;
            }
            else if (strcmp(val, "AR5K_ANTMODE_DEBUG") == 0){
               antenna_mode = AR5K_ANTMODE_DEBUG;
               exists = 1;
            }
         }
         if (exists) {
            WMP_MSG(stderr, "READ OPTION: %s = %s\n", param, val);
         }else{
            WMP_MSG(stderr, "WARNING ::: UKNOWN OPTION %s\n", param);
         }
      }
      WMP_MSG(stderr, "Done.\n");
      filp_close(f,0);
   }
   else
      WMP_MSG(stderr, "File /etc/rt-wmp/linux_ks-ath5k.ll not found, using default values.\n");

   return 0;
}

void closeLowLevelCom(void) {
   /* Restore the previous ath5k_raw interface state */
   if(!ath5k_if_was_up)
      dev_close(interfaz);
}

int configure_interface(struct net_device *dev) {
   union ath5k_ioctl_info ioctl_info;
   int ret = 0;

   /* Configure freq, rate and power*/
   ioctl_info.config_info.frequency = freq;
   ioctl_info.config_info.rate = rate;
   ioctl_info.config_info.tx_power_dbm = txpower_dbm;
   ioctl_info.config_info.antenna_mode = antenna_mode;
   if ((conf_ath5k(dev, &ioctl_info, SIO_SET_CONFIG)) < 0) {
      printk(KERN_ERR "Error configuring interface");
      ret--;
   }

   /* Set rx filter */
   ioctl_info.rxfilter_info.broadcast = true;
   ioctl_info.rxfilter_info.control = false;
   ioctl_info.rxfilter_info.promisc = true;
   if ((conf_ath5k(dev, &ioctl_info, SIO_SET_RXFILTER)) < 0) {
      printk(KERN_ERR "Error setting hw rx filter");
      ret--;
   }

   /* Select tx rate, disable soft retries and disable ACK waiting for unicast frames */
   ioctl_info.txcontrol_info.wait_for_ack = false;
   ioctl_info.txcontrol_info.use_short_preamble = false;
   ioctl_info.txcontrol_info.count = 1;
   if ((conf_ath5k(dev, &ioctl_info, SIO_SET_TXCONTROL)) < 0) {
      printk(KERN_ERR "Error setting tx control");
      ret--;
   }

   /* Disable ACK unicast frames */
   ioctl_info.disable_ack = true;
   if ((conf_ath5k(dev, &ioctl_info, SIO_SET_DISABLEACK)) < 0) {
      printk(KERN_ERR "Error disabling ack");
      ret--;
   }

   /* Set debug level */
   ioctl_info.debug_level = ATH5K_DEBUG_NONE;
   if ((conf_ath5k(dev, &ioctl_info, SIO_SET_DEBUG)) < 0) {
      printk(KERN_ERR "Error disabling debug");
      ret--;
   }

   return ret;
}

int initLowLevelCom(void) {

   struct ethhdr *ethernet_header;
   int ret = 0;
   unsigned char dst_mac[ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

   msleep(1000);

   if (readllcfg() == -1){
      ret--;
   }

   /* Get ath5k_raw interface to use */
   get_ath5k_slave_dev(&interfaz);

   if (!interfaz)
   {
      printk(KERN_ERR "Unable to get ath5k_raw interface\n");
      return 0;
   }

   WMP_DBG(LL_COM,
         "Using DEV:%s, %d Mhz, (%d * 100) kbps, %d dBm, Ant. mode %d\n",
         interfaz->name, freq, rate, txpower_dbm, antenna_mode);

   /* Store the ath5k_raw interface state and bring the interface up */
   ath5k_if_was_up = (interfaz->flags & IFF_UP);
   dev_open(interfaz);

   /* Configure the interface */
   ret -= configure_interface(interfaz);

   /* Build the ethernet header used for tx */
   ethernet_header = (struct ethhdr *) ethernet_frame;
   memcpy(ethernet_header->h_dest, dst_mac, ETH_ALEN);
   memcpy(ethernet_header->h_source, interfaz->dev_addr, ETH_ALEN);
   ethernet_header->h_proto = htons(WMP_PROTO);

   if (ret == 0) {
      WMP_DBG(LL_COM, "Low level com init ok.\n");
      return 1;
   }

   return 0;
}

int llsend(char * f, int size) {

   unsigned char *p;
   struct sk_buff *skb;

   /* Paranoid mode on */
   if (!(interfaz->flags & IFF_UP))
      return -ENETDOWN;

   if (size + ETH_HLEN > interfaz->mtu + interfaz->hard_header_len)
      return -EMSGSIZE;

   skb = alloc_skb(size + ETH_HLEN /*+ LL_RESERVED_SPACE(interfaz)*/, GFP_ATOMIC);

   if (skb == NULL)
      return -ENOBUFS;

   //skb_reserve(skb, LL_RESERVED_SPACE(interfaz));

   /* Copy the message into the ethernet frame */
   p = ethernet_frame + ETH_HLEN;
   memcpy(p, f, size);

   /* Copy the ethernet frame into the skb */
   memcpy(skb_put(skb, size+ETH_HLEN),ethernet_frame, size+ETH_HLEN);

   skb->protocol = WMP_PROTO;
   skb->dev = interfaz;

   /* Send the skb directly through the ath5k_raw interface */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30)
   interfaz->hard_start_xmit(skb, interfaz);
#else
   interfaz->netdev_ops->ndo_start_xmit(skb, interfaz);
#endif

   return size;
}

rxInfo llreceive(char *f, int timeout) {

   struct sk_buff *skb;
   rxInfo ret;
   char c_rate;
   struct ethhdr * eth_hdr = (struct ethhdr *) f;

   memset(&ret, 0, sizeof(ret));

   if (ath5k_rx_timeout(&skb, timeout)) {
      skb_push(skb, ETH_HLEN);        /* Recover the ethernet header */

      memcpy(f, skb->data, skb->len);
      c_rate = (*(f + ETH_HLEN + 1)); /* ath5k form */
      ret.proto = eth_hdr->h_proto;
      ret.rate = ath5k_rate[(int) c_rate];
      ret.size = skb->len - ETH_HLEN;
      memmove(f, f + ETH_HLEN, ret.size); /* Copy the IP packet */

      /* Free the skb */
      kfree_skb(skb); //dev_kfree_skb(skb);

      return ret;
   } else {
      ret.error = 1;
      return ret;
   }
}
