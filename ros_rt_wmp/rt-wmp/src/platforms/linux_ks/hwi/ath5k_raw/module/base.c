/*------------------------------------------------------------------------
 *---------------------        ATH5K Driver     	  --------------------
 *------------------------------------------------------------------------
 *                                                         V1.0B  04/08/09
 *
 *
 *  Aug 2009 - Samuel Cabrero <samuelcabrero@gmail.com>
 *		Initial release
 *
 *  ----------------------------------------------------------------------
 *  Copyright (C) 2000-2009, Universidad de Zaragoza, SPAIN
 *
 *  Autors:
 *		Samuel Cabrero       <samuelcabrero@gmail.com>
 *		Danilo Tardioli		 <dantard@unizar.es>
 *		Jose Luis Villarroel <jlvilla@unizar.es>
 *
 *  This is a modified version of the original ath5k driver for its use with
 *  RT-WMP protocol developped at Universidad de Zaragoza. It should work with 
 *  all Atheros 5xxx WLAN cards. The IEEE 802.11 layer have been removed so it 
 *  just send and receive frames over the air, as if it were a ethernet bus
 *  interface.
 * 
 *  Please read ath5k_interface.h for instructions.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  Read the following header for redistribution terms.
 *
 *---------------------------------------------------------------------------*/

/*-
 * Copyright (c) 2002-2005 Sam Leffler, Errno Consulting
 * Copyright (c) 2004-2005 Atheros Communications, Inc.
 * Copyright (c) 2006 Devicescape Software, Inc.
 * Copyright (c) 2007 Jiri Slaby <jirislaby@gmail.com>
 * Copyright (c) 2007 Luis R. Rodriguez <mcgrof@winlab.rutgers.edu>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
 *    redistribution must be conditioned upon including a substantially
 *    similar Disclaimer requirement for further binary redistribution.
 * 3. Neither the names of the above-listed copyright holders nor the names
 *    of any contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 *
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/if.h>
#include <linux/netdevice.h>
#include <linux/cache.h>
#include <linux/pci.h>
#include <linux/ethtool.h>
#include <linux/uaccess.h>
#include <linux/if_arp.h>
#include <linux/etherdevice.h>
#include <asm/unaligned.h>

#include "base.h"
#include "reg.h"
#include "debug.h"


/* RUBEN */
#include <linux/semaphore.h>
#include <linux/spinlock.h>


static u8 ath5k_calinterval = 10; /* Calibrate PHY every 10 secs (TODO: Fixme) */
static int modparam_all_channels = true;
static int use_beacon_frames = 0;
/******************\
* Internal defines *
\******************/

/* Module info */
MODULE_AUTHOR("Jiri Slaby");
MODULE_AUTHOR("Nick Kossifidis");
MODULE_AUTHOR("Samuel Cabrero");
MODULE_DESCRIPTION("Modified version of the ath5k driver for use with RT-WMP protocol.");
MODULE_SUPPORTED_DEVICE("Atheros 5xxx WLAN cards");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION("0.6.0 (EXPERIMENTAL)");


/* Known PCI ids */
static const struct pci_device_id ath5k_pci_id_table[] = {
	{ PCI_VDEVICE(ATHEROS, 0x0207) }, /* 5210 early */
	{ PCI_VDEVICE(ATHEROS, 0x0007) }, /* 5210 */
	{ PCI_VDEVICE(ATHEROS, 0x0011) }, /* 5311 - this is on AHB bus !*/
	{ PCI_VDEVICE(ATHEROS, 0x0012) }, /* 5211 */
	{ PCI_VDEVICE(ATHEROS, 0x0013) }, /* 5212 */
	{ PCI_VDEVICE(3COM_2,  0x0013) }, /* 3com 5212 */
	{ PCI_VDEVICE(3COM,    0x0013) }, /* 3com 3CRDAG675 5212 */
	{ PCI_VDEVICE(ATHEROS, 0x1014) }, /* IBM minipci 5212 */
	{ PCI_VDEVICE(ATHEROS, 0x0014) }, /* 5212 combatible */
	{ PCI_VDEVICE(ATHEROS, 0x0015) }, /* 5212 combatible */
	{ PCI_VDEVICE(ATHEROS, 0x0016) }, /* 5212 combatible */
	{ PCI_VDEVICE(ATHEROS, 0x0017) }, /* 5212 combatible */
	{ PCI_VDEVICE(ATHEROS, 0x0018) }, /* 5212 combatible */
	{ PCI_VDEVICE(ATHEROS, 0x0019) }, /* 5212 combatible */
	{ PCI_VDEVICE(ATHEROS, 0x001a) }, /* 2413 Griffin-lite */
	{ PCI_VDEVICE(ATHEROS, 0x001b) }, /* 5413 Eagle */
	{ PCI_VDEVICE(ATHEROS, 0x001c) }, /* PCI-E cards */
	{ PCI_VDEVICE(ATHEROS, 0x001d) }, /* 2417 Nala */
	{ 0 }
};
//MODULE_DEVICE_TABLE(pci, ath5k_pci_id_table);

/* Known SREVs */
static const struct ath5k_srev_name srev_names[] = {
	{ "5210",	AR5K_VERSION_MAC,	AR5K_SREV_AR5210 },
	{ "5311",	AR5K_VERSION_MAC,	AR5K_SREV_AR5311 },
	{ "5311A",	AR5K_VERSION_MAC,	AR5K_SREV_AR5311A },
	{ "5311B",	AR5K_VERSION_MAC,	AR5K_SREV_AR5311B },
	{ "5211",	AR5K_VERSION_MAC,	AR5K_SREV_AR5211 },
	{ "5212",	AR5K_VERSION_MAC,	AR5K_SREV_AR5212 },
	{ "5213",	AR5K_VERSION_MAC,	AR5K_SREV_AR5213 },
	{ "5213A",	AR5K_VERSION_MAC,	AR5K_SREV_AR5213A },
	{ "2413",	AR5K_VERSION_MAC,	AR5K_SREV_AR2413 },
	{ "2414",	AR5K_VERSION_MAC,	AR5K_SREV_AR2414 },
	{ "5424",	AR5K_VERSION_MAC,	AR5K_SREV_AR5424 },
	{ "5413",	AR5K_VERSION_MAC,	AR5K_SREV_AR5413 },
	{ "5414",	AR5K_VERSION_MAC,	AR5K_SREV_AR5414 },
	{ "2415",	AR5K_VERSION_MAC,	AR5K_SREV_AR2415 },
	{ "5416",	AR5K_VERSION_MAC,	AR5K_SREV_AR5416 },
	{ "5418",	AR5K_VERSION_MAC,	AR5K_SREV_AR5418 },
	{ "2425",	AR5K_VERSION_MAC,	AR5K_SREV_AR2425 },
	{ "2417",	AR5K_VERSION_MAC,	AR5K_SREV_AR2417 },
	{ "xxxxx",	AR5K_VERSION_MAC,	AR5K_SREV_UNKNOWN },
	{ "5110",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5110 },
	{ "5111",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5111 },
	{ "5111A",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5111A },
	{ "2111",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_2111 },
	{ "5112",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5112 },
	{ "5112A",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5112A },
	{ "5112B",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5112B },
	{ "2112",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_2112 },
	{ "2112A",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_2112A },
	{ "2112B",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_2112B },
	{ "2413",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_2413 },
	{ "5413",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5413 },
	{ "2316",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_2316 },
	{ "2317",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_2317 },
	{ "5424",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5424 },
	{ "5133",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5133 },
	{ "xxxxx",	AR5K_VERSION_RAD,	AR5K_SREV_UNKNOWN },
};

static const struct ieee80211_rate ath5k_rates[] = {
	{ .bitrate = 10,
	  .hw_value = ATH5K_RATE_CODE_1M, },
	{ .bitrate = 20,
	  .hw_value = ATH5K_RATE_CODE_2M,
	  .hw_value_short = ATH5K_RATE_CODE_2M | AR5K_SET_SHORT_PREAMBLE,
	  .flags = IEEE80211_RATE_SHORT_PREAMBLE },
	{ .bitrate = 55,
	  .hw_value = ATH5K_RATE_CODE_5_5M,
	  .hw_value_short = ATH5K_RATE_CODE_5_5M | AR5K_SET_SHORT_PREAMBLE,
	  .flags = IEEE80211_RATE_SHORT_PREAMBLE },
	{ .bitrate = 110,
	  .hw_value = ATH5K_RATE_CODE_11M,
	  .hw_value_short = ATH5K_RATE_CODE_11M | AR5K_SET_SHORT_PREAMBLE,
	  .flags = IEEE80211_RATE_SHORT_PREAMBLE },
	{ .bitrate = 60,
	  .hw_value = ATH5K_RATE_CODE_6M,
	  .flags = 0 },
	{ .bitrate = 90,
	  .hw_value = ATH5K_RATE_CODE_9M,
	  .flags = 0 },
	{ .bitrate = 120,
	  .hw_value = ATH5K_RATE_CODE_12M,
	  .flags = 0 },
	{ .bitrate = 180,
	  .hw_value = ATH5K_RATE_CODE_18M,
	  .flags = 0 },
	{ .bitrate = 240,
	  .hw_value = ATH5K_RATE_CODE_24M,
	  .flags = 0 },
	{ .bitrate = 360,
	  .hw_value = ATH5K_RATE_CODE_36M,
	  .flags = 0 },
	{ .bitrate = 480,
	  .hw_value = ATH5K_RATE_CODE_48M,
	  .flags = 0 },
	{ .bitrate = 540,
	  .hw_value = ATH5K_RATE_CODE_54M,
	  .flags = 0 },
	/* XR missing */
};


/*
 * Prototypes - PCI stack related functions
 */
static int __devinit	ath5k_pci_probe(struct pci_dev *pdev,
				const struct pci_device_id *id);
static void __devexit	ath5k_pci_remove(struct pci_dev *pdev);

static struct pci_driver ath5k_pci_driver = {
	.name		= "ath5k_pci",
	.id_table	= ath5k_pci_id_table,
	.probe		= ath5k_pci_probe,
	.remove		= __devexit_p(ath5k_pci_remove),
};



/*
 * Prototypes - MAC 802.11 stack related functions
 */
static int ath5k_tx(struct sk_buff *skb, struct net_device *netdev);
static int ath5k_reset(struct ath5k_softc *sc, struct ieee80211_channel *chan);
static int ath5k_reset_wake(struct ath5k_softc *sc);
static int ath5k_start(struct ath5k_softc *sc);
static void ath5k_stop(struct ath5k_softc *sc);

/*
 * Prototypes - Internal functions
 */
/* Attach detach */
static int  ath5k_attach(struct ath5k_softc *sc);
static void ath5k_detach(struct pci_dev *pdev, struct ath5k_softc *sc);
/* Channel/mode setup */
static inline short ath5k_ieee2mhz(short chan);
static unsigned int ath5k_copy_channels(struct ath5k_hw *ah,
				struct ieee80211_channel *channels,
				unsigned int mode,
				unsigned int max);
static int 	ath5k_setup_bands(struct ath5k_softc *sc);
static void	ath5k_mode_setup(struct ath5k_softc *sc);

/* Descriptor setup */
static int	ath5k_desc_alloc(struct ath5k_softc *sc,
				struct pci_dev *pdev);
static void	ath5k_desc_free(struct ath5k_softc *sc,
				struct pci_dev *pdev);
/* Buffers setup */
static int 	ath5k_rxbuf_setup(struct ath5k_softc *sc,
				struct ath5k_buf *bf);
static int 	ath5k_txbuf_setup(struct ath5k_softc *sc,
				struct ath5k_buf *bf);
static inline void ath5k_txbuf_free(struct ath5k_softc *sc,
				struct ath5k_buf *bf)
{
	BUG_ON(!bf);
	if (!bf->skb)
		return;
	pci_unmap_single(sc->pdev, bf->skbaddr, bf->skb->len,
			PCI_DMA_TODEVICE);
	dev_kfree_skb_any(bf->skb);
	bf->skb = NULL;
}

static inline void ath5k_rxbuf_free(struct ath5k_softc *sc,
				struct ath5k_buf *bf)
{
	BUG_ON(!bf);
	if (!bf->skb)
		return;
	pci_unmap_single(sc->pdev, bf->skbaddr, sc->rxbufsize,
			PCI_DMA_FROMDEVICE);
	dev_kfree_skb_any(bf->skb);
	bf->skb = NULL;
}


/* Queues setup */
static struct 	ath5k_txq *ath5k_txq_setup(struct ath5k_softc *sc,
				int qtype, int subtype);
static void 	ath5k_txq_drainq(struct ath5k_softc *sc,
				struct ath5k_txq *txq);
static void 	ath5k_txq_cleanup(struct ath5k_softc *sc);
static void 	ath5k_txq_release(struct ath5k_softc *sc);
/* Rx handling */
static int 	ath5k_rx_start(struct ath5k_softc *sc);
static void 	ath5k_rx_stop(struct ath5k_softc *sc);
static void 	ath5k_rx_done(struct ath5k_softc *sc);
/* Tx handling */
static void 	ath5k_tx_done(struct ath5k_softc *sc,
				struct ath5k_txq *txq);
/* Interrupt handling */
static int ath5k_init(struct ath5k_softc *sc);
static int ath5k_stop_locked(struct ath5k_softc *sc);
static int ath5k_stop_hw(struct ath5k_softc *sc);
static irqreturn_t ath5k_intr(int irq, void *dev_id);
static void ath5k_tasklet_reset(unsigned long data);
//static void ath5k_tasklet_calibrate(unsigned long data);
/* Net device */
static int  ath5k_open(struct net_device *netdev);
static int  ath5k_close (struct net_device *netdev);
/* Configuration and auxiliary */
static void ath5k_config_filter (struct ath5k_softc *sc, bool broadcast, bool control, bool promisc);
static int ath5k_config_tx_control (struct ath5k_softc *sc, 
		unsigned char count, bool wait_for_ack, bool use_short_preamble);
static int ath5k_config_disable_ack (struct ath5k_softc *sc, bool disable);
static int ath5k_config(struct ath5k_softc *sc, unsigned short channel, enum rates rate, unsigned char power,
		enum ath5k_ant_mode antenna_mode);
static struct ieee80211_channel * ath5k_get_channel(struct ath5k_softc *sc, unsigned int freq);
static int ath5k_get_rate_idx(struct ath5k_softc *sc, enum ieee80211_band band, enum rates rate);
static int ath5k_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30)
static const struct net_device_ops ath5k_netdev_ops = {
        .ndo_open               = ath5k_open,
        .ndo_stop               = ath5k_close,
        .ndo_start_xmit         = ath5k_tx,
        .ndo_do_ioctl           = ath5k_ioctl,
};
#endif


/* RUBEN */

/* Interface that will be used by RT-WMP */
static struct net_device* ath5k_slave_dev = NULL;

/* Queue for RT-WMP packet reception */
static struct rx_queue {
   struct sk_buff *skb;
   struct semaphore rx_sem;
   spinlock_t spin;
} rx_rtwmp_queue;

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
   }
   else if (0==down_timeout(&q->rx_sem, timeout)) {
      spin_lock_irqsave(&q->spin, flags);
      skb = q->skb;
      q->skb = NULL;
      while(!down_trylock(&q->rx_sem));
      spin_unlock_irqrestore(&q->spin, flags);
   }

   return skb;
}



/*
 * Module init/exit functions
 */


/* *** DANIIIIIIIIIIII
	static int
	hello_read_proc(char *buffer, char **start, off_t offset, int size, int *eof,
	                void *data)
	{
    char *hello_str = "Hello, world!\n";
    int len = strlen(hello_str);
    if (size < len)
            return -EINVAL;
    if (offset != 0)
            return 0;
    strcpy(buffer, hello_str);
    *eof = 1;
    return len;

}
*/


static int __init
init_ath5k_pci(void)
{
	int ret;

	printk(KERN_INFO "Module ath5k_raw loaded\n");

	ret = pci_register_driver(&ath5k_pci_driver);
	if (ret) {
		printk(KERN_ERR "ath5k_pci: can't register pci driver\n");
		return ret;
	}


	/* RUBEN */
   init_rx_queue(&rx_rtwmp_queue);

/* DANIIIIIIIIII
    if (create_proc_read_entry("hello_world", 0, NULL, hello_read_proc,
                                NULL) == 0) {
            printk(KERN_ERR,
                   "Unable to register \"Hello, world!\" proc file\n");
            return -ENOMEM;
    }
*/


	return 0;
}

static void __exit
exit_ath5k_pci(void)
{
	pci_unregister_driver(&ath5k_pci_driver);

   /* RUBEN */
   purge_rx_queue(&rx_rtwmp_queue);

	printk(KERN_INFO "Module ath5k_raw unloaded\n");
}

module_init(init_ath5k_pci);
module_exit(exit_ath5k_pci);


/********************\
* PCI Initialization *
\********************/

static const char *
ath5k_chip_name(enum ath5k_srev_type type, u_int16_t val)
{
	const char *name = "xxxxx";
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(srev_names); i++) {
		if (srev_names[i].sr_type != type)
			continue;

		if ((val & 0xf0) == srev_names[i].sr_val)
			name = srev_names[i].sr_name;

		if ((val & 0xff) == srev_names[i].sr_val) {
			name = srev_names[i].sr_name;
			break;
		}
	}

	return name;
}

static int __devinit
ath5k_pci_probe(struct pci_dev *pdev,
		const struct pci_device_id *id)
{
	struct net_device *netdev;
	struct ath5k_softc *sc;
	void __iomem *mem;
	int ret, rate_idx;
	u8 csz;
	u8 mac[ETH_ALEN];
	
	ret = 1;

	/* Inicializar netdevice */
	netdev = alloc_etherdev(sizeof(struct ath5k_softc));
	if (netdev == NULL) 
	{
		ATH5K_ERR(NULL, "can't allocate net_device structure\n");
		goto err;
	}

	sc = netdev_priv(netdev);
	sc->netdev = netdev;
	sc->pdev = pdev;
	SET_NETDEV_DEV(netdev, &pdev->dev);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30)
	netdev->open = ath5k_open;
	netdev->stop = ath5k_close;
	netdev->hard_start_xmit = ath5k_tx;
	netdev->do_ioctl = ath5k_ioctl;
#else
	netdev->netdev_ops = &ath5k_netdev_ops;
#endif
	netdev->tx_queue_len = ATH_TXBUF;

	ret = pci_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "can't enable device\n");
		goto err_netdev;
	}

	/* XXX 32-bit addressing only */
	ret = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "32-bit DMA not available\n");
		goto err_dis;
	}

	/*
	 * Cache line size is used to size and align various
	 * structures used to communicate with the hardware.
	 */
	pci_read_config_byte(pdev, PCI_CACHE_LINE_SIZE, &csz);
	if (csz == 0) {
		/*
		 * Linux 2.4.18 (at least) writes the cache line size
		 * register as a 16-bit wide register which is wrong.
		 * We must have this setup properly for rx buffer
		 * DMA to work so force a reasonable value here if it
		 * comes up zero.
		 */
		csz = L1_CACHE_BYTES >> 2;
		pci_write_config_byte(pdev, PCI_CACHE_LINE_SIZE, csz);
	}
	/*
	 * The default setting of latency timer yields poor results,
	 * set it to the value used by other systems.  It may be worth
	 * tweaking this setting more.
	 */
	pci_write_config_byte(pdev, PCI_LATENCY_TIMER, 0xa8);

	/* Enable bus mastering */
	pci_set_master(pdev);

	/*
	 * Disable the RETRY_TIMEOUT register (0x41) to keep
	 * PCI Tx retries from interfering with C3 CPU state.
	 */
	pci_write_config_byte(pdev, 0x41, 0);

	ret = pci_request_region(pdev, 0, "ath5k");
	if (ret) {
		dev_err(&pdev->dev, "cannot reserve PCI memory region\n");
		goto err_dis;
	}

	mem = pci_iomap(pdev, 0, 0);
	if (!mem) {
		dev_err(&pdev->dev, "cannot remap PCI memory region\n") ;
		ret = -EIO;
		goto err_reg;
	}

	/* Set debug level */
	ath5k_set_debug_level(sc);

	/*
	 * Mark the device as detached to avoid processing
	 * interrupts until setup is complete.
	 */
	__set_bit(ATH_STAT_INVALID, sc->status);

	sc->iobase = mem; /* So we can unmap it on detach */
	sc->cachelsz = csz << 2; /* convert to bytes */
	
	mutex_init(&sc->lock);
	spin_lock_init(&sc->rxbuflock);
	spin_lock_init(&sc->txbuflock);

	/* Set private data */
	pci_set_drvdata(pdev, netdev);

	/* Setup interrupt handler */
	/* In ath5k_open, when the user brings up the interface */

	/*If we passed the test malloc a ath5k_hw struct*/
	sc->ah = kzalloc(sizeof(struct ath5k_hw), GFP_KERNEL);
	if (!sc->ah) {
		ret = -ENOMEM;
		ATH5K_ERR(sc, "out of memory\n");
		goto err_reg;
	}

	sc->ah->ah_sc = sc;
	sc->ah->ah_iobase = sc->iobase;

	/* Initialize device */
	ret = ath5k_hw_attach(sc);
	if (ret) {
		ATH5K_ERR(sc, "error attaching hardware\n");
		goto err_free;
	}

	/* Finish private driver data initialization */
	ret = ath5k_attach(sc);
	if (ret)
		goto err_ah;

	/* Set MAC address */
	ath5k_hw_get_lladdr(sc->ah, mac);
	memcpy(netdev->dev_addr, mac, ETH_ALEN);
	memcpy(netdev->perm_addr, mac, ETH_ALEN);

	ATH5K_INFO(sc, "Atheros AR%s chip found (MAC: 0x%x, PHY: 0x%x)\n",
			ath5k_chip_name(AR5K_VERSION_MAC, sc->ah->ah_mac_srev),
					sc->ah->ah_mac_srev,
					sc->ah->ah_phy_revision);

	if (!sc->ah->ah_single_chip) {
		/* Single chip radio (!RF5111) */
		if (sc->ah->ah_radio_5ghz_revision &&
			!sc->ah->ah_radio_2ghz_revision) {
			/* No 5GHz support -> report 2GHz radio */
			if (!test_bit(AR5K_MODE_11A,
				sc->ah->ah_capabilities.cap_mode)) {
				ATH5K_INFO(sc, "RF%s 2GHz radio found (0x%x)\n",
					ath5k_chip_name(AR5K_VERSION_RAD,
						sc->ah->ah_radio_5ghz_revision),
						sc->ah->ah_radio_5ghz_revision);
			/* No 2GHz support (5110 and some
			 * 5Ghz only cards) -> report 5Ghz radio */
			} else if (!test_bit(AR5K_MODE_11B,
				sc->ah->ah_capabilities.cap_mode)) {
				ATH5K_INFO(sc, "RF%s 5GHz radio found (0x%x)\n",
					ath5k_chip_name(AR5K_VERSION_RAD,
						sc->ah->ah_radio_5ghz_revision),
						sc->ah->ah_radio_5ghz_revision);
			/* Multiband radio */
			} else {
				ATH5K_INFO(sc, "RF%s multiband radio found"
					" (0x%x)\n",
					ath5k_chip_name(AR5K_VERSION_RAD,
						sc->ah->ah_radio_5ghz_revision),
						sc->ah->ah_radio_5ghz_revision);
			}
		}
		/* Multi chip radio (RF5111 - RF2111) ->
		 * report both 2GHz/5GHz radios */
		else if (sc->ah->ah_radio_5ghz_revision &&
				sc->ah->ah_radio_2ghz_revision){
			ATH5K_INFO(sc, "RF%s 5GHz radio found (0x%x)\n",
				ath5k_chip_name(AR5K_VERSION_RAD,
					sc->ah->ah_radio_5ghz_revision),
					sc->ah->ah_radio_5ghz_revision);
			ATH5K_INFO(sc, "RF%s 2GHz radio found (0x%x)\n",
				ath5k_chip_name(AR5K_VERSION_RAD,
					sc->ah->ah_radio_2ghz_revision),
					sc->ah->ah_radio_2ghz_revision);
		}
	}


	/* ready to process interrupts */
	__clear_bit(ATH_STAT_INVALID, sc->status);

	/* Set channel pointers */
	sc->curchan = ath5k_get_channel(sc, 2412);
	if (sc->curchan == NULL)
	{
		ATH5K_ERR(sc, "Can't set channel %d\n", 2412);
		ret = -1;
		goto err_detach;
	}
	sc->curband = &sc->sbands[sc->curchan->band];

	/* Configurar tx_info */
	sc->tx_info.band = sc->curband->band;
	rate_idx = ath5k_get_rate_idx(sc, sc->curband->band, RATE_54M);
	if (rate_idx < 0 || rate_idx > AR5K_MAX_RATES)
	{
		ATH5K_ERR(sc, "Can't set rate %d\n", RATE_54M);
		ret = -1;
		goto err_detach;
	}
	sc->tx_info.rate_idx = 	rate_idx;
	sc->tx_info.count = 1;
	sc->tx_info.wait_for_ack = false;
	sc->tx_info.use_short_preamble = false;

	/* Configurar filtro */
	ath5k_config_filter(sc, true, false, false);

	/* Configurar el envio de ACK en respuesta a tramas unicast */
	ath5k_config_disable_ack(sc, false);

	/* Configurar potencia en pasos de medio dB
	 * Para las tarjetas Dlink -> 15 dBm.
	 * Para las tarjetas Ubiquity -> 15 dBm. Tienen 10 dBm de offset en la propia
	 * tarjeta en modo G, con lo que la potencia real es 25 dBm o 300 mW en G y
	 * 15 dBm en A.
	 */
	sc->power_level = 15;

	/* Registrar netdevice */
	ret = register_netdev(netdev);
	if (ret)
	{
		ATH5K_ERR(NULL, "can't register net_device structure\n");
		goto err_detach;
	}

	/* Print initial config */
	ATH5K_INFO(sc, "Freq %d Mhz, Rate (%d * 100) kps, Power %d dBm\n", 
			sc->curchan->center_freq, RATE_54M, sc->power_level);

   /* RUBEN */
   /* The first registered interface will be the one used by RT-WMP */
   if (!ath5k_slave_dev) {
      ath5k_slave_dev = netdev;
   }

	return 0;
err_detach:
	ath5k_detach(pdev, sc);
err_ah:
	ath5k_hw_detach(sc->ah);
err_free:
	pci_iounmap(pdev, mem);
err_reg:
	pci_release_region(pdev, 0);
err_dis:
	pci_disable_device(pdev);
err_netdev:
	pci_set_drvdata(pdev, NULL);
	free_netdev(netdev);
err:
	return ret;
}

static void __devexit
ath5k_pci_remove(struct pci_dev *pdev)
{
	struct net_device *netdev = pci_get_drvdata(pdev);

	if (netdev) {
		struct ath5k_softc *sc = netdev_priv(netdev);

		unregister_netdev(netdev);
		ath5k_detach(pdev, sc);
		ath5k_hw_detach(sc->ah);
		/* Free IRQ in ath5k_close */
		pci_iounmap(pdev, sc->iobase);
		free_netdev(netdev);
		pci_release_region(pdev, 0);
		pci_disable_device(pdev);
		pci_set_drvdata(pdev, NULL);
	}
}

	static int
ath5k_open(struct net_device *netdev)
{
	struct ath5k_softc *sc = netdev_priv(netdev);
	int err;

	if ((err = request_irq(sc->pdev->irq, ath5k_intr, IRQF_SHARED, sc->netdev->name, sc->netdev))) {
		ATH5K_ERR(sc, "request_irq failed\n");
		goto err_irq;
	}

	ath5k_start(sc);

	/* Half dB steps */
	ath5k_hw_set_txpower_limit(sc->ah, (sc->power_level * 2));

	/* Configurar modo de antena */
	ath5k_hw_set_antenna_mode(sc->ah, AR5K_ANTMODE_DEFAULT);

	/* Arrancar la cola de paquetes para tx cuando este todo listo */
	netif_start_queue(netdev);

	return 0;
	
err_irq:
	return err;
}

static int
ath5k_close(struct net_device *netdev)
{
	struct ath5k_softc *sc = netdev_priv(netdev);

	/* Parar la cola de transmision */
	netif_stop_queue(netdev);

	ath5k_stop(sc);
	
	free_irq(sc->pdev->irq, sc->netdev);

	return 0;
}

/***********************\
* Driver Initialization *
\***********************/

static int
ath5k_attach(struct ath5k_softc *sc)
{
	struct pci_dev *pdev = sc->pdev;
	struct ath5k_hw *ah = sc->ah;
	int ret;

	ATH5K_DBG(sc, ATH5K_DEBUG_ANY, "devid 0x%x\n", pdev->device);

	/*
	 * Check if the MAC has multi-rate retry support.
	 * We do this by trying to setup a fake extended
	 * descriptor.  MAC's that don't have support will
	 * return false w/o doing anything.  MAC's that do
	 * support it will return true w/o doing anything.
	 */
	ret = ah->ah_setup_mrr_tx_desc(ah, NULL, 0, 0, 0, 0, 0, 0);
	if (ret < 0)
		goto err;
	if (ret > 0)
		__set_bit(ATH_STAT_MRRETRY, sc->status);

	/*
	 * Collect the channel list.  The 802.11 layer
	 * is resposible for filtering this list based
	 * on settings like the phy mode and regulatory
	 * domain restrictions.
	 */
	ret = ath5k_setup_bands(sc);
	if (ret) {
		ATH5K_ERR(sc, "can't get channels\n");
		goto err;
	}

	/* NB: setup here so ath5k_rate_update is happy */
	if (test_bit(AR5K_MODE_11A, ah->ah_modes))
		sc->curband = &sc->sbands[IEEE80211_BAND_5GHZ];
	else
		sc->curband = &sc->sbands[IEEE80211_BAND_2GHZ];

	/*
	 * Allocate tx+rx descriptors and populate the lists.
	 */
	ret = ath5k_desc_alloc(sc, pdev);
	if (ret) {
		ATH5K_ERR(sc, "can't allocate descriptors\n");
		goto err;
	}

	/*
	 * Allocate hardware transmit queue. Note that hw functions 
	 * handle reseting these queues at the needed time.
	 */
	sc->txq = ath5k_txq_setup(sc, AR5K_TX_QUEUE_DATA, AR5K_WME_AC_BK);
	if (IS_ERR(sc->txq)) {
		ATH5K_ERR(sc, "can't setup xmit queue\n");
		ret = PTR_ERR(sc->txq);
		goto err_desc;
	}

	tasklet_init(&sc->restq, ath5k_tasklet_reset, (unsigned long)sc);
//	tasklet_init(&sc->calib, ath5k_tasklet_calibrate, (unsigned long)sc);

	/* All MAC address bits matter for ACKs */
	memset(sc->bssidmask, 0xff, ETH_ALEN);
	ath5k_hw_set_bssid_mask(sc->ah, sc->bssidmask);

	return 0;
err_desc:
	ath5k_desc_free(sc, pdev);
err:
	return ret;
}

static void
ath5k_detach(struct pci_dev *pdev, struct ath5k_softc *sc)
{
	/*
	 * NB: the order of these is important:
	 * o call the 802.11 layer before detaching ath5k_hw to
	 *   insure callbacks into the driver to delete global
	 *   key cache entries can be handled
	 * o reclaim the tx queue data structures after calling
	 *   the 802.11 layer as we'll get called back to reclaim
	 *   node state and potentially want to use them
	 * o to cleanup the tx queues the hal is called, so detach
	 *   it last
	 * XXX: ??? detach ath5k_hw ???
	 * Other than that, it's straightforward...
	 */
	ath5k_desc_free(sc, pdev);
	ath5k_txq_release(sc);
}




/********************\
* Channel/mode setup *
\********************/
/* Ugly macro to convert literal channel numbers into their mhz equivalents
 * There are certianly some conditions that will break this (like feeding it '30')
 * but they shouldn't arise since nothing talks on channel 30. */
#define ieee80211chan2mhz(x) \
        (((x) <= 14) ? \
        (((x) == 14) ? 2484 : ((x) * 5) + 2407) : \
        ((x) + 1000) * 5)

/*
 * Convert IEEE channel number to MHz frequency.
 */
static inline short
ath5k_ieee2mhz(short chan)
{
	if (chan <= 14 || chan >= 27)
		return ieee80211chan2mhz(chan);
	else
		return 2212 + chan * 20;
}

/*
 * Returns true for the channel numbers used without all_channels modparam.
 */
static bool ath5k_is_standard_channel(short chan)
{
	return ((chan <= 14) ||
		/* UNII 1,2 */
		((chan & 3) == 0 && chan >= 36 && chan <= 64) ||
		/* midband */
		((chan & 3) == 0 && chan >= 100 && chan <= 140) ||
		/* UNII-3 */
		((chan & 3) == 1 && chan >= 149 && chan <= 165));
}

static unsigned int
ath5k_copy_channels(struct ath5k_hw *ah,
		struct ieee80211_channel *channels,
		unsigned int mode,
		unsigned int max)
{
	unsigned int i, count, size, chfreq, freq, ch;

	if (!test_bit(mode, ah->ah_modes))
		return 0;

	switch (mode) {
	case AR5K_MODE_11A:
	case AR5K_MODE_11A_TURBO:
		/* 1..220, but 2GHz frequencies are filtered by check_channel */
		size = 220 ;
		chfreq = CHANNEL_5GHZ;
		break;
	case AR5K_MODE_11B:
	case AR5K_MODE_11G:
	case AR5K_MODE_11G_TURBO:
		size = 26;
		chfreq = CHANNEL_2GHZ;
		break;
	default:
		ATH5K_WARN(ah->ah_sc, "bad mode, not copying channels\n");
		return 0;
	}

	for (i = 0, count = 0; i < size && max > 0; i++) {
		ch = i + 1;
		freq = ath5k_ieee2mhz(ch);

		/* Check if channel is supported by the chipset */
		if (!ath5k_channel_ok(ah, freq, chfreq))
			continue;

		if (!modparam_all_channels && !ath5k_is_standard_channel(ch))
			continue;

		/* Write channel info and increment counter */
		channels[count].center_freq = freq;
		channels[count].band = (chfreq == CHANNEL_2GHZ) ?
			IEEE80211_BAND_2GHZ : IEEE80211_BAND_5GHZ;
		switch (mode) {
		case AR5K_MODE_11A:
		case AR5K_MODE_11G:
			channels[count].hw_value = chfreq | CHANNEL_OFDM;
			break;
		case AR5K_MODE_11A_TURBO:
		case AR5K_MODE_11G_TURBO:
			channels[count].hw_value = chfreq |
				CHANNEL_OFDM | CHANNEL_TURBO;
			break;
		case AR5K_MODE_11B:
			channels[count].hw_value = CHANNEL_B;
		}

		count++;
		max--;
	}

	return count;
}

static int
ath5k_setup_bands(struct ath5k_softc *sc)
{
	struct ath5k_hw *ah = sc->ah;
	struct ieee80211_supported_band *sband;
	int max_c, count_c = 0;
	int i;

	BUILD_BUG_ON(ARRAY_SIZE(sc->sbands) < IEEE80211_NUM_BANDS);
	max_c = ARRAY_SIZE(sc->channels);

	/* 2GHz band */
	sband = &sc->sbands[IEEE80211_BAND_2GHZ];
	sband->band = IEEE80211_BAND_2GHZ;
	sband->bitrates = &sc->rates[IEEE80211_BAND_2GHZ][0];

	if (test_bit(AR5K_MODE_11G, sc->ah->ah_capabilities.cap_mode)) {
		/* G mode */
		memcpy(sband->bitrates, &ath5k_rates[0],
		       sizeof(struct ieee80211_rate) * 12);
		sband->n_bitrates = 12;

		sband->channels = sc->channels;
		sband->n_channels = ath5k_copy_channels(ah, sband->channels,
					AR5K_MODE_11G, max_c);

		count_c = sband->n_channels;
		max_c -= count_c;
	} else if (test_bit(AR5K_MODE_11B, sc->ah->ah_capabilities.cap_mode)) {
		/* B mode */
		memcpy(sband->bitrates, &ath5k_rates[0],
		       sizeof(struct ieee80211_rate) * 4);
		sband->n_bitrates = 4;

		/* 5211 only supports B rates and uses 4bit rate codes
		 * (e.g normally we have 0x1B for 1M, but on 5211 we have 0x0B)
		 * fix them up here:
		 */
		if (ah->ah_version == AR5K_AR5211) {
			for (i = 0; i < 4; i++) {
				sband->bitrates[i].hw_value =
					sband->bitrates[i].hw_value & 0xF;
				sband->bitrates[i].hw_value_short =
					sband->bitrates[i].hw_value_short & 0xF;
			}
		}

		sband->channels = sc->channels;
		sband->n_channels = ath5k_copy_channels(ah, sband->channels,
					AR5K_MODE_11B, max_c);

		count_c = sband->n_channels;
		max_c -= count_c;
	}

	/* 5GHz band, A mode */
	if (test_bit(AR5K_MODE_11A, sc->ah->ah_capabilities.cap_mode)) {
		sband = &sc->sbands[IEEE80211_BAND_5GHZ];
		sband->band = IEEE80211_BAND_5GHZ;
		sband->bitrates = &sc->rates[IEEE80211_BAND_5GHZ][0];

		memcpy(sband->bitrates, &ath5k_rates[4],
		       sizeof(struct ieee80211_rate) * 8);
		sband->n_bitrates = 8;

		sband->channels = &sc->channels[count_c];
		sband->n_channels = ath5k_copy_channels(ah, sband->channels,
					AR5K_MODE_11A, max_c);

	}

	ath5k_debug_dump_bands(sc);

	return 0;
}

static void
ath5k_mode_setup(struct ath5k_softc *sc)
{
	struct ath5k_hw *ah = sc->ah;
	u32 rfilt;

	/* configure rx filter */
	rfilt = sc->filter_flags;
	ath5k_hw_set_rx_filter(ah, rfilt);

	if (ath5k_hw_hasbssidmask(ah))
		ath5k_hw_set_bssid_mask(ah, sc->bssidmask);

	/* configure operational mode */
	ath5k_hw_set_opmode(ah);

	ath5k_hw_set_mcast_filter(ah, 0, 0);
	ATH5K_DBG(sc, ATH5K_DEBUG_MODE, "RX filter 0x%x\n", rfilt);
}

/***************\
* Buffers setup *
\***************/

static
struct sk_buff *ath5k_rx_skb_alloc(struct ath5k_softc *sc, dma_addr_t *skb_addr)
{
	struct sk_buff *skb;
	unsigned int off;

	/*
	 * Allocate buffer with headroom_needed space for the
	 * fake physical layer header at the start.
	 */
	skb = dev_alloc_skb(sc->rxbufsize + sc->cachelsz - 1);

	if (!skb) {
		ATH5K_ERR(sc, "can't alloc skbuff of size %u\n",
				sc->rxbufsize + sc->cachelsz - 1);
		return NULL;
	}
	/*
	 * Cache-line-align.  This is important (for the
	 * 5210 at least) as not doing so causes bogus data
	 * in rx'd frames.
	 */
	off = ((unsigned long)skb->data) % sc->cachelsz;
	if (off != 0)
		skb_reserve(skb, sc->cachelsz - off);

	*skb_addr = pci_map_single(sc->pdev,
		skb->data, sc->rxbufsize, PCI_DMA_FROMDEVICE);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
	if (unlikely(pci_dma_mapping_error(sc->pdev, *skb_addr))) {
#else
	if (unlikely(pci_dma_mapping_error(*skb_addr))) {
#endif
		ATH5K_ERR(sc, "%s: DMA mapping failed\n", __func__);
		dev_kfree_skb(skb);
		return NULL;
	}
	return skb;
}

static int
ath5k_rxbuf_setup(struct ath5k_softc *sc, struct ath5k_buf *bf)
{
	struct ath5k_hw *ah = sc->ah;
	struct sk_buff *skb = bf->skb;
	struct ath5k_desc *ds;

	if (!skb) {
		skb = ath5k_rx_skb_alloc(sc, &bf->skbaddr);
		if (!skb)
			return -ENOMEM;
		bf->skb = skb;
	}

	/*
	 * Setup descriptors.  For receive we always terminate
	 * the descriptor list with a self-linked entry so we'll
	 * not get overrun under high load (as can happen with a
	 * 5212 when ANI processing enables PHY error frames).
	 *
	 * To insure the last descriptor is self-linked we create
	 * each descriptor as self-linked and add it to the end.  As
	 * each additional descriptor is added the previous self-linked
	 * entry is ``fixed'' naturally.  This should be safe even
	 * if DMA is happening.  When processing RX interrupts we
	 * never remove/process the last, self-linked, entry on the
	 * descriptor list.  This insures the hardware always has
	 * someplace to write a new frame.
	 */
	ds = bf->desc;
	ds->ds_link = bf->daddr;	/* link to self */
	ds->ds_data = bf->skbaddr;
	ah->ah_setup_rx_desc(ah, ds,
		skb_tailroom(skb),	/* buffer size */
		0);

	if (sc->rxlink != NULL)
		*sc->rxlink = bf->daddr;
	sc->rxlink = &ds->ds_link;
	return 0;
}

/* Contexto: UC
 *
 * Puede ocurrir:
 * - Lo peor, que nos saque una hardIRQ de la cpu -> spin_lock_irq
 */
static int
ath5k_txbuf_setup(struct ath5k_softc *sc, struct ath5k_buf *bf)
{
	struct ath5k_hw *ah = sc->ah;
	struct ath5k_txq *txq = sc->txq;
	struct ath5k_desc *ds = bf->desc;
	struct sk_buff *skb = bf->skb;
	struct ieee80211_tx_info *info = &sc->tx_info;
	unsigned int pktlen, flags, keyidx = AR5K_TXKEYIX_INVALID;
	struct ieee80211_rate *rate;
	int ret;
	u16 hw_rate;
	u16 cts_rate = 0;
	u16 duration = 0;

	flags = AR5K_TXDESC_INTREQ | AR5K_TXDESC_CLRDMASK;

	/* XXX endianness */
	bf->skbaddr = pci_map_single(sc->pdev, skb->data, skb->len,
			PCI_DMA_TODEVICE);
        
	if (info->rate_idx >= 0 && info->rate_idx < AR5K_MAX_RATES)
		rate = &sc->sbands[info->band].bitrates[info->rate_idx];
	else
	{
		ATH5K_ERR(sc, "Rate index invalid\n");
		return -1;
	}

	if (!info->wait_for_ack)
		flags |= AR5K_TXDESC_NOACK;

	hw_rate = info->use_short_preamble ? rate->hw_value_short : rate->hw_value;

	pktlen = skb->len;

	/* FIXME: If we are in g mode and rate is a CCK rate
	 * subtract ah->ah_txpower.txp_cck_ofdm_pwr_delta
	 * from tx power (value is in dB units already) */
	ret = ah->ah_setup_tx_desc(ah, ds, pktlen,
		ath5k_get_hdrlen_from_skb(skb), AR5K_PKT_TYPE_NORMAL,
		(sc->power_level * 2),
		hw_rate,
		info->count, keyidx, ah->ah_tx_ant, flags,
		cts_rate, duration);
	if (ret)
		goto err_unmap;

	ds->ds_link = 0;
	ds->ds_data = bf->skbaddr;

	spin_lock_irq(&txq->lock);
	list_add_tail(&bf->list, &txq->q);
	if (txq->link == NULL) /* is this first packet? */
		ath5k_hw_set_txdp(ah, txq->qnum, bf->daddr);
	else /* no, so only link it */
		*txq->link = bf->daddr;

	txq->link = &ds->ds_link;
	ath5k_hw_start_tx_dma(ah, txq->qnum);
	mmiowb();
	spin_unlock_irq(&txq->lock);

	return 0;
err_unmap:
	pci_unmap_single(sc->pdev, bf->skbaddr, skb->len, PCI_DMA_TODEVICE);
	return ret;
}

/*******************\
* Descriptors setup *
\*******************/

static int
ath5k_desc_alloc(struct ath5k_softc *sc, struct pci_dev *pdev)
{
	struct ath5k_desc *ds;
	struct ath5k_buf *bf;
	dma_addr_t da;
	unsigned int i;
	int ret;

	/* allocate descriptors */
	sc->desc_len = sizeof(struct ath5k_desc) *
			(ATH_TXBUF + ATH_RXBUF + 1);
	sc->desc = pci_alloc_consistent(pdev, sc->desc_len, &sc->desc_daddr);
	if (sc->desc == NULL) {
		ATH5K_ERR(sc, "can't allocate descriptors\n");
		ret = -ENOMEM;
		goto err;
	}
	ds = sc->desc;
	da = sc->desc_daddr;
	ATH5K_DBG(sc, ATH5K_DEBUG_ANY, "DMA map: %p (%zu) -> %llx\n",
		ds, sc->desc_len, (unsigned long long)sc->desc_daddr);

	bf = kcalloc(1 + ATH_TXBUF + ATH_RXBUF,
			sizeof(struct ath5k_buf), GFP_KERNEL);
	if (bf == NULL) {
		ATH5K_ERR(sc, "can't allocate bufptr\n");
		ret = -ENOMEM;
		goto err_free;
	}
	sc->bufptr = bf;

	INIT_LIST_HEAD(&sc->rxbuf);
	for (i = 0; i < ATH_RXBUF; i++, bf++, ds++, da += sizeof(*ds)) {
		bf->desc = ds;
		bf->daddr = da;
		list_add_tail(&bf->list, &sc->rxbuf);
	}

	INIT_LIST_HEAD(&sc->txbuf);
	sc->txbuf_len = ATH_TXBUF;
	for (i = 0; i < ATH_TXBUF; i++, bf++, ds++,
			da += sizeof(*ds)) {
		bf->desc = ds;
		bf->daddr = da;
		list_add_tail(&bf->list, &sc->txbuf);
	}

	return 0;
err_free:
	pci_free_consistent(pdev, sc->desc_len, sc->desc, sc->desc_daddr);
err:
	sc->desc = NULL;
	return ret;
}

static void
ath5k_desc_free(struct ath5k_softc *sc, struct pci_dev *pdev)
{
	struct ath5k_buf *bf;

	list_for_each_entry(bf, &sc->txbuf, list)
		ath5k_txbuf_free(sc, bf);
	list_for_each_entry(bf, &sc->rxbuf, list)
		ath5k_rxbuf_free(sc, bf);

	/* Free memory associated with all descriptors */
	pci_free_consistent(pdev, sc->desc_len, sc->desc, sc->desc_daddr);

	kfree(sc->bufptr);
	sc->bufptr = NULL;
}





/**************\
* Queues setup *
\**************/

static struct ath5k_txq *
ath5k_txq_setup(struct ath5k_softc *sc,
		int qtype, int subtype)
{
	struct ath5k_hw *ah = sc->ah;
	struct ath5k_txq *txq;
	struct ath5k_txq_info qi = {
		.tqi_subtype = subtype,
		.tqi_aifs = AR5K_TXQ_USEDEFAULT,
		.tqi_cw_min = AR5K_TXQ_USEDEFAULT,
		.tqi_cw_max = AR5K_TXQ_USEDEFAULT
	};
	int qnum;

	/*
	 * Enable interrupts only for EOL and DESC conditions.
	 * We mark tx descriptors to receive a DESC interrupt
	 * when a tx queue gets deep; otherwise waiting for the
	 * EOL to reap descriptors.  Note that this is done to
	 * reduce interrupt load and this only defers reaping
	 * descriptors, never transmitting frames.  Aside from
	 * reducing interrupts this also permits more concurrency.
	 * The only potential downside is if the tx queue backs
	 * up in which case the top half of the kernel may backup
	 * due to a lack of tx descriptors.
	 */
	qi.tqi_flags = AR5K_TXQ_FLAG_TXEOLINT_ENABLE |
				AR5K_TXQ_FLAG_TXDESCINT_ENABLE;
	qnum = ath5k_hw_setup_tx_queue(ah, qtype, &qi);
	if (qnum < 0) {
		/*
		 * NB: don't print a message, this happens
		 * normally on parts with too few tx queues
		 */
		return ERR_PTR(qnum);
	}
	if (qnum >= ARRAY_SIZE(sc->txqs)) {
		ATH5K_ERR(sc, "hw qnum %u out of range, max %tu!\n",
			qnum, ARRAY_SIZE(sc->txqs));
		ath5k_hw_release_tx_queue(ah, qnum);
		return ERR_PTR(-EINVAL);
	}
	txq = &sc->txqs[qnum];
	if (!txq->setup) {
		txq->qnum = qnum;
		txq->link = NULL;
		INIT_LIST_HEAD(&txq->q);
		spin_lock_init(&txq->lock);
		txq->setup = true;
	}
	return &sc->txqs[qnum];
}

/* Contexto UC y softIRQ
 *
 * Puede ocurrir:
 * - Nos interrumpa una hardIRQ -> spin_lock_irq
 * - Nos expulsen de la cpu -> spin_lock
 * - Una hardIRQ, softIRQ o UC esta en ejecucion en otra CPU -> spin_lock
 */
static void
ath5k_txq_drainq(struct ath5k_softc *sc, struct ath5k_txq *txq)
{
	struct ath5k_buf *bf, *bf0;

	/*
	 * NB: this assumes output has been stopped and
	 *     we do not need to block ath5k_tx_tasklet
	 */
	spin_lock_irq(&txq->lock);
	list_for_each_entry_safe(bf, bf0, &txq->q, list) {
		ath5k_debug_printtxbuf(sc, bf);

		ath5k_txbuf_free(sc, bf);

		spin_lock_bh(&sc->txbuflock);
		list_move_tail(&bf->list, &sc->txbuf);
		sc->txbuf_len++;
		spin_unlock_irq(&sc->txbuflock);
	}
	txq->link = NULL;
	spin_unlock_irq(&txq->lock);
}

/*
 * Drain the transmit queues and reclaim resources.
 */
static void
ath5k_txq_cleanup(struct ath5k_softc *sc)
{
	struct ath5k_hw *ah = sc->ah;
	unsigned int i;

	/* XXX return value */
	if (likely(!test_bit(ATH_STAT_INVALID, sc->status))) {
		/* don't touch the hardware if marked invalid */
		for (i = 0; i < ARRAY_SIZE(sc->txqs); i++)
			if (sc->txqs[i].setup) {
				ath5k_hw_stop_tx_dma(ah, sc->txqs[i].qnum);
				ATH5K_DBG(sc, ATH5K_DEBUG_RESET, "txq [%u] %x, "
					"link %p\n",
					sc->txqs[i].qnum,
					ath5k_hw_get_txdp(ah,
							sc->txqs[i].qnum),
					sc->txqs[i].link);
			}
	}
	netif_wake_queue(sc->netdev);

	for (i = 0; i < ARRAY_SIZE(sc->txqs); i++)
		if (sc->txqs[i].setup)
			ath5k_txq_drainq(sc, &sc->txqs[i]);
}

static void
ath5k_txq_release(struct ath5k_softc *sc)
{
	struct ath5k_txq *txq = sc->txqs;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(sc->txqs); i++, txq++)
		if (txq->setup) {
			ath5k_hw_release_tx_queue(sc->ah, txq->qnum);
			txq->setup = false;
		}
}




/*************\
* RX Handling *
\*************/

/*
 * Enable the receive h/w following a reset.
 *
 * Contexto SoftIRQ y UC.
 *
 * Mientras ejecuto en una CPU puede ocurrir:
 *  - Ser interrumpido por una hardIRQ si estoy en SoftIRQ o UC (bloquear irq)
 *  - Otra CPU ejecuta softIRQ si estoy en UC (spinlock)
 *  - Otra CPU ejecutando UC si estoy en UC (spinlock)
 */
static int
ath5k_rx_start(struct ath5k_softc *sc)
{
	struct ath5k_hw *ah = sc->ah;
	struct ath5k_buf *bf;
	int ret;

	sc->rxbufsize = roundup(IEEE80211_MAX_LEN, sc->cachelsz);

	ATH5K_DBG(sc, ATH5K_DEBUG_RESET, "cachelsz %u rxbufsize %u\n",
		sc->cachelsz, sc->rxbufsize);

	spin_lock_irq(&sc->rxbuflock);
	sc->rxlink = NULL;
	list_for_each_entry(bf, &sc->rxbuf, list) {
		ret = ath5k_rxbuf_setup(sc, bf);
		if (ret != 0) {
			spin_unlock_irq(&sc->rxbuflock);
			goto err;
		}
	}
	bf = list_first_entry(&sc->rxbuf, struct ath5k_buf, list);
	ath5k_hw_set_rxdp(ah, bf->daddr);
	spin_unlock_irq(&sc->rxbuflock);

	ath5k_hw_start_rx_dma(ah);	/* enable recv descriptors */
	ath5k_mode_setup(sc);		/* set filters, etc. */
	ath5k_hw_start_rx_pcu(ah);	/* re-enable PCU/DMA engine */

	return 0;
err:
	return ret;
}

/*
 * Disable the receive h/w in preparation for a reset.
 */
static void
ath5k_rx_stop(struct ath5k_softc *sc)
{
	struct ath5k_hw *ah = sc->ah;

	ath5k_hw_stop_rx_pcu(ah);	/* disable PCU */
	ath5k_hw_set_rx_filter(ah, 0);	/* clear recv filter */
	ath5k_hw_stop_rx_dma(ah);	/* disable DMA engine */

	ath5k_debug_printrxbuffs(sc, ah);

	sc->rxlink = NULL;		/* just in case */
}

/* 
 * Contexto hardIRQ.
 *
 * Mientras ejecuto en una CPU puede ocurrir:
 *
 *  - El kernel garantiza que una ISR no se ejecuta simultaneamente
 *    en varios uP, y un tasklet no puede interrumpir a una ISR.
 *  - Que un tasklet o UC se este ejecutando en otra CPU, luego hay que proteger
 *    con un spin_lock para esperar que termine la tarea en el otro uP.
 *
 * Ademas, hay que tener en cuenta que esta funcion reserva memoria con una
 * llamada a dev_alloc_skb, pero esta permitido ya que se reserva con
 * GFP_ATOMIC y el kernel garantiza que no duerme.
 */
static void
ath5k_rx_done(struct ath5k_softc *sc)
{
	struct ieee80211_rx_status rxs = {};
	struct ath5k_rx_status rs = {};
	struct sk_buff *skb, *next_skb;
	dma_addr_t next_skb_addr;
	struct ath5k_buf *bf;
	struct ath5k_desc *ds;
	int ret;
	int hdrlen;
	int padsize;

	unsigned char dst[ETH_ALEN];
	unsigned char src[ETH_ALEN];
	u16 ethertype, frame_control;
	struct ieee80211_hdr *hdr;
	unsigned char *payload;

	spin_lock(&sc->rxbuflock);
	if (list_empty(&sc->rxbuf)) {
		ATH5K_WARN(sc, "empty rx buf pool\n");
		goto unlock;
	}
	do {
		rxs.flag = 0;

		bf = list_first_entry(&sc->rxbuf, struct ath5k_buf, list);
		BUG_ON(bf->skb == NULL);
		skb = bf->skb;
		ds = bf->desc;

		/* bail if HW is still using self-linked descriptor */
		if (ath5k_hw_get_rxdp(sc->ah) == bf->daddr)
			break;

		ret = sc->ah->ah_proc_rx_desc(sc->ah, ds, &rs);

		if (unlikely(ret == -EINPROGRESS))
			break;
		else if (unlikely(ret)) {
			ATH5K_ERR(sc, "error in processing rx descriptor\n");
			goto unlock;
		}

		if (unlikely(rs.rs_more)) {
			ATH5K_WARN(sc, "unsupported jumbo\n");
			goto next;
		}

		if (unlikely(rs.rs_status)) {
			if (rs.rs_status & AR5K_RXERR_PHY)
				//ATH5K_ERR(sc, "PHY ERR");
				goto next;
			if (rs.rs_status & AR5K_RXERR_CRC)
				//ATH5K_ERR(sc, "CRC ERR");
				goto next;
			if (rs.rs_status & AR5K_RXERR_DECRYPT)
				//ATH5K_ERR(sc, "DEC ERR");
				goto next;
			if (rs.rs_status & AR5K_RXERR_MIC) {
				//ATH5K_ERR(sc, "MIC");
				goto accept;
			}
		}
accept:
		next_skb = ath5k_rx_skb_alloc(sc, &next_skb_addr);

		/*
		 * If we can't replace bf->skb with a new skb under memory
		 * pressure, just skip this packet
		 */
		if (!next_skb)
			goto next;

		pci_unmap_single(sc->pdev, bf->skbaddr, sc->rxbufsize,
				PCI_DMA_FROMDEVICE);
		skb_put(skb, rs.rs_datalen);

		/* The MAC header is padded to have 32-bit boundary if the
		 * packet payload is non-zero. The general calculation for
		 * padsize would take into account odd header lengths:
		 * padsize = (4 - hdrlen % 4) % 4; However, since only
		 * even-length headers are used, padding can only be 0 or 2
		 * bytes and we can optimize this a bit. In addition, we must
		 * not try to remove padding from short control frames that do
		 * not have payload. */
		hdrlen = ath5k_get_hdrlen_from_skb(skb);
		padsize = ath5k_pad_size(hdrlen);
		if (padsize) {
			memmove(skb->data + padsize, skb->data, hdrlen);
			skb_pull(skb, padsize);
		}

		rxs.freq = sc->curchan->center_freq;
		rxs.band = sc->curband->band;

		rxs.noise = sc->ah->ah_noise_floor;
		rxs.signal = rxs.noise + rs.rs_rssi;

		/* An rssi of 35 indicates you should be able use
		 * 54 Mbps reliably. A more elaborate scheme can be used
		 * here but it requires a map of SNR/throughput for each
		 * possible mode used */
		rxs.qual = rs.rs_rssi * 100 / 35;

		//unsigned char c = rs.rs_rssi;
		//ATH5K_ERR(sc, "ret:%d, DANI->RSSI: %d %u %d, %d\n",ret,c,c,rxs.qual, rxs.noise);

		/* rssi can be more than 35 though, anything above that
		 * should be considered at 100% */
		if (rxs.qual > 100)
			rxs.qual = 100;

		rxs.antenna = rs.rs_antenna;

		ath5k_debug_dump_skb(sc, skb, "RX  ", 0);

		/* skb contains the frame in IEEE 802.11 format, as it was sent over air */
		
		hdr = (struct ieee80211_hdr *)skb->data;
		frame_control = le16_to_cpu(hdr->frame_control);

		/* If no payload jump to next frame */
		if (skb->len < 24)
			goto next;

		/* Extract src/dst addresses */
		switch (frame_control & (IEEE80211_FCTL_FROMDS | IEEE80211_FCTL_TODS)) {
			case IEEE80211_FCTL_FROMDS:
				memcpy(dst, hdr->addr1, ETH_ALEN);
				memcpy(src, hdr->addr3, ETH_ALEN);
				break;
			case IEEE80211_FCTL_TODS:
				memcpy(dst, hdr->addr3, ETH_ALEN);
				memcpy(src, hdr->addr2, ETH_ALEN);
				break;
			case IEEE80211_FCTL_FROMDS | IEEE80211_FCTL_TODS:
				if (skb->len < IEEE80211_4ADDR_LEN)
					goto next;
				memcpy(dst, hdr->addr3, ETH_ALEN);
				memcpy(src, hdr->addr4, ETH_ALEN);
				break;
			case 0:
				memcpy(dst, hdr->addr1, ETH_ALEN);
				memcpy(src, hdr->addr2, ETH_ALEN);
				break;
		}
       
		payload = skb->data + hdrlen;
		ethertype = (payload[6] << 8) | payload[7];

		/* convert hdr + possible LLC headers into Ethernet header */
		if (skb->len - hdrlen >= 8)
		{
			/* remove RFC1042 or Bridge-Tunnel encapsulation and
			 * replace EtherType */
			skb_pull(skb, hdrlen + SNAP_SIZE);
			memcpy(skb_push(skb, ETH_ALEN), src, ETH_ALEN);
			memcpy(skb_push(skb, ETH_ALEN), dst, ETH_ALEN);
		} else {
			goto next;
		}


		/* Set skb fields and send to linux network layer */
		if (skb) 
		{
			skb->dev = sc->netdev;
			skb->protocol = eth_type_trans(skb, sc->netdev);
			skb->ip_summed = CHECKSUM_NONE; /* 802.11 crc not sufficient */


         /* RUBEN */
         /* If 'skb' is a RT-WMP packet, it is enqueued in the reception queue
          * if not, it is sent to the upper layer */
         if (skb->protocol == 0x6969 || skb->protocol == 0x6970) {
            /* XXX: RT-WMP Extension */
            //skb->data[14] = (char)rs.rs_rssi;
            //skb->data[15] = (char)rxs.noise;
            skb->data[0] = (char)rs.rs_rssi;
            skb->data[1] = (char)rxs.noise;

            /* 'skb' is enqueued */
            enqueue_rx_queue(&rx_rtwmp_queue, skb);
         }
         else {
            /* Not a RT-WMP packet, so 'skb' is sent to the upper layer */
            netif_rx(skb);
         }
		}

		bf->skb = next_skb;
		bf->skbaddr = next_skb_addr;
next:
		list_move_tail(&bf->list, &sc->rxbuf);
	} while (ath5k_rxbuf_setup(sc, bf) == 0);
unlock:
	spin_unlock(&sc->rxbuflock);
}




/*************\
* TX Handling *
\*************/

/*
 * Contexto HardIRQ
 *
 * Mientras se ejecuta puede ocurrir:
 * 
 * - El kernel garantiza que no se ejecutara la rutina de interrupcion en otras
 *   CPU.
 * - Puede estar ejecutandose un tasklet o UC en otra CPU, hay que bloquear con
 *   un spin_lock para esperar que terminen
 */  
static void
ath5k_tx_done(struct ath5k_softc *sc, struct ath5k_txq *txq)
{
	struct ath5k_tx_status ts = {};
	struct ath5k_buf *bf, *bf0;
	struct ath5k_desc *ds;
	struct sk_buff *skb;
	int ret;

	spin_lock(&txq->lock);
	list_for_each_entry_safe(bf, bf0, &txq->q, list) {
		ds = bf->desc;

		ret = sc->ah->ah_proc_tx_desc(sc->ah, ds, &ts);
		if (unlikely(ret == -EINPROGRESS))
			break;
		else if (unlikely(ret)) {
			ATH5K_ERR(sc, "error %d while processing queue %u\n",
				ret, txq->qnum);
			break;
		}

		skb = bf->skb;
		dev_kfree_skb_any(skb);
		bf->skb = NULL;

		pci_unmap_single(sc->pdev, bf->skbaddr, skb->len,
				PCI_DMA_TODEVICE);

		if (unlikely(ts.ts_status)) 
			sc->ll_stats.dot11ACKFailureCount++;

		spin_lock(&sc->txbuflock);
		list_move_tail(&bf->list, &sc->txbuf);
		sc->txbuf_len++;
		spin_unlock(&sc->txbuflock);
	}
	if (likely(list_empty(&txq->q)))
		txq->link = NULL;
	spin_unlock(&txq->lock);
	if (sc->txbuf_len > ATH_TXBUF / 5)
		netif_wake_queue(sc->netdev);
}


/********************\
* Interrupt handling *
\********************/

static int
ath5k_init(struct ath5k_softc *sc)
{
	struct ath5k_hw *ah = sc->ah;
	int ret;

	mutex_lock(&sc->lock);

	/*
	 * Stop anything previously setup.  This is safe
	 * no matter this is the first time through or not.
	 */
	ath5k_stop_locked(sc);

	/*
	 * The basic interface to setting the hardware in a good
	 * state is ``reset''.  On return the hardware is known to
	 * be powered up and with interrupts disabled.  This must
	 * be followed by initialization of the appropriate bits
	 * and then setup of the interrupt mask.
	 */
	sc->imask = AR5K_INT_RXOK | AR5K_INT_RXERR | AR5K_INT_RXEOL |
		AR5K_INT_RXORN | AR5K_INT_TXDESC | AR5K_INT_TXEOL |
		AR5K_INT_FATAL | AR5K_INT_GLOBAL | AR5K_INT_SWI;
	ret = ath5k_reset(sc, sc->curchan);
	if (ret)
		goto done;

	/* Set ack to be sent at low bit-rates */
	ath5k_hw_set_ack_bitrate_high(ah, false);

	/* Set PHY calibration inteval */
	ah->ah_cal_intval = ath5k_calinterval;

	ret = 0;
done:
	mmiowb();
	mutex_unlock(&sc->lock);
	return ret;
}

static int
ath5k_stop_locked(struct ath5k_softc *sc)
{
	struct ath5k_hw *ah = sc->ah;

	ATH5K_DBG(sc, ATH5K_DEBUG_RESET, "invalid %u\n",
			test_bit(ATH_STAT_INVALID, sc->status));

	/*
	 * Shutdown the hardware and driver:
	 *    stop output from above
	 *    disable interrupts
	 *    turn off timers
	 *    turn off the radio
	 *    clear transmit machinery
	 *    clear receive machinery
	 *    drain and release tx queues
	 *    reclaim beacon resources
	 *    power down hardware
	 *
	 * Note that some of this work is not possible if the
	 * hardware is gone (invalid).
	 */
	netif_stop_queue(sc->netdev);

	if (!test_bit(ATH_STAT_INVALID, sc->status)) {
		ath5k_hw_set_imr(ah, 0);
		synchronize_irq(sc->pdev->irq);
	}
	ath5k_txq_cleanup(sc);
	if (!test_bit(ATH_STAT_INVALID, sc->status)) {
		ath5k_rx_stop(sc);
		ath5k_hw_phy_disable(ah);
	} else
		sc->rxlink = NULL;

	return 0;
}

/*
 * Stop the device, grabbing the top-level lock to protect
 * against concurrent entry through ath5k_init (which can happen
 * if another thread does a system call and the thread doing the
 * stop is preempted).
 */
static int
ath5k_stop_hw(struct ath5k_softc *sc)
{
	int ret;

	mutex_lock(&sc->lock);
	ret = ath5k_stop_locked(sc);
	if (ret == 0 && !test_bit(ATH_STAT_INVALID, sc->status)) {
		/*
		 * Don't set the card in full sleep mode!
		 *
		 * a) When the device is in this state it must be carefully
		 * woken up or references to registers in the PCI clock
		 * domain may freeze the bus (and system).  This varies
		 * by chip and is mostly an issue with newer parts
		 * (madwifi sources mentioned srev >= 0x78) that go to
		 * sleep more quickly.
		 *
		 * b) On older chips full sleep results a weird behaviour
		 * during wakeup. I tested various cards with srev < 0x78
		 * and they don't wake up after module reload, a second
		 * module reload is needed to bring the card up again.
		 *
		 * Until we figure out what's going on don't enable
		 * full chip reset on any chip (this is what Legacy HAL
		 * and Sam's HAL do anyway). Instead Perform a full reset
		 * on the device (same as initial state after attach) and
		 * leave it idle (keep MAC/BB on warm reset) */
		ret = ath5k_hw_on_hold(sc->ah);

		ATH5K_DBG(sc, ATH5K_DEBUG_RESET,
				"putting device to sleep\n");
	}

	mmiowb();
	mutex_unlock(&sc->lock);

	tasklet_kill(&sc->restq);
//	tasklet_kill(&sc->calib);

	return ret;
}

static irqreturn_t
ath5k_intr(int irq, void *dev_id)
{
	struct net_device *netdev = (struct net_device *)dev_id;
	struct ath5k_softc *sc = netdev_priv(netdev);
	struct ath5k_hw *ah = sc->ah;
	enum ath5k_int status;
	unsigned int counter = 1000;

	if (unlikely(test_bit(ATH_STAT_INVALID, sc->status) ||
				!ath5k_hw_is_intr_pending(ah)))
		return IRQ_NONE;

	do {
		ath5k_hw_get_isr(ah, &status);		/* NB: clears IRQ too */
		ATH5K_DBG(sc, ATH5K_DEBUG_INTR, "status 0x%x/0x%x\n",
				status, sc->imask);
		if (unlikely(status & AR5K_INT_FATAL)) {
			/*
			 * Fatal errors are unrecoverable.
			 * Typically these are caused by DMA errors.
			 */
			ATH5K_ERR(sc, "Fatal error\n");
			tasklet_schedule(&sc->restq);
		} else if (unlikely(status & AR5K_INT_RXORN)) {
			ATH5K_ERR(sc, "Fatal error\n");
			tasklet_schedule(&sc->restq);
		} else {
			if (status & AR5K_INT_RXEOL) {
				/*
				* NB: the hardware should re-read the link when
				*     RXE bit is written, but it doesn't work at
				*     least on older hardware revs.
				*/
				sc->rxlink = NULL;
			}
			if (status & AR5K_INT_TXURN) {
				/* bump tx trigger level */
				ath5k_hw_update_tx_triglevel(ah, true);
			}
			if (status & (AR5K_INT_RXOK | AR5K_INT_RXERR))
				ath5k_rx_done(sc);
			if (status & (AR5K_INT_TXOK | AR5K_INT_TXDESC
					| AR5K_INT_TXERR | AR5K_INT_TXEOL))
				ath5k_tx_done(sc, sc->txq);
//			if (status & AR5K_INT_SWI)
//				tasklet_schedule(&sc->calib);
			if (status & AR5K_INT_MIB) {
				/*
				 * These stats are also used for ANI i think
				 * so how about updating them more often ?
				 */
				ath5k_hw_update_mib_counters(ah, &sc->ll_stats);
			}
		}
	} while (ath5k_hw_is_intr_pending(ah) && --counter > 0);

	if (unlikely(!counter))
		ATH5K_WARN(sc, "too many interrupts, giving up for now\n");

	ath5k_hw_calibration_poll(ah);

	return IRQ_HANDLED;
}

static void
ath5k_tasklet_reset(unsigned long data)
{
	struct ath5k_softc *sc = (void *)data;

	ath5k_reset_wake(sc);
}

/*
 * Periodically recalibrate the PHY to account
 * for temperature/environment changes.
 */
#if 0
static void
ath5k_tasklet_calibrate(unsigned long data)
{
	struct ath5k_softc *sc = (void *)data;
	struct ath5k_hw *ah = sc->ah;

	/* Only full calibration for now */
	if (ah->ah_swi_mask != AR5K_SWI_FULL_CALIBRATION)
		return;
	/* Stop queues so that calibration
	 * doesn't interfere with tx */
	netif_stop_queue(sc->netdev);

	ATH5K_DBG(sc, ATH5K_DEBUG_CALIBRATE, "channel %u/%x\n",
		ath5k_frequency_to_channel(sc->curchan->center_freq),
		sc->curchan->hw_value);

	if (ath5k_hw_gainf_calibrate(ah) == AR5K_RFGAIN_NEED_CHANGE) {
		/*
		 * Rfgain is out of bounds, reset the chip
		 * to load new gain values.
		 */
		ATH5K_DBG(sc, ATH5K_DEBUG_RESET, "calibration, resetting\n");
		ath5k_reset_wake(sc);
	}
	if (ath5k_hw_phy_calibrate(ah, sc->curchan))
		ATH5K_ERR(sc, "calibration of channel %u failed\n",
			ath5k_frequency_to_channel(
				sc->curchan->center_freq));

	ah->ah_swi_mask = 0;

	/* Wake queues */
	netif_wake_queue(sc->netdev);
}
#endif


/**********************\
* Interface functions *
\**********************/
static u8 P802_1H_OUI[P80211_OUI_LEN] = { 0x00, 0x00, 0xf8 };
static u8 RFC1042_OUI[P80211_OUI_LEN] = { 0x00, 0x00, 0x00 };

static int ieee80211_copy_snap(u8 * data, unsigned short h_proto)
{
	struct ieee80211_snap_hdr *snap;
	u8 *oui;

	snap = (struct ieee80211_snap_hdr *)data;
	snap->dsap = 0xaa;
	snap->ssap = 0xaa;
	snap->ctrl = 0x03;

	if (h_proto == htons(ETH_P_AARP) || h_proto == htons(ETH_P_IPX))
		oui = P802_1H_OUI;
	else
		oui = RFC1042_OUI;
	snap->oui[0] = oui[0];
	snap->oui[1] = oui[1];
	snap->oui[2] = oui[2];

	memcpy(data + SNAP_SIZE, &h_proto, sizeof(u16));

	return SNAP_SIZE + sizeof(u16);
}

/*
 * Contexto? UC?
 *
 * Puede ocurrir:
 * - Nos expulse una hardIRQ -> spin_lock_irq (peor caso)
 * - Nos expulse una softIRQ -> spin_lock_bh
 * - Nos expulsen del uP     -> spin_lock
 * - En otra CPU se este ejecutando algo -> spin_lock
 */
static int
ath5k_tx(struct sk_buff *skb, struct net_device *netdev)
{
	struct ath5k_softc *sc = netdev_priv(netdev);
	struct ath5k_buf *bf;
//	unsigned long flags;
	int hdrlen;
	int padsize;

	struct sk_buff *skb_new;
	unsigned short ethertype;
	struct ieee80211_hdr header;
	int frame_control;
	unsigned char dst[ETH_ALEN];
	unsigned char src[ETH_ALEN];

	/* Save source and destination addresses and ethertype */
	skb_copy_from_linear_data(skb, dst, ETH_ALEN);
	skb_copy_from_linear_data_offset(skb, ETH_ALEN, src, ETH_ALEN);
	ethertype = ((struct ethhdr *)skb->data)->h_proto;

	/* In ADHOC mode, no From/To DS, Addr1 = DA, Addr2 = SA, Addr3 = BSSID */
	memcpy(header.addr1, dst, ETH_ALEN);
	memcpy(header.addr2, src, ETH_ALEN);
	memcpy(header.addr3, sc->ah->ah_bssid, ETH_ALEN);
	
	if (use_beacon_frames){
		frame_control = IEEE80211_STYPE_BEACON | IEEE80211_STYPE_DATA;
	} else{
		frame_control = IEEE80211_FTYPE_DATA | IEEE80211_STYPE_DATA;
	}

	header.frame_control = cpu_to_le16(frame_control);

	//DANI XXX:
	header.duration_id = 0;

	hdrlen = 24;

	/* Advance the SKB to the start of the payload */
	skb_pull(skb, sizeof(struct ethhdr));

	/* Allocate new skb */
	skb_new = dev_alloc_skb(hdrlen + SNAP_SIZE + sizeof(u16) + skb->len);
	if (!skb_new)
	{
		ATH5K_ERR(sc, "can't alloc skbuff of size %u\n", 
				hdrlen + SNAP_SIZE + sizeof(u16) + skb->len);
		return -ENOMEM;
	}
	memset(skb_new->data, 0, skb_new->len);

	/* Copy header */
	memcpy(skb_put(skb_new, hdrlen), &header, hdrlen);

	/* Copy SNAP and ethertype */
	ieee80211_copy_snap(skb_put(skb_new, SNAP_SIZE + sizeof(u16)), ethertype);

	/* Copy payload */
	skb_copy_from_linear_data(skb, skb_put(skb_new, skb->len), skb->len);
	
	/* Free skb */
	dev_kfree_skb_any(skb);
	skb = skb_new;

	ath5k_debug_dump_skb(sc, skb, "TX  ", 1);
	
	/*
	 * the hardware expects the header padded to 4 byte boundaries
	 * if this is not the case we add the padding after the header
	 */
	hdrlen = ath5k_get_hdrlen_from_skb(skb);
	padsize = ath5k_pad_size(hdrlen);
	if (padsize) {
		if (skb_headroom(skb) < padsize) {
			ATH5K_ERR(sc, "tx hdrlen not %%4: %d not enough"
				  " headroom to pad %d\n", hdrlen, padsize);
			goto drop_packet;
		}
		skb_push(skb, padsize);
		memmove(skb->data, skb->data+padsize, hdrlen);
	}

	spin_lock_irq(&sc->txbuflock);
	if (list_empty(&sc->txbuf)) {
		ATH5K_ERR(sc, "no further txbuf available, dropping packet\n");
		spin_unlock_irq(&sc->txbuflock);
		netif_stop_queue(sc->netdev);
		goto drop_packet;
	}
	bf = list_first_entry(&sc->txbuf, struct ath5k_buf, list);
	list_del(&bf->list);
	sc->txbuf_len--;
	if (list_empty(&sc->txbuf))
		netif_stop_queue(sc->netdev);
	spin_unlock_irq(&sc->txbuflock);

	bf->skb = skb;

	if (ath5k_txbuf_setup(sc, bf)) {
		bf->skb = NULL;
		spin_lock_irq(&sc->txbuflock);
		list_add_tail(&bf->list, &sc->txbuf);
		sc->txbuf_len++;
		spin_unlock_irq(&sc->txbuflock);
		goto drop_packet;
	}
	return NETDEV_TX_OK;

drop_packet:
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

/*
 * Reset the hardware.  If chan is not NULL, then also pause rx/tx
 * and change to the given channel.
 */
static int
ath5k_reset(struct ath5k_softc *sc, struct ieee80211_channel *chan)
{
	struct ath5k_hw *ah = sc->ah;
	int ret;

	ATH5K_DBG(sc, ATH5K_DEBUG_RESET, "resetting\n");

	if (chan) {
		ath5k_hw_set_imr(ah, 0);
		ath5k_txq_cleanup(sc);
		ath5k_rx_stop(sc);

		sc->curchan = chan;
		sc->curband = &sc->sbands[chan->band];
	}
	ret = ath5k_hw_reset(ah, sc->curchan, true);
	if (ret) {
		ATH5K_ERR(sc, "can't reset hardware (%d)\n", ret);
		goto err;
	}

	ret = ath5k_rx_start(sc);
	if (ret) {
		ATH5K_ERR(sc, "can't start recv logic\n");
		goto err;
	}

	/* Enable interrupts */
	ath5k_hw_set_imr(ah, sc->imask);

	return 0;
err:
	return ret;
}

static int
ath5k_reset_wake(struct ath5k_softc *sc)
{
	int ret;

	ret = ath5k_reset(sc, sc->curchan);
	if (!ret)
		netif_wake_queue(sc->netdev);

	return ret;
}

static int ath5k_start(struct ath5k_softc *sc)
{
	return ath5k_init(sc);
}

static void ath5k_stop(struct ath5k_softc *sc)
{
	ath5k_stop_hw(sc);
}

static void ath5k_config_filter(struct ath5k_softc *sc, bool broadcast, bool control, bool promisc)
{
	struct ath5k_hw *ah = sc->ah;
	unsigned int rfilt;

	rfilt = 0;

	/* Always enable Unicast */
	rfilt |= AR5K_RX_FILTER_UCAST;

	if (broadcast)
		rfilt |= AR5K_RX_FILTER_BCAST;

	/* Permitir tramas de control? */
	if (control)
		rfilt |= AR5K_RX_FILTER_CONTROL;

	/* Modo promiscuo? */
	if (promisc)
	{
		rfilt |= AR5K_RX_FILTER_PROM;
		__set_bit (ATH_STAT_PROMISC, sc->status);
	}
	else
		__clear_bit (ATH_STAT_PROMISC, sc->status);

	/* Set filters */
	ath5k_hw_set_rx_filter(ah, rfilt);

	/* Set multicast bits */
	ath5k_hw_set_mcast_filter(ah, 0, 0);

	/* Set the cached hw filter flags, this will alter actually be set in HW */
	sc->filter_flags = rfilt;
}

static struct ieee80211_channel * ath5k_get_channel(struct ath5k_softc *sc, unsigned int freq)
{
	struct ieee80211_channel *aux;
	int i, n_channels;

	aux = NULL;
	if (freq < 3000)
	{
		aux = sc->sbands[IEEE80211_BAND_2GHZ].channels;
		n_channels = sc->sbands[IEEE80211_BAND_2GHZ].n_channels;
	}
	else if (sc->sbands[IEEE80211_BAND_5GHZ].channels != NULL)
	{
		aux = sc->sbands[IEEE80211_BAND_5GHZ].channels;
		n_channels = sc->sbands[IEEE80211_BAND_5GHZ].n_channels;
	}

	if (aux != NULL)
	{
		for (i = 0; i < n_channels; i++, aux++)
			if (aux->center_freq == freq)			
				break;
	}

	return aux;
}

static int ath5k_get_rate_idx(struct ath5k_softc *sc, enum ieee80211_band band, enum rates rate)
{
	struct ieee80211_rate *r;
	int i;
	bool valid = false;

	r = NULL;

	r = sc->sbands[band].bitrates;

	if (r != NULL)
		for (i = 0; i < sc->curband->n_bitrates; i++, r++)
		{
			if (r->bitrate == rate)
			{			
				valid = true;
				break;
			}
		}
	if (valid)
		return i;
	else
		return -1;
}

static int ath5k_config_tx_control (struct ath5k_softc *sc, 
		unsigned char count, bool wait_for_ack, bool use_short_preamble)
{
	struct ieee80211_rate *rate;

	/* Clear flags */
	sc->tx_info.wait_for_ack = false;
	sc->tx_info.use_short_preamble = false;

	/* Set tx count if ack not received */
	sc->tx_info.count = count;

	/* Wait for ack flag */
	if (wait_for_ack)
		sc->tx_info.wait_for_ack = true;

	/* Set rate code flags (short preamble) */
	rate = &sc->curband->bitrates[sc->tx_info.rate_idx];
	if (use_short_preamble && (rate->flags & IEEE80211_RATE_SHORT_PREAMBLE))
		sc->tx_info.use_short_preamble = true;
	
	return 0;
}

static int ath5k_config_disable_ack (struct ath5k_softc *sc, bool disable)
{
	struct ath5k_hw *ah;
   
	if (sc == NULL)
		return 1;	
	else
		ah = sc->ah;

	if (disable)
		AR5K_REG_ENABLE_BITS(ah, AR5K_DIAG_SW, AR5K_DIAG_SW_DIS_ACK);
	else
		AR5K_REG_DISABLE_BITS(ah, AR5K_DIAG_SW, AR5K_DIAG_SW_DIS_ACK);

	return 0;
}

/*
 * Buscar por sc->channels y tal
 */
static int ath5k_config (struct ath5k_softc *sc, 
		unsigned short channel, 
		enum rates rate, 
		unsigned char tx_power_dbm,
		enum ath5k_ant_mode antenna_mode)
{
	struct ath5k_hw *ah = sc->ah;
	struct ieee80211_channel *target_channel;
	int rate_idx;
	int ret = 0;

	mutex_lock(&sc->lock);

	/* Encontrar el canal */
	target_channel = ath5k_get_channel(sc, channel);
	if (target_channel == NULL)
	{
		ATH5K_ERR(sc, "Invalid channel\n");
		ret = -1;
		goto unlock;
	}

	/* Encontrar el bitrate */
	rate_idx = ath5k_get_rate_idx(sc, target_channel->band, rate);
	if (rate_idx < 0 || rate_idx > AR5K_MAX_RATES)
	{
		ATH5K_ERR(sc, "Invalid rate\n");
		ret = -1;
		goto unlock;
	}

	/* Configurar el rate */
	sc->tx_info.band = sc->curband->band;
	sc->tx_info.rate_idx = rate_idx;
	sc->tx_info.count = 1;
	sc->tx_info.wait_for_ack = false;
	sc->tx_info.use_short_preamble = false;

	if (sc->power_level != tx_power_dbm)
   	{


		/* Half dB steps */
		//DANI
		if (ath5k_hw_set_txpower_limit(ah, (tx_power_dbm * 2)) == 0){
			sc->power_level = tx_power_dbm;
		}else{
			ATH5K_ERR(sc, "Invalid power\n");
			ret = -1;
			goto unlock;
		}
	}

	ath5k_hw_set_antenna_mode(ah, antenna_mode);

	/* Hacer un reset para aplicar los cambios */
	ath5k_reset(sc, target_channel);

	ATH5K_INFO(sc, "Freq %d Mhz, Rate (%d * 100) kps, Power %d dBm, Ant. mode %d\n", 
			channel, rate, tx_power_dbm, antenna_mode);

unlock:
	mutex_unlock(&sc->lock);

	return ret;
}

static int ath5k_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd)
{
	struct ath5k_softc *sc;
	union ath5k_ioctl_info info;
	int ret;

   /*///////////////////////////////////////////////////////////////////////////
   struct timespec ts;
   *////////////////////////////////////////////////////////////////////////////
	/* Recuperar la estructura principal */
	sc = netdev_priv(netdev);

	ret = 0;
	switch(cmd)
	{
		case SIO_SEND_PACKET:
			ATH5K_ERR(sc, "SIO_SEND_PACKET not implemented yet.\n");
			ret = -1;
			break;

		case SIO_RECV_PACKET:
			ATH5K_ERR(sc, "SIO_RECV_PACKET not implemented yet.\n");
			ret = -1;
			break;

		case SIO_SET_CONFIG:
			ret = copy_from_user(&info.config_info, 
					(struct ath5k_config_info *)ifr->ifr_data, 
					sizeof(struct ath5k_config_info));
			if (ret)
				return ret;

			ret = ath5k_config(sc, info.config_info.frequency, info.config_info.rate, info.config_info.tx_power_dbm,
					info.config_info.antenna_mode);

			if (ret)
				return ret;

			break;

		case SIO_SET_DEBUG:
			ret = copy_from_user(&info.debug_level,
					(unsigned int *)ifr->ifr_data, 
					sizeof(int));
			if (ret)
				return ret;
   /*///////////////////////////////////////////////////////////////////////////
   getnstimeofday(&ts);
   printk(KERN_INFO "%lld\n", timespec_to_ns(&ts));
   *////////////////////////////////////////////////////////////////////////////

			sc->debug_level = info.debug_level;

			break;

		case SIO_SET_RXFILTER:
			ret = copy_from_user(&info.rxfilter_info,
					(struct ath5k_rxfilter_info *)ifr->ifr_data,
					sizeof(struct ath5k_rxfilter_info));

			if (ret)
				return ret;

			ath5k_config_filter(sc, info.rxfilter_info.broadcast,
							        info.rxfilter_info.control,
									info.rxfilter_info.promisc);

			break;

		case SIO_SET_TXCONTROL:
			ret = copy_from_user(&info.txcontrol_info,
					(struct ath5k_txcontrol_info *)ifr->ifr_data,
					sizeof(struct ath5k_txcontrol_info));

			if (ret)
				return ret;

			ret = ath5k_config_tx_control(sc, info.txcontrol_info.count,
											  info.txcontrol_info.wait_for_ack,
											  info.txcontrol_info.use_short_preamble);

			break;


		case SIO_SET_DISABLEACK:
			ret = copy_from_user(&info.disable_ack,
					(bool *)ifr->ifr_data,
					sizeof(bool));

			if (ret)
				return ret;

			ret = ath5k_config_disable_ack(sc, info.disable_ack);

			break;

		case SIO_SET_BSSID:
			ATH5K_ERR(sc, "SIO_SET_BSSID not implemented yet\n");
			ret = -1;
			break;

		case SIO_SET_BSSIDFILTER:
			ATH5K_ERR(sc, "SIO_SET_BSSIDFILTER not implemented yet\n");
			ret = -1;
			break;
		case SIO_SET_USE_BEACON_FRAMES:
			ret = copy_from_user(&use_beacon_frames, (int *) ifr->ifr_data, sizeof(int));

			if (ret)
				return ret;

		break;

			/*		case SIO_SET_POWER:
			int val_dbm = 0;
			ret = copy_from_user(&val_dbm,
								(int *)ifr->ifr_data,
								sizeof(int));
			sc->power_level = val_dbm * 2;
			break;*/
	}		
	return ret;
}







/* RUBEN */

/**
 *  'netdev' will point to the ath5k_raw interface that must be used by RT-WMP
 */
void get_ath5k_slave_dev(struct net_device** netdev)
{
   *netdev = ath5k_slave_dev;
}
EXPORT_SYMBOL(get_ath5k_slave_dev);

/**
 *  Attempts to dequeue a RT-WMP packet from the reception queue. If the queue
 *  is empty, the task will be put to sleep until there is a packet to be
 *  dequeued or until it is interrupted.
 *  On a successful reception, the function returns the packet length.
 *  If no packet has been dequeued, the return value is 0.
 */
int ath5k_rx(struct sk_buff **skb)
{

   *skb = dequeue_rx_queue(&rx_rtwmp_queue, 0);

   if (*skb == NULL){
      return 0;
   }
   else{
      return (*skb)->len;
   }
}
EXPORT_SYMBOL(ath5k_rx);

/**
 *  Attempts to dequeue a RT-WMP packet from the reception queue. If the queue
 *  is empty, the task will be put to sleep until there is a packet to be
 *  dequeued or until at least 'timeout' milliseconds have passed.
 *  On a successful reception, the function returns the packet length.
 *  If no packet has been dequeued, the return value is 0.
 */
int ath5k_rx_timeout(struct sk_buff **skb, int timeout)
{

   *skb = dequeue_rx_queue(&rx_rtwmp_queue, timeout);

   if (*skb == NULL){
      return 0;
   }
   else{
      return (*skb)->len;
   }
}
EXPORT_SYMBOL(ath5k_rx_timeout);

/**
 *  Unlocks the reception semaphore
 */
void unlock_rx_sem(void)
{
   up(&rx_rtwmp_queue.rx_sem);
}
EXPORT_SYMBOL(unlock_rx_sem);

/**
 *  Function used to configure an ath5k_raw interface from within another kernel module
 *  It works like ath5k_ioctl but is designed to be called from within the kernel context
 */
int conf_ath5k (struct net_device *netdev, union ath5k_ioctl_info *info, int cmd)
{
   struct ath5k_softc *sc;
   int ret;

   sc = netdev_priv(netdev);

   ret = 0;
   switch(cmd)
   {

      case SIO_SET_CONFIG:

         ret = ath5k_config(sc, info->config_info.frequency,
                                 info->config_info.rate,
                                 info->config_info.tx_power_dbm,
                                 info->config_info.antenna_mode);

         break;

      case SIO_SET_DEBUG:

         sc->debug_level = info->debug_level;

         break;

      case SIO_SET_RXFILTER:

         ath5k_config_filter(sc, info->rxfilter_info.broadcast,
                             info->rxfilter_info.control,
                           info->rxfilter_info.promisc);

         break;

      case SIO_SET_TXCONTROL:

         ret = ath5k_config_tx_control(sc, info->txcontrol_info.count,
                                   info->txcontrol_info.wait_for_ack,
                                   info->txcontrol_info.use_short_preamble);

         break;


      case SIO_SET_DISABLEACK:

         ret = ath5k_config_disable_ack(sc, info->disable_ack);

         break;
   }
   return ret;
}
EXPORT_SYMBOL(conf_ath5k);
