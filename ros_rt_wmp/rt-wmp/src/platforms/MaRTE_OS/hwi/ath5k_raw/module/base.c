/*------------------------------------------------------------------------------
 *-------------------------        ATH5K Driver          -----------------------
 *------------------------------------------------------------------------------
 *                                                           V1.0  08/02/2010
 *
 *
 *  Feb 2010 - Samuel Cabrero <samuelcabrero@gmail.com>
 *		Initial release
 *
 *  ----------------------------------------------------------------------------
 *  Copyright (C) 2000-2010, Universidad de Zaragoza, SPAIN
 *
 *  Autors:
 *		Samuel Cabrero        <samuelcabrero@gmail.com>
 *		Danilo Tardioli	      <dantard@unizar.es>
 *		Jose Luis Villarroel  <jlvilla@unizar.es>
 *
 *  This is a simplified version of the original ath5k driver. It should work 
 *  with all Atheros 5xxx WLAN cards. The 802.11 layer have been removed so it
 *  just send and receive frames over the air, as if it were an Ethernet bus
 *  interface.
 *
 *  Please read ath5k_interface.h for instructions.
 *
 *  This program is distributed under the terms of GPL version 2 and in the 
 *  hope that it will be useful, but WITHOUT ANY WARRANTY; without even the 
 *  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
 *  See the GNU General Public License for more details.
 *
 *----------------------------------------------------------------------------*/

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

#include <drivers/osdep.h>
#include "ath5k.h"
#include "base.h"
#include "reg.h"
#include "debug.h"
#include "ath5k_interface.h"

#include "ath5k_linux_layer.h"
#include "mac80211.h"

static int ath5k_calinterval = 10;
static int modparam_all_channels = true;

//DANI
static sem_t ws;

/******************\
* Internal defines *
\******************/

/* Known PCI ids */
static const struct ath5k_device_ids ath5k_pci_id_table[] = {
	{ .vendor = PCI_VENDOR_ID_ATHEROS, .device = 0x0207, .driver_data = AR5K_AR5210,
		.name = "5210" }, /* 5210 early */
	{ .vendor = PCI_VENDOR_ID_ATHEROS, .device = 0x0007, .driver_data = AR5K_AR5210,
		.name = "5210" }, /* 5210 */
	{ .vendor = PCI_VENDOR_ID_ATHEROS, .device = 0x0011, .driver_data = AR5K_AR5211,
		.name = "5311" }, /* 5311 - this is on AHB bus !*/
	{ .vendor = PCI_VENDOR_ID_ATHEROS, .device = 0x0012, .driver_data = AR5K_AR5211,
	    .name = "5211" }, /* 5211 */
	{ .vendor = PCI_VENDOR_ID_ATHEROS, .device = 0x0013, .driver_data = AR5K_AR5212,
	   	.name = "5212" }, /* 5212 */
	{ .vendor = PCI_VENDOR_ID_ATHEROS, .device = 0x0013, .driver_data = AR5K_AR5212,
	    .name = "3com 5212" }, /* 3com 5212 */
	{ .vendor = PCI_VENDOR_ID_ATHEROS, .device = 0x0013, .driver_data = AR5K_AR5212,
		.name = "3com 5212" }, /* 3com 3CRDAG675 5212 */
	{ .vendor = PCI_VENDOR_ID_ATHEROS, .device = 0x1014, .driver_data = AR5K_AR5212,
	    .name = "IBM 5212" }, /* IBM minipci 5212 */
	{ .vendor = PCI_VENDOR_ID_ATHEROS, .device = 0x0014, .driver_data = AR5K_AR5212,
	   	.name = "5212 compatible" }, /* 5212 combatible */
	{ .vendor = PCI_VENDOR_ID_ATHEROS, .device = 0x0015, .driver_data = AR5K_AR5212,
	   	.name = "5212 compatible" }, /* 5212 combatible */
	{ .vendor = PCI_VENDOR_ID_ATHEROS, .device = 0x0016, .driver_data = AR5K_AR5212,
	   	.name = "5212 compatible" }, /* 5212 combatible */
	{ .vendor = PCI_VENDOR_ID_ATHEROS, .device = 0x0017, .driver_data = AR5K_AR5212,
	   	.name = "5212 compatible" }, /* 5212 combatible */
	{ .vendor = PCI_VENDOR_ID_ATHEROS, .device = 0x0018, .driver_data = AR5K_AR5212,
	   	.name = "5212 compatible" }, /* 5212 combatible */
	{ .vendor = PCI_VENDOR_ID_ATHEROS, .device = 0x0019, .driver_data = AR5K_AR5212,
	   	.name = "5212 compatible" }, /* 5212 combatible */
	{ .vendor = PCI_VENDOR_ID_ATHEROS, .device = 0x001a, .driver_data = AR5K_AR5212,
		.name = "2413" }, /* 2413 Griffin-lite */
	{ .vendor = PCI_VENDOR_ID_ATHEROS, .device = 0x001b, .driver_data = AR5K_AR5212,
	   	.name = "5413" }, /* 5413 Eagle */
	{ .vendor = PCI_VENDOR_ID_ATHEROS, .device = 0x001c, .driver_data = AR5K_AR5212,
	   	.name = "PCI-E card" }, /* PCI-E cards */
	{ .vendor = PCI_VENDOR_ID_ATHEROS, .device = 0x001d, .driver_data = AR5K_AR5212,
	   	.name = "2417" }, /* 2417 Nala */
	{ 0 }
};

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
#define ATH5K_NR_RATES   12

/*
 * Prototypes - Internal functions
 */
/* Attach detach */
static int ath5k_attach(struct ath5k_softc *sc);

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
				struct pci_device *pdev);

/* Buffers setup */
static int 	ath5k_rxbuf_setup(struct ath5k_softc *sc,
				struct ath5k_buf *bf);
static int 	ath5k_txbuf_setup(struct ath5k_softc *sc,
				struct ath5k_buf *bf);
static inline void ath5k_skb_reset(struct sk_buff *skb)
{
	if (!skb)
	{
		ATH5K_ERR(sc, "Buffer not allocated.\n");
	}
	else
	{
		skb->len  = 0;
		skb->data = skb->head;
		skb->tail = skb->head;
		memset(skb->head, 0, skb->end - skb->head);
	}
}

/* Queues setup */
static struct ath5k_txq *ath5k_txq_setup(struct ath5k_softc *sc, int qtype, int subtype);
static void				 ath5k_txq_drainq(struct ath5k_softc *sc, struct ath5k_txq *txq);
static void				 ath5k_txq_cleanup(struct ath5k_softc *sc);

/* Rx handling */
static int 	ath5k_rx_start(struct ath5k_softc *sc);
static void ath5k_rx_stop(struct ath5k_softc *sc);
static void ath5k_rx_done(struct ath5k_softc *sc);

/* Tx handling */
static void ath5k_tx_done(struct ath5k_softc *sc, struct ath5k_txq *txq);

/* Interrupt handling */
static int ath5k_init(struct ath5k_softc *sc);
static int ath5k_intr(void *dev_id, intr_t irq);
static void ath5k_calibrate(struct ath5k_softc *sc);

/* Configuration and auxiliary */
static int ath5k_reset(struct ath5k_softc *sc, struct ieee80211_channel *chan);
void ath5k_config_filter (struct ath5k_softc *sc, bool broadcast, bool control, bool promisc);
int ath5k_config_disable_ack (struct ath5k_softc *sc, bool disable);
int ath5k_config(struct ath5k_softc *sc, unsigned short freq, enum rates rate, unsigned char tx_power_dbm,
			enum ath5k_ant_mode antenna_mode);
static struct ieee80211_channel * ath5k_get_channel(struct ath5k_softc *sc, unsigned int freq);
static int ath5k_get_rate_idx(struct ath5k_softc *sc, enum ieee80211_band band, enum rates rate);
static int ath5k_hw_rix_to_bitrate(struct ath5k_softc *sc, int hw_rix);

/********************\
* PCI Initialization *
\********************/

static const char *
ath5k_chip_name(enum ath5k_srev_type type, unsigned int val)
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

	struct ath5k_softc *
ath5k_find(const struct ath5k_softc *prev_sc,
		unsigned short freq, enum rates rate, unsigned char tx_power_dbm,
		enum ath5k_ant_mode antenna_mode)
{
	struct ath5k_softc *sc;
	struct pci_device *pdev, *prev_dev;
	void *mem;
	int i, ret, rate_idx;
	unsigned char csz;

	/* Allocate main driver struct */
	sc = (struct ath5k_softc *)malloc(sizeof(struct ath5k_softc));
	if (sc == NULL)
	{
		ATH5K_ERR(NULL, "Can't allocate main struct.\n");
		return NULL;
	}


	/* Set driver debug level */
	ath5k_set_debug_level(sc);

	/* Allocate pci_device structure */
	pdev = malloc(sizeof(struct pci_device));
	if (pdev == NULL)
	{
		ATH5K_ERR(sc, "Can't allocate pci_device struct.\n");
		free(sc);
		return NULL;
	}
	else
		sc->pdev = pdev;

	/* Find next device in the pci bus */
	if (prev_sc != NULL)
		prev_dev = ((struct ath5k_softc *)prev_sc)->pdev;
	else
		prev_dev = NULL;

	for (i = 0; i < ARRAY_SIZE(ath5k_pci_id_table); i++)
	{
		ret = pci_find_device(ath5k_pci_id_table[i].vendor, ath5k_pci_id_table[i].device, prev_dev, pdev);
		if (ret == 0)
		{
			pdev->name = ath5k_pci_id_table[i].name;
			break;
		}
	}
	if (ret)
	{
		ATH5K_ERR(sc, "Device not found.\n");
		goto err;
	}

	/*
	 * Cache line size is used to size and align various
	 * structures used to communicate with the hardware.
	 */
	pci_read_config_byte(pdev, PCI_CACHE_LINE_SIZE, &csz);
	if (csz == 0)
		ATH5K_WARN(sc, "Cache line size 0.\n");

	/* Set the device as bus master and adjust latency */
	adjust_pci_device(pdev);
	pci_write_config_byte(pdev, PCI_LATENCY_TIMER, 0xa8);

	/* Get the device base address and IRQ */
	mem = (void *)pci_bar_start (pdev, PCI_BASE_ADDRESS_0);
	ATH5K_DBG(sc, ATH5K_DEBUG_ANY, "Base memory: %X\n", pdev->membase);
	ATH5K_DBG(sc, ATH5K_DEBUG_ANY, "IO Addr: %X\n", pdev->ioaddr);
	ATH5K_DBG(sc, ATH5K_DEBUG_ANY, "IRQ: %u\n", pdev->irq);

	/*
	 * Mark the device as detached to avoid processing
	 * interrupts until setup is complete.
	 */
	__set_bit(ATH_STAT_INVALID, sc->status);

	sc->iobase = mem;
	sc->cachelsz = csz << 2; /* convert to bytes */
	/* Setup interrupt handler */
	ret = posix_intr_associate(pdev->irq, ath5k_intr, sc, sizeof(struct ath5k_softc)) ||
			posix_intr_unlock(pdev->irq);
	if (ret)
	{
		ATH5K_ERR(sc, "Can't request IRQ.\n");
		goto err;
	}

	/*If we passed the test malloc a ath5k_hw struct*/
	sc->ah = (struct ath5k_hw *)malloc(sizeof(struct ath5k_hw));
	if (!sc->ah) {
		ret = -ENOMEM;
		ATH5K_ERR(sc, "out of memory\n");
		goto err;
	}

	sc->ah->ah_sc = sc;
	sc->ah->ah_iobase = sc->iobase;

	/* Initialize device */
	ret = ath5k_hw_attach(sc);
	if (ret)
   	{
		ATH5K_ERR(sc, "Error attaching hardware.\n");
		goto err_irq;
	}

	/* Finish private driver data initialization */
	ret = ath5k_attach(sc);
	if (ret)
	{
		ATH5K_ERR(sc, "Error initializing.\n");
		goto err_ah;
	}

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
	sc->curchan = ath5k_get_channel(sc, freq);
	if (sc->curchan == NULL)
	{
		ATH5K_ERR(sc, "Can't set channel %d\n", freq);
		ret = -1;
		goto err_detach;
	}
	sc->curband = &sc->sbands[sc->curchan->band];

	/* Configurar tx_info */
	sc->tx_info.band = sc->curband->band;
	rate_idx = ath5k_get_rate_idx(sc, sc->curband->band, rate);
	if (rate_idx < 0 || rate_idx > AR5K_MAX_RATES)
	{
		ATH5K_ERR(sc, "Can't set rate %d\n", rate);
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
	ret = ath5k_config_disable_ack(sc, false);
	if (ret)
		goto err_detach;

	/* Enable interrupts and rx framework */
	ret = ath5k_init(sc);
	if (ret)
		goto err_detach;

	/* Configurar potencia en pasos de medio dB */
	sc->power_level = tx_power_dbm;
	ret = ath5k_hw_set_txpower_limit(sc->ah, (sc->power_level * 2));
	if (ret)
		goto err_detach;

	/* Configurar modo de antena */
	ath5k_hw_set_antenna_mode(sc->ah, antenna_mode);

	/* Resetear despues de configurar la potencia y la antena?
	 * En principio, de acuerdo con el codigo original no es necesario. */
	//ath5k_reset(sc, sc->curchan);

	/* Print initial config */
	ATH5K_INFO(sc, "Freq %d Mhz, Rate (%d * 100) kps, Power %d dBm, Ant. mode %d\n",
			sc->curchan->center_freq, rate, sc->power_level, antenna_mode);

	return sc;
err_detach:
	/* XXX: Free descriptors and buffers?. Maybe pointless in embedded. */
err_ah:
	ath5k_hw_detach(sc->ah);
err_irq:
	posix_intr_lock(pdev->irq);
	posix_intr_disassociate (pdev->irq, ath5k_intr);
err:
	free(pdev);
	free(sc);

	return NULL;
}



/***********************\
* Driver Initialization *
\***********************/

static int
ath5k_attach(struct ath5k_softc *sc)
{
	struct pci_device *pdev = sc->pdev;
	struct ath5k_hw *ah = sc->ah;
	int ret;

	//dani
	sem_init(&ws,0,0);

	ATH5K_DBG(sc, ATH5K_DEBUG_ANY, "devid 0x%x\n", pdev->dev_id);

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

	/* Inicializar el anillo de recepcion */
	ath5k_ring_init (&sc->rxring);

	sem_init(&sc->tx_sem, 0, 1);

	/* All MAC address bits matter for ACKs */
	memset(sc->bssidmask, 0xff, ETH_ALEN);
	ath5k_hw_set_bssid_mask(sc->ah, sc->bssidmask);

	return 0;
err_desc:
	/* XXX: free descriptors? Maybe pointless in embedded */
err:
	return ret;
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
struct sk_buff *ath5k_rx_skb_alloc(struct ath5k_softc *sc, struct sk_buff *skb)
{
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

	return skb;
}

static int
ath5k_rxbuf_setup(struct ath5k_softc *sc, struct ath5k_buf *bf)
{
	struct ath5k_hw *ah = sc->ah;
	struct sk_buff *skb = bf->skb;
	struct ath5k_desc *ds;

	/* Reiniciar el skb para recibir una nueva trama */
	ath5k_skb_reset(skb);

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
	ds->ds_link = (unsigned int)bf->desc;	/* link to self */
	ds->ds_data = (unsigned int)skb->data;
	ah->ah_setup_rx_desc(ah, ds,
		skb_tailroom(skb),	/* buffer size */
		0);

	if (sc->rxlink != NULL)
		*sc->rxlink = (unsigned int)bf->desc;
	sc->rxlink = &ds->ds_link;
	return 0;
}

/*
 * Esta funcion rellena el buffer que se le pasa como parametro, lo inserta al
 * final de sc->txq y llama al hw.
 *
 * Contexto: UC
 *
 * Puede ocurrir:
 * - Que la rutina de interrupcion nos expulse, luego hay que bloquear las
 *   interrupciones.
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
	{
		ATH5K_ERR(sc, "Error setting up tx descriptor\n");
		return ret;
	}

	ds->ds_link = 0;
	ds->ds_data = (unsigned int)skb->data;

	cli();
	list_add_tail(&bf->list, &txq->q);
	if (txq->link == NULL) /* is this first packet? */
		ath5k_hw_set_txdp(ah, txq->qnum, (unsigned int)bf->desc);
	else /* no, so only link it */
		*txq->link = (unsigned int)bf->desc;

	txq->link = &ds->ds_link;
	ath5k_hw_start_tx_dma(ah, txq->qnum);
	sti();

	return 0;
}

/*******************\
* Descriptors setup *
\*******************/

static int
ath5k_desc_alloc(struct ath5k_softc *sc, struct pci_device *pdev)
{
	struct ath5k_desc *ds;
	struct ath5k_buf *bf;
	unsigned int i;
	int ret;

	/* allocate descriptors */
	sc->desc_len = sizeof(struct ath5k_desc) * (ATH_TXBUF + ATH_RXBUF + 1);
	sc->desc = (struct ath5k_desc *)malloc(sc->desc_len);
	if (sc->desc == NULL) {
		ATH5K_ERR(sc, "can't allocate descriptors\n");
		ret = -ENOMEM;
		goto err;
	}
	ds = sc->desc;

	bf = calloc(ATH_TXBUF + ATH_RXBUF + 1, sizeof(struct ath5k_buf));
	if (bf == NULL) {
		ATH5K_ERR(sc, "can't allocate bufptr\n");
		ret = -ENOMEM;
		goto err_free;
	}
	sc->bufptr = bf;

	/* Tamaño del buffer de datos de los skb */
	sc->rxbufsize = roundup(IEEE80211_MAX_LEN, sc->cachelsz);
	ATH5K_DBG(sc, ATH5K_DEBUG_RESET, "cachelsz %u rxbufsize %u\n",
			sc->cachelsz, sc->rxbufsize);


	INIT_LIST_HEAD(&sc->rxbuf);
	for (i = 0; i < ATH_RXBUF; i++, bf++, ds++)
	{
		bf->skb  = ath5k_rx_skb_alloc(sc, bf->skb);
		bf->desc = ds;
		list_add_tail(&bf->list, &sc->rxbuf);
	}

	INIT_LIST_HEAD(&sc->txbuf);
	sc->txbuf_len = ATH_TXBUF;
	for (i = 0; i < ATH_TXBUF; i++, bf++, ds++)
	{
		bf->skb  = ath5k_rx_skb_alloc(sc, bf->skb);
		bf->desc = ds;
		list_add_tail(&bf->list, &sc->txbuf);
	}

	return 0;
err_free:
	free(sc->desc);
err:
	sc->desc = NULL;
	return ret;
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
		txq->setup = true;
	}
	return &sc->txqs[qnum];
}

/* Contexto UC e IRQ en el caso de que la ISR haga un reset
 *
 * Puede ocurrir:
 * - Nos interrumpa una hardIRQ -> cli
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
	cli();
	list_for_each_entry_safe(bf, bf0, &txq->q, list) {

		ath5k_skb_reset(bf->skb);

		list_move_tail(&bf->list, &sc->txbuf);
		sc->txbuf_len++;
	}
	txq->link = NULL;
	sti();
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
	if (!test_bit(ATH_STAT_INVALID, sc->status))
	{
		/* don't touch the hardware if marked invalid */
		for (i = 0; i < ARRAY_SIZE(sc->txqs); i++)
			if (sc->txqs[i].setup) {
				ath5k_hw_stop_tx_dma(ah, sc->txqs[i].qnum);
				ATH5K_DBG(sc, ATH5K_DEBUG_RESET, "txq [%u] %x, link %p\n",
					sc->txqs[i].qnum,
					ath5k_hw_get_txdp(ah, sc->txqs[i].qnum),
					sc->txqs[i].link);
			}
	}

	for (i = 0; i < ARRAY_SIZE(sc->txqs); i++)
		if (sc->txqs[i].setup)
			ath5k_txq_drainq(sc, &sc->txqs[i]);
}

/*************\
* RX Handling *
\*************/

/*
 * Enable the receive h/w following a reset.
 *
 * Contexto: UC
 *
 * Bloquear interrupciones.
 */
static int
ath5k_rx_start(struct ath5k_softc *sc)
{
	struct ath5k_hw *ah = sc->ah;
	struct ath5k_buf *bf;
	int ret;

	cli();
	sc->rxlink = NULL;
	list_for_each_entry(bf, &sc->rxbuf, list) {
		ret = ath5k_rxbuf_setup(sc, bf);
		if (ret != 0)
		{
			sti();
			goto err;
		}
	}
	bf = list_first_entry(&sc->rxbuf, struct ath5k_buf, list);
	ath5k_hw_set_rxdp(ah, (unsigned int)bf->desc);
	sti();

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

	sc->rxlink = NULL;		/* just in case */
}

static void
ath5k_rx_done(struct ath5k_softc *sc)
{
	struct ieee80211_rx_status rxs = {};
	struct ath5k_rx_status rs = {};
	struct sk_buff *skb;
	struct ath5k_buf *bf;
	struct ath5k_desc *ds;
	int ret;
	int hdrlen;
	int padsize;
	unsigned int sem_value;


	if (list_empty(&sc->rxbuf)) {
		ATH5K_WARN(sc, "empty rx buf pool\n");
		return;
	}
	do {
		rxs.flag = 0;

		bf = list_first_entry(&sc->rxbuf, struct ath5k_buf, list);
		BUG_ON(bf->skb == NULL);
		skb = bf->skb;
		ds = bf->desc;

		/* bail if HW is still using self-linked descriptor */
		if (ath5k_hw_get_rxdp(sc->ah) == (unsigned int)bf->desc)
			break;

		ret = sc->ah->ah_proc_rx_desc(sc->ah, ds, &rs);
		if (ret == -EINPROGRESS)
			break;
		else if (ret)
		{
			ATH5K_ERR(sc, "error in processing rx descriptor\n");
			return;
		}

		if (rs.rs_more)
		{
			ATH5K_WARN(sc, "unsupported jumbo\n");
			goto next;
		}

		if (rs.rs_status)
		{
			if (rs.rs_status & AR5K_RXERR_PHY)
				goto next;
			if (rs.rs_status & AR5K_RXERR_CRC)
				goto next;
			if (rs.rs_status & AR5K_RXERR_DECRYPT)
				goto next;
			if (rs.rs_status & AR5K_RXERR_MIC) {
				goto accept;
			}
		}
accept:
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

		/* rssi can be more than 35 though, anything above that
		 * should be considered at 100% */
		if (rxs.qual > 100)
			rxs.qual = 100;

		rxs.antenna = rs.rs_antenna;

		ath5k_debug_dump_skb(sc, skb, "RX  ", 0);

		/* skb contains the frame in IEEE 802.11 format, as it was sent over air */
		unsigned char dst[ETH_ALEN];
		unsigned char src[ETH_ALEN];
		unsigned short ethertype, frame_control;
		struct ieee80211_hdr *hdr;
		unsigned char *payload;

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

		/* Last 4 bytes are CRC */
		enum rates rate = ath5k_hw_rix_to_bitrate(sc, rs.rs_rate);
		ath5k_ring_insert (&sc->rxring, skb->data, skb->len - 4, rxs.qual, rxs.noise, rate);
		sem_getvalue(&sc->rxring.sem, &sem_value);
		if (sem_value == 0)
			sem_post(&sc->rxring.sem);

next:
		list_move_tail(&bf->list, &sc->rxbuf);
	} while (ath5k_rxbuf_setup(sc, bf) == 0);
}




/*************\
* TX Handling *
\*************/

/*
 * Contexto IRQ
 */
static void
ath5k_tx_done(struct ath5k_softc *sc, struct ath5k_txq *txq)
{
	struct ath5k_tx_status ts = {};
	struct ath5k_buf *bf, *bf0;
	struct ath5k_desc *ds;
	struct sk_buff *skb;
	int ret;

	list_for_each_entry_safe(bf, bf0, &txq->q, list) {
		ds = bf->desc;

		ret = sc->ah->ah_proc_tx_desc(sc->ah, ds, &ts);
		if (ret == -EINPROGRESS)
			break;
		else if (ret) {
			ATH5K_ERR(sc, "error %d while processing queue %u\n",
				ret, txq->qnum);
			break;
		}

		skb = bf->skb;
		ath5k_skb_reset(skb);

		if (ts.ts_status)
			sc->ll_stats.dot11ACKFailureCount++;

		list_move_tail(&bf->list, &sc->txbuf);
		sc->txbuf_len++;
	}
	if (list_empty(&txq->q))
		txq->link = NULL;
}

/********************\
* Interrupt handling *
\********************/

static int
ath5k_init(struct ath5k_softc *sc)
{
	struct ath5k_hw *ah = sc->ah;
	int ret;

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
	ah->ah_cal_tstamp.tv_sec = 0;
	ah->ah_cal_tstamp.tv_nsec = 0;
	ah->ah_cal_intval.tv_sec = ath5k_calinterval;
	ah->ah_cal_intval.tv_nsec = 0;

	ret = 0;
done:
	return ret;
}


static int ath5k_intr(void *data, intr_t irq)
{
	struct ath5k_softc *sc = (void*) data;
	struct ath5k_hw *ah = sc->ah;
	enum ath5k_int status;
	unsigned int counter = 1000, rx_intr = 0;

	if (test_bit(ATH_STAT_INVALID, sc->status) ||
				!ath5k_hw_is_intr_pending(ah))
		return IRQ_NONE;

	do
	{
		ath5k_hw_get_isr(ah, &status);		/* NB: clears IRQ too */
		ATH5K_DBG(sc, ATH5K_DEBUG_INTR, "status 0x%x/0x%x\n",
				status, sc->imask);
		if (status & AR5K_INT_FATAL) {
			/*
			 * Fatal errors are unrecoverable.
			 * Typically these are caused by DMA errors.
			 */
			ATH5K_ERR(sc, "Fatal error (fatal interrupt).\n");
		} else if (unlikely(status & AR5K_INT_RXORN)) {
			ATH5K_ERR(sc, "Fatal error (RX descriptors overrun).\n");
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
			if (status & (AR5K_INT_RXOK | AR5K_INT_RXERR)){
				ath5k_rx_done(sc);
				rx_intr++;
			}
			if (status & (AR5K_INT_TXOK | AR5K_INT_TXDESC
					| AR5K_INT_TXERR | AR5K_INT_TXEOL))
				ath5k_tx_done(sc, sc->txq);

			if (status & AR5K_INT_SWI)
				ath5k_calibrate(sc);
			if (status & AR5K_INT_MIB) {
				/*
				 * These stats are also used for ANI i think
				 * so how about updating them more often ?
				 */
				ath5k_hw_update_mib_counters(ah, &sc->ll_stats);
			}
		}
	} while (ath5k_hw_is_intr_pending(ah) && --counter > 0);

	if (rx_intr == 1){
		sem_post(&ws);
//		fprintf(stderr,"DANI: intr: %d\n", rx_intr);
	}


	if (!counter)
		ATH5K_WARN(sc, "too many interrupts, giving up for now\n");

	ath5k_hw_calibration_poll(ah);

	return IRQ_HANDLED;
}


void ath5k_waitFrame(){
	sem_wait(&ws);
}

/*
 * Periodically recalibrate the PHY to account
 * for temperature/environment changes.
 */
static void
ath5k_calibrate(struct ath5k_softc *sc)
{
	struct ath5k_hw *ah = sc->ah;

	/* Only full calibration for now */
	if (ah->ah_swi_mask != AR5K_SWI_FULL_CALIBRATION)
		return;

	/* Contexto de intr, luego no hay que parar las colas porque nadie
	 * puede expulsar de la cpu. */

#if 0
	ATH5K_DBG(sc, ATH5K_DEBUG_CALIBRATE, "channel %u/%x\n",
		ath5k_frequency_to_channel(sc->curchan->center_freq),
		sc->curchan->hw_value);

	if (ath5k_hw_gainf_calibrate(ah) == AR5K_RFGAIN_NEED_CHANGE) {
		/*
		 * Rfgain is out of bounds, reset the chip
		 * to load new gain values.
		 */
		ATH5K_DBG(sc, ATH5K_DEBUG_RESET, "calibration, resetting\n");
		//ath5k_reset_wake(sc);
	}
	if (ath5k_hw_phy_calibrate(ah, sc->curchan))
		ATH5K_ERR(sc, "calibration of channel %u failed\n",
			ath5k_frequency_to_channel(
				sc->curchan->center_freq));
#endif

	ah->ah_swi_mask = 0;
}

static int ath5k_hw_rix_to_bitrate(struct ath5k_softc *sc, int hw_rix)
{
	int i;

	for (i = 0; i < ATH5K_NR_RATES; i++)
	{
		if (ath5k_rates[i].hw_value == hw_rix)
			return ath5k_rates[i].bitrate;
	}

	ATH5K_ERR(sc, "ath5k: invalid rix %02x\n", hw_rix);
	return 10; /* use lowest rate */
}

/**********************\
* Interface functions *
\**********************/
	int
ath5k_recv (struct ath5k_softc *sc, frame_t *frame, const struct timespec *abs_timeout)
{
	unsigned int sem_value;

	if (abs_timeout == NULL)
	{
		if (sem_wait (&sc->rxring.sem))
		{
			ATH5K_ERR(sc, "sem_wait failed\n");
			return -1;
		}
	}
	else
	{
		if ((sem_value = sem_timedwait (&sc->rxring.sem, abs_timeout)) != 0)
		{
			ATH5K_DBG(sc, ATH5K_DEBUG_TIMEOUT,
				   	"sem_timedwait timeout exceeded (%d)\n", sem_value);
			return -2;
		}
	}

	if (posix_intr_lock (sc->pdev->irq))
	{
		ATH5K_ERR(sc, "posix_intr_lock failed\n");
		return -3;
	}
	//----------------------------------------------
	ath5k_ring_extract (&sc->rxring, frame);
	/* Como hemos consumido el semaforo en el wait, abrirlo de nuevo si el
	 * anillo no esta vacio */
	if (!ath5k_ring_empty (&sc->rxring))
	{
		sem_getvalue (&sc->rxring.sem, &sem_value);
		if (sem_value == 0)
			sem_post (&sc->rxring.sem);
	}
	//----------------------------------------------
	if (posix_intr_unlock (sc->pdev->irq))
	{
		ATH5K_ERR(sc, "posix_intr_lock failed\n");
		return -4;
	}

	return 0;
}

/*
 * Contexto: UC
 *
 * Esta funcion transforma la cabecera ethernet en una 802.11, saca la primera
 * entrada de la lista sc->txbuf, y llama a ath5k_txbuf_setup pasandosela como
 * parametro (esa funcion se encarga de rellenar el descriptor y encolarla en
 * sc->txq)
 *
 * Hay que bloquear las interrupciones ya que el hardware lanza una interrupcion
 * cuando termina de enviar un elemento de la cola, y la ISR recoge esos 
 * elementos de sc->txq para volverlos a insertar al final de sc->txbuf.
 */
	int
ath5k_send (struct ath5k_softc *sc, const unsigned char *buff, const int nbytes)
{
	struct ath5k_buf *bf;
	struct sk_buff *skb;
	int hdrlen;
	int padsize;

	struct ethhdr *ethernet_hdr;
	unsigned short ethertype;
	struct ieee80211_hdr header;
	struct ieee80211_snap_hdr snap;
	const unsigned char *payload;
	int frame_control, data_len;

	/* Check data len */
	data_len = nbytes - ETH_HLEN;
	if (data_len > IEEE80211_MAX_DATA_LEN)
	{
		ATH5K_ERR (NULL, "Data too large (%d B). Max size is %d.\n", data_len, IEEE80211_MAX_DATA_LEN);
		return -1;
	}
	else
		payload = buff + ETH_HLEN;

	/* Get the first entry from sc->txbuf */
	cli();
	if (list_empty(&sc->txbuf)) {
		ATH5K_ERR(sc, "no further txbuf available, dropping packet\n");
		sti();
		return 0;
	}
	bf = list_first_entry(&sc->txbuf, struct ath5k_buf, list);
	list_del(&bf->list);
	sc->txbuf_len--;
	sti();

	skb = bf->skb;

	/* Fill IEEE 802.11 header */
	ethernet_hdr = (struct ethhdr *)buff;
	memcpy(header.addr1, ethernet_hdr->h_dest, ETH_ALEN);
	memcpy(header.addr2, ethernet_hdr->h_source, ETH_ALEN);
	memcpy(header.addr3, sc->ah->ah_bssid, ETH_ALEN);

	frame_control = IEEE80211_FTYPE_DATA | IEEE80211_STYPE_DATA;
	header.frame_control = cpu_to_le16(frame_control);

	hdrlen = 24;

	memcpy(skb_put(skb, hdrlen), &header, hdrlen);

	/* Copy SNAP and ethertype */
	snap.dsap = 0xaa;
	snap.ssap = 0xaa;
	snap.ctrl = 0x03;
	snap.oui[0] = 0x00;
	snap.oui[1] = 0x00;
	snap.oui[2] = 0x00;
	memcpy(skb_put(skb, SNAP_SIZE), &snap, SNAP_SIZE);
	ethertype = ethernet_hdr->h_proto;
	memcpy(skb_put(skb, sizeof(short)), &ethertype, sizeof(short));

	/* Copy payload */
	memcpy(skb_put(skb, data_len), payload, data_len);

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
			return -1;
		}
		skb_push(skb, padsize);
		memmove(skb->data, skb->data+padsize, hdrlen);
	}


	if (ath5k_txbuf_setup(sc, bf)) {
		cli();
		list_add_tail(&bf->list, &sc->txbuf);
		sc->txbuf_len++;
		sti();
		return 0;
	}

	return 0;
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

	if (chan)
	{
		ath5k_hw_set_imr(ah, 0);
		ath5k_txq_cleanup(sc);
		ath5k_rx_stop(sc);

		sc->curchan = chan;
		sc->curband = &sc->sbands[chan->band];
	}

	/* Change channel */
	ret = ath5k_hw_reset(ah, sc->curchan, true);
	if (ret) {
		ATH5K_ERR(sc, "can't reset hardware (%d)\n", ret);
		goto err;
	}

	/* Enable RX */
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

int ath5k_config (struct ath5k_softc *sc,
	   unsigned short freq,
	   enum rates rate,
	   unsigned char tx_power_dbm,
	   enum ath5k_ant_mode antenna_mode)
{
	struct ath5k_hw *ah = sc->ah;
	struct ieee80211_channel *target_channel;
	int rate_idx;

	/* Encontrar el canal */
	target_channel = ath5k_get_channel(sc, freq);
	if (target_channel == NULL)
	{
		ATH5K_ERR(sc, "Invalid channel\n");
		return -1;
	}

	/* Encontrar el bitrate */
	rate_idx = ath5k_get_rate_idx(sc, target_channel->band, rate);
	if (rate_idx < 0 || rate_idx > AR5K_MAX_RATES)
	{
		ATH5K_ERR(sc, "Invalid rate\n");
		return -1;
	}

	/* Configurar el rate */
	sc->tx_info.band = sc->curband->band;
	sc->tx_info.rate_idx = rate_idx;
	sc->tx_info.count = 1;
	sc->tx_info.wait_for_ack = false;
	sc->tx_info.use_short_preamble = false;

	if (sc->power_level != tx_power_dbm)
   	{
		sc->power_level = tx_power_dbm;

		/* Half dB steps */
		ath5k_hw_set_txpower_limit(ah, (tx_power_dbm * 2));
	}

	ath5k_hw_set_antenna_mode(ah, antenna_mode);

	/* Hacer un reset para aplicar los cambios */
	ath5k_reset(sc, target_channel);

	ATH5K_INFO(sc, "Freq %d Mhz, Rate (%d * 100) kps, Power %d dBm, Ant. mode %d\n",
			freq, rate, tx_power_dbm, antenna_mode);

	return 0;
}

int ath5k_setTxPower(struct ath5k_softc *sc, int dbm){
	struct ath5k_hw *ah = sc->ah;
	ath5k_hw_set_txpower_limit(ah, (dbm * 2));
}


void ath5k_config_filter(struct ath5k_softc *sc, bool broadcast, bool control, bool promisc)
{
	struct ath5k_hw *ah = sc->ah;
	unsigned int rfilt;

	rfilt = 0;

	/* Always enable Unicast */
	rfilt |= AR5K_RX_FILTER_UCAST;

	if (broadcast)
		rfilt |= AR5K_RX_FILTER_BCAST;

	/* Allow control frames? */
	if (control)
		rfilt |= AR5K_RX_FILTER_CONTROL;

	/* Promisc mode? */
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

int ath5k_config_tx_control (struct ath5k_softc *sc,
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

int ath5k_config_disable_ack (struct ath5k_softc *sc, bool disable)
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


	inline void
ath5k_config_debug_level(struct ath5k_softc *sc, unsigned int debug_mask)
{
	sc->debug_level = debug_mask;
}

	inline void
ath5k_get_interface_mac(struct ath5k_softc *sc, unsigned char *mac)
{
	ath5k_hw_get_lladdr(sc->ah, mac);
}

