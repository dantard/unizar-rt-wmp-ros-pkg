/*
 * Copyright (c) 2007-2008 Bruno Randolf <bruno@thinktube.com>
 *
 *  This file is free software: you may copy, redistribute and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation, either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  This file is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * This file incorporates work covered by the following copyright and
 * permission notice:
 *
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
 */

#include "base.h"
#include "debug.h"

static unsigned int ath5k_debug;
//module_param_named(debug, ath5k_debug, uint, 0);

#ifdef CONFIG_ATH5K_DEBUG

/* functions used in other places */
void ath5k_set_debug_level (struct ath5k_softc *sc)
{
	sc->debug_level = ath5k_debug;
}

void
ath5k_debug_dump_bands(struct ath5k_softc *sc)
{
	unsigned int b, i;

	if (likely(!(sc->debug_level & ATH5K_DEBUG_DUMPBANDS)))
		return;

	BUG_ON(!sc->sbands);

	for (b = 0; b < IEEE80211_NUM_BANDS; b++) {
		struct ieee80211_supported_band *band = &sc->sbands[b];
		char bname[6];
		switch (band->band) {
		case IEEE80211_BAND_2GHZ:
			strcpy(bname, "2 GHz");
			break;
		case IEEE80211_BAND_5GHZ:
			strcpy(bname, "5 GHz");
			break;
		default:
			printk(KERN_DEBUG "Band not supported: %d\n",
				band->band);
			return;
		}
		printk(KERN_DEBUG "Band %s: channels %d, rates %d\n", bname,
				band->n_channels, band->n_bitrates);
		printk(KERN_DEBUG " channels:\n");
		for (i = 0; i < band->n_channels; i++)
			printk(KERN_DEBUG "  %3d %d %.4x\n",
					ath5k_frequency_to_channel(
						band->channels[i].center_freq),
					band->channels[i].center_freq,
					band->channels[i].hw_value);
		printk(KERN_DEBUG " rates:\n");
		for (i = 0; i < band->n_bitrates; i++)
			printk(KERN_DEBUG "  %4d %.4x %.4x %.4x\n",
					band->bitrates[i].bitrate,
					band->bitrates[i].hw_value,
					band->bitrates[i].flags,
					band->bitrates[i].hw_value_short);
	}
}

static inline void
ath5k_debug_printrxbuf(struct ath5k_buf *bf, int done,
		struct ath5k_rx_status *rs)
{
	struct ath5k_desc *ds = bf->desc;
	struct ath5k_hw_all_rx_desc *rd = &ds->ud.ds_rx;

	printk(KERN_DEBUG "R (%p %llx) %08x %08x %08x %08x %08x %08x %c\n",
		ds, (unsigned long long)bf->daddr,
		ds->ds_link, ds->ds_data,
		rd->rx_ctl.rx_control_0, rd->rx_ctl.rx_control_1,
		rd->u.rx_stat.rx_status_0, rd->u.rx_stat.rx_status_0,
		!done ? ' ' : (rs->rs_status == 0) ? '*' : '!');
}

void
ath5k_debug_printrxbuffs(struct ath5k_softc *sc, struct ath5k_hw *ah)
{
	struct ath5k_desc *ds;
	struct ath5k_buf *bf;
	struct ath5k_rx_status rs = {};
	int status;

	if (likely(!(sc->debug_level & ATH5K_DEBUG_RESET)))
		return;

	printk(KERN_DEBUG "rx queue %x, link %p\n",
		ath5k_hw_get_rxdp(ah), sc->rxlink);

	spin_lock_bh(&sc->rxbuflock);
	list_for_each_entry(bf, &sc->rxbuf, list) {
		ds = bf->desc;
		status = ah->ah_proc_rx_desc(ah, ds, &rs);
		if (!status)
			ath5k_debug_printrxbuf(bf, status == 0, &rs);
	}
	spin_unlock_bh(&sc->rxbuflock);
}

void
ath5k_debug_dump_skb(struct ath5k_softc *sc,
			struct sk_buff *skb, const char *prefix, int tx)
{
	char buf[16];

	if (likely(!((tx && (sc->debug_level & ATH5K_DEBUG_DUMP_TX)) ||
		     (!tx && (sc->debug_level & ATH5K_DEBUG_DUMP_RX)))))
		return;

	snprintf(buf, sizeof(buf), "phyX %s", prefix);

	print_hex_dump_bytes(buf, DUMP_PREFIX_NONE, skb->data,
		min(200U, skb->len));

	printk(KERN_DEBUG "\n");
}

void
ath5k_debug_printtxbuf(struct ath5k_softc *sc, struct ath5k_buf *bf)
{
	struct ath5k_desc *ds = bf->desc;
	struct ath5k_hw_5212_tx_desc *td = &ds->ud.ds_tx5212;
	struct ath5k_tx_status ts = {};
	int done;

	if (likely(!(sc->debug_level & ATH5K_DEBUG_RESET)))
		return;

	done = sc->ah->ah_proc_tx_desc(sc->ah, bf->desc, &ts);

	printk(KERN_DEBUG "T (%p %llx) %08x %08x %08x %08x %08x %08x %08x "
		"%08x %c\n", ds, (unsigned long long)bf->daddr, ds->ds_link,
		ds->ds_data, td->tx_ctl.tx_control_0, td->tx_ctl.tx_control_1,
		td->tx_ctl.tx_control_2, td->tx_ctl.tx_control_3,
		td->tx_stat.tx_status_0, td->tx_stat.tx_status_1,
		done ? ' ' : (ts.ts_status == 0) ? '*' : '!');
}

#endif /* ifdef CONFIG_ATH5K_DEBUG */
