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

#include "ath5k.h"
#include "mac80211.h"

/**
 * ieee80211_is_erp_rate - Check if a rate is an ERP rate
 * @phymode: The PHY-mode for this rate
 * @rate: Transmission rate to check, in 100 kbps
 *
 * Check if a given rate is an Extended Rate PHY (ERP) rate.
 */
static inline int ath5k_is_erp_rate(unsigned int mode, int rate)
{
	if (mode == AR5K_MODE_11G) {
		if (rate != 10 && rate != 20 &&
				rate != 55 && rate != 110)
			return 1;
	}
	return 0;
}

int ath5k_get_hdrlen(u16 fc)
{
	int hdrlen = 24;

	switch (fc & IEEE80211_FCTL_FTYPE) {
		case IEEE80211_FTYPE_DATA:
			if ((fc & IEEE80211_FCTL_FROMDS) && (fc & IEEE80211_FCTL_TODS))
				hdrlen = 30; /* Addr4 */
			/*
			 * The QoS Control field is two bytes and its presence is
			 * indicated by the IEEE80211_STYPE_QOS_DATA bit. Add 2 to
			 * hdrlen if that bit is set.
			 * This works by masking out the bit and shifting it to
			 * bit position 1 so the result has the value 0 or 2.
			 */
			hdrlen += (fc & IEEE80211_STYPE_QOS_DATA) >> 6;
			break;
		case IEEE80211_FTYPE_CTL:
			/*
			 * ACK and CTS are 10 bytes, all others 16. To see how
			 * to get this condition consider
			 *   subtype mask:   0b0000000011110000 (0x00F0)
			 *   ACK subtype:    0b0000000011010000 (0x00D0)
			 *   CTS subtype:    0b0000000011000000 (0x00C0)
			 *   bits that matter:         ^^^      (0x00E0)
			 *   value of those: 0b0000000011000000 (0x00C0)
			 */
			if ((fc & 0xE0) == 0xC0)
				hdrlen = 10;
			else
				hdrlen = 16;
			break;
	}

	return hdrlen;
}

int ath5k_get_hdrlen_from_skb(const struct sk_buff *skb)
{
	const struct ieee80211_hdr *hdr = (const struct ieee80211_hdr *) skb->data;
	int hdrlen;

	if (unlikely(skb->len < 10))
		return 0;
	hdrlen = ath5k_get_hdrlen(le16_to_cpu(hdr->frame_control));
	if (unlikely(hdrlen > skb->len))
		return 0;
	return hdrlen;
}

int ath5k_frame_duration(unsigned int mode, size_t len,
		int rate, int erp, int short_preamble)
{
	int dur;

	/* calculate duration (in microseconds, rounded up to next higher
	 * integer if it includes a fractional microsecond) to send frame of
	 * len bytes (does not include FCS) at the given rate. Duration will
	 * also include SIFS.
	 *
	 * rate is in 100 kbps, so divident is multiplied by 10 in the
	 * DIV_ROUND_UP() operations.
	 */

	if (mode == AR5K_MODE_11A || erp) {
		/*
		 * OFDM:
		 *
		 * N_DBPS = DATARATE x 4
		 * N_SYM = Ceiling((16+8xLENGTH+6) / N_DBPS)
		 *      (16 = SIGNAL time, 6 = tail bits)
		 * TXTIME = T_PREAMBLE + T_SIGNAL + T_SYM x N_SYM + Signal Ext
		 *
		 * T_SYM = 4 usec
		 * 802.11a - 17.5.2: aSIFSTime = 16 usec
		 * 802.11g - 19.8.4: aSIFSTime = 10 usec +
		 *      signal ext = 6 usec
		 */
		dur = 16; /* SIFS + signal ext */
		dur += 16; /* 17.3.2.3: T_PREAMBLE = 16 usec */
		dur += 4; /* 17.3.2.3: T_SIGNAL = 4 usec */
		dur += 4 * DIV_ROUND_UP((16 + 8 * (len + 4) + 6) * 10,
				4 * rate); /* T_SYM x N_SYM */
	} else {
		/*
		 * 802.11b or 802.11g with 802.11b compatibility:
		 * 18.3.4: TXTIME = PreambleLength + PLCPHeaderTime +
		 * Ceiling(((LENGTH+PBCC)x8)/DATARATE). PBCC=0.
		 *
		 * 802.11 (DS): 15.3.3, 802.11b: 18.3.4
		 * aSIFSTime = 10 usec
		 * aPreambleLength = 144 usec or 72 usec with short preamble
		 * aPLCPHeaderLength = 48 usec or 24 usec with short preamble
		 */
		dur = 10; /* aSIFSTime = 10 usec */
		dur += short_preamble ? (72 + 24) : (144 + 48);

		dur += DIV_ROUND_UP(8 * (len + 4) * 10, rate);
	}

	return dur;
}

/* Exported duration function for driver use */
__le16 ath5k_generic_frame_duration(unsigned int mode,
		bool short_preamble,
		size_t frame_len, int rate)
{
	u16 dur;
	int erp;

	erp = ath5k_is_erp_rate(mode, rate);
	dur = ath5k_frame_duration(mode, frame_len, rate, erp, short_preamble);

	return cpu_to_le16(dur);
}

int ath5k_frequency_to_channel(int freq)
{
	if (freq == 2484)
		return 14;

	if (freq < 2484)
		return (freq - 2407) / 5;

	/* FIXME: 802.11j 17.3.8.3.2 */
	return freq/5 - 1000;
}

