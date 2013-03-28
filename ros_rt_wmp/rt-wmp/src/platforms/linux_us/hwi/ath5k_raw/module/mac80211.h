/*
 * mac80211 <-> driver interface
 *
 * Copyright 2002-2005, Devicescape Software, Inc.
 * Copyright 2006-2007  Jiri Benc <jbenc@suse.cz>
 * Copyright 2007       Johannes Berg <johannes@sipsolutions.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef MAC80211_H
#define MAC80211_H

#include <linux/list.h>
#include <linux/skbuff.h>
#include <linux/if_ether.h>
#include "ieee80211.h"
#include "ath5k_interface.h"

/**
 * enum ieee80211_band - supported frequency bands
 *
 * The bands are assigned this way because the supported
 * bitrates differ in these bands.
 *
 * @IEEE80211_BAND_2GHZ: 2.4GHz ISM band
 * @IEEE80211_BAND_5GHZ: around 5GHz band (4.9-5.7)
 */
enum ieee80211_band {
        IEEE80211_BAND_2GHZ,
        IEEE80211_BAND_5GHZ,

        /* keep last */
        IEEE80211_NUM_BANDS
};

/**
 * struct ieee80211_channel - channel definition
 *
 * This structure describes a single channel for use
 * with cfg80211.
 *
 * @center_freq: center frequency in MHz
 * @hw_value: hardware-specific value for the channel
 * @band: band this channel belongs to.
 */
struct ieee80211_channel {
		enum ieee80211_band band;
        u16 center_freq; /* frequency in MHz */
        u16 hw_value; /* hw specific value for the channel */
};

/**
 * enum ieee80211_rate_flags - rate flags
 *
 * Hardware/specification flags for rates. These are structured
 * in a way that allows using the same bitrate structure for
 * different bands/PHY modes.
 *
 * @IEEE80211_RATE_SHORT_PREAMBLE: Hardware can send with short
 *      preamble on this bitrate; only relevant in 2.4GHz band and
 *      with CCK rates.
 * @IEEE80211_RATE_MANDATORY_A: This bitrate is a mandatory rate
 *      when used with 802.11a (on the 5 GHz band); filled by the
 *      core code when registering the wiphy.
 * @IEEE80211_RATE_MANDATORY_B: This bitrate is a mandatory rate
 *      when used with 802.11b (on the 2.4 GHz band); filled by the
 *      core code when registering the wiphy.
 * @IEEE80211_RATE_MANDATORY_G: This bitrate is a mandatory rate
 *      when used with 802.11g (on the 2.4 GHz band); filled by the
 *      core code when registering the wiphy.
 * @IEEE80211_RATE_ERP_G: This is an ERP rate in 802.11g mode.
 */
enum ieee80211_rate_flags {
        IEEE80211_RATE_SHORT_PREAMBLE   = 1<<0,
};

/**
 * struct ieee80211_rate - bitrate definition
 *
 * This structure describes a bitrate that an 802.11 PHY can
 * operate with. The two values @hw_value and @hw_value_short
 * are only for driver use when pointers to this structure are
 * passed around.
 *
 * @flags: rate-specific flags
 * @bitrate: bitrate in units of 100 Kbps
 * @hw_value: driver/hardware value for this rate
 * @hw_value_short: driver/hardware value for this rate when
 *      short preamble is used
 */
struct ieee80211_rate {
        u32 flags;
        u16 bitrate;
        u16 hw_value, hw_value_short;
};

struct ieee80211_tx_info {
	u8 band;
	s8 rate_idx;
	u8 count;
	bool use_short_preamble;
	bool wait_for_ack;
};

/**
 * struct ieee80211_rx_status - receive status
 *
 * The low-level driver should provide this information (the subset
 * supported by hardware) to the 802.11 code with each received
 * frame.
 *
 * @mactime: value in microseconds of the 64-bit Time Synchronization Function
 *      (TSF) timer when the first data symbol (MPDU) arrived at the hardware.
 * @band: the active band when this frame was received
 * @freq: frequency the radio was tuned to when receiving this frame, in MHz
 * @signal: signal strength when receiving this frame, either in dBm, in dB or
 *      unspecified depending on the hardware capabilities flags
 *      @IEEE80211_HW_SIGNAL_*
 * @noise: noise when receiving this frame, in dBm.
 * @qual: overall signal quality indication, in percent (0-100).
 * @antenna: antenna used
 * @rate_idx: index of data rate into band's supported rates or MCS index if
 *      HT rates are use (RX_FLAG_HT)
 * @flag: %RX_FLAG_*
 */
struct ieee80211_rx_status {
        u64 mactime;
        enum ieee80211_band band;
        int freq;
        int signal;
        int noise;
        int qual;
        int antenna;
        int rate_idx;
        int flag;
};

/**
 * enum nl80211_iftype - (virtual) interface types
 *
 * @NL80211_IFTYPE_UNSPECIFIED: unspecified type, driver decides
 * @NL80211_IFTYPE_ADHOC: independent BSS member
 * @NL80211_IFTYPE_STATION: managed BSS member
 * @NL80211_IFTYPE_AP: access point
 * @NL80211_IFTYPE_AP_VLAN: VLAN interface for access points
 * @NL80211_IFTYPE_WDS: wireless distribution interface
 * @NL80211_IFTYPE_MONITOR: monitor interface receiving all frames
 * @NL80211_IFTYPE_MESH_POINT: mesh point
 * @NL80211_IFTYPE_MAX: highest interface type number currently defined
 * @__NL80211_IFTYPE_AFTER_LAST: internal use
 *
 * These values are used with the %NL80211_ATTR_IFTYPE
 * to set the type of an interface.
 *
 */
enum nl80211_iftype {
        NL80211_IFTYPE_UNSPECIFIED,
        NL80211_IFTYPE_ADHOC,
        NL80211_IFTYPE_STATION,
        NL80211_IFTYPE_AP,
        NL80211_IFTYPE_AP_VLAN,
        NL80211_IFTYPE_WDS,
        NL80211_IFTYPE_MONITOR,
        NL80211_IFTYPE_MESH_POINT,

        /* keep last */
        __NL80211_IFTYPE_AFTER_LAST,
        NL80211_IFTYPE_MAX = __NL80211_IFTYPE_AFTER_LAST - 1
};

/**
 * struct ieee80211_supported_band - frequency band definition
 *
 * This structure describes a frequency band a wiphy
 * is able to operate in.
 *
 * @channels: Array of channels the hardware can operate in
 *      in this band.
 * @band: the band this structure represents
 * @n_channels: Number of channels in @channels
 * @bitrates: Array of bitrates the hardware can operate with
 *      in this band. Must be sorted to give a valid "supported
 *      rates" IE, i.e. CCK rates first, then OFDM.
 * @n_bitrates: Number of bitrates in @bitrates
 */
struct ieee80211_supported_band {
        struct ieee80211_channel *channels;
        struct ieee80211_rate *bitrates;
        enum ieee80211_band band;
        int n_channels;
        int n_bitrates;
};

struct ieee80211_low_level_stats {
        unsigned int dot11ACKFailureCount;
        unsigned int dot11RTSFailureCount;
        unsigned int dot11FCSErrorCount;
        unsigned int dot11RTSSuccessCount;
};

/**
 * ieee80211_generic_frame_duration - Calculate the duration field for a frame
 * @hw: pointer obtained from ieee80211_alloc_hw().
 * @vif: &struct ieee80211_vif pointer from &struct ieee80211_if_init_conf.
 * @frame_len: the length of the frame.
 * @rate: the rate (in 100kbps) at which the frame is going to be transmitted.
 *
 * Calculate the duration field of some generic frame, given its
 * length and transmission rate (in 100kbps).
 */
__le16 ath5k_generic_frame_duration(unsigned int mode,
										bool short_preamble,
                                        size_t frame_len,
                                        int rate);
/**
 * ieee80211_get_hdrlen_from_skb - get header length from data
 *
 * Given an skb with a raw 802.11 header at the data pointer this function
 * returns the 802.11 header length in bytes (not including encryption
 * headers). If the data in the sk_buff is too short to contain a valid 802.11
 * header the function returns 0.
 *
 * @skb: the frame
 */
int ath5k_get_hdrlen_from_skb(const struct sk_buff *skb);

/**
 * ieee80211_frequency_to_channel - convert frequency to channel number
 */
extern int ath5k_frequency_to_channel(int freq);


#endif /* MAC80211_H */

