/*
 * IEEE 802.11 defines
 *
 * Copyright (c) 2001-2002, SSH Communications Security Corp and Jouni Malinen
 * <jkmaline@cc.hut.fi>
 * Copyright (c) 2002-2003, Jouni Malinen <jkmaline@cc.hut.fi>
 * Copyright (c) 2005, Devicescape Software, Inc.
 * Copyright (c) 2006, Michael Wu <flamingice@sourmilk.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef IEEE80211_H
#define IEEE80211_H

#define FCS_LEN 4

#define IEEE80211_FCTL_VERS             0x0003
#define IEEE80211_FCTL_FTYPE            0x000c
#define IEEE80211_FCTL_STYPE            0x00f0
#define IEEE80211_FCTL_TODS             0x0100
#define IEEE80211_FCTL_FROMDS           0x0200
#define IEEE80211_FCTL_MOREFRAGS        0x0400
#define IEEE80211_FCTL_RETRY            0x0800
#define IEEE80211_FCTL_PM               0x1000
#define IEEE80211_FCTL_MOREDATA         0x2000
#define IEEE80211_FCTL_PROTECTED        0x4000
#define IEEE80211_FCTL_ORDER            0x8000

#define IEEE80211_SCTL_FRAG             0x000F
#define IEEE80211_SCTL_SEQ              0xFFF0

#define IEEE80211_FTYPE_MGMT            0x0000
#define IEEE80211_FTYPE_CTL             0x0004
#define IEEE80211_FTYPE_DATA            0x0008

/* management */
#define IEEE80211_STYPE_ASSOC_REQ       0x0000
#define IEEE80211_STYPE_ASSOC_RESP      0x0010
#define IEEE80211_STYPE_REASSOC_REQ     0x0020
#define IEEE80211_STYPE_REASSOC_RESP    0x0030
#define IEEE80211_STYPE_PROBE_REQ       0x0040
#define IEEE80211_STYPE_PROBE_RESP      0x0050
#define IEEE80211_STYPE_BEACON          0x0080
#define IEEE80211_STYPE_ATIM            0x0090
#define IEEE80211_STYPE_DISASSOC        0x00A0
#define IEEE80211_STYPE_AUTH            0x00B0
#define IEEE80211_STYPE_DEAUTH          0x00C0
#define IEEE80211_STYPE_ACTION          0x00D0

/* control */
#define IEEE80211_STYPE_BACK_REQ        0x0080
#define IEEE80211_STYPE_BACK            0x0090
#define IEEE80211_STYPE_PSPOLL          0x00A0
#define IEEE80211_STYPE_RTS             0x00B0
#define IEEE80211_STYPE_CTS             0x00C0
#define IEEE80211_STYPE_ACK             0x00D0
#define IEEE80211_STYPE_CFEND           0x00E0
#define IEEE80211_STYPE_CFENDACK        0x00F0

/* data */
#define IEEE80211_STYPE_DATA                    0x0000
#define IEEE80211_STYPE_DATA_CFACK              0x0010
#define IEEE80211_STYPE_DATA_CFPOLL             0x0020
#define IEEE80211_STYPE_DATA_CFACKPOLL          0x0030
#define IEEE80211_STYPE_NULLFUNC                0x0040
#define IEEE80211_STYPE_CFACK                   0x0050
#define IEEE80211_STYPE_CFPOLL                  0x0060
#define IEEE80211_STYPE_CFACKPOLL               0x0070
#define IEEE80211_STYPE_QOS_DATA                0x0080
#define IEEE80211_STYPE_QOS_DATA_CFACK          0x0090
#define IEEE80211_STYPE_QOS_DATA_CFPOLL         0x00A0
#define IEEE80211_STYPE_QOS_DATA_CFACKPOLL      0x00B0
#define IEEE80211_STYPE_QOS_NULLFUNC            0x00C0
#define IEEE80211_STYPE_QOS_CFACK               0x00D0
#define IEEE80211_STYPE_QOS_CFPOLL              0x00E0
#define IEEE80211_STYPE_QOS_CFACKPOLL           0x00F0


/* miscellaneous IEEE 802.11 constants */
#define IEEE80211_MAX_FRAG_THRESHOLD    2352
#define IEEE80211_MAX_RTS_THRESHOLD     2353
#define IEEE80211_MAX_AID               2007
#define IEEE80211_MAX_TIM_LEN           251
/* Maximum size for the MA-UNITDATA primitive, 802.11 standard section
   6.2.1.1.2.

   802.11e clarifies the figure in section 7.1.2. The frame body is
   up to 2304 octets long (maximum MSDU size) plus any crypt overhead. */
#define IEEE80211_MAX_DATA_LEN          2304
/* 30 byte 4 addr hdr, 2 byte QoS, 2304 byte MSDU, 12 byte crypt, 4 byte FCS */
#define IEEE80211_MAX_FRAME_LEN         2352

#define IEEE80211_MAX_SSID_LEN          32

#define IEEE80211_4ADDR_LEN 30
#define IEEE80211_3ADDR_LEN 24

struct ieee80211_hdr {
        __le16 frame_control;
        __le16 duration_id;
        u8 addr1[6];
        u8 addr2[6];
        u8 addr3[6];
        __le16 seq_ctrl;
        u8 addr4[6];
} __attribute__ ((packed));


#define P80211_OUI_LEN 3

struct ieee80211_snap_hdr {

        u8 dsap;                /* always 0xAA */
        u8 ssap;                /* always 0xAA */
        u8 ctrl;                /* always 0x03 */
        u8 oui[P80211_OUI_LEN]; /* organizational universal id */

} __attribute__ ((packed));

#define SNAP_SIZE sizeof(struct ieee80211_snap_hdr)


#endif /* IEEE80211_H */

