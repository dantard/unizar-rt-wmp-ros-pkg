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
 *  This program is distributed under the terms of GPL version 2 and in the 
 *  hope that it will be useful, but WITHOUT ANY WARRANTY; without even the 
 *  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
 *  See the GNU General Public License for more details.
 *
 *----------------------------------------------------------------------------*/

#ifndef _MARTE_ATH5K_INTERFACE_H_
#define _MARTE_ATH5K_INTERFACE_H_

#include <time.h>
#include "reg.h"
#include "mac80211.h"
#include "ieee80211.h"

/*******************\
*  IMPORTANT NOTES  *
\*******************/
/* CHIPSETS: This driver have been tested with the following chipsets, feel free
 * to complete the list:
 *    - AR5413
 *    - AR5414
 */

/*
 * FRAME FORMAT: For simplicity the interface works with ethernet frames. The
 * functions used for send and receive expect a buffer containing a ethernet
 * header before the data in network byte order.
 */

/*
 * OPERATIONAL MODE: This driver not support ad-hoc neither station mode. It
 * only send and receive frames over the air (like ah-demo mode in mad-wifi
 * driver).
 */

/*
 * CHANNELS AND FREQS: All regulatory restrictions have been removed and all
 * supported frequencies by the chipset's transceiver have been enabled. This
 * means that you can set frequency 2312 for example, so channel numbers have no 
 * sense in this driver. To see a list of all available modes, frequencies and 
 * rates supported by the chipset, set debug with AR5K_DEBUG_DUMP_BANDDS.
 * It's also illegal to tune to some of the supported frequencies in some 
 * countries, so use this at your own risk, you've been warned :)
 */


/***********\
*  DEFINES  *
\***********/
/*
 * This structure holds all buffers and device configuration.
 * There is one of this for each card in the system and the main app must store
 * a pointer to each one.
 */
struct ath5k_softc;

/*
 * This structure is used to store the received frames in the rx ring.
 *    @info: Ethernet header + payload
 *    @len: Length of data
 *    @link_quality: link quality in %
 *    @noise: noise floor in dBm
 *    @rate: the bitrate at witch the frame was received (see enum rates below)
 */
typedef struct
{
	unsigned char info[IEEE80211_MAX_FRAME_LEN];
	unsigned int len;
	int link_quality;
	int noise;
	int rate;
} frame_t;

/*
 * Data rates in 100 kbps units
 */
enum rates
{
	RATE_1M   = 10,		/* Valid for mode B/G */
   	RATE_2M   = 20,		/* Valid for mode B/G */
   	RATE_5_5M = 55,		/* Valid for mode B/G */
   	RATE_11M  = 110,	/* Valid for mode B/G */
   	RATE_6M   = 60,		/* Valid for mode A */
   	RATE_9M   = 90,		/* Valid for mode A */
	RATE_12M  = 120,	/* Valid for mode A/B/G */
   	RATE_18M  = 180,	/* Valid for mode A/B/G */
   	RATE_24M  = 240,	/* Valid for mode A/B/G */
   	RATE_36M  = 360,	/* Valid for mode A/B/G */
   	RATE_48M  = 480,	/* Valid for mode A/B/G */
   	RATE_54M  = 540		/* Valid for mode A/B/G */
};

/**
 * enum ath5k_debug_level - ath5k debug level
 *
 * @ATH5K_DEBUG_RESET: reset processing
 * @ATH5K_DEBUG_INTR: interrupt handling
 * @ATH5K_DEBUG_MODE: mode init/setup
 * @ATH5K_DEBUG_XMIT: basic xmit operation
 * @ATH5K_DEBUG_BEACON: beacon handling
 * @ATH5K_DEBUG_CALIBRATE: periodic calibration
 * @ATH5K_DEBUG_TXPOWER: transmit power setting
 * @ATH5K_DEBUG_LED: led management
 * @ATH5K_DEBUG_DUMP_RX: print received skb content
 * @ATH5K_DEBUG_DUMP_TX: print transmit skb content
 * @ATH5K_DEBUG_DUMPBANDS: dump bands
 * @ATH5K_DEBUG_TRACE: trace function calls
 * @ATH5K_DEBUG_ANY: show at any debug level
 *
 * The debug level is used to control the amount and type of debugging output
 * we want to see. The debug level is given in calls to ATH5K_DBG to specify
 * where the message should appear, and the user can control the debugging
 * messages he wants to see calling the function ath5k_config_debug_level.
 * These levels can be combined together by bitwise OR.
 */
enum ath5k_debug_level {
	ATH5K_DEBUG_NONE	= 0x00000000,
	ATH5K_DEBUG_RESET	= 0x00000001,
	ATH5K_DEBUG_INTR	= 0x00000002,
	ATH5K_DEBUG_MODE	= 0x00000004,
	ATH5K_DEBUG_XMIT	= 0x00000008,
	ATH5K_DEBUG_BEACON	= 0x00000010,
	ATH5K_DEBUG_CALIBRATE	= 0x00000020,
	ATH5K_DEBUG_TXPOWER	= 0x00000040,
	ATH5K_DEBUG_LED		= 0x00000080,
	ATH5K_DEBUG_DUMP_RX	= 0x00000100,
	ATH5K_DEBUG_DUMP_TX	= 0x00000200,
	ATH5K_DEBUG_DUMPBANDS	= 0x00000400,
	ATH5K_DEBUG_TIMEOUT 	= 0x00000800,
	ATH5K_DEBUG_TRACE	= 0x00001000,
	ATH5K_DEBUG_ANY		= 0xffffffff,
};

/*
 * Antenna mode
 */
enum ath5k_ant_mode {
	AR5K_ANTMODE_DEFAULT	= 0,	/* default antenna setup */
	AR5K_ANTMODE_FIXED_A	= 1,	/* only antenna A is present */
	AR5K_ANTMODE_FIXED_B	= 2,	/* only antenna B is present */
	AR5K_ANTMODE_SINGLE_AP	= 3,	/* sta locked on a single ap */
	AR5K_ANTMODE_SECTOR_AP	= 4,	/* AP with tx antenna set on tx desc */
	AR5K_ANTMODE_SECTOR_STA	= 5,	/* STA with tx antenna set on tx desc */
	AR5K_ANTMODE_DEBUG	= 6,	/* Debug mode -A -> Rx, B-> Tx- */
	AR5K_ANTMODE_MAX,
};


/******************\
*  INITIALIZATION  *
\******************/
/*
 * Init function. Find the card in the pci bus and initialize it.
 *    @prev_device: the previous found device, in the case that there are more
 *                  than one card in the system.
 *    @freq: Frecuency to set in Mhz units. There are no regulatory domain
 *           restrictions, be careful about legality. Available frequencies
 *           depend on chipset.
 *    @rate: Data rate used to tx frames in 100 kbps units. Available rates
 *           depend on used mode.
 *    @tx_power_dbm: Tx power in dBm units. Check your card specifications,
 *                   you can burn the transceiver. Also note that some hi-power
 *                   cards have a hardware coded offset.
 *    @antenna_mode: Control the antenna used for tx and diversity mode.
 */
extern struct ath5k_softc *ath5k_find (const struct ath5k_softc *prev_device,
                                       unsigned short freq, enum rates rate, 
                                       unsigned char tx_power_dbm,
                                       enum ath5k_ant_mode antenna_mode);

/*
 * Configure the debug level. For debug information produced in ath5k_find 
 * (like bands dumps) please modify 'debug_level' variable in debug.c
 *    @sc: Card to configure.
 *    @debug_level: Debugging mask.
 */
extern inline void ath5k_config_debug_level(struct ath5k_softc *sc, 
                                            unsigned int debug_mask);

/******************\
*  CONFIGURATION   *
\******************/
/* General configuration.
 *    @sc: Card to configure.
 *    @freq: Frequency to use in Mhz units. There are no regulatory domain
 *           restrictions, be careful about legality. Available frequencies
 *           depend on chipset.
 *    @rate: Data rate used to tx frames in 100 kbps units. Not all rates are 
             valid for each mode.
 *    @tx_power_dbm: Tx power in dBm units. Check your card specifications,
 *                   you can burn the transceiver. Also note that some hi-power
 *                   cards have a hardware coded offset above this value.
 *    @antenna_mode: Control the antenna used for tx and diversity mode.
 */
int ath5k_config (struct ath5k_softc *sc, unsigned short freq, enum rates rate, 
                  unsigned char tx_power_dbm, enum ath5k_ant_mode antenna_mode);


/*
 * Hardware RX filter configuration.
 *    @sc: Card to configure.
 *    @broadcast: Allow broadcast frames.
 *    @control: Allow control frames.
 *    @promisc: Promiscuous mode.
 */
extern void ath5k_config_filter (struct ath5k_softc *sc, bool broadcast, 
                                 bool control, bool promisc);

/*
 * TX parameters configuration.
 *    @sc: Card to configure.
 *    @count: if wait_for_ack is enabled the frames are transmitted up to 
 *            count + AR5K_TUNE_HWTXTRIES in the case that ack won't be 
 *            received. Minimum value is 1.
 *    @wait_for_ack: Wait for ack after TX a frame. When sending broadcast 
 *                   frames the receptor won't send ACK.
 *    @use_short_preamble: Use short preamble if it is available for the current
 *                         data rate.
 */
extern int  ath5k_config_tx_control (struct ath5k_softc *sc,
		                     unsigned char count, bool wait_for_ack, 
		                     bool use_short_preamble);

/*
 * Disable the ACK sending when a unicast frame is received.
 *    @sc: Card to configure.
 *    @disable: Disable ACK sending.
 */
extern int ath5k_config_disable_ack (struct ath5k_softc *sc, bool disable);

/*
 * XXX: NOT YET.
 * Configure the BSSID field used in the TX frames.
 *    @sc: Card to configure.
 *    @bssid: BSSID to use.
 */
//int ath5k_config_bssid(struct ath5k_softc *sc, unsigned char *bssid);

/*
 * XXX: NOT YET.
 * Configure the hardware BSSID filter. When configured, the HW only receive 
 * frames sent to this BSSID.
 *    @sc: Card to configure.
 *    @filter: BSSID to match.
 */
//int ath5k_config_bssid_filter(struct ath5k_softc *sc, unsigned char *filter);

/***************\
*  INFORMATION  *
\***************/
/*
 * Get the interface MAC address.
 *    @sc: Card to be requested.
 *    @mac: Buffer to hold the answer.
 */
inline void ath5k_get_interface_mac(struct ath5k_softc *sc, unsigned char *mac);

/********************\
*  SEND and RECEIVE  *
\********************/
/*
 * Receive function.
 *    @dev: Device to use.
 *    @frame: The received frame will be stored there (in ethernet format), with
 *            the rssi, noise and rate
 *    @abs_timeout: Wait until absolute timeout. If NULL, wait infinitely.
 */
int ath5k_recv (struct ath5k_softc *dev, frame_t *frame, 
                const struct timespec *abs_timeout);

/*
 * Send function.
 *    @dev: Device to use.
 *    @buff: Frame in ethernet format.
 *    @nbytes: Length of the buffer.
 */
int ath5k_send (struct ath5k_softc *dev, const unsigned char *buff,
                const int nbytes);


int ath5k_setTxPower(struct ath5k_softc *dev, int dbm);

#endif
