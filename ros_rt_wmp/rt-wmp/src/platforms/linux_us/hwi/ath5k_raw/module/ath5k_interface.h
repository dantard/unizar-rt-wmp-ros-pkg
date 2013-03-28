#ifndef _ATH5K_INTERFACE_H_
#define _ATH5K_INTERFACE_H_

#include <stdbool.h>

/* Private ioctl's */
#define SIO_SEND_PACKET     (SIOCDEVPRIVATE + 0)
#define SIO_RECV_PACKET     (SIOCDEVPRIVATE + 1)
#define SIO_SET_CONFIG      (SIOCDEVPRIVATE + 2)
#define SIO_SET_DEBUG       (SIOCDEVPRIVATE + 3)
#define SIO_SET_RXFILTER    (SIOCDEVPRIVATE + 4)
#define SIO_SET_TXCONTROL   (SIOCDEVPRIVATE + 5)
#define SIO_SET_DISABLEACK          (SIOCDEVPRIVATE + 6)
#define SIO_SET_BSSID       		(SIOCDEVPRIVATE + 7)
#define SIO_SET_BSSIDFILTER 		(SIOCDEVPRIVATE + 8)
#define SIO_SET_USE_BEACON_FRAMES 	(SIOCDEVPRIVATE + 9)
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
 * messages he wants to see, either by the module parameter 'debug' on module
 * load, or dynamically by using debugfs 'ath5k/phyX/debug'. these levels can
 * be combined together by bitwise OR.
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
	ATH5K_DEBUG_TRACE	= 0x00001000,
	ATH5K_DEBUG_ANY		= 0xffffffff
};

/*
 * Interface configuration
 *  @freq: Frecuency to use in Mhz units. There are no regulatory domain
 *		   restrictions, be carefull about legality. Available frequencies 
 *		   depend on chipset.
 *  @rate: Data rate used to tx frames in 100 kbps units. Not all rates are valid 
 *         for each mode.
 *  @tx_power_dbm: Tx power in dBm units. Please read your card specifications,
 *				   you can burn the transmitter. Also note that some hi-power
 *				   cards have a hardware coded offset abobe this value.
 *	@antenna_mode: Antenna mode control some things such diversity.
 */
struct ath5k_config_info
{
	unsigned short frequency;
	enum rates rate;
	unsigned char tx_power_dbm;
	enum ath5k_ant_mode antenna_mode;
};

/* 
 * Hardware RX filter configuration.
 *	@broadcast: Allow broadcast frames.
 *	@control: Allow control frames.
 *	@promisc: Pomiscuous mode.
 */
struct ath5k_rxfilter_info
{
	bool broadcast;
	bool control;
	bool promisc;
};

/* 
 * TX parameters configuration.
 *	@count: if wait_for_ack is enabled the frames are transmitted up to count +
 *	        AR5K_TUNE_HWTXTRIES in the case that ack won't be received. Minimun
 *	        value is 1.
 *	@wait_for_ack: Wait for ack after TX a frame. Be careful when sending broadcast 
 *	               frames becouse the receptor won't send ACK.
 *	@use_short_preamble: Use short preamble if it is available for the current
 *	                     data rate.
 */
struct ath5k_txcontrol_info
{
	unsigned char count;
	bool wait_for_ack;
	bool use_short_preamble;
};

/*
 * Ioctl data.
 *  @debug_level: Configure debug level (values defined in debug.h).
 *  @disable_ack: Disable the ACK sending when a unicast frame is received.
 */
union ath5k_ioctl_info
{
	unsigned int debug_level;
	struct ath5k_config_info config_info;
	struct ath5k_rxfilter_info rxfilter_info;
	struct ath5k_txcontrol_info txcontrol_info;
	bool disable_ack;
};

#endif
