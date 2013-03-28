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

#include "debug.h"
#include "base.h"

unsigned int ath5k_debug = ATH5K_DEBUG_NONE;

#if ATH5K_DEBUG

void ath5k_set_debug_level (struct ath5k_softc *sc)
{
	sc->debug_level = ath5k_debug;
}

/*
 * Funciones del kernel para imprimir buffers
 */
#define _U      0x01    /* upper */
#define _L      0x02    /* lower */
#define _D      0x04    /* digit */
#define _C      0x08    /* cntrl */
#define _P      0x10    /* punct */
#define _S      0x20    /* white space (space/lf/tab) */
#define _X      0x40    /* hex digit */
#define _SP     0x80    /* hard space (0x20) */

unsigned char _ctype[] = {
_C,_C,_C,_C,_C,_C,_C,_C,                        /* 0-7 */
_C,_C|_S,_C|_S,_C|_S,_C|_S,_C|_S,_C,_C,         /* 8-15 */
_C,_C,_C,_C,_C,_C,_C,_C,                        /* 16-23 */
_C,_C,_C,_C,_C,_C,_C,_C,                        /* 24-31 */
_S|_SP,_P,_P,_P,_P,_P,_P,_P,                    /* 32-39 */
_P,_P,_P,_P,_P,_P,_P,_P,                        /* 40-47 */
_D,_D,_D,_D,_D,_D,_D,_D,                        /* 48-55 */
_D,_D,_P,_P,_P,_P,_P,_P,                        /* 56-63 */
_P,_U|_X,_U|_X,_U|_X,_U|_X,_U|_X,_U|_X,_U,      /* 64-71 */
_U,_U,_U,_U,_U,_U,_U,_U,                        /* 72-79 */
_U,_U,_U,_U,_U,_U,_U,_U,                        /* 80-87 */
_U,_U,_U,_P,_P,_P,_P,_P,                        /* 88-95 */
_P,_L|_X,_L|_X,_L|_X,_L|_X,_L|_X,_L|_X,_L,      /* 96-103 */
_L,_L,_L,_L,_L,_L,_L,_L,                        /* 104-111 */
_L,_L,_L,_L,_L,_L,_L,_L,                        /* 112-119 */
_L,_L,_L,_P,_P,_P,_P,_C,                        /* 120-127 */
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,                /* 128-143 */
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,                /* 144-159 */
_S|_SP,_P,_P,_P,_P,_P,_P,_P,_P,_P,_P,_P,_P,_P,_P,_P,   /* 160-175 */
_P,_P,_P,_P,_P,_P,_P,_P,_P,_P,_P,_P,_P,_P,_P,_P,       /* 176-191 */
_U,_U,_U,_U,_U,_U,_U,_U,_U,_U,_U,_U,_U,_U,_U,_U,       /* 192-207 */
_U,_U,_U,_U,_U,_U,_U,_P,_U,_U,_U,_U,_U,_U,_U,_L,       /* 208-223 */
_L,_L,_L,_L,_L,_L,_L,_L,_L,_L,_L,_L,_L,_L,_L,_L,       /* 224-239 */
_L,_L,_L,_L,_L,_L,_L,_P,_L,_L,_L,_L,_L,_L,_L,_L};      /* 240-255 */


#define __ismask(x) (_ctype[(int)(unsigned char)(x)])
#define isprint(c)      ((__ismask(c)&(_P|_U|_L|_D|_SP)) != 0)
#define isascii(c) (((unsigned char)(c))<=0x7f)
#define hex_asc(x)      "0123456789abcdef"[x]

int scnprintf(char * buf, size_t size, const char *fmt, ...)
{
        va_list args;
        int i;

        va_start(args, fmt);
        i = vsnprintf(buf, size, fmt, args);
        va_end(args);
        return (i >= size) ? (size - 1) : i;
}

void hex_dump_to_buffer(const void *buf, size_t len, int rowsize,
                        int groupsize, char *linebuf, size_t linebuflen,
                        bool ascii)
{
        const u8 *ptr = buf;
        u8 ch;
        int j, lx = 0;
        int ascii_column;

        if (rowsize != 16 && rowsize != 32)
                rowsize = 16;

        if (!len)
                goto nil;
        if (len > rowsize)              /* limit to one line at a time */
                len = rowsize;
        if ((len % groupsize) != 0)     /* no mixed size output */
                groupsize = 1;

        switch (groupsize) {
        case 8: {
                const u64 *ptr8 = buf;
                int ngroups = len / groupsize;

                for (j = 0; j < ngroups; j++)
                        lx += scnprintf(linebuf + lx, linebuflen - lx,
                                "%16.16llx ", (unsigned long long)*(ptr8 + j));
                ascii_column = 17 * ngroups + 2;
                break;
        }

        case 4: {
                const u32 *ptr4 = buf;
                int ngroups = len / groupsize;

                for (j = 0; j < ngroups; j++)
                        lx += scnprintf(linebuf + lx, linebuflen - lx,
                                "%8.8x ", *(ptr4 + j));
                ascii_column = 9 * ngroups + 2;
                break;
        }

        case 2: {
                const u16 *ptr2 = buf;
                int ngroups = len / groupsize;

                for (j = 0; j < ngroups; j++)
                        lx += scnprintf(linebuf + lx, linebuflen - lx,
                                "%4.4x ", *(ptr2 + j));
                ascii_column = 5 * ngroups + 2;
                break;
        }

        default:
                for (j = 0; (j < rowsize) && (j < len) && (lx + 4) < linebuflen;
                     j++) {
                        ch = ptr[j];
                        linebuf[lx++] = hex_asc(ch >> 4);
                        linebuf[lx++] = hex_asc(ch & 0x0f);
                        linebuf[lx++] = ' ';
                }
                ascii_column = 3 * rowsize + 2;
                break;
        }
        if (!ascii)
                goto nil;

        while (lx < (linebuflen - 1) && lx < (ascii_column - 1))
                linebuf[lx++] = ' ';
        for (j = 0; (j < rowsize) && (j < len) && (lx + 2) < linebuflen; j++)
                linebuf[lx++] = (isascii(ptr[j]) && isprint(ptr[j])) ? ptr[j]
                                : '.';
nil:
        linebuf[lx++] = '\0';
}

void print_hex_dump(const char *prefix_str, int prefix_type,
		int rowsize, int groupsize,
		const void *buf, size_t len, bool ascii)
{
	const u8 *ptr = buf;
	int i, linelen, remaining = len;
	char linebuf[200];

	if (rowsize != 16 && rowsize != 32)
		rowsize = 16;

	for (i = 0; i < len; i += rowsize) {
		linelen = min(remaining, rowsize);
		remaining -= rowsize;
		hex_dump_to_buffer(ptr + i, linelen, rowsize, groupsize,
				linebuf, sizeof(linebuf), ascii);

		printk("%s%s\n", prefix_str, linebuf);
	}
}

void print_hex_dump_bytes(const char *prefix_str, int prefix_type,
                        const void *buf, size_t len)
{
        print_hex_dump(prefix_str, prefix_type, 16, 1, buf, len, 1);
}

/* 
 * Debug functions 
 */
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

	printk(KERN_DEBUG "R (%p %x) %08x %08x %08x %08x %08x %08x %c\n",
		ds, (unsigned int)bf->desc,
		ds->ds_link, ds->ds_data,
		rd->rx_ctl.rx_control_0, rd->rx_ctl.rx_control_1,
		rd->u.rx_stat.rx_status_0, rd->u.rx_stat.rx_status_0,
		!done ? ' ' : (rs->rs_status == 0) ? '*' : '!');
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

	print_hex_dump_bytes(buf, 0, skb->data,
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

	printk(KERN_DEBUG "T (%p %x) %08x %08x %08x %08x %08x %08x %08x "
		"%08x %c\n", ds, (unsigned int)bf->desc, ds->ds_link,
		ds->ds_data, td->tx_ctl.tx_control_0, td->tx_ctl.tx_control_1,
		td->tx_ctl.tx_control_2, td->tx_ctl.tx_control_3,
		td->tx_stat.tx_status_0, td->tx_stat.tx_status_1,
		done ? ' ' : (ts.ts_status == 0) ? '*' : '!');
}

#endif /* if ATH5K_DEBUG */
