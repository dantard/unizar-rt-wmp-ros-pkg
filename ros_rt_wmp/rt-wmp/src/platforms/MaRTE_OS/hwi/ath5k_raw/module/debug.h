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
 * Copyright (c) 2007 Bruno Randolf <bruno@thinktube.com>
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

#ifndef _ATH5K_DEBUG_H
#define _ATH5K_DEBUG_H

#include "ath5k_interface.h"

/* set this to 1 for debugging output */
#ifndef ATH5K_DEBUG
#define ATH5K_DEBUG	0
#endif

struct ath5k_softc;
struct ath5k_hw;
struct sk_buff;
struct ath5k_buf;

#if ATH5K_DEBUG

#define ATH5K_TRACE(_sc) do { \
	if ((_sc)->debug_level & ATH5K_DEBUG_TRACE) \
		printf("ath5k trace %s:%d\n", __func__, __LINE__); \
} while (0)

#define ATH5K_DBG(_sc, _m, _fmt, ...) do { \
	if ((_sc)->debug_level & (_m)) \
		printf("ath5k dbg:" _fmt, ##__VA_ARGS__); \
} while (0)

#define ATH5K_DBG_UNLIMIT(_sc, _m, _fmt, ...) ATH5K_DBG(_sc, _m, _fmt, ##__VA_ARGS__)

void
ath5k_set_debug_level(struct ath5k_softc *sc);

void
ath5k_debug_dump_bands(struct ath5k_softc *sc);

void
ath5k_debug_dump_skb(struct ath5k_softc *sc,
			struct sk_buff *skb, const char *prefix, int tx);

void
ath5k_debug_printtxbuf(struct ath5k_softc *sc,
			struct ath5k_buf *bf);


#else /* no debugging */

#define ATH5K_TRACE(_sc) /* empty */

static inline void __attribute__ ((format (printf, 3, 4)))
ATH5K_DBG(struct ath5k_softc *sc, unsigned int m, const char *fmt, ...) {}

static inline void __attribute__ ((format (printf, 3, 4)))
ATH5K_DBG_UNLIMIT(struct ath5k_softc *sc, unsigned int m, const char *fmt, ...)
{}

static inline void
ath5k_debug_init(void) {}

static inline void
ath5k_set_debug_level(struct ath5k_softc *sc) {}

static inline void
ath5k_debug_finish(void) {}

static inline void
ath5k_debug_finish_device(struct ath5k_softc *sc) {}

static inline void
ath5k_debug_dump_bands(struct ath5k_softc *sc) {}

static inline void
ath5k_debug_dump_skb(struct ath5k_softc *sc,
			struct sk_buff *skb, const char *prefix, int tx) {}

static inline void
ath5k_debug_printtxbuf(struct ath5k_softc *sc,
			struct ath5k_buf *bf, int done) {}

#endif /* if ATH5K_DEBUG */

#endif /* ifndef _ATH5K_DEBUG_H */
