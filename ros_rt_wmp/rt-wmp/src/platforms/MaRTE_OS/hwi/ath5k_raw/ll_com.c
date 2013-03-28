/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/platforms/MaRTE_OS/hwi/ath5k/ll_com.c
 *  Authors: Samuel Cabrero, Danilo Tardioli
 *  ----------------------------------------------------------------------
 *  Copyright (C) 2000-2010, Universidad de Zaragoza, SPAIN
 *
 *  Contact Addresses: Danilo Tardioli                   dantard@unizar.es
 *
 *  RT-WMP is free software; you can  redistribute it and/or  modify it
 *  under the terms of the GNU General Public License  as published by the
 *  Free Software Foundation;  either  version 2, or (at  your option) any
 *  later version.
 *
 *  RT-WMP  is distributed  in the  hope  that  it will be   useful, but
 *  WITHOUT  ANY  WARRANTY;     without  even the   implied   warranty  of
 *  MERCHANTABILITY  or  FITNESS FOR A  PARTICULAR PURPOSE.    See the GNU
 *  General Public License for more details.
 *
 *  You should have received  a  copy of  the  GNU General Public  License
 *  distributed with RT-WMP;  see file COPYING.   If not,  write to the
 *  Free Software  Foundation,  59 Temple Place  -  Suite 330,  Boston, MA
 *  02111-1307, USA.
 *
 *  As a  special exception, if you  link this  unit  with other  files to
 *  produce an   executable,   this unit  does  not  by  itself cause  the
 *  resulting executable to be covered by the  GNU General Public License.
 *  This exception does  not however invalidate  any other reasons why the
 *  executable file might be covered by the GNU Public License.
 *
 *----------------------------------------------------------------------*/

#include "config/compiler.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "core/include/definitions.h"
#include "core/include/frames.h"
#include "core/include/wmp_misc.h"
#include "core/include/wmp_utils.h"
#include "core/include/ml_com.h"

/////*** MARTE OS ********/////
#include <drivers/if_ether.h>
#include <drivers/osdep.h>
#include "module/ath5k_interface.h"

/* Interface configuration */

#define FREQ		5200
#define RATE		RATE_6M
/*
 * Las tarjetas EMP-8603 tienen un offset sobre este valor codificado
 * directamente en el hardware, ademas es diferente en la banda A y la banda G.
 * Consultar el datasheet para evitar quemar el amplificador
 * Offset 2Ghz -> 9dB
 * Offset 5Ghz -> 7dB
 */
#define TXPOWER_DBM 1
/*
 * Las tarjetas EMP-8603 utilizan una configuracion de antenas peculiar. De las
 * dos salidas una es para banda 2Ghz y otra para banda 5Ghz, pero para el
 * driver ambas son la antena A. No utilizar la configuracion por defecto en
 * estas tarjetas porque hace diversity, y la antena B esta conectada a
 * /dev/null, lo que provoca mal funcionamiento (emite a baja potencia, pierde
 * paquetes, etc)
 */
#define ANT_MODE	AR5K_ANTMODE_FIXED_A

/* TODO: mover a core/include/definitions.h */
#define WMP_PROTOCOL	0x6969

/* Pointer to the wifi card */
static struct ath5k_softc *tarjeta1;

/* Cached frame used for tx */
static unsigned char ethernet_frame[2500];

/* Frames received not belonging to RT-WMP */
int foreign_frames = 0;

static double ath5k_rate[] = { 0, 0, 0, 0, 0, 0, 0, 0, 48, 24, 12, 6, 54, 36,
		18, 9, 0, 0, 0, 0, 0, 0, 0, 0, 11, 5.5, 2, 1, 0, 0, 0, 0, 0 };

static void pause() {
	char key;
	printf(" press Enter...");
	key = getchar();
}

static void msg(char *s) {
	printf(s);
	pause();
}

void print_packet_hex(char *msg, unsigned char *packet, int len) {
	unsigned char *p = packet;

	printf("---------Packet---Starts----\n");
	printf("%s", msg);
	while (len--) {
		printf("%.2x ", *p);
		p++;
	}
	printf("\n--------Packet---Ends-----\n\n");
}

///*** MARTE OS ********/////

int readllcfg() {
	return 1;
}

void closeLowLevelCom() {
}

int initLowLevelCom() {
	bool broadcast, control, promisc, wait_for_ack, use_short_preamble;
	unsigned int count;
	unsigned char mac[ETH_ALEN];
	struct ethhdr *ethernet_header;

	/* Send output to serial port */
	//SERIAL_CONSOLE_INIT();

	/* Find and init the first card */
	tarjeta1 = ath5k_find(NULL, FREQ, RATE, TXPOWER_DBM, ANT_MODE);

	if (tarjeta1 == NULL)
		return 0;

	/* Config the hardware rx filter */
	broadcast = true;
	control = true;
	promisc = true;
	ath5k_config_filter(tarjeta1, broadcast, control, promisc);

	/* Select tx rate, disable soft retries and disable ACK waiting for unicast frames */
	wait_for_ack = false;
	use_short_preamble = false;
	count = 1;
	ath5k_config_tx_control(tarjeta1, 1, wait_for_ack, use_short_preamble);

	/* Disable ACK unicast frames */
	ath5k_config_disable_ack(tarjeta1, true);

	/* Set debug level */
	ath5k_config_debug_level(tarjeta1, ATH5K_DEBUG_NONE);

	/* Get card's mac address */
	ath5k_get_interface_mac(tarjeta1, mac);

	/* Init cached ethernet frame */
	ethernet_header = (struct ethhdr *) ethernet_frame;
	memset(ethernet_header->h_dest, 0xFF, ETH_ALEN);
	memcpy(ethernet_header->h_source, mac, ETH_ALEN);
	ethernet_header->h_proto = htons(WMP_PROTOCOL);

	printf("Initialization Completed...\n");

	return 1;
}

int llsend(char * f, int size) {
	int ret;
	unsigned char *p;

	p = ethernet_frame + ETH_HLEN;

	memcpy(p, f, size);
	p += size;

	ret = ath5k_send(tarjeta1, ethernet_frame, p - ethernet_frame);

	if (ret < 0){
		return -1;
	}
	return 1;
}
#include <math.h>
rxInfo llreceive(char *f, int timeout) {
	rxInfo ret;
	frame_t frame;
	struct ethhdr *ethernet_header = (struct ethhdr *) frame.info;;
	struct timespec abs_timeout,t1,t2;
	int read_bytes;

	clock_gettime(CLOCK_REALTIME, &abs_timeout);
	wmp_add_ms(&abs_timeout, timeout);
	int r1 = ath5k_recv(tarjeta1, &frame, &abs_timeout);
	if (r1 < 0) {
		ret.error = 1;
		return ret;
	}
	memcpy(f, frame.info + ETH_HLEN, frame.len - ETH_HLEN);
	ret.size = frame.len - ETH_HLEN;
	ret.rate = frame.rate;
	ret.error = 0;
	ret.proto = ethernet_header->h_proto;

//	drssi = 100*frame.link_quality/75;
//	/* calculo RSSI */
//	double dbm = frame.link_quality - 95;
//	double drssi = 4500*sqrt(pow(10,dbm/10));
//
//	if (drssi > 100){
//		drssi = 100;
//	}
//
//	drssi = 100*frame.link_quality/75;
//	/* calculo RSSI */
//	ret.rssi = (char) drssi;

	ret.rssi = (char) frame.link_quality;
	ret.noise = (char) frame.noise;
	ret.has_lq = 1;
	return ret;
}

int llsetPower(int dbm){
	fprintf(stderr,"Setting power to %d dBm\n",dbm);
	ath5k_setTxPower(tarjeta1,dbm);
}

int llsetConfig(unsigned short freq, enum rates rate, unsigned char tx_power_dbm){
	ath5k_config(tarjeta1,freq,rate,tx_power_dbm, AR5K_ANTMODE_FIXED_A);
}
