/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/platforms/linux_us/hwi/ath5k/ll_com.c
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

#include <sys/types.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "errno.h"
#include <pthread.h>
#include "config/compiler.h"
#include "core/include/definitions.h"
#include "core/interface/wmp_interface.h"
#include "core/include/wmp_misc.h"
#include "core/include/ml_com.h"


#include <assert.h>
#include <linux/if.h>
#include <linux/if_packet.h>
#include <linux/if_ether.h>
#include <sys/ioctl.h>
#include <linux/if_ether.h>
#include "module/ath5k_interface.h"

#define WMP_PROTO 0x6969 /* TODO: Mover a un sitio apropiado */

/* Interface default configuration */
#define DEFAULT_FREQ		5200
#define DEFAULT_RATE		RATE_6M
#define DEFAULT_TXPOWER_DBM 15
#define DEFAULT_ANTENNA_MODE AR5K_ANTMODE_DEFAULT

#define ATH5K_RATE_CODE_1M	0x1B
#define ATH5K_RATE_CODE_2M	0x1A
#define ATH5K_RATE_CODE_5_5M	0x19
#define ATH5K_RATE_CODE_11M	0x18
/* A and G */
#define ATH5K_RATE_CODE_6M	0x0B
#define ATH5K_RATE_CODE_9M	0x0F
#define ATH5K_RATE_CODE_12M	0x0A
#define ATH5K_RATE_CODE_18M	0x0E
#define ATH5K_RATE_CODE_24M	0x09
#define ATH5K_RATE_CODE_36M	0x0D
#define ATH5K_RATE_CODE_48M	0x08
#define ATH5K_RATE_CODE_54M	0x0C

static double ath5k_rate[] = { 0, 0, 0, 0, 0, 0, 0, 0, 48, 24, 12, 6, 54, 36,
		18, 9, 0, 0, 0, 0, 0, 0, 0, 0, 11, 5.5, 2, 1, 0, 0, 0, 0, 0 };

static int rx, tx; /* TX and RX sockets*/
static char dev[20];
static char param[256], val[256];

static short freq;
static short rate;
static unsigned char txpower_dbm;
static int antenna_mode;
static unsigned short raw_filter = WMP_PROTO;
static struct timeval tv;

/* Cached frame used for tx */
static unsigned char ethernet_frame[2500];

void* bridge(void *);
int iw_enum_devices(char  a[10][256]);
int iw_has_module(char * mod);

void print_packet_hex(char *msg, unsigned char *packet, int len) {
	unsigned char *p = packet;

	fprintf(stderr,"---------Packet---Starts----\n");
	printf("%s", msg);
	while (len--) {
		fprintf(stderr,"%.2x ", *p);
		p++;
	}
	fprintf(stderr,"\n--------Packet---Ends-----\n\n");
}

int readllcfg() {
	FILE * f;
	char filename[256], line[256];
	/* Set default values */
	sprintf(dev, "wlan0");
	freq = DEFAULT_FREQ;
	rate = DEFAULT_RATE;
	txpower_dbm = DEFAULT_TXPOWER_DBM;
	antenna_mode = DEFAULT_ANTENNA_MODE;

	snprintf(filename, 256, "%s/.rt-wmp/rt-wmp-us-ath5k.ll", getenv("HOME"));
	f = fopen(filename, "r");

	if (f > 0) {
		WMP_MSG(stderr, "Reading Low Level Configuration file (%s)... \n", filename);
		while (fgets(line,256,f) != NULL) {
			if (line[0]<65 || line[0]>90){
				continue;
			}
			sscanf(line,"%s %s",param,val);
			int exists = 0;
			if (strcmp(param, "DEVICE") == 0){
				strcpy(dev, val);
				exists = 1;
			}
			else if (strcmp(param, "FREQ") == 0){
				freq = atoi(val);
				exists = 1;
			}
			else if (strcmp(param, "RATE") == 0){
				rate = atoi(val);
				exists = 1;
			}
			else if (strcmp(param, "TXPOWER_DBM") == 0){
				txpower_dbm = atoi(val);
				exists = 1;
			}
			else if (strcmp(param, "ANTENNA_MODE") == 0) {
				if (strcmp(val, "AR5K_ANTMODE_DEFAULT") == 0){
					antenna_mode = AR5K_ANTMODE_DEFAULT;
					exists = 1;
				}
				else if (strcmp(val, "AR5K_ANTMODE_FIXED_A") == 0){
					antenna_mode = AR5K_ANTMODE_FIXED_A;
					exists = 1;
				}
				else if (strcmp(val, "AR5K_ANTMODE_FIXED_B") == 0){
					antenna_mode = AR5K_ANTMODE_FIXED_B;
					exists = 1;
				}
				else if (strcmp(val, "AR5K_ANTMODE_SINGLE_AP") == 0){
					antenna_mode = AR5K_ANTMODE_SINGLE_AP;
					exists = 1;
				}
				else if (strcmp(val, "AR5K_ANTMODE_SECTOR_AP") == 0){
					antenna_mode = AR5K_ANTMODE_SECTOR_AP;
					exists = 1;
				}
				else if (strcmp(val, "AR5K_ANTMODE_SECTOR_STA") == 0){
					antenna_mode = AR5K_ANTMODE_SECTOR_STA;
				}
				else if (strcmp(val, "AR5K_ANTMODE_DEBUG") == 0){
					antenna_mode = AR5K_ANTMODE_DEBUG;
					exists = 1;
				}
			}
			if (exists) {
				WMP_MSG(stderr, "READ OPTION: %s = %s\n", param, val);
			}else{
				WMP_MSG(stderr, "WARNING ::: UKNOWN OPTION %s\n", param, val);
			}
		}
		WMP_MSG(stderr, "Done.\n");

	} else
		WMP_MSG(stderr, "File %s not found, using default values.\n", filename);
	return 0;
}

void closeLowLevelCom() {
	close(rx);
	close(tx);
}

int configure_interface(int sock, char *ifname) {
	int ret = 0;
	struct ifreq ifr;

	strcpy(ifr.ifr_name, ifname);

	/* Configure freq, rate and power*/
	struct ath5k_config_info config_info;

	config_info.frequency = freq;
	config_info.rate = rate;
	config_info.tx_power_dbm = txpower_dbm;
	config_info.antenna_mode = antenna_mode;

	ifr.ifr_data = (void *) &config_info;
	if ((ioctl(sock, SIO_SET_CONFIG, &ifr)) < 0) {
		perror("Error configuring interface");
		ret--;
	}

	/* Set rx filter */
	struct ath5k_rxfilter_info rxfilter_info;

	rxfilter_info.broadcast = true;
	rxfilter_info.control = false;
	rxfilter_info.promisc = true;

	ifr.ifr_data = (void *) &rxfilter_info;
	if ((ioctl(sock, SIO_SET_RXFILTER, &ifr)) < 0) {
		perror("Error setting hw rx filter");
		ret--;
	}

	/* Select tx rate, disable soft retries and disable ACK waiting for unicast frames */
	struct ath5k_txcontrol_info txcontrol_info;

	txcontrol_info.wait_for_ack = false;
	txcontrol_info.use_short_preamble = false;
	txcontrol_info.count = 1;

	ifr.ifr_data = (void *) &txcontrol_info;
	if ((ioctl(sock, SIO_SET_TXCONTROL, &ifr)) < 0) {
		perror("Error setting tx control");
		ret--;
	}

	/* Disable ACK unicast frames */
	bool disable_ack = true;

	ifr.ifr_data = (void *) &disable_ack;
	if ((ioctl(sock, SIO_SET_DISABLEACK, &ifr)) < 0) {
		perror("Error disabling ack");
		ret--;
	}
	/* Set debug level */
	unsigned int debug_level = ATH5K_DEBUG_NONE;

	ifr.ifr_data = (void *) &debug_level;
	if ((ioctl(sock, SIO_SET_DEBUG, &ifr)) < 0) {
		perror("Error disabling debug");
		ret--;
	}
	return ret;
}

int initLowLevelCom() {
	struct sockaddr_ll rx_addr, tx_addr;
	struct ifreq ifr;
	struct ethhdr *ethernet_header;
	int ret = 0;
	unsigned char dst_mac[ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

	sleep(1);

	if (readllcfg() == -1){
		ret--;
	}

	llsetupModule(dev);

	WMP_DBG(LL_COM,
			"Using DEV:%s, %d Mhz, (%d * 100) kbps, %d dBm, Ant. mode %d\n",
			dev, freq, rate, txpower_dbm, antenna_mode);

	raw_filter = ETH_P_ALL;
	//raw_filter = WMP_PROTO;


	/* Create raw sockets using WMP_PROTO for the ethertype field */
	if ((rx = socket(PF_PACKET, SOCK_RAW, raw_filter)) < 0) {
		perror("Error creating rx socket");
		ret--;
	}
	if ((tx = socket(PF_PACKET, SOCK_RAW, WMP_PROTO)) < 0) {
		perror("Error creating tx socket");
		ret--;
	}

	memset(&ifr, 0, sizeof(struct ifreq));
	memset(&rx_addr, 0, sizeof(rx_addr));
	memset(&tx_addr, 0, sizeof(tx_addr));

	strncpy((char *) ifr.ifr_name, dev, IFNAMSIZ);

	/* Brinf iface up */
	if (ioctl(rx, SIOCGIFFLAGS, &ifr)) {
		perror("ioctl(SIOCSIFFLAGS)");
		ret--;
	}
	ifr.ifr_flags |= (IFF_UP | IFF_RUNNING);
	if (ioctl(rx, SIOCSIFFLAGS, &ifr)) {
		perror("ioctl(SIOCSIFFLAGS)");
		ret--;
	}

	/* Get the Interface Index */
	if ((ioctl(rx, SIOCGIFINDEX, &ifr)) < 0) {
		perror("Error getting Interface index");
		ret--;
	}

	/* Bind the raw sockets to this interface */
	rx_addr.sll_family = AF_PACKET;
	rx_addr.sll_ifindex = ifr.ifr_ifindex;
	rx_addr.sll_protocol = htons(raw_filter);

	tx_addr.sll_family = AF_PACKET;
	tx_addr.sll_ifindex = ifr.ifr_ifindex;
	tx_addr.sll_protocol = htons(WMP_PROTO);

	if ((bind(rx, (struct sockaddr *) &rx_addr, sizeof(struct sockaddr_ll)))
			< 0) {
		perror("rx bind error");
		ret--;
	}

	if ((bind(tx, (struct sockaddr *) &tx_addr, sizeof(struct sockaddr_ll)))
			< 0) {
		perror("tx bind error");
		ret--;
	}

	/* Configure interface */
	ret -= configure_interface(rx, dev);

	/* Get the interface MAC for building the ethernet header */
	if (ioctl(rx, SIOCGIFHWADDR, &ifr) < 0) {
		perror("ioctl(SIOCGIFHWADDR)");
		ret--;
	}

	/* Build ethernet header used for tx */
	ethernet_header = (struct ethhdr *) ethernet_frame;
	memcpy(ethernet_header->h_dest, dst_mac, ETH_ALEN);
	memcpy(ethernet_header->h_source, ifr.ifr_hwaddr.sa_data, ETH_ALEN);
	ethernet_header->h_proto = htons(WMP_PROTO);

	int val = 1;
	setsockopt(tx, SOL_SOCKET, SO_BROADCAST, &val, sizeof(val));

	if (ret == 0) {
		WMP_DBG(LL_COM, "Low level com init ok.\n");
		return 1;
	}

	return 0;
}

int llsend(char * f, int size) {
	unsigned char *p;
	int nbytes;

	p = ethernet_frame + ETH_HLEN;

	/* Copy data */
	memcpy(p, f, size);
	p += size;

	if ((nbytes = sendto(tx, ethernet_frame, p - ethernet_frame, 0, 0, 0)) < 0) {
		perror("sendto()");
	}

	return nbytes;
}

rxInfo llreceive(char *f, int timeout) {
	int r;
	rxInfo ret;
	struct ethhdr * eth_hdr = (struct ethhdr *) f;

	if (timeout > 0) {
		fd_set fd_rx;
		tv.tv_sec = 0;
		tv.tv_usec = 1000 * timeout; /* timeout in ms and not us */
		FD_ZERO(&fd_rx);
		FD_SET(rx, &fd_rx);
		fprintf(stderr,"TO >=  =  OK %d \n",timeout);

		r = select(FD_SETSIZE, &fd_rx, NULL, NULL, &tv);
	} else {
		r = 1;
	}

	if (r) {
		int nbytes = recvfrom(rx, (unsigned char *) f, MTU, 0, 0, 0);
		char c_rate = (*(f + ETH_HLEN + 1)); /* ath5k form */
		ret.proto = eth_hdr->h_proto;
		fprintf(stderr,"TO >=  =  proto %d \n",ret.proto);
		ret.error = 0;
		ret.rate = ath5k_rate[c_rate];
		ret.has_lq = 0;
		ret.size = nbytes - ETH_HLEN;
		memmove(f, f + ETH_HLEN, nbytes - ETH_HLEN);

//		/* calculo RSSI */
//		double dbm = f[0] - 95;
//		double drssi = 4500*sqrt(pow(10,dbm/10));
//
//		drssi = 100*f[0]/75;
//
//		if (drssi > 100){
//			drssi = 100;
//		}
//		/* calculo RSSI */
//
//		f[0] = (char) drssi;


		return ret;
	} else {
		ret.error = 1;
		return ret;
	}
}

int llconfig(unsigned short freq, enum rates rate, unsigned char power,
		enum ath5k_ant_mode antenna_mode) {
	struct ifreq ifr;
	int ret = 0;

	strcpy(ifr.ifr_name, dev);

	/* Configure freq, rate and power*/
	struct ath5k_config_info config_info;

	config_info.frequency = freq;
	config_info.rate = rate;
	config_info.tx_power_dbm = power;
	config_info.antenna_mode = antenna_mode;

	ifr.ifr_data = (void *) &config_info;
	if ((ioctl(rx, SIO_SET_CONFIG, &ifr)) < 0) {
		perror("Error configuring interface");
		ret--;
	}

	wmpSetRate((float) rate / 10.0);

	return ret;
}


int llsetPower(int f){

}


int llsetupModule(char * devi){
	int res = 0;
	if (iw_has_module("ath5k ")){
		fprintf(stderr,"Removing module(s)...");
		res = system("rmmod ath5k");
		fprintf(stderr,"Done\n");
	}

	if (! iw_has_module("ath5k_raw")){
		char module[256];
		fprintf(stderr,"Installing ath5k_raw...");
		snprintf(module, 256, "insmod %s/.rt-wmp/ath5k_raw.ko", getenv("HOME"));
		res = system(module);
		if (res == -1){
			fprintf(stderr,"*** Unable to install module, exiting\n");
			exit(1);
		}else{
			fprintf(stderr,"Done\n");
		}
	}

	fprintf(stderr,"Identifying interface...");
	char dev[10][256];
	int count = iw_enum_devices(dev), i;

	struct ifreq ifr;
	int sock;
	if ((sock = socket(PF_PACKET, SOCK_RAW, ETH_P_ALL)) < 0) {
		perror("Error creating socket, exiting...");
		exit(1);
	}
	int done = 0;
	for(i = 0; i< count; i++){
		strcpy(ifr.ifr_name, dev[i]);
		int disable_ack = 1;
		ifr.ifr_data = (void *) &disable_ack;
		if ((ioctl(sock, SIO_SET_DISABLEACK, &ifr)) < 0) {
			continue ;
		}else{
			fprintf(stderr,"%s\n",dev[i]);
			sprintf(devi,"%s",dev[i]);
			done = 1;
			break;
		}
	}

	if (!done){
		perror("Unable to found ath5k_raw interface, exiting...\n");
		exit(1);
	}
	return 1;
}

/* IW-CODE */
int iw_has_module(char * mod) {
	FILE * f = fopen("/proc/modules","r");
	char text[2000];
	fread(text,2000,1,f);
	fclose(f);
	return (strstr(text,mod) != 0);
}

static inline char * iw_get_ifname(char * name, int nsize, char * buf){
	char * end;
	while (isspace(*buf)){
		buf++;
	}
	end = strrchr(buf, ':');
	if ((end == NULL) || (((end - buf) + 1) > nsize))
		return (NULL);

	memcpy(name, buf, (end - buf));
	name[end - buf] = '\0';
	return (end);
}


int iw_enum_devices(char  a[10][256]) {
	char buff[1024];
	struct ifconf ifc;
	struct ifreq *ifr;
	int i, count = 0;

	FILE * fh = fopen("/proc/net/dev", "r");
	if (fh != NULL) {
		fgets(buff, sizeof(buff), fh);
		fgets(buff, sizeof(buff), fh);

		while (fgets(buff, sizeof(buff), fh)) {
			char name[IFNAMSIZ + 1], *s;

			if ((buff[0] == '\0') || (buff[1] == '\0')){
				continue;
			}
			s = iw_get_ifname(name, sizeof(name), buff);

			if (!s) {
				fprintf(stderr, "Cannot parse /proc/net/dev \n");
			} else {
				strcpy(a[count],name);
				count ++;
			}
		}
		fclose(fh);
		return count;
	} else {
		return 0;
	}
}


