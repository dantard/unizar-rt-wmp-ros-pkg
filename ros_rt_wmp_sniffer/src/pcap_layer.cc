/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: pcap_layer.cc
 *  Authors: Danilo Tardioli
 *  ----------------------------------------------------------------------
 *  Copyright (C) 2000-2012, Universidad de Zaragoza, SPAIN
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

#include <pcap.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <zlib.h>
#include <ctime>
#include <netinet/if_ether.h>
#include <netinet/udp.h>
#include <netinet/ip.h>
#include "pcap_layer.h"
#include "wmp_config.h"

#include "core/include/frames.h"
#include <stdlib.h>
#include <iostream>
#include "misc.h"

static pcap_t *handle;
static struct pcap_pkthdr header;
static int offset;
static bool pcap_keep_running;
static int pfd[2];

std::map<int, int> mac_ip;

int pcap_init(char * dev, int num_nodes) {
	char errbuf[PCAP_ERRBUF_SIZE];
	fprintf(stderr, "Sniffing on %s\n", dev);

	/* Open the session in promiscuous mode */

	handle = pcap_open_live(dev, BUFSIZ, 1, 1000, errbuf);

	if (handle == NULL) {
		fprintf(stderr, "Couldn't open device %s: %s\n", dev, errbuf);
		exit(1);
	}

	pcap_keep_running = true;
	return 0;
}

double int2Rate(unsigned char val) {
	double rate;
	switch (val) {
	case 2:
		rate = 1.0;
		break;
	case 0xb:
		rate = 5.5;
		break;
	case 0x16:
		rate = 11.0;
		break;
	case 0x0c:
		rate = 6.0;
		break;
	case 0x04:
		rate = 2.0;
		break;
	case 0x12:
		rate = 9.0;
		break;
	default:
		rate = val;
	}
	return rate;
}

int decompress(char * buf, int * size) {
	char tmp[2500];
	uLongf len = 2500;
	int res = uncompress((Bytef*) tmp, &len, (Bytef *) buf, *size);
	if (res != Z_OK) {
		fprintf(stderr, "UNZIP ERROR!\n");
		*size = -1;
		return 0;
	}
	*size = len;
	memcpy(buf, tmp, len);
	return 1;
}

static int serial = 100;
int pcap_sniff_packet(char * data, simData_Hdr & sd,
		unsigned long long &time_us, std::map<int, robo_pose_t> & poses) {
	int dl = pcap_datalink(handle);
	char zFrame[2500];
	pcap_keep_running = true;
	while (pcap_keep_running) {
		sd.frame_type = SP_FOREIGN;
		sd.is_wmp = false;
		int f_len = -1;
		char * f_data = NULL;
		int ds_status_offset = 0;

		const u_char * packet = pcap_next(handle, &header);
		if (packet == NULL) {
			return -1;
		}

		int eth_hdr_offset = 0;
		switch (dl) {
		case 1:
			offset = sizeof(ethhdr);
			eth_hdr_offset = 0;
			sd.rate = 0;
			break;

		case 127: {
			ieee80211_radiotap_header * rth =
					(ieee80211_radiotap_header*) packet;
			offset = rth->it_len + 24 + 8; //radiotap+80211
			ds_status_offset = rth->it_len + 1;
			sd.rate = int2Rate(packet[0x11]);
			eth_hdr_offset = rth->it_len + 4;
		}
			break;

		default:
			offset = 0;
			break;
		};

		const char * pkt_p = (char*) packet + offset;

		if (header.len > offset) {

			short protocol_type = ntohs(*(short*) (packet + offset - 2));
			int mac_src = ((packet + eth_hdr_offset)[10] << 8)
					+ ((packet + eth_hdr_offset)[11]);
			int mac_dst = ((packet + eth_hdr_offset)[4] << 8)
					+ ((packet + eth_hdr_offset)[5]);
			struct udphdr * udph = (struct udphdr *) (pkt_p
					+ sizeof(struct iphdr));
			struct iphdr * iph = (struct iphdr *) pkt_p;

			if (protocol_type == 0x0800 && iph->protocol == IPPROTO_UDP
					&& htons(udph->dest) == 0x6969) {
				sd.is_wmp = true;
				if ((dl == 1
						|| (dl == 127 && (packet[ds_status_offset] & 3)) == 1)) {
					sd.frame_type = SP_LUS_WMP_FRAME;
				} else {
					sd.frame_type = SP_LUS_WMP_FRAME; //_DUP;
				}
				f_len = htons(udph->len) - sizeof(struct udphdr);
				f_data = (char *) (pkt_p + sizeof(struct udphdr)
						+ sizeof(struct iphdr));
				sd.is_wmp = true;
				sd.frame_type = SP_LUS_WMP_FRAME;
			} else if (protocol_type == 0x0800 && iph->protocol == IPPROTO_UDP
					&& htons(udph->dest) == 0x6868) {
				if (mac_ip.find(mac_src) == mac_ip.end()
						|| mac_ip.find(mac_dst) == mac_ip.end()) {
					continue;
				}
				f_data = (char *) (pkt_p + sizeof(struct udphdr)
						+ sizeof(struct iphdr));
				wmpFrame * p = (wmpFrame *) f_data;

				p->hdr.type = MESSAGE;
				p->hdr.from = mac_ip[mac_src];
				p->hdr.to = mac_ip[mac_dst];

				f_len =
						header.len
								- (offset + sizeof(struct udphdr)
										+ sizeof(struct iphdr));
				sd.is_wmp = true;
				sd.frame_type = SP_LUS_WMP_FRAME;
			} else if (protocol_type == 0x0800 && iph->protocol == IPPROTO_UDP) {
				if (mac_ip.find(mac_src) == mac_ip.end()
						|| mac_ip.find(mac_dst) == mac_ip.end()) {
					continue;
				}

				f_data = (char *) (pkt_p + sizeof(struct iphdr)
						+ sizeof(struct udphdr));
				wmpFrame * p = (wmpFrame *) f_data;

				p->hdr.from = mac_ip[mac_src];
				p->hdr.to = mac_ip[mac_dst];
				p->hdr.type = MESSAGE;
				p->msg.len =
						header.len
								- (offset + sizeof(struct udphdr)
										+ sizeof(struct iphdr));
				f_len = htons(udph->len) - sizeof(struct udphdr);

				sd.is_wmp = true;
				sd.frame_type = SP_LUS_WMP_FRAME;

			} else if (protocol_type == 0x0800) {
				if (mac_ip.find(mac_src) == mac_ip.end()
						|| mac_ip.find(mac_dst) == mac_ip.end()) {
					continue;
				}
				f_data = (char *) (pkt_p + sizeof(struct iphdr));
				wmpFrame * p = (wmpFrame *) f_data;

				p->hdr.from = mac_ip[mac_src];
				p->hdr.to = mac_ip[mac_dst];

				p->hdr.type = MESSAGE;
				p->hdr.serial = serial++;
				p->msg.len = header.len - (offset + sizeof(struct iphdr));
				p->msg.src = p->hdr.from;
				p->msg.dest = p->hdr.to;
				p->hdr.retries = 0;
				p->msg.msg_hash = serial;

				f_len = header.len - (offset + sizeof(struct iphdr));
				if (f_len < sizeof(wmpFrame)) {
					f_len = sizeof(wmpFrame);
				}
				sd.is_wmp = true;
				sd.frame_type = SP_LUS_WMP_FRAME;

			} else if (protocol_type == 0x6969 || protocol_type == 0x6970
					|| protocol_type == 0x6971) {

				f_data = (char*) &packet[offset];
				f_len = header.len - offset;
				sd.proto = protocol_type;
				if (protocol_type == 0x6970) {

					memcpy(zFrame, f_data, f_len);
					decompress(zFrame, &f_len);
					f_data = zFrame;

				} else if (protocol_type == 0x6971) {
					memcpy(zFrame, f_data, f_len);
					f_data = zFrame;
				}
				sd.frame_type = SP_LUS_WMP_FRAME;
				sd.is_wmp = true;
			} else {
				sd.is_wmp = false;
			}
		} else {
			sd.is_wmp = false;
		}
		time_us = header.ts.tv_sec;
		time_us = time_us * 1000000 + header.ts.tv_usec;
		sd.time = time_us;
		sd.data_src = 32;

		if (sd.is_wmp) {
			wmpFrame * p = (wmpFrame*) f_data;
			memcpy(data, f_data, f_len);
			poses[p->hdr.from].reached = true;
			poses[p->hdr.from].pose_is_valid = false;
			return f_len;
		} else {
			return header.len;
		}
	}
	return 0;
}

void pcap_layer_close() {
	pcap_keep_running = false;
}
