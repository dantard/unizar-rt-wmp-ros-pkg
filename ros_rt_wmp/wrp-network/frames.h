/*
 * frames.h
 *
 *  Created on: Mar 23, 2015
 *      Author: danilo
 */

#ifndef FRAMES_H_
#define FRAMES_H_

struct __attribute__((__packed__)) ieee80211_frame {
	unsigned short fcs;
	unsigned short duration;
	unsigned char dst[6];
	unsigned char tx[6];
	unsigned char bssid[6];
	unsigned short frag_seq;
};

struct __attribute__((__packed__)) llc {
	unsigned char dsnap;
	unsigned char ssnap;
	unsigned char cf;
	unsigned char org_code[3];
	unsigned short proto;
};

struct __attribute__((__packed__)) L1Header {
	unsigned char from;
	unsigned char to;
	unsigned int len;
};

struct __attribute__((__packed__)) L2Header {
	unsigned char src;
	unsigned char dest;
	unsigned char priority;
	unsigned char port;
};


#endif /* FRAMES_H_ */
