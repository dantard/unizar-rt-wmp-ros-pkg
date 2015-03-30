/*
 * types.h
 *
 *  Created on: Mar 24, 2015
 *      Author: danilo
 */

#ifndef TYPES_H_
#define TYPES_H_

struct WRPPacketInfo {
	int protocol;
	char rssi;
	int channel;
	int rate;
	int noise_dbm;
	int antenna;
	int pwr;
	int valid;
	int foreign;
	int delay;
};



#endif /* TYPES_H_ */
