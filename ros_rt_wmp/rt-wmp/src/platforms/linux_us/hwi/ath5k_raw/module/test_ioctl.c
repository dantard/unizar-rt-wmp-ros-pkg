#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/if.h>
#include "ath5k_interface.h"

int config(int sock, struct ifreq *ifr, unsigned int freq, int rate, unsigned char txpowerdbm, int antmode)
{
	struct ath5k_config_info config_info;

	config_info.frequency = freq;
	config_info.rate = rate;
	config_info.tx_power_dbm = txpowerdbm;
	config_info.antenna_mode = antmode;

	ifr->ifr_data = (void *)&config_info;

	if((ioctl(sock, SIO_SET_CONFIG, ifr)) < 0)
	{
		perror("Error setting config");
		exit(1);
	}
		
	return 0;
}

int set_debug(int sock, struct ifreq *ifr, unsigned int debug)
{
	ifr->ifr_data = (void *)&debug;

	if((ioctl(sock, SIO_SET_DEBUG, ifr)) < 0)
	{
		perror("Error setting debug level");
		exit(1);
	}

	return 0;
}

int config_disable_ack(int sock, struct ifreq *ifr, int disable)
{
	ifr->ifr_data = (void *)&disable;

	if((ioctl(sock, SIO_SET_DISABLEACK, ifr)) < 0)
	{
		perror("Error setting disable ack");
		exit(1);
	}

	return 0;
}

int config_tx_control(int sock, struct ifreq *ifr, unsigned int count, bool wait_for_ack, bool use_short_preamble)
{
	struct ath5k_txcontrol_info info;

	info.count = count;
	info.wait_for_ack = wait_for_ack;
	info.use_short_preamble = use_short_preamble;

	ifr->ifr_data = (void *)&info;

	if((ioctl(sock, SIO_SET_TXCONTROL, ifr)) < 0)
	{
		perror("Error setting tx control");
		exit(1);
	}

	return 0;
}

int set_rxfilter(int sock, struct ifreq *ifr, int broad, int con, int prom)
{
	struct ath5k_rxfilter_info info;

	info.broadcast = broad;
	info.control = con;
	info.promisc = prom;

	ifr->ifr_data = (void *)&info;

	if((ioctl(sock, SIO_SET_RXFILTER, ifr)) < 0)
	{
		perror("Error setting tx control");
		exit(1);
	}

	return 0;


}

void usage()
{
	printf("Usage: ./athraw_config <interface>\n\n");
	exit(0);
}

int main(int argc, char *argv[])
{	
	int sock;
	struct ifreq ifr;

	if (argc < 4)
		usage();

	/* Create socket */
	if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
	{
		perror("Error creating socket");
		exit(1);
	}	
	strcpy(ifr.ifr_name,  argv[1]);

	if (strcmp(argv[2], "debug") == 0)
	{
		char *final;
		int base = 16;
		unsigned int debug = strtoul(argv[3], &final, base);

		printf("Setting debug mask %08x\n", debug);
		set_debug(sock, &ifr, debug);
	}
	else if (strcmp(argv[2], "rxfilter") == 0)
	{
		int broad = atoi(argv[3]);
		int control = atoi(argv[4]);
		int promisc = atoi(argv[5]);

		printf("Setting rx filter. broadcast %d, control %d, promisc %d\n", broad, control, promisc);
		set_rxfilter(sock, &ifr, broad, control, promisc);
	}
	else if (strcmp(argv[2], "disableack") == 0)
	{
		int disable = atoi(argv[3]);

		printf("Disable ack %d\n", disable);
		config_disable_ack(sock, &ifr, disable);
	}
	else if (strcmp(argv[2], "config") == 0)
	{
		int freq = atoi(argv[3]);
		int rate = atoi(argv[4]);
		int txpower = atoi(argv[5]);
		int antmode = atoi(argv[6]);

		printf("Setting freq %d, rate %d, power %d, ant. mode %d\n", freq, rate, txpower, antmode);
		config(sock, &ifr, freq, rate, txpower, antmode);
	}
	else if (strcmp(argv[2], "txcontrol") == 0)
	{
		int count = atoi(argv[3]);
		int wait_ack = atoi(argv[4]);
		int short_pre = atoi(argv[5]);

		printf("Setting tx control. Count %d, wait ack %d, short preamble %d\n", count,wait_ack,short_pre);
		config_tx_control(sock, &ifr, count,wait_ack,short_pre);
	}
}
