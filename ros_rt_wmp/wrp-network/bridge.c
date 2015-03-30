#include <stdio.h>
#include <stdarg.h>
#include <iw.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/if.h>
#include <netpacket/packet.h>
#include <net/ethernet.h>     /* the L2 protocols */
#include <asm/types.h>
#include <time.h>
#include <netinet/if_ether.h>
#include <netinet/in.h>

int iw(int num_param, ...) {
	int i;
	char * params[num_param + 1];

	va_list valist;
	va_start(valist, num_param);
	params[0] = malloc(5);
	sprintf(params[0], "%s", "iw");
	for (i = 1; i <= num_param; i++) {
		params[i] = malloc(64);
		sprintf(params[i], "%s", va_arg(valist, char*));
	}
	va_end(valist);
	return qmain(num_param + 1, params);
}

int set_network(char * interface, char * name, int freq, int rate) {
	char f[64], r[64];
	sprintf(f, "%d", freq);
	sprintf(r, "%d", rate);

	printf("Leaving ibss...\n");
	int res = iw(4, "dev", interface, "ibss", "leave");
	printf("Joining %s ibss at frequency %d and rate %d Mbps\n", name, freq, rate);
	res += iw(9, "dev", interface, "ibss", "join", name, f, "fixed-freq", "mcast-rate", r);
	return res;
}

int delete_interface(char * interface){
	printf("Deleting interface %s...\n", interface);
	return iw(3, "dev", interface, "del");
}

int create_monitor(char * interface, char *name){
	printf("Creating monitor interface...\n");
	return  iw(7, "dev", interface, "interface", "add" , name, "type", "monitor");
}

int set_ip(char *iface_name, const char *ip_addr) {
	if (!iface_name)
		return -1;

	int sockfd;
	struct ifreq ifr;
	struct sockaddr_in sin;

	sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (sockfd == -1) {
		fprintf(stderr, "Setting IP: could not get socket\n");
		return -1;
	}
	strncpy(ifr.ifr_name, iface_name, IFNAMSIZ);
	if (ioctl(sockfd, SIOCGIFFLAGS, &ifr) < 0) {
		fprintf(stderr, "Setting IP: ifdown failed ");
		perror(ifr.ifr_name);
		return -1;
	}
#ifdef ifr_flags
# define IRFFLAGS       ifr_flags
#else
# define IRFFLAGS       ifr_flagshigh
#endif
	if (!(ifr.IRFFLAGS & IFF_UP)) {
		fprintf(stdout, "Setting IP: device is currently down..setting up -- %u\n", ifr.IRFFLAGS);
		ifr.IRFFLAGS |= IFF_UP;
		if (ioctl(sockfd, SIOCSIFFLAGS, &ifr) < 0) {
			fprintf(stderr, "Setting IP: ifup failed ");
			perror(ifr.ifr_name);
			return -1;
		}
	}

	if (ip_addr == 0) {
		return 0;
	}

	sin.sin_family = AF_INET;
	inet_aton(ip_addr, (struct in_addr *) &sin.sin_addr.s_addr);
	memcpy(&ifr.ifr_addr, &sin, sizeof(struct sockaddr));
	if (ioctl(sockfd, SIOCSIFADDR, &ifr) < 0) {
		fprintf(stderr, "Setting IP: cannot set IP address ");
		perror(ifr.ifr_name);
		return -1;
	}
#undef IRFFLAGS
	return 0;
}

