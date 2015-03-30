
#ifndef BRIDGE_H
#define BRIDGE_H

int iw(int num_param, ...);
int set_network(char * interface, char * name, int freq, int rate);
int delete_interface(char * interface);
int create_monitor(char * interface, char *name);
int set_ip(char *iface_name, const char *ip_addr);


#endif
