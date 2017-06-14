/*                Copyright (C) 2000-2017, Danilo Tardioli               *
 *           Centro Universitario de la Defensa Zaragoza, SPAIN          *
 *
 *  Contact Addresses: Danilo Tardioli                   dantard@unizar.es
 *
 *  This is free software; you can  redistribute it and/or  modify it
 *  under the terms of the GNU General Public License  as published by the
 *  Free Software Foundation;  either  version 2, or (at  your option) any
 *  later version.
 *
 *  This software is distributed  in the  hope  that  it will be   useful,
 *  but WITHOUT  ANY  WARRANTY;   without  even the implied   warranty  of
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


#include "raw_comm.h"
#include "radiotap_iter.h"
#include "radiotap.h"
#include <time.h>

int raw_init(char * ifName, int protocol, Sock * sock){

    int sockfd = -1, sockopt;
    struct ifreq ifopts, if_ip;
    sock->eh = (struct ethhdr *) sock->txb;
    sock->eh->h_proto = protocol;

    memset(&if_ip, 0, sizeof(struct ifreq));

    /* Open PF_PACKET socket, listening for EtherType ETHER_TYPE */

    //if ((sockfd = socket(PF_PACKET, SOCK_RAW, htons(protocol))) == -1) {
    if ((sockfd = socket(PF_PACKET, SOCK_RAW, htons(protocol))) == -1) {
        perror("listener: socket");
        return -1;
    }

    strncpy(ifopts.ifr_name, ifName, IFNAMSIZ-1);
    ioctl(sockfd, SIOCGIFFLAGS, &ifopts);
    ifopts.ifr_flags |= IFF_PROMISC;
    ioctl(sockfd, SIOCSIFFLAGS, &ifopts);

    /* Allow the socket to be reused - incase connection is closed prematurely */
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &sockopt, sizeof sockopt) == -1) {
        perror("setsockopt");
        close(sockfd);
        return -1;
    }

    /* Get interface index in ifopts.ifr_ifindex */
    if (ioctl(sockfd, SIOCGIFINDEX, &ifopts) == -1) {
        perror("SIOCGIFINDEX");
        return -1;
    }

    int ifindex = ifopts.ifr_ifindex;

    /* Bind to device */
    //    if (setsockopt(sockfd, SOL_SOCKET, SO_BINDTODEVICE, ifName, IFNAMSIZ-1) == -1)	{
    //        perror("SO_BINDTODEVICE");
    //        close(sockfd);
    //        return -1;
    //    }

    /* Bind socket to interface to receive frame ONLY from that interface */
    struct sockaddr_ll sa;
    sa.sll_family = AF_PACKET;
    sa.sll_ifindex = ifindex;
    sa.sll_protocol = htons(protocol);
    if ((bind(sockfd, (struct sockaddr *) &sa, sizeof(sa))) == -1) {
        perror("SO_BINDTODEVICE: ");
        return -1;
    }

    /* Get iface HW address */
    if (ioctl(sockfd, SIOCGIFHWADDR, &ifopts) < 0){
        perror("SIOCGIFHWADDR: ");
        return -1;
    }

    /* Prepare source address */
    int i;
    for (i = 0; i< 6; i++){
        sock->source[i] = ifopts.ifr_hwaddr.sa_data[i];
    }
    memcpy((void *) sock->eh->h_source, (void*) sock->source, ETH_ALEN);

    /* Prepare destination */
    sock->destination.sll_family = PF_PACKET;
    sock->destination.sll_ifindex = ifindex;
    sock->destination.sll_halen = ETH_ALEN;
    sock->destination.sll_protocol = htons(protocol);
    sock->sock = sockfd;

    return sockfd;
}


int raw_receive(Sock * sock, char * buf, int BUF_SIZ){
    return recvfrom(sock->sock, buf, BUF_SIZ, 0, NULL, NULL);
}

int raw_receive_timed(Sock * sock, char * buf, int BUF_SIZ, int timeout){

    int r;
    static struct timeval tv;

    if (timeout > 0) {
        fd_set fd_rx;
        tv.tv_sec = 0;
        tv.tv_usec = 1000 * timeout; /* timeout in ms and not us */
        FD_ZERO(&fd_rx);
        FD_SET(sock->sock, &fd_rx);
        r = select(FD_SETSIZE, &fd_rx, NULL, NULL, &tv);
    }else{
        r = 1;
    }

    if (r > 0){
        int res = raw_receive(sock,buf, BUF_SIZ);
        return res;
    }else{
        return -1;
    }
}

int raw_send(Sock * sock, char * buf, int BUF_SIZ, unsigned char * address){
    memcpy((void *) sock->eh->h_dest, (void*) address, ETH_ALEN);
    memcpy(sock->txb + ETH_HLEN, buf, BUF_SIZ);
    sock->destination.sll_pkttype = PACKET_BROADCAST;
    return sendto(sock->sock, sock->txb, BUF_SIZ + ETH_HLEN, 0, (struct sockaddr*) &sock->destination, sizeof(sock->destination));
}

int raw_send_broadcast(Sock * sock, char * buf, int BUF_SIZ){
    return raw_send(sock, buf, BUF_SIZ, (char []){0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF });
}

rx_info_t raw_parse_radiotap(Sock * sock, char * buff, int len){

    int radiotap_len = 0;
    rx_info_t rxinfo;
    memset(&rxinfo, 0, sizeof(rx_info_t));

    if (len > ETH_HLEN && memcmp(&buff[6], sock->source, 6) == 0){
        rxinfo.mine = 1;
        rxinfo.data = &buff[ETH_HLEN];
        rxinfo.len = len - ETH_HLEN;
        rxinfo.proto = *((int *) &buff[12]);
        return rxinfo;
    }

    struct ieee80211_radiotap_header * radiotap_hdr = (struct ieee80211_radiotap_header *) buff;
    struct ieee80211_radiotap_iterator iterator;
    int ret = ieee80211_radiotap_iterator_init(&iterator, radiotap_hdr, radiotap_hdr->it_len, 0);

    if (ret == 0){
        while (ret == 0) {

            ret = ieee80211_radiotap_iterator_next(&iterator);

            /* see if this argument is something we can use */
            switch (iterator.this_arg_index) {

            /*
             * You must take care when dereferencing iterator.this_arg
             * for multibyte types... the pointer is not aligned.  Use
             * get_unaligned((type *)iterator.this_arg) to dereference
             * iterator.this_arg for type "type" safely on all arches.
             */

            case IEEE80211_RADIOTAP_RATE:
                rxinfo.rate = ((double)  (*iterator.this_arg) * 0.5);
                break;
            case IEEE80211_RADIOTAP_DBM_ANTSIGNAL:
                rxinfo.rssi = (*iterator.this_arg);
                break;
            case IEEE80211_RADIOTAP_ANTENNA:
                rxinfo.antenna = (*iterator.this_arg);
                break;
            case IEEE80211_RADIOTAP_DBM_TX_POWER:
                break;
            case IEEE80211_RADIOTAP_DATA_RETRIES:
                rxinfo.retries = (*iterator.this_arg);
                break;
            case IEEE80211_RADIOTAP_DB_ANTNOISE:
                //noise_dbm = *iterator.this_arg;
                break;
            case  IEEE80211_RADIOTAP_CHANNEL:
                rxinfo.channel = (* (unsigned short*) iterator.this_arg);
                break;
            default:
                break;
            }
        } /* while more rt headers */

        if (radiotap_hdr->it_len >= 0x24){
            rxinfo.mac_time = *((unsigned long long int *) &buff[0x10]);
        }
        radiotap_len = radiotap_hdr->it_len;
    }

    ieee80211_frame * frame_80211 = (ieee80211_frame*) &buff[radiotap_len];

    rxinfo.ieee80211_duration = frame_80211->duration;
    rxinfo.ieee80211_seq = frame_80211->seq;
    rxinfo.len = len - radiotap_len - sizeof(ieee80211_frame);

    sprintf(rxinfo.addr1,"%02x%02x%02x%02x%02x%02x", frame_80211->addr1[0],frame_80211->addr1[1],frame_80211->addr1[2],frame_80211->addr1[3],frame_80211->addr1[4],frame_80211->addr1[5]);
    sprintf(rxinfo.addr2,"%02x%02x%02x%02x%02x%02x", frame_80211->addr2[0],frame_80211->addr2[1],frame_80211->addr2[2],frame_80211->addr2[3],frame_80211->addr2[4],frame_80211->addr2[5]);
    sprintf(rxinfo.addr3,"%02x%02x%02x%02x%02x%02x", frame_80211->addr3[0],frame_80211->addr3[1],frame_80211->addr3[2],frame_80211->addr3[3],frame_80211->addr3[4],frame_80211->addr3[5]);

    int qos_padding;
    unsigned char ieee80211_type = ((unsigned char)buff[radiotap_len]);
    rxinfo.ieee80211_type = ieee80211_type;
    /* is a DATA frame */
    if (ieee80211_type == 0x08){
        qos_padding = 0;
    }
    /* is a QOS frame */
    else if (ieee80211_type == 0x88){
        qos_padding = 2;
    }
    /* is a Beacon frame */
    else if (ieee80211_type == 0x80){
        rxinfo.data = (char*) (frame_80211->data);
        rxinfo.ieee80211_is_beacon = 1;
        return rxinfo;
    }else{
        rxinfo.error = 1;
        return rxinfo;
    }

    unsigned char flags = buff[radiotap_len + 1];
    rxinfo.ieee80211_to_ds = (flags & 0x01) > 0;
    rxinfo.ieee80211_is_retry = (flags & 0x08) > 0;

    ieee80211_llc_snap_header * snap = (ieee80211_llc_snap_header*) (frame_80211->data + qos_padding);
    rxinfo.data = (char*) (frame_80211->data + sizeof(ieee80211_llc_snap_header) + qos_padding);
    rxinfo.len = rxinfo.len - sizeof(ieee80211_llc_snap_header) - qos_padding;

    rxinfo.snap_type = ntohs(snap->ethertype);
    rxinfo.proto = snap->ethertype;

    if (rxinfo.snap_type == 0x0800){

        rxinfo.is_ip = 1;
        iphdr * ip = (iphdr*) (frame_80211->data + sizeof(ieee80211_llc_snap_header) + qos_padding);
        rxinfo.data = (char*) (frame_80211->data + sizeof(ieee80211_llc_snap_header) + sizeof(iphdr) + qos_padding);
        rxinfo.len = rxinfo.len - sizeof(iphdr);

        if (ip->protocol == 0x11){
            rxinfo.is_udp = 1;
            udphdr * udp = (udphdr *) (frame_80211->data + sizeof(ieee80211_llc_snap_header) + sizeof(iphdr) + qos_padding);
            rxinfo.udp_port = ntohs(udp->dest);
            rxinfo.data = (char*) (frame_80211->data + sizeof(ieee80211_llc_snap_header) + sizeof(iphdr) + sizeof(udphdr) + qos_padding);
            rxinfo.len = rxinfo.len - sizeof(udphdr);
        }
    }
    rxinfo.len = rxinfo.len;// - 4;
    rxinfo.error = 0;
    return rxinfo;
}

rx_info_t raw_receive_timed_info(Sock * sock, char * buffer, int timeout_ms){
    int len = raw_receive_timed(sock, sock->rxb, sizeof(sock->rxb), timeout_ms);

    if (len > 0){
        rx_info_t rt = raw_parse_radiotap(sock, sock->rxb, len);
        if (!rt.error){
            memcpy(buffer,rt.data,rt.len);
        }
        return rt;
    }else{
        rx_info_t rt;
        rt.error = 1;
        rt.timeout = (len == -1);
        return rt;
    }
}

void raw_close(Sock * sock){
    close(sock->sock);
}
