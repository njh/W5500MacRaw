/*
 * Linux programme to test sending and receiving of Ethernet frames
 *
 *
 * Copyright (c) 2016, Nicholas Humfrey
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <net/if.h>
#include <netinet/ether.h>

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <linux/if.h>
#include <linux/if_packet.h>


uint8_t our_mac[6] = {0x1e, 0x65, 0x55, 0x3c, 0x84, 0xc3};
uint8_t their_mac[6] = {0x52, 0xff, 0xee, 0x1b, 0x44, 0x55};
uint16_t eth_type = 0x88b5;
const char *ifname = "eth1";
int ifindex = -1;


static int open_socket() {
    ifindex = if_nametoindex(ifname);
    if (ifindex <= 0) {
        perror("if_nametoindex");
        exit(-1);
    }

    int sockfd = socket(PF_PACKET, SOCK_RAW, htons(eth_type));
    if (sockfd == -1) {
        perror("socket(PF_PACKET)");
        exit(-1);
    }

    /* Set interface to promiscuous mode */
    struct ifreq ifopts;
    int sockopt = 0;
    strncpy(ifopts.ifr_name, ifname, IFNAMSIZ-1);
    ioctl(sockfd, SIOCGIFFLAGS, &ifopts);
    ifopts.ifr_flags |= IFF_PROMISC;
    ioctl(sockfd, SIOCSIFFLAGS, &ifopts);

    /* Allow the socket to be reused */
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &sockopt, sizeof sockopt) == -1) {
        perror("setsockopt(SO_REUSEADDR)");
        close(sockfd);
        exit(-1);
    }

    /* Bind to device */
    if (setsockopt(sockfd, SOL_SOCKET, SO_BINDTODEVICE, ifname, IFNAMSIZ-1) == -1)	{
        perror("setsockopt(SO_BINDTODEVICE)");
        close(sockfd);
        exit(-1);
    }
    
    return sockfd;
}


static void send_frame(int sockfd, const uint8_t *data, uint16_t datalen)
{
    struct sockaddr_ll socket_address;

    /* Index of the network device */
    socket_address.sll_ifindex = ifindex;

    /* Address length*/
    socket_address.sll_halen = ETH_ALEN;

    /* Destination MAC */
    memcpy(&socket_address.sll_addr, their_mac, 6);

    /* Send packet */
    int result = sendto(sockfd, data, datalen, 0, (struct sockaddr*)&socket_address, sizeof(struct sockaddr_ll));
    if (result <= 0) {
        perror("sendto");
    }
}

static uint16_t read_frame(int sockfd, uint8_t *buffer, uint16_t bufsize)
{

    do {
        int result = recv(sockfd, buffer, bufsize, 0);
        if (result <= 0) {
            if (errno != EAGAIN)
                perror("Failed to read");
            return 0;
        }
        
        if (buffer[12] == 0x88 && buffer[13] == 0xB5 &&
            memcmp(&buffer[0], our_mac, 6) == 0)
        {
            return result;
        }
    } while (1);
}


static unsigned long long get_ms()
{
    struct timespec ts;
    timespec_get(&ts, TIME_UTC);
    return (unsigned long long)(ts.tv_sec * 1000) + ((unsigned long long)ts.tv_nsec / 1000000);
};

int main() {
    int sockfd = open_socket();

    uint8_t buffer[1500];
    uint8_t counter = 0;
    for(int i=0; i<5000; i++) {
        printf("i=%d  counter=%d\n", i, counter);
    
        memcpy(&buffer[0], their_mac, 6);
        memcpy(&buffer[6], our_mac, 6);
        buffer[12] = 0x88;
        buffer[13] = 0xB5;
        buffer[14] = 0x00;
        buffer[15] = counter;
        send_frame(sockfd, buffer, 100);
        
        unsigned long long sent = get_ms();
        
        int len = read_frame(sockfd, buffer, sizeof(buffer));
        
        if (len) {
            int diff = get_ms() - sent;
            
            printf("  %dms\n", diff);
            
            if (buffer[14] != counter) {
                printf("  Arduino counter=%d\n", buffer[14]);
            }
            if (buffer[15] != counter) {
                printf("  Received counter=%d\n", buffer[14]);
            }
        } else {
            printf("Read fail.\n");
        }
        
        counter++;
    }

    close(sockfd);
}

