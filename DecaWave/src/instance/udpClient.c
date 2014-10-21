/*
 * udpClient.c
 *
 *  Created on: Oct 21, 2014
 *      Author: roy
 */

#include<stdio.h> //printf
#include<string.h> //memset
#include<stdlib.h> //exit(0);
#include<arpa/inet.h>
#include<sys/socket.h>

#include "deca_device_api.h"
#include "udpClient.h"
#include "spiDriver.h"

#define BUFLEN 512  //Max length of buffer

static struct sockaddr_in s_socket;
static int 	s_fd;

int UdpclinetConnect( const char * serverIp, int port )
{
    if (( s_fd=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
    	PINFO("can not open socket");
    	return -1;
    }

    memset((char *) &s_socket, 0, sizeof(s_socket));
    s_socket.sin_family = AF_INET;
    s_socket.sin_port = htons(port);

    if (inet_aton( serverIp , &s_socket.sin_addr) == 0)
    {
        PINFO( "inet_aton() failed\n");
        return -1;
    }
    return 0;
}

int UdpClinetSendReportTOF( unsigned char* buffer, int size)
{
	if( sendto( s_fd, buffer, size , 0 , (struct sockaddr *) &s_socket, sizeof(s_socket ) )==-1)
	{
		PINFO("UdpClinetSendReportTOF()");
		return -1;
	}
	return 0;
}

int UdpClinetGetReportTOF( unsigned char* buffer, int* size)
{
	unsigned int slen = sizeof(s_socket );
	if (recvfrom(s_fd, buffer, BUFLEN, 0, (struct sockaddr *) &s_socket, &slen ) == -1)
	{
		PINFO("UdpClinetGetReportTOF()");
		return -1;
	}
	return 0;
}

void UdpClinetCloseSocket()
{
	close(s_fd);
}
