/*
 * AfUnix.c
 *
 *  Created on: Mar 14, 2015
 *      Author: roy
 */


#include<stdio.h> //printf
#include<string.h> //memset
#include<stdlib.h> //exit(0);
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>


#include "deca_device_api.h"
#include "AfUnix.h"
#include "spiDriver.h"

static struct sockaddr_un s_socket;
static int 	s_fd;


int AfUnixClinetConnect()
{
	int rc;
    if (( s_fd=socket(AF_UNIX, SOCK_STREAM, 0)) == -1)
    {
    	PINFO("can not open socket");
    	return -1;
    }

    memset((char *) &s_socket, 0, sizeof(s_socket));
    s_socket.sun_family = AF_UNIX;
    strcpy(s_socket.sun_path, SERVER_PATH);
    rc = connect(s_fd, (struct sockaddr *)&s_socket, SUN_LEN(&s_socket));
	if (rc < 0)
	{
		PINFO("connect() failed");
		return -1;
	}
	return 0;
}

int AfUnixClinetSendReportTOF( unsigned char* buffer, int size)
{
	if( send( s_fd, buffer, size , MSG_NOSIGNAL )==-1)
	{
		PINFO("AfUnixClinetSendReportTOF() Error");
		return -1;
	}
	return 0;
}

void AfUnixClinetCloseSocket()
{
	close(s_fd);
}
