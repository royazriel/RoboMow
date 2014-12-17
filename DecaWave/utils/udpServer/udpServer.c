/*
    Simple udp server
    Silver Moon (m00n.silv3r@gmail.com)
*/
#include<stdio.h> //printf
#include<string.h> //memset
#include<stdlib.h> //exit(0);
#include<arpa/inet.h>
#include<sys/socket.h>
 
#define BUFLEN 512  //Max length of buffer
#define PORT 40010   //The port on which to listen for incoming data
 
void die(char *s)
{
    perror(s);
    exit(1);
}

void swap4Bytes( unsigned char* buf )
{
	unsigned char tmp = buf[0];
	buf[0] = buf[3];
	buf[3]= tmp;
	tmp = buf[1];
	buf[1] = buf[2];
	buf[2] = tmp;
}
 
int main(void)
{
    struct sockaddr_in si_me, si_other;
     
    int s, i, slen = sizeof(si_other) , recv_len;
    char buf[BUFLEN];
    int id[2];
    float range[2];
     
    //create a UDP socket
    if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        die("socket");
    }
     
    // zero out the structure
    memset((char *) &si_me, 0, sizeof(si_me));
     
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(PORT);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);
     
    //bind socket to port
    if( bind(s , (struct sockaddr*)&si_me, sizeof(si_me) ) == -1)
    {
        die("bind");
    }
    
    printf("Waiting for data...");
    fflush(stdout);
    //keep listening for data
    while(1)
    {
        //try to receive some data, this is a blocking call
        if ((recv_len = recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen)) == -1)
        {
            die("recvfrom()");
        }
        swap4Bytes( buf );
        swap4Bytes( buf + 4 );
        if( recv_len == 8 )
        {
	    if( *(unsigned int*)buf == 1)
	    {
            	id[0] = *(unsigned int*)buf;
            	range[0] = *(float*)(buf+4);
	    }
	    else
	    {
 		id[1] = *(unsigned int*)buf;
                range[1] = *(float*)(buf+4);
	    }

            printf( "\033cid=%d range=%4.2f | id=%d range=%4.2f\n",id[0],range[0],id[1],range[1]);
        }
    }
 
    close(s);
    return 0;
}
