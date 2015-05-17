/**************************************************************************/
/* This example program provides code for a server application that uses     */
/* AF_UNIX address family                                                 */
/**************************************************************************/

/**************************************************************************/
/* Header files needed for this sample program                            */
/**************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>


/**************************************************************************/
/* Constants used by this program                                         */
/**************************************************************************/
#define MAGIC_NO	0xFACEFEED
#define SERVER_PATH     "/var/run/uwb"
#define FALSE              0

typedef struct __Ranges
{
       int   magic_no;
       struct timeval time;
       int   anchorId;
       float range;
}__attribute__ ((__packed__)) Ranges;

FILE * csv;

void signal_callback_handler(int signum)
{
   printf("Caught signal %d\n",signum);
   // Cleanup and close up stuff her
   // Terminate program
   close(csv);
   exit(signum);
}


void main()
{	    
	int id[2];
	float range[2];
	/***********************************************************************/
	/* Variable and structure definitions.                                 */
	/***********************************************************************/
	int    sd=-1, sd2=-1;
	int    rc, length;
	Ranges result;
	struct sockaddr_un serveraddr;
    	struct tm* ptm;
    	char timeString[40];
	int fd_file;
	/* Create output file descriptor */
	
  	csv = fopen("/home/root/decaRange.csv", "w");
	// Register signal and signal handler
	signal(SIGINT, signal_callback_handler);

	/***********************************************************************/
	/* A do/while(FALSE) loop is used to make error cleanup easier.  The   */
	/* close() of each of the socket descriptors is only done once at the  */
	/* very end of the program.                                            */
	/***********************************************************************/

	/********************************************************************/
	/* The socket() function returns a socket descriptor, which represents   */
	/* an endpoint.  The statement also identifies that the UNIX        */
	/* address family with the stream transport (SOCK_STREAM) will be   */
	/* used for this socket.                                            */
	/********************************************************************/
	sd = socket(AF_UNIX, SOCK_STREAM, 0);
	if (sd < 0)
	{
		perror("socket() failed");
	}

	/********************************************************************/
	/* After the socket descriptor is created, a bind() function gets a */
	/* unique name for the socket.                                      */
	/********************************************************************/
	memset(&serveraddr, 0, sizeof(serveraddr));
	serveraddr.sun_family = AF_UNIX;
	strcpy(serveraddr.sun_path, SERVER_PATH);

	rc = bind(sd, (struct sockaddr *)&serveraddr, SUN_LEN(&serveraddr));
	if (rc < 0)
	{
 		perror("bind() failed");
	}

	/********************************************************************/
	/* The listen() function allows the server to accept incoming       */
	/* client connections.  In this example, the backlog is set to 10.  */
	/* This means that the system will queue 10 incoming connection     */
	/* requests before the system starts rejecting the incoming         */
	/* requests.                                                        */
	/********************************************************************/
	rc = listen(sd, 10);
	if (rc< 0)
	{
		perror("listen() failed");
	}

	printf("Ready for client connect().\n");

	/********************************************************************/
	/* The server uses the accept() function to accept an incoming      */
	/* connection request.  The accept() call will block indefinitely   */
	/* waiting for the incoming connection to arrive.                   */
	/********************************************************************/
	sd2 = accept(sd, NULL, NULL);
	if (sd2 < 0)
	{
		perror("accept() failed");
	}

	length = 8;
	rc = setsockopt(sd2, SOL_SOCKET, SO_RCVLOWAT,(char *)&length, sizeof(length));
	if (rc < 0)
	{
		perror("setsockopt(SO_RCVLOWAT) failed");
	}
	while(1) 
	{
		rc = recv(sd2, (uint8_t*)&result, 20, 0);
		if (rc < 0)
		{
			perror("recv() failed");
			break;
		}
		if( result.magic_no == MAGIC_NO )
		{
			if( result.anchorId == 1)
			{
				id[0] = result.anchorId;
				range[0] = result.range;
			}
			else
			{
				id[1] = result.anchorId;
				range[1] = result.range;
			}
			ptm =localtime(&result.time.tv_sec);
			strftime(timeString,sizeof(timeString),"%H:%M:%S",ptm); 
			printf( "\033ctime: %s.%03d id=%d range=%4.2f | id=%d range=%4.2f\n",timeString,(int)(result.time.tv_usec/1000),id[0],range[0],id[1],range[1]);
			fprintf( csv,"%s.%03d,%d,%4.2f\r\n",timeString,(int)(result.time.tv_usec/1000),id[0],range[0]);
		}
		else
		{
			printf("we are out of sync\r\n");
		} 
		if (rc == 0 || rc < sizeof(range))
		{
			printf("The client closed the connection before all of the\n");
			printf("data was sent\n");
			break;
		}

	}
    
 
   /***********************************************************************/
   /* Close down any open socket descriptors                              */
   /***********************************************************************/
   if (sd != -1)
      close(sd);

   if (sd2 != -1)
      close(sd2);

    /***********************************************************************/
   /* Remove the UNIX path name from the file system                      */
   /***********************************************************************/
   unlink(SERVER_PATH);
}

