/*
 * AfUnix.h
 *
 *  Created on: Mar 14, 2015
 *      Author: roy
 */

#ifndef INSTANCE_AFUNIX_H_
#define INSTANCE_AFUNIX_H_


#define SERVER_PATH     "/var/run/uwb"
#define MAGIC_NUMBER 	0xFACEFEED
int AfUnixClinetConnect();
int AfUnixClinetSendReportTOF( unsigned char* buffer, int size);
void AfunixClinetCloseSocket();

#endif /* INSTANCE_AFUNIX_H_ */
