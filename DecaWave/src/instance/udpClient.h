/*
 * udpClient.h
 *
 *  Created on: Oct 21, 2014
 *      Author: roy
 */

#ifndef INSTANCE_UDPCLIENT_H_
#define INSTANCE_UDPCLIENT_H_

int UdpclinetConnect( const char * serverIp, int port );
int UdpClinetSendReportTOF( unsigned char* buffer, int size);
int UdpClinetGetReportTOF( unsigned char* buffer, int* size);
void UdpClinetCloseSocket();
#endif /* INSTANCE_UDPCLIENT_H_ */
