#ifndef UDPPORT_H_
#define UDPPORT_H_

#include "stdio.h"
#include "stdlib.h"
#include <unistd.h>
#include <strings.h>
#include "sys/types.h"
#include "sys/socket.h"
#include "netinet/in.h"
#include "netdb.h"
#include <string>
#include <netinet/in.h>
#include <arpa/inet.h>

class udpPort
{
private:
	int sockfd;
	int fd;
	struct sockaddr_in UDPserveraddr;
	struct sockaddr_in clientaddr;
	struct timespec timeOUT;
	fd_set fds;
char buffer[100];

public:
	udpPort();
	bool OpenPort_read(char * address,int port);
	bool OpenPort_write(char * address,int port);

	void ClosePort();
	int CheckData();
	int ReadPort(char *rxbuffer,int len);
	int write(char *txbuffer,int len);

};

#endif



