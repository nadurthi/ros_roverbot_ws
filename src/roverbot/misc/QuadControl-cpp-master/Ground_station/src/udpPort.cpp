#include "udpPort.h"

udpPort::udpPort()
{

}


bool udpPort::OpenPort_read(char * address,int port)
{
	timeOUT.tv_sec = 0;
	timeOUT.tv_nsec = 1;

	//this->sockfd = sockfd;
	sockfd = socket(AF_INET,SOCK_DGRAM, 0);
	if(sockfd < 0)
	{
		printf("error opening socket \n");
		return false;
	}

	bzero((char*)&UDPserveraddr, sizeof(UDPserveraddr));
	UDPserveraddr.sin_family = AF_INET;
	UDPserveraddr.sin_port = htons(port);
	//UDPserveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
	UDPserveraddr.sin_addr.s_addr = inet_addr(address);

	if(bind(sockfd,(sockaddr*) &UDPserveraddr,sizeof(UDPserveraddr))<0)
	{
		printf("socket binding error \n");
		return false;
	}

	return true;
}

bool udpPort::OpenPort_write(char * address,int port)
{
	timeOUT.tv_sec = 0;
	timeOUT.tv_nsec = 1;


	sockfd = socket(AF_INET,SOCK_DGRAM, 0);
	if(sockfd < 0)
	{
		printf("error opening write socket \n");
		return false;
	}

	bzero((char*)&UDPserveraddr, sizeof(UDPserveraddr));
	UDPserveraddr.sin_family = AF_INET;
	UDPserveraddr.sin_port = htons(port);
	//UDPserveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
	UDPserveraddr.sin_addr.s_addr = inet_addr(address); //htonl(INADDR_ANY);

	if(bind(sockfd,(sockaddr*) &UDPserveraddr,sizeof(UDPserveraddr))<0)
	{
		printf("socket binding error \n");
		return false;
	}

	return true;
}

int udpPort::CheckData(){
	int len2 = sizeof(struct sockaddr_in);

		FD_ZERO(&fds);
		FD_SET(sockfd,&fds);
		int rc = pselect(sockfd,&fds,NULL,NULL,&timeOUT,NULL);

		    //flag=(MSG_PEEK | MSG_DONTWAIT) check if there is data and then read ht packet
		int incoming = recvfrom(sockfd,buffer,10,(MSG_PEEK | MSG_DONTWAIT),(struct sockaddr *) &clientaddr,(socklen_t *)&len2);

		return incoming;
}
int udpPort::ReadPort(char *rxbuffer,int len)
{

	int len2 = sizeof(struct sockaddr_in);

	FD_ZERO(&fds);
	FD_SET(sockfd,&fds);
	int rc = pselect(sockfd,&fds,NULL,NULL,&timeOUT,NULL);

	bzero(rxbuffer,len);
	    //flag=(MSG_PEEK | MSG_DONTWAIT) check if there is data and then read ht packet
	int incoming = recvfrom(sockfd,rxbuffer,len,0,(struct sockaddr *) &clientaddr,(socklen_t *)&len2);

	return incoming;


}

int udpPort::write(char *txbuffer,int len){


	//int sendto(sockfd, txbuffer,len, int flags, const struct sockaddr *dest_addr,socklen_t dest_len)

	if (sendto(sockfd, txbuffer,len, 0,(sockaddr*) &UDPserveraddr,sizeof(UDPserveraddr)) < 0) {
		perror("sendto failed");
		return 0;
	}
	return 1;
}


void udpPort::ClosePort()
{
	close(sockfd);
}




