/* A simple server in the internet domain using TCP
   The port number is passed as an argument */
#ifndef DATASOCKET_H_
#define DATASOCKET_H_

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <netdb.h>
#include <sys/types.h>

#include <arpa/inet.h>

using namespace std;

#define MAXDATASIZE 100 // max number of bytes we can get at once


void error(char *msg)
{
    perror(msg);
    exit(1);
}

class DataSocket{
	int sockfd, newsockfd, portno;
	unsigned int clilen;
	     char buffer[256];
	     struct sockaddr_in serv_addr, cli_addr;
	     int n;
	     struct sockaddr *sa;

	   //client variables
	     struct hostent *server;
	         int  numbytes;
	         char buf[MAXDATASIZE],buf1[30];
	         struct addrinfo hints, *servinfo, *p;
	         int rv;
	         char s[INET6_ADDRSTRLEN];

public:

	         void *get_in_addr(struct sockaddr *sa)
	         {
	             if (sa->sa_family == AF_INET) {
	                 return &(((struct sockaddr_in*)sa)->sin_addr);
	             }

	             return &(((struct sockaddr_in6*)sa)->sin6_addr);
	         }

	     void InitializeClient(char *hostname,int pp){
	    	 portno=pp;
	    	 memset(&hints, 0, sizeof hints);
	    	     hints.ai_family = AF_UNSPEC;
	    	     hints.ai_socktype = SOCK_STREAM;

	    	     snprintf(buf1,30,"%d",portno);
                 cout<<buf1<<endl;
	    	     if ((rv = getaddrinfo(hostname, buf1, &hints, &servinfo)) != 0) {
	    	            fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
	    	            exit(0);
	    	        }

	    	     // loop through all the results and connect to the first we can
	    	       for(p = servinfo; p != NULL; p = p->ai_next) {
	    	           if ((sockfd = socket(p->ai_family, p->ai_socktype,
	    	                   p->ai_protocol)) == -1) {
	    	               perror("client: socket");
	    	               continue;
	    	           }

	    	           if (connect(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
	    	               close(sockfd);
	    	               perror("client: connect");
	    	               continue;
	    	           }

	    	           break;
	    	       }

	    	       if (p == NULL) {
	    	           fprintf(stderr, "client: failed to connect\n");
	    	           exit(0);
	    	       }

	    	       inet_ntop(p->ai_family, get_in_addr((struct sockaddr *)p->ai_addr),
	    	               s, sizeof s);
	    	       printf("client: connecting to %s\n", s);

	    	       freeaddrinfo(servinfo); // all done with this structure



	     }

	      int Recieve_onClient(char * buff, int nn){
	    	  if ((numbytes = recv(sockfd, buff, nn-1, 0)) == -1) {
	    	         perror("recv");
	    	         exit(1);
	    	     }

	    	     buff[numbytes] = '\0';

	    	     return numbytes;

	      }


	     void InitializeServer(int p){
	    	 portno=p;
	    	  sockfd = socket(AF_INET, SOCK_STREAM, 0);

	    	  if (sockfd < 0)
	    	    error("ERROR opening socket");
	    	  bzero((char *) &serv_addr, sizeof(serv_addr));

	    	  serv_addr.sin_family = AF_INET;
	    	  serv_addr.sin_addr.s_addr = INADDR_ANY;
	    	  serv_addr.sin_port = htons(portno);

	    	  if (bind(sockfd, (struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
	    	                error("ERROR on binding");

	    	  listen(sockfd,5);

	    	       clilen = sizeof(cli_addr);
	    	       newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);

	    	       if (newsockfd < 0)
	    	            error("ERROR on accept");


	     }
	     int Send_onServer(char * buff,int msg_length){

	    	 n = write(newsockfd,buff,msg_length);

	    	 if (n < 0) error("ERROR writing to socket");

	     	return 0;
	     }

	     //return the pointer to the buffer data and the length of the msg.
	     int Receive_onServer(char * buff){
	    	 bzero(buff,256);
	    	 n = read( newsockfd,buffer,255 );
	    	     if (n < 0)
	    	     {
	    	         perror("ERROR reading from socket");
	    	         exit(1);
	    	     }
	     buff=buffer;
	     return n;
	     }
};


#endif
