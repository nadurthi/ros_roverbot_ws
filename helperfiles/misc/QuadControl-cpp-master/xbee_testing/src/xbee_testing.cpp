//============================================================================
// Name        : xbee_testing.cpp
// Author      : Nagnanamus
// Version     :
// Copyright   : Do what ever you want!!! I am not responsible
// Description : Hello World in C++, Ansi-style
//============================================================================


#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include "serialib.h"
#include "SimpleTimer.h"

using namespace std;

#define         DEVICE_PORT_ONBOARD             "/dev/ttyUSB1"
#define         DEVICE_PORT_GROUNDSTATION       "/dev/ttyUSB0"

serialib SerialGound,SerialOnboard ;
SimpleTimer simpletimer;

int main() {
	int Ret;
	char namebuf[500];
	double X[40];

	Ret=SerialGound.Open(DEVICE_PORT_ONBOARD,115200);                                        // Open serial link at 115200 bauds
			    if (Ret!=1) {                                                           // If an error occured...
			        printf ("Error while opening port. Permission problem ?\n");        // ... display a message ...
			        exit(1);                                                         // ... quit the application
			    }
			printf ("Serial port opened successfully: ON BOARD !\n");



			Ret=SerialOnboard.Open(DEVICE_PORT_GROUNDSTATION,115200);                                        // Open serial link at 115200 bauds
					    if (Ret!=1) {                                                           // If an error occured...
					        printf ("Error while opening port. Permission problem ?\n");        // ... display a message ...
					        exit(1);                                                         // ... quit the application
					    }
					printf ("Serial port opened successfully: Ground !\n");
/*

					namebuf[0]='+';
					namebuf[1]='+';
					namebuf[2]='+';
					namebuf[3]=0;

					// write to ground station
						Ret=SerialGound.WriteString(namebuf);
						    		if (Ret!=1)
						    		{printf ("Error while writing data\n");}
cout<<"writting +++"<<endl;
            while(SerialGound.Peek()<=0){}  //always wait for the buffer to recieve data...or else read() on linux will give crazy data

			Ret=SerialGound.ReadString(namebuf,'\r',500,5);
			if (Ret<=0){
			cout<<"Something wrong with reading from ground station: IMBA coder"<<endl;
			}

			cout<<namebuf<<endl;


            //snprintf(namebuf,500,"ATDNOnBoard,WR,CN\r");
            snprintf(namebuf,500,"ATDNOnBoard\r");
            //snprintf(namebuf,500,"ATNI\r");
			// write to ground station
									Ret=SerialGound.WriteString(namebuf);
									    		if (Ret!=1)
									    		{printf ("Error while writing data\n");}

				while(SerialGound.Peek()<=0){}  //always wait for the buffer to recieve data...or else read() on linux will give crazy data

				Ret=SerialGound.ReadString(namebuf,'\n',500,5);
				if (Ret<=0){
				cout<<"Something wrong with reading from ground station: IMBA coder"<<endl;
				}

cout<<namebuf<<endl;

*/

					SerialGound.FlushReceiver();
					SerialOnboard.FlushReceiver();

				while(1){


					int n=snprintf(namebuf,500,"%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",1,2,3,4,5,6,7,8);
					Ret=SerialGound.WriteString(namebuf);
					if (Ret!=1)
					{printf ("Error while writing data\n");}
                     cout<<"sending chars on ground n = "<<n<<endl;

                     simpletimer.ResetTimer();
                     while(SerialOnboard.Peek()<=0){}
                     cout<<"Latency 1= "<<simpletimer.GetTime_from_T0sec()<<endl;

                     Ret=SerialOnboard.ReadString(namebuf,'\n',500,10);
									if (Ret<=0){
									cout<<"Something wrong with reading from ground station: IMBA coder"<<endl;
									}
									cout<<"Chars received on onboard = "<<Ret<<endl;

//usleep(2000);
					n=snprintf(namebuf,500,"%.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",1,2,3,4,5,6,7);
					Ret=SerialOnboard.WriteString(namebuf);
										if (Ret!=1)
										{printf ("Error while writing data\n");}
					cout<<"sending chars on onboard n = "<<n<<endl;

					simpletimer.ResetTimer();
					while(SerialGound.Peek()<=0){}
					cout<<"Latency 2= "<<simpletimer.GetTime_from_T0sec()<<endl;

					Ret=SerialGound.ReadString(namebuf,'\n',500,10);
					if (Ret<=0){
					cout<<"Something wrong with reading from ground station: IMBA coder"<<endl;
					}
					cout<<"Chars received on ground = "<<Ret<<endl<<endl;

										//cout<<"Send - Return Latency = "<<simpletimer.GetTime_from_T0sec()<<endl<<endl;

					usleep(5000);
				}


}
