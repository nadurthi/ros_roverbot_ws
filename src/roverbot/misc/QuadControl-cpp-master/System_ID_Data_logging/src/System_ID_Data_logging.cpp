//============================================================================
// Name        : System_ID_Data_logging.cpp
// Author      : nag
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <stropts.h>
#include <cmath>
#include <stdio.h>
#include <string>
#include <stdint.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <time.h>
#include <sys/time.h>
#include "SimpleGPIO.h"
#include "Motor_spi_comm_modf_datalog.h"


using namespace std;

#define FSS 60
#define rstpin 30

void error(const char *msg)
{
    perror(msg);
    exit(0);
}


class SimpleTimer {
private:
	double Tsec,T0sec;
	struct timeval T0;
	struct timeval tv;
	struct timezone tz;
public:
	void ResetTimer(){
		gettimeofday(&tv,&tz);
		T0sec=(double)tv.tv_sec+(double)(tv.tv_usec)/1000000;
	}
	void SetTsec(double tsec){
		Tsec=tsec;
	}
	bool CheckTimer(){
		double tt;
		gettimeofday(&tv,&tz);
		tt=(double)tv.tv_sec+(double)(tv.tv_usec)/1000000;
		if (tt-T0sec>=Tsec)
		{return false;}
		else
		{return true;}
	}
};

int main() {

	int u,vel;
	ofstream Datalog;
	Datalog.open("InputOutputData.txt");

	string line;
	ifstream InputCtrl ("InputCtrl.txt");
	if (InputCtrl.is_open())
	{}
	else
	{cout<<"cannot open input control file";}

	char namebuf[150];

	snprintf(namebuf, sizeof(namebuf), "%s,%s\n","U","V");
	Datalog << namebuf;

	init_motor_spi_ss(FSS,rstpin);
	usleep(2500000);

	SimpleTimer CtrlTimer;
	CtrlTimer.SetTsec(0.1)	; //seconds for control update
	CtrlTimer.ResetTimer(); //reset the timer



	int Nctrl=2401;
	int ind;

for(ind=0;ind<Nctrl;ind++)
{

   // cout<<"Input the control rpm u = ";
   // cin>>u;

	//from file
	InputCtrl>>u;

	transfer_rpmdata(u,&vel,FSS);
    cout<<"Velocity in rpm = "<<vel<<endl;

	snprintf(namebuf, sizeof(namebuf), "%d,%d\n",u,vel);
	Datalog << namebuf;


	while(CtrlTimer.CheckTimer()){}
	CtrlTimer.ResetTimer();




}



Datalog.close();
InputCtrl.close();




	return 0;
}
