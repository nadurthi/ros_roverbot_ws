///////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) OMG Plc 2009.
// All rights reserved.  This software is protected by copyright
// law and international treaties.  No part of this software / document
// may be reproduced or distributed in any form or by any means,
// whether transiently or incidentally to some other use of this software,
// without the written permission of the copyright owner.
//
///////////////////////////////////////////////////////////////////////////////

#include "vicon_data_custom.h"

#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stropts.h>
#include <cmath>
#include <stdio.h>
#include <string>
#include <sys/types.h>
#include <time.h>
#include <sys/time.h>
#include <stdint.h>
#include <fstream>
#include <cmath>
#include "controllers.h"
#include "SimpleTimer.h"
#include "joystick.h"
#include "quaternion_operations.h"
#include "/home/nagnanamus/Dropbox/BBB/codes/quad_control/estimation/src/initialize_parameters.h"

using namespace std;

#include "matrix_operations.h"

#include "UDPcommGroundOnboard.h"
#include "kinematic_estimators.h"
#include "State_data_manager.h"
#include "/home/nagnanamus/Dropbox/BBB/codes/quad_control/source_codes/Eigen/Dense"

SimpleTimer GroundTimer,TicToc;

QuadStateVariable QuadState;


//EKF_Estimator_onboard1  GroundEstimator;
EKF_Estimator1  GroundEstimator;

UDPcommGroundOnboard GroundStationUDP;




using namespace std;



Vicon_data vicondata;


int main( int argc, char* argv[] )
{

char namebuf[500];


     //Setup joystick to constrol ref attitude and kq gain
     joystick joystick;
     if (joystick.start()==false)
		{cout<<"error with start of joystick"<<endl;
		return 0;
		}
joystick.ClearButtonFlag(15);


	//Setup UDP comm with onboard
		GroundStationUDP.Init_as_Ground();



	//set the state variables
		QuadState.Init_SetZero();
		Initialize_all_parameters(&QuadState);


	//Setup the ground timer
		GroundTimer.SetTsec(0.03);



	// Initialize the estimator
	//	GroundEstimator.InitializeFilter();


	//vicon initalize
			vicondata.initialize(1);
			vicondata.SetT0sec();

		TicToc.ResetTimer();


	double Data[8],Data_imu[7],u[4],tt, MarkerX[27];


double Timu,Dkq,Eul[3]={0,0,0};
char rxbuffer[100];
int i=0,m=0,option=1;
char S[500],msg[20];
int nn,flg=0;




//eternal while
GroundTimer.ResetTimer();



for(i=0;i<30;i++){
	vicondata.GetData(Data,MarkerX);
}
QuadState.xref[0]=Data[1];
QuadState.xref[1]=Data[2];
QuadState.xref[2]=Data[3];

cout<<"entering eternal while"<<endl;


	while(1){

		//default msg is the X Q i.e. x_meas and q_meas
		snprintf(msg,20,"X Q");
		QuadState.b1ref[0]=1;
		QuadState.b1ref[1]=0;
		QuadState.b1ref[2]=0;

		//check if the joystick emergency stop button has been pressed
		if(joystick.CheckifEmergencyStopRequest()==true){
			cout<<"Emergency Stop Requested : exit"<<endl;
			exit(1);
		}

		//increment kw
        if(joystick.GetButtonFlag(leftupbit)==1){
        	QuadState.xref[0]=joystick.IncrementVal_Button(QuadState.xref[0],0.01, -1, 1);
        	joystick.ClearButtonFlag(leftupbit);
        	strcat(msg," C");
        }
        //decrement kw
        if(joystick.GetButtonFlag(leftdownbit)==1){
        	QuadState.xref[0]=joystick.DecrementVal_Button(QuadState.xref[0],0.01,-1, 1);
        	joystick.ClearButtonFlag(leftdownbit);
        	strcat(msg," C");
        }

        //increment zref
        if(joystick.GetButtonFlag(handlemidbit)==1){
        	Eul[1]=joystick.IncrementVal_Button(Eul[1],0.001,-1.55, 1.55);
        	Euler2Quat(QuadState.qref,Eul);
        	quat_rot(QuadState.b1ref,QuadState.qref,QuadState.b1ref);
        	joystick.ClearButtonFlag(handlemidbit);
        	strcat(msg," D");
        }
        //decrement zref
        if(joystick.GetButtonFlag(handledownbit)==1){
        	Eul[1]=joystick.DecrementVal_Button(Eul[1],0.001, -1.55, 1.55);
        	Euler2Quat(QuadState.qref,Eul);
        	quat_rot(QuadState.b1ref,QuadState.qref,QuadState.b1ref);
        	joystick.ClearButtonFlag(handledownbit);
        	strcat(msg," D");
        }

        //yaw right clockwise change the b1d direction
        if(joystick.GetButtonFlag(handlerightbit)==1){
        	Eul[2]=joystick.IncrementVal_Button(Eul[2],0.005, -1.55, 1.55);
        	//Euler2Quat(QuadState.qref,Eul);
        	Euler2Quat(QuadState.qref,Eul);
        	quat_rot(QuadState.b1ref,QuadState.qref,QuadState.b1ref);

        	joystick.ClearButtonFlag(handlerightbit);
        	strcat(msg," D");
        }
        //yaw left anti-clock change the b1d direction
        if(joystick.GetButtonFlag(handleleftbit)==1){
        	Eul[2]=joystick.DecrementVal_Button(Eul[2],0.005, -1.55, 1.55);
        	//Euler2Quat(QuadState.qref,Eul);
        	Euler2Quat(QuadState.qref,Eul);
        	quat_rot(QuadState.b1ref,QuadState.qref,QuadState.b1ref);
        	joystick.ClearButtonFlag(handleleftbit);
        	strcat(msg," D");
        }

        //kv up
        if(joystick.GetButtonFlag(rightupbit)==1){
        	QuadState.xref[1]=joystick.IncrementVal_Button(QuadState.xref[1],0.01,-1,1);
        	//if(abs(QuadState.ctf)<0.001){
        	//	QuadState.ctf=0.001;
        	//}
        	joystick.ClearButtonFlag(rightupbit);
        	strcat(msg," C");
        }
        //kv down
        if(joystick.GetButtonFlag(rightdownbit)==1){
        	QuadState.xref[1]=joystick.DecrementVal_Button(QuadState.xref[1],0.01,-1, 1);
        	//if(abs(QuadState.ctf)<0.001){
        	//	QuadState.ctf=0.001;
        	//}
        	joystick.ClearButtonFlag(rightdownbit);
        	strcat(msg," C");
        }


            //check if the axis flags are up
			if(joystick.GetAxisFlag(2)==1){ //ctf axis is set
				Dkq=joystick.getaxis(2);
				QuadState.xref[2]=joystick.ReMapValue(Dkq,0.0001,1);

				joystick.ClearAxisFlag(2);
				strcat(msg," C");

			}
/*
			if(joystick.GetAxisFlag(0)==1){ //roll axis is set
				Dkq=joystick.getaxis(0);
				Eul[0]=-1*joystick.ReMapValue(Dkq,-0.3490658,0.3490658);
				Euler2Quat(QuadState.qref,Eul);
				quat_rot(QuadState.b1ref,QuadState.qref,QuadState.b1ref);
				joystick.ClearAxisFlag(0);

				nn=0;// checking if msg already has 'B'
				flg=0;
					while(msg[nn]!='\0'){
						if(msg[nn]=='D'){
						flg++;
						}
						nn++;
					}
					if(flg==0)
						strcat(msg," D");

			}

			if(joystick.GetAxisFlag(1)==1){ //pitch axis is set
				Dkq=joystick.getaxis(1);
				Eul[1]=joystick.ReMapValue(Dkq,-0.3490658,0.3490658);
				Euler2Quat(QuadState.qref,Eul);
				quat_rot(QuadState.b1ref,QuadState.qref,QuadState.b1ref);
				joystick.ClearAxisFlag(1);
				nn=0;// checking if msg already has 'B'
				flg=0;

					while(msg[nn]!='\0'){

						if(msg[nn]=='D'){

						flg++;
						}
						nn++;
					}

					if(flg==0)
						strcat(msg," D");
			}


*/


		GroundTimer.ResetTimer();


			vicondata.GetData(Data,MarkerX);
			QuadState.Update_Vicon_meas(Data,-1);
			GroundStationUDP.SendData2Onboard(msg,&QuadState);



		while(GroundTimer.CheckTimer()){}



	}

return 1;

}
