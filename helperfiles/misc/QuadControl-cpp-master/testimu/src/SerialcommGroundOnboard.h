/*
 * SerialcommGroundOnboard.h
 *
 *  Created on: Apr 19, 2014
 *      Author: nagnanamus
 */

#ifndef SERIALCOMMGROUNDONBOARD_H_
#define SERIALCOMMGROUNDONBOARD_H_

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include "serialib.h"
#include "State_data_manager.h"
#include "SimpleTimer.h"

using namespace std;

// on beagelbone we know which xbee is on whcih port
#define         DEVICE_PORT_ONBOARDtx             "/dev/ttyO1"
#define         DEVICE_PORT_ONBOARDrx             "/dev/ttyO2"

//on the ground computer the order we plugin the xbees depends on the ports assigned
#define         DEVICE_PORT_GROUND1       "/dev/ttyUSB0"
#define         DEVICE_PORT_GROUND2       "/dev/ttyUSB1"

#define ReadPeekTimeout 0.005  //time out to wait for data on serial port ...in seconds
/*
 * Two separate xbees are used. One to read and one to write
 */
class Serial_comm_Ground_Onboard {
	serialib SerialComTx,SerialComRx;
	int Ret;
	char namebuf[500],namebuf2[200];
	double X[40];


	SimpleTimer ReturnDataTimer;

public:
	int totalpackets,lostpackets;
	//Serial_comm_Ground_Onboard();
	//virtual ~Serial_comm_Ground_Onboard();
double GetPacketLossRate(){
	return (double)lostpackets/totalpackets;
}
	void Init_as_Onboard();
	void Init_as_GroundStation();

	int IsNodeIDtx(serialib * comm);
	void SetRO(serialib * comm);    //packetization timeout
    int EnterCommandModeXbee(serialib * comm);
    void ResetXbee(serialib * comm);
    void FactoryResetXbee(serialib * comm);
    void SetBaudRateXbee(serialib * comm);

	int CharArray2floatfields(double *X,char *S, int L );
	int floatfields2CharArray(double *X,int L,char * S,int Slen, int options );
	void Request_GroundStationData(int option,QuadStateVariable * StateData);
	void Fulfil_GroundStationRequest(QuadStateVariable * StateData);

	void GroundStation_SimpleSend(int option,QuadStateVariable * StateData);
	void GroundStation_SimpleRecieve(int opt,QuadStateVariable * StateData,double *Daccelgyro);


	//GroundStation places a data request to onboard controller
	void Request_OnboardData(int option,QuadStateVariable * StateData);
	//Ground station completes the onboard data request
	void Fulfil_OnboardRequest(QuadStateVariable * StateData);

	void Onboard_SimpleSend(QuadStateVariable * StateData);
	void Onboard_SimpleRecieve(int option,QuadStateVariable * StateData,double * DataVicon);

	int Checkif_DataPresent_onboard(int option){
		if(SerialComRx.Peek()>57 && option==8)  //onboard receives [t,x,q] from ground of 60 bytes
			return 1;
		else
			return -1;


	}

	int Checkif_DataPresent(){
		if(SerialComRx.Peek()>0)
			return 1;
		else
			return -1;


	}

};

#endif /* SERIALCOMMGROUNDONBOARD_H_ */
