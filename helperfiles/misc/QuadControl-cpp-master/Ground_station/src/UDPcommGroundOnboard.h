/*
 * UDPcommGroundOnboard.h
 *
 *  Created on: May 21, 2014
 *      Author: nagnanamus
 */

#ifndef UDPCOMMGROUNDONBOARD_H_
#define UDPCOMMGROUNDONBOARD_H_
#include "udpPort.h"
#include "State_data_manager.h"
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>


using namespace std;

#define GroundRx_ip "10.0.0.39"
#define GroundTx_ip "10.0.0.25"

#define OnboardRx_ip "10.0.0.25"
#define OnboardTx_ip "10.0.0.39"

#define GroundRx_port 1234
#define GroundTx_port 2345

#define OnboardRx_port 2345
#define OnboardTx_port 1234

class UDPcommGroundOnboard {
public:
	udpPort RxUDP,TxUDP;
	double X[30];
	char buffer[500];
	void Init_as_Ground();
	void Init_as_Onboard();
	void Send2Ground(int option,QuadStateVariable * StateData);


	/**********************************************8
	 * New Updated Data transfers
	 * for msg encoding use the switch tables in QuadStateVariable functions
	 */
	void SendData2Ground(char * msg,QuadStateVariable * StateData);
	void ReceiveData_on_Onboard(QuadStateVariable * StateData,int option);

	void SendData2Onboard(char * msg,QuadStateVariable * StateData);
	void ReceiveData_on_Ground(QuadStateVariable * StateData,int option);
//************************************************8

	bool CheckRxData(int n);

	/*
	 * Ground sends data to onboard
	 */
	void Send2Onboard(int option,QuadStateVariable * StateData);

	void Receive_on_Ground(QuadStateVariable * StateData);

	void Receive_on_Onboard(QuadStateVariable * StateData,double *DataVicon,int UPDToption);

	/*
	 * Convert char array/string to a float array
	 * X is output float array
	 * S is char array
	 * L is the number float fields in S hence also in X
	 * S has to end with "\n\0"
	 */
	int CharArray2floatfields(double *X,char *S, int L );

	/*
	 * Converst float array into a string
	 * %010.5f precicion is used  4 digits before decimal 5 digits after decimal
	 * X is the the float array
	 * L is the number of fields in X
	 * S is the output char array or string.
	 * Slen is length of cahr Array
	 * if option =0, then just sedn the string
	 * if option >0 then append it to the starting of the string and then send it
	 */
    int floatfields2CharArray(double *X,int L,char * S,int Slen, int options );



};

#endif /* UDPCOMMGROUNDONBOARD_H_ */
