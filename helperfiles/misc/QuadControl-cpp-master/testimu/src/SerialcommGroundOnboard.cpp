/*
 * SerialcommGroundOnboard.cpp
 *
 *  Created on: Apr 19, 2014
 *      Author: nagnanamus
 */

#include "SerialcommGroundOnboard.h"

//Serial_comm_Ground_Onboard::Serial_comm_Ground_Onboard() {
	// TODO Auto-generated constructor stub

//}

//Serial_comm_Ground_Onboard::~Serial_comm_Ground_Onboard() {
	// TODO Auto-generated destructor stub
//}

/*
 * parses the char array and separate the fields and converts them to float
 * the fields are separated by space. the char array has to end with '\n' char
 * there are n fields, the return double array has to be same size.
 * L=length of the char array without the \n or \0 chars
 */
int  Serial_comm_Ground_Onboard::CharArray2floatfields(double *X,char *S, int L ){
	int j=0,n=0;
	char buff[30];
	for(int i=0;i<L;i++){
      if(S[i]==' '|| S[i]=='\n'||S[i]=='\0')
      {

    	  buff[j]='\0';
    	  j=0;
    	  X[n]=atof(buff);
    	  n++;
      }
      else{
		buff[j]=S[i];
		j++;
      }
	}
	return n;
}
/*
 * Converts the array of floats to a char array
 * return the number of characters
 * L is the number of array in X
 */

int Serial_comm_Ground_Onboard::floatfields2CharArray(double *X,int L,char * S,int Slen, int options ){
int m=0;
	if(options==0){
	for(int n=0;n<L-1;n++){
		m=m+snprintf(S+m, Slen, "%.4f ",X[n]);
    }
	}
	else
	{
		m=m+snprintf(S+m, Slen, "%d ",options);
		for(int n=0;n<L-1;n++){
			m=m+snprintf(S+m, Slen, "%.4f ",X[n]);
	    }

	}
	snprintf(S+m, Slen, "%.4f\n",X[L-1]);
    return sizeof(S);
}

/*
 * Enter the command mode for an Xbee
 */
int Serial_comm_Ground_Onboard::EnterCommandModeXbee(serialib * comm){
	        namebuf[0]='+';
			namebuf[1]='+';
			namebuf[2]='+';
			namebuf[3]=0;
			Ret=comm->WriteString(namebuf);
						    		if (Ret!=1)
						    		{printf ("Error while writing data\n");}


			while(comm->Peek()<=0){}  //always wait for the buffer to recieve data...or else read() on linux will give crazy data

			Ret=comm->ReadString(namebuf,'\r',500,20);
			if (Ret<=0){
			cout<<"Something wrong with reading command"<<endl;
			exit(1);
			}
		cout<<"Entered Command mode : "<<namebuf<<endl;


		if(namebuf[0]=='O'){
		return 1;
		}
		else
			return -1;
}
/*
 * Software the Reset the Xbee module
 */
void Serial_comm_Ground_Onboard::ResetXbee(serialib * comm){

	EnterCommandModeXbee(comm);
	cout<<"Reseting the Xbee"<<endl;
	snprintf(namebuf,100,"ATFR,CN\r");
							Ret=comm->WriteString(namebuf);
							if (Ret!=1)
							{printf ("Error while writing data\n"); exit(1);}


							while(comm->Peek()<=0){}

							Ret=comm->ReadString(namebuf,'\r',500,20);
															if (Ret<=0){
															cout<<"Something wrong with reading command"<<endl;
															exit(1);
															}
								cout<<"Reseting under progress : "<<namebuf<<endl;
								usleep(500000);
								cout<<"Reseting Done .....I guess "<<endl;
}

/*
 * Set the packetization timeout. Send large data as bigger chunks
 */
void Serial_comm_Ground_Onboard::SetRO(serialib * comm){
	//first enter the command mode by writing '+++'

	EnterCommandModeXbee(comm);


	if(namebuf[0]=='O'){

					snprintf(namebuf,100,"ATRO80,WR,CN\r");
					Ret=comm->WriteString(namebuf);
								    		if (Ret!=1)
								    		{printf ("Error while writing data\n");}
					int n=0;
					cout<<"Waiting for the OKs"<<endl;
					while(n<3){
						while(comm->Peek()<=0){}
						Ret=comm->ReadString(namebuf,'\r',500,20);
								if (Ret<=0){
								cout<<"Something wrong with reading command"<<endl;
								exit(1);
								}
							cout<<namebuf<<endl;
							n++;

					}
					cout<<"Done reading the OKs n = "<<n<<endl;
					if(n==3){

						cout<<"Done writing the RO parameter. Reading it to confirm"<<endl;
usleep(100000);
						EnterCommandModeXbee(comm);

						snprintf(namebuf,100,"ATRO,CN\r");
						Ret=comm->WriteString(namebuf);
						if (Ret!=1)
						{printf ("Error while writing data\n"); exit(1);}


						while(comm->Peek()<=0){cout<<"Peek = "<<comm->Peek()<<endl;usleep(10000);}

						Ret=comm->ReadString(namebuf,'\r',500,20);
														if (Ret<=0){
														cout<<"Something wrong with reading command"<<endl;
														exit(1);
														}
							cout<<"The current RO value is : "<<namebuf<<endl;

					}
					else
						{cout<<"writing the requested RO para not possible"<<endl;exit(1);}


	}
	else
	{
		cout<<"Did not recieve OK for +++ "<<endl;
		exit(1);
	}
}

/*
 * This functions enters the xbee into command mode and checks the Node name of the xbee
 * This is to find if the port assigned to the xbee is a Rx xbee or Tx xbee
 */
int Serial_comm_Ground_Onboard::IsNodeIDtx(serialib * comm){

	//first enter the command mode by writing '+++'
	namebuf[0]='+';
	namebuf[1]='+';
	namebuf[2]='+';
	namebuf[3]=0;
	Ret=comm->WriteString(namebuf);
				    		if (Ret!=1)
				    		{printf ("Error while writing data\n");}


	while(comm->Peek()<=0){}  //always wait for the buffer to recieve data...or else read() on linux will give crazy data

	Ret=comm->ReadString(namebuf,'\r',500,20);
	if (Ret<=0){
	cout<<"Something wrong with reading command"<<endl;
	exit(1);
	}
cout<<namebuf<<endl;

	if(namebuf[0]=='O'){  //if u received OK, I just check if we received O
			namebuf[0]='A';
			namebuf[1]='T';
			namebuf[2]='N';
			namebuf[3]='I';
			namebuf[4]='\r';
			namebuf[5]=0;
			Ret=comm->WriteString(namebuf);
						    		if (Ret!=1)
						    		{printf ("Error while writing data\n");}

			while(comm->Peek()<=0){}      //receive the node identifier
			Ret=comm->ReadString(namebuf,'\r',500,20);
			if (Ret<=0){
			cout<<"Something wrong with reading command"<<endl;
			exit(1);
			}
			cout<<namebuf<<endl;
			int tx=0,rx=0;
			for(int i=0;i<Ret;i++){
				if(namebuf[i]==0 || namebuf[0]=='\r'){
					break;
				}

				if(namebuf[i]=='t' && namebuf[i+1]=='x'){ tx=tx+1;}

				if(namebuf[i]=='r' && namebuf[i+1]=='x'){ rx=rx+1;}
			}

			if(tx==1 && rx==1){cout<<"Both tx and rx are in the name"<<endl;exit(1);}

			if(tx==1){return 1;}
			else{return -1;}



	}
	else{
		cout<<"error cannot get the resposne for comamnd"<<endl;
				exit(1);
	}

}

void Serial_comm_Ground_Onboard::FactoryResetXbee(serialib * comm){
	EnterCommandModeXbee(comm);
	snprintf(namebuf,100,"ATRE,WR,CN\r");
						Ret=comm->WriteString(namebuf);
									    		if (Ret!=1)
									    		{printf ("Error while writing data\n");}

									    		usleep(10000);
}
void Serial_comm_Ground_Onboard::SetBaudRateXbee(serialib * comm){
	EnterCommandModeXbee(comm);
	snprintf(namebuf,100,"ATBD7,WR,CN\r");
							Ret=comm->WriteString(namebuf);
										    		if (Ret!=1)
										    		{printf ("Error while writing data\n");}

}


void Serial_comm_Ground_Onboard::Init_as_Onboard(){

	totalpackets=0;
	lostpackets=0;

	Ret=SerialComTx.Open(DEVICE_PORT_ONBOARDtx ,115200);                                        // Open serial link at 115200 bauds
		    if (Ret!=1) {                                                           // If an error occured...
		        printf ("Error while opening TX serial port. Permission problem ?\n");        // ... display a message ...
		        exit(1);                                                         // ... quit the application
		    }
		printf ("TX Serial port opened successfully: ON BOARD !\n");



		Ret=SerialComRx.Open(DEVICE_PORT_ONBOARDrx ,115200);                                        // Open serial link at 115200 bauds
			    if (Ret!=1) {                                                           // If an error occured...
			        printf ("Error while opening RX serial port. Permission problem ?\n");        // ... display a message ...
			        exit(1);                                                         // ... quit the application
			    }
			printf ("RX Serial port opened successfully: ON BOARD !\n");



			ReturnDataTimer.SetTsec(ReadPeekTimeout);

			SerialComTx.FlushReceiver();
			SerialComRx.FlushReceiver();
			SerialComTx.FlushTransmitter();
			SerialComRx.FlushTransmitter();

//Reset the crap to factory
			//FactoryResetXbee(&SerialComTx);
			//FactoryResetXbee(&SerialComRx);

			//Set other paras of Xbee
				//SetRO(&SerialComTx);
				//SetRO(&SerialComRx);

				//First reset the module
				  //ResetXbee(&SerialComTx);
				   //ResetXbee(&SerialComRx);
// Set new Baudrate of 115200
			//SetBaudRateXbee(&SerialComTx);
			//SetBaudRateXbee(&SerialComRx);
}
void Serial_comm_Ground_Onboard::Init_as_GroundStation(){

	totalpackets=0;
	lostpackets=0;

	Ret=SerialComTx.Open(DEVICE_PORT_GROUND1 ,115200);                                        // Open serial link at 115200 bauds
		    if (Ret!=1) {                                                           // If an error occured...
		        printf ("Error while opening port. Permission problem ?\n");        // ... display a message ...
		        exit(1);                                                         // ... quit the application
		    }
		//printf ("Serial port opened successfully: GROUND STATION  !\n");
		    SerialComTx.FlushReceiver();
/*
    if(IsNodeIDtx(&SerialComTx)<1){  //if the port is not a tx then close it set it right
		SerialComTx.Close();
		Ret=SerialComTx.Open(DEVICE_PORT_GROUND2 ,9600);                                        // Open serial link at 115200 bauds
				    if (Ret!=1) {                                                           // If an error occured...
				        printf ("Error while TX opening port. Permission problem ?\n");        // ... display a message ...
				        exit(1);                                                         // ... quit the application
				    }
		printf ("TX Serial port opened successfully: GROUND STATION  !\n");

		Ret=SerialComRx.Open(DEVICE_PORT_GROUND1 ,9600);                                        // Open serial link at 115200 bauds
				    if (Ret!=1) {                                                           // If an error occured...
				        printf ("Error while opening RX port. Permission problem ?\n");        // ... display a message ...
				        exit(1);                                                         // ... quit the application
				    }
		printf ("RX Serial port opened successfully: GROUND STATION  !\n");


	}
	else */{
		Ret=SerialComRx.Open(DEVICE_PORT_GROUND2 ,115200);                                        // Open serial link at 115200 bauds
						    if (Ret!=1) {                                                           // If an error occured...
						        printf ("Error while opening RX port. Permission problem ?\n");        // ... display a message ...
						        exit(1);                                                         // ... quit the application
						    }
		printf ("TX and RX Serial port opened successfully: GROUND STATION  !\n");
	}

    ReturnDataTimer.SetTsec(ReadPeekTimeout);

		SerialComTx.FlushReceiver();
		SerialComRx.FlushReceiver();
		SerialComTx.FlushTransmitter();
		SerialComRx.FlushTransmitter();

		//Reset the crap to factory
				//	FactoryResetXbee(&SerialComTx);
				//	FactoryResetXbee(&SerialComRx);

	//setting the props of various other paras on the xbee
		//SetRO(&SerialComTx);
		//SetRO(&SerialComRx);

		//First reset the module
	        //ResetXbee(&SerialComTx);
	        //ResetXbee(&SerialComRx);


//Set baudrate to 115200
		    //SetBaudRateXbee(&SerialComTx);
			//SetBaudRateXbee(&SerialComRx);
}


/*
 * Onboard just send [t,w,a] to ground. Does not receive anything
 */
void Serial_comm_Ground_Onboard::Onboard_SimpleSend(QuadStateVariable * StateData){

		X[0]=StateData->t;

		X[1]=StateData->w_meas[0];	X[2]=StateData->w_meas[1];	X[3]=StateData->w_meas[2];

		X[4]=StateData->a_meas[0];	X[5]=StateData->a_meas[1];	X[6]=StateData->a_meas[2];

		floatfields2CharArray(X,7,namebuf,500, 0 );
cout<<namebuf<<endl;
		// write to ground station
			Ret=SerialComTx.WriteString(namebuf);
			    		if (Ret!=1)
			    		{printf ("Error while writing data\n");}


}
/*
 * Onboard just receives [t,x,q]. Nothing is sent in return
 */
void Serial_comm_Ground_Onboard::Onboard_SimpleRecieve(int option,QuadStateVariable * StateData,double * Dvicondata){


	Ret=SerialComRx.ReadString(namebuf,'\n',500,7);
			if (Ret<=0){
				cout<<"Something wrong with reading from ground station: IMBA coder"<<endl;
	        }
cout<<"Ret = "<<Ret<<endl;
			cout<<namebuf;
			int n=CharArray2floatfields(X,namebuf, Ret );



			switch ((int)floor(X[0]+0.5)) {
				case 8:
					                if(n==9){
											Dvicondata[0]=X[1];
							    		    Dvicondata[1]=X[2];	Dvicondata[2]=X[3];	Dvicondata[3]=X[4];
							    		    Dvicondata[4]=X[5];	Dvicondata[5]=X[6];	Dvicondata[6]=X[7];	Dvicondata[7]=X[8];

							    		    StateData->Update_Vicon_meas(Dvicondata,option);
							    		    StateData->VICONupdateFLAG=1;
										}
					                	else
							    		    cout<<"Imba coder... need 8 fields from ground ,... no update"<<endl;

				  break;
				case 9:
					//    updates xref

				  break;
				default:
					cout<<"Option not found on simple receive onboard ... no update"<<endl;
					//SerialComRx.FlushReceiver();
					return;
					break;
				}
}


/*
 * Ground just send [t,x,q]
 */
void Serial_comm_Ground_Onboard::GroundStation_SimpleSend(int option,QuadStateVariable * StateData){

				X[0]=StateData->t;
		    	X[1]=StateData->x_meas[0];	X[2]=StateData->x_meas[1];	X[3]=StateData->x_meas[2];
		    	X[4]=StateData->q_meas[0];	X[5]=StateData->q_meas[1];	X[6]=StateData->q_meas[2]; X[7]=StateData->q_meas[3];

		    	floatfields2CharArray(X,8,namebuf,500, option);
		    	// write to ground station
		    				Ret=SerialComTx.WriteString(namebuf);
		    				    		if (Ret!=1)
		    				    		{printf ("################################################Error while writing data\n");}
}

/*
 * Ground just receives [t,w,a]
 */
void Serial_comm_Ground_Onboard::GroundStation_SimpleRecieve(int opt,QuadStateVariable * StateData,double *Daccelgyro){

	//double Daccelgyro[7];
//	cout<<"reading received string"<<endl;
	Ret=SerialComRx.ReadString(namebuf,'\n',500,10);
				if (Ret<=30|| Ret>=66){
					cout<<"Something wrong with reading from ground station: skipping update"<<endl;
					return;
		        }
				//cout<<"Ret = "<<Ret<<endl;
cout<<"namebuf = "<<namebuf<<endl;

				int n=CharArray2floatfields(X,namebuf, Ret );
//cout<<"Ret = "<<Ret<<"    n = "<<n<<endl;

if(n!=7){
	cout<<"received string does not have requried float fields"<<endl;
	return;
}
						 Daccelgyro[0]=X[0];
			    		 Daccelgyro[1]=X[4]; Daccelgyro[2]=X[5];  Daccelgyro[3]=X[6];
			    		 Daccelgyro[4]=X[1]; Daccelgyro[5]=X[2];  Daccelgyro[6]=X[3];

			    		 StateData->Update_AccelGyro_meas(Daccelgyro,opt);

}


/*
* request data from ground station for x and q.
* request is initiated by sending raw w_meas and a_meas to ground station
* send options:                                    [in return we get]
*           [1 t w_meas a_meas] for only return on [t x_meas q_meas]
*           [2 t w_meas a_meas] for only return on [t x_meas q_meas Fd a wdot qref]
*           [3 t w_meas a_meas] for only return on [t x_meas q_meas Fd a wdot ba bg qref]
*           [4 t w_meas a_meas] for only return on [t x_meas q_meas Fd a wdot ba bg rmis qmis qref]
*           [5 t w_meas a_meas w_est q_est wdot_est x_est v_est a_est] for only return on [t x_meas q_meas]
*           [6] for only return on [t x_meas q_meas]  //in this mode everything is done onboard. only vicon input is required
*           [7 t w_meas a_meas] for only return on [u] // this mode, everything is done on ground and only control inputs are sent here
*/

void Serial_comm_Ground_Onboard::Request_GroundStationData(int option,QuadStateVariable * StateData){
double Dvicondata[8],v3[3],v4[4],v;
	X[0]=StateData->t;

	X[1]=StateData->w_meas[0];	X[2]=StateData->w_meas[1];	X[3]=StateData->w_meas[2];

	X[4]=StateData->a_meas[0];	X[5]=StateData->a_meas[1];	X[6]=StateData->a_meas[2];

	X[7]=StateData->w[0];	X[8]=StateData->w[1];	X[9]=StateData->w[2];

	X[10]=StateData->q[0];	X[11]=StateData->q[1];	X[12]=StateData->q[2];	X[13]=StateData->q[3];

	X[14]=StateData->wdot[0];	X[15]=StateData->wdot[1];	X[16]=StateData->wdot[2];

	X[17]=StateData->x[0];	X[18]=StateData->x[1];	X[19]=StateData->x[2];

	X[20]=StateData->v[0];	X[21]=StateData->v[1];	X[22]=StateData->v[2];

	X[23]=StateData->a[0];	X[24]=StateData->a[1];	X[25]=StateData->a[2];

	switch (option) {
	case 1:
		floatfields2CharArray(X,7,namebuf,500, 1 );
	  break;
	case 2:
		floatfields2CharArray(X,7,namebuf,500, 2 );
	  break;
	case 3:
		floatfields2CharArray(X,7,namebuf,500, 3 );
	  break;
	case 4:
		floatfields2CharArray(X,7,namebuf,500, 4 );
	  break;
	case 5:
		floatfields2CharArray(X,26,namebuf,500, 5 );
	  break;
	case 6:
		snprintf(namebuf, 10, "%d\n",6);
		  break;
	case 7:
			floatfields2CharArray(X,7,namebuf,500, 7 );
		  break;
	default:
	  // Code
	  cout<<"Ground communication option not available"<<endl;
	  return;
		break;
	}

// write to ground station
	Ret=SerialComTx.WriteString(namebuf);
	    		if (Ret!=1)
	    		{printf ("Error while writing data\n");}

// Read returned data from ground station
	bzero(namebuf,300);
	Ret=0;

	ReturnDataTimer.ResetTimer();
    while(SerialComRx.Peek()<=0 && ReturnDataTimer.CheckTimer()){}  //always wait for the buffer to recieve data...or else read() on linux will give crazy data

		Ret=SerialComRx.ReadString(namebuf,'\n',500,10);
		if (Ret<=0){
			cout<<"Something wrong with reading from ground station: IMBA coder"<<endl;
        }

    //converting data to floats and store into data

   // cout<<namebuf<<endl;
    int n=CharArray2floatfields(X,namebuf, Ret );
    //cout<<"n = "<<n<<"  and Ret = "<<Ret <<endl;

    switch (option) {
    	case 1:
    		    //StateData->tground=X[0];
    		    //StateData->x_meas[0]=X[1];	StateData->x_meas[1]=X[2];	StateData->x_meas[2]=X[3];
    		    //StateData->q_meas[0]=X[4];	StateData->q_meas[1]=X[5];	StateData->q_meas[2]=X[6]; StateData->q_meas[3]=X[7];
    		if(n==8)
    		{

    		    Dvicondata[0]=X[0];
    		    Dvicondata[1]=X[1];	Dvicondata[2]=X[2];	Dvicondata[3]=X[3];
    		    Dvicondata[4]=X[4];	Dvicondata[5]=X[5];	Dvicondata[6]=X[6];	Dvicondata[7]=X[7];

    		    StateData->Update_Vicon_meas(Dvicondata,1);
    		}
    		else
    			cout<<"Recieved data incompatible: option 1 needs 8 fields"<<endl;

    	  break;
    	case 2:
    		//StateData->tground=X[0];
    		//StateData->x_meas[0]=X[1];	StateData->x_meas[1]=X[2];	StateData->x_meas[2]=X[3];
    		//StateData->q_meas[0]=X[4];	StateData->q_meas[1]=X[5];	StateData->q_meas[2]=X[6]; StateData->q_meas[3]=X[7];

    		if(n==19){

		    Dvicondata[0]=X[0];
		    Dvicondata[1]=X[1];	Dvicondata[2]=X[2];	Dvicondata[3]=X[3];
		    Dvicondata[4]=X[4];	Dvicondata[5]=X[5];	Dvicondata[6]=X[6];	Dvicondata[7]=X[7];


		    StateData->Update_Vicon_meas(Dvicondata,1);

		   StateData->Update_Fd(X[8]);

		    v3[0]=X[9]; v3[1]=X[10]; v3[2]=X[11];  StateData->Update_a(v3);

		    v3[0]=X[12]; v3[1]=X[13]; v3[2]=X[14];  StateData->Update_wdot(v3);

		    v4[0]=X[15]; v4[1]=X[16]; v4[2]=X[17]; v4[3]=X[18]; StateData->Update_qref(v4);


    		//StateData->a[0]=X[9]; StateData->a[1]=X[10]; StateData->a[2]=X[11];
    		//StateData->wdot[0]=X[12]; StateData->wdot[1]=X[13]; StateData->wdot[2]=X[14];
    		//StateData->qref[0]=X[15]; StateData->qref[1]=X[16]; StateData->qref[2]=X[17]; StateData->qref[3]=X[18];
    		}
    		else
    		    			cout<<"Recieved data incompatible: option 2 needs 19 fields"<<endl;


    	  break;
    	case 3:
    		//StateData->tground=X[0];
    		//StateData->x_meas[0]=X[1];	StateData->x_meas[1]=X[2];	StateData->x_meas[2]=X[3];
    		//StateData->q_meas[0]=X[4];	StateData->q_meas[1]=X[5];	StateData->q_meas[2]=X[6]; StateData->q_meas[3]=X[7];
    		if(n==21){

		    Dvicondata[0]=X[0];
		    Dvicondata[1]=X[1];	Dvicondata[2]=X[2];	Dvicondata[3]=X[3];
		    Dvicondata[4]=X[4];	Dvicondata[5]=X[5];	Dvicondata[6]=X[6];	Dvicondata[7]=X[7];

		    StateData->Update_Vicon_meas(Dvicondata,1);

		    StateData->Update_Fd(X[8]);
    		//StateData->Fd=X[8];

		    v3[0]=X[9]; v3[1]=X[10]; v3[2]=X[11];  StateData->Update_a(v3);
		    //StateData->a[0]=X[9]; StateData->a[1]=X[10]; StateData->a[2]=X[11];

		    v3[0]=X[12]; v3[1]=X[13]; v3[2]=X[14];  StateData->Update_wdot(v3);
		    //StateData->wdot[0]=X[12]; StateData->wdot[1]=X[13]; StateData->wdot[2]=X[14];

		    v4[0]=X[21]; v4[1]=X[22]; v4[2]=X[23]; v4[3]=X[24]; StateData->Update_qref(v4);
		    //StateData->qref[0]=X[21]; StateData->qref[1]=X[22]; StateData->qref[2]=X[23]; StateData->qref[3]=X[24];

		    v3[0]=X[15]; v3[1]=X[16]; v3[2]=X[17];  StateData->Update_ba(v3);
    		//StateData->ba[0]=X[15]; StateData->ba[1]=X[16]; StateData->ba[2]=X[17];

    		v3[0]=X[18]; v3[1]=X[19]; v3[2]=X[20];  StateData->Update_bg(v3);
    		//StateData->bg[0]=X[18]; StateData->bg[1]=X[19]; StateData->bg[2]=X[20];
    		}
    		else
    		    			cout<<"Recieved data incompatible: option 3 needs 21 fields"<<endl;


    	  break;
    	case 4:
    		//StateData->tground=X[0];
    		//StateData->x_meas[0]=X[1];	StateData->x_meas[1]=X[2];	StateData->x_meas[2]=X[3];
    		//StateData->q_meas[0]=X[4];	StateData->q_meas[1]=X[5];	StateData->q_meas[2]=X[6]; StateData->q_meas[3]=X[7];

    		if(n==32){

		    Dvicondata[0]=X[0];
		    Dvicondata[1]=X[1];	Dvicondata[2]=X[2];	Dvicondata[3]=X[3];
		    Dvicondata[4]=X[4];	Dvicondata[5]=X[5];	Dvicondata[6]=X[6];	Dvicondata[7]=X[7];

		    StateData->Update_Vicon_meas(Dvicondata,1);


    		//StateData->Fd=X[8];
		    StateData->Update_Fd(X[8]);

		    v3[0]=X[9]; v3[1]=X[10]; v3[2]=X[11];  StateData->Update_a(v3);
    		//StateData->a[0]=X[9]; StateData->a[1]=X[10]; StateData->a[2]=X[11];

    		v3[0]=X[12]; v3[1]=X[13]; v3[2]=X[14];  StateData->Update_wdot(v3);
    		//StateData->wdot[0]=X[12]; StateData->wdot[1]=X[13]; StateData->wdot[2]=X[14];

    		//StateData->ba[0]=X[15]; StateData->ba[1]=X[16]; StateData->ba[2]=X[17];
    		v3[0]=X[15];v3[1]=X[16];v3[2]=X[17];
    		StateData->Update_ba(v3);

    		//StateData->bg[0]=X[18]; StateData->bg[1]=X[19]; StateData->bg[2]=X[20];
    		v3[0]=X[18];v3[1]=X[19];v3[2]=X[20];
    		StateData->Update_bg(v3);

    		//StateData->rmis[0]=X[21]; StateData->rmis[1]=X[22]; StateData->rmis[2]=X[23];
    		v3[0]=X[21];v3[1]=X[22];v3[2]=X[23];
    		StateData->Update_rmis(v3);

    		//StateData->qmis[0]=X[24]; StateData->qmis[1]=X[25]; StateData->qmis[2]=X[26]; StateData->qmis[2]=X[27];
    		v4[0]=X[24];v4[1]=X[25];v4[2]=X[26];v4[3]=X[27];
    		StateData->Update_qmis(v4);

    		//StateData->qref[0]=X[28]; StateData->qref[1]=X[29]; StateData->qref[2]=X[30]; StateData->qref[3]=X[31];
    		v4[0]=X[28];v4[1]=X[29];v4[2]=X[30];v4[3]=X[31];
    		StateData->Update_qref(v4);
    		}
    		else
    		    			cout<<"Recieved data incompatible: option 4 needs 32 fields"<<endl;


    	  break;
    	case 5:
    		//StateData->tground=X[0];
    		//StateData->x_meas[0]=X[1];	StateData->x_meas[1]=X[2];	StateData->x_meas[2]=X[3];
    		//StateData->q_meas[0]=X[4];	StateData->q_meas[1]=X[5];	StateData->q_meas[2]=X[6]; StateData->q_meas[3]=X[7];

    		if(n==8){
		    Dvicondata[0]=X[0];
		    Dvicondata[1]=X[1];	Dvicondata[2]=X[2];	Dvicondata[3]=X[3];
		    Dvicondata[4]=X[4];	Dvicondata[5]=X[5];	Dvicondata[6]=X[6];	Dvicondata[7]=X[7];

		    StateData->Update_Vicon_meas(Dvicondata,1);
    		}
    		else
    		    			cout<<"Recieved data incompatible: option 5 needs 8 fields"<<endl;


    	  break;
    	case 6:
    		//StateData->tground=X[0];
    		//StateData->x_meas[0]=X[1];	StateData->x_meas[1]=X[2];	StateData->x_meas[2]=X[3];
    		//StateData->q_meas[0]=X[4];	StateData->q_meas[1]=X[5];	StateData->q_meas[2]=X[6]; StateData->q_meas[3]=X[7];

    		if(n==8){

		    Dvicondata[0]=X[0];
		    Dvicondata[1]=X[1];	Dvicondata[2]=X[2];	Dvicondata[3]=X[3];
		    Dvicondata[4]=X[4];	Dvicondata[5]=X[5];	Dvicondata[6]=X[6];	Dvicondata[7]=X[7];


		    StateData->Update_Vicon_meas(Dvicondata,1);
    		}
    		else
    		    			cout<<"Recieved data incompatible: option 6 needs 8 fields"<<endl;


    	  break;
    	case 7://only control inputs are received
    		//StateData->u[0]=X[0];	StateData->u[1]=X[1];	StateData->u[2]=X[2]; StateData->u[3]=X[3];
    		if(n==4){
    		StateData->Update_u(X);
    		}
    		else
    		    			cout<<"Recieved data incompatible: option 7 needs 4 fields"<<endl;


    	  break;
    	default:
    	  // Code
    	  cout<<"Ground communication option not available"<<endl;
    	  return;
    		break;
    	}


}

/* Runs on Ground
 * the onboard controller request data by first sending the option number follwed by the onboard data
 * So structure the data as per the option and send it.
 * So first we have to read the data and then send it.
 */
void Serial_comm_Ground_Onboard::Fulfil_OnboardRequest(QuadStateVariable * StateData){
int option;
double Daccelgyro[7];
	//first read from the onboard controller
	bzero(namebuf,300);
		Ret=0;
		Ret=SerialComRx.ReadString(namebuf,'\n',500,10);
		if (Ret<=0){
			cout<<"Something wrong with reading from onboard : Learn to code dude"<<endl;
        }

	    // split them into respective fields

	    int n=CharArray2floatfields(X,namebuf, Ret );

	    option =(int)floor(X[0]+0.5);

	    switch (option) {
	          case 1:
	        	  //StateData->t=X[1];
	        	  //StateData->w_meas[0]=X[2];StateData->w_meas[1]=X[3];StateData->w_meas[2]=X[4];
	        	  //StateData->a_meas[0]=X[5];StateData->a_meas[1]=X[6];StateData->a_meas[2]=X[7];

	        	  if(n==8){
	        	  Daccelgyro[0]=X[1];
	        	  Daccelgyro[1]=X[5]; Daccelgyro[2]=X[6];  Daccelgyro[3]=X[7];
	        	  Daccelgyro[4]=X[2]; Daccelgyro[5]=X[3];  Daccelgyro[6]=X[4];

	        	  StateData->Update_AccelGyro_meas(Daccelgyro,1);
	        	  }
	        	  else
	        	      	cout<<"Recieved data incompatible: option 1 needs 8 fields"<<endl;

	       	  break;
	          case 2:
	          	        	  //StateData->t=X[1];
	          	        	 // StateData->w_meas[0]=X[2];StateData->w_meas[1]=X[3];StateData->w_meas[2]=X[4];
	          	        	  //StateData->a_meas[0]=X[5];StateData->a_meas[1]=X[6];StateData->a_meas[2]=X[7];
	        	  	  	  	  if(n==8){
	        	        	  Daccelgyro[0]=X[1];
	        	        	  Daccelgyro[1]=X[5]; Daccelgyro[2]=X[6];  Daccelgyro[3]=X[7];
	        	        	  Daccelgyro[4]=X[2]; Daccelgyro[5]=X[3];  Daccelgyro[6]=X[4];

	        	        	  StateData->Update_AccelGyro_meas(Daccelgyro,1);
	        	  	  	  	  }
	        	  	  	  else
	        	  	  	      			cout<<"Recieved data incompatible: option 2 needs 8 fields"<<endl;

	          	       	  break;
	          case 3:
	          	        	 // StateData->t=X[1];
	          	        	 // StateData->w_meas[0]=X[2];StateData->w_meas[1]=X[3];StateData->w_meas[2]=X[4];
	          	        	 // StateData->a_meas[0]=X[5];StateData->a_meas[1]=X[6];StateData->a_meas[2]=X[7];

	        	  	  	  	  if(n==8){
	        	        	  Daccelgyro[0]=X[1];
	        	        	  Daccelgyro[1]=X[5]; Daccelgyro[2]=X[6];  Daccelgyro[3]=X[7];
	        	        	  Daccelgyro[4]=X[2]; Daccelgyro[5]=X[3];  Daccelgyro[6]=X[4];

	        	        	  StateData->Update_AccelGyro_meas(Daccelgyro,1);
	        	  	  	  	  }
	        	  	  	  else
	        	  	  	      			cout<<"Recieved data incompatible: option 3 needs 8 fields"<<endl;

	          	       	  break;
	          case 4:
	          	        	  //StateData->t=X[1];
	          	        	  //StateData->w_meas[0]=X[2];StateData->w_meas[1]=X[3];StateData->w_meas[2]=X[4];
	          	        	  //StateData->a_meas[0]=X[5];StateData->a_meas[1]=X[6];StateData->a_meas[2]=X[7];

	        	  	  	  	  if(n==8){

	        	        	  Daccelgyro[0]=X[1];
	        	        	  Daccelgyro[1]=X[5]; Daccelgyro[2]=X[6];  Daccelgyro[3]=X[7];
	        	        	  Daccelgyro[4]=X[2]; Daccelgyro[5]=X[3];  Daccelgyro[6]=X[4];

	        	        	  StateData->Update_AccelGyro_meas(Daccelgyro,1);
	        	  	  	  	  }
	        	  	  	  else
	        	  	  	      			cout<<"Recieved data incompatible: option 4 needs 8 fields"<<endl;



	          	       	  break;
	          case 5:  //[5 t w_meas a_meas w_est q_est wdot_est x_est v_est a_est]
	          	        	  //StateData->t=X[1];
	          	        	  //StateData->w_meas[0]=X[2];StateData->w_meas[1]=X[3];StateData->w_meas[2]=X[4];
	          	        	  //StateData->a_meas[0]=X[5];StateData->a_meas[1]=X[6];StateData->a_meas[2]=X[7];

	        	  if(n==27){



	        		          StateData->w[0]=X[8];StateData->w[1]=X[9];StateData->w[2]=X[10];
	          	        	  StateData->q[0]=X[11];StateData->q[1]=X[12];StateData->q[2]=X[13];StateData->q[3]=X[14];
	          	        	  StateData->wdot[0]=X[15];StateData->wdot[1]=X[16];StateData->wdot[2]=X[17];
	          	        	  StateData->x[0]=X[18];StateData->x[1]=X[19];StateData->x[2]=X[20];
	          	        	  StateData->v[0]=X[21];StateData->v[1]=X[22];StateData->v[2]=X[23];
	          	        	  StateData->a[0]=X[24];StateData->a[1]=X[25];StateData->a[2]=X[26];


	        	        	  Daccelgyro[0]=X[1];
	        	        	  Daccelgyro[1]=X[5]; Daccelgyro[2]=X[6];  Daccelgyro[3]=X[7];
	        	        	  Daccelgyro[4]=X[2]; Daccelgyro[5]=X[3];  Daccelgyro[6]=X[4];

	        	        	  StateData->Update_AccelGyro_meas(Daccelgyro,1);
	        	  }
	        	  else
	        	      			cout<<"Recieved data incompatible: option 5 needs 27 fields"<<endl;


	          	       	  break;
	          case 6:
	        	  	  	  if(n==1){

	        	  	  	  }

	        	  	  		else
	        	  	  		    cout<<"Recieved data incompatible: option 1 needs 8 fields"<<endl;


	          	       	  break;
	          case 7:
	          	        	  //StateData->t=X[1];
	          	        	  //StateData->w_meas[0]=X[2];StateData->w_meas[1]=X[3];StateData->w_meas[2]=X[4];
	          	        	  //StateData->a_meas[0]=X[5];StateData->a_meas[1]=X[6];StateData->a_meas[2]=X[7];

	        	  	  	  	  if(n==8){
	        	        	  Daccelgyro[0]=X[1];
	        	        	  Daccelgyro[1]=X[5]; Daccelgyro[2]=X[6];  Daccelgyro[3]=X[7];
	        	        	  Daccelgyro[4]=X[2]; Daccelgyro[5]=X[3];  Daccelgyro[6]=X[4];

	        	        	  StateData->Update_AccelGyro_meas(Daccelgyro,1);
	        	  	  	  	  }
	        	  	  	  else
	        	  	  	      			cout<<"Recieved data incompatible: option 7 needs 8 fields"<<endl;

	        	        	  break;
	          default:
	       	  // Code
	       	  cout<<"Onboard requested option not available"<<endl;
	       	return;
	       	  break;

	    }

	    //Now that we received the data from the onboard controller, send the requested data back
	    bzero(namebuf,300);

	        X[0]=StateData->t;
	    	X[1]=StateData->x_meas[0];	X[2]=StateData->x_meas[1];	X[3]=StateData->x_meas[2];
	    	X[4]=StateData->q_meas[0];	X[5]=StateData->q_meas[1];	X[6]=StateData->q_meas[2]; X[7]=StateData->q_meas[3];


	    switch (option) {
	    	          case 1:  //[t x_meas q_meas]
	    	        	  floatfields2CharArray(X,8,namebuf,500, 0);
	    	        	  	  	  break;

	    	          case 2:   //[t x_meas q_meas Fd a wdot qref]
	    	        	  X[8]=StateData->Fd;
	    	        	  X[9]=StateData->a[0]; X[10]=StateData->a[1]; X[11]=StateData->a[2];
	    	        	  X[12]=StateData->wdot[0]; X[13]=StateData->wdot[1]; X[14]=StateData->wdot[2];
	    	        	  X[15]=StateData->qref[0]; X[16]=StateData->qref[1]; X[17]=StateData->qref[2]; X[18]=StateData->qref[3];

	    	        	  floatfields2CharArray(X,19,namebuf,500, 0 );
	    	          	       	  break;
	    	          case 3:   //[t x_meas q_meas Fd a wdot ba bg qref]
	    	        	  	  	  	  	  	  	  X[8]=StateData->Fd;
	    	        	 	    	        	  X[9]=StateData->a[0]; X[10]=StateData->a[1]; X[11]=StateData->a[2];
	    	        	 	    	        	  X[12]=StateData->wdot[0]; X[13]=StateData->wdot[1]; X[14]=StateData->wdot[2];
	    	        	 	    	        	  X[15]=StateData->ba[0]; X[16]=StateData->ba[1]; X[17]=StateData->ba[2];
	    	        	 	    	              X[18]=StateData->bg[0]; X[19]=StateData->bg[1]; X[20]=StateData->bg[2];
	    	        	 	    	        	  X[21]=StateData->qref[0]; X[22]=StateData->qref[1]; X[23]=StateData->qref[2]; X[24]=StateData->qref[3];

	    	        	 	    	        	  floatfields2CharArray(X,25,namebuf,500, 0);
	    	          	       	  break;
	    	          case 4:   //[t x_meas q_meas Fd a wdot ba bg rmis qmis qref]
  	  	  	  	  	  	  X[8]=StateData->Fd;
 	    	        	  X[9]=StateData->a[0]; X[10]=StateData->a[1]; X[11]=StateData->a[2];
 	    	        	  X[12]=StateData->wdot[0]; X[13]=StateData->wdot[1]; X[14]=StateData->wdot[2];
 	    	        	  X[15]=StateData->ba[0]; X[16]=StateData->ba[1]; X[17]=StateData->ba[2];
 	    	              X[18]=StateData->bg[0]; X[19]=StateData->bg[1]; X[20]=StateData->bg[2];
 	    	              X[21]=StateData->rmis[0]; X[22]=StateData->rmis[1]; X[23]=StateData->rmis[2];
 	    	              X[24]=StateData->qmis[0]; X[25]=StateData->qmis[1]; X[26]=StateData->qmis[2]; X[27]=StateData->qmis[3];
 	    	        	  X[28]=StateData->qref[0]; X[29]=StateData->qref[1]; X[30]=StateData->qref[2]; X[31]=StateData->qref[3];

 	    	        	  floatfields2CharArray(X,32,namebuf,500, 0);
	    	          	       	  break;
	    	          case 5:  //[t x_meas q_meas]
	    	        	  floatfields2CharArray(X,8,namebuf,500, 0 );

	    	          	       	  break;
	    	          case 6:  //[t x_meas q_meas]
	    	        	   	  floatfields2CharArray(X,8,namebuf,500, 0 );
	    	          	       	  break;
	    	          case 7: //u
	    	          	        	  X[0]=StateData->u[0]; X[1]=StateData->u[1]; X[2]=StateData->u[2]; X[3]=StateData->u[3];
	    	          	        	  floatfields2CharArray(X,4,namebuf,500, 0 );
	    	          	       	  break;
	    	          default:
	    	       	  // Code
	    	       	  cout<<"Onboard requested option not available"<<endl;
	    	       	return;
	    	       	  break;

	    	    }
        //cout<<X[0]<<" "<<X[1]<<" "<<X[2]<<" "<<X[3]<<" "<<X[4]<<" "<<X[5]<<" "<<X[6]<<" "<<X[7]<<endl;

	    //snprintf(namebuf, 500, "%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",X[0], X[1], X[2],X[3],X[4],X[5],X[6],X[7]);
	    //snprintf(namebuf, 500, "%d %d %d %d %d %d %d %d\n",11111, 22222, 333333,44444,555555,666666,766666,866666);
	    //cout<<namebuf<<endl;
	    //cout<<namebuf<<endl;
	    // write to onboard

	    	Ret=SerialComTx.WriteString(namebuf);
	    	    		if (Ret!=1)
	    	    		{printf ("Error while writing data\n");}

}

/* Ground station initiates the communication by requesting data. By sending the control u ..... we get back [t, w_meas a_meas]
 * By sending this option... GROUND STATION BECOMES THE MASTER
* send options:                                    [in return we get]
*           [8 u] for only return on [t w_meas a_meas]
*
 */
//Ground Station Requests
void Serial_comm_Ground_Onboard::Request_OnboardData(int option,QuadStateVariable * StateData){

	    X[0]=StateData->u[0]; X[1]=StateData->u[1];	X[2]=StateData->u[2];	X[3]=StateData->u[3];


		switch (option) {
		case 8:
			floatfields2CharArray(X,4,namebuf,500, 8 );
		  break;
		default:
		  // Code
		  cout<<"Onboard Request from Ground Station haha lol lol option not available"<<endl;
			break;
		}
cout<<"Writing the control = "<<namebuf<<endl;

		// write to onboard
			Ret=SerialComTx.WriteString(namebuf);
			    		if (Ret!=1)
			    		{printf ("Error while writing data\n");}

			    		//SerialCom.Fsyncit();

			    		totalpackets++;


bzero(namebuf,100);
usleep(500);
			ReturnDataTimer.ResetTimer();
			while(SerialComRx.Peek()<=0 && ReturnDataTimer.CheckTimer()){}

		// Read returned data from onboard controller
			Ret=0;
			Ret=SerialComRx.ReadString(namebuf,'\n',500,10);
		if (Ret<=0){
		cout<<"Something wrong with reading from onboard controller: IMBA coder"<<endl;
		lostpackets++;
		SerialComRx.FlushReceiver();
		return;
		}
cout<<"Received chars = "<<Ret<<endl;



		    //converting data to floats and store into data

		    int n=CharArray2floatfields(X,namebuf, Ret );
		   cout<<namebuf<<endl;

double Daccelgyro[7];

		    switch (option) {
		    	case 8:
		    		//StateData->t=X[0];
		    		//StateData->w_meas[0]=X[1];	StateData->w_meas[1]=X[2];	StateData->w_meas[2]=X[3];
		    		//StateData->a_meas[0]=X[4];	StateData->a_meas[1]=X[5];	StateData->a_meas[2]=X[6];

		    		if(n==7){
		    		 Daccelgyro[0]=X[0];
		    		 Daccelgyro[1]=X[4]; Daccelgyro[2]=X[5];  Daccelgyro[3]=X[6];
		    		 Daccelgyro[4]=X[1]; Daccelgyro[5]=X[2];  Daccelgyro[6]=X[3];

		    		 StateData->Update_AccelGyro_meas(Daccelgyro,1);
		    		}
		    		else
		    			{cout<<"Received data from onboard not compatible: option 8 needs 7 fields"<<endl;
		    			lostpackets++;
		    			}

		    	  break;

		    	default:
		    	  // Code
		    	  cout<<"Onboard lol lol option not available"<<endl;
		    	  return;
		    	  break;
		    	}


}
//Onboard  controllers fulfils the GroundStation (MASTER) Request....This code runs on the onboard controller
void Serial_comm_Ground_Onboard::Fulfil_GroundStationRequest(QuadStateVariable * StateData){
	int option;
	double v4[4];
		//first read from the ground station
		bzero(namebuf,100);
			Ret=0;

			totalpackets++;


				Ret=SerialComRx.ReadString(namebuf,'\n',500,5);
				if (Ret<=0){
		            cout<<"could not read anything from the ground station"<<endl;
		            lostpackets++;
		            SerialComRx.FlushReceiver();
					return;
				}

		    // split them into respective fields
		    int n=CharArray2floatfields(X,namebuf, Ret );
		    cout<<"Recieved from Ground station = "<<namebuf<<endl;

		    option =(int)floor(X[0]+0.5);

		    switch (option) {
		    	          case 8: //receive  [u]
		    	        	  //StateData->u[0]=X[1]; StateData->u[1]=X[2]; StateData->u[2]=X[3]; StateData->u[3]=X[4];
                         if(n==5){
		    	        	  v4[0]=X[1];
		    	        	  v4[1]=X[2];
		    	        	  v4[2]=X[3];
		    	        	  v4[3]=X[4];

		    	        	  StateData->Update_u(v4);
                         }
                         else
                         {			cout<<"Received data from ground station not compatible: option 8 needs 5 fields"<<endl;
                                    lostpackets++;
                         }
		    	       	  break;

		    	          default:
		    	       	  // Code
		    	       	  cout<<"Onboard received data option from master ground station not available"<<endl;
		    	       	return;
		    	       	  break;

		    	    }

		    	    //Now that we received the data from the onboard controller, send the requested data back
		    	    bzero(namebuf,100);

		    	        X[0]=StateData->t;
		    	    	X[1]=StateData->w_meas[0];	X[2]=StateData->w_meas[1];	X[3]=StateData->w_meas[2];
		    	    	X[4]=StateData->a_meas[0];	X[5]=StateData->a_meas[1];	X[6]=StateData->a_meas[2];


		    	    switch (option) {
		    	    	          case 8:  // send [t w_meas a_meas]
		    	    	        	  floatfields2CharArray(X,7,namebuf,500, 0 );
		    	    	        	  	  	  break;

		    	    	          default:
		    	    	       	  // Code
		    	    	       	  cout<<"Onboard fulfilment to ground master request in trouble option not available"<<endl;
		    	    	       	return;
		    	    	       	  break;
		    	    	    }
		    	    // write to ground station
		    	    	Ret=SerialComTx.WriteString(namebuf);
		    	    	    		if (Ret!=1)
		    	    	    		{printf ("Error while writing data\n");}
		   	    cout<<"Done sending data = "<<namebuf<<endl;
}
