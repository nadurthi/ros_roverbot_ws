/*
 * UDPcommGroundOnboard.cpp
 *
 *  Created on: May 21, 2014
 *      Author: nagnanamus
 */

#include "UDPcommGroundOnboard.h"


void UDPcommGroundOnboard::Init_as_Ground(){
	char addr[50];
	sprintf(addr,"%s",GroundTx_ip);
	TxUDP.OpenPort_write(addr,GroundTx_port);
	sprintf(addr,"%s",GroundRx_ip);
	RxUDP.OpenPort_read(addr,GroundRx_port);

}

void UDPcommGroundOnboard::Init_as_Onboard(){
	char addr[50];
	sprintf(addr,"%s",OnboardTx_ip);
	TxUDP.OpenPort_write(addr,OnboardTx_port);
	sprintf(addr,"%s",OnboardRx_ip);
	RxUDP.OpenPort_read(addr,OnboardRx_port);

}

bool UDPcommGroundOnboard::CheckRxData(int n){
		if(RxUDP.CheckData()>=n)
			return true;
		else
			return false;
	}

void UDPcommGroundOnboard::Send2Ground(int option,QuadStateVariable * StateData){

}

//*******************************8

void UDPcommGroundOnboard::SendData2Ground(char * msg,QuadStateVariable * StateData){
	int m=StateData->MsgEncoder(buffer,msg);
	TxUDP.write(buffer,m);

}
void UDPcommGroundOnboard::ReceiveData_on_Onboard(QuadStateVariable * StateData,int option){
	    bzero(buffer,200);
		int Ret=RxUDP.ReadPort(buffer,200);
		StateData->MsgDecoder(buffer,option);



}

void UDPcommGroundOnboard::SendData2Onboard(char * msg,QuadStateVariable * StateData){

	int m=StateData->MsgEncoder(buffer,msg);
	TxUDP.write(buffer,m);
	cout<<buffer<<endl;

}
void UDPcommGroundOnboard::ReceiveData_on_Ground(QuadStateVariable * StateData,int option){
    bzero(buffer,200);
	int Ret=RxUDP.ReadPort(buffer,200);
	StateData->MsgDecoder(buffer,option);
}
//*******************************8


void UDPcommGroundOnboard::Send2Onboard(int option,QuadStateVariable * StateData){

double t;
StateData->tground.GetLastRowQ(&t);
int m;
	switch (option) {
		case 1:
							    X[0]=t;
						    	X[1]=StateData->x_meas[0];	X[2]=StateData->x_meas[1];	X[3]=StateData->x_meas[2];
						    	X[4]=StateData->q_meas[0];	X[5]=StateData->q_meas[1];	X[6]=StateData->q_meas[2]; X[7]=StateData->q_meas[3];

						    	m=floatfields2CharArray(X,8,buffer,500, 1 );
						    	cout<<"Sending Data = "<<buffer;

						    	TxUDP.write(buffer,m);
						    	break;
		case 2:                //this option is mainly to send joystick control data to onboard
							    X[0]=t;
						    	X[1]=StateData->x_meas[0];	X[2]=StateData->x_meas[1];	X[3]=StateData->x_meas[2];
						    	X[4]=StateData->q_meas[0];	X[5]=StateData->q_meas[1];	X[6]=StateData->q_meas[2]; X[7]=StateData->q_meas[3];
						    	X[8]=StateData->qref[0];	X[9]=StateData->qref[1];	X[10]=StateData->qref[2];  X[11]=StateData->qref[3];
						    	X[12]=StateData->kq;
						    	X[13]=StateData->kw;
						    	X[14]=StateData->kv;
						    	X[15]=StateData->xref[0];	X[16]=StateData->xref[1];	X[17]=StateData->xref[2];

						    	m=floatfields2CharArray(X,18,buffer,500, 2 );
						    	cout<<"Sending Data = "<<buffer;

						    	TxUDP.write(buffer,m);
						    	break;

		default:
		  // Code
		  cout<<"Ground to onboard communication option not available"<<endl;
		  return;
			break;
		}




}

void UDPcommGroundOnboard::Receive_on_Ground(QuadStateVariable * StateData){

}

void UDPcommGroundOnboard::Receive_on_Onboard(QuadStateVariable * StateData,double *Dvicondata,int UPDToption){

	bzero(buffer,200);
	int Ret=RxUDP.ReadPort(buffer,200);
	//cout<<"Ret = "<<Ret<<endl;
	//cout<<buffer<<endl;

	//double Dvicondata[8];
	//X[0]=-1;X[1]=-1;X[2]=-1;X[3]=-1;X[4]=-1;X[5]=-1;X[6]=-1;X[7]=-1;X[8]=-1;X[9]=-1;X[10]=-1;
	int n=CharArray2floatfields(X,buffer, Ret );
	//cout<<X[0]<<" "<<X[1]<<" "<<X[2]<<" "<<X[3]<<" "<<X[4]<<" "<<X[5]<<" "<<X[6]<<" "<<X[7]<<" "<<X[8]<<" "<<X[9]<<" "<<X[10]<<endl;

    //cout<<"nn = "<<n<<endl;
		    int option =(int)floor(X[0]+0.5);

		    switch (option) {
		          case 1:

		        	  if(n==9){
		        			Dvicondata[0]=X[1];
		        			Dvicondata[1]=X[2];	Dvicondata[2]=X[3];	Dvicondata[3]=X[4];
		        			Dvicondata[4]=X[5];	Dvicondata[5]=X[6];	Dvicondata[6]=X[7];	Dvicondata[7]=X[8];

		        			StateData->Update_Vicon_meas(Dvicondata,UPDToption);
		        			StateData->VICONupdateFLAG=1;
		        	  }
		        	  else
		        	      	cout<<"Recieved data incompatible: option 1 needs 8 fields"<<endl;

		       	  break;

		          case 2: //option to read additional joystick data
		              	 if(n==19){
		          		      Dvicondata[0]=X[1];
		          		      Dvicondata[1]=X[2];	Dvicondata[2]=X[3];	Dvicondata[3]=X[4];
		          		      Dvicondata[4]=X[5];	Dvicondata[5]=X[6];	Dvicondata[6]=X[7];	Dvicondata[7]=X[8];

		          		      StateData->qref[0]=X[9];StateData->qref[1]=X[10];StateData->qref[2]=X[11];StateData->qref[3]=X[12];
		          		      StateData->kq=X[13];
		          		      StateData->kw=X[14];
		          		      StateData->kv=X[15];
		          		      StateData->xref[0]=X[16];	StateData->xref[1]=X[17];	StateData->xref[2]=X[18];

		          		      StateData->Update_Vicon_meas(Dvicondata,UPDToption);
		          		      StateData->VICONupdateFLAG=1;
		          		         }
		          		  else
		          		      cout<<"Recieved data incompatible: option 2 needs 19 fields"<<endl;

		          break;

		          default:
		          		  cout<<"Ground to onboard reciever option not available"<<endl;
		          		  return;
		          break;

}
}

int UDPcommGroundOnboard::CharArray2floatfields(double *X,char *S, int L ){
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


int UDPcommGroundOnboard::floatfields2CharArray(double *X,int L,char * S,int Slen, int options ){
int m=0;
	if(options==0){
	for(int n=0;n<L-1;n++){
		m=m+snprintf(S+m, Slen, "%010.5f ",X[n]);
    }
	}
	else
	{
		m=m+snprintf(S+m, Slen, "%d ",options);
		for(int n=0;n<L-1;n++){
			m=m+snprintf(S+m, Slen, "%010.5f ",X[n]);
	    }

	}
	m=m+snprintf(S+m, Slen, "%010.5f\n",X[L-1]);
    return m;
}




