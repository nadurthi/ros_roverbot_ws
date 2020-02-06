/*
 * State_data_manager.h
 *
 *  Created on: Apr 19, 2014
 *      Author: nagnanamus
 */

#ifndef STATE_DATA_MANAGER_H_
#define STATE_DATA_MANAGER_H_

#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <stropts.h>
#include <stdio.h>
#include <string>
#include <sys/types.h>
#include <time.h>
#include <sys/time.h>
#include <fstream>
#include <cmath>
#include <pthread.h>
#include<fcntl.h>
#include <sys/stat.h>
#include <errno.h>
#include <stdint.h>

#include <algorithm>


using namespace std;

#include "SimpleTimer.h"


#define data_queue_length 5 //no. of time steps of data to remember( so it can be used for simple moving average filters)








class SimpleQueue {
public:
	double  **a,*stdev,*maxstdev,*mu;
	int rows,columns,currind;


	//dynamically create a Queue and initialize with zeros
	void CreateQ(int rows,int cols){
		currind=0;
		this->rows=rows;
		columns=cols;
		// allocation
		 stdev=new double [cols];
		 maxstdev=new double [cols];
		 mu=new double [cols];

			a = new double*[rows];
			for(int i = 0; i < rows; i++)
				a[i] = new double[cols];

			for(int r=0;r<rows;r++){
				for(int c=0;c<cols;c++){
					a[r][c]=0;
					stdev[c]=0;
					maxstdev[c]=0;
				}
			}
	}

	//dynamically create a Queue and initialize with zeros
	void DestroyQ(){

			for(int i = 0; i < rows; i++)
				delete[] a[i];

	}

	void SeriesFillQ(double *data);

	int * GetSizeQ(int *S){
		S[0]=rows;S[1]=columns;
		return S;
	}
    int CheckSizeQ(int r,int c){
    	if((r<rows && r>=0) && (c<columns && c>=0))
    	return 1;
    	else
    	return -1;
    }

	//append always to the last of queue. D has same columns as Q. The ques is shifted up. First element is lost
    //C is array of column numbers of D to be used
    void AppendQ(double *D,int *C);
    // replace the (r-1)th row...as counting starts from 0
    void ReplaceRowQ(int r,double *D);
    void ReplaceLastRowQ(double *D);

    //Operations along the rows. Each Col is independednt. From row r1 to r2
    // rmemeber r1 and r2 are c++ ref not matlab ref. ie.e counting starts from 0 in c++.
    //if u want  from 5th row to 9th row, r1=4,r2=8; rows r1 and r2 are also included in the calculation
    void MeanQ(int r1, int r2) ;
    void StdQ( int r1, int r2) ; // also updates the corresponding mu

    //get the data out
    double * GetRowQ(int r,double *D);
    double * GetColQ(int c,double *D);
    double * GetLastRowQ(double *D);
    double * GetLastButOneRowQ(double *D);
    double * GetStdQ(double *D){
    	for(int i=0;i<columns;i++)
    		D[i]=stdev[i];

    	return D;
    }
    double * GetVarQ(double *D){
        	for(int i=0;i<columns;i++)
        		D[i]=pow(stdev[i],2);

        	return D;
    }
    double * GetMeanQ(double *D){
            	for(int i=0;i<columns;i++)
            		D[i]=mu[i];

            	return D;
        }
    //Last row Outlier remove and replace with mean computed from row-1 rows
    //if no outlier just replace with all rows mean
    double * AvgOutlierFilterQ(double *D);
    double * AvgOutlierFilterQnorm(double *D); //this function only for quaternions as they have to be normalized

    //Just normalize the array of length columns and return it.
    //this function is specifically for quaternions moving avg filter
    double NormVec(double *D){
    	double mag=0;
    	for(int c=0;c<columns;c++){
    		mag=mag+pow(D[c],2);
    	}
    	for(int c=0;c<columns;c++){
    	    		D[c]=D[c]/mag;
    	}

    	return sqrt(mag);
    }

};




 //%%%%%%%%%%%%%%%%%%%%%%%   States used in your estimator/controller   %%%%%%%%%%%%%%%%%%%%%%
struct QuadStateVariable {

	 ofstream DataRaw,DataRawparas;


	 double t;
	 SimpleQueue tonboard,tground;

	 double q[4],w[3],x[3],v[3],a[3],wdot[3]; //all the current estimated quantities
	 double w_meas[3],a_meas[3],x_meas[3],q_meas[4]; //stores the history of IMU measurements
	 double q_raw[4],a_raw[3],x_raw[3],w_raw[3];
	 // raw data with not conditioning or estimation
	 //double Data_w_meas[3][data_queue_length],Data_a_meas[3][data_queue_length],Data_q_meas[4][data_queue_length],Data_x_meas[3][data_queue_length];
	 SimpleQueue Data_w_meas,Data_a_meas,Data_q_meas,Data_x_meas;

	 // filtered data after conditions and estimation
	 //double Data_w_est[3][data_queue_length],Data_a_est[3][data_queue_length],Data_q_est[4][data_queue_length],Data_x_est[3][data_queue_length];
	 //double Data_v_est[3][data_queue_length],Data_wdot_est[3][data_queue_length];
	 SimpleQueue Data_Vdiff,Data_Vest;


	 double Fd,Fd_prev,Fdnorm_limit,Fsafety; //latest force thrust required
	 int MotorFmax,MotorFmin;

	 //REMEMBER control is of the form [Front,Left,Back,Right] or [+x,+y,-x,-y]
	 double u[4],u_prev[4],ulim;   //control inputs  all units for this rotor thrust control force is in NEWTONS

	 double ba[3],bg[3],rmis[3],qmis[4];//all the constant est parameters
	 double ba_prev[3],bg_prev[3],rmis_prev[3],qmis_prev[4];//prev good values: used when current values make no sense
     double ba_lim,bg_lim,rmis_lim;

     double a_lim,wdot_lim,a_prev[3],wdot_prev[3],ainertmeasprev[3];

     //Setup gains
     double kx,kv,kq,kw;

     //setup reference values
     double qdes[4],qref[4],qref_prev[4],xref[3],xref_prev[3],vref[3],vref_prev[3],aref[3],aref_prev[3];
     double b1ref[3],b1ref_prev[3],wref[3],wref_prev[3];

     //Forces
     double M[3];

 	double mg,Quadmass,MomArm,ctf,I[3];

     //Flags
     int IMUupdateFLAG,VICONupdateFLAG,CONTROLupdateFLAG;

     //SafetyTimer Shutoff when the control is not updated
     SimpleTimer EmergencyShutoff;

 /*
 * Update Fd. The prev Fd is stored in Fd_prev and then Fd is loaded with new value
 */
     bool CheckEmergency_shutoff(){
     	if(EmergencyShutoff.CheckTimer()==false){
    	u[0]=0;
     	u[1]=0;
     	u[2]=0;
     	u[3]=0;
     	return true;
     	}
     	else
     		return false;

     }

	 void Update_Fd(double F){
		 if(F>Fdnorm_limit){
		 		Fd=Fd_prev;
		 		cout<<"Current Fd values is not compatible (greater than Fdnorm_limit): hence using previous Fd"<<endl;
		 	}
		 	else
		 		{
		 		Fd_prev=Fd;
		 		Fd=F;
		 		}

	 }

	 void Update_qmis(double *qv);
	 void Update_qref(double *qv);
	 void Update_ba(double *vv);
	 void Update_bg(double *vv);
	 void Update_rmis(double *vv);
	 void Update_u(double *vu);

	 void Update_a(double *vv);
	 void Update_wdot(double *vv);


void Init_SetZero();

void Update_AccelGyro_meas(double *accelgyro,int option);

void Update_Vicon_meas(double *vicondata,int option);

/*
 * Use this function when ()_meas is alread updated to new value in QuadStates
 */
void Update_x_meas(int option);
void Update_q_meas(int option);
void Update_w_meas(int option);
void Update_a_meas(int option);

void PrintStates();

void Record2buffer(char * buff,int len){

				//snprintf(buff,len, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",t,x_raw[0],x_raw[1],x_raw[2],q_raw[0],q_raw[1],q_raw[2],q_raw[3],a_raw[0],a_raw[1],a_raw[2],w_raw[0],w_raw[1],w_raw[2],x[0],x[1],x[2],q[0],q[1],q[2],q[3],v[0],v[1],v[2],a[0],a[1],a[2],w[0],w[1],w[2],wdot[0],wdot[1],wdot[2],Fd,u[0],u[1],u[2],u[3],M[0],M[1],M[2]);
				snprintf(buff,len, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",t,x_raw[0],x_raw[1],x_raw[2],q_raw[0],q_raw[1],q_raw[2],q_raw[3],a_raw[0],a_raw[1],a_raw[2],w_raw[0],w_raw[1],w_raw[2],x[0],x[1],x[2],q[0],q[1],q[2],q[3],v[0],v[1],v[2],a[0],a[1],a[2],w[0],w[1],w[2],Fd,u[0],u[1],u[2],u[3],M[0],M[1],M[2],qdes[0],qdes[1],qdes[2],qdes[3]);

            	//cout<<"buffer in record func = "<<buff<<endl;
				//DataRaw << namebuf;


}

/*
 * Msg Encoder
 * Send data with msg encoder
 * returns string in S and return number of characters
 * msg='q x v w' etc for example
 * QuadStates
 */
int MsgEncoder(char *S,char *msg);

/*
     * Update QuadStates Directly with received msg
     */
int MsgDecoder(char *S,int option);
int SeparateFields(double *D,char *S,int n);

};//struct end





#endif /* STATE_DATA_MANAGER_H_ */
