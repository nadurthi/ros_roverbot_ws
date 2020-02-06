/*
 * EKF.cpp
 *
 *  Created on: Jun 26, 2014
 *      Author: Valentin
 */

#include "EKF.h"
void EKF::InitializeFilter(){

	//Xatt_prev.resize(6);
	Xatt_prev.setZero();

	//Xatt_est.resize(6);
	Xatt_est.setZero();

	//Xatt.resize(6);
	Xatt.setZero();

	//Fatt.resize(6,6);
	Fatt.setZero();

	//Patt_prev.resize(6,6);
	Patt_prev.setZero();

	//Patt_est.resize(6,6);
	Patt_est.setZero();

	//Patt.resize(6,6);
	Patt.setZero();

	//Qatt.resize(6);
	Qatt.setZero();

	//Ratt.resize(6);
	Ratt.setZero();

	//hatt.resize(6);
	hatt.setZero();

	//Hatt.resize(6,6);
	Hatt.setZero();

	//Satt.resize(6,6);
	Satt.setZero();

	//Katt.resize(6,6);
	Katt.setZero();

	//yatt.resize(6);
	yatt.setZero();

	//kattyatt.resize(6);
	Kattyatt.setZero();



		//Xtran_prev.resize(9);
		Xtran_prev.setZero();

		//Xtran_est.resize(9);
		Xtran_est.setZero();

		//Xtran.resize(9);
		Xtran.setZero();

		//Ftran.resize(9,9);
		Ftran.setZero();

		//Ptran_prev.resize(9,9);
		Ptran_prev.setZero();

		//Ptran_est.resize(9,9);
		Ptran_est.setZero();

		//Ptran.resize(9,9);
		Ptran.setZero();

		//Qtran.resize(9);
		Qtran.setZero();

		//Rtran.resize(3);
		Rtran.setZero();

		//htran.resize(3);
		htran.setZero();

		//Htran.resize(3,9);
		Htran.setZero();

		//Stran.resize(3,3);
		Stran.setZero();

		//Ktran.resize(9,3);
		Ktran.setZero();

		//Ktranytran.resize(9);
		Ktranytran.setZero();

		//ytran.resize(3);
		ytran.setZero();

		//DCM.resize(3,3);
		DCM.setZero();

		//a_imu.resize(3);
		a_imu.setZero();

		//a_measured.resize(3);
		a_measured.setZero();


	//set the covariance values to default ones. These matrices can be changed outside as this whole thing is a freaking struct
	Patt(0,0)=1;Patt(1,1)=1;Patt(2,2)=1;
	Patt(3,3)=1;Patt(4,4)=1;Patt(5,5)=1;

	Patt_prev=Patt;
	Patt_est=Patt;

	Qatt(0)=pow(0.09,2);Qatt(1)=pow(0.09,2);Qatt(2)=pow(0.09,2);
	Qatt(3)=pow(0.01,2);Qatt(4)=pow(0.01,2);Qatt(5)=pow(0.01,2);

	Ratt(0)=pow(0.5,2);Ratt(1)=pow(0.5,2);Ratt(2)=pow(0.5,2);
	Ratt(3)=pow(0.0087,2);Ratt(4)=pow(0.0087,2);Ratt(5)=pow(0.0087,2);

		Ptran(0,0)=1;Ptran(1,1)=1;Ptran(2,2)=1;
		Ptran(3,3)=1;Ptran(4,4)=1;Ptran(5,5)=1;
		Ptran(6,6)=1;Ptran(7,7)=1;Ptran(8,8)=1;

		Ptran_prev=Ptran;
		Ptran_est=Ptran;

		Qtran(0)=pow(0.09,2);Qtran(1)=pow(0.09,2);Qtran(2)=pow(0.09,2);
		Qtran(3)=pow(0.01,2);Qtran(4)=pow(0.01,2);Qtran(5)=pow(0.01,2);
		Qtran(6)=pow(0.03,2);Qtran(7)=pow(0.03,2);Qtran(8)=pow(0.03,2);

		Rtran(0)=pow(0.05,2);Rtran(1)=pow(0.05,2);Rtran(2)=pow(0.05,2);


		DCM(0,0)=1;DCM(0,1)=0;DCM(0,2)=0;
		DCM(1,0)=0;DCM(1,1)=1;DCM(1,2)=0;
		DCM(2,0)=0;DCM(2,1)=0;DCM(2,2)=1;


}
/*
 * Dimu=[tcurr,ax[3],wx[3]]
 * Dvicon=[x[3],q[4]]
 */


void EKF::PropagateUpdateMeanCov(QuadStatemanager * DM){
double dt;

	const float g=9.81;

	Patt_prev=Patt;

	Zkatt<< DM->a_meas[0],
			DM->a_meas[1],
			DM->a_meas[2],
			DM->w_meas[0],
			DM->w_meas[1],
			DM->w_meas[2];

	cout<<"t1 = "<<DM->t1<<" t2 = "<<DM->t2<<endl;

	dt=DM->t2-DM->t1;


	//Assign value of general data to eigen matrices for computation
	Xatt_prev(0)=DM->Xatt[0];
	Xatt_prev(1)=DM->Xatt[1];
	Xatt_prev(2)=DM->Xatt[2];
	Xatt_prev(3)=DM->Xatt[3];
	Xatt_prev(4)=DM->Xatt[4];
	Xatt_prev(5)=DM->Xatt[5];


	//att cov ptop
	    // Jacobian of process

		Fatt<<  1	 ,   0	  ,   0   ,   dt  ,   0   ,   0	  ,
				0	 ,   1    ,   0   ,    0  ,   dt  ,   0	  ,
				0	 , 	 0    ,	  1	  ,    0  ,   0   ,	  dt  ,
				0	 ,   0	  ,   0   ,    0  ,   0   ,   0   ,
				0    ,   0    ,   0   ,    0  ,   0   ,   0   ,
				0    ,   0    ,   0   ,    0  ,   0   ,   0   ;

		Xatt_est=Fatt*Xatt_prev;

	    Patt_est=(Fatt*Patt_prev)*Fatt.transpose();

	    Patt_est(0,0)=Patt_est(0,0)+Qatt(0);Patt_est(1,1)=Patt_est(1,1)+Qatt(1);Patt_est(2,2)=Patt_est(2,2)+Qatt(2);
	    Patt_est(3,3)=Patt_est(3,3)+Qatt(3);Patt_est(4,4)=Patt_est(4,4)+Qatt(4);Patt_est(5,5)=Patt_est(5,5)+Qatt(5);


	    //NOW THE UPDATE EQUATIONS ----------- ATTITUDE
	    hatt<<  g*sin(Xatt_est(1)),
	    		-g*sin(Xatt_est(0)),
	    		-g*cos(Xatt_est(0))*cos(Xatt_est(1)),
	    		Xatt_est(3),
	    		Xatt_est(4),
	    		Xatt_est(5);

	    yatt=Zkatt-hatt;

	    Hatt<< 0 , g*cos(Xatt_est(1)) , 0 , 0 , 0 , 0 ,
	    	   -g*cos(Xatt_est(0)) , 0 , 0 , 0 , 0 , 0 ,
	           g*cos(Xatt_est(1))*sin(Xatt_est(0)) , g*sin(Xatt_est(1))*cos(Xatt_est(0)) , 0 , 0 , 0 , 0 ,
	           0 , 0 , 0 , 1 , 0 , 0 ,
	           0 , 0 , 0 , 0 , 1 , 0 ,
	           0 , 0 , 0 , 0 , 0 , 1 ;

	    Satt=(Hatt*Patt_est)*Hatt.transpose();

	    Satt(0,0)=Satt(0,0)+Ratt(0);Satt(1,1)=Satt(1,1)+Ratt(1);Satt(2,2)=Satt(2,2)+Ratt(2);
	    Satt(3,3)=Satt(3,3)+Ratt(3);Satt(4,4)=Satt(4,4)+Ratt(4);Satt(5,5)=Satt(5,5)+Ratt(5);

	    Katt=(Patt_est*Hatt.transpose())*Satt.inverse();

	    Kattyatt=Katt*yatt;



	    //copy the data used for computation in the general data
	   /* DM->Xatt[0]=(Xatt_est(0)+Kattyatt(0))*180/3.14159265;
	    DM->Xatt[1]=(Xatt_est(1)+Kattyatt(1))*180/3.14159265;
	    DM->Xatt[2]=(Xatt_est(2)+Kattyatt(2))*180/3.14159265;
	    DM->Xatt[3]=(Xatt_est(3)+Kattyatt(3))*180/3.14159265;
	    DM->Xatt[4]=(Xatt_est(4)+Kattyatt(4))*180/3.14159265;
	    DM->Xatt[5]=(Xatt_est(5)+Kattyatt(5))*180/3.14159265;*/

	    DM->Xatt[0]=(Xatt_est(0)+Kattyatt(0));
	    DM->Xatt[1]=(Xatt_est(1)+Kattyatt(1));
	    DM->Xatt[2]=Xatt_est(2)+Kattyatt(2);
	    DM->Xatt[3]=Xatt_est(3)+Kattyatt(3);
	    DM->Xatt[4]=Xatt_est(4)+Kattyatt(4);
	    DM->Xatt[5]=Xatt_est(5)+Kattyatt(5);

	    Patt=Patt_est-Katt*Hatt*Patt_est;



    //translation-----------------------------------------------------------------------

	double dt2;
	dt2=pow(dt,2);
	double Eul[3];
	double q[4];
	double DCM_arr[3][3];
	Eul[0]=DM->Xatt[0];
	Eul[1]=DM->Xatt[1];
	Eul[2]=DM->Xatt[2];
	Euler2Quat(q,Eul);
	Quat2Rot(DCM_arr,q);

	Xtran_prev=Xtran;
	Ptran_prev=Ptran;

	DCM<<	DCM_arr[0][0] , DCM_arr[0][1] , DCM_arr[0][2] ,
			DCM_arr[1][0] , DCM_arr[1][1] , DCM_arr[1][2] ,
			DCM_arr[2][0] , DCM_arr[2][1] , DCM_arr[2][2] ;

	a_measured<<	DM->a_meas[0],
					DM->a_meas[1],
					DM->a_meas[2];

	a_imu=DCM*a_measured;

	a_imu(0)=a_imu(0);
	a_imu(1)=a_imu(1);
	a_imu(2)=a_imu(2)+g;

	cout<<"acceleration = "<<a_imu(2)<<endl;

	Zktran=a_imu;

	//Assign value of general data to eigen matrices for computation
	Xtran_prev(0)=DM->Xtran[0];
	Xtran_prev(1)=DM->Xtran[1];
	Xtran_prev(2)=DM->Xtran[2];
	Xtran_prev(3)=DM->Xtran[3];
	Xtran_prev(4)=DM->Xtran[4];
	Xtran_prev(5)=DM->Xtran[5];
	Xtran_prev(6)=DM->Xtran[6];
	Xtran_prev(7)=DM->Xtran[7];
	Xtran_prev(8)=DM->Xtran[8];

	//trans cov prop
    Ftran<< 1,0,0,dt,0,0,dt2,0,0,
    		0,1,0,0,dt,0,0,dt2,0,
    		0,0,1,0,0,dt,0,0,dt2,
    		0,0,0,1,0,0,dt,0,0,
    		0,0,0,0,1,0,0,dt,0,
    		0,0,0,0,0,1,0,0,dt,
    		0,0,0,0,0,0,0,0,0,
    		0,0,0,0,0,0,0,0,0,
    		0,0,0,0,0,0,0,0,0;

    Xtran_est=Ftran*Xtran_prev;

    Ptran_est=(Ftran*Ptran_prev)*Ftran.transpose();

    Ptran_est(0,0)=Ptran_est(0,0)+Qtran(0);Ptran_est(1,1)=Ptran_est(1,1)+Qtran(1);Ptran_est(2,2)=Ptran_est(2,2)+Qtran(2);
    Ptran_est(3,3)=Ptran_est(3,3)+Qtran(3);Ptran_est(4,4)=Ptran_est(4,4)+Qtran(4);Ptran_est(5,5)=Ptran_est(5,5)+Qtran(5);
    Ptran_est(6,6)=Ptran_est(6,6)+Qtran(6);Ptran_est(7,7)=Ptran_est(7,7)+Qtran(7);Ptran_est(8,8)=Ptran_est(8,8)+Qtran(8);


    //NOW THE UPDATE EQUATIONS----------- Trans

    htran=a_measured;

    ytran=Zktran-htran;

    Htran<< 0,0,0,0,0,0,1,0,0,
    		0,0,0,0,0,0,0,1,0,
    		0,0,0,0,0,0,0,0,1;

    Stran=(Htran*Ptran_est)*Htran.transpose();

    Stran(0,0)=Stran(0,0)+Rtran(0);Stran(1,1)=Stran(1,1)+Rtran(1);Stran(2,2)=Stran(2,2)+Rtran(2);

    Ktran=(Ptran_est*Htran.transpose())*Stran.inverse();

    Ktranytran=Ktran*ytran;


    //copy the data used for computation in the general data
    DM->Xtran[0]=Xtran_est(0)+Ktranytran(0);
    DM->Xtran[1]=Xtran_est(1)+Ktranytran(1);
    DM->Xtran[2]=Xtran_est(2)+Ktranytran(2);
    DM->Xtran[3]=Xtran_est(3)+Ktranytran(3);
    DM->Xtran[4]=Xtran_est(4)+Ktranytran(4);
    DM->Xtran[5]=Xtran_est(5)+Ktranytran(5);
    DM->Xtran[6]=Xtran_est(6)+Ktranytran(6);
    DM->Xtran[7]=Xtran_est(7)+Ktranytran(7);
    DM->Xtran[8]=Xtran_est(8)+Ktranytran(8);

    Ptran=Ptran_est-Ktran*Htran*Ptran_est;


   // cout<<"t1 = "<<DM->t1<<" t2 = "<<DM->t2<<endl;
}
/*
void EKF::PrintFiltervals(){
	cout<<"dt = "<<dt<<endl;
	cout<<"------------------ Attitude Fields----------------------------"<<endl<<endl;

    cout<<"Fatt = "<<endl<<Fatt<<endl;
	cout<<"Patt = "<<endl<<Patt<<endl;
	cout<<"Qatt = "<<endl<<Qatt<<endl;
	cout<<"Ratt = "<<endl<<Ratt<<endl;
	cout<<"Hatt = "<<endl<<Hatt<<endl;
	cout<<"Satt = "<<endl<<Satt<<endl;
	cout<<"Katt = "<<endl<<Katt<<endl;
	cout<<"yatt = "<<endl<<yatt<<endl;


	cout<<"------------------ Translational Fields----------------------------"<<endl<<endl;
		cout<<"Ftran = "<<endl<<Ftran<<endl;
		cout<<"Ptran = "<<endl<<Ptran<<endl;
		cout<<"Qtran = "<<endl<<Qtran<<endl;
		cout<<"Rtran = "<<endl<<Rtran<<endl;
		cout<<"Htran = "<<endl<<Htran<<endl;
		cout<<"Stran = "<<endl<<Stran<<endl;
		cout<<"Ktran = "<<endl<<Ktran<<endl;
		cout<<"ytran = "<<endl<<ytran<<endl;

}
*/
