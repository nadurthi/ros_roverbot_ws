#include "kinematic_estimators.h"
#include<cmath>

#define P1 Patt.block<4,4>(0,0)
#define P2 Patt.block<4,3>(0,4)
#define P3 Patt.block<4,3>(0,7)

#define P4 Patt.block<3,4>(4,0)
#define P5 Patt.block<3,3>(4,4)
#define P6 Patt.block<3,3>(4,7)

#define P7 Patt.block<3,4>(7,0)
#define P8 Patt.block<3,3>(7,4)
#define P9 Patt.block<3,3>(7,7)

#define A11 Patt.block<4,4>(0,0)
#define A12 Patt.block<4,3>(0,4)
#define A21 Patt.block<3,4>(4,0)
#define A22 Patt.block<3,3>(4,4)

#define P1t Ptran.block<3,3>(0,0)
#define P2t Ptran.block<3,3>(0,3)
#define P3t Ptran.block<3,3>(0,6)

#define P4t Ptran.block<3,3>(3,0)
#define P5t Ptran.block<3,3>(3,3)
#define P6t Ptran.block<3,3>(3,6)

#define P7t Ptran.block<3,3>(6,0)
#define P8t Ptran.block<3,3>(6,3)
#define P9t Ptran.block<3,3>(6,6)

/*--------------------------------------------------------------------------------------------------------------------------------------------
 * Estimator (1): structure function definitions
 */
void EKF_Estimator1::InitializeFilter(){
	//Fatt.resize(10,10);
	Fatt.setZero();

	//Patt.resize(10,10);
	Patt.setZero();

	//Qatt.resize(10);
	Qatt.setZero();

	//Inoatt.resize(10);
	Inoatt.setZero();

	//Ratt.resize(7);
	Ratt.setZero();

	//Hatt.resize(7,10);
	Hatt.setZero();

	//Satt.resize(7,7);
	Satt.setZero();

	//Katt.resize(10,7);
	Katt.setZero();

	//yatt.resize(7);
	yatt.setZero();

	//yatt_est.resize(7);
	yatt_est.setZero();


		//Ftran.resize(9,9);
		Ftran.setZero();

		//Ptran.resize(9,9);
		Ptran.setZero();

		//Qtran.resize(9);
		Qtran.setZero();

		//Inotran.resize(9);
		Inotran.setZero();


		//Rtran.resize(6);
		Rtran.setZero();

		//Htran.resize(6,9);
		Htran.setZero();

		//Stran.resize(6,6);
		Stran.setZero();

		//Ktran.resize(9,6);
		Ktran.setZero();

		//ytran.resize(6);
		ytran.setZero();

		//ytran_est.resize(6);
		ytran_est.setZero();


	//set the covariance values to default ones. These matrices can be changed outside as this whole thing is a freaking struct
	Patt(0,0)=0.1;Patt(1,1)=0.1;Patt(2,2)=0.1;Patt(3,3)=0.1;
	Patt(4,4)=0.09;Patt(5,5)=0.09;Patt(6,6)=0.09;
	Patt(7,7)=0.0001;Patt(8,8)=0.0001;Patt(9,9)=0.0001;

	Qatt(0)=0.001;Qatt(1)=0.001;Qatt(2)=0.001;Qatt(3)=0.001;
	Qatt(4)=0.01;Qatt(5)=0.01;Qatt(6)=0.01;
	Qatt(7)=0.00001;Qatt(8)=0.00001;Qatt(9)=0.00001;

	Ratt(0)=0.05;Ratt(1)=0.05;Ratt(2)=0.05;Ratt(3)=0.05;
	Ratt(4)=0.01;Ratt(5)=0.01;Ratt(6)=0.01;

		Ptran(0,0)=0.01;Ptran(1,1)=0.01;Ptran(2,2)=0.01;
		Ptran(3,3)=0.09;Ptran(4,4)=0.09;Ptran(5,5)=0.09;
		Ptran(6,6)=0.001;Ptran(7,7)=0.0001;Ptran(8,8)=0.0001;

		Qtran(0)=0.0001;Qtran(1)=0.0001;Qtran(2)=0.0001;
		Qtran(3)=1;Qtran(4)=1;Qtran(5)=1;
		Qtran(6)=0.01;Qtran(7)=0.01;Qtran(8)=0.01;

		Rtran(0)=0.1;Rtran(1)=0.1;Rtran(2)=0.1;
		Rtran(3)=0.1;Rtran(4)=0.1;Rtran(5)=0.1;



}

void EKF_Estimator1::PropagateUpdateMeanCov(QuadStateVariable *QuadState){
	double qq[4],w_quat[4],t1,t2,dt2;

	double w1,w2,w3,q0,q1,q2,q3;

		w1=QuadState->w[0];
		w2=QuadState->w[1];
		w3=QuadState->w[2];

		q0=QuadState->q[0];
		q1=QuadState->q[1];
		q2=QuadState->q[2];
		q3=QuadState->q[3];

	QuadState->tonboard.GetLastRowQ(&t2);
	QuadState->tonboard.GetLastButOneRowQ(&t1);

	dt=t2-t1;
	dt2=pow(dt,2);

	Vec2Quat(w_quat,QuadState->w);

	quat_prod(qq, QuadState->q,w_quat);

	//att mean prop  wdot remains the same
	QuadState->q[0]=QuadState->q[0]+(dt/2)*qq[0];
	QuadState->q[1]=QuadState->q[1]+(dt/2)*qq[1];
	QuadState->q[2]=QuadState->q[2]+(dt/2)*qq[2];
	QuadState->q[3]=QuadState->q[3]+(dt/2)*qq[3];
	NormQuat(QuadState->q);

	QuadState->w[0]=QuadState->w[0]+(dt)*QuadState->wdot[0];
	QuadState->w[1]=QuadState->w[1]+(dt)*QuadState->wdot[1];
	QuadState->w[2]=QuadState->w[2]+(dt)*QuadState->wdot[2];


	//att cov ptop
	    // Jacobian of process

		Fatt<<     1   ,  -dt*w1/2,  -dt*w2/2,   -dt*w3/2,  -dt*q1/2,  -dt*q2/2	,-dt*q3/2, 0,0,0,
			 dt*w1/2,    1    ,    dt*w3/2,   -dt*w2/2,   dt*q0/2,	-dt*q3/2	 ,dt*q2/2, 0,0,0,
			 dt*w2/2, -dt*w3/2      ,1,        dt*w1/2,   dt*q3/2,	 dt*q0/2,	 -dt*q1/2, 0,0,0,
		     dt*w3/2,  dt*w2/2,    -dt*w1/2,     1    ,  -dt*q2/2,   dt*q1/2,	  dt*q0/2, 0,0,0,
		       0     ,   0    ,       0    ,     0    ,      1   ,      0   ,        0   , dt,0,0,
		       0     ,   0    ,       0    ,     0    ,      0   ,      1   ,        0   , 0,dt,0,
		       0     ,   0    ,       0    ,     0    ,      0   ,      0   ,        1   , 0,0,dt,
		       0     ,   0    ,       0    ,     0    ,      0   ,      0   ,        0   , 1,0,0,
		       0     ,   0    ,       0    ,     0    ,      0   ,      0   ,        0   , 0,1,0,
		       0     ,   0    ,       0    ,     0    ,      0   ,      0   ,        0   , 0,0,1;



	    Patt=(Fatt*Patt)*Fatt.transpose();


	    Patt(0,0)=Patt(0,0)+Qatt(0);Patt(1,1)=Patt(1,1)+Qatt(1);Patt(2,2)=Patt(2,2)+Qatt(2);Patt(3,3)=Patt(3,3)+Qatt(3);
	    Patt(4,4)=Patt(4,4)+Qatt(4);Patt(5,5)=Patt(5,5)+Qatt(5);Patt(6,6)=Patt(6,6)+Qatt(6);
	    Patt(7,7)=Patt(7,7)+Qatt(7);Patt(8,8)=Patt(8,8)+Qatt(8);Patt(9,9)=Patt(9,9)+Qatt(9);

	    if(QuadState->VICONupdateFLAG==1)
	   	    {


	    //NOW THE UPDATE EQUATIONS ----------- ATTITUDE
	        qq[0]=QuadState->qmis[0];
	        qq[1]=QuadState->qmis[1];
	        qq[2]=QuadState->qmis[2];
	        qq[3]=QuadState->qmis[3];

	        Hatt<< 1,0,0,0,0,0,0,0,0,0,
	        		0,1,0,0,0,0,0,0,0,0,
	        		0,0,1,0,0,0,0,0,0,0,
	        		0,0,0,1,0,0,0,0,0,0,
	        		0,0,0,0, 1-2*(pow(qq[2],2)+pow(qq[3],2)),2*qq[1]*qq[2]+2*qq[0]*qq[3],-2*qq[0]*qq[2]+2*qq[1]*qq[3] ,0,0,0,
	        		0,0,0,0, 2*qq[1]*qq[2]-2*qq[0]*qq[3],1-2*(pow(qq[1],2)+pow(qq[3],2)),2*qq[0]*qq[1]+2*qq[2]*qq[3],0,0,0,
	        		0,0,0,0, 2*qq[0]*qq[2]+2*qq[1]*qq[3],-2*qq[0]*qq[1]+2*qq[2]*qq[3],1-2*(pow(qq[1],2)+pow(qq[2],2)),0,0,0;



	        Satt=(Hatt*Patt)*Hatt.transpose();

	        Satt(0,0)=Satt(0,0)+Ratt(0);Satt(1,1)=Satt(1,1)+Ratt(1);Satt(2,2)=Satt(2,2)+Ratt(2);Satt(3,3)=Satt(3,3)+Ratt(3);
	        Satt(4,4)=Satt(4,4)+Ratt(4);Satt(5,5)=Satt(5,5)+Ratt(5);Satt(6,6)=Satt(6,6)+Ratt(6);


	        Katt=  ( Patt*Hatt.transpose())*Satt.inverse();

	        yatt<< QuadState->q_meas[0],
	        	   QuadState->q_meas[1],
	        	   QuadState->q_meas[2],
	        	   QuadState->q_meas[3],
	        	   QuadState->w_meas[0],
	        	   QuadState->w_meas[1],
	        	   QuadState->w_meas[2];


	        double wimu[3];
	        quat_inv(qq,QuadState->qmis);
	        quat_rot(wimu,qq,QuadState->w);

	        wimu[0]=wimu[0]+QuadState->bg[0];
	        wimu[1]=wimu[1]+QuadState->bg[1];
	        wimu[2]=wimu[2]+QuadState->bg[2];



	        yatt_est<< QuadState->q[0],
	            	   QuadState->q[1],
	            	   QuadState->q[2],
	            	   QuadState->q[3],
	            	   wimu[0],
	            	   wimu[1],
	            	   wimu[2];

	        Inoatt=Katt*(yatt-yatt_est);

	        QuadState->q[0]=QuadState->q[0]+Inoatt(0);
	        QuadState->q[1]=QuadState->q[1]+Inoatt(1);
	        QuadState->q[2]=QuadState->q[2]+Inoatt(2);
	        QuadState->q[3]=QuadState->q[3]+Inoatt(3);
	        NormQuat(QuadState->q);

	        QuadState->w[0]=QuadState->w[0]+Inoatt(4);
	        QuadState->w[1]=QuadState->w[1]+Inoatt(5);
	        QuadState->w[2]=QuadState->w[2]+Inoatt(6);

	        QuadState->wdot[0]=QuadState->wdot[0]+Inoatt(7);
	        QuadState->wdot[1]=QuadState->wdot[1]+Inoatt(8);
	        QuadState->wdot[2]=QuadState->wdot[2]+Inoatt(9);


	        Patt=Patt-Katt*Hatt*Patt;
	   	    }
	    else
	    	{

	    	 	 	 	qq[0]=QuadState->qmis[0];
	    		        qq[1]=QuadState->qmis[1];
	    		        qq[2]=QuadState->qmis[2];
	    		        qq[3]=QuadState->qmis[3];

	    		        Hatt2<< 0,0,0,0, 1-2*(pow(qq[2],2)+pow(qq[3],2)),2*qq[1]*qq[2]+2*qq[0]*qq[3],-2*qq[0]*qq[2]+2*qq[1]*qq[3] ,0,0,0,
	    		        		0,0,0,0, 2*qq[1]*qq[2]-2*qq[0]*qq[3],1-2*(pow(qq[1],2)+pow(qq[3],2)),2*qq[0]*qq[1]+2*qq[2]*qq[3],0,0,0,
	    		        		0,0,0,0, 2*qq[0]*qq[2]+2*qq[1]*qq[3],-2*qq[0]*qq[1]+2*qq[2]*qq[3],1-2*(pow(qq[1],2)+pow(qq[2],2)),0,0,0;

	    		        Satt2=(Hatt2*Patt)*Hatt2.transpose();

	    		        Satt2(0,0)=Satt2(0,0)+Ratt(4);Satt2(1,1)=Satt2(1,1)+Ratt(5);Satt2(2,2)=Satt2(2,2)+Ratt(6);

	    		        Katt2=  ( Patt*Hatt2.transpose())*Satt2.inverse();

	    		        yatt2<< QuadState->w_meas[0],
	    		        	    QuadState->w_meas[1],
	    		        	    QuadState->w_meas[2];


	    		        	        double wimu[3];
	    		        	        quat_inv(qq,QuadState->qmis);
	    		        	        quat_rot(wimu,qq,QuadState->w);

	    		        	        wimu[0]=wimu[0]+QuadState->bg[0];
	    		        	        wimu[1]=wimu[1]+QuadState->bg[1];
	    		        	        wimu[2]=wimu[2]+QuadState->bg[2];



	    		        	        yatt_est2<< wimu[0],
	    		        	            	    wimu[1],
	    		        	            	    wimu[2];

	    		        	        Inoatt=Katt2*(yatt2-yatt_est2);

	    		        	        QuadState->q[0]=QuadState->q[0]+Inoatt(0);
	    		        	        QuadState->q[1]=QuadState->q[1]+Inoatt(1);
	    		        	        QuadState->q[2]=QuadState->q[2]+Inoatt(2);
	    		        	        QuadState->q[3]=QuadState->q[3]+Inoatt(3);
	    		        	        NormQuat(QuadState->q);

	    		        	        QuadState->w[0]=QuadState->w[0]+Inoatt(4);
	    		        	        QuadState->w[1]=QuadState->w[1]+Inoatt(5);
	    		        	        QuadState->w[2]=QuadState->w[2]+Inoatt(6);

	    		        	        QuadState->wdot[0]=QuadState->wdot[0]+Inoatt(7);
	    		        	        QuadState->wdot[1]=QuadState->wdot[1]+Inoatt(8);
	    		        	        QuadState->wdot[2]=QuadState->wdot[2]+Inoatt(9);


	    		 Patt=Patt-Katt2*Hatt2*Patt;

	    	}
//############################################################################################################################################
    //translation-----------------------------------------------------------------------
	QuadState->x[0]=QuadState->x[0]+(dt)*QuadState->v[0];
	QuadState->x[1]=QuadState->x[1]+(dt)*QuadState->v[1];
	QuadState->x[2]=QuadState->x[2]+(dt)*QuadState->v[2];

	QuadState->v[0]=QuadState->v[0]+(dt)*QuadState->a[0];
	QuadState->v[1]=QuadState->v[1]+(dt)*QuadState->a[1];
	QuadState->v[2]=QuadState->v[2]+(dt)*QuadState->a[2];




    //trans cov prop
    Ftran<< 1,0,0,dt,0,0,0,0,0,
    		0,1,0,0,dt,0,0,0,0,
    		0,0,1,0,0,dt,0,0,0,
    		0,0,0,1,0,0,dt,0,0,
    		0,0,0,0,1,0,0,dt,0,
    		0,0,0,0,0,1,0,0,dt,
    		0,0,0,0,0,0,1,0,0,
    		0,0,0,0,0,0,0,1,0,
    		0,0,0,0,0,0,0,0,1;


    Ptran=(Ftran*Ptran)*Ftran.transpose();


    Ptran(0,0)=Ptran(0,0)+Qtran(0);Ptran(1,1)=Ptran(1,1)+Qtran(1);Ptran(2,2)=Ptran(2,2)+Qtran(2);Ptran(3,3)=Ptran(3,3)+Qtran(3);
    Ptran(4,4)=Ptran(4,4)+Qtran(4);Ptran(5,5)=Ptran(5,5)+Qtran(5);Ptran(6,6)=Ptran(6,6)+Qtran(6);
    Ptran(7,7)=Ptran(7,7)+Qtran(7);Ptran(8,8)=Ptran(8,8)+Qtran(8);


    if(QuadState->VICONupdateFLAG==1 )
    {


    //NOW THE UPDATE EQUATIONS----------- Trans
    qq[0]=QuadState->qmis[0];
    qq[1]=QuadState->qmis[1];
    qq[2]=QuadState->qmis[2];
    qq[3]=QuadState->qmis[3];
    quat_prod(qq,QuadState->q,QuadState->qmis);

    Htran<< 1,0,0,0,0,0,0,0,0,
    		0,1,0,0,0,0,0,0,0,
    		0,0,1,0,0,0,0,0,0,
    		0,0,0,0,0,0, 1-2*(pow(qq[2],2)+pow(qq[3],2)),2*qq[1]*qq[2]+2*qq[0]*qq[3],-2*qq[0]*qq[2]+2*qq[1]*qq[3],
    		0,0,0,0,0,0, 2*qq[1]*qq[2]-2*qq[0]*qq[3],1-2*(pow(qq[1],2)+pow(qq[3],2)),2*qq[0]*qq[1]+2*qq[2]*qq[3],
    		0,0,0,0,0,0, 2*qq[0]*qq[2]+2*qq[1]*qq[3],-2*qq[0]*qq[1]+2*qq[2]*qq[3],1-2*(pow(qq[1],2)+pow(qq[2],2));


    Stran=(Htran*Ptran)*Htran.transpose();

    Stran(0,0)=Stran(0,0)+Rtran(0);Stran(1,1)=Stran(1,1)+Rtran(1);Stran(2,2)=Stran(2,2)+Rtran(2);Stran(3,3)=Stran(3,3)+Rtran(3);
    Stran(4,4)=Stran(4,4)+Rtran(4);Stran(5,5)=Stran(5,5)+Rtran(5);


    Ktran=  ( Ptran*Htran.transpose())*Stran.inverse();

    ytran<<QuadState->x_meas[0],
    	   QuadState->x_meas[1],
    	   QuadState->x_meas[2],
    	   QuadState->a_meas[0],
    	   QuadState->a_meas[1],
    	   QuadState->a_meas[2];

    double aimu[3];
    AccTransform2imu(aimu,QuadState->a,QuadState->rmis,QuadState->q,QuadState->qmis,QuadState->w,QuadState->wdot,QuadState->ba);

    ytran_est<<QuadState->x[0],
        	   QuadState->x[1],
        	   QuadState->x[2],
        	   aimu[0],
        	   aimu[1],
        	   aimu[2];

    Inotran=Ktran*(ytran-ytran_est);
    QuadState->x[0]=QuadState->x[0]+Inotran(0);
    QuadState->x[1]=QuadState->x[1]+Inotran(1);
    QuadState->x[2]=QuadState->x[2]+Inotran(2);

    QuadState->v[0]=QuadState->v[0]+Inotran(3);
    QuadState->v[1]=QuadState->v[1]+Inotran(4);
    QuadState->v[2]=QuadState->v[2]+Inotran(5);

    QuadState->a[0]=QuadState->a[0]+Inotran(6);
    QuadState->a[1]=QuadState->a[1]+Inotran(7);
    QuadState->a[2]=QuadState->a[2]+Inotran(8);


    Ptran=Ptran-Ktran*Htran*Ptran;
    }
    else {

    	    qq[0]=QuadState->qmis[0];
    	    qq[1]=QuadState->qmis[1];
    	    qq[2]=QuadState->qmis[2];
    	    qq[3]=QuadState->qmis[3];
    	    quat_prod(qq,QuadState->q,QuadState->qmis);

    	Htran2<<    0,0,0,0,0,0, 1-2*(pow(qq[2],2)+pow(qq[3],2)),2*qq[1]*qq[2]+2*qq[0]*qq[3],-2*qq[0]*qq[2]+2*qq[1]*qq[3],
    	    		0,0,0,0,0,0, 2*qq[1]*qq[2]-2*qq[0]*qq[3],1-2*(pow(qq[1],2)+pow(qq[3],2)),2*qq[0]*qq[1]+2*qq[2]*qq[3],
    	    		0,0,0,0,0,0, 2*qq[0]*qq[2]+2*qq[1]*qq[3],-2*qq[0]*qq[1]+2*qq[2]*qq[3],1-2*(pow(qq[1],2)+pow(qq[2],2));

    	    Stran2=(Htran2*Ptran)*Htran2.transpose();

    	    Stran2(0,0)=Stran2(0,0)+Rtran(3);Stran2(1,1)=Stran2(1,1)+Rtran(4);Stran2(2,2)=Stran2(2,2)+Rtran(5);

        Ktran2=  ( Ptran*Htran2.transpose())*Stran2.inverse();

        ytran2<<QuadState->a_meas[0],
        	    QuadState->a_meas[1],
        	    QuadState->a_meas[2];

        double aimu[3];
        AccTransform2imu(aimu,QuadState->a,QuadState->rmis,QuadState->q,QuadState->qmis,QuadState->w,QuadState->wdot,QuadState->ba);

        ytran_est2<<aimu[0],
            	   aimu[1],
            	   aimu[2];

        Inotran=Ktran2*(ytran2-ytran_est2);
        QuadState->x[0]=QuadState->x[0]+Inotran(0);
        QuadState->x[1]=QuadState->x[1]+Inotran(1);
        QuadState->x[2]=QuadState->x[2]+Inotran(2);

        QuadState->v[0]=QuadState->v[0]+Inotran(3);
        QuadState->v[1]=QuadState->v[1]+Inotran(4);
        QuadState->v[2]=QuadState->v[2]+Inotran(5);

        QuadState->a[0]=QuadState->a[0]+Inotran(6);
        QuadState->a[1]=QuadState->a[1]+Inotran(7);
        QuadState->a[2]=QuadState->a[2]+Inotran(8);

    	  Ptran=Ptran-Ktran2*Htran2*Ptran;
    	    	}

    QuadState->VICONupdateFLAG=0;
    QuadState->IMUupdateFLAG=0;

}

//#######################################################################################

/*
 * Initialize the faster implementation of EKF
 */
void EKF_Estimator_onboard1::InitializeFilter(){
	//Fatt.resize(10,10);
		Fatt.setZero();

		//Patt.resize(10,10);
		Patt.setZero();

		//Qatt.resize(10);
		Qatt.setZero();

		//Inoatt.resize(10);
		Inoatt.setZero();

		//Ratt.resize(7);
		Ratt.setZero();

		//Hatt.resize(7,10);
		Hatt.setZero();

		//Satt.resize(7,7);
		Satt.setZero();

		//Katt.resize(10,7);
		Katt.setZero();

		//yatt.resize(7);
		yatt.setZero();

		//yatt_est.resize(7);
		yatt_est.setZero();


			//Ftran.resize(9,9);
			Ftran.setZero();

			//Ptran.resize(9,9);
			Ptran.setZero();

			//Qtran.resize(9);
			Qtran.setZero();

			//Inotran.resize(9);
			Inotran.setZero();


			//Rtran.resize(6);
			Rtran.setZero();

			//Htran.resize(6,9);
			Htran.setZero();

			//Stran.resize(6,6);
			Stran.setZero();

			//Ktran.resize(9,6);
			Ktran.setZero();

			//ytran.resize(6);
			ytran.setZero();

			//ytran_est.resize(6);
			ytran_est.setZero();


		//set the covariance values to default ones. These matrices can be changed outside as this whole thing is a freaking struct
		Patt(0,0)=0.01;Patt(1,1)=0.01;Patt(2,2)=0.01;Patt(3,3)=0.01;
		Patt(4,4)=0.09;Patt(5,5)=0.09;Patt(6,6)=0.09;


		Qatt(0)=0.05;Qatt(1)=0.05;Qatt(2)=0.05;Qatt(3)=0.05;
		Qatt(4)=0.25;Qatt(5)=0.25;Qatt(6)=0.25;
		Qatt(7)=4;Qatt(8)=4;Qatt(9)=4;

		Ratt(0)=0.01;Ratt(1)=0.01;Ratt(2)=0.01;Ratt(3)=0.01;
		Ratt(4)=0.09;Ratt(5)=0.09;Ratt(6)=0.09;

			Ptran(0,0)=0.01;Ptran(1,1)=0.01;Ptran(2,2)=0.01;
			Ptran(3,3)=0.1;Ptran(4,4)=0.1;Ptran(5,5)=0.1;
			Ptran(6,6)=0.5;Ptran(7,7)=0.5;Ptran(8,8)=0.5;

			Qtran(0)=0.0025;Qtran(1)=0.0025;Qtran(2)=0.0025;
			Qtran(3)=0.1;Qtran(4)=0.1;Qtran(5)=0.1;
			Qtran(6)=0.5;Qtran(7)=0.5;Qtran(8)=0.5;

			Rtran(0)=0.0001;Rtran(1)=0.0001;Rtran(2)=0.0001;
			Rtran(3)=4;Rtran(4)=4;Rtran(5)=4;




}

/*
 * EKF onboard implementation..MAKE IT FASTER
 */

void EKF_Estimator_onboard1::PropagateUpdateMeanCov(QuadStateVariable *QuadState){
	double qq[4],w_quat[4],t1,t2,dt2;

	double w1,w2,w3,q0,q1,q2,q3;

		w1=QuadState->w[0];
		w2=QuadState->w[1];
		w3=QuadState->w[2];

		q0=QuadState->q[0];
		q1=QuadState->q[1];
		q2=QuadState->q[2];
		q3=QuadState->q[3];

	QuadState->tonboard.GetLastRowQ(&t2);
	QuadState->tonboard.GetLastButOneRowQ(&t1);

	dt=t2-t1;
	dt2=pow(dt,2);

	Vec2Quat(w_quat,QuadState->w);

	quat_prod(qq, QuadState->q,w_quat);

	//att mean prop  wdot remains the same
	QuadState->q[0]=QuadState->q[0]+(dt/2)*qq[0];
	QuadState->q[1]=QuadState->q[1]+(dt/2)*qq[1];
	QuadState->q[2]=QuadState->q[2]+(dt/2)*qq[2];
	QuadState->q[3]=QuadState->q[3]+(dt/2)*qq[3];
	NormQuat(QuadState->q);

	QuadState->w[0]=QuadState->w[0]+(dt)*QuadState->wdot[0];
	QuadState->w[1]=QuadState->w[1]+(dt)*QuadState->wdot[1];
	QuadState->w[2]=QuadState->w[2]+(dt)*QuadState->wdot[2];


	//att cov ptop
	    // Jacobian of process

		Fatt<<     1   ,  -dt*w1/2,  -dt*w2/2,   -dt*w3/2,  -dt*q1/2,  -dt*q2/2	,-dt*q3/2, 0,0,0,
			 dt*w1/2,    1    ,    dt*w3/2,   -dt*w2/2,   dt*q0/2,	-dt*q3/2	 ,dt*q2/2, 0,0,0,
			 dt*w2/2, -dt*w3/2      ,1,        dt*w1/2,   dt*q3/2,	 dt*q0/2,	 -dt*q1/2, 0,0,0,
		     dt*w3/2,  dt*w2/2,    -dt*w1/2,     1    ,  -dt*q2/2,   dt*q1/2,	  dt*q0/2, 0,0,0,
		       0     ,   0    ,       0    ,     0    ,      1   ,      0   ,        0   , dt,0,0,
		       0     ,   0    ,       0    ,     0    ,      0   ,      1   ,        0   , 0,dt,0,
		       0     ,   0    ,       0    ,     0    ,      0   ,      0   ,        1   , 0,0,dt,
		       0     ,   0    ,       0    ,     0    ,      0   ,      0   ,        0   , 1,0,0,
		       0     ,   0    ,       0    ,     0    ,      0   ,      0   ,        0   , 0,1,0,
		       0     ,   0    ,       0    ,     0    ,      0   ,      0   ,        0   , 0,0,1;



	    Patt=(Fatt*Patt)*Fatt.transpose();


	    Patt(0,0)=Patt(0,0)+Qatt(0);Patt(1,1)=Patt(1,1)+Qatt(1);Patt(2,2)=Patt(2,2)+Qatt(2);Patt(3,3)=Patt(3,3)+Qatt(3);
	    Patt(4,4)=Patt(4,4)+Qatt(4);Patt(5,5)=Patt(5,5)+Qatt(5);Patt(6,6)=Patt(6,6)+Qatt(6);
	    Patt(7,7)=Patt(7,7)+Qatt(7);Patt(8,8)=Patt(8,8)+Qatt(8);Patt(9,9)=Patt(9,9)+Qatt(9);

	    cout<<"VICONupdateFLAG = "<<QuadState->VICONupdateFLAG<<endl;
	    if(QuadState->VICONupdateFLAG==1)
	   	    {


	    //NOW THE UPDATE EQUATIONS ----------- ATTITUDE
	        qq[0]=QuadState->qmis[0];
	        qq[1]=QuadState->qmis[1];
	        qq[2]=QuadState->qmis[2];
	        qq[3]=QuadState->qmis[3];

	        Hatt<< 1,0,0,0,0,0,0,0,0,0,
	        		0,1,0,0,0,0,0,0,0,0,
	        		0,0,1,0,0,0,0,0,0,0,
	        		0,0,0,1,0,0,0,0,0,0,
	        		0,0,0,0, 1-2*(pow(qq[2],2)+pow(qq[3],2)),2*qq[1]*qq[2]+2*qq[0]*qq[3],-2*qq[0]*qq[2]+2*qq[1]*qq[3] ,0,0,0,
	        		0,0,0,0, 2*qq[1]*qq[2]-2*qq[0]*qq[3],1-2*(pow(qq[1],2)+pow(qq[3],2)),2*qq[0]*qq[1]+2*qq[2]*qq[3],0,0,0,
	        		0,0,0,0, 2*qq[0]*qq[2]+2*qq[1]*qq[3],-2*qq[0]*qq[1]+2*qq[2]*qq[3],1-2*(pow(qq[1],2)+pow(qq[2],2)),0,0,0;



	        Satt=(Hatt*Patt)*Hatt.transpose();

	        Satt(0,0)=Satt(0,0)+Ratt(0);Satt(1,1)=Satt(1,1)+Ratt(1);Satt(2,2)=Satt(2,2)+Ratt(2);Satt(3,3)=Satt(3,3)+Ratt(3);
	        Satt(4,4)=Satt(4,4)+Ratt(4);Satt(5,5)=Satt(5,5)+Ratt(5);Satt(6,6)=Satt(6,6)+Ratt(6);


	        Katt=  ( Patt*Hatt.transpose())*Satt.inverse();

            //cout<<"q_recieve = "<<QuadState->q_meas[0]<<endl;

	        yatt<< QuadState->q_meas[0],
	        	   QuadState->q_meas[1],
	        	   QuadState->q_meas[2],
	        	   QuadState->q_meas[3],
	        	   QuadState->w_meas[0],
	        	   QuadState->w_meas[1],
	        	   QuadState->w_meas[2];


	        double wimu[3];
	        quat_inv(qq,QuadState->qmis);
	        quat_rot(wimu,qq,QuadState->w);

	        wimu[0]=wimu[0]+QuadState->bg[0];
	        wimu[1]=wimu[1]+QuadState->bg[1];
	        wimu[2]=wimu[2]+QuadState->bg[2];



	        yatt_est<< QuadState->q[0],
	            	   QuadState->q[1],
	            	   QuadState->q[2],
	            	   QuadState->q[3],
	            	   wimu[0],
	            	   wimu[1],
	            	   wimu[2];

	        Inoatt=Katt*(yatt-yatt_est);

	        QuadState->q[0]=QuadState->q[0]+Inoatt(0);
	        QuadState->q[1]=QuadState->q[1]+Inoatt(1);
	        QuadState->q[2]=QuadState->q[2]+Inoatt(2);
	        QuadState->q[3]=QuadState->q[3]+Inoatt(3);
	        NormQuat(QuadState->q);

	        QuadState->w[0]=QuadState->w[0]+Inoatt(4);
	        QuadState->w[1]=QuadState->w[1]+Inoatt(5);
	        QuadState->w[2]=QuadState->w[2]+Inoatt(6);

	        QuadState->wdot[0]=QuadState->wdot[0]+Inoatt(7);
	        QuadState->wdot[1]=QuadState->wdot[1]+Inoatt(8);
	        QuadState->wdot[2]=QuadState->wdot[2]+Inoatt(9);


	        Patt=Patt-Katt*Hatt*Patt;
	   	    }
	    else
	    	{

	    	 	 	 	qq[0]=QuadState->qmis[0];
	    		        qq[1]=QuadState->qmis[1];
	    		        qq[2]=QuadState->qmis[2];
	    		        qq[3]=QuadState->qmis[3];

	    		        Hatt2<< 0,0,0,0, 1-2*(pow(qq[2],2)+pow(qq[3],2)),2*qq[1]*qq[2]+2*qq[0]*qq[3],-2*qq[0]*qq[2]+2*qq[1]*qq[3] ,0,0,0,
	    		        		0,0,0,0, 2*qq[1]*qq[2]-2*qq[0]*qq[3],1-2*(pow(qq[1],2)+pow(qq[3],2)),2*qq[0]*qq[1]+2*qq[2]*qq[3],0,0,0,
	    		        		0,0,0,0, 2*qq[0]*qq[2]+2*qq[1]*qq[3],-2*qq[0]*qq[1]+2*qq[2]*qq[3],1-2*(pow(qq[1],2)+pow(qq[2],2)),0,0,0;

	    		        Satt2=(Hatt2*Patt)*Hatt2.transpose();

	    		        Satt2(0,0)=Satt2(0,0)+Ratt(4);Satt2(1,1)=Satt2(1,1)+Ratt(5);Satt2(2,2)=Satt2(2,2)+Ratt(6);

	    		        Katt2=  ( Patt*Hatt2.transpose())*Satt2.inverse();

	    		        yatt2<< QuadState->w_meas[0],
	    		        	    QuadState->w_meas[1],
	    		        	    QuadState->w_meas[2];


	    		        	        double wimu[3];
	    		        	        quat_inv(qq,QuadState->qmis);
	    		        	        quat_rot(wimu,qq,QuadState->w);

	    		        	        wimu[0]=wimu[0]+QuadState->bg[0];
	    		        	        wimu[1]=wimu[1]+QuadState->bg[1];
	    		        	        wimu[2]=wimu[2]+QuadState->bg[2];



	    		        	        yatt_est2<< wimu[0],
	    		        	            	    wimu[1],
	    		        	            	    wimu[2];

	    		        	        Inoatt=Katt2*(yatt2-yatt_est2);

	    		        	        QuadState->q[0]=QuadState->q[0]+Inoatt(0);
	    		        	        QuadState->q[1]=QuadState->q[1]+Inoatt(1);
	    		        	        QuadState->q[2]=QuadState->q[2]+Inoatt(2);
	    		        	        QuadState->q[3]=QuadState->q[3]+Inoatt(3);
	    		        	        NormQuat(QuadState->q);

	    		        	        QuadState->w[0]=QuadState->w[0]+Inoatt(4);
	    		        	        QuadState->w[1]=QuadState->w[1]+Inoatt(5);
	    		        	        QuadState->w[2]=QuadState->w[2]+Inoatt(6);

	    		        	        QuadState->wdot[0]=QuadState->wdot[0]+Inoatt(7);
	    		        	        QuadState->wdot[1]=QuadState->wdot[1]+Inoatt(8);
	    		        	        QuadState->wdot[2]=QuadState->wdot[2]+Inoatt(9);


	    		 Patt=Patt-Katt2*Hatt2*Patt;

	    	}
//############################################################################################################################################
    //translation-----------------------------------------------------------------------
	QuadState->x[0]=QuadState->x[0]+(dt)*QuadState->v[0];
	QuadState->x[1]=QuadState->x[1]+(dt)*QuadState->v[1];
	QuadState->x[2]=QuadState->x[2]+(dt)*QuadState->v[2];

	QuadState->v[0]=QuadState->v[0]+(dt)*QuadState->a[0];
	QuadState->v[1]=QuadState->v[1]+(dt)*QuadState->a[1];
	QuadState->v[2]=QuadState->v[2]+(dt)*QuadState->a[2];




    //trans cov prop
    Ftran<< 1,0,0,dt,0,0,0,0,0,
    		0,1,0,0,dt,0,0,0,0,
    		0,0,1,0,0,dt,0,0,0,
    		0,0,0,1,0,0,dt,0,0,
    		0,0,0,0,1,0,0,dt,0,
    		0,0,0,0,0,1,0,0,dt,
    		0,0,0,0,0,0,1,0,0,
    		0,0,0,0,0,0,0,1,0,
    		0,0,0,0,0,0,0,0,1;


    Ptran=(Ftran*Ptran)*Ftran.transpose();


    Ptran(0,0)=Ptran(0,0)+Qtran(0);Ptran(1,1)=Ptran(1,1)+Qtran(1);Ptran(2,2)=Ptran(2,2)+Qtran(2);Ptran(3,3)=Ptran(3,3)+Qtran(3);
    Ptran(4,4)=Ptran(4,4)+Qtran(4);Ptran(5,5)=Ptran(5,5)+Qtran(5);Ptran(6,6)=Ptran(6,6)+Qtran(6);
    Ptran(7,7)=Ptran(7,7)+Qtran(7);Ptran(8,8)=Ptran(8,8)+Qtran(8);


    if(QuadState->VICONupdateFLAG==1 )
    {


    //NOW THE UPDATE EQUATIONS----------- Trans
    qq[0]=QuadState->qmis[0];
    qq[1]=QuadState->qmis[1];
    qq[2]=QuadState->qmis[2];
    qq[3]=QuadState->qmis[3];
    quat_prod(qq,QuadState->q,QuadState->qmis);

    Htran<< 1,0,0,0,0,0,0,0,0,
    		0,1,0,0,0,0,0,0,0,
    		0,0,1,0,0,0,0,0,0,
    		0,0,0,0,0,0, 1-2*(pow(qq[2],2)+pow(qq[3],2)),2*qq[1]*qq[2]+2*qq[0]*qq[3],-2*qq[0]*qq[2]+2*qq[1]*qq[3],
    		0,0,0,0,0,0, 2*qq[1]*qq[2]-2*qq[0]*qq[3],1-2*(pow(qq[1],2)+pow(qq[3],2)),2*qq[0]*qq[1]+2*qq[2]*qq[3],
    		0,0,0,0,0,0, 2*qq[0]*qq[2]+2*qq[1]*qq[3],-2*qq[0]*qq[1]+2*qq[2]*qq[3],1-2*(pow(qq[1],2)+pow(qq[2],2));


    Stran=(Htran*Ptran)*Htran.transpose();

    Stran(0,0)=Stran(0,0)+Rtran(0);Stran(1,1)=Stran(1,1)+Rtran(1);Stran(2,2)=Stran(2,2)+Rtran(2);Stran(3,3)=Stran(3,3)+Rtran(3);
    Stran(4,4)=Stran(4,4)+Rtran(4);Stran(5,5)=Stran(5,5)+Rtran(5);


    Ktran=  ( Ptran*Htran.transpose())*Stran.inverse();

    ytran<<QuadState->x_meas[0],
    	   QuadState->x_meas[1],
    	   QuadState->x_meas[2],
    	   QuadState->a_meas[0],
    	   QuadState->a_meas[1],
    	   QuadState->a_meas[2];

    double aimu[3];
    AccTransform2imu(aimu,QuadState->a,QuadState->rmis,QuadState->q,QuadState->qmis,QuadState->w,QuadState->wdot,QuadState->ba);

    ytran_est<<QuadState->x[0],
        	   QuadState->x[1],
        	   QuadState->x[2],
        	   aimu[0],
        	   aimu[1],
        	   aimu[2];

    Inotran=Ktran*(ytran-ytran_est);
    QuadState->x[0]=QuadState->x[0]+Inotran(0);
    QuadState->x[1]=QuadState->x[1]+Inotran(1);
    QuadState->x[2]=QuadState->x[2]+Inotran(2);

    QuadState->v[0]=QuadState->v[0]+Inotran(3);
    QuadState->v[1]=QuadState->v[1]+Inotran(4);
    QuadState->v[2]=QuadState->v[2]+Inotran(5);

    QuadState->a[0]=QuadState->a[0]+Inotran(6);
    QuadState->a[1]=QuadState->a[1]+Inotran(7);
    QuadState->a[2]=QuadState->a[2]+Inotran(8);


    Ptran=Ptran-Ktran*Htran*Ptran;
    }
    else {

    	    qq[0]=QuadState->qmis[0];
    	    qq[1]=QuadState->qmis[1];
    	    qq[2]=QuadState->qmis[2];
    	    qq[3]=QuadState->qmis[3];
    	    quat_prod(qq,QuadState->q,QuadState->qmis);

    	Htran2<<    0,0,0,0,0,0, 1-2*(pow(qq[2],2)+pow(qq[3],2)),2*qq[1]*qq[2]+2*qq[0]*qq[3],-2*qq[0]*qq[2]+2*qq[1]*qq[3],
    	    		0,0,0,0,0,0, 2*qq[1]*qq[2]-2*qq[0]*qq[3],1-2*(pow(qq[1],2)+pow(qq[3],2)),2*qq[0]*qq[1]+2*qq[2]*qq[3],
    	    		0,0,0,0,0,0, 2*qq[0]*qq[2]+2*qq[1]*qq[3],-2*qq[0]*qq[1]+2*qq[2]*qq[3],1-2*(pow(qq[1],2)+pow(qq[2],2));

    	    Stran2=(Htran2*Ptran)*Htran2.transpose();

    	    Stran2(0,0)=Stran2(0,0)+Rtran(3);Stran2(1,1)=Stran2(1,1)+Rtran(4);Stran2(2,2)=Stran2(2,2)+Rtran(5);

        Ktran2=  ( Ptran*Htran2.transpose())*Stran2.inverse();

        ytran2<<QuadState->a_meas[0],
        	    QuadState->a_meas[1],
        	    QuadState->a_meas[2];

        double aimu[3];
        AccTransform2imu(aimu,QuadState->a,QuadState->rmis,QuadState->q,QuadState->qmis,QuadState->w,QuadState->wdot,QuadState->ba);

        ytran_est2<<aimu[0],
            	   aimu[1],
            	   aimu[2];

        Inotran=Ktran2*(ytran2-ytran_est2);
        QuadState->x[0]=QuadState->x[0]+Inotran(0);
        QuadState->x[1]=QuadState->x[1]+Inotran(1);
        QuadState->x[2]=QuadState->x[2]+Inotran(2);

        QuadState->v[0]=QuadState->v[0]+Inotran(3);
        QuadState->v[1]=QuadState->v[1]+Inotran(4);
        QuadState->v[2]=QuadState->v[2]+Inotran(5);

        QuadState->a[0]=QuadState->a[0]+Inotran(6);
        QuadState->a[1]=QuadState->a[1]+Inotran(7);
        QuadState->a[2]=QuadState->a[2]+Inotran(8);

    	  Ptran=Ptran-Ktran2*Htran2*Ptran;
    	    	}

    QuadState->VICONupdateFLAG=0;
    QuadState->IMUupdateFLAG=0;
}


/*
 * Printing all the estimator filter values
 */
void EKF_Estimator1::PrintFiltervals(){
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
	cout<<"yatt_est = "<<endl<<yatt_est<<endl;
	cout<<"Inoatt = "<<endl<<Inoatt<<endl<<endl;


	cout<<"------------------ Translational Fields----------------------------"<<endl<<endl;
		cout<<"Ftran = "<<endl<<Ftran<<endl;
		cout<<"Ptran = "<<endl<<Ptran<<endl;
		cout<<"Qtran = "<<endl<<Qtran<<endl;
		cout<<"Rtran = "<<endl<<Rtran<<endl;
		cout<<"Htran = "<<endl<<Htran<<endl;
		cout<<"Stran = "<<endl<<Stran<<endl;
		cout<<"Ktran = "<<endl<<Ktran<<endl;
		cout<<"ytran = "<<endl<<ytran<<endl;
		cout<<"ytran_est = "<<endl<<ytran_est<<endl;
		cout<<"Inotran = " <<endl<<Inotran<<endl<<endl;

}


/****************************************************************************************************************
 *                           SIMPLE ESTIMATOR
 *********************************************************************************************/



void Simple_Estimator_onboard::InitializeFilter(){



			Ratt[0]=0.1;Ratt[1]=0.1;// q-w

            Rtran[0]=0.1;Rtran[1]=0.6; //x-a



}

void Simple_Estimator_onboard::PropagateUpdateMeanCov(QuadStateVariable *QuadState){
	double qq[4],w_quat[4],t1,t2,dtimu,dtvic;

		double w1,w2,w3,q0,q1,q2,q3;
		int Cv[3]={0,1,2};

			w1=QuadState->w[0];
			w2=QuadState->w[1];
			w3=QuadState->w[2];

			q0=QuadState->q[0];
			q1=QuadState->q[1];
			q2=QuadState->q[2];
			q3=QuadState->q[3];

		QuadState->tonboard.GetLastRowQ(&t2);
		QuadState->tonboard.GetLastButOneRowQ(&t1);

		dtimu=t2-t1;
		if(dtimu==0)
			dtimu=0.01;

		QuadState->tground.GetLastRowQ(&t2);
		QuadState->tground.GetLastButOneRowQ(&t1);

		dtvic=t2-t1;
		if(dtvic==0)
			dtvic=0.01;


		Vec2Quat(w_quat,QuadState->w);

		quat_prod(qq, QuadState->q,w_quat);

		//att mean prop  wdot remains the same
		QuadState->q[0]=QuadState->q[0]+(dtimu/2)*qq[0];
		QuadState->q[1]=QuadState->q[1]+(dtimu/2)*qq[1];
		QuadState->q[2]=QuadState->q[2]+(dtimu/2)*qq[2];
		QuadState->q[3]=QuadState->q[3]+(dtimu/2)*qq[3];
		NormQuat(QuadState->q);

		QuadState->w[0]=QuadState->w[0]+dtimu*QuadState->M[0]/QuadState->I[0];
		QuadState->w[1]=QuadState->w[1]+dtimu*QuadState->M[1]/QuadState->I[1];
		QuadState->w[2]=QuadState->w[2]+dtimu*QuadState->M[2]/QuadState->I[2];


		QuadState->wdot[0]=0;
		QuadState->wdot[1]=0;
		QuadState->wdot[2]=0;


		    if(QuadState->VICONupdateFLAG==1)
		   	    {


		        QuadState->q[0]=Ratt[0]*QuadState->q[0]+(1-Ratt[0])*QuadState->q_meas[0];
		        QuadState->q[1]=Ratt[0]*QuadState->q[1]+(1-Ratt[0])*QuadState->q_meas[1];
		        QuadState->q[2]=Ratt[0]*QuadState->q[2]+(1-Ratt[0])*QuadState->q_meas[2];
		        QuadState->q[3]=Ratt[0]*QuadState->q[3]+(1-Ratt[0])*QuadState->q_meas[3];
		        NormQuat(QuadState->q);

		        QuadState->w[0]=Ratt[1]*QuadState->w[0]+(1-Ratt[1])*(QuadState->w_meas[0]-QuadState->bg[0]);
		        QuadState->w[1]=Ratt[1]*QuadState->w[0]+(1-Ratt[1])*(QuadState->w_meas[1]-QuadState->bg[1]);
		        QuadState->w[2]=Ratt[1]*QuadState->w[0]+(1-Ratt[1])*(QuadState->w_meas[2]-QuadState->bg[2]);
		   	    }
		    else
		    	{

		    	 QuadState->w[0]=Ratt[1]*QuadState->w[0]+(1-Ratt[1])*(QuadState->w_meas[0]-QuadState->bg[0]);
		    	 QuadState->w[1]=Ratt[1]*QuadState->w[0]+(1-Ratt[1])*(QuadState->w_meas[1]-QuadState->bg[1]);
		    	 QuadState->w[2]=Ratt[1]*QuadState->w[0]+(1-Ratt[1])*(QuadState->w_meas[2]-QuadState->bg[2]);

		    	}
	//############################################################################################################################################
	    //translation-----------------------------------------------------------------------
		double Vest_diff[3],ainertmeas[3],xmeasprev[3],ameasprev[3],ainertmeasprev[3];

		QuadState->x[0]=QuadState->x[0]+(dtimu)*QuadState->v[0];
		QuadState->x[1]=QuadState->x[1]+(dtimu)*QuadState->v[1];
		QuadState->x[2]=QuadState->x[2]+(dtimu)*QuadState->v[2];

		AccImu2Inert(ainertmeas,QuadState->q,QuadState->a_meas,QuadState->ba);

		QuadState->Data_a_meas.GetLastButOneRowQ(ameasprev);
		AccImu2Inert(ainertmeasprev,QuadState->q,ameasprev,QuadState->ba);

		QuadState->v[0]=QuadState->v[0]+dtimu*(ainertmeas[0]+ainertmeasprev[0])/2;
		QuadState->v[1]=QuadState->v[1]+dtimu*(ainertmeas[1]+ainertmeasprev[1])/2;
		QuadState->v[2]=QuadState->v[2]+dtimu*(ainertmeas[2]+ainertmeasprev[2])/2;



		if(QuadState->VICONupdateFLAG==1 )
	    {

	        QuadState->Data_x_meas.GetLastButOneRowQ(xmeasprev);
			Vest_diff[0]=(QuadState->x_meas[0]-xmeasprev[0])/dtvic;
			Vest_diff[1]=(QuadState->x_meas[1]-xmeasprev[1])/dtvic;
			Vest_diff[2]=(QuadState->x_meas[2]-xmeasprev[2])/dtvic;
			QuadState->Data_Vdiff.AppendQ(Vest_diff,Cv);
			//QuadState->Data_Vdiff.MeanQ(0,QuadState->Data_Vdiff.rows-1);
			//QuadState->Data_Vdiff.GetMeanQ(Vest_diff);

			//cout<<QuadState->x_meas[0]<<" "<<xmeasprev[0]<<" "<<dtvic<<endl;

			        QuadState->x[0]=Rtran[0]*QuadState->x[0]+(1-Rtran[0])*QuadState->x_meas[0];
			        QuadState->x[1]=Rtran[0]*QuadState->x[1]+(1-Rtran[0])*QuadState->x_meas[1];
			        QuadState->x[2]=Rtran[0]*QuadState->x[2]+(1-Rtran[0])*QuadState->x_meas[2];




					QuadState->v[0]=Rtran[1]*QuadState->v[0]-(1-Rtran[1])*Vest_diff[0];
					QuadState->v[1]=Rtran[1]*QuadState->v[1]-(1-Rtran[1])*Vest_diff[1];
					QuadState->v[2]=Rtran[1]*QuadState->v[2]-(1-Rtran[1])*Vest_diff[2];

	    }

		QuadState->Data_Vest.AppendQ(QuadState->v,Cv);
		QuadState->Data_Vest.MeanQ(2,QuadState->Data_Vest.rows-1);
		QuadState->Data_Vest.GetMeanQ(QuadState->v);

		QuadState->VICONupdateFLAG=0;
		QuadState->IMUupdateFLAG=0;
}




void Simple_Estimator_onboard::AngRateInert2Imu(double *wimu,QuadStateVariable *QuadState){

}
void Simple_Estimator_onboard::AngRateImu2Inert(double *winert,QuadStateVariable *QuadState){

}



void AccImu2Inert(double *ainert,double * q,double * a_meas,double * ba){
	double aimu[3];


    //a=q(aimu-ba)qinv-[0,0,9.8]
	aimu[0]=a_meas[0]-ba[0];
	aimu[1]=a_meas[1]-ba[1];
	aimu[2]=a_meas[2]-ba[2];
	quat_rot(ainert,q,aimu);
	ainert[2]=ainert[2]-9.81;



}

void AccInert2Imu(double *aimu,double * q,double * ainert,double * ba){
	  double qinv[4];
	  double aa[3];
	  aa[0]=ainert[0];
	  aa[1]=ainert[1];
	  aa[2]=ainert[2]+9.81;

	  quat_inv(qinv,q);
	  quat_rot(aimu,qinv,aa);
	  aimu[0]=aimu[0]+ba[0];
	  aimu[1]=aimu[1]+ba[1];
	  aimu[2]=aimu[2]+ba[2];
}

/*
 * Transform the inertial acc of body frame center to IMU frame center acc (this is what the accelerometer reads+ gravity).
 * Further any mis-alignmet is also taken care off (you have to provide misalignment quat and rmis (measured or estimated))
 */
void AccTransform2imu(double *aimu,double * a,double * r,double * q,double * qmis,double * w,double * wdot,double * ba){
double qq[4],qqinv[4],B[4],w_quat[4],wdot_quat[4],r_quat[4],qq1[4],qq2[4],Bvec[3];


Vec2Quat(w_quat,w);
Vec2Quat(wdot_quat,wdot);
Vec2Quat(r_quat,r);

B[0]=0;B[1]=0;B[2]=0;B[3]=0;

quat_add(B, quat_prod(qq1,quat_prod(qq,w_quat,w_quat),r_quat), quat_prod(qq2,quat_prod(qq,r_quat,w_quat),w_quat));
quat_add(B,B,quat_scalar_prod(qq2,quat_prod(qq1,quat_prod(qq,w_quat,r_quat),w_quat),-2));
quat_add(B,B,quat_scalar_prod(qq2,quat_prod(qq1,wdot_quat,r_quat),2));
quat_add(B,B,quat_scalar_prod(qq2,quat_prod(qq1,r_quat,wdot_quat),-2));


quat_rot(Bvec,q,B+1);

Bvec[0]=a[0]+0.25*Bvec[0];
Bvec[1]=a[1]+0.25*Bvec[1];
Bvec[2]=a[2]+0.25*Bvec[2]+9.8;

quat_prod(qq, q,qmis);
quat_inv(qqinv,qq);

quat_rot(aimu,qqinv,Bvec);

aimu[0]=aimu[0]+ba[0];
aimu[1]=aimu[1]+ba[1];
aimu[2]=aimu[2]+ba[2];
}



/*
 * Initialize the IMU sensor by finding the bias
 * Known: qstat: the present stationary orientation of vicon frame/body fromae
 *        rmis:  the rmis vector  in QuadState is used. If this is not correct update it in Quadstate and then call
 *        qmis: the inter lock quaternion  in QuadState or the default [1 0 0 0] is used
 *
 * Set:   ba
 *        bg
 *        stdev
 *        maxstdev
 */
void Initialize_IMU_onboard(QuadStateVariable * QuadState,MPU6050 *AccelGyroSensor,double *qstat){
	double accelgyromeas[7];
	SimpleTimer InitParaEst_timer;
	InitParaEst_timer.SetTsec(0.01);
	InitParaEst_timer.ResetTimer();


	int i=0;
	int N=200;  //the number of measurements to take to initialize the biases
	double bg[]={0,0,0},ba[]={0,0,0};
	double aimu_est[3];

	for(i=0;i<N;i++){

		InitParaEst_timer.ResetTimer();

		AccelGyroSensor->getScaledaccgyro_timestamped(accelgyromeas);
		QuadState->Update_AccelGyro_meas(accelgyromeas,0);  //load it into the queue

		QuadState->Data_w_meas.StdQ(0,data_queue_length-1); //calling the StdQ will calculate mean and maxstdev also
		QuadState->Data_a_meas.StdQ(0,data_queue_length-1);

	//	AccTransform2imu(double *aimu,double * a,double * r,double * q,double * qmis,double * w,double * wdot,double * ba)
		AccTransform2imu(aimu_est,QuadState->a,QuadState->rmis,qstat,QuadState->qmis,QuadState->w,QuadState->wdot,QuadState->ba);


		bg[0]=(i*bg[0]+QuadState->Data_w_meas.mu[0])/(i+1);
		bg[1]=(i*bg[1]+QuadState->Data_w_meas.mu[1])/(i+1);
		bg[2]=(i*bg[2]+QuadState->Data_w_meas.mu[2])/(i+1);

		//aimu_meas=aimu_est+ba
		ba[0]=(i*ba[0]+(QuadState->Data_a_meas.mu[0]-aimu_est[0]))/(i+1);
		ba[1]=(i*ba[1]+(QuadState->Data_a_meas.mu[1]-aimu_est[1]))/(i+1);
		ba[2]=(i*ba[2]+(QuadState->Data_a_meas.mu[2]-aimu_est[2]))/(i+1);


		while(InitParaEst_timer.CheckTimer()){} //wait for 0.01s to get over


	}

	QuadState->Update_bg(bg);
	QuadState->Update_ba(ba);

}


/*
 * To be completed:
 * Set:   qstat
 *        maxstdev
 *        stdev
 */
void Initialize_QX_onboard(QuadStateVariable * QuadState,Serial_comm_Ground_Onboard *GroundComm,double * qstat){

}


/*
 *To be run after vicon measurement inet has been called...we need the stationary q value
 * Run this function on the ground station to initialize and stabilize the vicon
 * parameters that are update: ba and bg and maxdevs
 */
void Initialize_IMU_ground(QuadStateVariable * QuadState ,SimpleQueue *QQ){
	QQ->StdQ(0,QQ->rows-1);

	QuadState->Data_a_meas.maxstdev[0]=QQ->maxstdev[0];
	QuadState->Data_a_meas.maxstdev[1]=QQ->maxstdev[1];
	QuadState->Data_a_meas.maxstdev[2]=QQ->maxstdev[2];

	QuadState->Data_w_meas.maxstdev[0]=QQ->maxstdev[3];
	QuadState->Data_w_meas.maxstdev[1]=QQ->maxstdev[4];
	QuadState->Data_w_meas.maxstdev[2]=QQ->maxstdev[5];

	//estimating ba
	double zz[3]={0,0,0},aimu[3];
	AccTransform2imu(aimu,zz,QuadState->rmis,QuadState->q,QuadState->qmis,zz,zz,zz);

	QuadState->ba[0]=QQ->mu[0]-aimu[0];
	QuadState->ba[1]=QQ->mu[1]-aimu[1];
	QuadState->ba[2]=QQ->mu[2]-aimu[2];


	//as the body is stationary w=0. Hence bg=wimu
	QuadState->bg[0]=QQ->mu[3];
	QuadState->bg[1]=QQ->mu[4];
	QuadState->bg[2]=QQ->mu[5];

}


/*
 *
 * Run this function on Ground station to initialize the IMU variables from onboard
 * parameters that are update: max devs
 */
void Initialize_QX_ground(QuadStateVariable * QuadState,SimpleQueue *QQ ){
	QQ->StdQ(0,QQ->rows-1);

	//use the meas value to form initial estimates
	QuadState->x[0]=QQ->mu[0];
	QuadState->x[1]=QQ->mu[1];
	QuadState->x[2]=QQ->mu[2];

	QuadState->q[0]=QQ->mu[3];
	QuadState->q[1]=QQ->mu[4];
	QuadState->q[2]=QQ->mu[5];
	QuadState->q[3]=QQ->mu[6];
    NormQuat(QuadState->q);
    // update the maxstddevs
	QuadState->Data_x_meas.maxstdev[0]=QQ->maxstdev[0];
	QuadState->Data_x_meas.maxstdev[1]=QQ->maxstdev[1];
	QuadState->Data_x_meas.maxstdev[2]=QQ->maxstdev[2];

	QuadState->Data_q_meas.maxstdev[0]=QQ->maxstdev[3];
	QuadState->Data_q_meas.maxstdev[1]=QQ->maxstdev[4];
	QuadState->Data_q_meas.maxstdev[2]=QQ->maxstdev[5];
	QuadState->Data_q_meas.maxstdev[3]=QQ->maxstdev[6];

}


/*
 *To be run after vicon measurement inet has been called...we need the stationary q value
 * Run this function on the ground station to initialize and stabilize the vicon
 * parameters that are update: ba and bg and maxdevs
 */
void Initialize_ba_bg_est(QuadStateVariable * QuadState ,SimpleQueue *QQvic,SimpleQueue *QQimu,SimpleQueue *BA,SimpleQueue *BG){
	int n;
	double Dvic[7],Dimu[6],ba[3],bg[3];
	double zz[3]={0,0,0},aimu[3],q[4];
	int C[3]={0,1,2};
	n=QQvic->rows;
	
	for(int i=0;i<n;i++){
	QQvic->GetRowQ(i,Dvic);
	QQimu->GetRowQ(i,Dimu);
	q[0]=Dvic[3];
	q[1]=Dvic[4];
	q[2]=Dvic[5];
	q[3]=Dvic[6];
	
	AccTransform2imu(aimu,zz,QuadState->rmis,q,QuadState->qmis,zz,zz,zz);
	aimu[0]=Dimu[0]-aimu[0];
	aimu[1]=Dimu[1]-aimu[1];
	aimu[2]=Dimu[2]-aimu[2];
	
	BA->AppendQ(aimu,C);
	
	aimu[0]=Dimu[3];
	aimu[1]=Dimu[4];
	aimu[2]=Dimu[5];
	BG->AppendQ(aimu,C);
	
	}
		BA->StdQ(0,BA->rows-1);
		//as the body is stationary w=0. Hence bg=wimu
	QuadState->ba[0]=BA->mu[0];
	QuadState->ba[1]=BA->mu[1];
	QuadState->ba[2]=BA->mu[2];
	
     BG->StdQ(0,BG->rows-1);
	//as the body is stationary w=0. Hence bg=wimu
	QuadState->bg[0]=BG->mu[0];
	QuadState->bg[1]=BG->mu[1];
	QuadState->bg[2]=BG->mu[2];
	
	
	
	QQvic->StdQ(0,QQvic->rows-1);
	QQimu->StdQ(0,QQimu->rows-1);
	QuadState->Data_a_meas.maxstdev[0]=QQimu->maxstdev[0];
	QuadState->Data_a_meas.maxstdev[1]=QQimu->maxstdev[1];
	QuadState->Data_a_meas.maxstdev[2]=QQimu->maxstdev[2];

	QuadState->Data_w_meas.maxstdev[0]=QQimu->maxstdev[3];
	QuadState->Data_w_meas.maxstdev[1]=QQimu->maxstdev[4];
	QuadState->Data_w_meas.maxstdev[2]=QQimu->maxstdev[5];

	QuadState->Data_x_meas.maxstdev[0]=QQvic->maxstdev[0];
	QuadState->Data_x_meas.maxstdev[1]=QQvic->maxstdev[1];
	QuadState->Data_x_meas.maxstdev[2]=QQvic->maxstdev[2];

	QuadState->Data_q_meas.maxstdev[0]=QQvic->maxstdev[3];
	QuadState->Data_q_meas.maxstdev[1]=QQvic->maxstdev[4];
	QuadState->Data_q_meas.maxstdev[2]=QQvic->maxstdev[5];
	QuadState->Data_q_meas.maxstdev[3]=QQvic->maxstdev[6];


}

/*
 *To be run after vicon measurement inet has been called...we need the stationary q value
 * Run this function on the ground station to initialize and stabilize the vicon
 * parameters that are update: ba and bg and maxdevs
 */
void Initialize_ba_bg_est_simplified(QuadStateVariable * QuadState ,SimpleQueue *BA,SimpleQueue *BG){

		BA->MeanQ(0,BA->rows-1);
		//as the body is stationary w=0. Hence bg=wimu
	QuadState->ba[0]=BA->mu[0];
	QuadState->ba[1]=BA->mu[1];
	QuadState->ba[2]=BA->mu[2];

     BG->MeanQ(0,BG->rows-1);
	//as the body is stationary w=0. Hence bg=wimu
	QuadState->bg[0]=BG->mu[0];
	QuadState->bg[1]=BG->mu[1];
	QuadState->bg[2]=BG->mu[2];



}
