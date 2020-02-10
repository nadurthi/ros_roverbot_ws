#ifndef _CONTROLLERS_H_
#define _CONTROLLERS_H_

#include <cmath>
#include<iostream>
#include "State_data_manager.h"
#include "quaternion_operations.h"
#include "/home/nagnanamus/Dropbox/BBB/codes/quad_control/source_codes/Eigen/Dense"

typedef Eigen::Matrix<double, 3, 3> Matrix3d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;

using namespace std;
//define quad properties

double limitto(double v,double lb,double ub){
	if(v>ub)
		v=ub;

	if(v<lb)
		v=lb;

	return v;
}


double signum(double q0){
	if(q0 > 0)
		return 1.0;

	if(q0 < 0)
		return -1.0;

	if(q0==0)
		return 0;

}



double f2rpm(double f){
	double rpm;
	f=(double)f/9.8;
	if(f<0)
	f=0;

	rpm=(-1621*pow(f,2)+6548*f+1313);

	if(rpm<900)
		rpm=900;

	if(rpm>9000)
		rpm=9000;


	return rpm;
}
double f2pwm(double f){
	double rpm;
	f=(double)f/9.8;
	if(f<0)
	f=0;

	if(f>25)
		f=25;

	rpm=(int)118*f+2000;


	return rpm;
}


void ComputeNonlinearControl(QuadStateVariable *QuadStates){


	// first compute the control for x,y,z error and total force F
     double ew[3],ex[3],ev[3],fd[3],R[3][3],b3d[3],b2d[3],nb1d[3],eq[4],qinv[4];
     double wdt[3];
     ex[0]=QuadStates->x[0]-QuadStates->xref[0];
     ex[1]=QuadStates->x[1]-QuadStates->xref[1];
     ex[2]=QuadStates->x[2]-QuadStates->xref[2];

     ev[0]=QuadStates->v[0]-QuadStates->vref[0];
     ev[1]=QuadStates->v[1]-QuadStates->vref[1];
     ev[2]=QuadStates->v[2]-QuadStates->vref[2];



     fd[0]=-QuadStates->kx*ex[0]-QuadStates->kv*ev[0]+ QuadStates->aref[0]*QuadStates->Quadmass;
     fd[1]=-QuadStates->kx*ex[1]-QuadStates->kv*ev[1]+ QuadStates->aref[1]*QuadStates->Quadmass;
     fd[2]=-QuadStates->kx/2*ex[2]-QuadStates->kv/2*ev[2]+ QuadStates->mg+ QuadStates->aref[2]*QuadStates->Quadmass;

     UnitVec(b3d,fd);
     UnitVec(b2d,CrossProd(b2d,b3d,QuadStates->b1ref));
     CrossProd(nb1d,b2d,b3d);

     R[0][0]=nb1d[0];R[1][0]=nb1d[1];R[2][0]=nb1d[2];
     R[0][1]=b2d[0];R[1][1]=b2d[1];R[2][1]=b2d[2];
     R[0][2]=b3d[0];R[1][2]=b3d[1];R[2][2]=b3d[2];

     //generate the desired quaternion qd

     Rot2Quat(QuadStates->qdes,R);

     // required thrust
     quat_inv(qinv,QuadStates->qdes);
     quat_prod(eq,qinv,QuadStates->q);

     QuadStates->Fd=NormVec(fd);
     QuadStates->Fd=QuadStates->Fd*(2*pow(eq[0],2)+2*pow(eq[3],2)-1);

     if(abs(QuadStates->Fd)>QuadStates->Fsafety)
    	 QuadStates->Fd=QuadStates->Fsafety;


	//compute the required moments

     	quat_rot(wdt,quat_inv(qinv,eq),QuadStates->wref);



     	ew[0]=QuadStates->w[0]-wdt[0];
     	ew[1]=QuadStates->w[1]-wdt[1];
     	ew[2]=QuadStates->w[2]-wdt[2];


        double sg=signum(eq[0]);
        sg=1;
     	QuadStates->M[0]=-sg*QuadStates->kq*eq[1]-QuadStates->kw*limitto(ew[0],-4,4);
     	QuadStates->M[1]=-sg*QuadStates->kq*eq[2]-QuadStates->kw*limitto(ew[1],-4,4);
     	QuadStates->M[2]=-sg*QuadStates->kq*eq[3]-QuadStates->kw*limitto(ew[2],-4,4);

	// compute the individual rpms for each rotor.

     	QuadStates->u[0]=QuadStates->Fd/4-QuadStates->M[1]/(2*QuadStates->MomArm)+QuadStates->M[2]/(4*QuadStates->ctf);
     	QuadStates->u[1]=QuadStates->Fd/4+QuadStates->M[0]/(2*QuadStates->MomArm)-QuadStates->M[2]/(4*QuadStates->ctf);
     	QuadStates->u[2]=QuadStates->Fd/4+QuadStates->M[1]/(2*QuadStates->MomArm)+QuadStates->M[2]/(4*QuadStates->ctf);
     	QuadStates->u[3]=QuadStates->Fd/4-QuadStates->M[0]/(2*QuadStates->MomArm)-QuadStates->M[2]/(4*QuadStates->ctf);
double FF=QuadStates->Fd;
     	while(QuadStates->u[0]<0 || QuadStates->u[1]<0 ||QuadStates->u[2]<0||QuadStates->u[3]<0){
     		FF=FF+0.2;
     		        QuadStates->u[0]=FF/4-QuadStates->M[1]/(2*QuadStates->MomArm)+QuadStates->M[2]/(4*QuadStates->ctf);
     		     	QuadStates->u[1]=FF/4+QuadStates->M[0]/(2*QuadStates->MomArm)-QuadStates->M[2]/(4*QuadStates->ctf);
     		     	QuadStates->u[2]=FF/4+QuadStates->M[1]/(2*QuadStates->MomArm)+QuadStates->M[2]/(4*QuadStates->ctf);
     		     	QuadStates->u[3]=FF/4-QuadStates->M[0]/(2*QuadStates->MomArm)-QuadStates->M[2]/(4*QuadStates->ctf);
     	}

}


/*
 * ATTITUDE only CONTROLLER
 * Have to supply:   Fsafety as the thrust force
 * 					 All gaisn
 * 					 qref,wref
 *
 */
void ComputeControl_att(QuadStateVariable *QuadStates){


	// first compute the control for x,y,z error and total force F
     double ew[3],eq[4],qinv[4];
     double wdt[3];


     //desired quaternion qd is nothing but the qref

     quat_inv(qinv,QuadStates->qref);
     quat_prod(eq,qinv,QuadStates->q);

     QuadStates->Fd=QuadStates->Fsafety;


	//compute the required moments

     	quat_rot(wdt,quat_inv(qinv,eq),QuadStates->wref);

     	ew[0]=QuadStates->w[0]-wdt[0];
     	ew[1]=QuadStates->w[1]-wdt[1];
     	ew[2]=QuadStates->w[2]-wdt[2];

        double sg=signum(eq[0]);
        sg=1;
     	QuadStates->M[0]=-sg*QuadStates->kq*eq[1]-QuadStates->kw*limitto(ew[0],-1,1);
     	QuadStates->M[1]=-sg*QuadStates->kq*eq[2]-QuadStates->kw*limitto(ew[1],-1,1);
     	QuadStates->M[2]=-sg*QuadStates->kq*eq[3]-QuadStates->kw*limitto(ew[2],-1,1);

	// compute the individual rpms for each rotor.

     	QuadStates->u[0]=QuadStates->Fd/4-QuadStates->M[1]/(2*QuadStates->MomArm)+QuadStates->M[2]/(4*QuadStates->ctf);
     	QuadStates->u[1]=QuadStates->Fd/4+QuadStates->M[0]/(2*QuadStates->MomArm)-QuadStates->M[2]/(4*QuadStates->ctf);
     	QuadStates->u[2]=QuadStates->Fd/4+QuadStates->M[1]/(2*QuadStates->MomArm)+QuadStates->M[2]/(4*QuadStates->ctf);
     	QuadStates->u[3]=QuadStates->Fd/4-QuadStates->M[0]/(2*QuadStates->MomArm)-QuadStates->M[2]/(4*QuadStates->ctf);
double FF=QuadStates->Fd;
     	while(QuadStates->u[0]<0 || QuadStates->u[1]<0 ||QuadStates->u[2]<0||QuadStates->u[3]<0){
     		FF=FF+0.2;
     		        QuadStates->u[0]=FF/4-QuadStates->M[1]/(2*QuadStates->MomArm)+QuadStates->M[2]/(4*QuadStates->ctf);
     		     	QuadStates->u[1]=FF/4+QuadStates->M[0]/(2*QuadStates->MomArm)-QuadStates->M[2]/(4*QuadStates->ctf);
     		     	QuadStates->u[2]=FF/4+QuadStates->M[1]/(2*QuadStates->MomArm)+QuadStates->M[2]/(4*QuadStates->ctf);
     		     	QuadStates->u[3]=FF/4-QuadStates->M[0]/(2*QuadStates->MomArm)-QuadStates->M[2]/(4*QuadStates->ctf);
     	}

}
/*
 * ATTITUDE only CONTROLLER  -- Euler angle control
 * Have to supply:   Fsafety as the thrust force
 * 					 All gaisn
 * 					 qref,wref
 *
 */
void ComputeControl_Euleratt(QuadStateVariable *QuadStates){


	// first compute the control for x,y,z error and total force F
     double ew[3],eq[4],qinv[4],Eul[3],Eulref[3];
     double wdt[3];

     Quat2Euler(Eul ,QuadStates->q);//z-x-y
     Quat2Euler(Eulref ,QuadStates->qref);//z-x-y

     //desired quaternion qd is nothing but the qref

    quat_inv(qinv,QuadStates->qref);
    quat_prod(eq,qinv,QuadStates->q);

     QuadStates->Fd=QuadStates->Fsafety;


	//compute the required moments

     	quat_rot(wdt,quat_inv(qinv,eq),QuadStates->wref);

     	ew[0]=QuadStates->w[0]-wdt[0];
     	ew[1]=QuadStates->w[1]-wdt[1];
     	ew[2]=QuadStates->w[2]-wdt[2];

     	QuadStates->M[1]=-QuadStates->kq*(Eul[1]-Eulref[1])-QuadStates->kw*ew[1];
     	QuadStates->M[0]=-QuadStates->kq*(Eul[2]-Eulref[2])-QuadStates->kw*ew[0];
     	QuadStates->M[2]=-QuadStates->kq*(Eul[0]-Eulref[0])-QuadStates->kw*ew[2];

     	//QuadStates->M[0]=-sg*QuadStates->kq*eq[1]-QuadStates->kw*ew[0];
     	//QuadStates->M[1]=-sg*QuadStates->kq*eq[2]-QuadStates->kw*ew[1];
     	//QuadStates->M[2]=-sg*QuadStates->kq*eq[3]-QuadStates->kw*ew[2];

	// compute the individual rpms for each rotor.

      	QuadStates->u[0]=QuadStates->Fd/4-QuadStates->M[1]/(2*QuadStates->MomArm)+QuadStates->M[2]/(4*QuadStates->ctf);
         	QuadStates->u[1]=QuadStates->Fd/4+QuadStates->M[0]/(2*QuadStates->MomArm)-QuadStates->M[2]/(4*QuadStates->ctf);
         	QuadStates->u[2]=QuadStates->Fd/4+QuadStates->M[1]/(2*QuadStates->MomArm)+QuadStates->M[2]/(4*QuadStates->ctf);
         	QuadStates->u[3]=QuadStates->Fd/4-QuadStates->M[0]/(2*QuadStates->MomArm)-QuadStates->M[2]/(4*QuadStates->ctf);
    double FF=QuadStates->Fd;
         	while(QuadStates->u[0]<0 || QuadStates->u[1]<0 ||QuadStates->u[2]<0||QuadStates->u[3]<0){
         		FF=FF+0.2;
         		        QuadStates->u[0]=FF/4-QuadStates->M[1]/(2*QuadStates->MomArm)+QuadStates->M[2]/(4*QuadStates->ctf);
         		     	QuadStates->u[1]=FF/4+QuadStates->M[0]/(2*QuadStates->MomArm)-QuadStates->M[2]/(4*QuadStates->ctf);
         		     	QuadStates->u[2]=FF/4+QuadStates->M[1]/(2*QuadStates->MomArm)+QuadStates->M[2]/(4*QuadStates->ctf);
         		     	QuadStates->u[3]=FF/4-QuadStates->M[0]/(2*QuadStates->MomArm)-QuadStates->M[2]/(4*QuadStates->ctf);
         	}

}
/*
 * ATTITUDE only CONTROLLER for only 1 axis namely : about the y-axis. only M2/M[1] is set
 * Have to supply:   Fsafety as the thrust force
 * 					 All gaisn
 * 					 qref,wref
 *
 */
void ComputeSingleAxis_att(QuadStateVariable *QuadStates){
	double Eul[3],qdot[4],q[4];

			//compute the required moments

		     double w[4],qinv[4],wdt[3],ew[3],Eulref[3];
		     	quat_prod(w,quat_inv(qinv,q),qdot);
		     	//quat_rot(wdt,quat_inv(qinv,q),wd);
		     	//quat_rot(wdt,eq,wd);



		Quat2Euler(Eul ,QuadStates->q);//z-x-y
		Quat2Euler(Eulref ,QuadStates->qref);//z-x-y

		 QuadStates->M[1]=-QuadStates->kq*(Eul[1]-Eulref[1])-QuadStates->kw*QuadStates->w[1];

	     QuadStates->u[0]=(QuadStates->Fsafety/2-QuadStates->M[1]/(2*QuadStates->MomArm));
	     QuadStates->u[2]=(QuadStates->Fsafety/2+QuadStates->M[1]/(2*QuadStates->MomArm));

	     QuadStates->u[1]=(0);
	     QuadStates->u[3]=(0);



}


/*
Raffael paper controller
Angular rate shaping by first order system in quaternions
*/
void Control_angrate_shaping(QuadStateVariable *QuadStates){


	// first compute the control for x,y,z error and total force F
     double ew[3],ex[3],ev[3],fd[3],R[3][3],b3d[3],b2d[3],nb1d[3],eq[4],qinv[4];
     double wdt[3],wref[4],Eulref[3];
     ex[0]=QuadStates->x[0]-QuadStates->xref[0];
     ex[1]=QuadStates->x[1]-QuadStates->xref[1];
     ex[2]=QuadStates->x[2]-QuadStates->xref[2];

     ev[0]=QuadStates->v[0]-QuadStates->vref[0];
     ev[1]=QuadStates->v[1]-QuadStates->vref[1];
     ev[2]=QuadStates->v[2]-QuadStates->vref[2];



     fd[0]=-QuadStates->kx*ex[0]-QuadStates->kv*ev[0]+ QuadStates->aref[0]*QuadStates->Quadmass;
     fd[1]=-QuadStates->kx*ex[1]-QuadStates->kv*ev[1]+ QuadStates->aref[1]*QuadStates->Quadmass;
     fd[2]=-QuadStates->kx*ex[2]-QuadStates->kv*ev[2]+ QuadStates->mg+ QuadStates->aref[2]*QuadStates->Quadmass;


     UnitVec(b3d,fd);
     UnitVec(b2d,CrossProd(b2d,b3d,QuadStates->b1ref));
     CrossProd(nb1d,b2d,b3d);

     R[0][0]=nb1d[0];R[1][0]=nb1d[1];R[2][0]=nb1d[2];
     R[0][1]=b2d[0];R[1][1]=b2d[1];R[2][1]=b2d[2];
     R[0][2]=b3d[0];R[1][2]=b3d[1];R[2][2]=b3d[2];

     //generate the desired quaternion qd

     Rot2Quat(QuadStates->qdes,R);
     //cout<<"ev = "<<ev[0]<<" "<<ev[1]<<" "<<ev[2]<<" "<<endl;
     //cout<<"qdes = [ "<<QuadStates->qdes[0]<<" "<<QuadStates->qdes[1]<<" "<<QuadStates->qdes[2]<<" "<<QuadStates->qdes[3]<<" ] "<<endl;
     Quat2Euler(Eulref ,QuadStates->qdes);//z-x-y
     //cout<<"Eulref = "<<Eulref[0]*180/3.14<<" "<<Eulref[1]*180/3.14<<" "<<Eulref[2]*180/3.14<<endl;
     // required thrust

     //QuadStates->qdes[0]=1;
     //QuadStates->qdes[1]=0;
     //QuadStates->qdes[2]=0;
     //QuadStates->qdes[3]=0;

     quat_inv(qinv,QuadStates->qdes);
     quat_prod(eq,qinv,QuadStates->q);

     QuadStates->Fd=NormVec(fd);
     QuadStates->Fd=QuadStates->Fd*(2*pow(eq[0],2)+2*pow(eq[3],2)-1);

     if(abs(QuadStates->Fd)>QuadStates->Fsafety)
    	 QuadStates->Fd=QuadStates->Fsafety;


	//compute the required moments
		double A[4]; //This is a diagonal hurwitz matrix
  
		
		A[0]=2;
		A[1]=2;
		A[2]=2;
		A[3]=1;
		
		A[0]=A[0]*(1-abs(eq[0]));
		A[1]=A[1]*(0-eq[1]);
		A[2]=A[2]*(0-eq[2]);
		A[3]=A[3]*(0-eq[3]);
		
		quat_inv(qinv,eq);
		quat_prod(wref,qinv,A);
		wref[0]=2*wref[0];
		wref[1]=2*wref[1];
		wref[2]=2*wref[2];
		wref[3]=2*wref[3];
		QuadStates->wref[0]=wref[1];
		QuadStates->wref[1]=wref[2];
		QuadStates->wref[2]=wref[3];
		
		//cout<<"wref + w = [ "<<wref[1]<<" "<<wref[2]<<" "<<wref[3]<<" "<<QuadStates->w[0]<<" "<<QuadStates->w[1]<<" "<<QuadStates->w[2]<<" ] "<<endl;
		


}


/*
Raffael paper controller
Angular rate shaping by first order system in quaternions
*/
void Angrate_Tracking(QuadStateVariable *QuadStates){
	    QuadStates->M[0]=QuadStates->kw*(limitto(QuadStates->wref[0],-2,2)-QuadStates->w[0]);
     	QuadStates->M[1]=QuadStates->kw*(limitto(QuadStates->wref[1],-2,2)-QuadStates->w[1]);
     	QuadStates->M[2]=QuadStates->kw*(limitto(QuadStates->wref[2],-2,2)-QuadStates->w[2]);



     	QuadStates->u[0]=QuadStates->Fd/4-QuadStates->M[1]/(2*QuadStates->MomArm)+QuadStates->M[2]/(4*QuadStates->ctf);
     	QuadStates->u[1]=QuadStates->Fd/4+QuadStates->M[0]/(2*QuadStates->MomArm)-QuadStates->M[2]/(4*QuadStates->ctf);
     	QuadStates->u[2]=QuadStates->Fd/4+QuadStates->M[1]/(2*QuadStates->MomArm)+QuadStates->M[2]/(4*QuadStates->ctf);
     	QuadStates->u[3]=QuadStates->Fd/4-QuadStates->M[0]/(2*QuadStates->MomArm)-QuadStates->M[2]/(4*QuadStates->ctf);

}


#endif
