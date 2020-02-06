/*
 * initialize_parameters.h
 * a single file to keep track of all the parameters used by various algorithms
 */


/*
 * Front clockwise
 * Back clockwise
 */
#ifndef STATE_INIT_H_
#define STATE_INIT_H_

#include"State_data_manager.h"


void Initialize_all_parameters(QuadStateVariable *QuadState){

	QuadState->t=0;

	QuadState->EmergencyShutoff.SetTsec(0.1);  //100ms shutoff timer for control



	QuadState->Fdnorm_limit=20;







	//##########################   Quad props #########################################

	QuadState->mg=1.2*9.8;
	QuadState->Quadmass=1.2;
	QuadState->MomArm=0.3;
	QuadState->ctf=0.01;
	QuadState->I[0]=1;
	QuadState->I[1]=1;
	QuadState->I[2]=1;

	QuadState->ulim=20;  //in Newtons

	//##########################   GAINS #########################################
	QuadState->kx=4;     //kx=3 and kv=1.5
	QuadState->kv=2;

	QuadState->kq=0.1;
	QuadState->kw=0.3; //kw = 0.3

	QuadState->Fsafety=25.0;

    //##########################  MOTOR PWM min and max  #########################
	QuadState->MotorFmax=30;
	QuadState->MotorFmin=0;

    //##########################  Reference Parameters   ####################


	QuadState->aref[0]=0;
	QuadState->aref[1]=0;
	QuadState->aref[2]=0;

	QuadState->vref[0]=0;
	QuadState->vref[1]=0;
	QuadState->vref[2]=0;

	QuadState->xref[0]=0;
	QuadState->xref[1]=0;
	QuadState->xref[2]=0;

		QuadState->qref[0]=1; //0.994328 -0.00259497 -0.0592353 -0.0882999
		QuadState->qref[1]=0;//0.92592 0.21457 0.30043 -0.07979
		QuadState->qref[2]=0;  //0.99633 0.01268 -0.04374 -0.07251
		QuadState->qref[3]=0;

		QuadState->wref[0]=0;
		QuadState->wref[1]=0;
		QuadState->wref[2]=0;

		QuadState->b1ref[0]=1;
		QuadState->b1ref[1]=0;
		QuadState->b1ref[2]=0;
   //####################       Sensor Parameters   #####################################3

		QuadState->rmis[0]=-0.004;
		QuadState->rmis[1]=0.001;
		QuadState->rmis[2]=-0.005;

		QuadState->qmis[0]=1;
		QuadState->qmis[1]=0;
		QuadState->qmis[2]=0;
		QuadState->qmis[3]=0;

		QuadState->ba[0]=0;
		QuadState->ba[1]=0;
		QuadState->ba[2]=0;

		QuadState->bg[0]=0;
		QuadState->bg[1]=0;
		QuadState->bg[2]=0;

		QuadState->ba_lim=2;  // m/s^2

			QuadState->bg_lim=2;  // rad/s

			QuadState->rmis_lim=0.05; // m

//########################Initial conditions  #############################################3
	QuadState->q[0]=1;
	QuadState->q[1]=0;
	QuadState->q[2]=0;
	QuadState->q[3]=0;

	QuadState->x[0]=0;
	QuadState->x[1]=0;
	QuadState->x[2]=0;

	QuadState->v[0]=0;
	QuadState->v[1]=0;
	QuadState->v[2]=0;

	QuadState->a[0]=0;
	QuadState->a[1]=0;
	QuadState->a[2]=0;

	QuadState->wdot[0]=0;
	QuadState->wdot[1]=0;
	QuadState->wdot[2]=0;

	QuadState->w_meas[0]=0;
	QuadState->w_meas[1]=0;
	QuadState->w_meas[2]=0;

	QuadState->a_meas[0]=0;
	QuadState->a_meas[1]=0;
	QuadState->a_meas[2]=0;
	QuadState->ainertmeasprev[0]=0;
	QuadState->ainertmeasprev[1]=0;
	QuadState->ainertmeasprev[2]=0;

	QuadState->x_meas[0]=0;
	QuadState->x_meas[1]=0;
	QuadState->x_meas[2]=0;

	QuadState->q_meas[0]=1;
	QuadState->q_meas[1]=0;
	QuadState->q_meas[2]=0;
	QuadState->q_meas[3]=0;


	QuadState->Fd=0;
	QuadState->Fd_prev=0;

	QuadState->u[0]=0;
	QuadState->u[1]=0;
	QuadState->u[2]=0;
	QuadState->u[3]=0;

	QuadState->M[0]=0.1;
	QuadState->M[1]=0.1;
	QuadState->M[2]=0.1;

}

void initialize_MAXDEVS(QuadStateVariable *QuadState){
	// Corrector filter maxstdevs
	QuadState->Data_x_meas.maxstdev[0]=1;
	QuadState->Data_x_meas.maxstdev[1]=1;
	QuadState->Data_x_meas.maxstdev[2]=1;

	QuadState->Data_q_meas.maxstdev[0]=1;
	QuadState->Data_q_meas.maxstdev[1]=1;
	QuadState->Data_q_meas.maxstdev[2]=1;
	QuadState->Data_q_meas.maxstdev[3]=1;

	QuadState->Data_a_meas.maxstdev[0]=1;
	QuadState->Data_a_meas.maxstdev[1]=1;
	QuadState->Data_a_meas.maxstdev[2]=1;

	QuadState->Data_w_meas.maxstdev[0]=1;
	QuadState->Data_w_meas.maxstdev[1]=1;
	QuadState->Data_w_meas.maxstdev[2]=1;

}




#endif
