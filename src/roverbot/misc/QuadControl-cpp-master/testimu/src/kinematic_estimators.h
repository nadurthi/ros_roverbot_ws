







#ifndef _KINEMATIC_ESTIMATORS_H_
#define _KINEMATIC_ESTIMATORS_H_



#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <sys/types.h>
#include <fstream>
#include <cmath>
#include "quaternion_operations.h"
#include <algorithm>
#include "/home/nagnanamus/Dropbox/BBB/codes/quad_control/working_source_codes/Eigen/Dense"
#include "State_data_manager.h"
#include "MPU6050.h"
#include "SerialcommGroundOnboard.h"

using namespace std;

using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::VectorXd;

typedef Eigen::Matrix<double, 4, 3> Matrix43d;
typedef Eigen::Matrix<double, 3, 4> Matrix34d;
typedef Eigen::Matrix<double, 7, 7> Matrix7d;
typedef Eigen::Matrix<double, 10, 10> Matrix10d;
typedef Eigen::Matrix<double, 9, 9> Matrix9d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 7, 6> Matrix76d;
typedef Eigen::Matrix<double, 6, 7> Matrix67d;
typedef Eigen::Matrix<double, 6, 9> Matrix69d;
typedef Eigen::Matrix<double, 9, 6> Matrix96d;
typedef Eigen::Matrix<double, 7, 10> Matrix710d;
typedef Eigen::Matrix<double, 10, 7> Matrix107d;
typedef Eigen::Matrix<double, 10, 1> Vector10d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;

#include "SimpleTimer.h"



void AccTransform2imu(double *aimu,double * a,double * r,double * q,double * qmis,double * w,double * wdot,double * ba);


/*
 * Run this function on the Onboard controller to setup imu
 */
void Initialize_IMU_onboard(QuadStateVariable * QuadState,MPU6050 *AccelGyroSensor,double *qstat);


/*
 * To be completed:
 * Set:   qstat
 *        maxstdev
 *        stdev
 */
void Initialize_QX_onboard(QuadStateVariable * QuadState,Serial_comm_Ground_Onboard *GroundComm,double * qstat);


/*To be completed:
 *
 * Run this function on the ground station to initialize and stabilize the vicon
 * parameters that are update: bg and ba;
 */
void Initialize_IMU_ground(QuadStateVariable * QuadState ,SimpleQueue *QQ);


/*To be completed:
 *
 * Run this function on Ground station to initialize the IMU variables from onboard
 * parameters that are update:
 */
void Initialize_QX_ground(QuadStateVariable * QuadState,SimpleQueue *QQ );

void Initialize_ba_bg_est(QuadStateVariable * QuadState ,SimpleQueue *QQvic,SimpleQueue *QQimu,SimpleQueue *BA,SimpleQueue *BG);

void Initialize_ba_bg_est_simplified(QuadStateVariable * QuadState ,SimpleQueue *BA,SimpleQueue *BG);

void AccInert2Imu(double *aimu,double * q,double * ainert,double * ba);// convert QuadState.a to aimu (Inertial to body frame)
void AccImu2Inert(double *ainert,double * q,double * a_meas,double * ba);// convert QuadState.a_meas to ainert (imu meas to inert frame)

/*-------------------------------------------------------------------------------------------------------------------------------------------
 * EKF  Ground estimator (1) : ALL estimation is done ground+onboard 2 implementation
 *                          EKF filter
 *                          2 Parallel estimators: Postion + Attitude
 *                          Ground Stattion gets only (x,q)
 *                          100hz uses all measurements at once
 *                          States: [q.w.alpha,x,v,a] but propagated indivudally for efficiency.
 *                          Meas:  [qvic,wimu] for att and [a,x] for translation
 */
//position estimation (estimate x,v,a): assume you know the current q_est, w_est, wdot_est
class EKF_Estimator1{
public:
	SimpleTimer TicToc;

	    Matrix10d Fatt,Patt;
		Vector10d Qatt,Inoatt;
		Vector7d Ratt,yatt,yatt_est;
		Vector3d yatt2,yatt_est2;

		Matrix710d Hatt;
		Matrix107d Katt;
	    Matrix7d  Satt;

	    Eigen::Matrix<double, 3, 10> Hatt2;
	    Eigen::Matrix<double, 10, 3> Katt2;
	    Matrix3d Satt2;

		Matrix9d Ftran,Ptran;
		Matrix6d Stran;
		Matrix69d Htran;
		Matrix96d Ktran;

	    Eigen::Matrix<double, 3, 9> Htran2;
	    Eigen::Matrix<double, 9, 3> Ktran2;
	    Matrix3d Stran2;

		Vector9d Qtran,Inotran;
		Vector6d Rtran,ytran,ytran_est;
		Vector3d ytran2,ytran_est2;

	double dt;

void InitializeFilter();//call this function to initialize all the matrices

void PropagateUpdateMeanCov(QuadStateVariable *QuadState); //propagate and update

void PrintFiltervals(); //print the filter values

};


/*-------------------------------------------------------------------------------------------------------------------------------------------
 * EKF  Onboard estimator (1) : ALL estimation is done onboard
 *                          EKF filter    #########  FASTER USING MATRIX SIMPLIFICATIONS ############
 *                          2 Parallel estimators: Postion + Attitude
 *                          Ground Station gets only (x,q)
 *                          100hz uses all measurements at once
 *                          States: [q,w,alpha,x,v,a] but propagated individually for efficiency.
 *                          Meas:  [qvic,wimu] for att and [xvic,aimu] for translation
 */
//position estimation (estimate x,v,a): assume you know the current q_est, w_est, wdot_est
class EKF_Estimator_onboard1{
public:
	SimpleTimer TicToc;
    Matrix10d Fatt,Patt;
		Vector10d Qatt,Inoatt;
		Vector7d Ratt,yatt,yatt_est;
		Vector3d yatt2,yatt_est2;

		Matrix710d Hatt;
		Matrix107d Katt;
	    Matrix7d  Satt;

	    Eigen::Matrix<double, 3, 10> Hatt2;
	    Eigen::Matrix<double, 10, 3> Katt2;
	    Matrix3d Satt2;

		Matrix9d Ftran,Ptran;
		Matrix6d Stran;
		Matrix69d Htran;
		Matrix96d Ktran;

	    Eigen::Matrix<double, 3, 9> Htran2;
	    Eigen::Matrix<double, 9, 3> Ktran2;
	    Matrix3d Stran2;

		Vector9d Qtran,Inotran;
		Vector6d Rtran,ytran,ytran_est;
		Vector3d ytran2,ytran_est2;

	double dt;

void InitializeFilter();//call this function to initialize all the matrices

void PropagateUpdateMeanCov(QuadStateVariable *QuadState); //propagate and update

void PrintFiltervals(); //print the filter values

};

/*-------------------------------------------------------------------------------------------------------------------------------------------
 * Simple Quick  Onboard estimator (1) : ALL estimation is done onboard
 *
 */

class Simple_Estimator_onboard{
public:

	double Ratt[2],Rtran[2];

	double dt;

void InitializeFilter();//call this function to initialize all the matrices

void PropagateUpdateMeanCov(QuadStateVariable *QuadState); //propagate and update


void AngRateInert2Imu(double *wimu,QuadStateVariable *QuadState);// convert QuadState.w to wimu (Inertial to body frame)
void AngRateImu2Inert(double *winert,QuadStateVariable *QuadState);// convert QuadState.w_meas to winert (imu meas to inert frame)

};

/*-------------------------------------------------------------------------------------------------------------------------------------------
 *  UKF onboard estimator (2) : ALL estimation is done ground
 *                          UKF filter
 *                          2 Parallel estimators: Postion + Attitude
 *                          Ground Stattion gets only (x,q)
 *                          100hz uses all measurements at once
 */

//position estimation (estimate x,v,a): assume you know the current q_est, w_est, wdot_est
class UKF_Estimator1{
public:

	SimpleTimer TicToc;
	MatrixXd F;

void InitializeFilter();
double * PropagateMeanCov_xva(QuadStateVariable * QuadState);
double * PropagateMeanCov_qwalpha(QuadStateVariable *QuadState);

double * ExtKalmanGain_xva();
double * ExtKalmanGain_qwalpha();

double * UpdateMeanCov_xva();
double * UpdateMeanCov_qwalpha();
};




#endif
