/*
 * EKF.h
 *
 *  Created on: Jun 26, 2014
 *      Author: Valentin
 */

#ifndef EKF_H_
#define EKF_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <sys/types.h>
#include <fstream>
#include <cmath>
#include <algorithm>
#include "DataManager.h"
#include "quaternion_operations.h"
#include "/home/nagnanamus/Dropbox/BBB/codes/quad_control/working_source_codes/Eigen/Dense"

using namespace std;

using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::VectorXd;

typedef Eigen::Matrix<double, 9, 9> Matrix9d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;
typedef Eigen::Matrix<double, 3, 9> Matrix39d;
typedef Eigen::Matrix<double, 9, 3> Matrix93d;

typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;

#include "SimpleTimer.h"

class EKF {
	public:

		SimpleTimer TicToc;

	    Matrix6d Fatt,Patt_prev,Patt_est,Patt,Hatt,Katt,Satt;
		Vector6d Zkatt,Xatt_prev,Xatt_est,Xatt,Qatt,Ratt,yatt,hatt,Kattyatt;

		Matrix9d Ftran,Ptran_prev,Ptran_est,Ptran;
		Matrix3d Stran;
		Matrix39d Htran;
		Matrix93d Ktran;

		Vector9d Qtran,Xtran_prev,Xtran_est,Xtran,Ktranytran;
		Vector3d Rtran,ytran,Zktran,htran;

		Vector3d a_measured,a_imu;
		Vector6d Dimu;
		Matrix3d DCM;


	void InitializeFilter();//call this function to initialize all the matrices

	void PropagateUpdateMeanCov(QuadStatemanager * DM); //propagate and update

	void PrintFiltervals(); //print the filter values

};


#endif /* EKF_H_ */
