/*
 * DataManger.h
 *
 *  Created on: Jun 26, 2014
 *      Author: Valentin
 */

#ifndef DM_H_
#define DM_H_

struct QuadStatemanager{
	double a_meas[3],a_measured[3],a_imu[3],w_meas[3],Dimu[7];
	double Xatt_prev[6],Xatt_est[6],Xatt[6];
	double Xtran_prev[9],Xtran_est[9],Xtran[9];
//	double T1,T2;
	double t1,t2;


	void initialize(){
		t1=0;
		t2=0;
//		T1=0;
//		T2=0;
		a_meas[0]=0;
		a_meas[1]=0;
		a_meas[2]=0;
		a_measured[0]=0;
		a_measured[1]=0;
		a_measured[2]=0;
		a_imu[0]=0;
		a_imu[1]=0;
		a_imu[2]=0;
		w_meas[0]=0;
		w_meas[1]=0;
		w_meas[2]=0;
		Dimu[0]=0;
		Dimu[1]=0;
		Dimu[2]=0;
		Dimu[3]=0;
		Dimu[4]=0;
		Dimu[5]=0;
		Dimu[6]=0;

			Xatt_prev[0]=0;
			Xatt_prev[1]=0;
			Xatt_prev[2]=0;
			Xatt_prev[3]=0;
			Xatt_prev[4]=0;
			Xatt_prev[5]=0;
			Xatt_est[0]=0;
			Xatt_est[1]=0;
			Xatt_est[2]=0;
			Xatt_est[3]=0;
			Xatt_est[4]=0;
			Xatt_est[5]=0;
			Xatt[0]=0;
			Xatt[1]=0;
			Xatt[2]=0;
			Xatt[3]=0;
			Xatt[4]=0;
			Xatt[5]=0;

		Xtran_prev[0]=0;
		Xtran_prev[1]=0;
		Xtran_prev[2]=0;
		Xtran_prev[3]=0;
		Xtran_prev[4]=0;
		Xtran_prev[5]=0;
		Xtran_prev[6]=0;
		Xtran_prev[7]=0;
		Xtran_prev[8]=0;
		Xtran_est[0]=0;
		Xtran_est[1]=0;
		Xtran_est[2]=0;
		Xtran_est[3]=0;
		Xtran_est[4]=0;
		Xtran_est[5]=0;
		Xtran_est[6]=0;
		Xtran_est[7]=0;
		Xtran_est[8]=0;
		Xtran[0]=0;
		Xtran[1]=0;
		Xtran[2]=0;
		Xtran[3]=0;
		Xtran[4]=0;
		Xtran[5]=0;
		Xtran[6]=0;
		Xtran[7]=0;
		Xtran[8]=0;
	}

};

#endif
