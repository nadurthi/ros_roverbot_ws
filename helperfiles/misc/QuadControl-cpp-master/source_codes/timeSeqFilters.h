// header for time seires filtering/averaging
// also ekf and ukf will be implemented
#ifndef TIMESEQFILTERS_H_
#define TIMESEQFILTERS_H_

#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#define histlength 5

// time series Q last element is the latest element
// REMEMBER X,Y,Z,Q0,Q1,Q2,Q3 ARE IN ORDER
class XYZ_QUAT_Queue {
private:
	double Tseries[histlength];
	double Dseries[histlength][14];
	//double DDseries[histlength][7];
	//int meth;
	//int window;
	double T0;
	//int Nt;
	//double *PolyCoeff;
public:
	void UpdateXYZQUAT(double * D, double T);

	void GetRawData(double *X){
		//double X[14];
        X[0]=Dseries[histlength-1][0];
        X[1]=Dseries[histlength-1][1];
        X[2]=Dseries[histlength-1][2];
        X[3]=Dseries[histlength-1][3];
        X[4]=Dseries[histlength-1][4];
        X[5]=Dseries[histlength-1][5];
        X[6]=Dseries[histlength-1][6];
        X[7]=Dseries[histlength-1][7];
        X[8]=Dseries[histlength-1][8];
        X[9]=Dseries[histlength-1][9];
        X[10]=Dseries[histlength-1][10];
        X[11]=Dseries[histlength-1][11];
        X[12]=Dseries[histlength-1][12];
        X[13]=Dseries[histlength-1][13];


        //return X;
	}

	void GetFilterXYZQUAT(double *X);

	void InitDataQueue(){
		for(int i=0;i<histlength;i++){
			for(int j=0;j<14;j++){
				Dseries[i][j]=0;
			}
			Tseries[i]=0;
		}
	}
    void InitT0(double T){
    	T0=T;
    }
};



#endif
