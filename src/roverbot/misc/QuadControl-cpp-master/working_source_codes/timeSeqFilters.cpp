// src for time seires filtering/averaging
// also ekf and ukf will be implemented

#include "timeSeqFilters.h"
#include<math.h>


void XYZ_QUAT_Queue::UpdateXYZQUAT(double * D, double T){
	for(int i=0;i<histlength-1;i++){
	             Dseries[i][0]=Dseries[i+1][0];
	             Dseries[i][1]=Dseries[i+1][1];
	             Dseries[i][2]=Dseries[i+1][2];
	             Dseries[i][3]=Dseries[i+1][3];
	             Dseries[i][4]=Dseries[i+1][4];
	             Dseries[i][5]=Dseries[i+1][5];
	             Dseries[i][6]=Dseries[i+1][6];
	             Dseries[i][7]=Dseries[i+1][7];
	             Dseries[i][8]=Dseries[i+1][8];
	             Dseries[i][9]=Dseries[i+1][9];
	             Dseries[i][10]=Dseries[i+1][10];
	             Dseries[i][11]=Dseries[i+1][11];
	             Dseries[i][12]=Dseries[i+1][12];
	             Dseries[i][13]=Dseries[i+1][13];
	             Tseries[i]=Tseries[i+1];
			}
	             Dseries[histlength-1][0]=D[0];
	             Dseries[histlength-1][1]=D[1];
	             Dseries[histlength-1][2]=D[2];
	             Dseries[histlength-1][3]=D[3];
	             Dseries[histlength-1][4]=D[4];
	             Dseries[histlength-1][5]=D[5];
	             Dseries[histlength-1][6]=D[6];

	             Tseries[histlength-1]=T-T0;

	             double d=(double)(Tseries[histlength-1]-Tseries[histlength-2]);
	             Dseries[histlength-1][7]=(double)(D[0]-Dseries[histlength-2][0])/d;
	             Dseries[histlength-1][8]=(double)(D[1]-Dseries[histlength-2][1])/d;
	             Dseries[histlength-1][9]=(double)(D[2]-Dseries[histlength-2][2])/d;
	             Dseries[histlength-1][10]=(double)(D[3]-Dseries[histlength-2][3])/d;
	             Dseries[histlength-1][11]=(double)(D[4]-Dseries[histlength-2][4])/d;
	             Dseries[histlength-1][12]=(double)(D[5]-Dseries[histlength-2][5])/d;
	             Dseries[histlength-1][13]=(double)(D[6]-Dseries[histlength-2][6])/d;



}

void XYZ_QUAT_Queue::GetFilterXYZQUAT(double *X){
// a simple 5 point moving avg filter is implemented
	int Nt=5;
	double c=(double)1/Nt;
	//double X[14];
	X[0]=0;X[1]=0;X[2]=0;X[3]=0;X[4]=0;X[5]=0;X[6]=0;
	X[7]=0;X[8]=0;X[9]=0;X[10]=0;X[11]=0;X[12]=0;X[13]=0;
	for(int i=1;i<=Nt;i++){
		X[0]=X[0]+Dseries[histlength-i][0];
		X[1]=X[1]+Dseries[histlength-i][1];
		X[2]=X[2]+Dseries[histlength-i][2];
		X[3]=X[3]+Dseries[histlength-i][3];
		X[4]=X[4]+Dseries[histlength-i][4];
		X[5]=X[5]+Dseries[histlength-i][5];
		X[6]=X[6]+Dseries[histlength-i][6];
		X[7]=X[7]+Dseries[histlength-i][7];
		X[8]=X[8]+Dseries[histlength-i][8];
		X[9]=X[9]+Dseries[histlength-i][9];
		X[10]=X[10]+Dseries[histlength-i][10];
		X[11]=X[11]+Dseries[histlength-i][11];
		X[12]=X[12]+Dseries[histlength-i][12];
		X[13]=X[13]+Dseries[histlength-i][13];
	}
	//Averaging out
	X[0]=X[0]*c;
	X[1]=X[1]*c;
	X[2]=X[2]*c;
	X[3]=X[3]*c;
	X[4]=X[4]*c;
	X[5]=X[5]*c;
	X[6]=X[6]*c;
	X[7]=X[7]*c;
	X[8]=X[8]*c;
	X[9]=X[9]*c;
	X[10]=X[10]*c;
	X[11]=X[11]*c;
	X[12]=X[12]*c;
	X[13]=X[13]*c;

	//Normalizing the averaged quaternion
	c=(double)1/sqrt(pow(X[3],2)+pow(X[4],2)+pow(X[5],2)+pow(X[6],2));
	X[3]=X[3]*c;
	X[4]=X[4]*c;
	X[5]=X[5]*c;
	X[6]=X[6]*c;
	//return X;
}

/*	void DataQueue::FilterQ(){ //always call filter after u have updated data

		if(meth==2){  //quadratic filter using least squares for last 5 values
			double coeff[3];

			double t1=Tseries[0];
			double t2=Tseries[1];
			double t3=Tseries[2];
			double t4=Tseries[3];
			double t5=Tseries[4];

			double a;
			double b;
			double c;
			double d;
			double e;

			a=pow(t1,3)+pow(t2,3)+pow(t3,3)+pow(t4,3)+pow(t5,3);
            b=pow(t1,2)+pow(t2,2)+pow(t3,2)+pow(t4,2)+pow(t5,2);
            c=pow(t1,4)+pow(t2,4)+pow(t3,4)+pow(t4,4)+pow(t5,4);
            d=t1+t2+t3+t4+t5;
            e=5*pow(a,2)+pow(b,3)-5*b*c-2*a*b*d+c*pow(d,2);


			coeff[2]= (double)((pow(a,2) - b*c + c*d*t1 + pow(b,2)*pow(t1,2) - a*t1*(b + d*t1))*Dseries[0]+(pow(a,2) - b*c + c*d*t2 + pow(b,2)*pow(t2,2) - a*t2*(b + d*t2))*Dseries[1]+(pow(a,2) - b*c + c*d*t3 + pow(b,2)*pow(t3,2) - a*t3*(b + d*t3))*Dseries[2]+(pow(a,2) - b*c + c*d*t4 + pow(b,2)*pow(t4,2) - a*t4*(b + d*t4))*Dseries[3]+(pow(a,2) - b*c + c*d*t5 + pow(b,2)*pow(t5,2) - a*t5*(b + d*t5))*Dseries[4])/e;
			coeff[1]= (double)((c*(d - 5*t1) + b*t1*(b - d*t1) - a*(b - 5*pow(t1,2)))*Dseries[0]+(c*(d - 5*t2) + b*t2*(b - d*t2) - a*(b - 5*pow(t2,2)))*Dseries[1]+(c*(d - 5*t3) + b*t3*(b - d*t3) - a*(b - 5*pow(t3,2)))*Dseries[2]+(c*(d - 5*t4) + b*t4*(b - d*t4) - a*(b - 5*pow(t4,2)))*Dseries[3]+(c*(d - 5*t5) + b*t5*(b - d*t5) - a*(b - 5*pow(t5,2)))*Dseries[4])/e;
			coeff[0]= (double)((pow(b,2) - a*d + 5*a*t1 + pow(d,2)*pow(t1,2) - b*t1*(d + 5*t1))*Dseries[0]+(pow(b,2) - a*d + 5*a*t2 + pow(d,2)*pow(t2,2) - b*t2*(d + 5*t2))*Dseries[1]+(pow(b,2) - a*d + 5*a*t3 + pow(d,2)*pow(t3,2) - b*t3*(d + 5*t3))*Dseries[2]+(pow(b,2) - a*d + 5*a*t4 + pow(d,2)*pow(t4,2) - b*t4*(d + 5*t4))*Dseries[3]+(pow(b,2) - a*d + 5*a*t5 + pow(d,2)*pow(t5,2) - b*t5*(d + 5*t5))*Dseries[4])/e;

			PolyCoeff=coeff;
		}
		if(meth==3){  //cubic filter
			double coeff[4];

			PolyCoeff=coeff;
		}

	}
	void DataQueue::UpdateDTQ(double D, double T){
		for(int i=0;i<Nt-1;i++){
             Dseries[i]=Dseries[i+1];
             //Tseries[i]=Tseries[i+1];
		}
             Dseries[Nt]=D;
           //  Tseries[Nt]=T-T0;
             //TimeDataQueue::FilterQ();

	}

		double DataQueue::GetFilterdata(){
			double DF=0;
			if(meth==2){  //quadratic filter
				DF=PolyCoeff[0]*(pow(Tseries[Nt],2))+PolyCoeff[1]*(Tseries[Nt])+PolyCoeff[2];

			}
			if(meth==3){  //cubic filter
				DF=PolyCoeff[0]*(pow(Tseries[Nt],3))+PolyCoeff[1]*(pow(Tseries[Nt],2))+PolyCoeff[2]*(Tseries[Nt])+PolyCoeff[3];
			}
			return DF;
		}
		double DataQueue::GetFilterDdata(){
			double DF=0;
						if(meth==2){  //quadratic filter
							DF=2*PolyCoeff[0]*(Tseries[Nt])+PolyCoeff[1];

						}
						if(meth==3){  //cubic filter
							DF=3*PolyCoeff[0]*(pow(Tseries[Nt],2))+2*PolyCoeff[1]*(Tseries[Nt])+PolyCoeff[2];
						}
						return DF;

		}
		double DataQueue::GetFilterDDdata(){
			double DF=0;
						if(meth==2){  //quadratic filter
							DF=PolyCoeff[2];

						}
						if(meth==3){  //cubic filter
							DF=6*PolyCoeff[0]*(Tseries[Nt])+2*PolyCoeff[1];
						}
						return DF;
		}

*/
