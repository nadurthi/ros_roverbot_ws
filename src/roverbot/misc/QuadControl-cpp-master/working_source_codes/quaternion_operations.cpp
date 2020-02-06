#include "quaternion_operations.h"
#include <cmath>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
using namespace std;


double * quat_prod(double * q,double * q1,double * q2){
	//double q[4];
	q[0]=(q1[0])*q2[0]-(q1[1]*q2[1]+q1[2]*q2[2]+q1[3]*q2[3]);
	q[1]=q1[0]*q2[1]+q2[0]*q1[1]+q1[2]*q2[3]-q1[3]*q2[2];
	q[2]=q1[0]*q2[2]+q2[0]*q1[2]+q1[3]*q2[1]-q1[1]*q2[3];
	q[3]=q1[0]*q2[3]+q2[0]*q1[3]+q1[1]*q2[2]-q1[2]*q2[1];
	return q;
}
double * quat_inv(double *qinv,double * q){
	qinv[0]=q[0];
	qinv[1]=-q[1];
	qinv[2]=-q[2];
	qinv[3]=-q[3];
	return qinv;
}
double * quat_rot(double *rotr,double * q,double * r){
	// r is 3x1 vector
	double rr[4],qr[4],qq[4],qinv[4];
	rr[0]=0;
	rr[1]=r[0];
	rr[2]=r[1];
	rr[3]=r[2];
	quat_prod(qr,quat_prod(qq,q,rr),quat_inv(qinv,q));
	rotr[0]=qr[1];
	rotr[1]=qr[2];
	rotr[2]=qr[3];
	return rotr;
}
double * CrossProd(double * w,double * u,double * v){
    w[0]=(double)(u[1]*v[2]-u[2]*v[1]);
    w[1]=(double)(u[2]*v[0]-u[0]*v[2]);
    w[2]=(double)(u[0]*v[1]-u[1]*v[0]);
	return w;
}
double  DotProd(double * u,double * v){
	return (u[0]*v[0]+u[1]*v[1]+u[2]*v[2]);
}
double * UnitVec(double *vu,double *v){
    double c=sqrt(pow(v[0],2)+pow(v[1],2)+pow(v[2],2));
    vu[0]=(double)v[0]/c;
    vu[1]=(double)v[1]/c;
    vu[2]=(double)v[2]/c;
	return vu;
}
double NormVec(double *v){
	return (double)sqrt(pow(v[0],2)+pow(v[1],2)+pow(v[2],2));
}
void Rot2Quat(double * q,double R[][3]){
    double qs[4];
    double c=(double)1/4;
    qs[0]=(double)c*(R[0][0]+R[1][1]+R[2][2]+1);
    qs[1]=(double)c*(R[0][0]-R[1][1]-R[2][2]+1);
    qs[2]=(double)c*(-R[0][0]+R[1][1]-R[2][2]+1);
    qs[3]=(double)c*(-R[0][0]-R[1][1]+R[2][2]+1);
    

	if(abs(qs[0])<0.000000000000001)
	qs[0]=0;
	
	if(abs(qs[1])<0.000000000000001)
	qs[1]=0;
	
	if(abs(qs[2])<0.000000000000001)
	qs[2]=0;
	
	if(abs(qs[3])<0.000000000000001)
	qs[3]=0;
	


	q[0]=sqrt(qs[0]);
    if((R[2][1]-R[1][2])>=0)
    	q[1]=sqrt(qs[1]);
    else
    	q[1]=-sqrt(qs[1]);

    if((R[0][2]-R[2][0])>=0)
        	q[2]=sqrt(qs[2]);
        else
        	q[2]=-sqrt(qs[2]);

    if((R[1][0]-R[0][1])>=0)
        	q[3]=sqrt(qs[3]);
        else
        	q[3]=-sqrt(qs[3]);

    //cout<<"q= "<<q[0]<<" "<<q[1]<<" "<<q[2]<<" "<<q[3]<<" "<< sqrt(pow(q[0],2)+pow(q[1],2)+pow(q[2],2)+pow(q[3],2))<<endl;
}
void Quat2Rot(double R[][3],double * q){
    R[0][0]=1-2*(pow(q[2],2)+pow(q[3],2));
    R[0][1]=2*(q[1]*q[2]-q[3]*q[0]);
    R[0][2]=2*(q[1]*q[3]+q[2]*q[0]);

    R[1][0]=2*(q[1]*q[2]+q[3]*q[0]);
    R[1][1]=1-2*(pow(q[1],2)+pow(q[3],2));
    R[1][2]=2*(q[2]*q[3]-q[1]*q[0]);

    R[2][0]=2*(q[1]*q[3]-q[2]*q[0]);
    R[2][1]=2*(q[2]*q[3]+q[1]*q[0]);
    R[2][2]=1-2*(pow(q[1],2)+pow(q[2],2));


}
void Rot2Euler(double  *Eul ,double R[][3])
{
Eul[0]=atan2(R[1][0],R[0][0]);
Eul[1]=atan2(-R[2][0],sqrt(pow(R[2][1],2)+pow(R[2][2],2)));
Eul[2]=atan2(R[2][1],R[2][2]);
}
void Quat2Euler(double  *Eul ,double * q){
	double R[3][3];
	Quat2Rot(R,q);
	Rot2Euler(Eul ,R);

}
