#ifndef QUATERNIONS_OPER_H_
#define QUATERNIONS_OPER_H_

#include <stdlib.h>
#include <unistd.h>


double * quat_prod(double * q, double * q1,double * q2);
double * quat_inv(double *qinv,double * q);
double * quat_rot(double *rotr,double * q,double * r);

double * CrossProd(double * w,double * u,double * v);
double  DotProd(double * u,double * v);
double * UnitVec(double *vu,double *v);
double NormVec(double *v);
void Rot2Quat(double * q,double R[][3]);
void Quat2Rot(double  R[][3] ,double * q);
void Rot2Euler(double  *Eul ,double R[][3]);
void Quat2Euler(double  *Eul ,double * q);

#endif
