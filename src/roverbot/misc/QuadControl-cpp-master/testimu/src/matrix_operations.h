
#ifndef _MATRIX_OPERATIONS_H_
#define _MATRIX_OPERATIONS_H_

#include<cmath>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

class SimpleMat {

public:
	int rows,columns;
	double  **M;
	//dynamically create a Queue and initialize with zeros
		void CreateMat(int rows,int cols){
			this->rows=rows;
			columns=cols;
			// allocation


				M = new double*[rows];
				for(int i = 0; i < rows; i++)
					M[i] = new double[cols];

				for(int r=0;r<rows;r++){
					for(int c=0;c<cols;c++){
						M[r][c]=0;

					}
				}
		}

		void Display(){
			cout<<endl;
			for(int r=0;r<rows;r++){
								for(int c=0;c<columns;c++){
									cout<<M[r][c]<<" ";

								}
								cout<<endl;
							}
		}



};


SimpleMat * Copy_Submat(SimpleMat *P,SimpleMat *P1,int pr_start,int pc_start);

SimpleMat * Add_mat(SimpleMat *P,SimpleMat *P1,SimpleMat *P2);

SimpleMat * Multiply_mat(SimpleMat *P,SimpleMat *P1,SimpleMat *P2);

double Determinant_mat(SimpleMat *P);

SimpleMat * Inverse_mat(SimpleMat *IP,SimpleMat *P);

/*
double ** Transpose_mat(double **PT,double **P){

}

double ** Multiply_mat(double **PM,double **P1,double **P2){

}
double ** Add_mat(double **PM,double **P1,double **P2){

}

double ** GetSubBlock_mat(double **Psub,double **P){

}

double ** SetSubBlock_mat(double **P,double **Psub){

}

double ** Add2SubBlock_mat(double **P,double **Psub){

}


*/

#endif
