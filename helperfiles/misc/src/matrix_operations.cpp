#include "matrix_operations.h"



/*
 * Copy  mat A into submat of P : dimensions of submat have to be given
 */
SimpleMat * Copy_Submat(SimpleMat *P,SimpleMat *A,int pr_start,int pc_start){

	for(int r=0;r<A->rows;r++){
		for(int c=0;c<A->columns;c++){
			P->M[pr_start+r][pc_start+c]=A->M[r][c];
		}
	}


	return P;
}

/*
 * Add 2 simplemats of dimension 9 by 9 and put the result into Simple mat P
 */
SimpleMat * Add_mat(SimpleMat *P,SimpleMat *P1,SimpleMat *P2){

	if((P1->rows == P2->rows) && (P1->columns == P2->columns)){ }
	else{
		cout<<"Incompatible dimensions for addition"<<endl;
		exit(1);
	}

	if(P1->columns==3){
		for(int r=0;r<P1->rows;r++){
			P->M[r][0]=P1->M[r][0]+P2->M[r][0];
			P->M[r][1]=P1->M[r][1]+P2->M[r][1];
			P->M[r][2]=P1->M[r][2]+P2->M[r][2];
		}
		return P;
	}

	if(P1->columns==4){
		for(int r=0;r<P1->rows;r++){
			P->M[r][0]=P1->M[r][0]+P2->M[r][0];
			P->M[r][1]=P1->M[r][1]+P2->M[r][1];
			P->M[r][2]=P1->M[r][2]+P2->M[r][2];
		}
		return P;
	}

if(P1->columns==9){
	for(int r=0;r<P1->rows;r++){
		P->M[r][0]=P1->M[r][0]+P2->M[r][0];
		P->M[r][1]=P1->M[r][1]+P2->M[r][1];
		P->M[r][2]=P1->M[r][2]+P2->M[r][2];
		P->M[r][3]=P1->M[r][3]+P2->M[r][3];
		P->M[r][4]=P1->M[r][4]+P2->M[r][4];
		P->M[r][5]=P1->M[r][5]+P2->M[r][5];
		P->M[r][6]=P1->M[r][6]+P2->M[r][6];
		P->M[r][7]=P1->M[r][7]+P2->M[r][7];
		P->M[r][8]=P1->M[r][8]+P2->M[r][8];
	}
	return P;
}

if(P1->columns==10){
	for(int r=0;r<P1->rows;r++){
		P->M[r][0]=P1->M[r][0]+P2->M[r][0];
		P->M[r][1]=P1->M[r][1]+P2->M[r][1];
		P->M[r][2]=P1->M[r][2]+P2->M[r][2];
		P->M[r][3]=P1->M[r][3]+P2->M[r][3];
		P->M[r][4]=P1->M[r][4]+P2->M[r][4];
		P->M[r][5]=P1->M[r][5]+P2->M[r][5];
		P->M[r][6]=P1->M[r][6]+P2->M[r][6];
		P->M[r][7]=P1->M[r][7]+P2->M[r][7];
		P->M[r][8]=P1->M[r][8]+P2->M[r][8];
		P->M[r][9]=P1->M[r][9]+P2->M[r][9];
	}
	return P;
}
//general case
for(int r=0;r<P1->rows;r++){
	for(int c=0;c<P1->columns;c++){
		P->M[r][c]=P1->M[r][c]+P2->M[r][c];
	}
}

return P;
}

/*
 * Multiply 2 simplemats of dimension 9 by 9 and put the result into Simple mat P
 */
SimpleMat * Multiply_mat(SimpleMat *P,SimpleMat *P1,SimpleMat *P2){

	if(P->rows==P1->rows && P->columns==P2->columns){}
	else{
		cout<<"Incompatible dimensions for Output P multiplication"<<endl;
				exit(1);
	}

	if(P1->columns == P2->rows){ }
	else{
		cout<<"Incompatible dimensions for multiplication"<<endl;
		exit(1);
	}

	if(P1->rows==3 && P1->columns==3 && P2->rows==3 && P2->columns==3){
		P->M[0][0]=P1->M[0][0]*P2->M[0][0]	+	P1->M[0][1]*P2->M[1][0]	+	P1->M[0][2]*P2->M[2][0];
		P->M[0][1]=P1->M[0][0]*P2->M[0][1]	+	P1->M[0][1]*P2->M[1][1]	+	P1->M[0][2]*P2->M[2][1];
		P->M[0][2]=P1->M[0][0]*P2->M[0][2]	+	P1->M[0][1]*P2->M[1][2]	+	P1->M[0][2]*P2->M[2][2];

		P->M[1][0]=P1->M[1][0]*P2->M[0][0]	+	P1->M[1][1]*P2->M[1][0]	+	P1->M[1][2]*P2->M[2][0];
		P->M[1][1]=P1->M[1][0]*P2->M[0][1]	+	P1->M[1][1]*P2->M[1][1]	+	P1->M[1][2]*P2->M[2][1];
		P->M[1][2]=P1->M[1][0]*P2->M[0][2]	+	P1->M[1][1]*P2->M[1][2]	+	P1->M[1][2]*P2->M[2][2];

		P->M[2][0]=P1->M[2][0]*P2->M[0][0]	+	P1->M[2][1]*P2->M[1][0]	+	P1->M[2][2]*P2->M[2][0];
		P->M[2][1]=P1->M[2][0]*P2->M[0][1]	+	P1->M[2][1]*P2->M[1][1]	+	P1->M[2][2]*P2->M[2][1];
		P->M[2][2]=P1->M[2][0]*P2->M[0][2]	+	P1->M[2][1]*P2->M[1][2]	+	P1->M[2][2]*P2->M[2][2];


		return P;
	}

	if(P1->rows==4 && P1->columns==4 && P2->rows==4 && P2->columns==4){
		P->M[0][0]=P1->M[0][0]*P2->M[0][0]	+	P1->M[0][1]*P2->M[1][0]	+	P1->M[0][2]*P2->M[2][0]	+	P1->M[0][3]*P2->M[3][0];
		P->M[0][1]=P1->M[0][0]*P2->M[0][1]	+	P1->M[0][1]*P2->M[1][1]	+	P1->M[0][2]*P2->M[2][1]	+	P1->M[0][3]*P2->M[3][1];
		P->M[0][2]=P1->M[0][0]*P2->M[0][2]	+	P1->M[0][1]*P2->M[1][2]	+	P1->M[0][2]*P2->M[2][2]	+	P1->M[0][3]*P2->M[3][2];
		P->M[0][3]=P1->M[0][0]*P2->M[0][3]	+	P1->M[0][1]*P2->M[1][3]	+	P1->M[0][2]*P2->M[2][3]	+	P1->M[0][3]*P2->M[3][3];


		P->M[1][0]=P1->M[1][0]*P2->M[0][0]	+	P1->M[1][1]*P2->M[1][0]	+	P1->M[1][2]*P2->M[2][0]	+	P1->M[1][3]*P2->M[3][0];
		P->M[1][1]=P1->M[1][0]*P2->M[0][1]	+	P1->M[1][1]*P2->M[1][1]	+	P1->M[1][2]*P2->M[2][1]	+	P1->M[1][3]*P2->M[3][1];
		P->M[1][2]=P1->M[1][0]*P2->M[0][2]	+	P1->M[1][1]*P2->M[1][2]	+	P1->M[1][2]*P2->M[2][2]	+	P1->M[1][3]*P2->M[3][2];
		P->M[1][3]=P1->M[1][0]*P2->M[0][3]	+	P1->M[1][1]*P2->M[1][3]	+	P1->M[1][2]*P2->M[2][3]	+	P1->M[1][3]*P2->M[3][3];

		P->M[2][0]=P1->M[2][0]*P2->M[0][0]	+	P1->M[2][1]*P2->M[1][0]	+	P1->M[2][2]*P2->M[2][0]	+	P1->M[2][3]*P2->M[3][0];
		P->M[2][1]=P1->M[2][0]*P2->M[0][1]	+	P1->M[2][1]*P2->M[1][1]	+	P1->M[2][2]*P2->M[2][1]	+	P1->M[2][3]*P2->M[3][1];
		P->M[2][2]=P1->M[2][0]*P2->M[0][2]	+	P1->M[2][1]*P2->M[1][2]	+	P1->M[2][2]*P2->M[2][2]	+	P1->M[2][3]*P2->M[3][2];
		P->M[2][3]=P1->M[2][0]*P2->M[0][3]	+	P1->M[2][1]*P2->M[1][3]	+	P1->M[2][2]*P2->M[2][3]	+	P1->M[2][3]*P2->M[3][3];

		P->M[3][0]=P1->M[3][0]*P2->M[0][0]	+	P1->M[3][1]*P2->M[1][0]	+	P1->M[3][2]*P2->M[2][0]	+	P1->M[3][3]*P2->M[3][0];
		P->M[3][1]=P1->M[3][0]*P2->M[0][1]	+	P1->M[3][1]*P2->M[1][1]	+	P1->M[3][2]*P2->M[2][1]	+	P1->M[3][3]*P2->M[3][1];
		P->M[3][2]=P1->M[3][0]*P2->M[0][2]	+	P1->M[3][1]*P2->M[1][2]	+	P1->M[3][2]*P2->M[2][2]	+	P1->M[3][3]*P2->M[3][2];
		P->M[3][3]=P1->M[3][0]*P2->M[0][3]	+	P1->M[3][1]*P2->M[1][3]	+	P1->M[3][2]*P2->M[2][3]	+	P1->M[3][3]*P2->M[3][3];

		return P;
	}

	if(P1->rows==4 && P1->columns==3 && P2->rows==3 && P2->columns==4){
			P->M[0][0]=P1->M[0][0]*P2->M[0][0]	+	P1->M[0][1]*P2->M[1][0]	+	P1->M[0][2]*P2->M[2][0]	;
			P->M[0][1]=P1->M[0][0]*P2->M[0][1]	+	P1->M[0][1]*P2->M[1][1]	+	P1->M[0][2]*P2->M[2][1]	;
			P->M[0][2]=P1->M[0][0]*P2->M[0][2]	+	P1->M[0][1]*P2->M[1][2]	+	P1->M[0][2]*P2->M[2][2]	;
			P->M[0][3]=P1->M[0][0]*P2->M[0][3]	+	P1->M[0][1]*P2->M[1][3]	+	P1->M[0][2]*P2->M[2][3]	;


			P->M[1][0]=P1->M[1][0]*P2->M[0][0]	+	P1->M[1][1]*P2->M[1][0]	+	P1->M[1][2]*P2->M[2][0];
			P->M[1][1]=P1->M[1][0]*P2->M[0][1]	+	P1->M[1][1]*P2->M[1][1]	+	P1->M[1][2]*P2->M[2][1];
			P->M[1][2]=P1->M[1][0]*P2->M[0][2]	+	P1->M[1][1]*P2->M[1][2]	+	P1->M[1][2]*P2->M[2][2];
			P->M[1][3]=P1->M[1][0]*P2->M[0][3]	+	P1->M[1][1]*P2->M[1][3]	+	P1->M[1][2]*P2->M[2][3]	;

			P->M[2][0]=P1->M[2][0]*P2->M[0][0]	+	P1->M[2][1]*P2->M[1][0]	+	P1->M[2][2]*P2->M[2][0];
			P->M[2][1]=P1->M[2][0]*P2->M[0][1]	+	P1->M[2][1]*P2->M[1][1]	+	P1->M[2][2]*P2->M[2][1];
			P->M[2][2]=P1->M[2][0]*P2->M[0][2]	+	P1->M[2][1]*P2->M[1][2]	+	P1->M[2][2]*P2->M[2][2];
			P->M[2][3]=P1->M[2][0]*P2->M[0][3]	+	P1->M[2][1]*P2->M[1][3]	+	P1->M[2][2]*P2->M[2][3];

			P->M[3][0]=P1->M[3][0]*P2->M[0][0]	+	P1->M[3][1]*P2->M[1][0]	+	P1->M[3][2]*P2->M[2][0];
			P->M[3][1]=P1->M[3][0]*P2->M[0][1]	+	P1->M[3][1]*P2->M[1][1]	+	P1->M[3][2]*P2->M[2][1];
			P->M[3][2]=P1->M[3][0]*P2->M[0][2]	+	P1->M[3][1]*P2->M[1][2]	+	P1->M[3][2]*P2->M[2][2];
			P->M[3][3]=P1->M[3][0]*P2->M[0][3]	+	P1->M[3][1]*P2->M[1][3]	+	P1->M[3][2]*P2->M[2][3];

			return P;
		}

	if(P1->rows==3 && P1->columns==4 && P2->rows==4 && P2->columns==3){
		P->M[0][0]=P1->M[0][0]*P2->M[0][0]	+	P1->M[0][1]*P2->M[1][0]	+	P1->M[0][2]*P2->M[2][0]	+	P1->M[0][3]*P2->M[3][0];
		P->M[0][1]=P1->M[0][0]*P2->M[0][1]	+	P1->M[0][1]*P2->M[1][1]	+	P1->M[0][2]*P2->M[2][1]	+	P1->M[0][3]*P2->M[3][1];
		P->M[0][2]=P1->M[0][0]*P2->M[0][2]	+	P1->M[0][1]*P2->M[1][2]	+	P1->M[0][2]*P2->M[2][2]	+	P1->M[0][3]*P2->M[3][2];

		P->M[1][0]=P1->M[1][0]*P2->M[0][0]	+	P1->M[1][1]*P2->M[1][0]	+	P1->M[1][2]*P2->M[2][0]	+	P1->M[1][3]*P2->M[3][0];
		P->M[1][1]=P1->M[1][0]*P2->M[0][1]	+	P1->M[1][1]*P2->M[1][1]	+	P1->M[1][2]*P2->M[2][1]	+	P1->M[1][3]*P2->M[3][1];
		P->M[1][2]=P1->M[1][0]*P2->M[0][2]	+	P1->M[1][1]*P2->M[1][2]	+	P1->M[1][2]*P2->M[2][2]	+	P1->M[1][3]*P2->M[3][2];

		P->M[2][0]=P1->M[2][0]*P2->M[0][0]	+	P1->M[2][1]*P2->M[1][0]	+	P1->M[2][2]*P2->M[2][0]	+	P1->M[2][3]*P2->M[3][0];
		P->M[2][1]=P1->M[2][0]*P2->M[0][1]	+	P1->M[2][1]*P2->M[1][1]	+	P1->M[2][2]*P2->M[2][1]	+	P1->M[2][3]*P2->M[3][1];
		P->M[2][2]=P1->M[2][0]*P2->M[0][2]	+	P1->M[2][1]*P2->M[1][2]	+	P1->M[2][2]*P2->M[2][2]	+	P1->M[2][3]*P2->M[3][2];


		return P;
	}


	for(int r=0;r<P1->rows;r++){
		for(int c=0;c<P2->columns;c++){
			for(int i=0;i<P1->columns;i++){
			P->M[r][c]=P1->M[r][i]*P2->M[i][c];
			}
		}
	}

return P;
}

/*
 * Determinant of the matrix
 */
double Determinant_mat(SimpleMat *P){
	double det;
	if(P->rows!=P->columns){
			cout<<"Not square for determinant"<<endl;
			exit(1);
		}


	if(P->rows==3){

		det=P->M[0][0]*(P->M[1][1]*P->M[2][2]-P->M[1][2]*P->M[2][1])-P->M[0][1]*(P->M[1][0]*P->M[2][2]-P->M[1][2]*P->M[2][0]) + P->M[0][2]*(P->M[1][0]*P->M[2][1]-P->M[1][1]*P->M[2][0]);

	}

	if(P->rows==4){

		det=	P->M[0][0]*P->M[1][1]*P->M[2][2]*P->M[3][3]	+	P->M[0][0]*P->M[1][2]*P->M[2][3]*P->M[3][1] +	P->M[0][0]*P->M[1][3]*P->M[2][1]*P->M[3][2]
		       +P->M[0][1]*P->M[1][0]*P->M[2][3]*P->M[3][2]	+	P->M[0][1]*P->M[1][2]*P->M[2][0]*P->M[3][3] +	P->M[0][1]*P->M[1][3]*P->M[2][2]*P->M[3][0]
		       +P->M[0][2]*P->M[1][0]*P->M[2][1]*P->M[3][3]	+	P->M[0][2]*P->M[1][1]*P->M[2][3]*P->M[3][0] +	P->M[0][2]*P->M[1][3]*P->M[2][0]*P->M[3][1]
		       +P->M[0][3]*P->M[1][0]*P->M[2][2]*P->M[3][1]	+	P->M[0][3]*P->M[1][1]*P->M[2][0]*P->M[3][2] +	P->M[0][3]*P->M[1][2]*P->M[2][1]*P->M[3][0]
		       -P->M[0][0]*P->M[1][1]*P->M[2][3]*P->M[3][2]	-	P->M[0][0]*P->M[1][2]*P->M[2][1]*P->M[3][3] -	P->M[0][0]*P->M[1][3]*P->M[2][2]*P->M[3][1]
		       -P->M[0][1]*P->M[1][0]*P->M[2][2]*P->M[3][3]	-	P->M[0][1]*P->M[1][2]*P->M[2][3]*P->M[3][0] -	P->M[0][1]*P->M[1][3]*P->M[2][0]*P->M[3][2]
		       -P->M[0][2]*P->M[1][0]*P->M[2][3]*P->M[3][1]	-	P->M[0][2]*P->M[1][1]*P->M[2][0]*P->M[3][3] -	P->M[0][2]*P->M[1][3]*P->M[2][1]*P->M[3][0]
		       -P->M[0][3]*P->M[1][0]*P->M[2][1]*P->M[3][2]	-	P->M[0][3]*P->M[1][1]*P->M[2][2]*P->M[3][0] -	P->M[0][3]*P->M[1][2]*P->M[2][0]*P->M[3][1];


	}

cout<<det<<endl;
	return det;
}

/*
 * Inverse of P into IP
 */
SimpleMat * Inverse_mat(SimpleMat *IP,SimpleMat *P){

	if(P->rows!=P->columns){
		cout<<"Not square for inverse"<<endl;
		exit(1);
	}

	double det=Determinant_mat(P);

	if(det==0){
		cout<<"Det is zero cant inverse it"<<endl;
		exit(1);
	}



	double a,b;
	int pr=0;

	for(int i=0;i<P->rows;i++){
		IP->M[i][i]=1;
		}


for(int c=0;c<P->columns;c++){
	//Gauss Jordan algo for inverse
/*	for(int r=0;r<P->rows;r++){
      if(P->M[r][c]!=0){       //get pivot row of that column
    	  pr=r;
    	  break;
      }
	}
*/

      for(int rr=0;rr<P->rows;rr++){

    	  a=P->M[pr][c];
    	  b=P->M[pr][c];
    	  for(int cc=0;cc<P->columns;cc++){
    		  IP->M[pr][cc]=IP->M[pr][cc]/a;
    		  P->M[pr][cc]=P->M[pr][cc]/b;


    	  }

    	  if(pr!=rr){
    		  a=P->M[rr][c];
    		  b=P->M[rr][c];
    		  for(int cc=0;cc<P->columns;cc++){

    			  IP->M[rr][cc]=IP->M[rr][cc]-b*IP->M[pr][cc];
    			  P->M[rr][cc]=P->M[rr][cc]-a*P->M[pr][cc];


    		  }
    	  }
      }



pr++;
	}


	return IP;
}
