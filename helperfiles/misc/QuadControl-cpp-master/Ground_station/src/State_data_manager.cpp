/*
 * State_data_manager.cpp
 *
 *  Created on: Apr 19, 2014
 *      Author: nagnanamus
 */

#include"State_data_manager.h"




/*
 * Queue class functions: These functions are written such that they are more useful only for this project
 */
void SimpleQueue::AppendQ(double *D,int *C){
	for(int c=0;c<columns;c++){
		for(int r=0;r<rows;r++){
			if(r<(rows-1)){
				a[r][c]=a[r+1][c]; }
			else{
				a[rows-1][c]=D[ C[c] ];}
		}
	}

}
/*
 * Just fill the queue in series without shifting
 */
void SimpleQueue::SeriesFillQ(double *data){
	if(currind==rows)
	{
		cout<<"Queue is full"<<endl;
		return;
	}

	for(int c=0;c<columns;c++){
			a[currind][c]=data[c];
	}
	currind=currind+1;
}


void SimpleQueue::MeanQ( int r1, int r2) {
	  if(CheckSizeQ(r1,0)+CheckSizeQ(r2,0)<2)
		  exit(1);

	   int r,c;
	   double D;
	   for(c=0;c<columns;c++){
		   D=0;
		   for(r=r1;r<=r2;r++){
			   D=D+a[r][c];
		   }
		   mu[c]=D/(r2-r1+1);
	   }
}

void SimpleQueue::StdQ(int r1, int r2){
	if(CheckSizeQ(r1,0)+CheckSizeQ(r2,0)<2)
			  exit(1);

	int r,c;
	double D;
	MeanQ(r1, r2);
	for(c=0;c<columns;c++){
			   D=0;
			   for(r=r1;r<=r2;r++){
				   D=D+pow(a[r][c]-mu[c],2);
			   }

			   D=sqrt(D/(r2-r1+1));

			   if(D>maxstdev[c])
				   maxstdev[c]=D;

			   stdev[c]=D;
		   }
}

double * SimpleQueue::GetRowQ(int r,double *D){
	if(CheckSizeQ(r,0)<1)
	{cout<<"error in size"<<endl; exit(1);}

	for(int i=0;i<columns;i++){
		D[i]=a[r][i];
	}

	return D;
}

double * SimpleQueue::GetColQ(int c,double *D){
	if(CheckSizeQ(0,c)<1)
			  exit(1);

	for(int i=0;i<rows;i++){
		D[i]=a[i][c];
	}

	return D;
}

double * SimpleQueue::GetLastRowQ(double *D){
	for(int i=0;i<columns;i++){
		D[i]=a[rows-1][i];
	}
	return D;
}
double * SimpleQueue::GetLastButOneRowQ(double *D){
	for(int i=0;i<columns;i++){
			D[i]=a[rows-2][i];
		}
		return D;
}
void SimpleQueue::ReplaceRowQ(int r,double *D){
	if(CheckSizeQ(r,0)<1)
			  exit(1);

	for(int c=0;c<columns;c++){
	a[r][c]=D[c];
	}
}
void SimpleQueue::ReplaceLastRowQ(double *D){
	for(int c=0;c<columns;c++){
	a[rows-1][c]=D[c];
	}
}
double * SimpleQueue::AvgOutlierFilterQ(double *D){

	//outlier remover and averager
	StdQ(0, rows-2);//mu for N-1 rows is evalauted here already
    for(int c=0;c<columns;c++){
    	if(abs(mu[c]-a[rows-1][c])>5*(0.5*maxstdev[c]+0.5*stdev[c])){
    		a[rows-1][c]=mu[c];
    	}
    		D[c]=((rows-1)*mu[c]+a[rows-1][c])/(rows); //movign average. mu was calculated above for N-1 rows (N total rows)


    }

return D;

}
double * SimpleQueue::AvgOutlierFilterQnorm(double *D){


	//outlier remover and averager
	StdQ(0, rows-2);//mu for N-1 rows is evalauted here already
	double cnorm=sqrt(pow(mu[0],2)+pow(mu[1],2)+pow(mu[2],2)+pow(mu[3],2));
	mu[0]=mu[0]/cnorm;
	mu[1]=mu[1]/cnorm;
	mu[2]=mu[2]/cnorm;
	mu[3]=mu[3]/cnorm;


    for(int c=0;c<columns;c++){
    	if(abs(mu[c]-a[rows-1][c])>5*(0.5*maxstdev[c]+0.5*stdev[c])){
    		a[rows-1][c]=mu[c];
    	}
    		D[c]=((rows-1)*mu[c]+a[rows-1][c])/(rows); //movign average. mu was calculated above for N-1 rows (N total rows)
    }

        cnorm=sqrt(pow(D[0],2)+pow(D[1],2)+pow(D[2],2)+pow(D[3],2));
    	D[0]=D[0]/cnorm;
    	D[1]=D[1]/cnorm;
    	D[2]=D[2]/cnorm;
    	D[3]=D[3]/cnorm;

return D;

}

void QuadStateVariable::Init_SetZero(){


/*
			DataRaw.open ("Raw_onboard_EKF.txt");
			DataRawparas.open ("Raw_onboard_paras_EKF.txt");

			//snprintf(namebuf, sizeof(namebuf), "%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s\n","t","x_meas","y_meas","z_meas","q0_meas","q1_meas","q2_meas","q3_meas","ax_meas","ay_meas","az_meas","wx_meas","wy_meas","wz_meas","x","y","z","q0","q1","q2","q3","ax","ay","az","wx","wy","wz","Marker1x","Marker1y","Marker1z"  ,"Marker2x","Marker2y","Marker2z"  ,"Marker3x","Marker3y","Marker3z"  ,"Marker4x","Marker4y","Marker4z"	,"Marker5x","Marker5y","Marker5z"	,"Marker6x","Marker6y","Marker6z"	,"Marker7x","Marker7y","Marker7z");
			snprintf(namebuf, sizeof(namebuf), "%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s\n","t","x_meas","y_meas","z_meas","q0_meas","q1_meas","q2_meas","q3_meas","ax_meas","ay_meas","az_meas","wx_meas","wy_meas","wz_meas","x","y","z","q0","q1","q2","q3","vx","vy","vz","ax","ay","az","wx","wy","wz","wxdot","wydot","wzdot","Fd","u1","u2","u3","u4","M1","M2","M3");
			DataRaw << namebuf;

			snprintf(namebuf, sizeof(namebuf), "%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s\n","bax","bay","baz","bgx","bgy","bgz","maxdevx","maxdevy","maxdevz","maxdevq0","maxdevq1","maxdevq2","maxdevq3","maxdevax","maxdevay","maxdevaz","maxdevwx","maxdevwy","maxdevwz");
			DataRawparas<<namebuf;
*/


tground.CreateQ(data_queue_length,1);
tonboard.CreateQ(data_queue_length,1);


Data_w_meas.CreateQ(data_queue_length,3);
Data_a_meas.CreateQ(data_queue_length,3);
Data_q_meas.CreateQ(data_queue_length,4);
Data_x_meas.CreateQ(data_queue_length,3);

Data_Vdiff.CreateQ(data_queue_length,3);
Data_Vest.CreateQ(data_queue_length,3);

int i,j;
for(i=0;i<data_queue_length;i++){
	tground.a[i][0]=0;
	tonboard.a[i][0]=0;
	for(j=0;j<3;j++){
		Data_w_meas.a[i][j]=0;
		Data_a_meas.a[i][j]=0;
		Data_x_meas.a[i][j]=0;

		Data_Vdiff.a[i][j]=0;
		Data_Vest.a[i][j]=0;
		}
	for(j=0;j<4;j++){
		if(j==0){
			Data_q_meas.a[i][j]=1;
				}
		else{
		Data_q_meas.a[i][j]=0;
		     }
		}
}



}


/*
 * Updating qmis: if qmis is unresonable (norm >1)...use the prev value
 */
void QuadStateVariable::Update_qmis(double *qv){
	 double c=sqrt(pow(qv[0],2)+pow(qv[1],2)+pow(qv[2],2)+pow(qv[3],2));

		 if(c>1.2){
		 		qmis[0]=qmis_prev[0];
		 		qmis[1]=qmis_prev[1];
		 		qmis[2]=qmis_prev[2];
		 		qmis[3]=qmis_prev[3];
		 		cout<<"Current qmis values is not compatible (norm greater than 1): hence using previous qmis"<<endl;
		 	}
		 	else
		 		{
		 		qmis_prev[0]=qmis[0];
		 		qmis_prev[1]=qmis[1];
		 		qmis_prev[2]=qmis[2];
		 		qmis_prev[3]=qmis[3];

		 		qmis[0]=qv[0];
		 		qmis[1]=qv[1];
		 		qmis[2]=qv[2];
		 		qmis[3]=qv[3];
		 		}

	 }



/*
 * Updating qref: if qref is unresonable (norm >1)...use the prev value
 */
void QuadStateVariable::Update_qref(double *qv){
	 double c=sqrt(pow(qv[0],2)+pow(qv[1],2)+pow(qv[2],2)+pow(qv[3],2));

		 if(c>1.2){
		 		qref[0]=qref_prev[0];
		 		qref[1]=qref_prev[1];
		 		qref[2]=qref_prev[2];
		 		qref[3]=qref_prev[3];
		 		cout<<"Current qref values is not compatible (norm greater than 1): hence using previous qref"<<endl;
		 	}
		 	else
		 		{
		 		qref_prev[0]=qref[0];
		 		qref_prev[1]=qref[1];
		 		qref_prev[2]=qref[2];
		 		qref_prev[3]=qref[3];

		 		qref[0]=qv[0];
		 		qref[1]=qv[1];
		 		qref[2]=qv[2];
		 		qref[3]=qv[3];
		 		}

	 }


/*   accelgyro=[t a w]
 *  option for with or without outlier correction+avg filter
 *  option =0: just load into queue and do not correct
 *  option =1: yes do the outlier correction with mov avg filter
 *             The direct/corrected+averaged result is put in the respective _meas variables
 *  option = -1:  Just load into _meas the raw data...DONOT Queue it and DONOT correct it
 */

void QuadStateVariable::Update_AccelGyro_meas(double *accelgyro,int option){
	int Ct[1]={0},Ca[3]={1,2,3},Cg[3]={4,5,6};

	t=accelgyro[0];

	a_meas[0]=accelgyro[1];
	a_meas[1]=accelgyro[2];
	a_meas[2]=accelgyro[3];

	w_meas[0]=accelgyro[4];
	w_meas[1]=accelgyro[5];
	w_meas[2]=accelgyro[6];

	a_raw[0]=a_meas[0];
	a_raw[1]=a_meas[1];
	a_raw[2]=a_meas[2];

	w_raw[0]=w_meas[0];
	w_raw[1]=w_meas[1];
	w_raw[2]=w_meas[2];

	if(option ==-1){
		return;
	}



	tonboard.AppendQ(accelgyro,Ct); //first column in accelgyro is time

	Data_a_meas.AppendQ(accelgyro,Ca);
	Data_w_meas.AppendQ(accelgyro,Cg);
	if(option==1){

		Data_a_meas.AvgOutlierFilterQ(a_meas);
		Data_w_meas.AvgOutlierFilterQ(w_meas);


	}
}

/*   accelgyro=[t x q]
 *  option for with or without outlier correction+avg filter
 *  option=0:  Just load into the queue but do not do any correction
 *  option =1: yes do the outlier correction with mov avg filter
 *             The direct/corrected+averaged result is put in the respective _meas variables
 *  option =-1: Just load the _meas with raw measurements.... Do not load into Queue and do not correct
 *
 */
void QuadStateVariable::Update_Vicon_meas(double *vicondata,int option){

	int Ct[1]={0},Cx[3]={1,2,3},Cq[4]={4,5,6,7};

	//t=vicondata[0];

	tground.AppendQ(vicondata,Ct);

	x_meas[0]=vicondata[1];
	x_meas[1]=vicondata[2];
	x_meas[2]=vicondata[3];

	q_meas[0]=vicondata[4];
	q_meas[1]=vicondata[5];
	q_meas[2]=vicondata[6];
	q_meas[3]=vicondata[7];

	if(option ==-1){
		return;
	}

	Data_x_meas.AppendQ(vicondata,Cx);
	Data_q_meas.AppendQ(vicondata,Cq);

	if(option==1){

		Data_x_meas.AvgOutlierFilterQ(x_meas);
		Data_q_meas.AvgOutlierFilterQnorm(q_meas);

	}

}

void QuadStateVariable::Update_x_meas(int option){

	int C[3]={0,1,2};

		x_raw[0]=x_meas[0];
		x_raw[1]=x_meas[1];
		x_raw[2]=x_meas[2];

	if(option ==-1){
		return;
	}

	Data_x_meas.AppendQ(x_meas,C);

	if(option==1){

		Data_x_meas.AvgOutlierFilterQ(x_meas);

	}

}
void QuadStateVariable::Update_w_meas(int option){

	int C[3]={0,1,2};
		w_raw[0]=w_meas[0];
		w_raw[1]=w_meas[1];
		w_raw[2]=w_meas[2];


	if(option ==-1){
		return;
	}

	Data_w_meas.AppendQ(w_meas,C);

	if(option==1){

		Data_w_meas.AvgOutlierFilterQ(w_meas);

	}

}

void QuadStateVariable::Update_a_meas(int option){

	int C[3]={0,1,2};

	a_raw[0]=a_meas[0];
	a_raw[1]=a_meas[1];
	a_raw[2]=a_meas[2];

	if(option ==-1){
		return;
	}

	Data_a_meas.AppendQ(a_meas,C);

	if(option==1){

		Data_a_meas.AvgOutlierFilterQ(a_meas);

	}

}
void QuadStateVariable::Update_q_meas(int option){

	int C[4]={0,1,2,3};
		q_raw[0]=q_meas[0];
		q_raw[1]=q_meas[1];
		q_raw[2]=q_meas[2];
		q_raw[3]=q_meas[3];

	if(option ==-1){
		return;
	}

	Data_q_meas.AppendQ(q_meas,C);

	if(option==1){

		Data_q_meas.AvgOutlierFilterQ(q_meas);

	}

}
void QuadStateVariable::Update_a(double *vv){
	double c=sqrt(pow(vv[0],2)+pow(vv[1],2)+pow(vv[2],2));

	if(c>a_lim){
		 		a[0]=a_prev[0];
		 		a[1]=a_prev[1];
		 		a[2]=a_prev[2];

		 		cout<<"Current a values is not compatible (norm greater than a_lim): hence using previous a"<<endl;
		 	}
		 	else
		 		{
		 		a_prev[0]=a[0];
		 		a_prev[1]=a[1];
		 		a_prev[2]=a[2];


		 		a[0]=vv[0];
		 		a[1]=vv[1];
		 		a[2]=vv[2];
	 	 		}
}

void QuadStateVariable::Update_wdot(double *vv){
	double c=sqrt(pow(vv[0],2)+pow(vv[1],2)+pow(vv[2],2));

	if(c>wdot_lim){
		wdot[0]=wdot_prev[0];
		wdot[1]=wdot_prev[1];
		wdot[2]=wdot_prev[2];

		 		cout<<"Current wdot values is not compatible (norm greater than wdot_lim): hence using previous wdot"<<endl;
		 	}
		 	else
		 		{
		 		wdot_prev[0]=wdot[0];
		 		wdot_prev[1]=wdot[1];
		 		wdot_prev[2]=wdot[2];


		 		wdot[0]=vv[0];
		 		wdot[1]=vv[1];
		 		wdot[2]=vv[2];
	 	 		}

}


void QuadStateVariable::Update_ba(double *vv){
	double c=sqrt(pow(vv[0],2)+pow(vv[1],2)+pow(vv[2],2));

	if(c>ba_lim){
		 		ba[0]=ba_prev[0];
		 		ba[1]=ba_prev[1];
		 		ba[2]=ba_prev[2];

		 		cout<<"Current ba values is not compatible (norm greater than ba lim): hence using previous ba"<<endl;
		 	}
		 	else
		 		{
		 		ba_prev[0]=ba[0];
		 		ba_prev[1]=ba[1];
		 		ba_prev[2]=ba[2];


		 		ba[0]=vv[0];
		 		ba[1]=vv[1];
		 		ba[2]=vv[2];
	 	 		}
}
void QuadStateVariable::Update_bg(double *vv){
	double c=sqrt(pow(vv[0],2)+pow(vv[1],2)+pow(vv[2],2));

	if(c>bg_lim){
		 		bg[0]=bg_prev[0];
		 		bg[1]=bg_prev[1];
		 		bg[2]=bg_prev[2];

		 		cout<<"Current bg values is not compatible (norm greater than bg lim): hence using previous bg"<<endl;
		 	}
		 	else
		 		{
		 		bg_prev[0]=bg[0];
		 		bg_prev[1]=bg[1];
		 		bg_prev[2]=bg[2];


		 		bg[0]=vv[0];
		 		bg[1]=vv[1];
		 		bg[2]=vv[2];
	 	 		}
}
void QuadStateVariable::Update_rmis(double *vv){
	double c=sqrt(pow(vv[0],2)+pow(vv[1],2)+pow(vv[2],2));

	if(c>rmis_lim){
		rmis[0]=rmis_prev[0];
		rmis[1]=rmis_prev[1];
		rmis[2]=rmis_prev[2];

		 		cout<<"Current rmis values is not compatible (norm greater than rmis lim): hence using previous rmis"<<endl;
		 	}
		 	else
		 		{
		 		rmis_prev[0]=rmis[0];
		 		rmis_prev[1]=rmis[1];
		 		rmis_prev[2]=rmis[2];


		 		rmis[0]=vv[0];
		 		rmis[1]=vv[1];
		 		rmis[2]=vv[2];
	 	 		}
}
void QuadStateVariable::Update_u(double *vu){


	if(vu[0]>ulim || vu[1]>ulim || vu[2]>ulim || vu[3]>ulim){
		u[0]=u_prev[0];
		u[1]=u_prev[1];
		u[2]=u_prev[2];
		u[3]=u_prev[3];

		 		cout<<"Current u values is not compatible (greater than ulim): hence using previous u"<<endl;
		 	}
		 	else
		 		{
		 		u_prev[0]=u[0];
		 		u_prev[1]=u[1];
		 		u_prev[2]=u[2];
		 		u_prev[3]=u[3];


		 		u[0]=vu[0];
		 		u[1]=vu[1];
		 		u[2]=vu[2];
		 		u[3]=vu[3];
	 	 		}

}

void QuadStateVariable::PrintStates(){
	cout<<"--------------------------------------------- NEW DATA -------------------------------------------------------------------------";
	cout<<endl;
	cout<<"t    = "<<t<<"    "<<"tground = "<<t<<endl;
	cout<<"x    = "<<x[0]<<" "<<x[1]<<" "<<x[2]  <<endl;
	cout<<"v    = "<<v[0]<<" "<<v[1]<<" "<<v[2]  <<endl;
	cout<<"a    = "<<a[0]<<" "<<a[1]<<" "<<a[2]  <<"	a limit = "<<a_lim<<endl;
	cout<<"w    = "<<w[0]<<" "<<w[1]<<" "<<w[2]  <<endl;
	cout<<"wdot = "<<wdot[0]<<" "<<wdot[1]<<" "<<wdot[2]<<"	wdot limit = "<<wdot_lim<<endl;
	cout<<"q    = "<<q[0]<<" "<<q[1]<<" "<<q[2]<<" "<<q[3]  <<endl<<endl;


	cout<<"x_meas = "<<x_meas[0]<<" "<<x_meas[1]<<" "<<x_meas[2]  <<endl;
	cout<<"a_meas = "<<a_meas[0]<<" "<<a_meas[1]<<" "<<a_meas[2]<<endl;
	cout<<"w_meas = "<<w_meas[0]<<" "<<w_meas[1]<<" "<<w_meas[2]<<endl;
	cout<<"q_meas = "<<q_meas[0]<<" "<<q_meas[1]<<" "<<q_meas[2]<<" "<<q_meas[3]  <<endl<<endl;


	cout<<"Fd = "<<Fd<<"Fd_lim = "<<Fdnorm_limit<<"    ulimit  = "<< ulim<<endl;
	cout<<"u  = "<<u[0]<<" "<<u[1]<<" "<<u[2]<<" "<<u[3]<<endl<<endl;


	cout<<"rmis  = "<<rmis[0]<<" "<<rmis[1]<<" "<<rmis[2]<<"	rmislimit = "<<rmis_lim<<endl;
	cout<<"ba  = "<<ba[0]<<" "<<ba[1]<<" "<<ba[2]<<"	ba limit = "<<ba_lim<<endl;
	cout<<"bg  = "<<bg[0]<<" "<<bg[1]<<" "<<bg[2]<<"	bg limit = "<<bg_lim<<endl;
	cout<<"qmis  = "<<qmis[0]<<" "<<qmis[1]<<" "<<qmis[2]<<" "<<qmis[3]<<endl;


	cout<<"tground		Data_w_meas		Data_a_meas		Data_x_meas		Data_q_meas"<<endl;

	double D3[3],D4[4],D;

	for(int i=0;i<data_queue_length;i++){

		tground.GetRowQ(i,&D);
		cout<<D<<"		";
		Data_w_meas.GetRowQ(i,D3);
		cout<< D3[0] <<" " << D3[1] << " " << D3[2] << "			";
		Data_a_meas.GetRowQ(i,D3);
		cout<< D3[0] <<" " << D3[1] << " " << D3[2] << "			";
		Data_x_meas.GetRowQ(i,D3);
		cout<< D3[0] <<" " << D3[1] << " " << D3[2] << "			";
		Data_q_meas.GetRowQ(i,D4);
		cout<< D4[0] <<" " << D4[1] << " " << D4[2] <<" "<<D4[3]<<endl ;

	}



	cout<<endl<<endl;


	Data_w_meas.StdQ(0,data_queue_length-2);
	cout<<"Std: w_meas = "<< Data_w_meas.stdev[0] <<" " << Data_w_meas.stdev[1] << " " << Data_w_meas.stdev[2] <<endl;

	Data_a_meas.StdQ(0,data_queue_length-2);
	cout<<"Std: a_meas = "<< Data_a_meas.stdev[0] <<" " << Data_a_meas.stdev[1] << " " << Data_a_meas.stdev[2] << endl;

	Data_x_meas.StdQ(0,data_queue_length-2);
	cout<<"Std: x_meas = "<< Data_x_meas.stdev[0] <<" " << Data_x_meas.stdev[1] << " " << Data_x_meas.stdev[2] << endl;

	Data_q_meas.StdQ(0,data_queue_length-2);
	cout<<"Std: q_meas = "<< Data_q_meas.stdev[0] <<" " << Data_q_meas.stdev[1] << " " << Data_q_meas.stdev[2] << " " << Data_q_meas.stdev[3] <<endl;

	cout<<endl<<endl;

		cout<<"Mean: w_meas = "<< Data_w_meas.mu[0] <<" " << Data_w_meas.mu[1] << " " << Data_w_meas.mu[2] << "		";

		cout<<"Mean: a_meas = "<< Data_a_meas.mu[0] <<" " << Data_a_meas.mu[1] << " " << Data_a_meas.mu[2] << "		";

		cout<<"Mean: x_meas = "<< Data_x_meas.mu[0] <<" " << Data_x_meas.mu[1] << " " << Data_x_meas.mu[2] << "		";

		cout<<"Mean: q_meas = "<< Data_q_meas.mu[0] <<" " << Data_q_meas.mu[1] << " " << Data_q_meas.mu[2] << " " << Data_q_meas.mu[3] <<endl<<endl;

		cout<<endl<<endl;


			cout<<"MaxStddev: w_meas = "<< Data_w_meas.maxstdev[0] <<" " << Data_w_meas.maxstdev[1] << " " << Data_w_meas.maxstdev[2] << "		";

			cout<<"MaxStddev: a_meas = "<< Data_a_meas.maxstdev[0] <<" " << Data_a_meas.maxstdev[1] << " " << Data_a_meas.maxstdev[2] << "		";

			cout<<"MaxStddev: x_meas = "<< Data_x_meas.maxstdev[0] <<" " << Data_x_meas.maxstdev[1] << " " << Data_x_meas.maxstdev[2] << "		";

			cout<<"MaxStddev: q_meas = "<< Data_q_meas.maxstdev[0] <<" " << Data_q_meas.maxstdev[1] << " " << Data_q_meas.maxstdev[2] << " " << Data_q_meas.maxstdev[3] <<endl<<endl;


	cout<<"Final entry outlier Corrected Data: Previous row        Corrected Row"<<endl;


			tground.GetLastRowQ(&D);
			cout<<D<<"		";
			Data_w_meas.GetLastRowQ(D3);
			cout<< D3[0] <<" " << D3[1] << " " << D3[2] << "		";
			Data_a_meas.GetLastRowQ(D3);
			cout<< D3[0] <<" " << D3[1] << " " << D3[2] << "		";
			Data_x_meas.GetLastRowQ(D3);
			cout<< D3[0] <<" " << D3[1] << " " << D3[2] << "		";
			Data_q_meas.GetLastRowQ(D4);
			cout<< D4[0] <<" " << D4[1] << " " << D4[2] <<" "<<D4[3]<<endl ;


}



int QuadStateVariable::SeparateFields(double *D,char *S,int n){

	char X[50];
    int i=0,j=0;

	for(int k=0;k<n;k++){
		i++;
		j=0;
		while(1){
			if(S[i]==' ')
			{	X[j]='\0';
				break;    }

			X[j]=S[i];
			j++;
			i++;
		}

		D[k]=atof(X);
	}
	return i;


}

    int QuadStateVariable::MsgEncoder(char *S,char *msg){
    	int m=0,i=0;
    	while(msg[i]!='\0'){

    		if(msg[i]==' ')
    			{i++;continue;}

    	switch(msg[i]){

    	case 'q':   //send current quaternions
    		//cout<<msg[i]<<endl;

    		m=m+snprintf(S+m, 50, "q%08.5f %08.5f %08.5f %08.5f ",q[0],q[1], q[2], q[3]);

    		break;
    	case 'w':   //send current  ang rates
    	    m=m+snprintf(S+m, 50, "w%08.4f %08.4f %08.4f ", w[0], w[1], w[2]);
      	    //cout<<"m = "<<m<<" S = "<<S<<endl;
        	//	exit(1);
    	    break;

    	case 'x':   //send current x
    	    m=m+snprintf(S+m, 50, "x%08.4f %08.4f %08.4f ", x[0], x[1], x[2]);
    	    break;

    	case 'a':   //send current  a
    	     m=m+snprintf(S+m, 50, "a%08.4f %08.4f %08.4f ", a[0], a[1], a[2]);
    	     break;

    	case 'v':   //send current v
    	     m=m+snprintf(S+m, 50, "v%08.4f %08.4f %08.4f ", v[0], v[1], v[2]);
    	     break;

       	case 'W':   //send meas w
        	     m=m+snprintf(S+m, 50, "W%08.4f %08.4f %08.4f ", w_meas[0], w_meas[1], w_meas[2]);
        	     break;
       	case 'A':   //send meas a
        	     m=m+snprintf(S+m, 50, "A%08.4f %08.4f %08.4f ", a_meas[0], a_meas[1], a_meas[2]);
        	     break;
       	case 'X':   //send meas x
        	     m=m+snprintf(S+m, 50, "X%07.4f %07.4f %07.4f ", x_meas[0], x_meas[1], x_meas[2]);
        	     break;
       	case 'Q':   //send meas x
        	     m=m+snprintf(S+m, 50, "Q%08.5f %08.5f %08.5f %08.5f ", q_meas[0], q_meas[1], q_meas[2], q_meas[3]);
        	     break;

       	case 'F':   //send Fd or F desired
       	         m=m+snprintf(S+m, 50, "F%010.5f ", Fd);
       	         break;

     	case 'f':   //send Fsafety
       	         m=m+snprintf(S+m, 50, "f%010.5f ", Fsafety);
       	         break;

     	case 'E':   //send kx
       	         m=m+snprintf(S+m, 50, "E%06.3f ", kx);
       	         break;
     	case 'R':   //send kv
       	         m=m+snprintf(S+m, 50, "R%06.3f ", kv);
       	         break;
     	case 'T':   //send kq
       	         m=m+snprintf(S+m, 50, "T%06.3f ", kq);
       	         break;
     	case 'Y':   //send kw
       	         m=m+snprintf(S+m, 50, "Y%06.3f ", kw);
       	         break;
       	case 'U':   //send current ba
        	     m=m+snprintf(S+m, 50, "U%08.4f %08.4f %08.4f ", ba[0], ba[1], ba[2]);
        	     break;
       	case 'I':   //send current bg
        	     m=m+snprintf(S+m, 50, "I%08.4f %08.4f %08.4f ", bg[0], bg[1], bg[2]);
        	     break;
       	case 'O':   //send qmis
       	    	m=m+snprintf(S+m, 50, "O%08.5f %08.5f %08.5f %08.5f ", qmis[0], qmis[1], qmis[2], qmis[3]);
       	    	break;
       	case 'P':   //send rmis
       	    	m=m+snprintf(S+m, 50, "P%06.3f %06.3f %06.3f ", rmis[0], rmis[1], rmis[2]);
       	    	break;
       	case 'H':   //send qdes
       	       	m=m+snprintf(S+m, 50, "A%08.5f %08.5f %08.5f %08.5f ", qdes[0], qdes[1], qdes[2], qdes[3]);
       	       	break;
       	case 'B':   //send qref
       	       	m=m+snprintf(S+m, 50, "B%08.5f %08.5f %08.5f %08.5f ", qref[0], qref[1], qref[2], qref[3]);
       	       	break;
    	case 'C':   //send current xref
    	       m=m+snprintf(S+m, 50, "C%07.4f %07.4f %07.4f ", xref[0], xref[1], xref[2]);
    	       break;
    	case 'D':   //send current b1ref
    	       m=m+snprintf(S+m, 50, "D%07.4f %07.4f %07.4f ", b1ref[0], b1ref[1], b1ref[2]);
    	       break;
    	case 'G':   //send current wref
    	       m=m+snprintf(S+m, 50, "G%08.4f %08.4f %08.4f ", wref[0], wref[1], wref[2]);
    	       break;
    	case 'M':   //send current M
    	       m=m+snprintf(S+m, 50, "M%08.4f %08.4f %08.4f ", M[0], M[1], M[2]);
    	       break;
     	case 'c':   //send ctf
       	         m=m+snprintf(S+m, 50, "c%08.4f ", ctf);
       	         break;
     	default:
     		cout<<"Msg code not present";

    	}
		i++;

    	}
    	return m;
    }

    int QuadStateVariable::MsgDecoder(char *S,int option){

        int i=1;
        int m=0;

        while(S[m]!='\0'){

    	switch(S[m]){

    	case 'q':   //send current quaternions
    		q[0]=0;
            m=m+SeparateFields( q,m+S,4);
            i=i+4;
    		break;
    	case 'w':   //send current  ang rates
    		m=m+SeparateFields( w,m+S,3);
    		i=i+3;
    		break;

    	case 'x':   //send current x
    		m=m+SeparateFields( x,m+S,3);
    		i=i+3;
    	    break;

    	case 'a':   //send current  a
    		 m=m+SeparateFields( w,m+S,3);
    		 i=i+3;
    	     break;

    	case 'v':   //send current v
    		m=m+SeparateFields( v,m+S,3);
    		i=i+3;
    	     break;

       	case 'W':   //send meas w
       		m=m+SeparateFields( w_meas,m+S,3);
       		i=i+3;
       		Update_w_meas(option);

        	     break;
       	case 'A':   //send meas a
       		m=m+SeparateFields( a_meas,m+S,3);
       		i=i+3;
       		Update_a_meas(option);
        	     break;
       	case 'X':   //send meas x
       		m=m+SeparateFields( x_meas,m+S,3);

       		i=i+3;
       		Update_x_meas(option);
       		VICONupdateFLAG=1;
        	     break;
       	case 'Q':   //send meas q
       		m=m+SeparateFields( q_meas,m+S,4);

       		i=i+4;
       		Update_q_meas(option);
       		VICONupdateFLAG=1;
        	     break;

       	case 'F':   //send Fd or F desired
       		m=m+SeparateFields(&Fd,m+S,1);
       		i=i+1;
       	         break;

     	case 'f':   //send Fsafety
     		m=m+SeparateFields(&Fsafety,m+S,1);
     		i=i+1;
       	         break;

     	case 'E':   //send kx
     		m=m+SeparateFields(&kx,m+S,1);
     		i=i+1;
       	         break;
     	case 'R':   //send kv
     		m=m+SeparateFields(&kv,m+S,1);
     		i=i+1;
       	         break;
     	case 'T':   //send kq
     		m=m+SeparateFields(&kq,m+S,1);
     		i=i+1;
       	         break;
     	case 'Y':   //send kw
     		m=m+SeparateFields(&kw,m+S,1);
     		i=i+1;
       	         break;
       	case 'U':   //send current ba
       		m=m+SeparateFields( ba,m+S,3);
       		i=i+3;
        	     break;
       	case 'I':   //send current bg
       		m=m+SeparateFields( bg,m+S,3);
       		i=i+3;
        	     break;
       	case 'O':   //send qmis
       		m=m+SeparateFields( qmis,m+S,4);
       		i=i+4;
       	    	break;
       	case 'P':   //send rmis
       		m=m+SeparateFields( rmis,m+S,3);
       	    	break;
       	case 'H':   //send qdes
       		m=m+SeparateFields( qdes,m+S,4);
       	       	break;
       	case 'B':   //send qref
       		m=m+SeparateFields( qref,m+S,4);
       		i=i+4;
       	       	break;
    	case 'C':   //send current xref
    		m=m+SeparateFields( xref,m+S,3);
    	       break;
    	case 'D':   //send current b1ref
    		m=m+SeparateFields( b1ref,m+S,3);
    		i=i+3;
    	       break;
    	case 'G':   //send current wref
    		m=m+SeparateFields( wref,m+S,3);
    		i=i+3;
    	       break;
    	case 'M':   //send current M
    		m=m+SeparateFields( M,m+S,3);
    		i=i+3;
    	       break;
     	case 'c':   //send ctf
     		m=m+SeparateFields(&ctf,m+S,1);
     		i=i+1;
       	         break;
     	case ' ':
     		m=m+1;
     		     break;
     	default:
     		cout<<"Msg code not present";

    	}

        }
        return i;
    }

