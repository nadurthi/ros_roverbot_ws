/*
 * BlackBoxRecorder.h
 *
 *  Created on: Jun 9, 2014
 *      Author: nagnanamus
 */

#ifndef BLACKBOXRECORDER_H_
#define BLACKBOXRECORDER_H_

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include<fcntl.h>
#include<string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <stropts.h>
#include <time.h>
#include <sys/time.h>
#include <fstream>




#include <stdint.h>


#define NUM_THREADS	 1
#define myFIFO "/home/root/Desktop/codes/myfifo"   // once created right click the fifo file and check the permissions. if not give the permissions there


struct thread_data
{

};
struct thread_data thread_data_1;




void *SaveData(void *threadarg){

	ofstream DataRaw;
	char buffer[500];
	DataRaw.open ("Raw_onboard_EKF.txt");


	//snprintf(namebuf, sizeof(namebuf), "%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s\n","t","x_meas","y_meas","z_meas","q0_meas","q1_meas","q2_meas","q3_meas","ax_meas","ay_meas","az_meas","wx_meas","wy_meas","wz_meas","x","y","z","q0","q1","q2","q3","ax","ay","az","wx","wy","wz","Marker1x","Marker1y","Marker1z"  ,"Marker2x","Marker2y","Marker2z"  ,"Marker3x","Marker3y","Marker3z"  ,"Marker4x","Marker4y","Marker4z"	,"Marker5x","Marker5y","Marker5z"	,"Marker6x","Marker6y","Marker6z"	,"Marker7x","Marker7y","Marker7z");
	//snprintf(namebuf, sizeof(namebuf), "%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s\n","t","x_meas","y_meas","z_meas","q0_meas","q1_meas","q2_meas","q3_meas","ax_meas","ay_meas","az_meas","wx_meas","wy_meas","wz_meas","x","y","z","q0","q1","q2","q3","ax","ay","az","wx","wy","wz","Marker1x","Marker1y","Marker1z"  ,"Marker2x","Marker2y","Marker2z"  ,"Marker3x","Marker3y","Marker3z"  ,"Marker4x","Marker4y","Marker4z"	,"Marker5x","Marker5y","Marker5z"	,"Marker6x","Marker6y","Marker6z"	,"Marker7x","Marker7y","Marker7z");
	//snprintf(buffer, sizeof(buffer), "%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s\n","t","x_meas","y_meas","z_meas","q0_meas","q1_meas","q2_meas","q3_meas","ax_meas","ay_meas","az_meas","wx_meas","wy_meas","wz_meas","x","y","z","q0","q1","q2","q3","vx","vy","vz","ax","ay","az","wx","wy","wz","wxdot","wydot","wzdot","Fd","u1","u2","u3","u4","M1","M2","M3");
	snprintf(buffer, sizeof(buffer), "%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s\n","t","x_meas","y_meas","z_meas","q0_meas","q1_meas","q2_meas","q3_meas","ax_meas","ay_meas","az_meas","wx_meas","wy_meas","wz_meas","x","y","z","q0","q1","q2","q3","vx","vy","vz","ax","ay","az","wx","wy","wz","Fd","u1","u2","u3","u4","M1","M2","M3");

	DataRaw << buffer;
	DataRaw.flush();

cout<<buffer<<endl;



    struct thread_data *my_data;

   my_data = (struct thread_data *) threadarg;

   int fd = open(myFIFO , O_RDONLY| O_NONBLOCK); //| O_NONBLOCK
   if(fd<0){
	   cout<<"error in thread pipe open"<<endl;
   }
int n;
bzero(buffer ,  sizeof(buffer));
   while(1){

	    n=read(fd,buffer ,  sizeof(buffer));
//cout<<"n = "<< n<<".......... "<<buffer<<endl;
	    if(n>0)
		{
	    	DataRaw << buffer;
	    	DataRaw.flush();
	    	bzero(buffer ,  sizeof(buffer));
		}



	    usleep(10000);

   }


   pthread_exit(NULL);
}


int Init_thread(){
	int threadfifo_fd;

	pthread_t t;
	int rc = pthread_create(&t, NULL, SaveData, (void *)&thread_data_1);
	 if (rc) {
	   printf("ERROR; return code from pthread_create() is %d\n", rc);
	   exit(-1);
	   }
	 cout<<"done opening the thread"<<endl;
	usleep(100000);

	int status = mkfifo(myFIFO, 0666);
	threadfifo_fd = open(myFIFO, O_WRONLY| O_NONBLOCK);
	cout<<"done opening fifo pipe for recording data\n";
	usleep(100000);

	return threadfifo_fd;
}

void PushData2BlackBoxPipe(int threadfifo_fd ,char * namebuf,int len){

   // cout<<"push function = "<<namebuf;
    write (threadfifo_fd,namebuf , len);
}
/*
 *
 class BlackBoxRecorder {
public:
	BlackBoxRecorder();
	virtual ~BlackBoxRecorder();
};
*/
#endif /* BLACKBOXRECORDER_H_ */
