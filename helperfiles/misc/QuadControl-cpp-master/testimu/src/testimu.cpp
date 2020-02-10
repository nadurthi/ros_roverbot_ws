
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stropts.h>
#include <cmath>
#include <stdio.h>
#include <string>
#include <sys/types.h>
#include <limits>
#include <time.h>
#include <sys/time.h>
#include <stdint.h>
#include <fstream>
#include <cmath>
#include "EKF.h"
#include "DataManager.h"
#include "UDPcommGroundOnboard.h"
#include "SimpleTimer.h"
#include "State_data_manager.h"

UDPcommGroundOnboard OnboardUDP;
//cd /home/nagnanamus/Dropbox/BBB/codes/quad_control/Ground_station/Release

#include "I2Cdev.h"
#include "MPU6050.h"


MPU6050 AccelGyroSensor;

SimpleTimer OnboardTimer,TicToc;

using namespace std;

int main() {

ofstream a_file;
a_file.open ("testfile.txt");
a_file <<"t"<<","<<"ax"<<","<<"ay"<<","<<"az"<<","<<"wx"<<","<<"wy"<<" "<<"wz" << ","<< "x" << ","<< "y" << ","<< "z" << ","<< "q1" << ","<< "q2" << ","<< "q3" << ","<< "q4" << '\n';
a_file.flush();

//initialize the imu
AccelGyroSensor.initialize();

OnboardTimer.SetTsec(0.1);

//Setup the communication with ground station
	OnboardUDP.Init_as_Onboard();

	QuadStateVariable QuadState;


//double DataVicon[8];
double Data_imu[7],DataVicon[8];



    while(1){
    	OnboardTimer.ResetTimer();


    	cout<<"wait"<<endl;
    	while(OnboardUDP.CheckRxData(10)==false){}

    	OnboardUDP.ReceiveData_on_Onboard(&QuadState,0);

    	DataVicon[1]=QuadState.x_meas[0];DataVicon[2]=QuadState.x_meas[1];DataVicon[3]=QuadState.x_meas[2];
    	DataVicon[4]=QuadState.q_meas[0];DataVicon[5]=QuadState.q_meas[1];DataVicon[6]=QuadState.q_meas[2];DataVicon[7]=QuadState.q_meas[3];


    				AccelGyroSensor.getScaledaccgyro_timestamped(Data_imu);
    				//cout<<"IMU = "<<Data_imu[0]<<" "<<Data_imu[1]<<" "<<Data_imu[2]<<" "<<Data_imu[3]<<" "<<Data_imu[4]<<" "<<Data_imu[5]<<" "<<Data_imu[6]<<endl;
    				cout<<"VICON = "<<DataVicon[1]<<" "<<DataVicon[2]<<" "<<DataVicon[3]<<" "<<DataVicon[4]<<" "<<DataVicon[5]<<" "<<DataVicon[6]<<" "<<DataVicon[7]<<endl;

a_file << Data_imu[0] << "," << Data_imu[1] << ","<< Data_imu[2] <<","<< Data_imu[3] <<","<<Data_imu[4]<<","<<Data_imu[5]<<","<<Data_imu[6] <<","<< DataVicon[1] << ","<< DataVicon[2] << ","<< DataVicon[3] << ","<< DataVicon[4] << ","<< DataVicon[5] << ","<< DataVicon[6] << ","<< DataVicon[7] << '\n';
a_file.flush();

    	while(OnboardTimer.CheckTimer()){}

    } //while eternal


    return 1;
}
