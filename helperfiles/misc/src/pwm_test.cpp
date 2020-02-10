#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <iostream>
#include <stdlib.h>
#include "pwm_4motor.h"
using namespace std;

#define MAX_BUF 64
/****************************************************************
 * gpio_export
 ****************************************************************/

PWMmotors Motorpwms;

int main(int argc, char **argv)
{
	int M1;
	double F,u[4];

	//Setup motor pwm control
		Motorpwms.set_max_min_force();   //#######   SET the MAX FORCE HERE  ##########################
		Motorpwms.init_escs();


	while(1){
		/*
		//Give pwm values directly 100 to 250 range
		cout<<"Give motor pwms = ";
		cin>>M1;  //100 to 250
		cout<<endl;
		Motor_pwm.set_duties(M1*10000,M1*10000,M1*10000,M1*10000);
		*/

		//Give force values directly (assumed that the pwm to force calibraton is done)
		cout<<"Give required force = ";
		cin>>F;  //see .h file for range
		cout<<endl;
		u[0]=F;
		u[1]=F;
		u[2]=F;
		u[3]=F;
		Motorpwms.set_pwms(u);

	}
	Motorpwms.stop_pwms();
	return 0;
}
