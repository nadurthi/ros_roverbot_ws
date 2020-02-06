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
	int M1,M2,M3,M4;
	double F,u[4];

	//Setup motor pwm control
		Motorpwms.set_max_min_force();   //#######   SET the MAX FORCE HERE  ##########################
		Motorpwms.init_escs();


	while(1){
/*
		//Give pwm values directly 100 to 250 range
		cout<<"Give motor pwms M1 M2 M3 M4 = ";
		cin>>M1;  //100 to 250
		cin>>M2;
		cin>>M3;
		cin>>M4;

		cout<<endl;
		Motorpwms.set_duties(M1*10000,M2*10000,M3*10000,M4*10000);
*/

		//Give force values directly (assumed that the pwm to force calibraton is done)
		cout<<"Give required force = ";
		cin>>M1;  //100 to 250
		cout<<endl;
		Motorpwms.set_duties(M1*10000,M1*10000,M1*10000,M1*10000);

	}
	Motorpwms.stop_pwms();
	return 0;
}
