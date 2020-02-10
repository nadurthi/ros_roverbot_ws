#ifndef PWMCONTROLfourS_H_
#define PWMCONTROLfourS_H_

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
#include <cmath>
#include "State_data_manager.h"
using namespace std;

#define minpwm 1120000
#define maxpwm 1570000
#define MAX_BUF 64

class PWMmotors {
	 unsigned long int duty1,dutyF,dutyL,dutyB,dutyR,period;
	 int maxForce,minForce;

public:
	 void set_max_min_force(){
		 minForce=0;
		 maxForce=3; //in newtons
	 }
	 void export_pwms(){
	int fd, len;
	char buf[MAX_BUF];
    struct stat st;

    //check if already exported...just checking one pwm
	if(stat("/sys/class/pwm/pwm0",&st)==0)
	{printf("pwms already exported\n");return;}

	for(int i=0;i<4;i++){
	fd = open("/sys/class/pwm/export", O_WRONLY);
	if (fd < 0) {
		perror("pwm/export");
		return;
	}

	len = snprintf(buf, sizeof(buf), "%d", i);
	write(fd, buf, len);
	close(fd);
	}

	return;
	 }

 void set_period(){
	int fd, len;
	char buf[MAX_BUF];

	for(int i=0;i<4;i++){
	snprintf(buf, sizeof(buf), "/sys/class/pwm/pwm%d/period_ns", i);
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("pwm/period");
		return;
	}

	len = snprintf(buf, sizeof(buf), "%lu", 20000000);
	write(fd, buf, len);
	close(fd);
	}

	return;
 }
  void set_duties(unsigned long int duty1,unsigned long int duty2,unsigned long int duty3,unsigned long int duty4){
	 int fd, len;
	 char buf[MAX_BUF];

	// write 1 rpm
	fd = open("/sys/class/pwm/pwm0/duty_ns", O_WRONLY);
	if (fd < 0) {
		perror("pwm0/duty");
		return;
	}
	len = snprintf(buf, sizeof(buf), "%lu", duty1);
	write(fd, buf, len);
	close(fd);

	// write 2 rpm
	fd = open("/sys/class/pwm/pwm1/duty_ns", O_WRONLY);
	if (fd < 0) {
		perror("pwm1/duty");
		return;
	}
	len = snprintf(buf, sizeof(buf), "%lu", duty2);
	write(fd, buf, len);
	close(fd);

	// write 3 rpm
	fd = open("/sys/class/pwm/pwm2/duty_ns", O_WRONLY);
	if (fd < 0) {
		perror("pwm2duty");
		return;
	}
	len = snprintf(buf, sizeof(buf), "%lu", duty3);
	write(fd, buf, len);
	close(fd);

	// write 4 rpm
	fd = open("/sys/class/pwm/pwm3/duty_ns", O_WRONLY);
	if (fd < 0) {
		perror("pwm3/duty");
		return;
	}
	len = snprintf(buf, sizeof(buf), "%lu", duty4);
	write(fd, buf, len);
	close(fd);
	 return;
 }

 void start_pwms(){
	int fd, len;
	char buf[MAX_BUF];

	for(int i=0;i<4;i++){
	snprintf(buf, sizeof(buf), "/sys/class/pwm/pwm%d/run", i);
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("pwm/run");
		return;
	}

	len = snprintf(buf, sizeof(buf), "%d", 1);
	write(fd, buf, len);
	close(fd);
	}

	return;
 }

  void stop_pwms(){
	int fd, len;
	char buf[MAX_BUF];

	for(int i=0;i<4;i++){
	snprintf(buf, sizeof(buf), "/sys/class/pwm/pwm%d/run", i);
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("pwm/run");
		return;
	}

	len = snprintf(buf, sizeof(buf), "%d", 0);
	write(fd, buf, len);
	close(fd);
	}

	return;
 }
void init_escs(){
	export_pwms(); // export pwm0, pwm1, pwm2, pwm3
	set_period(); // 20ms for servos
	set_duties(1000000,1000000,1000000,1000000); //should be < period and in nano seconds
	                    // 1500000 nano seconds= 1.5ms(milli seconds) : center position of servo
	start_pwms();
	usleep(100000);
	set_duties(2500000,2500000,2500000,2500000);
	usleep(100000);
	set_duties(1000000,1000000,1000000,1000000);
	usleep(100000);
	duty1=(maxpwm-minpwm)/(maxForce-minForce);
}
void set_pwms(QuadStateVariable *QuadStates){



    	dutyF=Force2PwmMap(QuadStates->u[0]);
    	dutyL=Force2PwmMap(QuadStates->u[1]);
    	dutyB=Force2PwmMap(QuadStates->u[2]);
    	dutyR=Force2PwmMap(QuadStates->u[3]);


    set_duties(dutyF,dutyL,dutyB,dutyR);



}

unsigned long int  Force2PwmMap(double F){
	unsigned long int duty;

	if (F<minForce)
    	F=minForce;

    if (F>maxForce)
    	F=maxForce;

    duty=(-0.3115*pow(F,2)+14.79*F+115.7)*10000;

    	if(duty>maxpwm)
    		duty=maxpwm;
    	if(duty<minpwm)
    		duty=minpwm;

    return duty;

}
 };
#endif
