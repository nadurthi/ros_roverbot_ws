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
#include <math.h>
using namespace std;

#define minpwm 10000  //times 100
#define maxpwm 24000  //times 100
#define MAX_BUF 64

class PWMmotors {
	 unsigned long int duty1,dutyF,dutyL,dutyB,dutyR,period;
	 int maxForce,minForce;

public:
	 void set_max_min_force(int Fmin,int Fmax){
		 minForce=Fmin;
		 maxForce=Fmax;
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
void set_pwms(double * fpwm,double * frpm){

	if (frpm[0]<minForce)
		frpm[0]=minForce;

	if (frpm[0]>maxForce)
		frpm[0]=maxForce;


	dutyF=(frpm[0]*duty1+minpwm)*100;

////////////////////////////////////
	if (frpm[1]<minForce)
			frpm[1]=minForce;

    if (frpm[1]>maxForce)
			frpm[1]=maxForce;

    dutyL=(frpm[1]*duty1+minpwm)*100;
//////////////////////////////////////
    if (frpm[2]<minForce)
    		frpm[2]=minForce;

    if (frpm[2]>maxForce)
    		frpm[2]=maxForce;

    dutyB=(frpm[2]*duty1+minpwm)*100;
//////////////////////////
    if (frpm[3]<minForce)
    			frpm[3]=minForce;

    if (frpm[3]>maxForce)
    			frpm[3]=maxForce;

    dutyR=(frpm[3]*duty1+minpwm)*100;

   // cout<<dutyF<<endl;
   // cout<<dutyL<<endl;
   // cout<<dutyB<<endl;
   // cout<<dutyR<<endl;
    fpwm[0]=dutyF;
    fpwm[1]=dutyL;
    fpwm[2]=dutyB;
    fpwm[3]=dutyR;
    set_duties(dutyF,dutyL,dutyB,dutyR);

}
 };
#endif
