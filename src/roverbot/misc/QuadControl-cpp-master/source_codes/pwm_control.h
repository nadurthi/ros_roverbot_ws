#ifndef PWMCONTROL_H_
#define PWMCONTROL_H_

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

#define MAX_BUF 64

class PWM {
	 unsigned long int duty1,period;
public:
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
void rpm2pwm_run(double *frpm){
	//frpm is order Front,left,back,right (computed from theory control)

	//pwm set duties in order: front right,back left
	set_duties(0.0066*pow(frpm[0],2)+44.69*frpm[0]+998600,106.2*frpm[3]+1027000,0.0066*pow(frpm[2],2)+44.69*frpm[2]+998600,106.2*frpm[1]+1027000);
}
 };
/* USAGE--------------------------------------------------
  PWM Motor_pwm;
    // assuming that the overlay has already been manually loaded
    	Motor_pwm.export_pwms(); // export pwm0, pwm1, pwm2, pwm3
    	Motor_pwm.set_period(); // 20ms for servos
    	Motor_pwm.set_duties(1000000,1000000,1000000,1000000); //should be < period and in nano seconds
    	// 1500000 nano seconds= 1.5ms(milli seconds) : center position of servo
    	//Arming the ESCs
    	Motor_pwm.start_pwms();
    	usleep(100000);
    	Motor_pwm.set_duties(2500000,2500000,2500000,2500000);
    	usleep(100000);
    	Motor_pwm.set_duties(1000000,1000000,1000000,1000000);
    	usleep(100000);
  */
 */
#endif
