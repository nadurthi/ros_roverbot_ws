
#ifndef MOTORI2C_H_
#define MOTORI2C_H_

#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stropts.h>
#include <math.h>
#include <stdio.h>
#include <string>
#include <sys/types.h>




using namespace std;

#define I2CBUS 1


class MotorControl {
private:
  int i2cadd,mode,del,kp,minpos,rpm,i2cbus;
  float kd;
public:
  int write2i2c(int *);
  void setI2Caddress(int x){
  i2cadd=x;
  }
 void setI2Cbus(int x){
  i2cbus=x;
 }
  void setkp(int g){
	  int x[3];
	  x[0]=2;
	  x[1]=g/100;
	  x[2]=g-x[1]*100;
	  write2i2c(x);
	  kp=g;
  };
  void setkd(float g){
	  int x[3];
	  x[0]=6;
	  x[1]=g;
	  x[2]=(g-x[1])*100;
	  write2i2c(x);
	  kd=g;
  };
  void setminpos(int g){
	  int x[3];
	  x[0]=5;
	  x[1]=g/100;
	  x[2]=g-x[1]*100;
	  write2i2c(x);
	  minpos=g;
  };
  void setdelay(int d){
	  int x[3];
	  x[0]=1;
	  x[1]=d/100;
	  x[2]=d-x[1]*100;
	  write2i2c(x);
	  del=d;
  };
  void setspeed(int s){
	  int x[3];
	  x[0]=3;
	  x[1]=s/100;
	  x[2]=s-x[1]*100;
	  write2i2c(x);
	  rpm=s;
  }
  void setmode(int m){
	  int x[3];
	  x[0]=4;
	  x[1]=m/100;
	  x[2]=m-x[1]*100;
	  write2i2c(x);
	  mode=m;
  }
  int getspeed(){return(rpm);}
  int getmode(){return(mode);}
  int getkp(){return(kp);}
  float getkd(){return(kd);}
  int getminpos(){return(minpos);}
  int getdelay(){return(del);}
  int getI2Caddress(){return(i2cadd);}
  int getI2Cbus(){return(i2cbus); }


};


int MotorControl::write2i2c (int *x){
	  // x[0] is the switch
	  // x[1] and x[2] are the data
	  char namebuf[100];
	     	snprintf(namebuf, sizeof(namebuf), "/dev/i2c-%d", i2cbus);
	      int file;
	      if ((file = open(namebuf, O_RDWR)) < 0){
	              cout << "Failed to open BMA180 Sensor on " << namebuf << " I2C Bus" << endl;
	              return -1;
	      }
		      if (ioctl(file, I2C_SLAVE, i2cadd) < 0){
	              cout << "I2C_SLAVE address " << i2cadd << " failed..." << endl;
	              return -1;
	      }


	        char buffer[3];
	      	buffer[0] = x[0];
	      	buffer[1] = x[1];
	      	buffer[2] = x[2];

	      if ( write(file, buffer, 3) != 3) {
	          cout << "Failure to write values to I2C Device address." << endl;
	          return -1;
	      }
	      close(file);
	    return 0;

}










#endif
