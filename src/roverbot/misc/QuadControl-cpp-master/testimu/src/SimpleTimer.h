/*
 * SimpleTimer.h
 *
 *  Created on: Mar 30, 2014
 *      Author: nagnanamus
 */

#ifndef SIMPLETIMER_H_
#define SIMPLETIMER_H_

#include <time.h>
#include <sys/time.h>
class SimpleTimer {
private:
	double Tsec,T0sec,Tmax;
	struct timeval tv;
	struct timezone tz;
public:
	SimpleTimer(){
		Tmax=0;
		T0sec=0;
		Tsec=0;
	}
	void ResetTimer(){
		gettimeofday(&tv,&tz);
		T0sec=(double)tv.tv_sec+(double)(tv.tv_usec)/1000000;
	}
	void SetTsec(double tsec){
		Tsec=tsec;
	}
	/*
	 *  returns false if it crosses the set timer value
	 *  so use it as while(CheckTimer()){}
	 */
	bool CheckTimer(){
		double tt;
		gettimeofday(&tv,&tz);
		tt=(double)tv.tv_sec+(double)(tv.tv_usec)/1000000;
		if ((tt-T0sec)>=Tsec)
		{return false;}
		else
		{return true;}
	}

	double GetTime_from_T0sec(){
		double tt;
		gettimeofday(&tv,&tz);
		tt=(double)tv.tv_sec+(double)(tv.tv_usec)/1000000;
	    return (tt-T0sec);
	}
	/*
	 * This function reads the current time from T0sec and updates the max time
	 *  This is to keep track of max wall time when called
	 */
	double UpdateTmax_from_T0sec(){
		double tt;
		gettimeofday(&tv,&tz);
		tt=(double)tv.tv_sec+(double)(tv.tv_usec)/1000000;
	    tt= tt-T0sec;

	    if (tt>Tmax)
	    Tmax=tt;

	    return Tmax;
	}
};


#endif /* SIMPLETIMER_H_ */
