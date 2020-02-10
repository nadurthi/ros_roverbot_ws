/**
 * @file joystick.cpp
 * @brief Acquire data from joystick.\n
 * Copyright (c) 2013 Smart Laboratory at SUNY at Buffalo
 * This computer program includes confidential, proprietary
 * information and is a trade secret of the Author. All
 * use, disclosure, and/or reproduction is prohibited unless
 * expressly authorized in writing.
 * All rights reserved.
 */

#ifndef JOYSTICK_H_
#define JOYSTICK_H_


#include <pthread.h>
#include <linux/joystick.h>

#include <string>

void* update(void *ptr);

#define Stopbit 0


/// @brief Hold \b ALL joystick information
struct stjoystick{
	/// @brief Hold description of joystick.
	struct {
		/// amount of axis
		int axis;
		/// amount of buttons
		int button;
		/// Joystick name
		char name[256];
		/// File name for joystick only for linux
		char* file;
		/// IF only for windows
		int id;
	}info;
	/// Normalize all axis in range [-1 1]
	double axisNorm[11];
	/// State of each button press or unpress
	int button[39],buttonflags[11],axisflags[3];
	/// The joystick state is updated
	bool update;
	/// Which axis is throttle
	int thtl;

	/// Linux custom descriptions
	int fd;
	struct {
		pthread_t thread;
	}thread;
	js_event event;
	int axis[11];
	int axisOld[11];

};


///@brief Small and simple class to acquire joystick information
class joystick {
private:
public:
	/// Joystick information
	stjoystick joy;
	/**
	 *  Custom constructor
	 * @param file_
	 */
	joystick(std::string file_="/dev/input/js0");
	/**
	 * Initialize the device
	 * @return true if success
	 */
	bool start();
	double getaxis(int axisno);
	void getbuttons(int * Dbutt);
	/*
	 * if buttno >=11 clear all 11 button flags
	 */
	void ClearButtonFlag(int buttno);
	int GetButtonFlag(int buttno);
	bool CheckifEmergencyStopRequest();
	void ClearAxisFlag(int axisno);
	int GetAxisFlag(int axisno);
	void GetReMapAxis(double * Daxis,double * lb,double * ub);

	/*
	 * Re,ap the value D in [-1,1] to a value in [lu,ub] linearly
	 */
	double ReMapValue(double D, double lb, double ub);
	/**
	 * Initialize the device
	 * @param file_ set the joystick before start
	 * @return true if success
	 */
	bool checkif_eventoccured();
	bool checkif_Axiseventoccured();
	bool checkif_Buttoneventoccured();
	bool start(char* file_);
	/**
	 * Standard destructor
	 */
	virtual ~joystick();
};

#endif /* JOYSTICK_H_ */
