/**
 * @file joystick.cpp
 * @brief Aquire data from joystick.\n
 * Copyright (c) 2013 Smart Laboratory at SUNY at Buffalo
 * This computer program includes confidential, proprietary
 * information and is a trade secret of the Author. All
 * use, disclosure, and/or reproduction is prohibited unless
 * expressly authorized in writing.
 * All rights reserved.
 */


#include "joystick.h"
#include <stdio.h>
#include <iostream>
#include <fcntl.h>

#include <unistd.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <pthread.h>


#include <stdlib.h>
using namespace std;


//===========================================================================
//===========================================================================
void* update(void *ptr){
	stjoystick* joy=static_cast<stjoystick*>(ptr);
	while(true){
		joy->update=false;
		read(joy->fd, &joy->event, sizeof(struct js_event));
				/* see what to do with the event */
			switch (joy->event.type & ~JS_EVENT_INIT)
			{

				case JS_EVENT_AXIS:
					if(	abs((joy->axisOld[ joy->event.number])-(joy->event.value)) >= 10){
						joy->axis   [ joy->event.number ] = joy->event.value;
						joy->axisOld[ joy->event.number]= joy->event.value;

						if(joy->event.number==joy->thtl){
							joy->axisNorm [ joy->event.number ] = -double (joy->event.value/32767.);
						}
						else{
							joy->axisNorm [ joy->event.number ] = -double (joy->event.value/32767.);
						}
						joy->axisflags[joy->event.number ]=1; //set the flag

						joy->update=true;
cout<<"Axis = "<<joy->axisNorm[2]<<endl;
					}
					break;
				case JS_EVENT_BUTTON:
					joy->button [ joy->event.number ] = joy->event.value;
					if (joy->button [ joy->event.number ]==1)
						joy->buttonflags[joy->event.number]=1;

					joy->update=true;
					break;
				default:
					break;
			}
			usleep(100);
	}
	printf("while loop end\n");
	return 0;
}
double joystick::getaxis(int axisno){
	joy.axisflags[axisno]=0; //clear the flag for this axis
	return joy.axisNorm[axisno];

}


void joystick::getbuttons(int * Dbutt){
	for(int i=0;i<11;i++)
	Dbutt[i]=joy.button[i];

}
bool joystick::checkif_eventoccured(){
	int i=0,j=0;
	for(int k=0;k<3;k++)
		i=i+joy.axisflags[k];

	for(int k=0;k<11;k++)
			j=j+joy.buttonflags[k];


	if(i>0 || j>0)
		return true;
	else
		return false;
}
bool joystick::checkif_Axiseventoccured(){
	int i=0;
	for(int k=0;k<3;k++)
		i=i+joy.axisflags[k];

	if(i>0)
		return true;
	else
		return false;
}
bool joystick::checkif_Buttoneventoccured(){
	int i=0;
	for(int k=0;k<11;k++)
		i=i+joy.buttonflags[k];

	if(i>0)
		return true;
	else
		return false;
}

void joystick::ClearButtonFlag(int buttno){
	if(buttno >=11)
	{
		for(int i=0;i<11;i++)
			joy.buttonflags[i]=0;
	}
	else
		joy.buttonflags[buttno]=0;

}
void joystick::ClearAxisFlag(int axisno){
	if(axisno >=3)
	{		for(int i=0;i<3;i++)
		joy.axisflags[i]=0;
	}
	else
		joy.axisflags[axisno]=0;

}
int joystick::GetAxisFlag(int axisno){

		return joy.axisflags[axisno];

}

int joystick::GetButtonFlag(int buttno){
	return joy.buttonflags[buttno];
}
bool joystick::CheckifEmergencyStopRequest(){
	if(joy.buttonflags[Stopbit]==1){
		joy.buttonflags[Stopbit]=0; //clear the flag
		return true;
	}
	else
		return false;
}

double joystick::ReMapValue(double D, double lb, double ub){
return (lb+ub)/2+0.5*(ub-lb)*D;

}

//===========================================================================
//===========================================================================
joystick::joystick(std::string file_) {
	joy.info.file=(char*)file_.c_str();
	joy.update=false;

}

bool joystick::start(char* file_) {
	joy.info.file=file_;
	return start();
}
//=========================================================================

//========================================================================


//===========================================================================
//===========================================================================
joystick::~joystick() {


}
//===========================================================================
//===========================================================================

bool joystick::start(){
joy.update=false;

#ifdef __unix__
sleep(1);
for(int i=0;i<11;i++)
	joy.buttonflags[i]=0;

	joy.fd = open( joy.info.file , O_RDONLY);
	if( joy.fd == -1 ){
		cout<<"error in fd open"<<endl;
		return false;
	}
	ioctl( joy.fd, JSIOCGAXES, &joy.info.axis );
	ioctl( joy.fd, JSIOCGBUTTONS, &joy.info.button );
	ioctl( joy.fd, JSIOCGNAME(80), &joy.info.name );
	fcntl( joy.fd, F_SETFL, O_NONBLOCK );
	int ret=pthread_create( &joy.thread.thread, NULL, update, (void*) &(this->joy));
	if(ret!=0){
		cout<<"error in pthread open"<<endl;
		return false;
	}
#endif

#ifdef _MSC_VER
		printf("Opening joystick...");
		int devs=joyGetNumDevs();
		if(devs!=0) {
			JOYCAPS  pjc;
			joyGetDevCaps( joy.info.id,  &pjc,  sizeof(pjc));
			joy.info.axis=pjc.wNumAxes;
			if(joy.info.axis==0){
					TXT_FAIL_NL;
			TXT_ERROR;
			printf( "No joystick detected\n" );
			TXT_WHITE;
			return false;
			}
			TXT_OK_NL;
			joy.info.button=pjc.wNumButtons;
			joy.info.button=4;//WINDOWS SIMPLE API ONLY HANDLE 4 BUTTONS.
			for(int i=0;i<32;i++)joy.info.name[i]=(char)pjc.szPname[i];
			printf("Starting Joystick thread...");
			uintptr_t thread=_beginthread( update, 0, (void*)  &(this->joy) );
			if(thread!=NULL){
				TXT_OK;
			}
			else TXT_FAIL;
				printf("\n");
		}
		else{
			TXT_FAIL_NL;
			TXT_ERROR;
			printf( "No joystick driver detected\n" );
			TXT_WHITE;
			return false;
		}
#endif

		joy.thtl=2;
		if(joy.info.axis==6)joy.thtl=3;
	return true;
}
//===========================================================================
//===========================================================================
