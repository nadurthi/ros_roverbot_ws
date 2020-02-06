//============================================================================
// Name        : joystick_test.cpp
// Author      : Nagnanamus
// Version     :
// Copyright   : Do what ever you want!!! I am not responsible
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <unistd.h>
#include "joystick.h"
#include <stdlib.h>
using namespace std;

int main() {
	cout << "!!!Hello World!!!" << endl; // prints !!!Hello World!!!


	joystick joystick;
	if (joystick.start()==false)
		cout<<"error with start"<<endl;

	cout<<"No of axis = "<<joystick.joy.info.axis<<endl;
	cout<<"No of buttons = "<<joystick.joy.info.button<<endl;
	cout<<"Name = "<<joystick.joy.info.name<<endl;
double Daxis[3],Dkq;
int Dbutt[11];
cout<<"Entering while"<<endl;
int i=0;
joystick.ClearButtonFlag(15);
	while(1){
		if(joystick.CheckifEmergencyStopRequest()==true){
				cout<<"Emergency Stop Requested : exit"<<endl;
				exit(1);
				//return 1;
			}

		if(joystick.GetAxisFlag(2)==1){ //kq axis is set
						Dkq=joystick.getaxis(2);
						Dkq=joystick.ReMapValue(Dkq,25,50);

						joystick.ClearAxisFlag(2);
cout<<"kq = "<<Dkq<<endl;

					}
		joystick.getbuttons(Dbutt);
		cout<<Dbutt[0]<<" "<<Dbutt[1]<<" "<<Dbutt[2]<<" "<<Dbutt[3]<<" "<<Dbutt[4]<<" "<<Dbutt[5]<<" "<<Dbutt[6]<<" "<<Dbutt[7]<<" "<<Dbutt[8]<<" "<<Dbutt[9]<<" "<<Dbutt[10]<<endl;
usleep(100000);
	}
	return 0;
}
