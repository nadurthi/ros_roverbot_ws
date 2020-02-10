///////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) OMG Plc 2009.
// All rights reserved.  This software is protected by copyright
// law and international treaties.  No part of this software / document
// may be reproduced or distributed in any form or by any means,
// whether transiently or incidentally to some other use of this software,
// without the written permission of the copyright owner.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef _VICONMODULE_H_
#define _VICONMODULE_H_


#include "Client.h"
#include <iostream>
#include <string>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>

using namespace std;


using namespace ViconDataStreamSDK::CPP;

namespace
{
  std::string Adapt( const bool i_Value )
  {
    return i_Value ? "True" : "False";
  }

  std::string Adapt( const Direction::Enum i_Direction )
  {
    switch( i_Direction )
    {
      case Direction::Forward:
        return "Forward";
      case Direction::Backward:
        return "Backward";
      case Direction::Left:
        return "Left";
      case Direction::Right:
        return "Right";
      case Direction::Up:
        return "Up";
      case Direction::Down:
        return "Down";
      default:
        return "Unknown";
    }
  }

  std::string Adapt( const DeviceType::Enum i_DeviceType )
  {
    switch( i_DeviceType )
    {
      case DeviceType::ForcePlate:
        return "ForcePlate";
      case DeviceType::Unknown:
      default:
        return "Unknown";
    }
  }

  std::string Adapt( const Unit::Enum i_Unit )
  {
    switch( i_Unit )
    {
      case Unit::Meter:
        return "Meter";
      case Unit::Volt:
        return "Volt";
      case Unit::NewtonMeter:
        return "NewtonMeter";
      case Unit::Newton:
        return "Newton";
      case Unit::Unknown:
      default:
        return "Unknown";
    }
  }

}

class Vicon_data {

	  string Quadname,Marker1,Marker2,Marker3,Marker4,Marker5,Marker6,Marker7,Marker8,Marker9;
	  Client MyClient;
	  double Data[8]; //[t,x,y,z,q0,q1,q2,q3] t is from start
	  std::string HostName;
		struct timeval tv;
		struct timezone tz;
		double Tsec,T0sec;
public:
	  void initialize(int delay){
		  Quadname="ryan";
		  	  	  Marker1="backup";
		 		  Marker2="rightdown";
		 		  Marker3="frontdown2";
		 		  Marker4="frontdown1";
		 		  Marker5="rightup";
		 		  Marker6="leftup";
		 		  Marker7="leftdown";
		 		  Marker8="backdown";
		 		  Marker9="center";




		  HostName="10.0.0.1";
		  bool TransmitMulticast = false;
		  while( !MyClient.IsConnected().Connected )
		      { MyClient.Connect( HostName );
		        sleep(delay);
		      }
		  MyClient.EnableSegmentData();
		  MyClient.EnableMarkerData();
		  MyClient.EnableUnlabeledMarkerData();
		  MyClient.EnableDeviceData();

		  //MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ServerPush );
		  //MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPullPreFetch );
		MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPull );

		  MyClient.SetAxisMapping( Direction::Forward,Direction::Left,Direction::Up );

		  //read some values to stabilize
		  for(int i=0;i<25;i++){
			  while( MyClient.GetFrame().Result != Result::Success )
			 		        { sleep(1);}
			  usleep(9000);//sleep for 9 ms
		  }
		  gettimeofday(&tv,&tz);
		  T0sec=(double)tv.tv_sec+(double)(tv.tv_usec)/1000000;
		  cout<<"ready"<<endl;
	  }


	  /*
	   * Get the vicon body  data and the marker data too
	   */
	  double * GetData(double * X,double * MarkerX){
		  while( MyClient.GetFrame().Result != Result::Success )
		        { sleep(1);}
		  //timestamping
		  gettimeofday(&tv,&tz);
		  Tsec=(double)tv.tv_sec+(double)(tv.tv_usec)/1000000-T0sec;

	unsigned int SubjectCount = MyClient.GetSubjectCount().SubjectCount;
	for( unsigned int SubjectIndex = 0 ; SubjectIndex < SubjectCount ; ++SubjectIndex )
	{
		           // Get the subject name
		          std::string SubjectName = MyClient.GetSubjectName( SubjectIndex ).SubjectName;
		          if(SubjectName.compare(Quadname)!=0){
		          	continue;
		          }

		          // Count the number of segments
		          unsigned int SegmentCount = MyClient.GetSegmentCount( SubjectName ).SegmentCount;

		          for( unsigned int SegmentIndex = 0 ; SegmentIndex < SegmentCount ; ++SegmentIndex )
		          {
		            // Get the segment name
		            std::string SegmentName = MyClient.GetSegmentName( SubjectName, SegmentIndex ).SegmentName;

		            if(SegmentName.compare(Quadname)!=0){
		            	continue;
		            }

                    X[0]=Tsec;
                    Output_GetSegmentGlobalTranslation _Output_GetSegmentGlobalTranslation = MyClient.GetSegmentGlobalTranslation( SubjectName, SegmentName );
                    X[1]=_Output_GetSegmentGlobalTranslation.Translation[ 0 ]/1000;
                    X[2]=_Output_GetSegmentGlobalTranslation.Translation[ 1 ]/1000;
                    X[3]=_Output_GetSegmentGlobalTranslation.Translation[ 2 ]/1000;

                    Output_GetSegmentGlobalRotationQuaternion _Output_GetSegmentGlobalRotationQuaternion =MyClient.GetSegmentGlobalRotationQuaternion( SubjectName, SegmentName );
                    X[4]=_Output_GetSegmentGlobalRotationQuaternion.Rotation[ 3 ];
                    X[5]=_Output_GetSegmentGlobalRotationQuaternion.Rotation[ 0 ];
                    X[6]=_Output_GetSegmentGlobalRotationQuaternion.Rotation[ 1 ];
                    X[7]=_Output_GetSegmentGlobalRotationQuaternion.Rotation[ 2 ];
		           }


                  // Count the number of markers for this subject
                  unsigned int MarkerCount = MyClient.GetMarkerCount( SubjectName ).MarkerCount;
                 // std::cout << "    Markers (" << MarkerCount << "):" << std::endl;
                  for( unsigned int MarkerIndex = 0 ; MarkerIndex < MarkerCount ; ++MarkerIndex )
                  {
                    // Get the marker name
                    std::string MarkerName = MyClient.GetMarkerName( SubjectName, MarkerIndex ).MarkerName;

                    // Get the marker parent
                    std::string MarkerParentName = MyClient.GetMarkerParentName( SubjectName, MarkerName ).SegmentName;

                    // Get the global marker translation
                    Output_GetMarkerGlobalTranslation _Output_GetMarkerGlobalTranslation =
                      MyClient.GetMarkerGlobalTranslation( SubjectName, MarkerName );
                    if(MarkerName.compare(Marker1)==0){

                    	MarkerX[0]=_Output_GetMarkerGlobalTranslation.Translation[ 0 ]/1000;
                    	MarkerX[1]=_Output_GetMarkerGlobalTranslation.Translation[ 1 ]/1000;
                    	MarkerX[2]=_Output_GetMarkerGlobalTranslation.Translation[ 2 ]/1000;
                    }

                    if(MarkerName.compare(Marker2)==0){

                    	MarkerX[3]=_Output_GetMarkerGlobalTranslation.Translation[ 0 ]/1000;
                    	MarkerX[4]=_Output_GetMarkerGlobalTranslation.Translation[ 1 ]/1000;
                    	MarkerX[5]=_Output_GetMarkerGlobalTranslation.Translation[ 2 ]/1000;
                    }
                    if(MarkerName.compare(Marker3)==0){

                    	MarkerX[6]=_Output_GetMarkerGlobalTranslation.Translation[ 0 ]/1000;
                    	MarkerX[7]=_Output_GetMarkerGlobalTranslation.Translation[ 1 ]/1000;
                    	MarkerX[8]=_Output_GetMarkerGlobalTranslation.Translation[ 2 ]/1000;
                    		                    }

                     if(MarkerName.compare(Marker4)==0){

                    	 MarkerX[9]=_Output_GetMarkerGlobalTranslation.Translation[ 0 ]/1000;
                    	 MarkerX[10]=_Output_GetMarkerGlobalTranslation.Translation[ 1 ]/1000;
                    	 MarkerX[11]=_Output_GetMarkerGlobalTranslation.Translation[ 2 ]/1000;
                                                }
                     if(MarkerName.compare(Marker5)==0){

                    	 MarkerX[12]=_Output_GetMarkerGlobalTranslation.Translation[ 0 ]/1000;
                    	 MarkerX[13]=_Output_GetMarkerGlobalTranslation.Translation[ 1 ]/1000;
                    	 MarkerX[14]=_Output_GetMarkerGlobalTranslation.Translation[ 2 ]/1000;
                                                 }
                     if(MarkerName.compare(Marker6)==0){

                    	 MarkerX[15]=_Output_GetMarkerGlobalTranslation.Translation[ 0 ]/1000;
                    	 MarkerX[16]=_Output_GetMarkerGlobalTranslation.Translation[ 1 ]/1000;
                    	 MarkerX[17]=_Output_GetMarkerGlobalTranslation.Translation[ 2 ]/1000;
                                                 		                    }

                     if(MarkerName.compare(Marker7)==0){

                    	 MarkerX[18]=_Output_GetMarkerGlobalTranslation.Translation[ 0 ]/1000;
                    	 MarkerX[19]=_Output_GetMarkerGlobalTranslation.Translation[ 1 ]/1000;
                    	 MarkerX[20]=_Output_GetMarkerGlobalTranslation.Translation[ 2 ]/1000;
                                                                             }

                     if(MarkerName.compare(Marker8)==0){

                    	 MarkerX[21]=_Output_GetMarkerGlobalTranslation.Translation[ 0 ]/1000;
                    	 MarkerX[22]=_Output_GetMarkerGlobalTranslation.Translation[ 1 ]/1000;
                    	 MarkerX[23]=_Output_GetMarkerGlobalTranslation.Translation[ 2 ]/1000;
                                                                             }
                     if(MarkerName.compare(Marker9)==0){

                    	 MarkerX[24]=_Output_GetMarkerGlobalTranslation.Translation[ 0 ]/1000;
                    	 MarkerX[25]=_Output_GetMarkerGlobalTranslation.Translation[ 1 ]/1000;
                    	 MarkerX[26]=_Output_GetMarkerGlobalTranslation.Translation[ 2 ]/1000;
                                                                             }
                    /*
                     *  std::cout << "      Marker #" << MarkerIndex            << ": "
                                                  << MarkerName             << " ("
                                                  << _Output_GetMarkerGlobalTranslation.Translation[ 0 ]  << ", "
                                                  << _Output_GetMarkerGlobalTranslation.Translation[ 1 ]  << ", "
                                                  << _Output_GetMarkerGlobalTranslation.Translation[ 2 ]  << ") "
                                                  << Adapt( _Output_GetMarkerGlobalTranslation.Occluded ) << std::endl;
                  */
                  }

     }

		  return X;

	  }

	  void SetT0sec(){
		  gettimeofday(&tv,&tz);
		  T0sec=(double)tv.tv_sec+(double)(tv.tv_usec)/1000000;

	  }
	  void disconnect(){
		  MyClient.DisableSegmentData();
		      MyClient.DisableMarkerData();
		      MyClient.DisableUnlabeledMarkerData();
		      MyClient.DisableDeviceData();

		      MyClient.Disconnect();
	  }

};

#endif
