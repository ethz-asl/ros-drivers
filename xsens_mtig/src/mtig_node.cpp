/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THI
*  iS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*  This code is an improvement of two programs: 
*  mtig_node build by Gonçalo Cabrita on 08/11/2010
*  example_linux furnished with cmt library
*
* Author: Benoit LESCOT on 29/04/2011
*********************************************************************/



#include "cmtsrc/cmtdef.h"
#include "cmtsrc/xsens_time.h"
#include "cmtsrc/xsens_list.h"
#include "cmtsrc/cmtscan.h"
#include "cmtsrc/cmt3.h"
#include "cmtsrc/example_linux.h"
#include "cmtsrc/cmt2.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <xsens_mtig/Thermistor.h>
#include <xsens_mtig/GPSInfoStatus2.h>
#include <xsens_mtig/GPSInfoStatus.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <cmath>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>

#include <stdio.h>
#include <stdlib.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/publisher.h>

// this macro tests for an error and exits the program with a message if there was one
#define EXIT_ON_ERROR(res,comment) if (res != XRV_OK) { ROS_INFO("Error %d occurred in " comment ": %s\n",res,xsensResultText(res)); exit(1); }


// Converter fonction from integer to string /////////////////////////////////////////////////////////
std::string int2str ( int number )
{
  std::ostringstream oss;

  // Works just like cout
  oss<< number;

  // Return the underlying string
  return oss.str();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////




using namespace std;
using namespace xsens;


                                 

int main(int argc, char* argv[])
{
	(void) argc; (void) argv;	// Make the compiler stop complaining about unused parameters
	
	
	//XSENS Params
	xsens::Cmt3 cmt3;
	xsens::Cmt2s cmt2;
	unsigned long mtCount = 0;
	CmtDeviceId deviceIds[256];

	XsensResultValue res = XRV_OK;

		
	
	//ROS
	ros::init(argc, argv, "mtig_node");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	
	// Params
	std::string portname;
	int baudrate;
	int temp_init=1;
	double tdata;
	double min_temp=99, max_temp=99;
	int nb_sat_OK=0;
		
	std::string frame_id;
	pn.param<std::string>("port", portname, "/dev/serial/by-id/usb-Xsens_Xsens_USB-serial_converter_XSRF9M1S-if00-port0");
	pn.param("baudrate", baudrate, 115200);
	pn.param<std::string>("frame_id", frame_id, "/base_imu");
	
	

	//Topics definition
	ros::Publisher mti_pub = n.advertise<sensor_msgs::Imu>("imu/data", 10);
        ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("pos_nav", 10);
   	ros::Publisher vel_pub = n.advertise<geometry_msgs::TwistStamped>("velocity",10);
   	ros::Publisher magn_pub = n.advertise<geometry_msgs::Vector3Stamped>("magnetic",10);
	ros::Publisher temp_pub = n.advertise<xsens_mtig::Thermistor>("temperature",10);
	ros::Publisher gpsstatus_pub = n.advertise<xsens_mtig::GPSInfoStatus2>("info_gps",10);
	ros::Publisher gravity_pub = n.advertise<geometry_msgs::Vector3Stamped>("gravity",10);
	ros::Publisher status_pub = n.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics_agg",10);
	
	//diagnostic_updater::updater<diagnostic_msgs::DiagnosticArray> diag_pub_;
	//diagnostic_updater::Updater updater;

	ros::Rate r(20);
	
	CmtOutputMode mode;
	CmtOutputSettings settings;

		
	// Perform hardware scan
	mtCount = doHardwareScan(cmt3, deviceIds);
	
	
	// Set user input settings
	getUserInputs(mode, settings);
	// Set device to user input settings
	doMtSettings(cmt3, mode, settings, deviceIds);

	
	//structs to hold data.
	CmtCalData caldata;
	CmtQuat qat_data;
	CmtEuler euler_data;
	CmtMatrix matrix_data;


	while(ros::ok())
	{
		//Building Messages
		sensor_msgs::Imu mti_msg;
    sensor_msgs::NavSatFix gps_msg;
    xsens_mtig::GPSInfoStatus2 gpsstatus_msg;
    geometry_msgs::TwistStamped vel_msg;
    geometry_msgs::Vector3Stamped magn_msg;
    xsens_mtig::Thermistor temp_msg;
    geometry_msgs::Vector3Stamped gravity_msg;
    diagnostic_msgs::DiagnosticArray status_msg;
      
      
      
      
// Initialize packet for data
Packet* packet = new Packet((unsigned short) mtCount, cmt3.isXm());

while( (ros::ok()) && (res == XRV_OK))
{
cmt3.waitForDataMessage(packet);
    
  for (unsigned int i = 0; i < mtCount; i++) 
    {
      // Output Temperature
      if ((mode & CMT_OUTPUTMODE_TEMP) != 0) 
      {	
        if (packet->containsTemp()) 
        {				
          
          
          tdata = packet->getTemp(i);
          temp_msg.reading = tdata;
        
            //Min/Max temperature
            if (temp_init==1)
            {
            temp_msg.max_reading = tdata;
            temp_msg.min_reading = tdata;
            min_temp=tdata;
            max_temp=tdata;
            temp_init ++;
            }
        
            else
            {
              if (tdata>max_temp)
              {
              temp_msg.max_reading = tdata;
              }
              if (tdata<min_temp)
              {
              temp_msg.min_reading = tdata;
              }
            
            }
          
        }	
        else
        {
        ROS_ERROR("No Temperature data available\n");
        }					
      }
      
      // Output Calibrated (Acc, Ang velocity, Mag)
      if ((mode & CMT_OUTPUTMODE_CALIB) != 0) 
      {					
        if (packet->containsCalData()) 
        {
        caldata = packet->getCalData(i);

          //Linear Acceleration
          if (packet->containsCalAcc()) 
          {
          mti_msg.linear_acceleration.x = caldata.m_acc.m_data[0];
          mti_msg.linear_acceleration.y = caldata.m_acc.m_data[1];
          mti_msg.linear_acceleration.z = caldata.m_acc.m_data[2];
          }
          else
          {
          ROS_ERROR("No acceleration data available\n");
          }

          //Angular velocity
          if (packet->containsCalGyr()) 
          {
          mti_msg.angular_velocity.x =caldata.m_gyr.m_data[0] ;
          mti_msg.angular_velocity.y =caldata.m_gyr.m_data[1];
          mti_msg.angular_velocity.z =caldata.m_gyr.m_data[2];
          
          vel_msg.twist.angular.x = caldata.m_gyr.m_data[0];
                vel_msg.twist.angular.y = caldata.m_gyr.m_data[1];
                vel_msg.twist.angular.z = caldata.m_gyr.m_data[2];						
          }
          
          else
          {
          ROS_ERROR("No gyroscope data available\n");
          }	
          
          //Magnetic field
          if (packet->containsCalMag()) 
          {
          magn_msg.vector.x = caldata.m_mag.m_data[0];
              magn_msg.vector.y = caldata.m_mag.m_data[1];
              magn_msg.vector.z = caldata.m_mag.m_data[2];
          }
          
          else
          {
          ROS_ERROR("No Magnetic data available\n");
          }

        }
        
        else
        {
        ROS_ERROR("No calibrate data available\n");
        }
        
      }

      //Output Orientation
      
      if ((mode & CMT_OUTPUTMODE_ORIENT) != 0)
      {
                      
        if (packet->containsOri()) 
        {
            
          switch (settings & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) 
          {
          case CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION:
          {
          // Output: quaternion
          qat_data = packet->getOriQuat(i);
          mti_msg.orientation.w =qat_data.m_data[0];
          mti_msg.orientation.x =qat_data.m_data[1];
          mti_msg.orientation.y =qat_data.m_data[2];
          mti_msg.orientation.z =qat_data.m_data[3];
                      
          //Conversion to gravity vector
          double q0,q1,q2,q3;
          q0=mti_msg.orientation.w;
          q1=mti_msg.orientation.x;
          q2=mti_msg.orientation.y;
          q3=mti_msg.orientation.z;
          
          double norme2=sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
          #define pi 3.1415926535897931
          
          
          //PRINT EULER ANGLES
          /*double Roll = double(atan2((2*(q0*q1+q2*q3)),(1-2*(q1*q1+q2*q2))));
          double Pitch = double(asin(2*(q0*q2-q1*q3)));
          double Yaw = double(atan2((2*(q0*q3+q1*q2)),(1-2*(q2*q2+q3*q3))));
          
          ROS_INFO("Roll: %f ;\t Pitch : %f ;\t Yaw :%f;\n", Roll*180/pi,Pitch*180/pi,Yaw*180/pi);
          */
          double q0n,q1n,q2n,q3n;
          q0n=q0/norme2;
          q1n=q1/norme2;
          q2n=q2/norme2;
          q3n=q3/norme2;
          
          //Apply matrix rotation to [0,0,-1]
          double x,y,z;
          x=-2*(q1n*q3n-q0n*q2n);
          y=-2*(q2n*q3n+q0n*q1n);
          z=-(1-2*q1n*q1n-2*q2n*q2n);
          
          double norme=sqrt(x*x+y*y+z*z);
          gravity_msg.vector.x = x/norme;
              gravity_msg.vector.y = y/norme;
              gravity_msg.vector.z = z/norme;
              }
          break;

          case CMT_OUTPUTSETTINGS_ORIENTMODE_EULER:
          {
          // Output: Euler
          euler_data = packet->getOriEuler(i);
          /*ROS_INFO(yt++, xt, "%6.1f\t%6.1f\t%6.1f", euler_data.m_roll, 
              euler_data.m_pitch, euler_data.m_yaw);*/
          }		
          break;

          case CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX:
          {
          // Output: Cosine Matrix
          matrix_data = packet->getOriMatrix(i);
          /*ROS_INFO(yt++, xt, "%6.3f\t%6.3f\t%6.3f", matrix_data.m_data[0][0], 
              matrix_data.m_data[0][1], matrix_data.m_data[0][2]);
          ROS_INFO(yt++, xt, "%6.3f\t%6.3f\t%6.3f\n", matrix_data.m_data[1][0], 
              matrix_data.m_data[1][1], matrix_data.m_data[1][2]);
          ROS_INFO(yt++, xt, "%6.3f\t%6.3f\t%6.3f\n", matrix_data.m_data[2][0], 
              matrix_data.m_data[2][1], matrix_data.m_data[2][2]);*/
          break;
          }
          }
        }
        
        else
        {
          ROS_ERROR("No Orientation data available\n");
        }
      }

      /* Output position */
      if ((mode & CMT_OUTPUTMODE_POSITION) != 0) 
      {
        if (packet->containsPositionLLA()) 
        {				
          
          CmtVector positionLLA = packet->getPositionLLA();
            
          gps_msg.latitude = positionLLA.m_data[0];
                      gps_msg.longitude = positionLLA.m_data[1];
                      gps_msg.altitude = positionLLA.m_data[2];
                      gps_msg.status.service = gps_msg.status.SERVICE_GPS;									
        } 
        
        else 
        {
          ROS_ERROR("No position data available\n");
        }
    
        /*REQUEST GPS*/
                          
        CmtGpsStatus gps_dataref;
        CmtGpsStatus& gps_data=gps_dataref;
        
        //Timeout setting for GPS Status
        res=cmt3.setTimeout(50);	
        if (res!= XRV_OK)
        {	ROS_ERROR("error %d SetTimeout", res);
        
        }
        
        else
        {
          res=cmt3.getGpsStatus(gps_data,*deviceIds);							
          if (res != XRV_OK)
          {
            ROS_ERROR("error %d GPS", res);
                      
          }
         
          else 
          {
          //ROS_INFO("Donnees GPS enregistrees \n");
          
            for (int i = 0; i < CMT_MAX_SVINFO; i++)
          
            {	
              gpsstatus_msg.gps[i].m_id = gps_data.m_svInfo[i].m_id;
              gpsstatus_msg.gps[i].m_navigationStatus=gps_data.m_svInfo[i].m_navigationStatus;
              gpsstatus_msg.gps[i].m_signalQuality=gps_data.m_svInfo[i].m_signalQuality;
              gpsstatus_msg.gps[i].m_signalStrength=gps_data.m_svInfo[i].m_signalStrength;
              if (gps_data.m_svInfo[i].m_signalQuality == 7)						
              {
              nb_sat_OK ++;
              }
              gpsstatus_msg.m_nb_sat=nb_sat_OK;
            }	
          
          }
        }

      /* OUTPUT VELOCITY */
      if ((mode & CMT_OUTPUTMODE_VELOCITY) != 0) 
      {
        if (packet->containsVelocity()) 
        {				
        CmtVector Velocity = packet->getVelocity();

        vel_msg.twist.linear.x = Velocity.m_data[0];
              vel_msg.twist.linear.y = Velocity.m_data[1];
              vel_msg.twist.linear.z = Velocity.m_data[2];
                          
        } 
          
        else 
        {
          ROS_ERROR("No velocity data available\n");
        }

         }
          
          
        }
          
        //OUPUT STATUS DATA AND DIAGNOSTIC
      if ((mode & CMT_OUTPUTMODE_STATUS) != 0) 
      {
        if (packet->containsStatus()) 
        {				
        unsigned int statusGPS = packet->getStatus();
        unsigned int mask1=0x01;
          
          
        //ROS_INFO ("STATUS %d",statusGPS);
        //NO_ROTATION_STATUS is not program hear because Setnorotation is not avaible in MTIG	
        
        diagnostic_updater::DiagnosticStatusWrapper test;
          
            for  (int j = 0; j < 5; j++)
            
            {					
              
              if (j==0)
              {
              test.name = "Diagnostic IMU";
                              
                        test.hardware_id="Imu MTi-g 00500708";
              
                if ((res == XRV_OK))
                {
                test.level = 0;
                test.message = "No error occured";
                }
                                
                if ((res != XRV_OK))
                {
                test.level = 2;
                
                
                switch (int(res))
                {
                case 1 :  test.message = "Error : 1  No bus communication possible";break;
                case 2 :  test.message = "Error : 2  InitBus and/or SetBID are not issued";break;
                case 3 :  test.message = "Error : 3  Period sent is invalid";break;
                case 4 :  test.message = "Error : 4  The message is invalid or not implemented";break;
                case 16 :  test.message = "Error : 16  A slave did not respond to WaitForSetBID";break;
                case 17 :  test.message = "Error : 17  An incorrect answer received after WaitForSetBID"; break;
                case 18 :  test.message = "Error : 18  After four bus-scans still undetected Motion Trackers"; break;
                case 20 :  test.message = "Error : 20  No reply to SetBID message during SetBID procedure"; break;
                case 21 :  test.message = "Error : 21  Other than SetBIDAck received";break;
                case 24 :  test.message = "Error : 24  Timer overflow - period too short to collect all data from Motion Trackers";break;
                case 25 :  test.message = "Error : 25  Motion Tracker responds with other than SlaveData message";break;
                case 26 :  test.message = "Error : 26  Total bytes of data of Motion Trackers including sample counter exceeds 255 bytes";break;
                case 27 :  test.message = "Error : 27  Timer overflows during measurement";break;
                case 28 :  test.message = "Error : 28  Timer overflows during measurement";break;
                case 29 :  test.message = "Error : 29  No correct response from Motion Tracker during measurement";break;
                case 30 :  test.message = "Error : 30  Other than SetBIDAck received";break;
                case 32 :  test.message = "Error : 32  Other than SetBIDAck received";break;
                case 33 :  test.message = "Error : 33  Other than SetBIDAck received";break;
                case 35 :  test.message = "Error : 35  Other than SetBIDAck received";break;
                case 36  : test.message = "Error : 36  Other than SetBIDAck received";break;
                case 256 :   test.message = "Error : 256  A generic error occurred";break;
                case 257 :   test.message = "Error : 257  Operation not implemented in this version (yet)"; break;
                case 258 :   test.message = "Error : 258  A timeout occurred";break;
                case 259 :   test.message = "Error : 259  Operation aborted because of no data read";break;
                case 260 :   test.message = "Error :260  Checksum fault occurred";break;
                case 261 :   test.message = "Error : 261  No internal memory available";break;
                case 262 :   test.message = "Error : 262  The requested item was not found";break;
                case 263 :   test.message = "Error : 263  Unexpected message received (e.g. no acknowledge message received)";break;
                case 264 :   test.message = "Error : 264  Invalid id supplied";break;
                case 265 :   test.message = "Error : 265 Operation is invalid at this point";break;
                case 266 :   test.message = "Error : 266  Insufficient buffer space available";break;
                case 267 :   test.message = "Error : 267 The specified input device can not be opened";break;
                case 268 :   test.message = "Error : 268  The specified outpu device can not be opened";break;
                case 269 :   test.message = "Error : 269  An I/O device is already opened with this object"; break;
                case 270 :   test.message = "Error : 270   End of file is reached";break;
                case 271 :   test.message = "Error : 271  A required settings file could not be opened or is missing some data";break;
                case 272 :   test.message = "Error : 272  No data is available";break;
                case 273 :   test.message = "Error : 273  Tried to change a read-only value";break;
                case 274 :   test.message = "Error : 274  Tried to supply a NULL value where it is not allowed";break;
                case 275 :   test.message = "Error : 275  Insufficient data was supplied to a function";break;
                case 276 :   test.message = "Error : 276  Busy processing, try again later";break;
                case 277 :   test.message = "Error : 277  Invalid instance called";break;
                case 278 :   test.message = "Error : 278  A trusted data stream proves to contain corrupted data";break;
                case 279 :   test.message = "Error : 279  Failure during read of settings";break;
                case 280 :   test.message = "Error : 280  Could not find any MVN-compatible hardware";break;
                case 281 :   test.message = "Error : 281 Found only one responding Xbus Master";break;
                case 282 :   test.message = "Error : 282  No sensors found";break;
                case 283 :   test.message = "Error : 283   One or more sensors are not where they were expected";break;
                case 284 :   test.message = "Error : 284  Not enough sensors were found";break;
                case 285 :   test.message = "Error : 285  Failure during initialization of Fusion Engine"; break;
                case 286 :   test.message = "Error : 286  Something else was received than was requested"; break;
                case 287 :   test.message = "Error : 287  No file opened for reading/writing";break;
                case 288 :   test.message = "Error : 288  No serial port opened for reading/writing";break;
                case 289 :   test.message = "Error : 289  No file or serial port opened for reading/writing"; break;
                case 290 :   test.message = "Error : 290 A required port could not be found";break;
                case 291 :   test.message = "Error : 291  The low-level port handler failed to initialize"; break;
                case 292 :   test.message = "Error : 292  A calibration routine failed";break;
                case 293 :   test.message = "Error : 293  The in-config check of the device failed";break;
                case 294 :   test.message = "Error : 294  The operation is once only and has already been performed";break;
                case 295 :   test.message = "Error : 295  The single connected device is configured as a slave";break;
                case 296 :   test.message = "Error : 296  More than one master was detected";break;
                case 297 :   test.message = "Error : 297  A device was detected that was neither master nor slave";break;
                case 298 :   test.message = "Error : 298  No master detected";break;
                case 299 :   test.message = "Error : 299  A device is not sending enough data";break;
                case 300 :   test.message = "Error : 300  The version of the object is too low for the requested operation";break;
                case 301 :   test.message = "Error : 301  The object has an unrecognized version, so it's not safe to perform the operation";break;
                
                }
                
                }
                                                
                              
              status_msg.status.push_back(test);
              }
              
                                            
              if (j==1)
              {
              test.name = "Self Test";
                              
                        test.hardware_id="Imu MTi-g 00500708";
              
                if ((statusGPS & mask1) == mask1)
                {
                test.level = 0;
                test.message = "The power-up self test completed successfully";
                }								
                else
                {
                test.level = 2;
                test.message = "The power-up self test did not complete successfully";
                }
                
              status_msg.status.push_back(test);
                
              }
              
              if (j==2)
              {
              mask1 = mask1 << 1;
              test.name="XKF Valid";
              test.hardware_id="Imu MTi-g 00500708";
              
                if ((statusGPS & mask1) == mask1)
                {
                test.level=0;
                test.message="Input into the XKF orientation filer is reliable and /or complete."; 
                }								
                else
                {
                test.level = 2;;
                test.message="Input into the XKF orientation filer is not reliable and /or incomplete. For MTi-g the GPS status could remain invalid for an extended period"; 
                
                }
              
              status_msg.status.push_back(test);
              }
              
              if (j==3)
              {
              mask1 = mask1 << 1;
              test.name="GPS Fix";
								
								test.hardware_id="Imu MTi-g 00500708";
								
									if ((statusGPS & mask1) == mask1)
									{
									test.level=0;
									test.message="The GPS unit has a proper fix";
									}								
									else
									{
									test.level=2;
									test.message="The GPS unit has not a proper fix";
									}
								status_msg.status.push_back(test);
								
								}
								
														
																
								if (j==4)
								{
								test.name = "Number of satellite";
                     						test.hardware_id="Imu MTi-g 00500708";
								
								int mean_sat=3;
									
									
									if (nb_sat_OK<(mean_sat -1))
									{
									test.level = 2;
									test.message = "Not enought satellites ";
									}								
									if (nb_sat_OK== mean_sat)
									{
									test.level = 1;
									test.message = "Only 3 satellites";
									}
									if (nb_sat_OK > (mean_sat+1))
									{
									test.level = 0;
									test.message = "Sufficient number of satellites";
									}
									
								status_msg.status.push_back(test);
									
								}						
								
							
							  }	
							  
							
							unsigned int mask2=0x100;
							
							if ((mask2 & statusGPS)==mask2)
							{
							 gps_msg.status.status = gps_msg.status.STATUS_FIX;							
							} 
							else
							{
		            				gps_msg.status.status = gps_msg.status.STATUS_NO_FIX;
							}
					}
						
					else 
					{
					ROS_ERROR("No Status data available\n");
					}
					  
					  
				    }
			
		  		
		  	}	  
			/*header*/
			mti_msg.header.stamp = ros::Time::now();
			mti_msg.header.frame_id = frame_id.c_str();
			vel_msg.header.stamp = ros::Time::now();
			vel_msg.header.frame_id = frame_id.c_str();
			gps_msg.header.stamp = ros::Time::now();
			gps_msg.header.frame_id = frame_id.c_str();
			temp_msg.header.stamp = ros::Time::now();
			temp_msg.header.frame_id = frame_id.c_str();
			magn_msg.header.stamp = ros::Time::now();
			magn_msg.header.frame_id = frame_id.c_str();
			gravity_msg.header.stamp = ros::Time::now();
			gravity_msg.header.frame_id = frame_id.c_str();
			status_msg.header.stamp = ros::Time::now();
			status_msg.header.frame_id=frame_id.c_str();
		
			
			//For header with the GPSSTATUS, CHOOSE GPSInfoStatus2 as type msg
			
			//gpsstatus_msg.header.stamp = ros::Time::now();
			//gpsstatus_msg.header.frame_id = frame_id.c_str();
			
			//Publication
			temp_pub.publish(temp_msg);
			mti_pub.publish(mti_msg);
                	gps_pub.publish(gps_msg);
			vel_pub.publish(vel_msg);
			magn_pub.publish(magn_msg);
			temp_pub.publish(temp_msg);
			vel_pub.publish(vel_msg);
			gpsstatus_pub.publish(gpsstatus_msg);
			gravity_pub.publish(gravity_msg);
			status_pub.publish(status_msg);
			//updater.publish(status_msg.status);
			status_msg.status.erase(status_msg.status.begin(),status_msg.status.end());				
			r.sleep();
			
					
		}     
		delete packet;
		cmt3.closePort(); 
		break;
				
	}
	
	
	return 0;
}

//////////////////////////////////////////////////////////////////////////
// doHardwareScan
//
// Checks available COM ports and scans for MotionTrackers
int doHardwareScan(xsens::Cmt3 &cmt3, CmtDeviceId deviceIds[])
{
	XsensResultValue res;
	List<CmtPortInfo> portInfo;
	unsigned long portCount = 0;
	int mtCount;
	
	ROS_INFO("Scanning for connected Xsens devices...");
	xsens::cmtScanPorts(portInfo);
	portCount = portInfo.length();
	ROS_INFO("done\n");

	if (portCount == 0) {
		ROS_ERROR("No MotionTrackers found\n\n");
		return 0;
	}

	for(int i = 0; i < (int)portCount; i++) {	
		ROS_INFO("Using COM port %s at ", portInfo[i].m_portName);
		
		switch (portInfo[i].m_baudrate) {
		case B9600  : ROS_INFO("9k6");   break;
		case B19200 : ROS_INFO("19k2");  break;
		case B38400 : ROS_INFO("38k4");  break;
		case B57600 : ROS_INFO("57k6");  break;
		case B115200: ROS_INFO("115k2"); break;
		case B230400: ROS_INFO("230k4"); break;
		case B460800: ROS_INFO("460k8"); break;
		case B921600: ROS_INFO("921k6"); break;
		default: ROS_INFO("0x%lx", portInfo[i].m_baudrate);
		}
		ROS_INFO(" baud\n\n");
	}

	ROS_INFO("Opening ports...");
	//open the port which the device is connected to and connect at the device's baudrate.
	for(int p = 0; p < (int)portCount; p++){
		res = cmt3.openPort(portInfo[p].m_portName, portInfo[p].m_baudrate);
		EXIT_ON_ERROR(res,"cmtOpenPort");  
	}
	ROS_INFO("done\n\n");

	//get the Mt sensor count.
	ROS_INFO("Retrieving MotionTracker count (excluding attached Xbus Master(s))\n");
	mtCount = cmt3.getMtCount();
	ROS_INFO("MotionTracker count: %d\n\n", mtCount);

	// retrieve the device IDs 
	ROS_INFO("Retrieving MotionTrackers device ID(s)\n");
	for(int j = 0; j < mtCount; j++){
		res = cmt3.getDeviceId((unsigned char)(j+1), deviceIds[j]);
		EXIT_ON_ERROR(res,"getDeviceId");
		ROS_INFO("Device ID at busId %i: %08lx\n\n",j+1,(long) deviceIds[j]);
	}
	
	return mtCount;
}

//////////////////////////////////////////////////////////////////////////
// getUserInputs
//
// Request user for output data
void getUserInputs(CmtOutputMode &mode, CmtOutputSettings &settings)
{
	mode = CMT_OUTPUTMODE_ORIENT | CMT_OUTPUTMODE_POSITION | CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_VELOCITY |CMT_OUTPUTMODE_TEMP | CMT_OUTPUTMODE_STATUS;
	

	if ((mode & CMT_OUTPUTMODE_ORIENT) != 0) 
	{
		settings = CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION;
		
	}
	settings |= CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
}

//////////////////////////////////////////////////////////////////////////
// doMTSettings
//
// Set user settings in MTi/MTx
// Assumes initialized global MTComm class
void doMtSettings(xsens::Cmt3 &cmt3, CmtOutputMode &mode, 
		CmtOutputSettings &settings, CmtDeviceId deviceIds[]) 
{
	XsensResultValue res;
	unsigned long mtCount = cmt3.getMtCount();

	// set sensor to config sate
	res = cmt3.gotoConfig();
	if (res != XRV_OK)
	{
		EXIT_ON_ERROR(res,"gotoConfig");
		ROS_ERROR("Go to config state impossible");
	}
	

	unsigned short sampleFreq;
	sampleFreq = cmt3.getSampleFrequency();

	// set the device output mode for the device(s)
	ROS_INFO("Configuring your mode selection\n");

	for (unsigned int i = 0; i < mtCount; i++) 
	{
		CmtDeviceMode deviceMode(mode, settings, sampleFreq);
		if ((deviceIds[i] & 0xFFF00000) != 0x00500000) {
			// not an MTi-G, remove all GPS related stuff
			deviceMode.m_outputMode &= 0xFF0F;
		}
		
		res = cmt3.setDeviceMode(deviceMode, true, deviceIds[i]);
		if (res != XRV_OK)
		{
		ROS_ERROR("Set the device impossible");
		EXIT_ON_ERROR(res,"setDeviceMode");
		}
		 
	}

	// start receiving data
	res = cmt3.gotoMeasurement();
	if (res == XRV_OK)
	{
	ROS_INFO("MTi -- Setup complete! Initiating data streaming...");
	}
	
	else
	{
	ROS_ERROR("Impossible gotoMeasurement");
	EXIT_ON_ERROR(res,"gotoMeasurement");
	}
}

//////////////////////////////////////////////////////////////////////////












