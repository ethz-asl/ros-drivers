///////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2009, 2010 Patrick Beeson, Jack O'Quin, Ken Tossell
//  ROS port of the Player 1394 camera driver.
//
// Copyright (C) 2004 Nate Koenig, Andrew Howard
//  Player driver for IEEE 1394 digital camera capture
//
// Copyright (C) 2000-2003 Damien Douxchamps, Dan Dennedy
//  Bayer filtering from libdc1394
//
// NOTE: On 4 Jan. 2011, this file was re-licensed under the GNU LGPL
// with permission of the original GPL authors: Nate Koenig, Andrew
// Howard, Damien Douxchamps and Dan Dennedy.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2 of
// the License, or (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
//
///////////////////////////////////////////////////////////////////////////

// $Id: dev_matrix_vision_camera.cpp 36357 2011-03-05 01:36:03Z ktossell $

/** @file

 @brief libdc1394 digital camera library interface implementation

 This device interface is partly derived from the Player 1394
 camera driver.

 The ROS image pipeline provides Bayer filtering at a higher level
 (in image_proc).  In some cases it is useful to run the driver
 without the entire image pipeline, so libdc1394 Bayer decoding is
 also provided here.

 */

#include <stdint.h>

#include <sensor_msgs/image_encodings.h>
#include "dev_matrix_vision_camera.h"
#include "features.h"
#include "formats.h"

#define NUM_DMA_BUFFERS 4

// @todo eliminate these macros
//! Macro for throwing an exception with a message
#define CAM_EXCEPT(except, msg)					\
		{								\
	char buf[100];						\
	snprintf(buf, 100, "[MatrixVisionCamera::%s]: " msg, __FUNCTION__); \
	throw except(buf);						\
		}

//! Macro for throwing an exception with a message, passing args
#define CAM_EXCEPT_ARGS(except, msg, ...)				\
		{									\
	char buf[100];							\
	snprintf(buf, 100, "[MatrixVisionCamera::%s]: " msg, __FUNCTION__, __VA_ARGS__); \
	throw except(buf);							\
		}

using namespace matrix_vision_camera;
using namespace mvIMPACT::acquire;
using namespace std;


MatrixVisionCamera::MatrixVisionCamera() :
    cam_(NULL)
{
  rosTimeOffset_ = -1;

  int ret = 0;

#define xstr(s) str(s)
#define str(s) #s

  ret += setenv("GENICAM_ROOT", xstr(GENICAM_ROOT), 1);
  ret += setenv("GENICAM_ROOT_V2_3", xstr(GENICAM_ROOT_V2_3), 1);
  ret += setenv("GENICAM_GENTL64_PATH", xstr(GENICAM_GENTL64_PATH), 1);
  ret += setenv("GENICAM_LOG_CONFIG_V2_3", xstr(GENICAM_LOG_CONFIG_V2_3), 1);

  ROS_WARN_STREAM_COND(ret != 0, "couldn't set BlueCOUGAR environment variables ");

  ROS_INFO_STREAM("ENV_test:"<<getenv("GENICAM_ROOT"));

  dev_mgr_.reset(new mvIMPACT::acquire::DeviceManager);
}

MatrixVisionCamera::~MatrixVisionCamera()
{
  SafeCleanup();
}

/** Open the matrix_vision_camera device and start streaming
 *
 *  @param newconfig new configuration parameters
 *  @return 0 if successful
 *
 *  TODO (if successful):
 *     * update newconfig.guid
 *     * validate newconfig.video_mode
 *     * initialize Features class
 */
int MatrixVisionCamera::open(matrix_vision_camera::MatrixVisionCameraConfig &newconfig)
{
  dev_mgr_->updateDeviceList();

  if (dev_mgr_->deviceCount() == 0)
  {
    CAM_EXCEPT(matrix_vision_camera::Exception, "No cameras found");
    return -1;
  }

  if (newconfig.guid == "" || newconfig.guid.size() < 8)
  {
    ROS_INFO_STREAM("no GUID specified or not properly specified (guid=\"" << newconfig.guid << "\"), trying to open first available camera");
    for (unsigned int i = 0; i < dev_mgr_->deviceCount(); i++)
    {
      cam_ = dev_mgr_->getDevice(i);
      if (cam_ != NULL)
      {
        try
        {
          cam_->open();
          break;
        }
        catch (const mvIMPACT::acquire::ImpactAcquireException& e)
        {
          cam_ = NULL;
	  ROS_WARN_STREAM("Unable to open camera: " << e.what());
        }
      }
    }
  }
  else
  {
    std::string serial = newconfig.guid.substr(newconfig.guid.size() - 8);
    ROS_INFO_STREAM("trying to open camera " << serial);
    cam_ = dev_mgr_->getDeviceBySerial(serial);
    if (cam_ != NULL)
    {
      try
      {
        cam_->open();
      }
      catch (const mvIMPACT::acquire::ImpactAcquireException& e)
      {
        cam_ = NULL;
	ROS_WARN_STREAM("Unable to open camera: " << e.what());
      }
    }
  }

  if (cam_ == NULL)
  {
    if (newconfig.guid == "" || newconfig.guid.find("MATRIX_VISION_CAMERA_") == std::string::npos)
    {
      CAM_EXCEPT(matrix_vision_camera::Exception, "Could not find a camera");
    }
    else
    {
      CAM_EXCEPT_ARGS(matrix_vision_camera::Exception, "Could not find camera with guid %s", newconfig.guid.c_str());
    }
    return -1;
  }

  device_id_ = "MATRIX_VISION_CAMERA_" + cam_->serial.read();

  cam_fi_.reset(new FunctionInterface(cam_));
  cam_stats_.reset(new Statistics(cam_));
  cam_ss_.reset(new SystemSettings(cam_));
  cam_irc_.reset(new ImageRequestControl(cam_));

  ROS_INFO_STREAM("camera family:"<<cam_->family.read()<<"; camera model: " << cam_->product.read() << "; firmware version: " << cam_->firmwareVersion.read() << "; serial number: " << cam_->serial.read());

  //////////////////////////////////////////////////////////////
  // initialize camera
  //////////////////////////////////////////////////////////////

  int MAX_REQUESTS = 4;
  // make sure enough requests are available:
  cam_ss_->requestCount.write(MAX_REQUESTS);
  int request_result = DMR_NO_ERROR;
  int request_count = 0;
  while ((request_result = cam_fi_->imageRequestSingle(cam_irc_.get())) == DMR_NO_ERROR)
  {
    ++request_count;
  }

  use_ros_time_ = newconfig.use_ros_time;

  //////////////////////////////////////////////////////////////
  // initialize feature settings
  //////////////////////////////////////////////////////////////

  // TODO: pass newconfig here and eliminate initialize() method
  features_.reset(new Features(cam_));
    
  // get features list:
/* ComponentList cl = cam_->getDeviceDriverFeatureList();
    std::string  s1 = cl.contentDescriptor();
    std::string  s2 = cl.docString();
    std::string s3 = cl.flagsAsString();
    int valid = cl.size();
    //ROS_INFO_STREAM("CL:" << s1 << s2 << s3 << inttostr(valid));
    printf(" %i \n",valid);*/
    
 //   CameraSettingsMatrixVisionCamera s(cam_);
 //   s.lineDelay_clk.write(100);
 //   int o = s.lineDelay_clk.read();
 //   printf("lineDelay_clk: %i", o);
    
    

    // generate the property map
    generatePropertyMap();
    
    
  return 0;
}




/** Safe Cleanup -- may get called more than once. */
void MatrixVisionCamera::SafeCleanup()
{

}

/** close the device */
int MatrixVisionCamera::close()
{
  if (cam_)
  {
    cam_fi_->imageRequestReset(0, 0);
    cam_->close();
      
      // reset the timestamp corrector:
      timestampCorrector_ = sm::timing::TimestampCorrector<double>();
      
  }

  // Free resources
  SafeCleanup();

  return 0;
}



void MatrixVisionCamera::fillSensorMsgs(sensor_msgs::Image& image, const Request* req, ros::Time time_now ) {

    
    // save image info in published message:
    image.data.resize(req->imageSize.read());
    image.height = req->imageHeight.read();
    image.width = req->imageWidth.read();
    image.step = req->imageLinePitch.read();
    image.header.seq = req->infoFrameNr.read();
    // get the image encoding and bayer infos
    if(req->imageBayerMosaicParity.read() != mvIMPACT::acquire::bmpUndefined)
    {
        int bpp = req->imageBytesPerPixel.read()*8;
        image.encoding = bayerString(req->imageBayerMosaicParity.read(), bpp);
        // ROS_INFO_STREAM_THROTTLE(1, "raw image, encoding: "<<image.encoding<<" bpp"<<bpp);
    }
    else
    {
        image.encoding = pixelFormat(req->imagePixelFormat.read());
        // ROS_INFO_STREAM_THROTTLE(1, "(processed) image, encoding: "<<image.encoding);
    }
    // copy the image data
    memcpy(&image.data[0], req->imageData.read(), image.data.size());
    
    if (use_ros_time_)
    {
        image.header.stamp = time_now;
    }
    else
    {
        /*
        // if the offset is not initialised => do with current frame:
        if (rosTimeOffset_ == -1) {
            // current ros time (measured before image processing)
            // uint64_t rosTimeNs = time_now.toNSec();
            // image capture time 
            int64_t imageTimeUs = req->infoTimeStamp_us.read();
            int64_t exposureTime = req->infoExposeTime_us.read();
            // store
            rosTimeOffset_ = imageTimeUs + exposureTime; // 
            // ROS_INFO_STREAM("rosTimeOffset initialised: " << rosTimeOffset_);
            
        }
        int64_t currentImageTimeUs = req->infoTimeStamp_us.read();
        int64_t currentExposureTime = req->infoExposeTime_us.read();
        // substract the offset and exposure time:
        image.header.stamp = time_now - ros::Duration( double(currentImageTimeUs-rosTimeOffset_ - currentExposureTime) / (1*10e+6)  );*/
        
       
        
        // get the image timestamp:
        int64_t currentImageTimeS = req->infoTimeStamp_us.read(); //  / (1*10e+6)
        // add the corrected timestamp:       
        image.header.stamp = ros::Time( timestampCorrector_.correctTimestamp(currentImageTimeS, time_now.toSec())) ;
        
        
     //   image.header.seq = req->infoTimeStamp_us.read();
        
        
        //          ROS_INFO_STREAM("TimeStamp:" << image.header.stamp.toSec());
    }

    
}

// requests and saves a single image
void MatrixVisionCamera::readSingleImage(sensor_msgs::Image& image) {
    
    ROS_ASSERT_MSG(cam_, "Attempt to read from camera that is not open.");
    
    // request one image:
    int requestResult = cam_fi_->imageRequestSingle(); // cam_irc_.get()
    // max wait time:
    const int iMaxWaitTime_ms = 500;
    // wait for results from the default capture queue
    int requestNr = cam_fi_->imageRequestWaitFor( iMaxWaitTime_ms );
    
    ros::Time time_now = ros::Time::now();
    
    // check if the image has been captured without any problems
    if( !cam_fi_->isRequestNrValid( requestNr ) )
    {
        ROS_ERROR_STREAM("Failed to Capture Image. RequestNr: " << requestNr );
        return;
    }
    
    const Request* req = cam_fi_->getRequest(requestNr);
    // check request validity
    if( !cam_fi_->isRequestOK(req) )
    {
        ROS_ERROR_STREAM("Request result: " << req->requestResult.readS());
        return;
    }
    
    fillSensorMsgs(image, req, time_now);
    
    // unlock the request
    cam_fi_->imageRequestUnlock( requestNr );
    
}

void MatrixVisionCamera::clearRequestQueue() {
    cam_fi_->imageRequestReset( 0, 0 );
}


/** Return an image frame */
void MatrixVisionCamera::readData(sensor_msgs::Image& image)
{
  ROS_ASSERT_MSG(cam_, "Attempt to read from camera that is not open.");
  static int err_cnt = 0;

  // always keep request queue filled
  int request_result = DMR_NO_ERROR;
  int request_count = 0;
  while ((request_result = cam_fi_->imageRequestSingle(cam_irc_.get())) == DMR_NO_ERROR)
  {
    ++request_count;
  }

  // wait for results from the default capture queue
  int timeout_ms = static_cast<int> (1.0 / features_->getFPS() * 1.0e3 * 2); // wait max 200% of frametime
  int request_nr;

  if(err_cnt < 5)
    request_nr = cam_fi_->imageRequestWaitFor(timeout_ms);
  else
    request_nr = cam_fi_->imageRequestWaitFor(timeout_ms*5);

  const mvIMPACT::acquire::Request * req;
  ros::Time time_now = ros::Time::now();

  if (cam_fi_->isRequestNrValid(request_nr))
  {
    req = cam_fi_->getRequest(request_nr);
    if (req->isOK())
    {
        
        
      // float secondsTime = time_now.toSec();
      // uint64_t rosTimeUs = time_now.toNSec() / 1000;
      // int64_t imageTimeUs = req->infoTimeStamp_us.read();
       
      // uint64_t deltaT = rosTimeUs - imageTimeUs;
      // ROS_INFO_STREAM("Delta: " << deltaT << " Ros: " << rosTimeUs << " image: " << imageTimeUs);
        
      // ROS_INFO_STREAM("ROSTime:" << secondsTime << " - " << nanoSeconds);
  
      // int timeDiffRosImage = nanoSeconds / 1000 - req->infoExposeStart_us.read();
        
      // ROS_INFO_STREAM("Delta: " << timeDiffRosImage << " exposureTime:" << exposureTime << " Image Time:" );
        
 //     ROS_INFO_STREAM("Timings: exp_start= "<< req->infoExposeStart_us.read() << " ts= " << req->infoTimeStamp_us.read() << " delay= " << req->infoTransferDelay_us.read());

 /*     image.data.resize(req->imageSize.read());
      image.height = req->imageHeight.read();
      image.width = req->imageWidth.read();
      image.step = req->imageLinePitch.read();
      image.header.seq = req->infoFrameNr.read();
      //ROS_INFO_STREAM_THROTTLE(1, "pixformat cam: "<<req->imagePixelFormat.readS()<<" bayer pattern "<<req->imageBayerMosaicParity.readS());
      if(req->imageBayerMosaicParity.read() != mvIMPACT::acquire::bmpUndefined)
      {
        int bpp = req->imageBytesPerPixel.read()*8;
        image.encoding = bayerString(req->imageBayerMosaicParity.read(), bpp);
        // ROS_INFO_STREAM_THROTTLE(1, "raw image, encoding: "<<image.encoding<<" bpp"<<bpp);
      }
      else
      {
        image.encoding = pixelFormat(req->imagePixelFormat.read());
        // ROS_INFO_STREAM_THROTTLE(1, "(processed) image, encoding: "<<image.encoding);
      }

      memcpy(&image.data[0], req->imageData.read(), image.data.size());

      if (use_ros_time_)
      {
        image.header.stamp = time_now;
      }
      else
      {
          // if the offset is not initialised => do with current frame:
          if (rosTimeOffset_ == -1) {
              // current ros time (measured before image processing)
              // uint64_t rosTimeNs = time_now.toNSec();
              // image capture time 
              int64_t imageTimeUs = req->infoTimeStamp_us.read();
              int64_t exposureTime = req->infoExposeTime_us.read();
              // store
              rosTimeOffset_ = imageTimeUs + exposureTime; // 
  //            ROS_INFO_STREAM("rosTimeOffset initialised: " << rosTimeOffset_);
              
          }
          int64_t currentImageTimeUs = req->infoTimeStamp_us.read();
          int64_t currentExposureTime = req->infoExposeTime_us.read();
          // substract the offset and exposure time:
          image.header.stamp = time_now - ros::Duration( double(currentImageTimeUs-rosTimeOffset_ - currentExposureTime) / (1*10e+6)  );
          
          // double TST = double(currentImageTimeUs-rosTimeOffset_ - currentExposureTime) / (1*10e+6);
          // ROS_INFO_STREAM("TST:" << TST);
//          ROS_INFO_STREAM("TimeStamp:" << image.header.stamp.toSec());
      }
*/
        
      fillSensorMsgs(image, req, time_now);
        

        
      features_->getImageInfo().width = image.width;
      features_->getImageInfo().height = image.height;
      features_->getImageInfo().color_coding = image.encoding;
      features_->getImageInfo().exposure_time = usToS(req->infoExposeTime_us.read());
      features_->getImageInfo().gain = req->infoGain_dB.read();
    }
    else
    {
      cam_fi_->imageRequestUnlock(request_nr);
      CAM_EXCEPT_ARGS(matrix_vision_camera::Exception, "Error while grabbing frame: %s", req->requestResult.readS().c_str() );
    }
      
      
      

    // this image has been displayed thus the buffer is no longer needed...
    cam_fi_->imageRequestUnlock(request_nr);
//    // send a new image request into the capture queue
//    cam_fi_->imageRequestSingle(/*cam_irc_.get()*/);

//    // send a new image request into the capture queue
//    int req_single_res = cam_fi_->imageRequestSingle(/*cam_irc_.get()*/);
//
//    if(req_single_res != DMR_NO_ERROR)
//    {
//      ROS_WARN_STREAM("state: " << cam_->state.read(0) );
//      CAM_EXCEPT_ARGS(matrix_vision_camera::Exception, "imageRequestSingle failed! reason: %d", req_single_res);
//    }




      // print some statistics:
//      string fps = cam_stats_->framesPerSecond.readS();
//      string capT = cam_stats_->captureTime_s.readS();
//      cout << "fps: " << fps << " capT:" << capT << endl;
      
      
    err_cnt = 0;
  }
  else
  {
    err_cnt ++;

    ROS_WARN_STREAM("state: "<<cam_->state.read(0)<<" fps "<<features_->getFPS()<< " timeout "<< timeout_ms);

//    // send a new image request into the capture queue
//    int req_single_res = cam_fi_->imageRequestSingle(/*cam_irc_.get()*/);
//
//    if(req_single_res != DMR_NO_ERROR)
//    {
//      ROS_WARN_STREAM("state: " << cam_->state.read(0) );
//      CAM_EXCEPT_ARGS(matrix_vision_camera::Exception, "imageRequestSingle failed! reason: %d", req_single_res);
//    }

    CAM_EXCEPT_ARGS(matrix_vision_camera::Exception, "imageRequestWaitFor failed! reason: %d", request_nr);
  }
    
}




/** Generates a Property Map which links Camera Properties to an Identifier String:
 *
 * @param newconfig configuration parameters
 * @return true, if successful
 *
 * if successful:
 *   state_ is Driver::OPENED
 *   camera_name_ set to GUID string
 */
void MatrixVisionCamera::generatePropertyMap() {
    
    
    
    DeviceComponentLocator locator(cam_, dltSetting, "Base");
    populatePropertyMap( propertyMap_, ComponentIterator(locator.searchbase_id()).firstChild() );
    
    try
    {
        // this category is not supported by every device, thus we can expect an exception if this feature is missing
        locator = DeviceComponentLocator(cam_, dltIOSubSystem);
        populatePropertyMap( propertyMap_, ComponentIterator(locator.searchbase_id()).firstChild() );
    } 
    catch( const ImpactAcquireException& ) {}
    
    locator = DeviceComponentLocator(cam_, dltRequest);
    populatePropertyMap( propertyMap_, ComponentIterator(locator.searchbase_id()).firstChild() );
    locator = DeviceComponentLocator(cam_, dltSystemSettings);
    populatePropertyMap( propertyMap_, ComponentIterator(locator.searchbase_id()).firstChild(), string("SystemSettings") );
    locator = DeviceComponentLocator(cam_, dltInfo);
    populatePropertyMap( propertyMap_, ComponentIterator(locator.searchbase_id()).firstChild(), string("Info") );
    populatePropertyMap( propertyMap_, ComponentIterator(cam_->hDev()).firstChild(), string("Device") );
    
    
} 


// mvIMPACT SDK Examples
// Helper funciton to generate the property maps
//-----------------------------------------------------------------------------
void MatrixVisionCamera::populatePropertyMap( StringPropMap& m, ComponentIterator it, const std::string& currentPath )
//-----------------------------------------------------------------------------
{
    while( it.isValid() )
    {
        std::string fullName( currentPath );
        if( fullName != "" )
        {
            fullName += "/";
        }
        fullName += it.name();
        if( it.isList() )
        {
            populatePropertyMap( m, it.firstChild(), fullName );
        }
        else if( it.isProp() )
        {
            m.insert( make_pair( fullName, Property( it ) ) );
        }
        ++it;
        // method object will be ignored...
    }
}
// \mvIMPACT SDK Examples


void MatrixVisionCamera::saveCameraSettings( std::string path) {

    cam_fi_->saveSetting(path, sfFile);
    
}

void MatrixVisionCamera::loadCameraSettings( std::string path ) {

    cam_fi_->loadSetting(path, sfFile);

}


