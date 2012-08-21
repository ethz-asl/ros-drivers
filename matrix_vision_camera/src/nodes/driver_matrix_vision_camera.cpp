// $Id: driver1394.cpp 35691 2011-02-02 04:28:58Z joq $

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2009, 2010 Jack O'Quin, Patrick Beeson
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
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
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
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <boost/format.hpp>

#include <driver_base/SensorLevels.h>
#include <tf/transform_listener.h>

#include "driver_matrix_vision_camera.h"
#include "matrix_vision_camera/MatrixVisionCameraConfig.h"
#include "features.h"

#include "matrix_vision_camera/propertyMap.h"
#include "propertyMapInterface.h"



/** @file

@brief ROS driver for IIDC-compatible IEEE 1394 digital cameras.

This is a ROS driver for 1394 cameras, using libdc1394.  It can be
instantiated as either a node or a nodelet.  It is written with with
minimal dependencies, intended to fill a role in the ROS image
pipeline similar to the other ROS camera drivers.

@par Advertises

 - @b camera/image_raw topic (sensor_msgs/Image) raw 2D camera images

 - @b camera/camera_info topic (sensor_msgs/CameraInfo) Calibration
   information for each image.

*/
using namespace std;


string to_lower(string s) {
    transform(s.begin(), s.end(), s.begin(), (int(*)(int)) tolower );
    return s;
}

struct key_lcase_equal {
    string lcs;
    key_lcase_equal(const string& s) : lcs(to_lower(s)) {}
    bool operator()(const StringPropMap::value_type& p) const {
        return to_lower(p.first) == lcs;
    }
};

StringPropMap::iterator find_ignore_case(StringPropMap& m, const string& s) {
    return find_if(m.begin(), m.end(), key_lcase_equal(s));
}






namespace matrix_vision_camera_driver
{
  // some convenience typedefs
  typedef matrix_vision_camera::MatrixVisionCameraConfig Config;
  typedef driver_base::Driver Driver;
  typedef driver_base::SensorLevels Levels;

  MatrixVisionCameraDriver::MatrixVisionCameraDriver(ros::NodeHandle priv_nh,
                                     ros::NodeHandle camera_nh):
    state_(Driver::CLOSED),
    reconfiguring_(false),
    priv_nh_(priv_nh),
    camera_nh_(camera_nh),
    camera_name_("camera"),
    dev_(new matrix_vision_camera::MatrixVisionCamera()),
    srv_(priv_nh),
    cycle_(1.0),                        // slow poll when closed
    cinfo_(new camera_info_manager::CameraInfoManager(camera_nh_)),
    calibration_matches_(true),
    it_(new image_transport::ImageTransport(camera_nh_)),
    image_pub_(it_->advertiseCamera("image_raw", 1))
  {

      // publish service:
      serviceServer_ = camera_nh_.advertiseService("pollPropertyList", &MatrixVisionCameraDriver::pollPropertyMapCallback, this);
      cout << camera_name_ << endl << endl;
      continuousPoll_ = true;
      

      
  }

  MatrixVisionCameraDriver::~MatrixVisionCameraDriver()
  {}

  /** Close camera device
   *
   *  postcondition: state_ is Driver::CLOSED
   */
  void MatrixVisionCameraDriver::closeCamera()
  {
    if (state_ != Driver::CLOSED)
      {
        ROS_INFO_STREAM("[" << camera_name_ << "] closing device");
        dev_->close();
        state_ = Driver::CLOSED;
      }
  }
    
        
    
    
    // mvIMPACT SDK Examples
    std::string MatrixVisionCameraDriver::getPropertyData( const mvIMPACT::acquire::Property& prop )
    //-----------------------------------------------------------------------------
    {
        
        string output = prop.name();
        output += "\n-------------------------------\n";
        output += "Flags: " + prop.flagsAsString() + "\n";
        
        output += "Properties: ";
        
        if( prop.hasMinValue() )
        {
            output += "\t min: " + prop.readS( mvIMPACT::acquire::PROP_MIN_VAL ) + "\n";
        }
        if( prop.hasMaxValue() )
        {
            output += "\t max: " + prop.readS( mvIMPACT::acquire::PROP_MAX_VAL ) + "\n";
        }
        if( prop.hasStepWidth() )
        {
           output += "\t step: " + prop.readS( mvIMPACT::acquire::PROP_STEP_WIDTH ) + "\n";
        }
        if( prop.hasDict() )
        {
            output += "\n Dictionary: \n";
            output += "\t Key: Value \n";
            
            
            
            mvIMPACT::acquire::TComponentType type = prop.type();
            if( type == mvIMPACT::acquire::ctPropInt )
            {
                
                mvIMPACT::acquire::PropertyI p(prop);
                
                std::vector<std::pair<std::string, int> > dict;
                p.getTranslationDict( dict );

                
                for (std::vector<std::pair<std::string, int> >::iterator iter = dict.begin(); iter != dict.end(); ++iter) {
                    ostringstream convert;
                    convert << iter->second;
                    output += "\t'" + iter->first + "': " + convert.str() + "\n";  
                } 

            }
            else if( type == mvIMPACT::acquire::ctPropInt64 )
            {
                std::vector<std::pair<std::string, int> > dict;
                
                mvIMPACT::acquire::PropertyI p(prop);
                p.getTranslationDict( dict );
                
                
                for (std::vector<std::pair<std::string, int> >::iterator iter = dict.begin(); iter != dict.end(); ++iter) {
                    ostringstream convert;
                    convert << iter->second;
                    
                    output += "\t'" + iter->first + "': " + convert.str() + "\n";  
                } 
            }
            else if( type == mvIMPACT::acquire::ctPropFloat )
            {
                std::vector<std::pair<std::string, double> > dict;
                mvIMPACT::acquire::PropertyF p(prop);
                p.getTranslationDict( dict );
                
                
                for (std::vector<std::pair<std::string, double> >::iterator iter = dict.begin(); iter != dict.end(); ++iter) {
                    ostringstream convert;
                    convert << iter->second;
                    
                    output += "\t'" + iter->first + "': " + convert.str() + "\n";  
                } 
            }
            else
            {
                std::cout << "Error! Unhandled enum prop type: " << prop.typeAsString() << std::endl;
            }
        }
        
        output += "Value: " + prop.readS() + " \n";

        return output;
  
    }
    // \mvIMPACT SDK Examples
    
    
    
    
    
    
    

    
    
    
    
    
            
  bool MatrixVisionCameraDriver::pollPropertyMapCallback(matrix_vision_camera::propertyMap::Request  &req, matrix_vision_camera::propertyMap::Response &res ) {
                       
      // parse commands
      int cmd = req.command;
      string outputString = "\n";
      int outputStatus = 1; // return always success (we are optimistic...)
      
      // returns the whole list separated by \n
      if( cmd == PROPERTYMAP_GET_PROPERTY_LIST ) {  // identifier and value empty
      
          ROS_INFO("sending property list");
          
          // loop the propertymap:
          StringPropMap::const_iterator it;
          for ( it = dev_->propertyMap_.begin(); it != dev_->propertyMap_.end(); ++it) {
              
              // validate property
              if( it->second.isValid() ) {
                  outputString += it->first;
                  
                  if(req.value == PROPERTYMAP_SHOW_FLAGS || req.identifier == PROPERTYMAP_SHOW_FLAGS) {
                      outputString += " [ " + it->second.flagsAsString() + " ] ";
                  }
                  if(req.value == PROPERTYMAP_SHOW_VALUES || req.identifier == PROPERTYMAP_SHOW_VALUES) {
                      outputString += " [ " + it->second.readSArray() + " ] ";
                  }
                  
              }
              outputString += "\n";
              
          }
          
      }
      // returns an explanation text with the property stuff
      else if( cmd == PROPERTYMAP_GET_PROPERTY_INFO ) { // identifier contains the string identifier of the property
      
          ROS_INFO("sending property details");
          
          // search for the requested property
          
          // StringPropMap::const_iterator it = dev_->propertyMap_.find( req.identifier );
          StringPropMap::const_iterator it = find_ignore_case(dev_->propertyMap_, req.identifier);
          if( it == dev_->propertyMap_.end() ) {
              outputString = "Property + [" + req.identifier + "] not found!";
              outputStatus = 0;
          }
          else {
              outputString = getPropertyData( it->second );
              if( it->second.hasDict() ) {
                  outputString += "This function expects the string representation as input!\n";
              }
            //  modifyPropertyValue( it->second );
          }

          
      }
      // returns the resulting new property value
      else if( cmd == PROPERTYMAP_SET_PROPERTY ) { // identifier containts the string identifier and value the new value 
          ROS_INFO("modify property");
          
          
          // catch exceptions
          try
          {
              // find the property
              // StringPropMap::const_iterator it = dev_->propertyMap_.find( req.identifier );
              StringPropMap::const_iterator it = find_ignore_case(dev_->propertyMap_, req.identifier);
              if( it == dev_->propertyMap_.end() )
              {
                  outputString = "Property + [" + req.identifier + "] not found!";
                  outputStatus = 0;
              }
              Property prop = it->second;
              
              // save old value
              string oldValue = prop.readS();
              
              // set index and new parameter value
              int valIndex = 0;
              string index = "";    // currently ignored!
              string param = req.value;
              
              if( prop.isWriteable() )
              {
                  if( param.empty() )
                  {
                      if( prop.valCount() > 1 )
                      {
                          outputString += "--- Unexpected behaviour, multivalued field change! --- \n probably the wrong property was manipulated! \n";
                          outputStatus = 0;
                      }
                      prop.writeS( param, valIndex );
                  }
                  else
                  {
                      if( !index.empty() )
                      {
                          valIndex = atoi( index.c_str() );
                      }
                      prop.writeS( param, valIndex );
                  }
              }
              else
              {
                  outputString += "Read-Only Property, value unchanged. \n";
                  outputStatus = 0;
              }
              string newValue = prop.readS();
              outputString += "Old Value: " + oldValue + " \n New Value: " + newValue + "\n";
              
              
          }
          catch( const mvIMPACT::acquire::ImpactAcquireException& e )
          {
              outputString += "!Exception! \n";
              outputString += e.getErrorString() + "(error code: " + e.getErrorCodeAsString() + ")" + "\n";
              outputStatus = 0;
          }
          
      }
      // search for keys which contain portion of string
      else if( cmd == PROPERTYMAP_SEARCH_PROPERTY_MAP ) {
          
          // loop the propertymap:
          StringPropMap::const_iterator it;
          string s1;
          for ( it = dev_->propertyMap_.begin(); it != dev_->propertyMap_.end(); ++it) {
              s1 = to_lower(it->first);
              
              size_t found=s1.find( to_lower(req.identifier) );
              if (found != string::npos) {
                  outputString += " " + it->first +  " \n";
              }
              
          }
          
      }
      else if( cmd == PROPERTYMAP_SAVE_SETTINGS ) {
          dev_->saveCameraSettings(req.identifier);
      }
      else if( cmd == PROPERTYMAP_LOAD_SETTINGS ) {
          dev_->loadCameraSettings(req.identifier);
      }
      // capture process control:
      else if(cmd == DI_START_CAPTURE_PROCESS) {
          continuousPoll_ = 1;
      }
      else if(cmd == DI_STOP_CAPTURE_PROCESS) {
          continuousPoll_ = 0;
          // clean requests on camera:
          dev_->clearRequestQueue();
      }
      else if(cmd == DI_CAPTURE_SINGLE_FRAME) {
          pollSingle(outputString);
          dev_->clearRequestQueue();
      }
      else if(cmd == DI_RESTART_DEVICE) {  // close device and restart and initialise 
          // take lock
          boost::mutex::scoped_lock lock(mutex_);
          // close
          closeCamera();  
          // reopen
          if ( openCamera(config_) ) {
              outputString += "Camera Reopened. \n";
          }
          else {
              outputString += "Error occured while opening the camera...";
              outputStatus = 0;
          }
      }
      else if ( cmd == DI_CLOSE_DEVICE) {
          // take lock
          boost::mutex::scoped_lock lock(mutex_);
          // close
          closeCamera();  
          
          if (state_ == Driver::CLOSED) 
              outputString +="Camera Closed!";
          else
              outputString +="Could not close camera!";

      }
      else if ( cmd == DI_OPEN_DEVICE) {
          boost::mutex::scoped_lock lock(mutex_);
          if ( openCamera(config_) ) {
              outputString += "Camera Reopened. \n";
          }
          else {
              outputString += "Error occured while opening the camera...";
              outputStatus = 0;
          }
      }
      
      res.result = outputString;
      res.status = outputStatus;
      return true;
                 
  }
                        
             
             

    
    
    
    

  /** Open the camera device.
   *
   * @param newconfig configuration parameters
   * @return true, if successful
   *
   * if successful:
   *   state_ is Driver::OPENED
   *   camera_name_ set to GUID string
   */
  bool MatrixVisionCameraDriver::openCamera(Config &newconfig)
  {
    bool success = false;
    int retries = 2;                    // number of retries, if open fails
    do
      {
        try
          {
            if (0 == dev_->open(newconfig))
              {
                if (camera_name_ != dev_->device_id_)
                  {
                    camera_name_ = dev_->device_id_;
                    if (!cinfo_->setCameraName(camera_name_))
                      {
                        // GUID is 16 hex digits, which should be valid.
                        // If not, use it for log messages anyway.
                        ROS_WARN_STREAM("[" << camera_name_
                                        << "] name not valid"
                                        << " for camera_info_manger");
                      }
                  }
                ROS_INFO_STREAM("[" << camera_name_ << "] opened!");
//                                << newconfig.video_mode << ", "
                             //   << newconfig.PixelClock_KHz23 << " fps, "
                           //     << newconfig.__CONST_PIXEL_CLOCK_NAME << " MHz pixel clock");
                state_ = Driver::OPENED;
                calibration_matches_ = true;
                success = true;
              }

          }
        catch (matrix_vision_camera::Exception& e)
          {
            state_ = Driver::CLOSED;    // since the open() failed
            if (retries > 0)
              ROS_WARN_STREAM("[" << camera_name_
                              << "] exception opening device (retrying): "
                              << e.what());
            else
              ROS_ERROR_STREAM("[" << camera_name_
                               << "] device open failed: " << e.what());
          }
      }
    while (!success && --retries >= 0);
      
     // ComponentList cl =  dev_->deviceDriverFeatureList;

    return success;
  }


  /** device poll */
  void MatrixVisionCameraDriver::poll(void)
  {
    // Do not run concurrently with reconfig().
    //
    // The mutex lock should be sufficient, but the Linux pthreads
    // implementation does not guarantee fairness, and the reconfig()
    // callback thread generally suffers from lock starvation for many
    // seconds before getting to run.  So, we avoid acquiring the lock
    // if there is a reconfig() pending.
    bool do_sleep = true;

    if (!reconfiguring_ && continuousPoll_) // only enter loop if in continuous poll mode.
      {
        boost::mutex::scoped_lock lock(mutex_);
        do_sleep = (state_ == Driver::CLOSED);
        if (!do_sleep)
          {
            // driver is open, read the next image still holding lock
            sensor_msgs::ImagePtr image(new sensor_msgs::Image);

            if (read(image))
              {
                publish(image);
              }
          }
      }

    if (do_sleep)
      {
        // device was closed or poll is not running, sleeping avoids
        // busy wait (DO NOT hold the lock while sleeping)
        cycle_.sleep();
      }
  }
    
  
    
    
    
    void MatrixVisionCameraDriver::pollSingle(std::string& outputString) {
    
        // make sure continuous polling is not active
        if ( !continuousPoll_ ) {
        
            boost::mutex::scoped_lock lock(mutex_);
            bool do_sleep = (state_ == Driver::CLOSED);
            if (!do_sleep)
            {
                // driver is open, read the next image still holding lock
                sensor_msgs::ImagePtr image(new sensor_msgs::Image);

                dev_->readSingleImage(*image);
                publish(image);

            } 
            else {
                outputString += "Camera not opened.";
            }
        }
        else {
            outputString += "Continuous Stream active. Disable to take single images.";
        }
    }
    
        

  /** Publish camera stream topics
   *
   *  @param image points to latest camera frame
   */
  void MatrixVisionCameraDriver::publish(const sensor_msgs::ImagePtr &image)
  {
    image->header.frame_id = config_.frame_id;
      
    // get current CameraInfo data
    sensor_msgs::CameraInfoPtr
      ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));

    // check whether CameraInfo matches current video mode
    if (!dev_->checkCameraInfo(*image, *ci))
      {
        // image size does not match: publish a matching uncalibrated
        // CameraInfo instead
        if (calibration_matches_)
          {
            // warn user once
            calibration_matches_ = false;
            ROS_WARN_STREAM("[" << camera_name_
                            << "] calibration does not match video mode "
                            << "(publishing uncalibrated data)");
          }
        ci.reset(new sensor_msgs::CameraInfo());
        ci->height = image->height;
        ci->width = image->width;
      }
    else if (!calibration_matches_)
      {
        // calibration OK now
        calibration_matches_ = true;
        ROS_WARN_STREAM("[" << camera_name_
                        << "] calibration matches video mode now");
      }

    // fill in operational parameters
    dev_->setOperationalParameters(*ci);

    ci->header.frame_id = config_.frame_id;
    ci->header.stamp = image->header.stamp;

    // @todo log a warning if (filtered) time since last published
    // image is not reasonably close to configured frame_rate
	  
    // Publish via image_transport
    image_pub_.publish(image, ci);
  }

  /** Read camera data.
   *
   * @param image points to camera Image message
   * @return true if successful, with image filled in
   */
  bool MatrixVisionCameraDriver::read(sensor_msgs::ImagePtr &image)
  {
    bool success = true;
    try
      {
        // Read data from the Camera
        dev_->readData(*image);
      }
    catch (matrix_vision_camera::Exception& e)
      {
        ROS_WARN_STREAM("[" << camera_name_
                        << "] Exception reading data: " << e.what());
        success = false;
      }
    return success;
  }

    

    
    
  /** Dynamic reconfigure callback
   *
   *  Called immediately when callback first defined. Called again
   *  when dynamic reconfigure starts or changes a parameter value.
   *
   *  @param newconfig new Config values
   *  @param level bit-wise OR of reconfiguration levels for all
   *               changed parameters (0xffffffff on initial call)
   **/
  void MatrixVisionCameraDriver::reconfig(Config &newconfig, uint32_t level)
  {
    // Do not run concurrently with poll().  Tell it to stop running,
    // and wait on the lock until it does.
    reconfiguring_ = true;
    boost::mutex::scoped_lock lock(mutex_);
    ROS_DEBUG("dynamic reconfigure level 0x%x", level);

    // resolve frame ID using tf_prefix parameter
    if (newconfig.frame_id == "")
      newconfig.frame_id = "camera";
    std::string tf_prefix = tf::getPrefixParam(priv_nh_);
    ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
    newconfig.frame_id = tf::resolve(tf_prefix, newconfig.frame_id);

    if (state_ != Driver::CLOSED && (level & Levels::RECONFIGURE_CLOSE))
      {
        // must close the device before updating these parameters
        closeCamera();                  // state_ --> CLOSED
      }

    if (state_ == Driver::CLOSED)
      {
        // open with new values
        if (openCamera(newconfig))
          {
            // update GUID string parameter
            // TODO move into dev_->open()
            newconfig.guid = camera_name_;
          }
      }

    if (config_.camera_info_url != newconfig.camera_info_url)
      {
        // set the new URL and load CameraInfo (if any) from it
        if (cinfo_->validateURL(newconfig.camera_info_url))
          {
            cinfo_->loadCameraInfo(newconfig.camera_info_url);
          }
        else
          {
            // new URL not valid, use the old one
            newconfig.camera_info_url = config_.camera_info_url;
          }
      }

    if (state_ != Driver::CLOSED)       // openCamera() succeeded?
      {
        // configure IIDC features
        if (level & Levels::RECONFIGURE_CLOSE)
          {
            // initialize all features for newly opened device
            if (false == dev_->features_->initialize(&newconfig))
              {
                ROS_ERROR_STREAM("[" << camera_name_
                                 << "] feature initialization failure");
                closeCamera();          // can't continue
              }
          }
        else
          {
            // update any features that changed
            // TODO replace this with a dev_->reconfigure(&newconfig);
            dev_->features_->reconfigure(&newconfig);
          }
      }

    config_ = newconfig;                // save new parameters
    reconfiguring_ = false;             // let poll() run again

    ROS_DEBUG_STREAM("[" << camera_name_
                     << "] reconfigured: frame_id " << newconfig.frame_id
                     << ", camera_info_url " << newconfig.camera_info_url);
  }


  /** driver initialization
   *
   *  Define dynamic reconfigure callback, which gets called
   *  immediately with level 0xffffffff.  The reconfig() method will
   *  set initial parameter values, then open the device if it can.
   */
  void MatrixVisionCameraDriver::setup(void)
  {
    srv_.setCallback(boost::bind(&MatrixVisionCameraDriver::reconfig, this, _1, _2));
  }


  /** driver termination */
  void MatrixVisionCameraDriver::shutdown(void)
  {
    closeCamera();
  }

}; // end namespace matrix_vision_camera_driver
