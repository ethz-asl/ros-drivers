/* $Id: nodelet.cpp 35613 2011-01-30 11:46:37Z joq $ */

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010 Jack O'Quin
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

#include <signal.h>

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "driver_matrix_vision_camera.h"

/** @file

    @brief ROS driver nodelet for IIDC-compatible IEEE 1394 digital cameras.

*/

/** IEEE 1394 camera driver nodelet implementation. */
class MatrixVisionCameraNodelet: public nodelet::Nodelet
{
public:
  MatrixVisionCameraNodelet():
    running_(false)
  {}

  ~MatrixVisionCameraNodelet()
  {
    if (running_)
      {
        NODELET_INFO("shutting down driver thread");
        running_ = false;
        deviceThread_->join();
        NODELET_INFO("driver thread stopped");
      }
    dvr_->shutdown();
  }

private:
  virtual void onInit();
  virtual void devicePoll();

  volatile bool running_;               ///< device is running
  boost::shared_ptr<matrix_vision_camera_driver::MatrixVisionCameraDriver> dvr_;
  boost::shared_ptr<boost::thread> deviceThread_;
};

/** Nodelet initialization.
 *
 *  @note MUST return immediately.
 */
void MatrixVisionCameraNodelet::onInit()
{
  ros::NodeHandle priv_nh(getPrivateNodeHandle());
  ros::NodeHandle node(getNodeHandle());
  ros::NodeHandle camera_nh(node, "camera");
  dvr_.reset(new matrix_vision_camera_driver::MatrixVisionCameraDriver(priv_nh, camera_nh));
  dvr_->setup();

  // spawn device thread
  running_ = true;
  deviceThread_ = boost::shared_ptr< boost::thread >
    (new boost::thread(boost::bind(&MatrixVisionCameraNodelet::devicePoll, this)));
}

/** Nodelet device poll thread main function. */
void MatrixVisionCameraNodelet::devicePoll()
{
  while (running_)
    {
      dvr_->poll();
    }
}

// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(matrix_vision_camera, driver,
                        MatrixVisionCameraNodelet, nodelet::Nodelet);
