/*
 * frame_align.h
 *
 *  Created on: Jul 20, 2011
 *      Author: acmarkus
 */

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Array>
#include <Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <vector>

#ifndef FRAME_ALIGN_H_
#define FRAME_ALIGN_H_

class FrameAlign
{
  typedef std::vector<geometry_msgs::TransformStamped> TransformStampedVec;
private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_frame1_, sub_frame2_;
  ros::Publisher pub_frame2_;

  Eigen::Vector3d curr_pos1_, curr_pos2_;
  Eigen::Matrix3Xd pos1_, pos2_;
  Eigen::Transform<double, 3, Eigen::Affine> frame_transform_;

  void frame1Callback(const geometry_msgs::TransformStampedConstPtr & msg);
  void frame2Callback(const geometry_msgs::PointStampedConstPtr & msg);

public:
  FrameAlign();
  bool computeTransform();
  void grabPoses();
};

#endif /* FRAME_ALIGN_H_ */
