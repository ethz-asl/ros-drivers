/*
 * frame_align.cpp
 *
 *  Created on: Jul 20, 2011
 *      Author: acmarkus
 */

#include "frame_align.h"

FrameAlign::FrameAlign() :
  pnh_(nh_)
{
  sub_frame1_ = nh_.subscribe("frame1", 10, &FrameAlign::frame1Callback, this);
  sub_frame2_ = nh_.subscribe("frame2", 10, &FrameAlign::frame2Callback, this);

  pub_frame2_ = nh_.advertise<geometry_msgs::PointStamped> ("frame2_out", 1);
  frame_transform_.setIdentity();

  curr_pos1_ = Eigen::Vector3d::Zero();
  curr_pos2_ = Eigen::Vector3d::Zero();
}

void FrameAlign::frame1Callback(const geometry_msgs::TransformStampedConstPtr & msg)
{
  curr_pos1_[0] = msg->transform.translation.x;
  curr_pos1_[1] = msg->transform.translation.y;
  curr_pos1_[2] = msg->transform.translation.z;
}
void FrameAlign::frame2Callback(const geometry_msgs::PointStampedConstPtr & msg)
{
  curr_pos2_[0] = msg->point.x;
  curr_pos2_[1] = msg->point.y;
  curr_pos2_[2] = msg->point.z;

  //  // re-publish the transformed pose w.r.t coordinate frame 1
  geometry_msgs::PointStampedPtr msg_out(new geometry_msgs::PointStamped);

  Eigen::Vector3d pos_out = frame_transform_ * curr_pos2_;
  msg_out->point.x = pos_out[0];
  msg_out->point.y = pos_out[1];
  msg_out->point.z = pos_out[2];

  msg_out->header = msg->header;
  pub_frame2_.publish(msg_out);
}

void FrameAlign::grabPoses()
{
  Eigen::Vector3d curr_pos1 = curr_pos1_;
  Eigen::Vector3d curr_pos2 = curr_pos2_;

  while(curr_pos1 == curr_pos1_ || curr_pos2 == curr_pos2_ ){
    usleep(100e3);
    ros::spinOnce();
  }

  // resize ... quite inefficient ;-)
  pos1_.conservativeResize(Eigen::NoChange, pos1_.cols() + 1);
  pos2_.conservativeResize(Eigen::NoChange, pos2_.cols() + 1);

  pos1_.block(0, pos1_.cols() - 1, 3, 1) = curr_pos1_;
  pos2_.block(0, pos2_.cols() - 1, 3, 1) = curr_pos2_;

  std::cout << "pos1:" << std::endl << curr_pos1_.transpose() << std::endl;
  std::cout << "pos2:" << std::endl << curr_pos2_.transpose() << std::endl;
}

bool FrameAlign::computeTransform()
{
  if (pos1_.cols() < 3)
  {
    std::cout << "WARNNIG: Not enough points, need at least 3!" << std::endl;
    return false;
  }
  std::cout << "Points from coordinate frame 1:" << std::endl << pos1_ << std::endl;
  std::cout << "Points from coordinate frame 2:" << std::endl << pos2_ << std::endl;

  // do the work:
  frame_transform_ = Eigen::umeyama(pos2_, pos1_, true);

  Eigen::Matrix3d scale_m;
  Eigen::Matrix3d rotation;
  frame_transform_.computeScalingRotation(&scale_m, &rotation);
  double scale = scale_m.diagonal().sum() / 3.0;

  std::cout<<"Result (transforms coordinates from frame2 to frame 1):"<<std::endl
      <<"homogeneous transform matrix:"<<std::endl<<frame_transform_.matrix()<<std::endl
      <<"rotation matrix:"<<std::endl<<rotation<<std::endl
      <<"rotation quaternion (x y z w):"<<std::endl<<Eigen::Quaterniond(rotation).coeffs().transpose()<<std::endl
      <<"translation:"<<std::endl<<frame_transform_.translation().transpose()<<std::endl
      <<"scale: "<<scale<<std::endl<<scale_m<<std::endl;

  return true;
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "frame_align");

  FrameAlign fa;

  char kb = 0;
  char dummy = 0;

  while (ros::ok())
  {
    printf("press " "g" " + enter to grab pose, press " "f" "+ enter when you're done\n");
    kb = getchar();
    dummy = getchar();
    if (kb == 'g')
      fa.grabPoses();
    else if (kb == 'f')
    {
      if (fa.computeTransform())
        break;
    }
    else
      printf("key %d has no meaning\n", (int)kb);
  }

  ros::spin();

  return 0;
}
