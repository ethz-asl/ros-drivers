/*
 * TfDistort.cpp
 *
 *  Created on: Jun 8, 2011
 *      Author: acmarkus
 */

#include "tf_distort.h"

namespace tf_distort{

void whiteGaussianNoise(double * n1, double * n2)
{
  // Box Muller Method - from http://www.dspguru.com/dsp/howtos/how-to-generate-white-gaussian-noise
  double v1, v2, s;

  do
  {
    v1 = 2 * static_cast<double> (rand()) / rand_max_ - 1.0;
    v2 = 2 * static_cast<double> (rand()) / rand_max_ - 1.0;
    s = v1 * v1 + v2 * v2;
  } while (s >= 1);

  *n1 = sqrt(-2.0 * log(s) / s) * v1;
  if (n2)
    *n2 = sqrt(-2.0 * log(s) / s) * v2;
}


TfDistort::TfDistort() :
  nh_(""), pnh_("~")
{
  srand(ros::Time::now().toNSec());

  // bring up dynamic reconfigure
  reconf_srv_ = new ReconfServer(pnh_);
  reconf_srv_->setCallback(boost::bind(&TfDistort::reconfCb, this, _1, _2));

  pub_thread_ = boost::thread(&TfDistort::pubThread, this);
}

TfDistort::~TfDistort()
{
  delete reconf_srv_;
}


void TfDistort::addNoise(tf::StampedTransform & tf)
{
  const double deg2rad = M_PI / 180.0;
  static ros::Time last_time = ros::Time(0);
  static bool once = false;

  if (!once)
  {
    once = true;
    last_time = tf.stamp_;
    return;
  }

  double dt = (tf.stamp_-last_time).toSec();
  double noise_x = 0, noise_y = 0, noise_z = 0, noise_roll = 0, noise_pitch = 0, noise_yaw = 0;

  if (config_.noise_type == vicon_bridge::tf_distort_NORMAL)
  {
    whiteGaussianNoise(&noise_roll, &noise_pitch);
    whiteGaussianNoise(&noise_yaw, &noise_x);
    whiteGaussianNoise(&noise_y, &noise_z);

    noise_roll *= config_.sigma_roll_pitch * deg2rad;
    noise_pitch *= config_.sigma_roll_pitch * deg2rad;
    noise_yaw *= config_.sigma_yaw * deg2rad;

    noise_x *= config_.sigma_xy;
    noise_y *= config_.sigma_xy;
    noise_z *= config_.sigma_z;
  }
  else if (config_.noise_type == vicon_bridge::tf_distort_UNIFORM)
  {
    noise_roll = uniformNoise(config_.sigma_roll_pitch * deg2rad);
    noise_pitch = uniformNoise(config_.sigma_roll_pitch * deg2rad);
    noise_yaw = uniformNoise(config_.sigma_yaw * deg2rad);
    noise_x = uniformNoise(config_.sigma_xy);
    noise_y = uniformNoise(config_.sigma_xy);
    noise_z = uniformNoise(config_.sigma_z);
  }

  noise_x += random_walk_x_.update(dt) * config_.random_walk_k_xy;
  noise_y += random_walk_y_.update(dt) * config_.random_walk_k_xy;
  noise_z += random_walk_z_.update(dt) * config_.random_walk_k_z;

  tf::Vector3 & p = tf.getOrigin();
  p.setX(p.x() + noise_x);
  p.setY(p.y() + noise_y);
  p.setZ(p.z() + noise_z);

  tf::Quaternion q;
  q.setRPY(noise_roll, noise_pitch, noise_yaw);
  tf.setRotation(tf.getRotation() * q);

  last_time = tf.stamp_;
}


void TfDistort::reconfCb(Config & config, uint32_t level)
{
  if(tf_cb_.connected())
    tf_cb_.disconnect();

  pub_period_.fromSec(1.0/config.tf_pub_rate);
  delay_.fromNSec(config.delay*1e6);

  random_walk_x_.configure(config.random_walk_sigma_xy, config.random_walk_tau_xy);
  random_walk_y_.configure(config.random_walk_sigma_xy, config.random_walk_tau_xy);
  random_walk_z_.configure(config.random_walk_sigma_z, config.random_walk_tau_z);

  if(config.tf_publish && !tf_cb_.connected()){
    tf_cb_ = tf_listener_.addTransformsChangedListener(boost::bind(&TfDistort::tfCb, this));
  }

  if(config.tf_frame_out != config_.tf_frame_out){
    boost::mutex::scoped_lock lock(pose_pub_mutex_);
    ros::NodeHandle nh;
    pose_pub_.reset(new ros::Publisher);
    *pose_pub_= nh.advertise<geometry_msgs::TransformStamped>(config.tf_frame_out, 1);
  }

  config_ = config;
}


void TfDistort::tfCb()
{
  tf::StampedTransform pose;
  static tf::StampedTransform last_pose;
  ros::Time tf_time(0);
  ros::Time time_now = ros::Time::now();

  if (tf_listener_.canTransform(config_.tf_ref_frame, config_.tf_frame_in, tf_time))
  {
    tf_listener_.lookupTransform(config_.tf_ref_frame, config_.tf_frame_in, tf_time, pose);

    // ckeck if new pose
    if (pose.getOrigin().x() != last_pose.getOrigin().x())
    {
      last_pose = pose;
      if (time_now - last_pub_time_ > pub_period_)
      {
        pose.child_frame_id_ = config_.tf_frame_out;
        boost::mutex::scoped_lock(tf_queue_mutex_);
        //        addNoise(pose);
        tf_queue_.push(DelayedTransform(pose, time_now + delay_));
        last_pub_time_ = time_now;
      }

    }
  }
}

void TfDistort::pubThread()
{
  ros::Duration d(0.005);
  uint32_t cnt = 0;
  uint32_t msg_cnt = 0;
  geometry_msgs::TransformStamped pose;

  while (nh_.ok())
  {
    cnt++;
    d.sleep();
    boost::mutex::scoped_lock lock(tf_queue_mutex_);

    if (tf_queue_.empty())
      continue;

    DelayedTransform & dt = tf_queue_.front();

    if (std::abs(ros::Time::now().toSec() - dt.time_to_publish.toSec()) < d.toSec() * 0.75
        || dt.time_to_publish.toSec() - ros::Time::now().toSec() < 0)
    {
      addNoise(dt.transform);
      tf_broadcaster.sendTransform(dt.transform);
      tf::transformStampedTFToMsg(dt.transform, pose);
      {
        boost::mutex::scoped_try_lock lock(pose_pub_mutex_);
        if(lock.owns_lock())
          pose_pub_->publish(pose);
      }

      tf_queue_.pop();
      msg_cnt++;
    }
    if (cnt > 1.0 / d.toSec())
    {
      ROS_INFO("queue size: %d publishing at %d Hz", tf_queue_.size(), msg_cnt);
      msg_cnt = 0;
      cnt = 0;
    }
  }
}

}// end namespace tf_distort

int main(int argc, char** argv)
{

  ros::init(argc, argv, "tf_distort"/*, ros::init_options::AnonymousName*/);

  tf_distort::TfDistort tfd;

  ros::spin();

  return 0;
}


