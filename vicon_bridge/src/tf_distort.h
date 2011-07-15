/*
 * TfDistort.h
 *
 *  Created on: Jun 8, 2011
 *      Author: acmarkus
 */

#ifndef TFDISTORT_H_
#define TFDISTORT_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <vicon_bridge/tf_distortConfig.h>

#include <queue>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

namespace tf_distort{

typedef dynamic_reconfigure::Server<vicon_bridge::tf_distortConfig> ReconfServer;
typedef vicon_bridge::tf_distortConfig Config;

class DelayedTransform
{
public:
  DelayedTransform();
  DelayedTransform(const tf::StampedTransform & _transform, const ros::Time & _time) :
    transform(_transform), time_to_publish(_time)
  {
  }
  ;
  tf::StampedTransform transform;
  ros::Time time_to_publish;
};

typedef std::queue<DelayedTransform> DelayedTransformQueue;

const double rand_max_ = RAND_MAX;
void whiteGaussianNoise(double * n1, double * n2 = NULL);

class RandomWalk
{
private:
  double x_;
  double sigma_gm_;
  double tau_;
public:
  RandomWalk() :
    x_(0), sigma_gm_(0), tau_(0)
  {
    //srand(time(NULL));
  }
  void configure(const double & sigma_gm, const double & tau)
  {
    sigma_gm_ = sigma_gm;
    tau_ = tau;
  }
  double update(const double & dt)
  {
    double n;
    whiteGaussianNoise(&n);
    x_ = n * sqrt(1.0 - exp(-dt / tau_)) + x_ * exp(-dt / tau_);
    return x_;
  }
  double get()
  {
    return x_;
  }
};

class TfDistort
{
private:

  ReconfServer* reconf_srv_;
  ros::NodeHandle nh_, pnh_;
  Config config_;

  ros::Duration pub_period_;
  ros::Duration delay_;
  ros::Time last_pub_time_;

  DelayedTransformQueue tf_queue_;
  boost::mutex tf_queue_mutex_;

  boost::signals::connection tf_cb_;
  tf::TransformBroadcaster tf_broadcaster;
  tf::TransformListener tf_listener_;
  boost::mutex pose_pub_mutex_;
  boost::shared_ptr<ros::Publisher> pose_pub_; ///< pose publisher for convenience


  boost::thread pub_thread_;

  RandomWalk random_walk_x_;
  RandomWalk random_walk_y_;
  RandomWalk random_walk_z_;

  void pubThread();
  void tfCb();
  void reconfCb(Config & config, uint32_t level);
  void addNoise(tf::StampedTransform & transform);
  inline double uniformNoise(const double & mag = 1)
  {
    // TODO: use noise::uni() or noise::uniform() ???
    return (static_cast<double> (rand()) / rand_max_ * 2.0 - 1.0) * mag;
  }

public:
  TfDistort();
  ~TfDistort();
  void poll();
};

}// end namespace tf_distort

#endif /* TFDISTORT_H_ */
