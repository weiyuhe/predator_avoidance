#ifndef MCL_SUB_H
#define MCL_SUB_H

#include <ros/ros.h>
#include <vector>
#include <message_filters/subscriber.h>
/*#include <message_filters/time_synchronizer.h>*/
//#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include "mcl.h"

class mclSub
{
private:
	  ros::NodeHandle nh_;
  message_filters::Subscriber<nav_msgs::Odometry> sub_1_;
  message_filters::Subscriber<sensor_msgs::LaserScan> sub_2_;

  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::LaserScan> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;

public:
	mclSub(ros::NodeHandle* nodehandle);
	void callback(const nav_msgs::Odometry::ConstPtr& odom, const sensor_msgs::LaserScan::ConstPtr& scan);

};





#endif