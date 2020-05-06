#include "mcl_sub.h"

mclSub::mclSub(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
	odom_sub.subscribe(nh_, "odom", 1);
    scan_sub.subscribe(nh_, "scan", 1);
    sync_.reset(new Sync(MySyncPolicy(10), odom_sub, scan_sub));
    sync_->registerCallback(boost::bind(&mclSub::callback, this, _1, _2));

	/*message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/odom", 1); //100?
	message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, "/scan", 1);
	typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry, sensor_msgs::LaserScan> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, scan_sub);
	//message_filters::Synchronizer<nav_msgs::Odometry, sensor_msgs::LaserScan> sync(odom_sub, scan_sub, 10);//50?
	sync.registerCallback(&mclSub::callback, this);*/
}

void mclSub::callback(const nav_msgs::Odometry::ConstPtr& odom, const sensor_msgs::LaserScan::ConstPtr& scan)
{
	//ROS_INFO(" Inside Synchronizer Callback");
	cout<<"odom timestamp: "<< odom->header.stamp.toSec()<<" scan timestamp: "<<scan->header.stamp.toSec()<<endl;
}