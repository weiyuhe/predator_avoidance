#include "mcl_sub.h"

mclSub::mclSub(ros::NodeHandle* nodehandle):nh_(*nodehandle),monte(&nh_)
{
	monte.init();
	odom_sub.subscribe(nh_, "odom", 1);
	scan_sub.subscribe(nh_, "scan", 1);
	sync_.reset(new Sync(MySyncPolicy(10), odom_sub, scan_sub));
	sync_->registerCallback(boost::bind(&mclSub::callback, this, _1, _2));

/*	mcl monte(&nh_);
	monte.init();
	pArray = monte.Particles;
	cout<<"number of particles: "<<pArray.size()<<endl;*/
}

void mclSub::callback(const nav_msgs::Odometry::ConstPtr& odom, const sensor_msgs::LaserScan::ConstPtr& scan)
{
	//ROS_INFO(" Inside Synchronizer Callback");
	//cout<<"odom timestamp: "<< odom->header.stamp.toSec()<<" scan timestamp: "<<scan->header.stamp.toSec()<<endl;
	monte.predictionUpdate(odom);
	monte.measurementUpdate(scan);
	monte.resampling();
	monte.publish_odom();
	//cout<<"scan range size"<<scan->ranges.size()<<endl;
}