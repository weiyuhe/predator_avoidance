#include <ros/ros.h>
#include <vector>
#include "kdtree.h"

class mcl
{
private:
	vector<vector<double> > occupied_map;
	ros::Subscriber laserSub;
	ros::Subscriber odomSub;
	//ros::Publisher odomPub;
	ros::NodeHandle n;
public:
	mcl();
	void init();
	void predictionUpdate();
	void measurementUpdate();
	float likelihood_field_range_finder(float zt, float xt, occupied_map);
	void resampling();
	void odom_to_map();

};