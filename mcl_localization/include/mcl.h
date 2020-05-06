#ifndef MCL_H
#define MCL_H

#include <ros/ros.h>
#include <vector>
#include <random>
#include <cmath>
#include <visualization_msgs/Marker.h>
#include "kdtree.h"

using namespace std;
struct particle
{
	float x; //in mter
	float y;
	float theta; // in rad/s
	float weight;
};

class mcl
{
private:
	vector<vector<double> > occupied_map;
	ros::Subscriber laserSub;
	ros::Subscriber odomSub;
	ros::Publisher vizPoint_pub;
	ros::Publisher vizLine_pub;
	//ros::Publisher odomPub;
	ros::NodeHandle n_;
	int num_particles;
	vector<particle> Particles;
	const float xmin;
	const float xmax;
	const float ymin;
	const float ymax;
	const float m_per_pixel;
	visualization_msgs::Marker visPoints;
	visualization_msgs::Marker visLines;
public:
	mcl();
	mcl(ros::NodeHandle* nodehandle);
	void init();
	float getRand(float min, float max);
	void predictionUpdate();
	void measurementUpdate();
	float likelihood_field_range_finder(float zt, float xt, vector<vector<double> > map);
	void resampling();
	void odom_to_map();
	void visulizePoint(visualization_msgs::Marker points);
	void visulizeLine(visualization_msgs::Marker line_list);

};

#endif