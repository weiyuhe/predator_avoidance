#ifndef MCL_H
#define MCL_H

#include <ros/ros.h>
#include <vector>
#include <random>
#include <cmath>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <random>
#include "kdtree.h"
#include "get_map.h"

using namespace std;
struct particle
{
	float x; //in meter
	float y;
	float theta; // in rad/s
	float weight;
};

class mcl
{
private:
	vector<vector<double> > occupied_map;
	ros::Publisher vizPoint_pub;
	ros::Publisher vizLine_pub;
	//ros::Publisher odomPub;
	ros::NodeHandle n_;
	int num_particles;
	const float xmin;
	const float xmax;
	const float ymin;
	const float ymax;
	const float m_per_pixel;
	//visualization_msgs::Marker visPoints;
	//visualization_msgs::Marker visLines;
	std::random_device rd;
	std::mt19937 gen;
public:
	mcl();
	mcl(ros::NodeHandle* nodehandle);
	void init();
	float getRand(float min, float max);
	void predictionUpdate(const nav_msgs::Odometry::ConstPtr& odom);
	void measurementUpdate();
	double likelihood_field_range_finder(float zt, float xt, vector<vector<double> > map);
	void resampling();
	void odom_to_map();
	void visulizePoint(visualization_msgs::Marker points);
	void visulizeLine(visualization_msgs::Marker line_list);
	float normalize(float angle);
	vector<vector<double> > occMap;
	vector<particle> Particles;
	vector<float> last_pose;

};

#endif