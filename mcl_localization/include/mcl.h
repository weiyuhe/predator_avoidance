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
#include <sensor_msgs/LaserScan.h>
#include "kdtree.h"
#include "get_map.h"

using namespace std;
struct particle
{
	float x; //in meter
	float y;
	float theta; // in rad/s
	double weight;
};

class mcl
{
private:
	kdtree mytree;
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
	int downsample_num;
	float map_x_min;
	float map_x_max;
	float map_y_min;
	float map_y_max;
	double ang_min;
	double ang_max;
	double ang_inc;
	float range_max;
	float range_min;
	float step;
	float zhit;
	float zrand;
	float zmax;
	float sigma_hit;
	double total_weight;
public:
	mcl();
	mcl(ros::NodeHandle* nodehandle);
	void init();
	float getRand(float min, float max);
	void predictionUpdate(const nav_msgs::Odometry::ConstPtr& odom);
	void measurementUpdate(const sensor_msgs::LaserScan::ConstPtr& scan);
	double likelihood_field_range_finder(vector<float> zt, particle p);
	void resampling();
	void odom_to_map();
	void visulizePoint(visualization_msgs::Marker points);
	void visulizeLine(visualization_msgs::Marker line_list);
	float normalizeAngle(float angle);
	void normalizeWeight();
	vector<vector<double> > occMap;
	vector<particle> Particles;
	vector<float> last_pose;

};

#endif