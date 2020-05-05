#include <ros/ros.h>
#include <vector>
#include <random>
#include <cmath>
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
	//ros::Publisher odomPub;
	ros::NodeHandle n_;
	int num_particles;
	vector<particle> particles;
	float xmin;
	float xmax;
	float ymin;
	float ymax;
	float m_per_pixel;
public:
	mcl(ros::NodeHandle* nodehandle);
	void init();
	float getRand(float min, float max);
	void predictionUpdate();
	void measurementUpdate();
	float likelihood_field_range_finder(float zt, float xt, occupied_map);
	void resampling();
	void odom_to_map();

};