
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "std_msgs/Header.h"
#include <visualization_msgs/Marker.h>
#include <vector>


using namespace std;


class map_class
{
private:
	ros::Publisher marker_pub;
	ros::Subscriber map_sub;
	ros::NodeHandle n;
	//visualization_msgs::Marker points;
	//vector<pointxy> occupied_points;
	int occupied;

public:
	map_class();
	
	void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

	void pub_points(visualization_msgs::Marker points, float scale);

	vector<vector<double> > occupiedMatrix;

};
