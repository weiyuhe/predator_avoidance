#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "std_msgs/Header.h"
#include <visualization_msgs/Marker.h>
#include <vector>
#include "get_map.h"
#include "kdtree.h"

#define X_OFFSET 9
#define Y_OFFSET 9.95

map_class::map_class()
{
	occupied = 0;
	map_sub = n.subscribe("map",2000, &map_class::map_callback,this);
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
}

void map_class::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	std_msgs::Header header = msg->header;
	nav_msgs::MapMetaData info = msg->info;
	int x;
	int y;
	int count = 0;
	ROS_INFO("Got map %d %d", info.width, info.height);

	//loop through the map data and store the data in occupied_points.
	for (x = 0; x < info.width; x++)
	{
		for (y = 0; y < info.height; y++)
		{
			if (msg->data[x+ info.width * y] == -1 || msg->data[x+ info.width * y] == 100)
			{

				occupied++;
				
				//ROS_INFO("Number of occupied is: %i", occupied);
				//ROS_INFO("map data index: ", msg->data[x+ info.width * y]);
				

				vector<double> row;
				row.push_back(x*0.1 - X_OFFSET);
				row.push_back(y*0.1 - Y_OFFSET);
				occupiedMatrix.push_back(row);

				geometry_msgs::Point p;
				p.x = occupiedMatrix[count][0];
				p.y = occupiedMatrix[count][1];
				p.z = 0.1;
				points.points.push_back(p);

				count += 1;

			}
		}
	}
	// for visualizing points
	while(ros::ok())
	{
		pub_points();
	}

}

void map_class::pub_points()
{
	points.header.frame_id  = "/map";
	points.header.stamp = ros::Time::now();
	points.ns = "points";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w  = 1.0;
	//points.id = 0;
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = 0.05;
	points.scale.y = 0.05;
	points.color.g = 1.0f;
	points.color.a = 1.0;
	marker_pub.publish(points);
	//ROS_INFO("publishing points");
	//ROS_INFO("Number of occupied is: %i", occupied);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "get_map");
	 vector<vector<double> > test_map{	{ 1, 2 },
										{ 4, 5 },
										{ 7, 4 },
										{ 3, 8 },
										{ 8, 9 },
										{10, 15},
										{ 2, 7 },
										{20, 18}};
	//map_class mapclass;
	/*kdtree mytree;
	kdtree.construct(map);*/
	kdtree mytree;
	mytree.construct(test_map);

	ros::spin();
	return 0;
}