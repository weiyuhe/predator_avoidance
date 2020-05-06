#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "std_msgs/Header.h"
#include <visualization_msgs/Marker.h>
#include <vector>
#include "get_map.h"
#include "kdtree.h"

#define X_OFFSET 9
#define Y_OFFSET 9.95

map_class::map_class(ros::NodeHandle* nodehandle):n(*nodehandle)
{
	occupied = 0;
	map_sub = n.subscribe("map",2000, &map_class::map_callback,this);
	cout<<"A"<<endl;
	
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	cout<<"B"<<endl;
}

void map_class::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	cout<<"C"<<endl;
	std_msgs::Header header = msg->header;
	nav_msgs::MapMetaData info = msg->info;
	visualization_msgs::Marker points;
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
				scale = 0.05;

				count += 1;

			}
		}
	}

	cout<<"OccupiedMatrix size: "<<occupiedMatrix.size()<<endl;
	
	/*kdtree mytree;
	mytree.construct(occupiedMatrix);*/
	/*vector<double> testPoint{ 0, 5 };
	NNpoint testNNPoint = mytree.nearestNeighbor(testPoint);
	cout<<" nearest point: x: "<< testNNPoint.nearest_point[0]<<" y: "<<testNNPoint.nearest_point[1]<<endl;
	cout<<" nearest distance: "<< testNNPoint.nearest_dist<<endl;

	visualization_msgs::Marker points1;
	geometry_msgs::Point p1;
	geometry_msgs::Point p2;
	p1.x = testPoint[0];
	p1.y = testPoint[1];
	p2.x = testNNPoint.nearest_point[0];
	p2.y = testNNPoint.nearest_point[1];
	p1.z = p2.z = 0.1;
	points1.points.push_back(p1);
	points1.points.push_back(p2);
	scale = 0.2;*/

	//publish points
	/*while(ros::ok())
	{
		pub_points(points1, scale);
	}*/
}
/*vector<vector<double> > map_class::getMatrix()
{

	return occupiedMatrix;
}*/
void map_class::pub_points(visualization_msgs::Marker points, float scale)
{
	points.header.frame_id  = "/map";
	points.header.stamp = ros::Time::now();
	points.ns = "points";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w  = 1.0;
	//points.id = 0;
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = scale;
	points.scale.y = scale;
	points.color.g = 1.0f;
	points.color.a = 1.0;
	marker_pub.publish(points);
	//ROS_INFO("publishing points");
	//ROS_INFO("Number of occupied is: %i", occupied);
}

