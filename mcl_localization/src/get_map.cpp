#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "std_msgs/Header.h"
#include <visualization_msgs/Marker.h>
#include <vector>

using namespace std;

struct pointxy
{
    double x, y;

    pointxy(double paramx, double paramy) : x(paramx), y(paramy) {}
};

class map_class{
private:
  ros::Publisher marker_pub;
  ros::Subscriber map_sub;
  ros::NodeHandle n;
  visualization_msgs::Marker points;
  vector<pointxy> occupied_points;
  int occupied;

public:
  map_class(){
    occupied = 0;
    map_sub = n.subscribe("map",2000, &map_class::map_callback,this);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  }

  void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    std_msgs::Header header = msg->header;
    nav_msgs::MapMetaData info = msg->info;
    int x;
    int y;
    int count = 0;
    ROS_INFO("Got map %d %d", info.width, info.height);
    for (x = 0; x < info.width; x++){
      for (y = 0; y < info.height; y++){
        if (msg->data[x+ info.width * y] == -1 || msg->data[x+ info.width * y] == 100){

          occupied++;
          occupied_points.push_back(pointxy(x*0.1 - 9,y*0.1 - 9.95));
          geometry_msgs::Point p;
          p.x = occupied_points[count].x;
          p.y = occupied_points[count].y;
          p.z = 0.1;
          points.points.push_back(p);
          //ROS_INFO("Number of occupied is: %i", occupied);
          //ROS_INFO("map data index: ", msg->data[x+ info.width * y]);
          count += 1;
        }
      }
    }

      while(ros::ok()){
        pub_points();
      }

  }

  void pub_points(){
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


};

int main(int argc, char **argv){
  ros::init(argc, argv, "get_map");

  map_class mapclass;

  ros::spin();
  return 0;
}