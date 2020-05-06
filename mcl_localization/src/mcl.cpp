#include "mcl.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define X_OFFSET 9
#define Y_OFFSET 9.95

mcl::mcl(ros::NodeHandle* nodehandle):n_(*nodehandle),xmin(0),xmax(180),ymin(0),ymax(200),m_per_pixel(0.1)
{
	/*laserSub = n_.subscribe("scan",2000, &mcl::laserCallback,this);
	odomSub = n_.subscribe("odom",2000, &mcl::odomCallback,this);*/
	vizPoint_pub = n_.advertise<visualization_msgs::Marker>("mcl_points", 10);
	vizLine_pub = n_.advertise<visualization_msgs::Marker>("mcl_liness", 10);
	num_particles = 500;
	//TODO: load map and kdtree
	
	//loadmap_kdtree();
	ros::Rate loop_rate(10);
	map_class getmap(&n_);
	while(ros::ok())
	{
		occMap = getmap.occupiedMatrix;
		if(occMap.size() > 0)
		{
			cout<<"exiting"<<endl;
			break;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	//occMap = getmap.occupiedMatrix;
	kdtree mytree;
	cout<<"occMap size is : "<<occMap.size()<<endl;
	mytree.construct(occMap);

	vector<double> testPoint{ -5, -2 };
	NNpoint testNNPoint = mytree.nearestNeighbor(testPoint);
	cout<<" nearest point: x: "<< testNNPoint.nearest_point[0]<<" y: "<<testNNPoint.nearest_point[1]<<endl;
	cout<<" nearest distance: "<< testNNPoint.nearest_dist<<endl;

	
	geometry_msgs::Point p1;
	geometry_msgs::Point p2;
	p1.x = testPoint[0];
	p1.y = testPoint[1];
	p2.x = testNNPoint.nearest_point[0];
	p2.y = testNNPoint.nearest_point[1];
	p1.z = p2.z = 0.1;
	visPoints.points.push_back(p1);
	visPoints.points.push_back(p2);
	init();
	/*while(ros::ok())
	{
		vizPoint_pub.publish(visPoints);
		//cout<<"publishing"<<endl;
	}*/

}

float mcl::getRand(float min, float max)
{
	static default_random_engine generator;
	uniform_real_distribution<float> distribution(min, max);
	float number = distribution(generator); //in pixel
	return number;

}

void mcl::init()
{
	for(int i = 0; i < num_particles; i++)
	{
		particle p;
		p.x = getRand(xmin, xmax) * m_per_pixel - X_OFFSET; //in meter
		p.y = getRand(ymin, ymax) * m_per_pixel - Y_OFFSET; 
		p.theta = getRand(0, 2*M_PI);
		p.weight = 0;
		Particles.push_back(p);

		//convert from euler to quaternion
		/*tf2::Quaternion quat_tf;
		quat_tf.setRPY( 0, 0, p.theta );
		geometry_msgs::Quaternion quat_msg;
		tf2::convert(quat_msg , quat_tf);*/

		//For visuzalization
		geometry_msgs::Point gp;
		gp.x = p.x;
		gp.y = p.y;
		gp.z = 0.2;
		//visPoints.points.push_back(gp);
		visLines.points.push_back(gp);
		gp.x = gp.x + 0.5*cos(p.theta);
		gp.y = gp.y + 0.5*sin(p.theta);
		visLines.points.push_back(gp);

		//cout<<"gp: (%f,%f)"<<gp.x<<gp.y<<endl;

	}

	while(ros::ok())
	{
		visulizePoint(visPoints);
		//visulizeLine(visLines);
	}
}

void mcl::predictionUpdate()
{
	//TODO: increment particles x,y and theta give odom, store last odom
}

void mcl::measurementUpdate()
{

}

//Algorthm: page 12 on https://people.eecs.berkeley.edu/~pabbeel/cs287-fa12/slides/ScanMatching.pdf
float mcl::likelihood_field_range_finder(float zt, float xt, vector<vector<double> > map)
{

}

void mcl::resampling()
{

}

void mcl::odom_to_map()
{
	
}

void mcl::visulizePoint(visualization_msgs::Marker points)
{
	points.header.frame_id  = "/map";
	points.header.stamp = ros::Time::now();
	points.ns = "points";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w  = 1.0;
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = 0.1;
	points.scale.y = 0.1;
	points.color.g = 1.0f;
	points.color.a = 1.0;
	vizPoint_pub.publish(points);
}

void mcl::visulizeLine(visualization_msgs::Marker line_list)
{
	line_list.header.frame_id  = "/map";
	line_list.header.stamp = ros::Time::now();
	line_list.ns = "lines";
	line_list.action = visualization_msgs::Marker::ADD;
	line_list.pose.orientation.w  = 1.0;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.scale.x = 0.02;
	line_list.color.g = 1.0f;
	line_list.color.a = 1.0;
	vizLine_pub.publish(line_list);
}