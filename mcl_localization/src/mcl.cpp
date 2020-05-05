#include "mcl.h"
#define X_OFFSET 9
#define Y_OFFSET 9.95

mcl::mcl(ros::NodeHandle* nodehandle):n_(*nodehandle),xmin(0),xmax(180),ymin(0),ymax(200),m_per_pixel(0.1)
{
	/*laserSub = n_.subscribe("scan",2000, &mcl::laserCallback,this);
	odomSub = n_.subscribe("odom",2000, &mcl::odomCallback,this);*/
	vizPoint_pub = n_.advertise<visualization_msgs::Marker>("mcl_points", 10);
	num_particles = 1500;

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

		geometry_msgs::Point gp;
		gp.x = p.x;
		gp.y = p.y;
		gp.z = 0.2;
		visPoint.points.push_back(gp);
		//cout<<"gp: (%f,%f)"<<gp.x<<gp.y<<endl;

	}

	while(ros::ok())
	{
		visulizePoint(visPoint);
	}
}

void mcl::predictionUpdate()
{

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