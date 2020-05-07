#include "mcl.h"


#define X_OFFSET 9
#define Y_OFFSET 9.95

mcl::mcl():xmin(0),xmax(180),ymin(0),ymax(200),m_per_pixel(0.1)
{}

mcl::mcl(ros::NodeHandle* nodehandle):n_(*nodehandle),xmin(0),xmax(180),ymin(0),ymax(200),m_per_pixel(0.1)
{

	vizPoint_pub = n_.advertise<visualization_msgs::Marker>("mcl_points", 10);
	vizLine_pub = n_.advertise<visualization_msgs::Marker>("mcl_liness", 10);
	gen.seed(rd());
	num_particles = 600;
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
/*
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
	visPoints.points.push_back(p2);*/
	//init();
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
	visualization_msgs::Marker visPoints;
	visualization_msgs::Marker visLines;
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

	/*while(ros::ok())
	{
		visulizePoint(visPoints);
		//visulizeLine(visLines);
	}*/
}

//sample motion model: p24 on http://ais.informatik.uni-freiburg.de/teaching/ss11/robotics/slides/06-motion-models.pdf
void mcl::predictionUpdate(const nav_msgs::Odometry::ConstPtr& odom)
{
	visualization_msgs::Marker visPoints;
	visualization_msgs::Marker visLines;
	//TODO: increment particles x,y and theta give odom, store last odom
	float x = odom->pose.pose.position.x;
	float y = odom->pose.pose.position.y;
	tf::Quaternion quat(odom->pose.pose.orientation.x, 
						odom->pose.pose.orientation.y, 
						odom->pose.pose.orientation.z, 
						odom->pose.pose.orientation.w);
	tf::Matrix3x3 mat(quat);
	double roll, pitch, yaw;
	mat.getRPY(roll, pitch, yaw);
	vector<float> pose{ x , y ,(float)yaw };

	if(last_pose.size() == 0)
	{
		cout<<" Receiving first odometry."<<endl;
		last_pose = pose;
		return;
	}
	
	float delta_x = x - last_pose[0];
	float delta_y = y - last_pose[1];
	float tran = sqrt(delta_y*delta_y + delta_x*delta_x);

	//handle rotations
	float rot1 = atan2(delta_y,delta_x) - last_pose[2];
	rot1 = normalize(rot1);
	if(tran < 0.01) //if the robot is rotating only
	{
		rot1 = 0.0;
	}
	float rot2 = pose[2] - last_pose[2] - rot1;
	rot2 = normalize(rot2);
	//cout<<"rot1: "<<rot1<<endl;
	//cout<<"rot2: "<<rot2<<endl;

	//Motion noise parameters: [alpha1, alpha2, alpha3, alpha4]
	vector<float> alphas{0.1, 0.1, 0.05, 0.05};
	float sigma_rot1 = alphas[0] * fabs(rot1) + alphas[1] * tran;
	float sigma_trans = alphas[2] * tran + alphas[3] * (fabs(rot1) + fabs(rot2));
	float sigma_rot2 = alphas[0] * fabs(rot2) + alphas[1] * tran;

	for(int i = 0; i < num_particles; i++)
	{
		normal_distribution<float> norm_trans(0,sigma_trans);
		normal_distribution<float> norm_rot1(0,sigma_rot1);
		normal_distribution<float> norm_rot2(0,sigma_rot2);

		float tran_noised = tran + norm_trans(gen); 
		float rot1_noised = rot1 + norm_rot1(gen);
		float rot2_noised = rot2 + norm_rot2(gen);
		//cout<<"tran_noised: "<<tran_noised<<endl;
		//cout<<"Particles.x: "<<Particles[i].x<<endl;

		Particles[i].x += tran * cos(normalize(Particles[i].theta + rot1_noised));
		Particles[i].y += tran * sin(normalize(Particles[i].theta + rot1_noised));
		Particles[i].theta += (rot1_noised + rot2_noised);
		Particles[i].theta = normalize(Particles[i].theta);
		geometry_msgs::Point gp;
		gp.x = Particles[i].x;
		gp.y = Particles[i].y;
		gp.z = 0.2;
		//visPoints.points.push_back(gp);
		visLines.points.push_back(gp);
		gp.x = gp.x + 0.5*cos(Particles[i].theta);
		gp.y = gp.y + 0.5*sin(Particles[i].theta);
		visLines.points.push_back(gp);
	}
	visulizeLine(visLines);

	last_pose = pose;
}

void mcl::measurementUpdate()
{

}

//Algorthm: page 12 on https://people.eecs.berkeley.edu/~pabbeel/cs287-fa12/slides/ScanMatching.pdf
double mcl::likelihood_field_range_finder(float zt, float xt, vector<vector<double> > map)
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

float mcl::normalize(float angle)
{
	if(angle < -M_PI)
	{
		return angle + 2.0 * M_PI;
	}
	else
	{
		return angle > M_PI ? angle - 2.0 * M_PI : angle;
	}
}