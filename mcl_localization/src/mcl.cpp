#include "mcl.h"


#define X_OFFSET 9 //map offset, meter
#define Y_OFFSET 9.95
#define GUESS_FACTOR 0.10 //use to restrict initial location of particles
#define LIDAR_X_OFFSET 0.15 //lidar's x offset w.r.t base_link , in meter

mcl::mcl():xmin(0),xmax(180),ymin(0),ymax(200),m_per_pixel(0.1)
{}

mcl::mcl(ros::NodeHandle* nodehandle):n_(*nodehandle),xmin(0),xmax(180),ymin(0),ymax(200),m_per_pixel(0.1)
{

	vizPoint_pub = n_.advertise<visualization_msgs::Marker>("mcl_points", 10);
	vizLine_pub = n_.advertise<visualization_msgs::Marker>("mcl_liness", 10);
	odomPub = n_.advertise<nav_msgs::Odometry>("/pf/odom", 50);

	downsample_num = 10;
	num_particles = 180;
	ros::Rate loop_rate(10);
	zhit = 0.85;
	zrand = 0.10;
	zmax = 0.05;
	sigma_hit = 0.9;

	gen.seed(rd());

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
	cout<<"occMap size is : "<<occMap.size()<<endl;
	mytree.construct(occMap);

}

float mcl::getRand(float min, float max)
{
	//static default_random_engine generator;
	uniform_real_distribution<float> distribution( min, max);
	float number = distribution(gen); //in pixel
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
		p.x = GUESS_FACTOR * p.x;
		p.y = GUESS_FACTOR * p.y;
		p.theta = getRand(0, 2*M_PI);
		p.weight = 0.0;
		Particles.push_back(p);

		//For visuzalization
/*		geometry_msgs::Point gp;
		gp.x = p.x;
		gp.y = p.y;
		gp.z = 0.2;
		//visPoints.points.push_back(gp);
		visLines.points.push_back(gp);
		gp.x = gp.x + 0.5*cos(p.theta);
		gp.y = gp.y + 0.5*sin(p.theta);
		visLines.points.push_back(gp);*/

	}
	map_x_min = xmin * m_per_pixel - X_OFFSET;
	map_x_max = xmax * m_per_pixel - X_OFFSET;
	map_y_min = ymin * m_per_pixel - Y_OFFSET;
	map_y_max = ymax * m_per_pixel - Y_OFFSET;
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
	rot1 = normalizeAngle(rot1);
	if(tran < 0.01) //if the robot is rotating only
	{
		rot1 = 0.0;
	}
	float rot2 = pose[2] - last_pose[2] - rot1;
	rot2 = normalizeAngle(rot2);
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

		Particles[i].x += tran * cos(normalizeAngle(Particles[i].theta + rot1_noised));
		Particles[i].y += tran * sin(normalizeAngle(Particles[i].theta + rot1_noised));
		Particles[i].theta += (rot1_noised + rot2_noised);
		Particles[i].theta = normalizeAngle(Particles[i].theta);
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
	//visulizePoint(visPoints);

	last_pose = pose;
}

void mcl::measurementUpdate(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	ang_min = scan->angle_min;
	ang_max = scan->angle_max;
	ang_inc = scan->angle_increment;
	range_max = scan->range_max;
	range_min = scan->range_min;
	int size = scan->ranges.size();
	step = size/downsample_num;

	//subsample the incoming scan
	vector<float > downsample_range; //meter
	vector<float > downsample_angle; //radian
	for(int i = 0; i< size; i+=size/downsample_num)
	{
		downsample_range.push_back(scan->ranges[i]);
		downsample_angle.push_back(ang_min + i * ang_inc);
	}

	total_weight = 0.0;
	for(int i = 0; i < num_particles; i++)
	{
		float weight = likelihood_field_range_finder(downsample_range, Particles[i]);
		Particles[i].weight = weight;
		total_weight += weight;
	}

	normalizeWeight();
}

//Algorthm: page 12 on https://people.eecs.berkeley.edu/~pabbeel/cs287-fa12/slides/ScanMatching.pdf
float mcl::likelihood_field_range_finder(vector<float> zt, particle p)
{
	double q = 0;
	//position of the Lidar in the map frame
	float lidar_x = p.x + (LIDAR_X_OFFSET * cos(p.theta));
	float lidar_y = p.y + (LIDAR_X_OFFSET * sin(p.theta));
	float lidar_theta = p.theta;

	if(lidar_x <= map_x_min || lidar_x >= map_x_max || lidar_y <= map_y_min || lidar_y >= map_y_max) //lidar is outside the map
	{
		return 0.0;
	}

	for(int i = 0; i < zt.size(); i++)
	{
		if(zt[i] > range_max || zt[i] < range_min)
		{
			continue;
		}

		float theta_map = i *  step * ang_inc + ang_min + lidar_theta; 
		double end_x = lidar_x + zt[i] * cos(theta_map);
		double end_y = lidar_y + zt[i] * sin(theta_map);

		vector<double> findpoint{ end_x, end_y };
		NNpoint closePoint = mytree.nearestNeighbor(findpoint);
		float dx = end_x - closePoint.nearest_point[0];
		float dy = end_y - closePoint.nearest_point[1];
		float dist = sqrt(dx * dx + dy * dy);

		float q_hit = zhit * exp(-(dist*dist)/(2.0*sigma_hit*sigma_hit)) / (sigma_hit * sqrt(2.0*M_PI));
		float q_rand = zrand * 1.0/range_max;
		float q_max = zt[i] == range_max ? zmax : 0;

		q = q + q_hit + q_rand + q_max;
	}

	return q;
}

//using sampling wheel technique; Low variance sampling will also work
void mcl::resampling()
{
	vector<particle> newParticles;
	int index = getRand(0, num_particles);
	float beta = 0.0;

	for(int i = 0 ; i < num_particles; i++)
	{
		beta += getRand(0, 2*max_weight);
		while(beta>Particles[index].weight)
		{
			beta -= Particles[index].weight;
			index = (index + 1) % num_particles;
		}

		particle p = Particles[index];
		newParticles.push_back(p);
	}

	Particles = newParticles;

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
	line_list.scale.x = 0.05;
	line_list.color.g = 1.0f;
	line_list.color.a = 1.0;
	vizLine_pub.publish(line_list);
}

float mcl::normalizeAngle(float angle)
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

void mcl::normalizeWeight()
{
	max_weight = 0;

	for(int i = 0; i < num_particles; i++)
	{
		Particles[i].weight /= total_weight;
		if(Particles[i].weight > max_weight)
		{
			max_weight = Particles[i].weight;
		}
	}
}

void mcl::publish_odom()
{
	double x = 0.0;
	double y = 0.0;
	double theta  = 0.0;
	for(int i = 0; i < num_particles; i++)
	{
		x = x + Particles[i].x * Particles[i].weight;
		y = y + Particles[i].y * Particles[i].weight;
		theta = theta + Particles[i].theta * Particles[i].weight;
	}

	tf::Transform transform;
	transform.setOrigin( tf::Vector3(x, y, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, theta);
	transform.setRotation(q);
	geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(theta);

	nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "/map";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = quat;
    odomPub.publish(odom);
}