#include "mcl.h"

mcl::mcl():n_(*nodehandle)
{
	num_particles = 1500;
	xmin = 0;
	xmax = 180; //in pixel
	ymin = 0;
	ymax = 200;
	m_per_pixel = 0.1;
}

float mcl::getRand(float min, float max)
{
	default_random_engine generator;
	uniform_int_distribution<float> distribution(min, max);
	float number = distribution(generator); //in pixel
	return number;

}

void mcl::init()
{
	for(int i = 0; i < num_particles; i++)
	{
		particle p;
		p.x = gerRand(xmin, xmax) * m_per_pixel; //in meter
		p.y = getRand(ymin, ymax) * m_per_pixel; 
		p.theta = getRand(0, 2*M_PI);
		p.weight = 0;
		Particles.push_back(p);
	}
}

void mcl::predictionUpdate()
{

}

void mcl::measurementUpdate()
{

}

float mcl::likelihood_field_range_finder(float zt, float xt, occupied_map)
{

}

void mcl::resampling()
{

}

void mcl::odom_to_map()
{
	
}