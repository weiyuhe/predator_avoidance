#include <ros/ros.h>
#include <vector>

class mcl
{
private:

public:
	mcl();
	void init();
	void predictionUpdate();
	void measurementUpdate();
	void resampling();
	void odom_to_map();

};