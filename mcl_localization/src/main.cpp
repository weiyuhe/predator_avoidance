#include "get_map.h"
#include "kdtree.h"
#include "mcl.h"
#include "mcl_sub.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "run_mcl");
	ros::NodeHandle nh_;


	//mcl mclClass(&nh_);
	mclSub mysub(&nh_);

	ros::spin();
	return 0;
}