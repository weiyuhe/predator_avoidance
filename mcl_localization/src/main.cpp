#include "get_map.h"
#include "kdtree.h"
#include "mcl.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "run_mcl");
	ros::NodeHandle nh_;
	//map_class mapclass(&nh_);
	mcl mclClass(&nh_);
	mclClass.init();

	/*vector<vector<double> > test_map{	{ 1, 2 },
										{ 4, 5 },
										{ 7, 4 },
										{ 3, 8 },
										{ 8, 9 },
										{10, 15},
										{ 2, 7 },
										{20, 18}};
	kdtree mytree;
	mytree.construct(test_map);*/

	ros::spin();
	return 0;
}