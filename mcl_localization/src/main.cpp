#include "get_map.h"
#include "kdtree.h"
#include "mcl.h"
#include "mcl_sub.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "run_mcl");
	ros::NodeHandle nh_;
/*	ros::Rate loop_rate(10);
	map_class mapclass(&nh_);
	while(ros::ok())
	{
		vector<vector<double> > occMap = mapclass.occupiedMatrix;
		if(occMap.size() > 0)
		{
			cout<<"exiting"<<endl;
			break;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	vector<vector<double> > occMap = mapclass.occupiedMatrix;
	cout<<"occMap size: "<<occMap.size()<<endl;*/

	mcl mclClass(&nh_);

	ros::spin();
	return 0;
}