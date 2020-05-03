
#include <vector>
#include "get_map.h"

using namespace std;


struct Node 
{
	Node  *left;
	Node  *right;
	vector<double> xy;
	int axis;
};


class kdtree
{
private:

public:
	kdtree();
	void kdtree::insert(pointxy point);
	Node *kdtree::insert_recursive(Node *node, pointxy point, int depth)
	pointxy nearestPoint(pointxy point);
	void delete_recursive();
	~kdtree();

	Node *root;
};