#include <iostream>
#include <vector>
#include <cmath>
#include <iterator>
#include <algorithm>

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
	void construct(vector<vector<double> > map);
	Node* construcRecursive( vector<vector<double> > sub_map, int depth);
	/*void kdtree::insert(pointxy point);
	Node *kdtree::insert_recursive(Node *node, pointxy point, int depth);*/
	/*pointxy nearestPoint(pointxy point);*/
/*	bool compareX(const vector<double>& v1, const vector<double>& v2);
	bool compareY(const vector<double>& v1, const vector<double>& v2);*/
	//double distance();
	void delete_recursive(Node *node);
	~kdtree();

	Node *root;
};