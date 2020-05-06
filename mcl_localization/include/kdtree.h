#ifndef KDTREE_H
#define KDTREE_H

#include <iostream>
#include <vector>
#include <cmath>
#include <iterator>
#include <algorithm>
#include <limits>

using namespace std;

struct Node 
{
	Node  *left;
	Node  *right;
	vector<double> xy;
	int axis;
};

struct NNpoint
{
	double nearest_dist;
	vector<double> nearest_point;
};


class kdtree
{
private:
	int leaf_count = 0;
	vector<double> target;


public:
	kdtree();
	void construct(vector<vector<double> > map);
	Node* construcRecursive( vector<vector<double> > sub_map, int depth);
	/*void kdtree::insert(pointxy point);
	Node *kdtree::insert_recursive(Node *node, pointxy point, int depth);*/
	/*pointxy nearestPoint(pointxy point);*/
/*	bool compareX(const vector<double>& v1, const vector<double>& v2);
	bool compareY(const vector<double>& v1, const vector<double>& v2);*/
	double calcDistance(vector<double> A, vector<double> B);
	NNpoint nearestNeighbor(vector<double> target_point);
	NNpoint nearestRecursive(Node *node, double max_dist);
	void delete_recursive(Node *node);
	~kdtree();

	Node *root;
};

#endif