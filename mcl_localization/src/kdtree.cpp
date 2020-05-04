#include "kdtree.h"


kdtree::kdtree()
{
	root = nullptr;
}

void kdtree::construct(vector<vector<double> > map)
{
	
	int size = map.size();
	int midIndex = size / 2;
	//sort(map.begin(), map.end(),&kdtree::compareX);
	sort(map.begin(), map.end(), [](const vector<double>& v1, const vector<double>& v2)
	{ 
		return v1[0] < v2[0]; 
	});

	vector<vector<double> > left;
	vector<vector<double> > right;
	//vector<int>::iterator middleItr(map.begin() + map.size() / 2);
	vector<vector<double> >::iterator middleItr(map.begin() + midIndex);

	for (auto it = map.begin(); it != map.end(); ++it) 
	{
		if (distance(it, middleItr) > 0) //std::distance
			left.push_back(*it);
		else
			right.push_back(*it);
	}
	right.erase(right.begin());

	if(root == nullptr)
	{
		root = new Node;
		root->axis = 0;
		root->left = construcRecursive( left, 1);
		root->right = construcRecursive( right, 1);
		root->xy =  map[midIndex];
		cout<<"root x: "<<map[midIndex][0]<<" root y: "<<map[midIndex][1]<<endl;
	}
	else
	{
		cout << "Root is not NULL!"<<endl;
		return;
	}

}

Node *kdtree::construcRecursive( vector<vector<double> > sub_map, int depth)
{
	int size = sub_map.size();
	if(size == 1) // leaf, end recursion
	{
		Node *node = new Node;
		node->axis = depth%2;
		node->left = nullptr;
		node->right = nullptr;
		node->xy = sub_map[0];
		leaf_count += 1;
		//cout<<"leaf count: "<< leaf_count<<endl;
		//cout<<"leaf x: "<<sub_map[0][0]<<" leaf y: "<<sub_map[0][1]<<endl;
		return node;
	}
	int dimension = depth%2;
	
	//sort(sub_map.begin(), sub_map.end(),&kdtree::compareX);
	sort(sub_map.begin(), sub_map.end(), [dimension](const vector<double>& v1, const vector<double>& v2)
	{ 
		return v1[dimension] < v2[dimension]; 
	});
	
	
	int midIndex = size / 2;

	vector<vector<double> > left;
	vector<vector<double> > right;
	//vector<int>::iterator middleItr(sub_map.begin() + sub_map.size() / 2);
	vector<vector<double> >::iterator middleItr(sub_map.begin() + midIndex);

	for (auto it = sub_map.begin(); it != sub_map.end(); ++it) {
		if (distance(it, middleItr) > 0) //std::distance
			left.push_back(*it);
		else
			right.push_back(*it);
	}


	if(size == 2)
	{
		Node *node = new Node;
		node->axis = depth%2;
		node->left = construcRecursive(left, depth+1);
		node->right = nullptr;
		node->xy = sub_map[midIndex];
		return node;
	}

    right.erase(right.begin());

	Node *node = new Node;
	node->axis = depth%2;
	node->left = construcRecursive( left, depth + 1);
	node->right = construcRecursive( right, depth + 1);
	node->xy =  sub_map[midIndex];
	return node;
}

//Pseudo algorithm: page 9 on https://www.ri.cmu.edu/pub_files/pub1/moore_andrew_1991_1/moore_andrew_1991_1.pdf
NNpoint kdtree::nearestNeighbor(vector<double> target_point)
{
	target = target_point;
	double max = numeric_limits<double>::max();
	return kdtree::nearestRecursive(root, max);
}

NNpoint kdtree::nearestRecursive(Node *node, double max_dist)
{
	NNpoint nnpoint;
	vector<double> nearestPoint;
	double nearestDist;
	Node *nearestNode;
	Node *furtherNode;


	if(node == nullptr)
	{
		nnpoint.nearest_dist = numeric_limits<double>::max();
		nnpoint.nearest_point = nearestPoint;
		return nnpoint;
	}

	int split = node->axis;
	vector<double> pivot = node->xy;

	if(target[split] <= pivot[split])
	{
		nearestNode = node->left;
		furtherNode = node->right;
	}
	else
	{
		nearestNode = node->right;
		furtherNode = node->left;
	}

	nnpoint = nearestRecursive(nearestNode, max_dist);
	nearestPoint = nnpoint.nearest_point;
	nearestDist = nnpoint.nearest_dist;

	if(nearestDist < max_dist)
	{
		max_dist = nearestDist;
	}

	double check_dist = fabs(pivot[split] - target[split]);
	if(max_dist <= check_dist )// < ?
	{
		return nnpoint;
	}

	double dist = calcDistance(pivot, target);

	if(dist < nearestDist)
	{
		nearestPoint = pivot;
		nearestDist = dist;
		max_dist = nearestDist;
	}

	NNpoint nnpointFar = nearestRecursive(furtherNode, max_dist);
	if(nnpointFar.nearest_dist < nearestDist)
	{
		nearestDist = nnpointFar.nearest_dist;
		nearestPoint = nnpointFar.nearest_point;

	}

	NNpoint result;
	result.nearest_dist = nearestDist;
	result.nearest_point = nearestPoint;
	return result;

}

double kdtree::calcDistance(vector<double> A, vector<double> B)
{
	double dx = A[0] - B[0]; 
	double dy = A[1] - B[1];
	double dist;
	dist = sqrt(pow(dx, 2) + pow(dy, 2));                  
	return dist;
}

kdtree::~kdtree() 
{
	delete_recursive(root); 
}

void kdtree::delete_recursive(Node *node)
{
	if (node != NULL)
	{
		delete_recursive(node->right);
		delete_recursive(node->left);
		delete node;
	}
}