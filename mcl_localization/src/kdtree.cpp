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

	for (auto it = map.begin(); it != map.end(); ++it) {
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
		node->axis = depth;
		node->left = nullptr;
		node->right = nullptr;
		node->xy = sub_map[0];
		cout<<"leaf x: "<<sub_map[0][0]<<" leaf y: "<<sub_map[0][1]<<endl;
		return node;
	}
	else if(size == 2){

	}

	if(depth%2 == 0) //if axis is X
	{
		//sort(sub_map.begin(), sub_map.end(),&kdtree::compareX);
		sort(sub_map.begin(), sub_map.end(), [](const vector<double>& v1, const vector<double>& v2)
		{ 
			return v1[0] < v2[0]; 
		});
	}
	else 			//if axis is Y
	{
		//sort(sub_map.begin(), sub_map.end(),&kdtree::compareY);
		sort(sub_map.begin(), sub_map.end(), [](const vector<double>& v1, const vector<double>& v2)
		{ 
			return v1[1] < v2[1]; 
		});
	}
	
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
		node->axis = depth;
		node->left = construcRecursive(left, depth+1);
		node->right = nullptr;
		node->xy = sub_map[midIndex];
		return node;
	}

	/*vector<vector<double> >::iterator rightIT;
	rightIT = right.begin(); 
	right.erase(rightIT); */

    right.erase(right.begin());

	Node *node = new Node;
	node->axis = depth;
	node->left = construcRecursive( left, depth + 1);
	node->right = construcRecursive( right, depth + 1);
	node->xy =  sub_map[midIndex];
	return node;
}

/*void kdtree::insert(pointxy point)
{
	vector<double> xydata{point.x, point.y}; 
	if(root == NULL)
	{
		root = new Node;
		root->axis = 0;
		root->left = NULL;
		root->right = NULL;
		root->xy =  xydata;
	}
	else
	{
		insert_recursive(root, point, 0);
	}
}

//*node is the prev node that you need to compare
Node *kdtree::insert_recursive(Node *node, pointxy point, int depth){
	vector<double> xydata{point.x, point.y}; 
	if (node == NULL) //creates new node
	{
		node = new Node;
		node->axis = 0;
		node->left = NULL;
		node->right = NULL;
		node->xy =  xydata;
	}
	else
	{

		int axis = depth%2 == 0 ? 0 : 1;
		if(xydata[axis] < node->xy[axis])
		{
			node->left = insert_recursive(node->left, point, node->axis + 1);
		}
		else
		{
			node->right = insert_recursive(node->right, data, node->axis + 1);
		}

	}

	return node;

}*/


/*pointxy kdtree::nearestPoint(pointxy point)
{
	if (root == NULL)
	{
		cout<<"Root is NULL, returning original point! "<<endl;
		return point;
	}



}*/


/*bool kdtree::compareX(const vector<double>& v1, const vector<double>& v2)
{ 
	return v1[0] < v2[0]; 
} 

bool kdtree::compareY(const vector<double>& v1, const vector<double>& v2)
{ 
	return v1[1] < v2[1]; 
} */



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