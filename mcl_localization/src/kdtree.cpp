#include "kdtree.h"


kdtree::kdtree()
{
	root = NULL;
}

void kdtree::insert(pointxy point)
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


}


pointxy kdtree::nearestPoint(pointxy point)
{
	if (root == NULL)
	{
		cout<<"Root is NULL, returning original point! "<<endl;
		return point;
	}



}


kdtree::~kdtree() 
{
	delete_recursive(this->root); 
}

void KDTree::delete_recursive(Node *node)
{
	if (node != NULL)
	{
		delete_recursive(node->right);
		delete_recursive(node->left);
		delete node;
	}
}