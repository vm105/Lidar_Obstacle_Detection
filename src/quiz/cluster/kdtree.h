/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

namespace my_pcl
{
// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};


struct KdTree
{
	Node* root;
	int dimension;
	KdTree()
	: root(NULL)
	{
		dimension = 2;
	}

	void set_dimension(int new_dimension)
	{
		dimension = new_dimension;
	}

	int get_split_by_index(int depth)
	{
		return (depth % dimension);
	}

	bool within_sphere(std::vector<float> target, std::vector<float> point, float distanceTol)
	{
		float x_dist = point[0] - target[0];
		float y_dist = point[1] - target[1];
		float z_dist = point[2] - target[2];

		return ((x_dist * x_dist) + (y_dist * y_dist) + (z_dist * z_dist)) <= (distanceTol*distanceTol);
	}

	bool within_box(std::vector<float> target, std::vector<float> point, float distanceTol)
	{
		bool within_box = true;
		for (int i = 0; i < target.size(); ++i)
		{
			within_box = within_box && (point[i] <= (target[i] + distanceTol)) && (point[i] >= (target[i] - distanceTol));
		}
		return within_box;	
	}

	Node* insert(Node* node, int depth, std::vector<float> point, int id)
	{
		int index = get_split_by_index(depth);
	
		if (node == nullptr)
		{
			node = new Node(point, id);
		}
		else if (point[index] <= node->point[index])
		{
			node->left = insert(node->left, ++depth, point, id);
		}
		else
		{
			node->right = insert(node->right, ++depth, point, id);
		}

		return node;
	}

	void insert(std::vector<float> point, int id)
	{	
		root = insert(root, 0, point, id);
	}

	void search(std::vector<int>& neighbors, std::vector<float> target, float distanceTol, Node* node, int depth)
	{
		if (node == nullptr)
		{
			return;
		}
		int index = get_split_by_index(depth);

		if (within_box(target, node->point, distanceTol) && within_sphere(target, node->point, distanceTol))
		{
			neighbors.push_back(node->id);
		}
		if ((target[index] - distanceTol) < node->point[index])
		{
			search(neighbors, target, distanceTol, node->left, depth + 1);
		}
		if ((target[index] + distanceTol) > node->point[index])
		{
			search(neighbors, target, distanceTol, node->right, depth + 1);
		}	
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search(ids, target, distanceTol, root, 0);
		return ids;
	}
	
};
}


