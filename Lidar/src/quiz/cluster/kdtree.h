/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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

	~Node()
	{
		delete left;
		delete right;
	}

};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertNodes(root, point, id, 0);

	}

	void insertNodes(Node*& node, const std::vector<float>& point, int id, int depth)
	{
		//Check if to check X or Y value: odd value is X and even is Y value
		auto dimension = depth % 3;
		
		if(node == nullptr)		
		{
			node = new Node(point, id);
		}
		else
		{
			//check if point value is smaller or bigger. If bigger go right, if smaller go left
			if(point[dimension] < (node)->point[dimension])
			{
				insertNodes(node->left, point, id, depth++);
			}
			else
			{
				insertNodes(node->right, point, id, depth++);
			}
		}
	}

	void searchForClosestPoint(Node*& node, uint8_t depth, 
								const std::vector<float>& target, float distanceTol, std::vector<int>& ids)
	{
		
		if(node != nullptr)
		{	
			uint dim = depth % 3;
			// check if point in this node is within distance tolerance from the target
			// in both x and y directions
			if((target[0] >= node->point[0] - distanceTol && target[0] < node->point[0] + distanceTol)
				&& (target[1] >= node->point[1] - distanceTol && target[1] < node->point[1] + distanceTol)
				&& (target[2] >= node->point[2] - distanceTol && target[2] < node->point[2] + distanceTol))
			{
				auto distance = sqrt(pow(node->point[0] - target[0], 2) + pow(node->point[1] - target[1], 2) + pow(node->point[2] - target[2], 2));
				if(distance <= distanceTol)
					ids.emplace_back(node->id);
			}

			// check along the boundaries to where to search next
			if(target[dim] - distanceTol < node->point[dim])
			{
				searchForClosestPoint(node->left, depth++, target, distanceTol, ids);
			}
			if (target[dim] + distanceTol > node->point[dim])
			{
				searchForClosestPoint(node->right, depth++, target, distanceTol, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchForClosestPoint(root, 0, target, distanceTol, ids);

		return ids;
	}
	


};




