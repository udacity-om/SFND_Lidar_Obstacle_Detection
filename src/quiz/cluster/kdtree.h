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
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node** node, unsigned int depth, std::vector<float> point, int id)
	{
		// tree is empty
		if (NULL == *node) {
			*node = new Node(point, id);
		}
		else
		{
			// Calculate current dim
			unsigned cd = depth % 2;

			// If the point is less than the node then create a left branch from current node
			if (point[cd] < ((*node)->point[cd])) {
				insertHelper(&((*node)->left), depth + 1, point, id);
			}// Otherwise create a right branch from current node
			else {
				insertHelper(&((*node)->right), depth + 1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);

	}

	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if (NULL != node) {
			float nodeX = node->point[0];
			float nodeY = node->point[1];
			float targetX = target[0];
			float targetY = target[1];
			/* Check if the node is within the target box. */
			if ((nodeX >= (targetX - distanceTol)) &&
				(nodeX <= (targetX + distanceTol)) &&
				(nodeY >= (targetY - distanceTol)) &&
				(nodeY <= (targetY + distanceTol))) {
				float distance = sqrt((nodeX - targetX)*(nodeX - targetX) +
					(nodeY - targetY)*(nodeY - targetY));

				if (distance <= distanceTol) {
					ids.push_back(node->id);
				}
			}

			// Check across boundary to decide if we want to flow to the right or left of the node in the tree
			if ((target[depth % 2] - distanceTol) < node->point[depth % 2]) {
				searchHelper(target, node->left, depth + 1, distanceTol, ids);
			}
			if ((target[depth % 2] + distanceTol) > node->point[depth % 2]) {
				searchHelper(target, node->right, depth + 1, distanceTol, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper(target, root, 0, distanceTol, ids);

		return ids;
	}
};




