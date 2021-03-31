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
	typedef std::pair<std::vector<float>, int> PointWithId;

	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		PointWithId data(point, id);
		insertHelper(root, 0, data);
	}

	void insertHelper(Node *&node, int level, PointWithId& data)
	{
		const int coordIdx = level % 2;

		if(node == NULL)
		{
			node = new Node(data.first, data.second);
		}
		else if(data.first.at(coordIdx) < node->point.at(coordIdx))
		{
			insertHelper(node->left, level + 1, data);
		}
		else
		{
			insertHelper(node->right, level + 1, data);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper(root, 0, target, distanceTol, ids);

		return ids;
	}

	void searchHelper(Node *&node, const int level, const std::vector<float>& target, const float distanceTol, std::vector<int>& ids)
	{

		if(node == NULL)
		{
			return;
		}

		const int compCoordIdx = level % 2;
		const int otherCoordIdx = (level + 1) % 2;
		const float compCoordNode = node->point.at(compCoordIdx), otherCoordNode = node->point.at(otherCoordIdx);
		const float compCoordTarget = target.at(compCoordIdx), otherCoordTarget = target.at(otherCoordIdx);
		const float absCompCoordDelta = std::abs(compCoordNode - compCoordTarget);
		const float absOtherCoordDelta = std::abs(otherCoordNode - otherCoordTarget);

		const bool followLeft = compCoordTarget < compCoordNode;

		if(absCompCoordDelta <= distanceTol)
		{

			if(absOtherCoordDelta <= distanceTol)
			{
				const float distance = std::sqrt(absCompCoordDelta * absCompCoordDelta + absOtherCoordDelta * absOtherCoordDelta);
				if(distance <= distanceTol)
				{
					ids.push_back(node->id);
				}
			}

			if(followLeft)
			{
				searchHelper(node->right, level + 1, target, distanceTol, ids);
			}
			else
			{
				searchHelper(node->left, level + 1, target, distanceTol, ids);
			}

		}

		if(followLeft)
		{
			searchHelper(node->left, level + 1, target, distanceTol, ids);
		}
		else
		{
			searchHelper(node->right, level + 1, target, distanceTol, ids);
		}

	}

};

struct KdTree3D
{
	typedef std::pair<std::vector<float>, int> PointWithId;

	Node* root;

	KdTree3D()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		PointWithId data(point, id);
		insertHelper(root, 0, data);
	}

	void insertHelper(Node *&node, int level, PointWithId& data)
	{
		const int coordIdx = level % 3;

		if(node == NULL)
		{
			node = new Node(data.first, data.second);
		}
		else if(data.first[coordIdx] < node->point[coordIdx])
		{
			insertHelper(node->left, level + 1, data);
		}
		else
		{
			insertHelper(node->right, level + 1, data);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper(root, 0, target, distanceTol, ids);

		return ids;
	}

	void searchHelper(Node *&node, const int level, const std::vector<float>& target, const float distanceTol, std::vector<int>& ids)
	{

		if(node == NULL)
		{
			return;
		}

		const int compCoordIdx = level % 3;
		const int otherCoordIdx = (level + 1) % 3;
		const int otherCoord2Idx = (level + 2) % 3;

		const float compCoordNode = node->point.at(compCoordIdx);
		const float otherCoordNode = node->point.at(otherCoordIdx);
		const float otherCoord2Node = node->point.at(otherCoord2Idx);

		const float compCoordTarget = target.at(compCoordIdx);
		const float otherCoordTarget = target.at(otherCoordIdx);
		const float otherCoord2Target = target.at(otherCoord2Idx);

		const float absCompCoordDelta = std::abs(compCoordNode - compCoordTarget);
		const float absOtherCoordDelta = std::abs(otherCoordNode - otherCoordTarget);
		const float absOtherCoord2Delta = std::abs(otherCoord2Node - otherCoord2Target);

		const bool followLeft = compCoordTarget < compCoordNode;

		if(absCompCoordDelta <= distanceTol)
		{

			if(absOtherCoordDelta <= distanceTol)
			{
				const float distance = std::sqrt(absCompCoordDelta * absCompCoordDelta + absOtherCoordDelta * absOtherCoordDelta + absOtherCoord2Delta * absOtherCoord2Delta);
				if(distance <= distanceTol)
				{
					ids.push_back(node->id);
				}
			}

			if(followLeft)
			{
				searchHelper(node->right, level + 1, target, distanceTol, ids);
			}
			else
			{
				searchHelper(node->left, level + 1, target, distanceTol, ids);
			}

		}

		if(followLeft)
		{
			searchHelper(node->left, level + 1, target, distanceTol, ids);
		}
		else
		{
			searchHelper(node->right, level + 1, target, distanceTol, ids);
		}

	}

};




