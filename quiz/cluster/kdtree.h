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


	void insert(std::vector<float> point, int id){
			// TODO: Fill in this function to insert a new point into the tree
			// the function should create a new node and place correctly with in the root
			int depth = 0;
			add_node(&root, depth, point, id);
	}

	void add_node(Node** node, int depth, std::vector<float> point, int id){
			if((*node)==NULL)
				(*node)= new Node(point,id);
			else {
				int d = depth%3;
				if(point[d] < (*node)->point[d])
					add_node(&((*node)->left), depth+1, point, id);
				else
				  add_node(&((*node)->right), depth+1, point, id);
				}
			}

					// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
						std::vector<int> ids;
						search_kdtree(&root, 0, ids, distanceTol, target);
			return ids;
	}

	void search_kdtree(Node** node, int depth, std::vector<int> &ids, float dist, const std::vector<float> &target){
		if((*node)!=NULL){
			//std::cout << "Target" << " x y z: " << target[0] << " " << target[1] << " " << target[2] << std::endl;
			if(sqrt(pow((target[0] - (*node)->point[0]),2) +
			        pow((target[1] - (*node)->point[1]),2) +
							pow((target[2] -(*node)->point[2]), 2)) <= dist)
					ids.push_back((*node)->id);

			int d = depth%3;
			if(target[d]-dist < (*node)->point[d])
				search_kdtree((&(*node)->left), depth+1, ids, dist, target);
			if(target[d]+dist > (*node)->point[d])
				search_kdtree((&(*node)->right), depth+1, ids, dist, target);
			}
		}
};
