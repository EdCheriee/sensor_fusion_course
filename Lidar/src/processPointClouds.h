// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/statistical_multiscale_interest_region_extraction.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include <boost/filesystem.hpp>
#include <Eigen/Eigenvalues>

#include "quiz/cluster/kdtree.h"

// Structure to represent node of kd tree
// template<typename PointT>
// struct Node
// {
// 	PointT point;
// 	int id;
// 	Node* left;
// 	Node* right;

// 	Node(PointT arr, int setId)
// 	:	point(arr), id(setId), left(NULL), right(NULL)
// 	{}

// 	~Node()
// 	{
// 		delete left;
// 		delete right;
// 	}
// };

// template<typename PointT>
// struct KdTree
// {
// 	Node<PointT>* root;

// 	KdTree()
// 	: root(NULL)
// 	{}

// 	~KdTree()
// 	{
// 		delete root;
// 	}

// 	void insert(PointT point, int id)
// 	{
// 		// TODO: Fill in this function to insert a new point into the tree
// 		// the function should create a new node and place correctly with in the root 
// 		insertNodes(root, point, id, 0);

// 	}

// 	void insertNodes(Node<PointT>*& node, const PointT& point, int id, int depth)
// 	{
// 		//Check if to check X or Y value: odd value is X and even is Y value
// 		auto dimension = depth % 3;
		
// 		if(node == nullptr)		
// 		{
// 			node = new Node<PointT>(point, id);
// 		}
// 		else
// 		{
// 			//check if point value is smaller or bigger. If bigger go right, if smaller go left
// 			if(point[dimension] < (node)->point[dimension])
// 			{
// 				insertNodes(node->left, point, id, depth++);
// 			}
// 			else
// 			{
// 				insertNodes(node->right, point, id, depth++);
// 			}
// 		}
// 	}

//     void searchForClosestPoint(Node<PointT>*& node, uint8_t depth, 
// 								const PointT& target, float distanceTol, std::vector<int>& ids)
// 	{
		
// 		if(node != nullptr)
// 		{	
// 			uint dim = depth % 3;
// 			// check if point in this node is within distance tolerance from the target
// 			// in both x and y directions
// 			if((target[0] >= node->point[0] - distanceTol && target[0] < node->point[0] + distanceTol)
// 				&& (target[1] >= node->point[1] - distanceTol && target[1] < node->point[1] + distanceTol)
// 				&& (target[2] >= node->point[2] - distanceTol && target[2] < node->point[2] + distanceTol))
// 			{
// 				auto distance = sqrt(pow(node->point[0] - target[0], 2) + pow(node->point[1] - target[1], 2) + pow(node->point[2] - target[2], 2));
// 				if(distance <= distanceTol)
// 					ids.emplace_back(node->id);
// 			}

// 			// check along the boundaries to where to search next
// 			if(target[dim] - distanceTol < node->point[dim])
// 			{
// 				searchForClosestPoint(node->left, depth++, target, distanceTol, ids);
// 			}
// 			if (target[dim] + distanceTol > node->point[dim])
// 			{
// 				searchForClosestPoint(node->right, depth++, target, distanceTol, ids);
// 			}
// 		}
// 	}

// 	// return a list of point ids in the tree that are within distance of target
// 	std::vector<int> search(PointT target, float distanceTol)
// 	{
// 		std::vector<int> ids;
// 		searchForClosestPoint(root, 0, target, distanceTol, ids);

// 		return ids;
// 	}
// };

template<typename PointT>
class ProcessPointClouds 
{
    public:

        //constructor
        ProcessPointClouds();
        //deconstructor
        ~ProcessPointClouds();

        void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

        typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

        std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

        std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

        std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

        std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);
        std::vector<typename pcl::PointCloud<PointT>::Ptr> ClusteringCustom(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

        Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);
        BoxQ boundingBoxWithOrientation(typename pcl::PointCloud<PointT>::Ptr cluster);

        void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

        typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

        std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

        //For kd tree search of euclidean distance
    private:
        std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(const typename pcl::PointCloud<PointT>::Ptr& points,
                                                    KdTree* tree, float distanceTol, int minSize, int maxSize);
        void checkProximity(KdTree* tree, typename pcl::PointCloud<PointT>::Ptr cluster, 
                    const typename pcl::PointCloud<PointT>::Ptr& cloud, std::vector<uint8_t>& processedPoints,
                    int index, float distanceTol);
};
#endif /* PROCESSPOINTCLOUDS_H_ */