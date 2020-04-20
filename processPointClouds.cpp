// PCL lib Functions for processing point clouds
#include <unordered_set>
#include "processPointClouds.h"
// #include "ransac3d.cpp"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


struct cartesian {
	float x;
	float y;
	float z;
};

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac_plane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol) {

  	int l = cloud->points.size();
  	cartesian point1, point2, point3;
    float A,B,C,D;
  	float dist;
  	int inliers;
  	int counter = 0;
  	std::unordered_set<int> inlierResult;
  	while(counter <= maxIterations){
  		std::unordered_set<int> inlier;
  		std::vector<int>random;
  		for(int i = 0; i <3; i++)
  			random.push_back(rand()%l);

  		int index = random[0];

  		point1.x = cloud->points[index].x;
  		point1.y = cloud->points[index].y;
  		point1.z = cloud->points[index].z;
  		index = random[1];

  		point2.x = cloud->points[index].x;
  		point2.y = cloud->points[index].y;
  		point2.z = cloud->points[index].z;
  		index = random[2];

  		point3.x = cloud->points[index].x;
  		point3.y = cloud->points[index].y;
  		point3.z = cloud->points[index].z;
  		std::vector<float> v1 {point2.x - point1.x, point2.y - point1.y, point2.z - point1.z};
  		std::vector<float> v2 {point3.x - point1.x, point3.y - point1.y, point3.z - point1.z};
  		std::vector<float> v {(point2.y - point1.y)*(point3.z - point1.z) - (point2.z - point1.z)*(point3.y - point1.y),
  		                      (point2.z - point1.z)*(point3.x - point1.x) - (point2.x - point1.x)*(point3.z - point1.z),
  												  (point2.x - point1.x)*(point3.y - point1.y) - (point2.y - point1.y)*(point3.x - point1.x)};

  		A = v[0];
  		B = v[1];
  		C = v[2];

  		D = -(A*point1.x + B*point1.y + C*point1.z);
  		int c = 0;
  		for(auto point : cloud->points) {
  			dist = fabs(A*point.x + B*point.y + C*point.z +D)/sqrt(A*A + B*B + C*C);
  			if (dist <= distanceTol){
  					inliers +=1;
  					inlier.insert(c);
  			}
  			c +=1;
  		}
  		if(inlier.size() > inlierResult.size()) {
  			inlierResult = inlier;
  		}
  		counter +=1;
  	}
  	return inlierResult;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr filterCloud(new pcl::PointCloud<PointT>);
    std::vector<int> indices;
    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> vox_filter;
    pcl::CropBox<PointT> crop_box(true);
    // Eigen::Vector4f min (-1.5, -1.7, -1, 1);
    // Eigen::Vector4f max (2.6, 1.7, -4, 1);
    // crop_box.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    // crop_box.setMax(Eigen::Vector4f(2.6, 1.7, -4, 1));
    crop_box.setMin(minPoint);
    crop_box.setMax(maxPoint);
    vox_filter.setInputCloud(cloud);
    vox_filter.setLeafSize(filterRes, filterRes, filterRes);
    vox_filter.filter(*filterCloud);
    //crop_box.applyFilter(filterCloud);
    crop_box.setInputCloud(filterCloud);
    crop_box.filter(*filterCloud);

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -4, 1));
    roof.setInputCloud(filterCloud);
    roof.filter(indices);
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices)
       inliers->indices.push_back(point);
    //
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(filterCloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
     extract.filter(*filterCloud);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filterCloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr road{new pcl::PointCloud<PointT>()};
    typename pcl::PointCloud<PointT>::Ptr obstacle{new pcl::PointCloud<PointT>()};
    for (auto index : inliers->indices){
      road->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacle);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle, road);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    // TODO:: Fill in this function to find inliers for the cloud.

    //pcl::SACSegmentation<PointT> seg;
    //pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients};

    // Setting hyper-parameters of segmentation model
    // seg.setModelType(pcl::SACMODEL_PLANE);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setMaxIterations(maxIterations);
    // seg.setDistanceThreshold(distanceThreshold);

    // segmentation
    //seg.setInputCloud(cloud);
    //seg.segment(*inliers, *coefficients);

    std::unordered_set<int> inliers_points = Ransac_plane(cloud, maxIterations, distanceThreshold);
    for(auto point : inliers_points)
      inliers->indices.push_back(point);
    if (inliers -> indices.size() == 0){
      std::cout << " Could not find a planar model for the given cloud input" << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximately(int index, const std::vector<std::vector<float>>&points, std::vector<int>& cluster, std::vector<bool>& visited, KdTree* tree, float dist) {
// 	Marking point as porcessed
 	 visited[index] = true;
// 	Adding point to cluster
   cluster.push_back(index);

// Searching got nearby points
 	 std::vector<int> nearbyPoints = tree->search(points[index], dist);

   for(int loc : nearbyPoints){
 		 if(!visited[loc])
  	  	proximately(loc, points, cluster, visited,tree, dist);
		}
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>> points, KdTree* tree, float distanceTol)
{
	// TODO: Fill out this function to return list of indices for each cluster
	std::vector<std::vector<int>> clusters;
	std::vector<bool> visited(points.size(), false);

	int counter = 0;
	while(counter < points.size()){
	if(!visited[counter]){
	 		std::vector<int> cluster;
	   	proximately(counter, points, cluster, visited, tree, distanceTol);
	 		clusters.push_back(cluster);
	}
	counter +=1;
	}
	return clusters;
}




template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    KdTree* tree = new KdTree;
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles/ Creating the KdTree object for the search method of the extraction
    // typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    //tree->setInputCloud(cloud);
    //std::vector<pcl::PointIndices> cluster_indices;
    //pcl::EuclideanClusterExtraction<PointT> ec;
    //ec.setClusterTolerance(clusterTolerance);
    //ec.setMinClusterSize(minSize);
    //ec.setMaxClusterSize(maxSize);
    //ec.setSearchMethod(tree);
    //ec.setInputCloud(cloud);
    //ec.extract(cluster_indices);
    std::vector<std::vector<float>> euc_points;
    for(int i = 0; i < cloud->points.size(); i++){
      std::vector<float> temp_point({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z});
      tree->insert(temp_point, i);
      euc_points.push_back(temp_point);
    }

    std::vector<std::vector<int>> cluster_indices = euclideanCluster(euc_points, tree, clusterTolerance);

    for (auto getIndices : cluster_indices) {
      typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);

      for(auto index: getIndices){
        cloudCluster->points.push_back(cloud->points[index]);
      }
      cloudCluster ->width = cloudCluster ->points.size();
      cloudCluster ->height = 1;
      cloudCluster ->is_dense = true;

      if(cloudCluster->width >=minSize && cloudCluster->width <=maxSize)
      clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
