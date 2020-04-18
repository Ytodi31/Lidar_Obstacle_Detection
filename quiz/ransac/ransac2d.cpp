/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol){
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// TODO: Fill in this function
	int l = cloud->points.size();
	// For max iterations
	std::pair<int, int> random;
	std::pair<float, float> point1;
	std::pair<float, float> point2;
	int inliers;
	float A, B, C;
	int max = 0;
	float d;
	int counter = 0;
	while (counter <= maxIterations){
		random.first = rand() % (l);
		random.second = rand() %(l);
		point1.first = cloud->points[random.first].x;
		point1.second = cloud->points[random.first].y;
		point2.first = cloud->points[random.second].x;
		point2.second = cloud->points[random.second].y;
		inliers = 0;
		std::vector<int> v;
		for (int i = 0; i < l; i++){
			A = point2.second - point1.second;
			B = point2.first - point1.first;
			C = point1.first*point2.second - point2.second*point1.second;
			d = fabs(A*cloud->points[i].x + B*cloud->points[i].y + C)/sqrt(A*A + B*B);
			if (d <= distanceTol){
				inliers +=1;
				v.push_back(i);
			}
		}
		if(inliers>max){
			max = inliers;
			std::copy(v.begin(),v.end(),std::inserter(inliersResult,inliersResult.end()));
		}
		counter +=1;
	}
	return inliersResult;
}

struct cartesian{
	float x;
	float y;
	float z;
};

std::unordered_set<int> Ransac_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol){
	std::vector<int>random;
	int l = cloud->points.size();
	cartesian point1, point2, point3;
  float A,B,C,D;
	float dist;
	int inliers;
	std::unordered_set<int> inlier;
	std::unordered_set<int> inlierResult;
	int counter = 0;
	while(counter <= maxIterations){
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
		                      (point2.z - point1.z)*(point3.z - point1.x) - (point2.x = point1.z)*(point3.z - point1.z),
												  (point2.x - point1.x)*(point3.y - point1.y) - (point2.y - point1.y)*(point3.x - point1.x)};
		A = v[0];
		B = v[1];
		C = v[2];
		D = -(A*point1.x + B*point1.y + C*point1.z);
		int c = 0;
		for(auto point : cloud->points){
			dist = fabs(A*point.x + B*point.y + C*point.z +D)/sqrt(A*A + B*B + C*C);
			if (dist <= distanceTol){
					inliers +=1;
					inlier.insert(c);
					c +=1;
			}
		}
		if(inlier.size() > inlierResult.size()){
			inlierResult = inlier;
		}
		counter +=1;
	}
	return inlierResult;
}




int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();


	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac_plane(cloud, 1000, 0.1);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(0,0,1));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}

  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}

}
