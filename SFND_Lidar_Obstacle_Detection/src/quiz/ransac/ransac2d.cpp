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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	auto numPoints = cloud->points.size();

	int i1, i2;
	pcl::PointXYZ X, Y;
	// For max iterations 
	int m, n;
	for(m = 0; m < maxIterations; ++m)
	{
		std::unordered_set<int> inliersTempResult;

		// Randomly sample subset and fit line
		do
		{
			i1 = rand() % numPoints;
			i2 = rand() % numPoints;
			//std::cout << "i1 = " << i1 << ", i2 = " << i2 << std::endl;
		}
		while(i1 == i2);

		//std::cout << "Found i1 = " << i1 << ", i2 = " << i2 << std::endl;

		X = cloud->points[i1];
		Y = cloud->points[i2];

		float A = X.y - Y.y;
		float B = Y.x - X.x;
		float C = X.x * Y.y - Y.x * X.y;

		// Measure distance between every point and fitted line
		for(n = 0; n < numPoints; ++n)
		{
			auto point = cloud->points[n];
			float d = std::fabs(A * point.x + B * point.y + C) / std::sqrt(A * A + B * B);		

			// If distance is smaller than threshold count it as inlier
			if(d <= distanceTol)
			{
				inliersTempResult.insert(n);
			}
		}

		if(inliersTempResult.size() > inliersResult.size())
		{
			inliersResult = inliersTempResult;
		}

		//inliersTempResult.clear();
	}

	// Return indicies of inliers from fitted line with most inliers	
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	auto numPoints = cloud->points.size();

	int i1, i2, i3;
	pcl::PointXYZ P1, P2, P3;
	Vect3 V1, V2, V1xV2;
	// For max iterations 
	int m, n;
	for(m = 0; m < maxIterations; ++m)
	{
		std::unordered_set<int> inliersTempResult;

		// Randomly sample subset and fit line
		do
		{
			i1 = rand() % numPoints;
			i2 = rand() % numPoints;
			i3 = rand() % numPoints;
			//std::cout << "i1 = " << i1 << ", i2 = " << i2 << std::endl;
		}
		while((i1 == i2) || (i1 == i3) || (i2 == i3));

		//std::cout << "Found i1 = " << i1 << ", i2 = " << i2 << std::endl;

		P1 = cloud->points[i1];
		P2 = cloud->points[i2];
		P3 = cloud->points[i3];
		V1 = Vect3(P2.x - P1.x, P2.y - P1.y, P2.z - P1.z);
		V2 = Vect3(P3.x - P1.x, P3.y - P1.y, P3.z - P1.z);
		V1xV2 = Vect3((P2.y - P1.y) * (P3.z - P1.z) - (P2.z - P1.z) * (P3.y - P1.y),
					  (P2.z - P1.z) * (P3.x - P1.x) - (P2.x - P1.x) * (P3.z - P1.z),
					  (P2.x - P1.x) * (P3.y - P1.y) - (P2.y - P1.y) * (P3.x - P1.x));

		float A = V1xV2.x;
		float B = V1xV2.y;
		float C = V1xV2.z;
		float D = -(V1xV2.x * P1.x + V1xV2.y * P2.y + V1xV2.z * P3.z);

		// Measure distance between every point and fitted line
		for(n = 0; n < numPoints; ++n)
		{
			auto point = cloud->points[n];
			float d = std::fabs(A * point.x + B * point.y + C * point.z + D) / std::sqrt(A * A + B * B + C * C);		

			// If distance is smaller than threshold count it as inlier
			if(d <= distanceTol)
			{
				inliersTempResult.insert(n);
			}
		}

		if(inliersTempResult.size() > inliersResult.size())
		{
			inliersResult = inliersTempResult;
		}

		//inliersTempResult.clear();
	}

	// Return indicies of inliers from fitted line with most inliers	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);
	std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.25);

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
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
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
