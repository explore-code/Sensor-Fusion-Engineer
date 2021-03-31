/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h"

typedef std::pair<std::vector<float>, int> PointWithId;
const bool mode3D = true;

// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area
pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);

	if(mode3D)
	{
  		viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, window.z_min, window.z_max, 1, 1, 1, "window");
	}
	else
	{
		viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0, 1, 1, 1, "window");
	}
  	return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(std::vector<std::vector<float>> points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	
  	for(int i = 0; i < points.size(); i++)
  	{
  		pcl::PointXYZ point;
  		point.x = points[i][0];
  		point.y = points[i][1];
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D(std::vector<std::vector<float>> points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	
  	for(int i = 0; i < points.size(); i++)
  	{
  		pcl::PointXYZ point;
  		point.x = points[i][0];
  		point.y = points[i][1];
  		point.z = points[i][2];

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

void render2DTree(Node* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0)
{

	if(node!=NULL)
	{
		Box upperWindow = window;
		Box lowerWindow = window;
		// split on x axis
		if(depth%2==0)
		{
			viewer->addLine(pcl::PointXYZ(node->point[0], window.y_min, 0),pcl::PointXYZ(node->point[0], window.y_max, 0),0,0,1,"line"+std::to_string(iteration));
			lowerWindow.x_max = node->point[0];
			upperWindow.x_min = node->point[0];
		}
		// split on y axis
		else
		{
			viewer->addLine(pcl::PointXYZ(window.x_min, node->point[1], 0),pcl::PointXYZ(window.x_max, node->point[1], 0),1,0,0,"line"+std::to_string(iteration));
			lowerWindow.y_max = node->point[1];
			upperWindow.y_min = node->point[1];
		}
		iteration++;

		render2DTree(node->left,viewer, lowerWindow, iteration, depth+1);
		render2DTree(node->right,viewer, upperWindow, iteration, depth+1);


	}

}

void render3DTree(Node* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0)
{

	if(node!=NULL)
	{
		Box upperWindow = window;
		Box lowerWindow = window;
		// split on x axis
		if(depth%3==0)
		{
			viewer->addCube(node->point[0], node->point[0], window.y_min, window.y_max, window.z_min, window.z_max, 1, 0, 0, "plane" + std::to_string(iteration));
			lowerWindow.x_max = node->point[0];
			upperWindow.x_min = node->point[0];
		}
		// split on y axis
		else if(depth%3==1)
		{
			viewer->addCube(window.x_min, window.x_max, node->point[1], node->point[1], window.z_min, window.z_max, 0, 1, 0, "plane" + std::to_string(iteration));
			lowerWindow.y_max = node->point[1];
			upperWindow.y_min = node->point[1];
		}
		// split on z axis
		else
		{
			viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, node->point[2], node->point[2], 0, 0, 1, "plane" + std::to_string(iteration));
			lowerWindow.z_max = node->point[2];
			upperWindow.z_min = node->point[2];
		}
		iteration++;

		render3DTree(node->left,viewer, lowerWindow, iteration, depth+1);
		render3DTree(node->right,viewer, upperWindow, iteration, depth+1);

	}

}

void proximity(const std::vector<std::vector<float>>& points, KdTree* tree, const float distanceTol, std::vector<int>& cluster, std::vector<bool>& visited, const PointWithId& target)
{
	visited.at(target.second) = true;
	cluster.push_back(target.second);
	std::vector<int> pointsNearby = tree->search(target.first, distanceTol);
	for(int idx : pointsNearby)
	{
		if(!visited.at(idx))
		{
			const PointWithId newTarget(points.at(idx), idx);
			proximity(points, tree, distanceTol, cluster, visited, newTarget);
		}
	}
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;

	std::vector<bool> visited(points.size(), false);

	for(int i = 0; i < points.size(); ++i)
	{
		if(!visited.at(i))
		{
			const PointWithId pointWithId(points.at(i), i);
			std::vector<int> cluster;
			proximity(points, tree, distanceTol, cluster, visited, pointWithId);
			clusters.push_back(cluster);
		}
	}
 
	return clusters;

}

void proximity3D(const std::vector<std::vector<float>>& points, KdTree3D* tree, const float distanceTol, std::vector<int>& cluster, std::vector<bool>& visited, const PointWithId& target)
{
	visited.at(target.second) = true;
	cluster.push_back(target.second);
	std::vector<int> pointsNearby = tree->search(target.first, distanceTol);
	for(int idx : pointsNearby)
	{
		if(!visited.at(idx))
		{
			const PointWithId newTarget(points.at(idx), idx);
			proximity3D(points, tree, distanceTol, cluster, visited, newTarget);
		}
	}
}

std::vector<std::vector<int>> euclideanCluster3D(const std::vector<std::vector<float>>& points, KdTree3D* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;

	std::vector<bool> visited(points.size(), false);

	for(int i = 0; i < points.size(); ++i)
	{
		if(!visited.at(i))
		{
			const PointWithId pointWithId(points.at(i), i);
			std::vector<int> cluster;
			proximity3D(points, tree, distanceTol, cluster, visited, pointWithId);
			clusters.push_back(cluster);
		}
	}
 
	return clusters;

}

int main ()
{	
	// Create viewer
	Box window;
  	window.x_min = -10;
  	window.x_max =  10;
  	window.y_min = -10;
  	window.y_max =  10;
  	window.z_min = -10;
  	window.z_max =  10;
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene(window, 25);

	// Create data
	std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3}, {7.2,6.1}, {8.0,5.3}, {7.2,7.1}, {0.2,-7.1}, {1.7,-6.9}, {-1.2,-7.2}, {2.2,-8.9} };
	std::vector<std::vector<float>> points3D = { {-6.2,7,1.2}, {-6.3,8.4, 5.3}, {-5.2,7.1, -3.7}, {-5.7,6.3,-8.2}, {7.2,6.1, 9.7}, {8.0,5.3, 0.0}, {7.2,7.1, 4.5}, {0.2,-7.1, 4.6}, {1.7,-6.9, 2.3}, {-1.2,-7.2, -0.6}, {2.2,-8.9, -6.2}, {2.0, -8.8, -6.5}, {2.4, -9, -6.0} };
	//std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3} };
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(points);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3D = CreateData3D(points3D);

	KdTree* tree = new KdTree;
	KdTree3D* tree3D = new KdTree3D;
  
    for (int i=0; i<points.size(); i++) 
    	tree->insert(points[i],i); 

	for(int i = 0; i<points3D.size(); ++i)
	{
		tree3D->insert(points3D[i], i);
	}

	if(mode3D) 
	{
		int it3D = 0;
		render3DTree(tree3D->root,viewer,window, it3D);
	}
	else
	{	
  		int it = 0;
  		render2DTree(tree->root,viewer,window, it);
	}
  
  	std::cout << "Test Search" << std::endl;

	std::vector<int> nearby;
	if(mode3D)
	{
  		nearby = tree3D->search({-6, 7, 5.0}, 3.0);
	}
	else
	{
		nearby = tree->search({-6,7},3.0);
	}	

  	for(int index : nearby)
      std::cout << index << ",";
  	std::cout << std::endl;

  	// Time segmentation process
  	auto startTime = std::chrono::steady_clock::now();
  	//

	std::vector<std::vector<int>> clusters;
	if(mode3D)
	{
		clusters = euclideanCluster3D(points3D, tree3D, 3.0);
	}
	else
	{
		clusters = euclideanCluster(points, tree, 3.0);
	}

  	//
  	auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

  	// Render clusters
  	int clusterId = 0;
	std::vector<Color> colors = { Color(1,0,0), Color(0,1,0), Color(0,0,1) };
  	for(std::vector<int> cluster : clusters)
  	{
  		pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
  		for(int indice: cluster)
		{
			if(mode3D)
			{
				clusterCloud->points.push_back(pcl::PointXYZ(points3D[indice][0], points3D[indice][1], points3D[indice][2]));
			}
			else
			{
				clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0], points[indice][1], 0));
			}
		}
  		renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
  		++clusterId;
  	}
	
  	if(clusters.size()==0)
	{
		if(mode3D)
		{
  			renderPointCloud(viewer,cloud3D,"data3D");
		}
		else
		{
			renderPointCloud(viewer,cloud,"data");
		}
	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
