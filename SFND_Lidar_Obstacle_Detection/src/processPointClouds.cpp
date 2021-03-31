// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr filteredCloud(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*filteredCloud);

    typename pcl::PointCloud<PointT>::Ptr croppedFilteredCloud(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> regionOfInterest;
    regionOfInterest.setInputCloud(filteredCloud);
    regionOfInterest.setMin(minPoint);
    regionOfInterest.setMax(maxPoint);
    regionOfInterest.filter(*croppedFilteredCloud);

    typename pcl::PointCloud<PointT>::Ptr withoutRoofCloud(new pcl::PointCloud<PointT>);
    Eigen::Vector4f roofMinPoint(-1.5, -1.7, -1, 1);
    Eigen::Vector4f roofMaxPoint(2.6, 1.7, -0.4, 1);
    pcl::CropBox<PointT> roofBox;
    roofBox.setInputCloud(croppedFilteredCloud);
    roofBox.setMin(roofMinPoint);
    roofBox.setMax(roofMaxPoint);
    roofBox.setNegative(true);
    roofBox.filter(*withoutRoofCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return withoutRoofCloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr cloud_p(new pcl::PointCloud<PointT>), cloud_f(new pcl::PointCloud<PointT>);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_p);

    extract.setNegative(true);
    extract.filter(*cloud_f);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_f, cloud_p);
    return segResult;
}


template<typename PointT>
pcl::PointIndices::Ptr ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	auto numPoints = cloud->points.size();

	int i1, i2, i3;
	PointT P1, P2, P3;
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

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for(auto iterator = inliersResult.cbegin(); iterator != inliersResult.cend(); iterator++) {
        inliers->indices.push_back(*iterator);
    }

	// Return indicies of inliers from fitted line with most inliers	
	return inliers;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    /* pcl based approach
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // TODO:: Fill in this function to find inliers for the cloud.    
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud);
    seg.setMaxIterations(maxIterations);
    seg.segment(*inliers, *coefficients);
    */

    // Own approach
    pcl::PointIndices::Ptr inliers = RansacPlane(cloud, maxIterations, distanceThreshold);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for(pcl::PointIndices inds : cluster_indices) {
        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        for(std::vector<int>::const_iterator cit = inds.indices.cbegin(); cit != inds.indices.cend(); ++cit) {
            cluster->push_back(cloud->at(*cit));
        }
        cluster->width = inds.indices.size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.push_back(cluster);
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


template<typename PointT>
BoxQ ProcessPointClouds<PointT>::optimalBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    /* Based on the blog post referenced in the course description */
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);

    /*typename pcl::PointCloud<PointT>::Ptr projectedPCACluster(new pcl::PointCloud<PointT>);
    pcl::PCA<PointT> pca;
    pca.setInputCloud(cluster);
    pca.project(*cluster, *projectedPCACluster);
    std::cout << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;
    std::cout << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;
    Eigen::Matrix3f eigenVectorsPCA = pca.getEigenVectors();*/
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr projectedCluster(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *projectedCluster, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*projectedCluster, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());
    std::cerr << "minPoint: " << minPoint << std::endl;
    std::cerr << "maxPoint: " << maxPoint << std::endl;

    // Final transform
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    //const Eigen::Quaternionf bboxQuaternion(Eigen::Matrix3f::Identity());
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
    //const Eigen::Vector3f bboxTransform = Eigen::Vector3f::Zero();

    BoxQ boxq;
    boxq.bboxTransform = bboxTransform;
    boxq.bboxQuaternion = bboxQuaternion;
    boxq.cube_length = maxPoint.x - minPoint.x;
    boxq.cube_width = maxPoint.y - minPoint.y;
    boxq.cube_height = maxPoint.z - minPoint.z;

    return boxq;
}

