
#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        //int normType = cv::NORM_HAMMING;
        int normType = descriptorType.compare("DES_BINARY") == 0 ? cv::NORM_HAMMING : cv::NORM_L2;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        matcher = cv::FlannBasedMatcher::create();
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)

        int k = 2;
        std::vector<std::vector<cv::DMatch>> twoNNMatches;
        matcher->knnMatch(descSource, descRef, twoNNMatches, k);

        float relThreshold = 0.8f;
        float numDiscardedMatches = 0.0f;
        for(auto twoNN = twoNNMatches.begin(); twoNN != twoNNMatches.end(); twoNN++)
        {
            cv::DMatch firstNN = twoNN->front();
            cv::DMatch secondNN = twoNN->back();
            float firstDistance = firstNN.distance;
            float secondDistance = secondNN.distance;

            float distanceRatio;
            cv::DMatch bestNN;
            if(firstDistance > secondDistance)
            {
                bestNN = secondNN;
                distanceRatio = secondDistance / firstDistance;
            }
            else
            {
                bestNN = firstNN;
                distanceRatio = firstDistance / secondDistance;
            }

            if((secondDistance == 0 && firstDistance == 0) || (distanceRatio <= relThreshold))
            {
                matches.push_back(bestNN);
            }
            else
            {
                numDiscardedMatches++;
            }
            
        }

        float percentageOfDiscardedMatches = numDiscardedMatches / twoNNMatches.size() * 100.0f;
        std::cout << "Number of matched keypoints = " << matches.size() << std::endl;
    }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if(descriptorType.compare("BRIEF") == 0)
    {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else if(descriptorType.compare("ORB") == 0)
    {
        extractor = cv::ORB::create();
    }
    else if(descriptorType.compare("FREAK") == 0)
    {
        extractor == cv::xfeatures2d::FREAK::create();
    }
    else if(descriptorType.compare("AKAZE") == 0)
    {
        extractor = cv::AKAZE::create();
    }
    else if(descriptorType.compare("SIFT") == 0)
    {
        extractor == cv::xfeatures2d::SIFT::create();
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    int blockSize = 2;
    int apertureSize = 3;
    int minResponse = 100;
    double k = 0.04;

    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);

    double t = (double)cv::getTickCount();

    cv::cornerHarris(img, dst, blockSize, apertureSize, k);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    for(int i = 0; i < dst_norm_scaled.rows; ++i)
    {

        for(int j = 0; j < dst_norm_scaled.cols; ++j)
        {

            int thisResponse = dst_norm_scaled.at<uchar>(i, j);

            if(thisResponse >= minResponse)
            {
                int minI = std::max(i - (apertureSize - 1) / 2, 0);
                int maxI = std::min(i + (apertureSize - 1) / 2, dst_norm_scaled.rows - 1);
                int minJ = std::max(j - (apertureSize - 1) / 2, 0);
                int maxJ = std::min(j + (apertureSize - 1) / 2, dst_norm_scaled.cols - 1);

                bool thisIsMax = true;

                for(int k = minI; k <= maxI; ++k)
                {
                    for(int l = minJ; l <= maxJ; ++l)
                    {
                        int r = dst_norm_scaled.at<uchar>(k, l);
                        if(thisResponse < r)
                        {
                            //std::cout << "Neighbor (" << k << ", " << l << ") of (" << i << ", " << j << ") has higher response" << std::endl;
                            thisIsMax = false;
                            break;
                        }
                    }

                    if(!thisIsMax)
                    {
                        break;
                    }

                    if(thisIsMax)
                    {
                        cv::KeyPoint keyPoint(j, i, blockSize, -1, thisResponse);
                        keypoints.push_back(keyPoint);
                    }
                }
            }

        }

    }

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Harris corner detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Harris Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis)
{
    cv::Ptr<cv::FeatureDetector> featureDetector;
    if(detectorType.compare("FAST") == 0)
    {
        featureDetector = cv::FastFeatureDetector::create();
    }
    else if(detectorType.compare("BRISK") == 0)
    {
        featureDetector = cv::BRISK::create();
    }
    else if(detectorType.compare("ORB") == 0)
    {
        featureDetector = cv::ORB::create();
    }
    else if(detectorType.compare("AKAZE") == 0)
    {
        featureDetector = cv::AKAZE::create();
    }
    else if(detectorType.compare("SIFT") == 0)
    {
        featureDetector = cv::xfeatures2d::SIFT::create();
    }

    double t = (double)cv::getTickCount();

    featureDetector->detect(img, keypoints);

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << detectorType << " detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
}

