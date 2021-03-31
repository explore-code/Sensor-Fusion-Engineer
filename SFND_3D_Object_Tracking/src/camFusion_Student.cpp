
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{

    std::vector<cv::KeyPoint> bbKeypoints;
    std::vector<cv::DMatch>* bbMatches = new std::vector<cv::DMatch>();
    double euclideanMeanDist = 0;

    /*std::cout << kptMatches.size() << " incoming keypoint matches" << std::endl;
    std::cout << kptsPrev.size() << " incoming keypoints in the previous image" << std::endl;
    std::cout << kptsCurr.size() << " incoming keypoints in the current image" << std::endl;*/

    // Iterate over all kptMatches
    for(auto kptMatch : kptMatches)
    {

        //std::cout << "incoming keypoint match: " << kptMatch.queryIdx << ", " << kptMatch.trainIdx << std::endl;

        cv::KeyPoint currKpt = kptsCurr.at(kptMatch.trainIdx);
        cv::KeyPoint prevKpt = kptsPrev.at(kptMatch.queryIdx);

        // Extract all keypoints from kptsCurr that lie within boundingBox's roi rect, add to a temporary vector
        if(boundingBox.roi.contains(currKpt.pt))
        {
            //bbKeypoints.push_back(currKpt);
            bbMatches->push_back(kptMatch);

            // Add to sum of euclidean distances
            euclideanMeanDist += cv::norm(currKpt.pt - prevKpt.pt);
            //euclideanMeanDist += cv::sqrt((currKpt.pt.x - prevKpt.pt.x) * (currKpt.pt.x - prevKpt.pt.x) + (currKpt.pt.y - prevKpt.pt.y) * (currKpt.pt.y - prevKpt.pt.y));
        }

    }

    // Compute average distance between keypoint matches
    euclideanMeanDist /= (double) bbMatches->size();
    
    std::cout << "Keypoints extracted. Found " << bbMatches->size() << " matches with average euclidean distance of " << euclideanMeanDist << std::endl;

    const double factor = 1.5;
    // Iterate over all matches
    auto kptMatch = bbMatches->begin();
    while(kptMatch != bbMatches->end())
    {

        //std::cout << "Filtering keypoint match: " << kptMatch->queryIdx << ", " << kptMatch->trainIdx << std::endl;

        // Get current keypoint
        cv::KeyPoint currKpt = kptsCurr.at(kptMatch->trainIdx);
        cv::KeyPoint prevKpt = kptsPrev.at(kptMatch->queryIdx);

        // Compute distance of this match
        double distL2 = cv::norm(currKpt.pt - prevKpt.pt);

        // Remove this match if its distance is greater than factor * meanDistance
        if(distL2 > factor * euclideanMeanDist)
        {
            kptMatch = bbMatches->erase(kptMatch);
        }
        else
        {
            kptMatch++;
        }        

    }

    // Add filtered keypoints to the boundingBox
    boundingBox.kptMatches = *bbMatches;

    std::cout << "Keypoints filtered. Found " << bbMatches->size() << " matches after filtering" << std::endl;

    /*for(auto kptMatch : boundingBox.kptMatches)
    {
        std::cout << "match: " << kptMatch.queryIdx << ", " << kptMatch.trainIdx << std::endl;
    }*/

}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{

    std::vector<double>* distRatios = new std::vector<double>;

    // Iterate over all keypoints of the current image
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer keypoint loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner keypoint loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero
                double distRatio = distCurr / distPrev;
                distRatios->push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios->size() == 0)
    {
        TTC = NAN;
        return;
    }

    // Calculate median of distance ratios
    auto medianIdx = distRatios->size() / 2;
    // Perform partial sorting
    std::nth_element(distRatios->begin(), distRatios->begin() + medianIdx, distRatios->end());
    const double medianDistanceRatio = distRatios->at(medianIdx);

    // Compute TTC from median of distance ratios
    TTC = -1.0 / (frameRate * (1 - medianDistanceRatio));
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    std::vector<LidarPoint> lidarPointsPrevCp = lidarPointsPrev;
    std::vector<LidarPoint> lidarPointsCurrCp = lidarPointsCurr;

    std::vector<int> sizes = { 3, (int) lidarPointsPrevCp.size(), (int) lidarPointsCurrCp.size() };
    int n = *std::min_element(sizes.begin(), sizes.end());

    // Find n closest Lidar points in the previous frame
    std::sort(lidarPointsPrevCp.begin(), lidarPointsPrevCp.end(),
                [](LidarPoint lp1, LidarPoint lp2)
                {
                    return lp1.x < lp2.x;
                });
     
    std::vector<LidarPoint> prevClosestLidarPoints(lidarPointsPrevCp.begin(), lidarPointsPrevCp.begin() + n);
    /*std::cout << "Closest points of previous image: " << std::endl;
    for(auto lidarPoint : prevClosestLidarPoints)
    {
        std::cout << "(" << lidarPoint.x << ", " << lidarPoint.y << ", " << lidarPoint.z << ")" << std::endl;
    }*/

    // Calculate the median distance of these Lidar points
    const double prevMedianDistance = prevClosestLidarPoints.at(n / 2).x;

    // Find n closest Lidar points in the current frame
    std::sort(lidarPointsCurrCp.begin(), lidarPointsCurrCp.end(),
                [](LidarPoint lp1, LidarPoint lp2)
                {
                    return lp1.x < lp2.x;
                });
       
    std::vector<LidarPoint> currClosestLidarPoints(lidarPointsCurrCp.begin(), lidarPointsCurrCp.begin() + n);
    /*std::cout << "Closest points of current image: " << std::endl;
    for(auto lidarPoint : currClosestLidarPoints)
    {
        std::cout << "(" << lidarPoint.x << ", " << lidarPoint.y << ", " << lidarPoint.z << ")" << std::endl;
    }*/

    // Calculate the median distance of these Lidar points
    const double currMedianDistance = currClosestLidarPoints.at(n / 2).x;

    // Calculate the TTC
    TTC = currMedianDistance / (frameRate * (prevMedianDistance - currMedianDistance));
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // Create a map<std::pair<int, int>, int> storing combinations of bounding boxes from the previous frame and the current frame
    std::map<std::pair<int, int>, int> boxMatchCounters;

    // Iterate over all matches
    for(auto match : matches)
    {

        // For each match get queryKeypoint and trainKeypoint via the indices stored in the match
        int prevKeypointIdx = match.queryIdx;
        int currKeypointIdx = match.trainIdx;
        cv::KeyPoint prevKeypoint = prevFrame.keypoints.at(prevKeypointIdx);
        cv::KeyPoint currKeypoint = currFrame.keypoints.at(currKeypointIdx);

        std::vector<int> prevBbIdxs;
        // Iterate over all bounding boxes of the previous frame
        for(auto bb : prevFrame.boundingBoxes)
        {

            // Identify the bounding box's boxID that the queryKeypoint lies in and store them in a vector
            if(bb.roi.contains(prevKeypoint.pt))
            {
                prevBbIdxs.push_back(bb.boxID);
            }

        }

        std::vector<int> currBbIdxs;
        // Iterate over all bounding boxes of the current frame
        for(auto bb : currFrame.boundingBoxes)
        {

            // Identify the bounding boxes boxID that the trainKeypoint lies in and store them in a vector
            if(bb.roi.contains(currKeypoint.pt))
            {
                currBbIdxs.push_back(bb.boxID);
            }
            
        }

        // Build all possible pairs of boxIDs from the two assembled vectors of ints
        for(int i : prevBbIdxs)
        {

            for(int j : currBbIdxs)
            {

                const std::pair<int, int> key(i, j);
                // For each pair, increase the counter by one
                if(boxMatchCounters.find(key) != boxMatchCounters.end())
                {
                    boxMatchCounters.at(key)++;
                }
                else
                {
                    boxMatchCounters.insert(std::pair<std::pair<int, int>, int>(key, 1));
                }                

            }

        }

    }

    // Iterate over all boxes of the current frame
    for(auto bb : currFrame.boundingBoxes)
    {
        
        std::pair<std::pair<int, int>, int> bestMatchWithCounter(std::pair<int, int>(-1, -1), -1);
        // Extract the match for this bb with the highest counter
        for(auto comb : boxMatchCounters)
        {

            // Update current best match if a combination for bb is found with a higher counter
            if(bb.boxID == comb.first.second && comb.second > bestMatchWithCounter.second)
            {
                bestMatchWithCounter = comb;
            }

        }

        // Add the best match to the output of the function
        if(bestMatchWithCounter.second >= 0)
        {
            bbBestMatches.insert(bestMatchWithCounter.first);
        }

    }

    /*for(auto cit = bbBestMatches.cbegin(); cit != bbBestMatches.end(); cit++)
    {
        std::cout << "Box " << cit->second << " of current frame has matched with " << cit->first << " of previous frame" << std::endl;
    }*/

}
