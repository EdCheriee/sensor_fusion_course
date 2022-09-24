
#include <iostream>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"
#include <set>

void debugging(std::string s)
{
    #if DEBUG
    std::cout << s << std::endl;
    #endif
}

double computeMedian(std::vector<double> values)
{
    // Check vector lenght if even or odd
    if(values.size() % 2 == 0)
    {
        // If even we check the middle 2 and take average
        auto median_it1 = values.begin() + values.size() / 2 - 1;
        auto median_it2 = values.begin() + values.size() / 2;

        std::nth_element(values.begin(), median_it1 , values.end());
        std::nth_element(values.begin(), median_it2 , values.end());

        const auto e1 = *median_it1;
        const auto e2 = *median_it2;

        return (e1 + e2) / 2;
    }
    else
    {
        auto median = values.begin() + values.size() / 2;
        std::nth_element(values.begin(), median, values.end());
        return *median;    
    }
}


bool isItNotAnOutlier(double value, double median, double score)
{
    // (value - median) / score
    if(std::abs(value - median) / score < 3 * std::abs(score))
        return true;

    return false;
}

double calculateMADS(const std::vector<double>& values, double median)
{
    std::vector<double> median_absolute_dev;
    for(auto value : values)
        median_absolute_dev.push_back(std::fabs(value - median));
    
    auto median_of_median_absolute_dev = computeMedian(median_absolute_dev);
    // 1.4826 is because of assumption that data is linearly distributed
    auto median_final_score = 1.4826 * median_of_median_absolute_dev;

    return median_final_score;
}

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

        std::vector<std::vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (std::vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
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
cv::Mat show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
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


    if(bWait)
    {
        // display image
        std::string windowName = "3D Objects";
        cv::namedWindow(windowName, 1);
        cv::imshow(windowName, topviewImg);
        cv::waitKey(0); // wait for key to be pressed
    }

    return topviewImg;
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    
    std::vector<cv::KeyPoint> keyPointsWithinBB;
    std::vector<cv::DMatch> keyPointMatchesWithinBB;
    std::vector<double> distanceBetweenMatchingKeypoints;

    debugging("start");
    // trainIdx --> current frame
    // queryIdx --> previous frame
    // Calculate distance between matching pairs of keypoints
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { 
        // Get pair of the matching keypoints in current and previous frame
        cv::KeyPoint currKp = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint prevKp = kptsPrev.at(it1->queryIdx);

        // Check if both points are within bounding box
        if(boundingBox.roi.contains(currKp.pt) && boundingBox.roi.contains(prevKp.pt))
        {
            // Calculate Euclidean distance between points
            double distKpPrevVsCurr = sqrt( pow((currKp.pt.x-prevKp.pt.x), 2) + pow ((currKp.pt.y-prevKp.pt.y), 2));
            distanceBetweenMatchingKeypoints.push_back(distKpPrevVsCurr);
        }
    }
    debugging("found contained");
    std::vector<int> medianindeces;
    auto medianMatchDistance = computeMedian(distanceBetweenMatchingKeypoints);
    auto MADSScore = calculateMADS(distanceBetweenMatchingKeypoints, medianMatchDistance);
    
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { 
        // Get pair of the matching keypoints in current and previous frame
        cv::KeyPoint currKp = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint prevKp = kptsPrev.at(it1->queryIdx);

        // Calculate Euclidean distance between points
        double distKpPrevVsCurr = sqrt( pow((currKp.pt.x-prevKp.pt.x), 2) + pow ((currKp.pt.y-prevKp.pt.y), 2));
        
        if(isItNotAnOutlier(distKpPrevVsCurr, medianMatchDistance, MADSScore) && boundingBox.roi.contains(currKp.pt)
        && boundingBox.roi.contains(prevKp.pt))
        {
            debugging("Adding no outliers " + std::to_string(keyPointMatchesWithinBB.size()));
            keyPointsWithinBB.push_back(currKp);
            keyPointMatchesWithinBB.push_back(*it1);
        }
    }    
    debugging("outliers not added, kptmatch size " + std::to_string(keyPointMatchesWithinBB.size()));

    boundingBox.keypoints = keyPointsWithinBB;
    boundingBox.kptMatches = keyPointMatchesWithinBB;
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
double computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    std::cout << "kptmatches size => " << std::to_string(kptMatches.size()) << std::endl;
    if (kptMatches.size() < 1)
    {
        TTC = NAN;
        return NAN;
    }

    // compute distance ratios between all matched keypoints
    std::vector<std::pair<double,double>> distRatiosAndDiffs;
    std::vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame

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
                distRatios.push_back(distRatio);
                distRatiosAndDiffs.push_back(std::make_pair(distRatio, distPrev - distCurr));
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return NAN;
    }

    std::vector<int> medianindeces;
    double medianDistRatio = computeMedian(distRatios);

    double dT = 1 / frameRate;
    TTC = -dT / (1 - medianDistRatio);

    return medianDistRatio;
}


double computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // Please take a look at the "Lesson 3: Engineering a Collision Detection System"
    // of this course to revisit the theory behind TTC estimation. 
    // Also, please implement the estimation in a way that makes it robust 
    // against outliers which might be way too close and thus lead to faulty estimates of the TTC. 
    // Please return your TCC to the main function at the end of computeTTCLidar.

    // The task is complete once the code is functional and returns the specified output.
    // Also, the code is able to deal with outlier Lidar points in a statistically robust
    // way to avoid severe estimation errors.

    double dT = 1 / frameRate;        // time between two measurements in seconds
    // find closest distance to Lidar points within ego lane
    double minXPrev = 1e9, minXCurr = 1e9;

    // Get median X value (for removing outliers)
    std::vector<double> prevFrameXpoints, currFrameXpoints;

    for(auto previous : lidarPointsPrev)
        prevFrameXpoints.push_back(previous.x);

    for(auto current : lidarPointsCurr)
        currFrameXpoints.push_back(current.x);


    // Calculate median
    auto prevFrameMedian = computeMedian(prevFrameXpoints);
    auto currFrameMedian = computeMedian(currFrameXpoints);

    // Calculate MADS score
    auto prevFrameMADS = calculateMADS(prevFrameXpoints, prevFrameMedian);
    auto currFrameMADS = calculateMADS(currFrameXpoints, currFrameMedian);


    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        if(isItNotAnOutlier(it->x, prevFrameMedian, prevFrameMADS))
            minXPrev = minXPrev > it->x ? it->x : minXPrev;
    }

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        if(isItNotAnOutlier(it->x, currFrameMedian, currFrameMADS))
            minXCurr = minXCurr > it->x ? it->x : minXCurr;
    }

    // compute TTC from both measurements
    TTC = minXCurr * dT / (minXPrev - minXCurr);
    return minXCurr;

}


double matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    //// TASK FP.1 -> match list of 3D objects (vector<BoundingBox>) between current and previous frame (implement ->matchBoundingBoxes)
    // In this task, please implement the method "matchBoundingBoxes", 
    // which takes as input both the previous and the current data frames
    // and provides as output the ids of the matched regions of interest
    // (i.e. the boxID property)“. 
    // Matches must be the ones with the highest number of keypoint correspondences.
    
    // The cv::DMatch object has queryIdx (indices for the keypoints in the previous frame) 
    // and trainIdx (for current frame)
    double t = (double)cv::getTickCount();

    int point_counts[prevFrame.boundingBoxes.size()][currFrame.boundingBoxes.size()] = {0};

    for (auto it1 = matches.begin(); it1 != matches.end(); ++it1)
    {
        auto prev_kpointIndex = it1->queryIdx;
        auto current_kpointIndex = it1->trainIdx;

        cv::KeyPoint prevKeyPt = prevFrame.keypoints[prev_kpointIndex];
        cv::KeyPoint currentKeyPt = currFrame.keypoints[current_kpointIndex];

        std::vector<int> prevFrameBBIds, currFrameBBIds;

        // Check if ROI contains the keypoint in previous and current frame bounding boxes.
        for(auto it2 = prevFrame.boundingBoxes.begin(); it2 != prevFrame.boundingBoxes.end(); ++it2)
        {
            if(it2->roi.contains(prevKeyPt.pt))
                prevFrameBBIds.emplace_back(it2->boxID);
        }

        for(auto it3 = currFrame.boundingBoxes.begin(); it3 != currFrame.boundingBoxes.end(); ++it3)
        {
            if(it3->roi.contains(currentKeyPt.pt))
                currFrameBBIds.emplace_back(it3->boxID);
        }

    
        // Update BB_match storage with keypoint matches
        for (auto prevId : prevFrameBBIds)
        {
            for (auto currId : currFrameBBIds)
            {
                point_counts[prevId][currId]++;
            }
        }

    }

    // Select BB matches based on highest number in each row
    for (int prevId = 0; prevId < prevFrame.boundingBoxes.size(); ++prevId)
    {
        int maxCount = 0;
        int maxId = 0;
        for (int currId = 0; currId < currFrame.boundingBoxes.size(); ++currId)
        {
            if (point_counts[prevId][currId] > maxCount)
            {
                maxCount = point_counts[prevId][currId];
                maxId = currId;
            }
        }
        bbBestMatches.insert({prevId, maxId});
    }

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    return 1000 * t / 1.0;
}
