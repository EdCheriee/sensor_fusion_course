#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>


void cornerHarrisDetector()
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1.png");

    // convert image to grayscale
    cv::Mat imgGray; 
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

    // Detector parameters
    int blockSize = 2; // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3; // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04; // Harris parameter (see equation for details)

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(imgGray.size(), CV_32FC1 );
    cv::cornerHarris( imgGray, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT ); 
    cv::normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
    cv::convertScaleAbs( dst_norm, dst_norm_scaled );

    std::vector<cv::KeyPoint> keypoints;
    double maximumOverlap = 0.0;
    int threshold = 100;

    for(int i = 0; i < dst_norm.rows; i++)
        for(int j = 0; j < dst_norm.cols; j++)
        {
            int currentPointValue = (int)dst_norm.at<float>(i, j);
            if(currentPointValue > threshold)
            {
                cv::KeyPoint tempKeyPoint;
                tempKeyPoint.pt = cv::Point2f(j, i);
                tempKeyPoint.size = apertureSize * 2;
                tempKeyPoint.response = currentPointValue;

                // perform non-maximum suppression (NMS) in local neighbourhood around new key point
                bool bOverlap = false;
                for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
                {
                    double kptOverlap = cv::KeyPoint::overlap(tempKeyPoint, *it);
                    if (kptOverlap > maximumOverlap)
                    {
                        bOverlap = true;
                        if (tempKeyPoint.response > (*it).response)
                        {                      // if overlap is >t AND response is higher for new kpt
                            *it = tempKeyPoint; // replace old key point with new one
                            break;             // quit loop over keypoints
                        }
                    }
                }

                if (!bOverlap)
                {                                     // only add new key point if no overlap has been found in previous NMS
                    keypoints.push_back(tempKeyPoint); // store new keypoint in dynamic list
                }
            }
        }


    // visualize results
    std::string windowName = "Harris Corner Detector Response Matrix";
    cv::namedWindow( windowName, 4 );
    cv::Mat visImage = dst_norm_scaled.clone();
    cv::drawKeypoints(dst_norm_scaled, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow(windowName, visImage);
    cv::waitKey(0);
}


int main()
{
    cornerHarrisDetector();

    return 0;
}