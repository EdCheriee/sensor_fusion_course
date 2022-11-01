#include <numeric>
#include "matching2D.hpp"

// Find best matches for keypoints in two camera images based on several matching methods
double matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorCategory, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    double t = (double)cv::getTickCount();

    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType;
        // with SIFT
        if (descriptorCategory.compare("DES_HOG") == 0)
        {
            normType = cv::NORM_L2;
        }
        // with all other binary descriptors
        else
        {
            normType = cv::NORM_HAMMING;
        }

        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        // with SIFT
        // if (descriptorCategory.compare("DES_HOG") == 0)
        // {
        //     matcher = cv::FlannBasedMatcher::create();
        // }
        // else
        // {
        //     // with all other binary descriptorTypes
        //     const cv::Ptr<cv::flann::IndexParams>& indexParams = cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2);
        //     matcher = cv::makePtr<cv::FlannBasedMatcher>(indexParams);
        // }

        if (descSource.type() != CV_32F)
        { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
            descSource.convertTo(descSource, CV_32F);
            descRef.convertTo(descRef, CV_32F);
        }

        //... TODO : implement FLANN matching
        std::cout << "FLANN matching";
        //-- Step 2: Matching descriptor vectors with a FLANN based matcher
        // Since SURF is a floating-point descriptor NORM_L2 is used
        matcher = cv::FlannBasedMatcher::create();

        // matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    }


    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)

        std::vector< std::vector<cv::DMatch> > knn_matches;
        matcher->knnMatch( descSource, descRef, knn_matches, 2 );
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        // TODO : filter matches using descriptor distance ratio test
            //-- Filter matches using the Lowe's ratio test
        const float ratio_thresh = 0.8f;
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                matches.push_back(knn_matches[i][0]);
            }
        }
        std::cout << "# keypoints removed = " << knn_matches.size() - matches.size() << std::endl;
    }

    return 1000 * t / 1.0;
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
double descKeypoints(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, std::string descriptorType)
{
    // select appropriate descriptor
    double t = (double)cv::getTickCount();

    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {
        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("BRIEF") == 0)
    {
        int n_Bytes = 32;
        bool orientation = false;
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create(n_Bytes, orientation);
    }
    else if(descriptorType.compare("ORB") == 0)
    {
        int nfeatures = 500;
        float scaleFactor = 1.2f;
        int nlevels = 8;
        int edgeThreshold = 31;
        int firstLevel = 0;
        int WTA_K = 2;
        auto scoreType = cv::ORB::HARRIS_SCORE;
        int patchSize = 31;
        int fastThreshold = 20;
        extractor = cv::ORB::create(
            nfeatures, scaleFactor,
            nlevels, edgeThreshold,
            firstLevel, WTA_K,
            scoreType, patchSize, 
            fastThreshold
        );
    }
    else if(descriptorType.compare("FREAK") == 0)
    {
        bool orientationNormalized = true;
        bool scaleNormalized = true;
        float patternScale = 22.0f;
        int nOctaves = 4;

        extractor = cv::xfeatures2d::FREAK::create(
            orientationNormalized,
            scaleNormalized,
            patternScale,
            nOctaves);
    }
    else if(descriptorType.compare("AKAZE") == 0)
    {
        auto descriptor_type = cv::AKAZE::DESCRIPTOR_MLDB;
        int descriptor_size = 0;
        int descriptor_channels = 3;
        float threshold = 0.001f;
        int nOctaves = 4;
        int nOctaveLayers = 4;
        auto diffusivity = cv::KAZE::DIFF_PM_G2;

        extractor = cv::AKAZE::create(
            descriptor_type,
            descriptor_size,
            descriptor_channels,
            threshold,
            nOctaves,
            nOctaveLayers,
            diffusivity);
    }
    else if (descriptorType.compare("SIFT") == 0)
    {
        int nfeatures = 0;
        int nOctaveLayers = 3;
        double contrastThreshold = 0.04;
        double edgeThreshold = 10;
        double sigma = 1.6;

        extractor = cv::xfeatures2d::SIFT::create(
            nfeatures,
            nOctaveLayers,
            contrastThreshold,
            edgeThreshold,
            sigma
        );
    }
    else
    {
        std::cout << "ERROR: Incorrect descriptor selected. Descriptor selected " << descriptorType << std::endl;
    }

    // perform feature description
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    return 1000 * t / 1.0;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
double detKeypointsShiTomasi(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / std::max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    std::vector<cv::Point2f> corners;
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

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
    return 1000 * t / 1.0;
}

double detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // Detector parameters
    int blockSize = 2; // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3; // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04; // Harris parameter (see equation for details)

    // Detect Harris corners and normalize output

    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1 );
    double t = (double)cv::getTickCount();
    cv::cornerHarris( img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT ); 
    cv::normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
    cv::convertScaleAbs( dst_norm, dst_norm_scaled );

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

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string windowName = "Harris Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }

    return 1000 * t / 1.0;
}

double detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis)
{
    ////AKAZE, SIFT
    double t = (double)cv::getTickCount();
    if(detectorType.compare("FAST") == 0)
    {
        detKeypointsFAST(keypoints, img, bVis);
    }
    else if(detectorType.compare("BRISK") == 0)
    {
        detKeypointsBRISK(keypoints, img, bVis);
    }
    else if(detectorType.compare("ORB") == 0)
    {
        detKeypointsORB(keypoints, img, bVis);
    }
    else if(detectorType.compare("AKAZE") == 0)
    {
        detKeypointsAKAZE(keypoints, img, bVis);
    }
    else if(detectorType.compare("SIFT") == 0)
    {
        detKeypointsSIFT(keypoints, img, bVis);
    }
    else
    {
        std::cout<<"Descriptor type " << detectorType << "cannot be used. Choose a different type" << std::endl;
        return 0;
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    return 1000 * t / 1.0;
}

void detKeypointsFAST(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    cv::Ptr<cv::FastFeatureDetector> fastDetector = cv::FastFeatureDetector::create(80, true);

    fastDetector->detect(img, keypoints);

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string windowName = "FAST Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}


void detKeypointsBRISK(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    cv::Ptr<cv::BRISK> briskDetector = cv::BRISK::create();
    briskDetector->detect(img, keypoints);

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string windowName = "BRISK Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsORB(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    cv::Ptr<cv::ORB> orbDetector = cv::ORB::create();
    orbDetector->detect(img, keypoints);
    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string windowName = "ORB Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }    
}

void detKeypointsAKAZE(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    cv::Ptr<cv::AKAZE> orbDetector = cv::AKAZE::create();
    orbDetector->detect(img, keypoints);

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string windowName = "AKAZE Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }

}

void detKeypointsSIFT(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    cv::Ptr<cv::xfeatures2d::SIFT> orbDetector = cv::xfeatures2d::SIFT::create();
    orbDetector->detect(img, keypoints);
    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string windowName = "SIFT Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}
 

