
/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <experimental/filesystem>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"
#include "objectDetection2D.hpp"
#include "lidarData.hpp"
#include "camFusion.hpp"
#include "ringBuffer.h"

namespace fs = std::experimental::filesystem;

struct PerformanceStorage
{
    std::string detectorType;          // Detector type
    std::string descriptorType;        // Descriptor type
    std::string matcherType;           // MAT_BF, MAT_FLANN
    std::string descriptorContentType; // DES_BINARY, DES_HOG
    std::string selectorType;          // SEL_NN, SEL_KNN

    // Time holders
    double detectorPerformance = 0.0;
    double descriptorPerformance = 0.0;
    double matcherSelectorPerformance = 0.0;
    double matchBBoxPerformance = 0.0;
    double totalPerformance = 0.0;
    double TTCCamera = 0.0;
    double TTCLidar = 0.0;
    double LidarDistance = 0.0;

    // N keypoint holders
    unsigned long filteredKeypoints = 0l;
    unsigned long totalKeypoints = 0l;
    unsigned long matchedKeypoints = 0l;
};

bool doesFileExist(const std::string &name)
{
    std::ifstream f(name.c_str());
    return f.good();
}

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    /* INIT VARIABLES AND DATA STRUCTURES */
    // CSV file
    std::string file_name = "../Results/comparison_results.csv";
    // data location
    std::string dataPath = "../";
    std::string finalImagesPath = "../Results";
    std::string finalImagesDirectoryName = "";
    // camera
    std::string imgBasePath = dataPath + "images/";
    std::string imgPrefix = "KITTI/2011_09_26/image_02/data/000000"; // left camera, color
    std::string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 18;  // last file index to load
    int imgStepWidth = 1;
    int imgFillWidth = 4; // no. of digits which make up the file index (e.g. img-0001.png)

    // object detection
    std::string yoloBasePath = dataPath + "dat/yolo/";
    std::string yoloClassesFile = yoloBasePath + "coco.names";
    std::string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
    std::string yoloModelWeights = yoloBasePath + "yolov3.weights";

    // Lidar
    std::string lidarPrefix = "KITTI/2011_09_26/velodyne_points/data/000000";
    std::string lidarFileType = ".bin";

    // calibration data for camera and lidar
    cv::Mat P_rect_00(3, 4, cv::DataType<double>::type); // 3x4 projection matrix after rectification
    cv::Mat R_rect_00(4, 4, cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat RT(4, 4, cv::DataType<double>::type);        // rotation matrix and translation vector

    RT.at<double>(0, 0) = 7.533745e-03;
    RT.at<double>(0, 1) = -9.999714e-01;
    RT.at<double>(0, 2) = -6.166020e-04;
    RT.at<double>(0, 3) = -4.069766e-03;
    RT.at<double>(1, 0) = 1.480249e-02;
    RT.at<double>(1, 1) = 7.280733e-04;
    RT.at<double>(1, 2) = -9.998902e-01;
    RT.at<double>(1, 3) = -7.631618e-02;
    RT.at<double>(2, 0) = 9.998621e-01;
    RT.at<double>(2, 1) = 7.523790e-03;
    RT.at<double>(2, 2) = 1.480755e-02;
    RT.at<double>(2, 3) = -2.717806e-01;
    RT.at<double>(3, 0) = 0.0;
    RT.at<double>(3, 1) = 0.0;
    RT.at<double>(3, 2) = 0.0;
    RT.at<double>(3, 3) = 1.0;

    R_rect_00.at<double>(0, 0) = 9.999239e-01;
    R_rect_00.at<double>(0, 1) = 9.837760e-03;
    R_rect_00.at<double>(0, 2) = -7.445048e-03;
    R_rect_00.at<double>(0, 3) = 0.0;
    R_rect_00.at<double>(1, 0) = -9.869795e-03;
    R_rect_00.at<double>(1, 1) = 9.999421e-01;
    R_rect_00.at<double>(1, 2) = -4.278459e-03;
    R_rect_00.at<double>(1, 3) = 0.0;
    R_rect_00.at<double>(2, 0) = 7.402527e-03;
    R_rect_00.at<double>(2, 1) = 4.351614e-03;
    R_rect_00.at<double>(2, 2) = 9.999631e-01;
    R_rect_00.at<double>(2, 3) = 0.0;
    R_rect_00.at<double>(3, 0) = 0;
    R_rect_00.at<double>(3, 1) = 0;
    R_rect_00.at<double>(3, 2) = 0;
    R_rect_00.at<double>(3, 3) = 1;

    P_rect_00.at<double>(0, 0) = 7.215377e+02;
    P_rect_00.at<double>(0, 1) = 0.000000e+00;
    P_rect_00.at<double>(0, 2) = 6.095593e+02;
    P_rect_00.at<double>(0, 3) = 0.000000e+00;
    P_rect_00.at<double>(1, 0) = 0.000000e+00;
    P_rect_00.at<double>(1, 1) = 7.215377e+02;
    P_rect_00.at<double>(1, 2) = 1.728540e+02;
    P_rect_00.at<double>(1, 3) = 0.000000e+00;
    P_rect_00.at<double>(2, 0) = 0.000000e+00;
    P_rect_00.at<double>(2, 1) = 0.000000e+00;
    P_rect_00.at<double>(2, 2) = 1.000000e+00;
    P_rect_00.at<double>(2, 3) = 0.000000e+00;

    // misc
    double sensorFrameRate = 10.0 / imgStepWidth;                     // frames per second for Lidar and camera
    int dataBufferSize = 2;                                           // no. of images which are held in memory (ring buffer) at the same time
    RingBuffer<DataFrame> dataBuffer(dataBufferSize);                 // list of data frames which are held in memory at the same time // list of data frames which are held in memory at the same time
    bool bVis = false;                                                // visualize results
    bool saveKeyPointImages = true;                                   // flag for saving keypoint image pairs
    bool saveTTCImages = true;                                        // flag for saving TTC images
    bool saveTopDown = true;                                            // flag for saving topdown images
    std::vector<std::pair<std::string, cv::Mat>> keyPoinImagesHolder; // used for saving images
    std::vector<std::pair<std::string, cv::Mat>> topDownImagesHolder; // used for saving images
    std::vector<std::pair<std::string, cv::Mat>> TTCImagesHolder; // used for saving images


    PerformanceStorage perStorage;

    if (doesFileExist(file_name))
        remove(file_name.c_str());

    // Clear old results
    //Prepare results folder
    if (saveKeyPointImages || saveTopDown || saveTTCImages)
    {
        fs::remove_all(fs::path(finalImagesPath));
    }

    if(!fs::is_directory(fs::path(finalImagesPath)))
        fs::create_directory(fs::path(finalImagesPath));


    std::ofstream output_data(file_name);

    // Generating different combos
    std::vector<std::string> detectors = {
        "SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};

    std::vector<std::string> descriptors = {
        "BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"};

    std::vector<std::string> matcherTypes = {
        "MAT_BF", "MAT_FLANN"};

    std::vector<std::string> descriptorTypes = {
        "DES_BINARY", "DES_HOG"};

    std::vector<std::string> selectorTypes = {
        "SEL_NN", "SEL_KNN"};

    // Generate Header
    output_data << "Detector Type"
                << ","
                << "Descriptor Type"
                << ","
                << "Matcher Type"
                << ","
                << "Descriptor Content Type"
                << ","
                << "Selector Type"
                << ","
                << "Total N keypoints"
                << ","
                << "Only on vehicle Keypoints"
                << ","
                << "N of Matched keypoints"
                << ","
                << "Detector performance"
                << ","
                << "Descriptor performance"
                << ","
                << "Matcher+Selector performance"
                << ","
                << "Detector+Descriptor performance"
                << ","
                << "Matching BBox performance"
                << ","
                << "Total time"
                << ","
                << "Image pairs"
                << ","
                << "TTC Camera"
                << ","
                << "TTC Lidar" 
                << "," 
                << "Lidar distance to car"
                << std::endl;

    /* MAIN LOOP OVER ALL IMAGES */
    try
    {
        /* code */
        for (auto detector : detectors)
        {
            for (auto descriptor : descriptors)
            {
                if (detector.compare("SIFT") == 0 && descriptor.compare("ORB") == 0)
                {
                    std::cout << "SIFT and ORB dont work together. Skipping..." << std::endl;
                    continue;
                }

                for (auto matcher : matcherTypes)
                {
                    for (auto selector : selectorTypes)
                    {
                        perStorage.detectorType = detector;
                        // AKAZE only works with AKAZE keypoints

                        if ((detector.compare("AKAZE") != 0 && descriptor.compare("AKAZE") == 0))
                        {
                            std::cout << "AKAZE only works with AKAZE. Skipping..." << std::endl;
                            continue;
                        }

                        perStorage.descriptorContentType = descriptor == "SIFT" ? "DES_HOG" : "DES_BINARY";
                        perStorage.descriptorType = descriptor;
                        perStorage.matcherType = matcher;
                        perStorage.selectorType = selector;

                        finalImagesDirectoryName = perStorage.detectorType + "_" + perStorage.descriptorType + "_" + perStorage.matcherType + "_" + perStorage.descriptorContentType + "_" + perStorage.selectorType;

                        std::cout << "Current combination: " << finalImagesDirectoryName << std::endl;

                        for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex += imgStepWidth)
                        {
                            /* LOAD IMAGE INTO BUFFER */
                            std::cout << "++++++IMAGE INDEX++++++++++++: => " + std::to_string(imgIndex) << std::endl;
                            // assemble filenames for current index
                            std::ostringstream imgNumber;
                            imgNumber << std::setfill('0') << std::setw(imgFillWidth) << imgStartIndex + imgIndex;
                            std::string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

                            // load image from file
                            cv::Mat img = cv::imread(imgFullFilename);

                            // push image into data frame buffer
                            DataFrame frame;
                            frame.cameraImg = img;
                            dataBuffer.push_back(frame);

                            std::cout << "#1 : LOAD IMAGE INTO BUFFER done" << std::endl;

                            /* DETECT & CLASSIFY OBJECTS */

                            float confThreshold = 0.2;
                            float nmsThreshold = 0.4;

                            detectObjects(dataBuffer.end().cameraImg, dataBuffer.end().boundingBoxes, confThreshold, nmsThreshold,
                                          yoloBasePath, yoloClassesFile, yoloModelConfiguration, yoloModelWeights, bVis);

                            std::cout << "#2 : DETECT & CLASSIFY OBJECTS done" << std::endl;

                            /* CROP LIDAR POINTS */

                            // load 3D Lidar points from file
                            std::string lidarFullFilename = imgBasePath + lidarPrefix + imgNumber.str() + lidarFileType;
                            std::vector<LidarPoint> lidarPoints;
                            loadLidarFromFile(lidarPoints, lidarFullFilename);

                            // remove Lidar points based on distance properties
                            float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1; // focus on ego lane
                            cropLidarPoints(lidarPoints, minX, maxX, maxY, minZ, maxZ, minR);

                            dataBuffer.end().lidarPoints = lidarPoints;

                            std::cout << "#3 : CROP LIDAR POINTS done" << std::endl;

                            /* CLUSTER LIDAR POINT CLOUD */

                            // associate Lidar points with camera-based ROI
                            float shrinkFactor = 0.10; // shrinks each bounding box by the given percentage to avoid 3D object merging at the edges of an ROI
                            clusterLidarWithROI(dataBuffer.end().boundingBoxes, dataBuffer.end().lidarPoints, shrinkFactor, P_rect_00, R_rect_00, RT);

                            // Visualize 3D objects
                            cv::Mat topdownimage;
                            if (saveTopDown)
                            {
                                topdownimage = show3DObjects(dataBuffer.end().boundingBoxes, cv::Size(4.0, 20.0), cv::Size(2000, 2000), false);

                                std::string imagePairName = "TopDownFrame_" + std::to_string(imgIndex) + ".bmp";

                                topDownImagesHolder.push_back(std::make_pair(imagePairName, topdownimage));
                            }
                            bVis = false;

                            std::cout << "#4 : CLUSTER LIDAR POINT CLOUD done" << std::endl;

                            // REMOVE THIS LINE BEFORE PROCEEDING WITH THE FINAL PROJECT
                            // continue; // skips directly to the next image without processing what comes beneath

                            /* DETECT IMAGE KEYPOINTS */

                            // convert current image to grayscale
                            cv::Mat imgGray;
                            cv::cvtColor(dataBuffer.end().cameraImg, imgGray, cv::COLOR_BGR2GRAY);

                            // extract 2D keypoints from current image
                            std::vector<cv::KeyPoint> keypoints; // create empty feature list for current image
                            double tDetectorPerformance = 0.0;
                            if (perStorage.detectorType.compare("SHITOMASI") == 0)
                            {
                                tDetectorPerformance = detKeypointsShiTomasi(keypoints, imgGray, false);
                            }
                            else if (perStorage.detectorType.compare("HARRIS") == 0)
                            {
                                tDetectorPerformance = detKeypointsHarris(keypoints, imgGray, false);
                            }
                            else
                            {
                                tDetectorPerformance = detKeypointsModern(keypoints, imgGray, perStorage.detectorType, false);
                            }

                            perStorage.detectorPerformance += tDetectorPerformance;

                            // optional : limit number of keypoints (helpful for debugging and learning)
                            bool bLimitKpts = false;
                            if (bLimitKpts)
                            {
                                int maxKeypoints = 50;

                                if (perStorage.detectorType.compare("SHITOMASI") == 0)
                                { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                                    keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
                                }
                                cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                                std::cout << " NOTE: Keypoints have been limited!" << std::endl;
                            }

                            // push keypoints and descriptor for current frame to end of data buffer
                            dataBuffer.end().keypoints = keypoints;
                            perStorage.totalKeypoints = keypoints.size();

                            std::cout << "#5 : DETECT KEYPOINTS done" << std::endl;

                            /* EXTRACT KEYPOINT DESCRIPTORS */

                            cv::Mat descriptors;
                            perStorage.descriptorPerformance += descKeypoints(dataBuffer.end().keypoints, dataBuffer.end().cameraImg, descriptors, perStorage.descriptorType);

                            // push descriptors for current frame to end of data buffer
                            dataBuffer.end().descriptors = descriptors;

                            std::cout << "#6 : EXTRACT DESCRIPTORS done" << std::endl;

                            if (dataBuffer.full()) // wait until at least two images have been processed
                            {
                                // Retrieve the 2 stored frames in the buffer
                                auto dataBufferCopy = dataBuffer;
                                DataFrame prev_frame = dataBufferCopy.get();
                                DataFrame current_frame = dataBufferCopy.get();
                                /* MATCH KEYPOINT DESCRIPTORS */

                                std::vector<cv::DMatch> matches;
                                // std::string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
                                // std::string descriptorType = "DES_BINARY"; // DES_BINARY, DES_HOG
                                // std::string selectorType = "SEL_NN";       // SEL_NN, SEL_KNN

                                perStorage.matcherSelectorPerformance += matchDescriptors(prev_frame.keypoints, current_frame.keypoints,
                                                                                          prev_frame.descriptors, current_frame.descriptors,
                                                                                          matches, perStorage.descriptorContentType, perStorage.matcherType, perStorage.selectorType);

                                // store matches in current data frame
                                current_frame.kptMatches = matches;

                                std::cout << "#7 : MATCH KEYPOINT DESCRIPTORS done" << std::endl;

                                /* TRACK 3D OBJECT BOUNDING BOXES */

                                //// STUDENT ASSIGNMENT
                                //// TASK FP.1 -> match list of 3D objects (vector<BoundingBox>) between current and previous frame (implement ->matchBoundingBoxes)
                                std::map<int, int> bbBestMatches;
                                perStorage.matchBBoxPerformance += matchBoundingBoxes(matches, bbBestMatches, prev_frame, current_frame); // associate bounding boxes between current and previous frame using keypoint matches
                                //// EOF STUDENT ASSIGNMENT

                                // store matches in current data frame
                                current_frame.bbMatches = bbBestMatches;

                                std::cout << "#8 : TRACK 3D OBJECT BOUNDING BOXES done" << std::endl;

                                /* COMPUTE TTC ON OBJECT IN FRONT */

                                // loop over all BB match pairs
                                for (auto it1 = current_frame.bbMatches.begin(); it1 != current_frame.bbMatches.end(); ++it1)
                                {
                                    // find bounding boxes associates with current match
                                    BoundingBox *prevBB, *currBB;
                                    for (auto it2 = current_frame.boundingBoxes.begin(); it2 != current_frame.boundingBoxes.end(); ++it2)
                                    {
                                        if (it1->second == it2->boxID) // check wether current match partner corresponds to this BB
                                        {
                                            currBB = &(*it2);
                                        }
                                    }

                                    for (auto it2 = prev_frame.boundingBoxes.begin(); it2 != prev_frame.boundingBoxes.end(); ++it2)
                                    {
                                        if (it1->first == it2->boxID) // check wether current match partner corresponds to this BB
                                        {
                                            prevBB = &(*it2);
                                        }
                                    }

                                    // compute TTC for current match
                                    if (currBB->lidarPoints.size() > 0 && prevBB->lidarPoints.size() > 0) // only compute TTC if we have Lidar points
                                    {
                                        //// STUDENT ASSIGNMENT
                                        //// TASK FP.2 -> compute time-to-collision based on Lidar data (implement -> computeTTCLidar)
                                        double ttcLidar;
                                        perStorage.LidarDistance = computeTTCLidar(prevBB->lidarPoints, currBB->lidarPoints, sensorFrameRate, ttcLidar);
                                        perStorage.TTCLidar = ttcLidar;
                                        //// EOF STUDENT ASSIGNMENT
                                        std::cout << "COMPUTE LIDAR TTC done" << std::endl;

                                        //// STUDENT ASSIGNMENT
                                        //// TASK FP.3 -> assign enclosed keypoint matches to bounding box (implement -> clusterKptMatchesWithROI)
                                        //// TASK FP.4 -> compute time-to-collision based on camera (implement -> computeTTCCamera)
                                        double ttcCamera;
                                        clusterKptMatchesWithROI(*currBB, prev_frame.keypoints, current_frame.keypoints, current_frame.kptMatches);
                                        computeTTCCamera(prev_frame.keypoints, current_frame.keypoints, currBB->kptMatches, sensorFrameRate, ttcCamera);
                                        perStorage.TTCCamera = ttcCamera;
                                        perStorage.filteredKeypoints = currBB->keypoints.size();
                                        perStorage.matchedKeypoints += currBB->kptMatches.size();

                                        //// EOF STUDENT ASSIGNMENT
                                        std::cout << "COMPUTE CAMERA TTC done" << std::endl;

                                        if (bVis || saveKeyPointImages)
                                        {
                                            cv::Mat matchImg;
                                            cv::drawMatches(prev_frame.cameraImg, prev_frame.keypoints,
                                                            current_frame.cameraImg, current_frame.keypoints,
                                                            currBB->kptMatches, matchImg,
                                                            cv::Scalar::all(-1), cv::Scalar::all(-1),
                                                            std::vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                                            if (saveKeyPointImages && keyPoinImagesHolder.empty() && imgIndex - 1 != imgEndIndex)
                                            {
                                                std::string imagePairName = "Keypoints_" + std::to_string(imgIndex - 1) + "_" + std::to_string(imgIndex) + ".bmp";

                                                keyPoinImagesHolder.push_back(std::make_pair(imagePairName, matchImg));
                                            }
                                            else if (bVis)
                                            {
                                                std::string windowName = "Matching keypoints between two camera images";
                                                cv::namedWindow(windowName, 7);
                                                cv::imshow(windowName, matchImg);
                                                std::cout << "Press key to continue to next image" << std::endl;
                                                cv::waitKey(0); // wait for key to be pressed
                                            }
                                        }


                                        // bVis = true;
                                        if (bVis || saveTTCImages)
                                        {
                                            cv::Mat visImg = current_frame.cameraImg.clone();
                                            showLidarImgOverlay(visImg, currBB->lidarPoints, P_rect_00, R_rect_00, RT, &visImg);
                                            cv::rectangle(visImg, cv::Point(currBB->roi.x, currBB->roi.y), cv::Point(currBB->roi.x + currBB->roi.width, currBB->roi.y + currBB->roi.height), cv::Scalar(0, 255, 0), 2);

                                            char str[200];
                                            sprintf(str, "TTC Lidar : %f s, TTC Camera : %f s", ttcLidar, ttcCamera);
                                            putText(visImg, str, cv::Point2f(80, 50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255));

                                            if (saveTTCImages && TTCImagesHolder.empty() && imgIndex - 1 != imgEndIndex)
                                            {
                                                std::string imagePairName = "TTC_" +  std::to_string(imgIndex - 1)
                                                                                + "_"
                                                                                + std::to_string(imgIndex)
                                                                                + ".bmp";
                                                TTCImagesHolder.push_back(std::make_pair(imagePairName, visImg));
                                            }
                                            else if (bVis)
                                            {
                                                std::string windowName = "Final Results : TTC";
                                                cv::namedWindow(windowName, 4);
                                                cv::imshow(windowName, visImg);
                                                std::cout << "Press key to continue to next frame" << std::endl;
                                                cv::waitKey(0);
                                            }
                                        }

                                        bVis = false;
                                    }
                                     // eof TTC computation
                                }

                                // Total performance
                                perStorage.totalPerformance = perStorage.descriptorPerformance
                                                            + perStorage.detectorPerformance
                                                            + perStorage.matcherSelectorPerformance
                                                            + perStorage.matchBBoxPerformance;
                                // // Generate Header
                                // output_data << "Detector Type"
                                //             << ","
                                //             << "Descriptor Type"
                                //             << ","
                                //             << "Matcher Type"
                                //             << ","
                                //             << "Descriptor Content Type"
                                //             << ","
                                //             << "Selector Type"
                                //             << ","
                                //             << "Total N keypoints"
                                //             << ","
                                //             << "Only on vehicle Keypoints"
                                //             << ","
                                //             << "N of Matched keypoints"
                                //             << ","
                                //             << "Detector performance"
                                //             << ","
                                //             << "Descriptor performance"
                                //             << ","
                                //             << "Matcher+Selector performance"
                                //             << ","
                                //             << "Detector+Descriptor performance"
                                //             << ","
                                //             << "Matching BBox performance"
                                //             << ","
                                //             << "Total time"
                                //             << ","
                                //             << "TTC Camera"
                                //             << ","
                                //             << "TTC Lidar" << std::endl;

                                output_data << perStorage.detectorType << ","
                                            << perStorage.descriptorType << ","
                                            << perStorage.matcherType << ","
                                            << perStorage.descriptorContentType << ","
                                            << perStorage.selectorType << ","
                                            << perStorage.totalKeypoints << ","
                                            << perStorage.filteredKeypoints << ","
                                            << perStorage.matchedKeypoints << ","
                                            << perStorage.detectorPerformance << ","
                                            << perStorage.descriptorPerformance << ","
                                            << perStorage.matcherSelectorPerformance << ","
                                            << perStorage.detectorPerformance + perStorage.descriptorPerformance << ","
                                            << perStorage.matchBBoxPerformance << ","
                                            << perStorage.totalPerformance << ","
                                            << (TTCImagesHolder.empty() 
                                             ? std::string("TTC_" + std::to_string(imgIndex - 1) + "_" + std::to_string(imgIndex) + ".bmp")
                                             : TTCImagesHolder.front().first) << ","
                                            << perStorage.TTCCamera << ","
                                            << perStorage.TTCLidar << ","
                                            << perStorage.LidarDistance 
                                            << std::endl;


                                perStorage.descriptorPerformance = 0.0;
                                perStorage.detectorPerformance = 0.0;
                                perStorage.matcherSelectorPerformance = 0.0;
                                perStorage.matchBBoxPerformance = 0.0;
                                perStorage.totalPerformance = 0.0;

                                perStorage.totalKeypoints = 0l;
                                perStorage.filteredKeypoints = 0l;
                                perStorage.matchedKeypoints = 0l;
                                perStorage.TTCCamera = 0.0;
                                perStorage.TTCLidar = 0.0;
                                perStorage.LidarDistance = 0.0;
                                
                                // Save images based on flags
                                if(saveKeyPointImages && keyPoinImagesHolder.size() != 0)
                                {
                                    auto fullPath = fs::current_path() / fs::path(finalImagesPath) / fs::path(finalImagesDirectoryName);

                                    if(!fs::exists(fullPath))
                                        fs::create_directory(fullPath);

                                    auto nameAndImage = keyPoinImagesHolder.front();

                                    auto pathWithImageName = fullPath / nameAndImage.first;
                                    cv::imwrite(pathWithImageName, nameAndImage.second);
                                    //Just to have some delay between image pairs
                                    std::this_thread::sleep_for(std::chrono::milliseconds(10));

                                    keyPoinImagesHolder.clear();
                                }

                                if(saveTTCImages && TTCImagesHolder.size() != 0)
                                {
                                    auto fullPath = fs::current_path() / fs::path(finalImagesPath) / fs::path(finalImagesDirectoryName);

                                    if(!fs::exists(fullPath))
                                        fs::create_directory(fullPath);

                                    auto nameAndImage = TTCImagesHolder.front();

                                    auto pathWithImageName = fullPath / nameAndImage.first;
                                    cv::imwrite(pathWithImageName, nameAndImage.second);
                                    //Just to have some delay between image pairs
                                    std::this_thread::sleep_for(std::chrono::milliseconds(10));

                                    TTCImagesHolder.clear();
                                }
                                
                                if(saveTopDown && topDownImagesHolder.size() != 0)
                                {
                                    auto fullPath = fs::current_path() / fs::path(finalImagesPath) / fs::path(finalImagesDirectoryName);
                                    if(!fs::exists(fullPath))
                                        fs::create_directory(fullPath);

                                    for(auto i = 0; i < topDownImagesHolder.size(); i++)
                                    {
                                        auto nameAndImage = topDownImagesHolder[i];

                                        auto pathWithImageName = fullPath / nameAndImage.first;
                                        cv::imwrite(pathWithImageName, nameAndImage.second);
                                        //Just to have some delay between image pairs
                                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                                    }

                                    topDownImagesHolder.clear();    
                                }

                                // dataBuffer.reset();
                                // eof loop over all BB matches
                            }
                        } // eof loop over all images
                        dataBuffer.reset();
                    }
                }
            }
        }
    }
    catch (cv::Exception &e)
    {
        std::cerr << "CAUGHT OPENCV EXCEPTION: " << e.what() << '\n';
    }

    std::cout << "Complete..." << std::endl;
    return 0;
}
