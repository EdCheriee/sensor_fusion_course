/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <filesystem>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"
#include "ringBuffer.h"

struct PerformanceStorage
{
    std::string detectorType; //Detector type
    std::string descriptorType; //Descriptor type
    std::string matcherType;        // MAT_BF, MAT_FLANN
    std::string descriptorContentType; // DES_BINARY, DES_HOG
    std::string selectorType;       // SEL_NN, SEL_KNN

    // Time holders
    double detectorPerformance = 0.0;
    double descriptorPerformance = 0.0;
    double matcherSelectorPerformance = 0.0;
    double totalPerformance = 0.0;

    //N keypoint holders
    unsigned long filteredKeypoints = 0l;
    unsigned long totalKeypoints = 0l;
    unsigned long matchedKeypoints = 0l;
};

namespace fs = std::filesystem;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // CSV file
    std::ofstream output_data;
    std::string file_name = "Results/comparison_results.csv";
    // data location
    std::string dataPath = "../";
    std::string finalImagesPath = "../Results";
    std::string finalImagesDirectoryName = "";
    // camera
    std::string imgBasePath = dataPath + "images/";
    std::string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    std::string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    RingBuffer<DataFrame> dataBuffer(dataBufferSize); // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results
    bool saveImages = false;       // flag to save images for later inspection
    std::vector<std::pair<std::string, cv::Mat>> imagesHolder; //used for saving images
    // Generating different combos
    std::vector<std::string> detectors = {
        "SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"
    };

    std::vector<std::string> descriptors = {
        "BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"
    };

    std::vector<std::string> matcherTypes = {
        "MAT_BF", "MAT_FLANN"
    };

    std::vector<std::string> descriptorTypes = {
        "DES_BINARY", "DES_HOG"
    };

    std::vector<std::string> selectorTypes = {
        "SEL_NN", "SEL_KNN"
    };
    
    PerformanceStorage perStorage;

    //Prepare results folder
    for(auto& de : std::filesystem::directory_iterator(fs::current_path() / fs::path(finalImagesPath))) 
    {
        // Remove subdir content:
        fs::remove_all(de.path());
    }

    //Prepare the statistics file
    if(fs::exists(fs::path(dataPath + file_name)))
        fs::remove(fs::path(dataPath + file_name));

    if(!output_data.is_open())
        output_data.open(dataPath + file_name, std::ios::app);

    output_data << "Detector Type" << "," 
                << "Descriptor Type" << ","
                << "Matcher Type" << "," 
                << "Descriptor Content Type" << ","
                << "Selector Type" << ","
                << "Total N keypoints" << ","
                << "Only on vehicle Keypoints" << ","
                << "N of Matched keypoints" << ","
                << "Detector performance" << ","
                << "Descriptor performance" << ","
                << "Matcher+Selector performance" << ","
                << "Detector+Descriptor performance" << ","
                << "Total time" << std::endl;

    /* MAIN LOOP OVER ALL IMAGES */
    try
    {
        /* code */
        for(auto detector : detectors)
        {
            for(auto descriptor : descriptors)
            {
                if(detector.compare("SIFT") == 0 && descriptor.compare("ORB") == 0)
                {
                    std::cout << "SIFT and ORB dont work together. Skipping..." << std::endl;
                    continue;
                }

                for(auto matcher : matcherTypes)
                {
                    for(auto selector : selectorTypes)
                    {
                        perStorage.detectorType = detector;
                        // AKAZE only works with AKAZE keypoints

                        if((detector.compare("AKAZE") != 0 && descriptor.compare("AKAZE") == 0))
                        {
                            std::cout << "AKAZE only works with AKAZE. Skipping..." << std::endl;
                            continue;
                        }

                        perStorage.descriptorContentType = descriptor == "SIFT" ? "DES_HOG" : "DES_BINARY";
                        perStorage.descriptorType = descriptor;
                        perStorage.matcherType = matcher;
                        perStorage.selectorType = selector;

                        finalImagesDirectoryName = perStorage.detectorType
                                                    + "_"
                                                    + perStorage.descriptorType
                                                    + "_"
                                                    + perStorage.matcherType
                                                    + "_"
                                                    + perStorage.descriptorContentType
                                                    + "_"
                                                    + perStorage.selectorType;

                        std::cout << "Current combination: " << finalImagesDirectoryName << std::endl;

                        for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
                        {
                            /* LOAD IMAGE INTO BUFFER */

                            // assemble filenames for current index
                            std::ostringstream imgNumber;
                            imgNumber << std::setfill('0') << std::setw(imgFillWidth) << imgStartIndex + imgIndex;
                            std::string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

                            // load image from file and convert to grayscale
                            cv::Mat img, imgGray;
                            img = cv::imread(imgFullFilename);
                            cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

                            //// STUDENT ASSIGNMENT
                            //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

                            // push image into data frame buffer
                            DataFrame frame;
                            frame.cameraImg = imgGray;
                            dataBuffer.push_back(frame);

                            //// EOF STUDENT ASSIGNMENT
                            std::cout << "#1 : LOAD IMAGE INTO BUFFER done" << std::endl;

                            /* DETECT IMAGE KEYPOINTS */

                            // extract 2D keypoints from current image
                            std::vector<cv::KeyPoint> keypoints; // create empty feature list for current image

                            //// STUDENT ASSIGNMENT
                            //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
                            //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
                            double tDetectorPerformance = 0.0;
                            if (perStorage.detectorType.compare("SHITOMASI") == 0)
                            {
                                tDetectorPerformance = detKeypointsShiTomasi(keypoints, imgGray, false);
                            }
                            else if(perStorage.detectorType.compare("HARRIS") == 0)
                            {
                                tDetectorPerformance = detKeypointsHarris(keypoints, imgGray, false);
                            }
                            else
                            {
                                tDetectorPerformance = detKeypointsModern(keypoints, imgGray, perStorage.detectorType, false);
                            }

                            perStorage.detectorPerformance += tDetectorPerformance;
                            //// EOF STUDENT ASSIGNMENT

                            //// STUDENT ASSIGNMENT
                            //// TASK MP.3 -> only keep keypoints on the preceding vehicle

                            // only keep keypoints on the preceding vehicle
                            bool bFocusOnVehicle = true;
                            cv::Rect vehicleRect(535, 180, 180, 150);
                            if (bFocusOnVehicle)
                            {
                                //Remove all points that are outside the boundind box
                                auto initialSize = keypoints.size();


                                keypoints.erase(std::remove_if(keypoints.begin(), keypoints.end(),
                                [&vehicleRect](const cv::KeyPoint& point) {return !vehicleRect.contains(point.pt);}), keypoints.end());

                                perStorage.totalKeypoints += initialSize;
                                perStorage.filteredKeypoints += keypoints.size();
                                std::cout << "Out of total " << initialSize << " keypoints only " << keypoints.size() << " remained."  << std::endl;
                            }

                            //// EOF STUDENT ASSIGNMENT

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
                            std::cout << "#2 : DETECT KEYPOINTS done" << std::endl;

                            /* EXTRACT KEYPOINT DESCRIPTORS */

                            //// STUDENT ASSIGNMENT
                            //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
                            //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

                            cv::Mat descriptors;
                            perStorage.descriptorPerformance += descKeypoints(dataBuffer.end().keypoints, dataBuffer.end().cameraImg, descriptors, perStorage.descriptorType);
                            //// EOF STUDENT ASSIGNMENT

                            // push descriptors for current frame to end of data buffer
                            dataBuffer.end().descriptors = descriptors;

                            std::cout << "#3 : EXTRACT DESCRIPTORS done" << std::endl;

                            if (dataBuffer.full()) // wait until at least two images have been processed
                            {

                                /* MATCH KEYPOINT DESCRIPTORS */

                                std::vector<cv::DMatch> matches;

                                // Retrieve the 2 stored frames in the buffer
                                DataFrame prev_frame = dataBuffer.get();
                                DataFrame current_frame = dataBuffer.get();

                                //// STUDENT ASSIGNMENT
                                //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
                                //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

                                perStorage.matcherSelectorPerformance += matchDescriptors(prev_frame.keypoints, current_frame.keypoints,
                                                prev_frame.descriptors, current_frame.descriptors,
                                                matches, perStorage.descriptorContentType, perStorage.matcherType, perStorage.selectorType);

                                //// EOF STUDENT ASSIGNMENT



                                // store matches in current data frame
                                current_frame.kptMatches = matches;
                                perStorage.matchedKeypoints += matches.size();

                                std::cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << std::endl;

                                // visualize matches between current and previous image
                                if (bVis || saveImages)
                                {
                                    cv::Mat matchImg;
                                    cv::drawMatches(prev_frame.cameraImg, prev_frame.keypoints,
                                                    current_frame.cameraImg, current_frame.keypoints,
                                                    matches, matchImg,
                                                    cv::Scalar::all(-1), cv::Scalar::all(-1),
                                                    std::vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);


                                    if(saveImages && imagesHolder.empty())
                                    {
                                        std::string imagePairName =  std::to_string(imgIndex - 1)
                                                                    + "_"
                                                                    + std::to_string(imgIndex)
                                                                    + ".bmp";

                                        imagesHolder.push_back(std::make_pair(imagePairName, matchImg));
                                    }
                                    else if(bVis)
                                    {
                                        std::string windowName = "Matching keypoints between two camera images";
                                        cv::namedWindow(windowName, 7);
                                        cv::imshow(windowName, matchImg);
                                        std::cout << "Press key to continue to next image" << std::endl;
                                        cv::waitKey(0); // wait for key to be pressed
                                    }

                                }

                                // output_data << "Detector Type" << "," 
                                //             << "Descriptor Type" << ","
                                //             << "Matcher Type" << "," 
                                //             << "Descriptor Content Type" << ","
                                //             << "Selector Type" << ","
                                //             << "IMG 1 Total N keypoints" << ","
                                //             << "IMG 1 Only on vehicle Keypoints" << ","
                                //             << "IMG 1 Detector performance" << ","
                                //             << "IMG 1 Descriptor performance" << ","
                                //             << "IMG 2 Total N keypoints" << ","
                                //             << "IMG 2 Only on vehicle Keypoints" << ","
                                //             << "IMG 2 Detector performance" << ","
                                //             << "IMG 2 Descriptor performance" << ","
                                //             << "Matcher+Selector performance" << ","
                                //             << "Total time" << std::endl;





                                dataBuffer.reset();


                            }

                        }
                        // Total performance
                        perStorage.totalPerformance = perStorage.descriptorPerformance
                                                    + perStorage.detectorPerformance
                                                    + perStorage.matcherSelectorPerformance;
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
                                    << perStorage.totalPerformance
                                    << std::endl;
                        perStorage.descriptorPerformance = 0.0;
                        perStorage.detectorPerformance = 0.0;
                        perStorage.matcherSelectorPerformance = 0.0;
                        perStorage.totalPerformance = 0.0;

                        perStorage.totalKeypoints = 0l;
                        perStorage.filteredKeypoints = 0l;
                        perStorage.matchedKeypoints = 0l;

                        if(saveImages && imagesHolder.size() != 0)
                        {
                            std::cout << "IMAGES HOLDER SIZE " << imagesHolder.size() << std::endl;
                            auto fullPath = fs::current_path() / fs::path(finalImagesPath) / fs::path(finalImagesDirectoryName);

                            if(!fs::exists(fullPath))
                                fs::create_directory(fullPath);

                            auto nameAndImage = imagesHolder.front();

                            auto pathWithImageName = fullPath / nameAndImage.first;
                            cv::imwrite(pathWithImageName, nameAndImage.second);
                            //Just to have some delay between image pairs
                            std::this_thread::sleep_for(std::chrono::milliseconds(10));

                            imagesHolder.clear();
                        } // eof loop over all images
                    }
                }
            }
        }
    }
    catch(cv::Exception& e)
    {
        std::cerr << "CAUGHT OPENCV EXCEPTION: " << e.what() << '\n';
    }

    std::cout << "Complete..." << std::endl;
    return 0;
}