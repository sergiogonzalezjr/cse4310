/*
Sergio Gonzalez
CSE 4310 HW3

This program was built on top of the existing 'cv_pmog'
*/

//
//    Copyright 2018 Christopher D. McMurrough
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

/*******************************************************************************************************************//**
 * @file cv_pmog.cpp
 * @brief C++ example of MOG2 background subtraction in video using OpenCV
 * @author Christopher D. McMurrough
 **********************************************************************************************************************/

// include necessary dependencies
#include <iostream>
#include <cstdio>
#include "opencv2/opencv.hpp"

// configuration parameters
#define NUM_COMNMAND_LINE_ARGUMENTS 1

/*******************************************************************************************************************//**
 * @brief program entry point
 * @param[in] argc number of command line arguments
 * @param[in] argv string array of command line arguments
 * @return return code (0 for normal termination)
 * @author Christoper D. McMurrough
 **********************************************************************************************************************/
int main(int argc, char **argv)
{
    // store video capture parameters
    std::string fileName;

    // validate and parse the command line arguments
    if(argc != NUM_COMNMAND_LINE_ARGUMENTS + 1)
    {
        std::printf("USAGE: %s <file_path> \n", argv[0]);
        return 0;
    }
    else
    {
        fileName = argv[1];
    }

    // open the video file
    cv::VideoCapture capture(fileName);
    if(!capture.isOpened())
    {
        std::printf("Unable to open video source, terminating program! \n");
        return 0;
    }

    // get the video source parameters
    int captureWidth = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
    int captureHeight = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
    int captureFPS = static_cast<int>(capture.get(cv::CAP_PROP_FPS));
    std::cout << "Video source opened successfully (width=" << captureWidth << " height=" << captureHeight << " fps=" << captureFPS << ")!" << std::endl;

    // create image window
    cv::namedWindow("captureFrame", cv::WINDOW_AUTOSIZE);
    //cv::namedWindow("fgMask", cv::WINDOW_AUTOSIZE);
    
    cv::Mat fgMask;
/*
	// set background filtering parameters
    const int bgHistory = 100;
    const float bgThreshold = 80;
    const bool bgShadowDetection = false;
    //cv::Mat fgMask; //fg mask generated by MOG2 method
    cv::Ptr<cv::BackgroundSubtractor> pMOG2; //MOG2 Background subtractor
    pMOG2 = cv::createBackgroundSubtractorMOG2(bgHistory, bgThreshold, bgShadowDetection);
*/
    // process data until program termination
    bool doCapture = true;
    int frameCount = 0;
    while(doCapture)
    {
        // get the start time
        double startTicks = static_cast<double>(cv::getTickCount());

        // attempt to acquire and process an image frame
        cv::Mat captureFrame;
        cv::Mat grayFrame;
        cv::Mat frameRectangles;
        //cv::Mat processedFrame;
        bool captureSuccess = capture.read(captureFrame);
        if(captureSuccess)
        {
			// pre-process the raw image frame
            const int rangeMin = 0;
            const int rangeMax = 255;
            cv::cvtColor(captureFrame, grayFrame, cv::COLOR_BGR2GRAY);
            cv::normalize(grayFrame, grayFrame, rangeMin, rangeMax, cv::NORM_MINMAX, CV_8UC1);

	    cv::Mat frameEdges;
	    const double cannyThreshold1 = 100;
	    const double cannyThreshold2 = 150;
	    const int cannyAperture = 3;
	    cv::Canny(grayFrame, frameEdges, cannyThreshold1, cannyThreshold2, cannyAperture);
	    
	    // erode and dilate
	    int morphologySize = 2;
	    cv::Mat edgesDilated;
	    cv::dilate(frameEdges, edgesDilated, cv::Mat(), cv::Point(-1,-1), morphologySize);
	    cv::Mat edgesEroded;
	    cv::erode(edgesDilated, edgesEroded, cv::Mat(), cv::Point(-1,-1), morphologySize);
	    
	    std::vector<std::vector<cv::Point>> frameContours;
	    cv::findContours(edgesEroded, frameContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0,0));
	    
	    for(int i = 0; i < frameContours.size(); i++)
	    {
	    	if(frameContours[i].size() <= 5)
	    	{
			frameContours.erase(frameContours.begin() + i);
	    	}
	    }
	    
	    cv::RNG rand(1234567);
	    
	    fgMask = cv::Mat::zeros(frameEdges.size(), CV_8UC3);
	    
	    for(int i = 0; i < frameContours.size(); i++)
	    {
	    	cv::drawContours(fgMask, frameContours, i, cv::Scalar(rand.uniform(0,256), rand.uniform(0,256), rand.uniform(0,256)));
	    }
	    
	    std::vector<cv::RotatedRect> minAreaRect(frameContours.size());

	    for(int i = 0; i < frameContours.size(); i++)
	    {
	    	minAreaRect[i] = cv::minAreaRect(frameContours[i]);
	    
	    }
	    
	    frameRectangles = cv::Mat::zeros(edgesEroded.size(), CV_8UC3);
	    for(int i = 0; i < frameContours.size(); i++)
	    {
	    	cv::Point2f rectanglePoints[4];
	    	minAreaRect[i].points(rectanglePoints);
	    	
	    	for(int j = 0; j < 4; j++)
	    	{
	    		cv::line(frameRectangles, rectanglePoints[j], rectanglePoints[(j+1) % 4], cv::Scalar(rand.uniform(0,256), rand.uniform(0,256), rand.uniform(0,256)));
	    	}
	    }

            frameCount++;
        }
        else
        {
            std::printf("Unable to acquire image frame! \n");
        }

        // update the GUI window if necessary
        if(captureSuccess)
        {
            cv::imshow("captureFrame", captureFrame);
	    cv::imshow("fgMask", fgMask);
	    cv::imshow("frameRectangles", frameRectangles);

            // get the number of milliseconds per frame
            int delayMs = (1.0 / captureFPS) * 1000;

            // check for program termination
            if(((char) cv::waitKey(delayMs)) == 'q')
            {
                doCapture = false;
            }
        }

        // compute the frame processing time
        double endTicks = static_cast<double>(cv::getTickCount());
        double elapsedTime = (endTicks - startTicks) / cv::getTickFrequency();
        //std::cout << "Frame processing time: " << elapsedTime << std::endl;
    }

    // release program resources before returning
    capture.release();
    cv::destroyAllWindows();
}

