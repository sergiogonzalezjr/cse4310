/*
Sergio Gonzalez
CSE 4310 HW3

This program was built using the existing cv_pmog as a base.
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
 bool rectangle_sorter(cv::RotatedRect const& e1, cv::RotatedRect const& e2)
 {
 	return e1.size.width < e2.size.width;
 }
 
 int white_count(cv::Mat snap)
 {
 	int white_count = 0;
 	
 	for(int x = 0; x < snap.cols; x++)
 	{
 		for(int y = 0; y < snap.rows; y++)
 		{
 			cv::Vec3b eyedrop_sample = snap.at<cv::Vec3b>(y,x);
 			
 			if(((int)eyedrop_sample[0] == 255) && ((int)eyedrop_sample[1] == 255) && ((int)eyedrop_sample[2] == 255))
 			{
 				white_count++;
 			}
 		}
 	}
 	
 	return white_count;
 }
 
 
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
    //cv::namedWindow("captureFrame", cv::WINDOW_AUTOSIZE);
	//cv::namedWindow("fgMask", cv::WINDOW_AUTOSIZE);

	// set background filtering parameters
    const int bgHistory = 500;
    const float bgThreshold = 50;
    const bool bgShadowDetection = true;
    cv::Mat fgMask; //fg mask generated by MOG2 method
    cv::Ptr<cv::BackgroundSubtractor> pMOG2; //MOG2 Background subtractor
    pMOG2 = cv::createBackgroundSubtractorMOG2(bgHistory, bgThreshold, bgShadowDetection);
    
    cv::Rect gate1(cv::Point(238,0), cv::Point(401,10));
    cv::Rect gate2(cv::Point(238,40), cv::Point(401,60));
    cv::Rect gate3(cv::Point(238,145), cv::Point(401,185));
    cv::Rect gate4(cv::Point(238,230), cv::Point(401,270));
    
    int gate_status[4];
    for(int i = 0; i < 4; i++)
    {
    	gate_status[i] = 0;
    }

    bool print = false;
    int westbound_count = 0;
    int eastbound_count = 0;
	
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
        cv::Mat fgClone;
        cv::Mat fgEdges;
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

			// extract the foreground mask from image
			pMOG2->apply(grayFrame, fgMask);

	    //fgClone = cv::Mat::zeros(fgMask.size(), CV_8UC3);
	    fgClone = fgMask.clone();
	    cv::cvtColor(fgClone, fgClone, cv::COLOR_GRAY2BGR);
	    
	    //cv::line(captureFrame, cv::Point(0, 30), cv::Point(639, 30), cv::Scalar(0,0,0), 5, cv::LINE_8, 0);
	    cv::line(fgClone, cv::Point(0, 26), cv::Point(639, 26), cv::Scalar(0,0,0), 5, cv::LINE_8, 0);
	    cv::line(fgClone, cv::Point(0, 90), cv::Point(639, 90), cv::Scalar(0,0,0), 5, cv::LINE_8, 0);
	    cv::line(fgClone, cv::Point(0, 140), cv::Point(639, 140), cv::Scalar(0,0,0), 5, cv::LINE_8, 0);
	    cv::line(fgClone, cv::Point(0, 210), cv::Point(639, 210), cv::Scalar(0,0,0), 5, cv::LINE_8, 0);
	    
	    cv::Mat frameEdges;
	    const double cannyThreshold1 = 50;
	    const double cannyThreshold2 = 200;
	    const int cannyAperture = 3;
	    cv::Canny(fgClone, frameEdges, cannyThreshold1, cannyThreshold2, cannyAperture);
	    
	    // erode and dilate
	    int morphologySize = 2;
	    cv::Mat edgesDilated;
	    cv::dilate(frameEdges, edgesDilated, cv::Mat(), cv::Point(-1,-1), morphologySize);
	    cv::Mat edgesEroded;
	    cv::erode(edgesDilated, edgesEroded, cv::Mat(), cv::Point(-1,-1), morphologySize);
	    
	    std::vector<std::vector<cv::Point>> frameContours;
	    cv::findContours(edgesEroded, frameContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0,0));
	    
	    //std::sort(frameContours.begin(), frameContours.end(), &vector_sorter);
	    
	    int t = frameContours.size();
	    bool restart = false;
	    for(int i = 0; i < t; i++)
	    {
	    	if(restart == true)
	    	{
	    		i = 0;
	    		restart = false;
	    	}
	    	if(frameContours[i].size() <= 5)
	    	{
	    		frameContours.erase(frameContours.begin()+i);	    		
	    		t = frameContours.size();
	    		restart = true;
	    	}
	    }
	    //std::cout<< "outside" << std::endl;
	    
	    cv::RNG rand(1234567);
	    
	    fgEdges = cv::Mat::zeros(frameEdges.size(), CV_8UC3);
	    
	    for(int i = 0; i < frameContours.size(); i++)
	    {
	    	cv::drawContours(fgEdges, frameContours, i, cv::Scalar(rand.uniform(0,256), rand.uniform(0,256), rand.uniform(0,256)));
	    }
	    
	    std::vector<cv::RotatedRect> minAreaRect(frameContours.size());

	    for(int i = 0; i < frameContours.size(); i++)
	    {
	    	minAreaRect[i] = cv::minAreaRect(frameContours[i]);
	    
	    }
	    
	    restart = false;
	    t = minAreaRect.size();
	    for(int i = 0; i < t; i++)
	    {
	    	if(restart == true)
	    	{
	    		i = 0;
	    		restart = false;
	    	}
	    	
	    	if(minAreaRect[i].size.width < 100)
	    	{
	    		minAreaRect.erase(minAreaRect.begin()+i);	    		
	    		t = minAreaRect.size();
	    		restart = true;
	    	}
	    }	    
	    
	    frameRectangles = cv::Mat::zeros(edgesEroded.size(), CV_8UC3);
	    for(int i = 0; i < minAreaRect.size(); i++)
	    {
	    	cv::Point2f rectanglePoints[4];
	    	minAreaRect[i].points(rectanglePoints);
	    	
	    	for(int j = 0; j < 4; j++)
	    	{
	    		cv::line(captureFrame, rectanglePoints[j], rectanglePoints[(j+1) % 4], cv::Scalar(rand.uniform(0,256), rand.uniform(0,256), rand.uniform(0,256)));
	    	}
	    }
	    
	    
	    cv::rectangle(captureFrame, gate1, cv::Scalar(0,0,255), 1, cv::LINE_8, 0);
	    cv::rectangle(captureFrame, gate2, cv::Scalar(0,0,255), 1, cv::LINE_8, 0);
	    cv::rectangle(captureFrame, gate3, cv::Scalar(0,0,255), 1, cv::LINE_8, 0);
	    cv::rectangle(captureFrame, gate4, cv::Scalar(0,0,255), 1, cv::LINE_8, 0);
	    
	    int gate_activation_count[4];
	    
	    gate_activation_count[0] =  white_count(fgClone(gate1));
	    gate_activation_count[1] =  white_count(fgClone(gate2));
	    gate_activation_count[2] =  white_count(fgClone(gate3));
	    gate_activation_count[3] =  white_count(fgClone(gate4));
	    
	    for(int i = 0; i < 4; i++)
	    {
	    	if((gate_activation_count[i] >= 125) && (gate_status[i] == 0))
	    	{
	    		gate_status[i] = 1;
	    		
	    		if(i < 2)
	    		{
	    			westbound_count++;
	    			print = true;
	    		}
	    		else
	    		{
	    			eastbound_count++;
	    		}
	    		//print = true;
	    	}
	    	else if(gate_activation_count[i] <= 10)
	    	{
	    		gate_status[i] = 0;
	    	}
	    }
	    
	    //std::cout << gate_activation_count[0] << "\t" << gate_activation_count[1] << "\t" << gate_activation_count[2] << "\t" << gate_activation_count[3] << "\n" << std::endl;
	    
	    
	    if(print == true)
	    {
	    	std::cout << "WESTBOUND COUNT: " << westbound_count << "\nEASTBOUND COUNT: " << eastbound_count << "\n\n" << std::endl;
	    	print = false;
	    }
	    
	    
	    //std::cout << gate_activation_count[0] << "\t" << gate_activation_count[1] << "\t" << gate_activation_count[2] << "\t" << gate_activation_count[3] << "\n" << std::endl;
	    //std::cout << gate1.area() << "\t" << gate2.area() << "\t" << gate3.area() << "\t" << gate4.area() << std::endl;
	    /*
	    for(int i = 0; i < 5; i++)
	    {
	    	for(int j = 0; j < 3; j++)
	    	{
	    		if(i == 0)
	    		{
	    			cv::rectangle(fgClone, cv::Point(j * (640/3), 0), cv::Point((j+1) * (640/3), (i+1) * 20), cv::Scalar(0,0,255), 1, cv::LINE_8, 0);
	    		}
	    	}
	    }*/
	    //cv::rectangle(captureFrame, cv::Point(0, 0), cv::Point(320, 240), cv::Scalar(0,0,255), 10, cv::LINE_8, 0);
	    //cv::rectangle(fgClone, cv::Point(0, 0), cv::Point(320, 240), cv::Scalar(255,255,255), 10, cv::LINE_8, 0);
            // increment the frame counter
            //frameCount++;
        }
        else
        {
            //std::printf("Unable to acquire image frame! \n");
            return 0;
        }

        // update the GUI window if necessary
        if(captureSuccess)
        {
	    //std::cout << "Frame opened successfully (width=" << fgMask.size().width << " height=" << fgMask.size().height << std::endl;
	    //cv::line(captureFrame, cv::Point(0, 25), cv::Point(639, 25), cv::Scalar(0,0,0), 5, cv::LINE_8, 0);
	    //cv::line(captureFrame, cv::Point(0, 25), cv::Point(639, 25), cv::Scalar(0,0,0), 5, cv::LINE_8, 0);
	    
            cv::imshow("captureFrame", captureFrame);
	    //cv::imshow("fgClone", fgClone);
	    //cv::imshow("fgEdges", fgEdges);
	    //cv::imshow("frameRectangles", frameRectangles);

            // get the number of milliseconds per frame
            int delayMs = (1.0 / captureFPS) * 1000;

            // check for program termination
            if(((char) cv::waitKey(delayMs)) == 'q')
            {
                doCapture = false;
            }
            
        }

        // compute the frame processing time
        //double endTicks = static_cast<double>(cv::getTickCount());
        //double elapsedTime = (endTicks - startTicks) / cv::getTickFrequency();
        //std::cout << "Frame processing time: " << elapsedTime << std::endl;
        
        if(frameCount > 1500)
        {
            return 0;
        }
    }

    // release program resources before returning
    capture.release();
    cv::destroyAllWindows();
}
