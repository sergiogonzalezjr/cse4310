/*
Sergio Gonzalez
CSE 4310 HW1

This program was built on top of the existing 'cv_clickable'.
*/

//
//    Copyright 2021 Christopher D. McMurrough
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
 * @file cv_clickable.cpp
 * @brief C++ example of basic interaction with an image in OpenCV
 * @author Christopher D. McMurrough
 **********************************************************************************************************************/

// include necessary dependencies
#include <iostream>
#include <stdbool.h>
#include <string>
#include "opencv2/opencv.hpp"


// global variables
std::string inputFileName; // contains filename
int current_tool = 0; // used to keep track of current tool 
bool printed = false; // flag used to track whether the current tool has been printed to the console
bool drawing = false; // flag for pencil tool engagement, set to true when left click, false when left click released
cv::Mat imageIn; // container for image
cv::Vec3b eyedrop_sample(255,255,255); // color in eyedrop tool, always starts as white
cv::Point p1, p2; // point variables for crop tool

/*******************************************************************************************************************//**
 * @brief handler for image click callbacks
 * @param[in] event number of command line arguments
 * @param[in] x string array of command line arguments
 * @param[in] y string array of command line arguments
 * @param[in] flags string array of command line arguments
 * @param[in] userdata string array of command line arguments
 * @return return code (0 for normal termination)
 * @author Christoper D. McMurrough
 **********************************************************************************************************************/
static void clickCallback(int event, int x, int y, int flags, void* userdata)
{	
	
    if (printed == false)
    {
    	// enumeration of the tools is as below
        if (current_tool == 0)
            std::cout << "CURRENT TOOL: EYEDROPPER" << std::endl;
        else if (current_tool == 1)
            std::cout << "CURRENT TOOL: CROP" << std::endl;
        else if (current_tool == 2)
            std::cout << "CURRENT TOOL: PENCIL" << std::endl;
        else if (current_tool == 3)
            std::cout << "CURRENT TOOL: PAINTBUCKET" << std::endl;
        else if (current_tool == 4)
            std::cout << "CURRENT TOOL: RESET" << std::endl;
        
	printed = true;
    }

    if(event == cv::EVENT_LBUTTONDOWN)
    {
        if(current_tool == 0)
        {
        	if((eyedrop_sample[0] != imageIn.at<cv::Vec3b>(y,x)[0]) || (eyedrop_sample[1] != imageIn.at<cv::Vec3b>(y,x)[1]) || (eyedrop_sample[2] != imageIn.at<cv::Vec3b>(y,x)[2]))
        	{
			eyedrop_sample = imageIn.at<cv::Vec3b>(y,x);
			std::cout << "\tCURRENT COLOR: " << (int)eyedrop_sample[0] << "-" << (int)eyedrop_sample[1] << "-" << (int)eyedrop_sample[2] << std::endl;
        	}
        }
        else if(current_tool == 1)
        {
        	p1 = cv::Point(x,y);
        }
        else if(current_tool == 2)
        {
        	drawing = true;
        	imageIn.at<cv::Vec3b>(y,x) = eyedrop_sample;
        	
        	cv::imshow("imageIn", imageIn);
    		cv::waitKey();
        }
        else if(current_tool == 3)
        {
        	floodFill(imageIn, cv::Point(x,y), eyedrop_sample, 0, cv::Scalar(), cv::Scalar(), 4);
        	
        	cv::imshow("imageIn", imageIn);
    		cv::waitKey();
        }
        
    }
    else if(event == cv::EVENT_RBUTTONDOWN)
    {
    	// this code block iterates throughout the tools, and flips the 'printed' variable to false
        current_tool++;
        current_tool %= 5;
        
        printed = false;
    }
    else if((event == cv::EVENT_MOUSEMOVE) && (drawing == true) && (current_tool == 2))
    {
	imageIn.at<cv::Vec3b>(y,x) = eyedrop_sample;
	cv::imshow("imageIn", imageIn);
	cv::waitKey();
    }
    else if(event == cv::EVENT_LBUTTONUP)
    {
    	if(current_tool == 1)
    	{
    		p2 = cv::Point(x,y);
    		
    		imageIn = imageIn(cv::Rect(p1,p2));
    		cv::imshow("imageIn", imageIn);
	    	cv::waitKey();
    	}
    	else if(current_tool == 2)
    	{
    		drawing = false;
    	}
    }
    else if(event == cv::EVENT_LBUTTONDBLCLK && (current_tool == 4))
    {
        imageIn = cv::imread(inputFileName, cv::IMREAD_COLOR);
	cv::imshow("imageIn", imageIn);
    	cv::waitKey();
    }
}

/*******************************************************************************************************************//**
 * @brief program entry point
 * @param[in] argc number of command line arguments
 * @param[in] argv string array of command line arguments
 * @return return code (0 for normal termination)
 * @author Christoper D. McMurrough
 **********************************************************************************************************************/
int main(int argc, char **argv)
{
    // open the input image
    inputFileName = argv[1];
    imageIn = cv::imread(inputFileName, cv::IMREAD_COLOR);

    // check for file error
    if(!imageIn.data)
    {
        std::cout << "Error while opening file " << inputFileName << std::endl;
        return 0;
    }

    cv::imshow("imageIn", imageIn);
    cv::setMouseCallback("imageIn", clickCallback, &imageIn);
    cv::waitKey();
}

