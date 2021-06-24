/*
Sergio Gonzalez
CSE 4310 HW1
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
 * @file cv_annotation.cpp
 * @brief C++ example of basic image annotation and ROIs in OpenCV
 * @author Christopher D. McMurrough
 **********************************************************************************************************************/

// include necessary dependencies
#include <iostream>
#include <stdbool.h>
#include <string>
#include "opencv2/opencv.hpp"

enum tool {EYEDROPPER, CROP, PENCIL, PAINTBUCKET, RESET};

tool current_tool = EYEDROPPER;
bool printed = false;

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
        printed = true;

        if (current_tool == EYEDROPPER)
            std::cout << "CURRENT TOOL: EYEDROPPER" << std::endl;
        else if (current_tool == CROP)
            std::cout << "CURRENT TOOL: CROP" << std::endl;
        else if (current_tool == PENCIL)
            std::cout << "CURRENT TOOL: PENCIL" << std::endl;
        else if (current_tool == PAINTBUCKET)
            std::cout << "CURRENT TOOL: PAINTBUCKET" << std::endl;
        else if (current_tool == RESET)
            std::cout << "CURRENT TOOL: RESET" << std::endl;
    }

    if(event == cv::EVENT_LBUTTONDOWN)
    {
        std::cout << "LEFT CLICK (" << x << ", " << y << ")" << std::endl;
    }
    else if(event == cv::EVENT_RBUTTONDOWN)
    {
        current_tool++;
        current_tool%5;
    }
    else if(event == cv::EVENT_MBUTTONDOWN)
    {
        std::cout << "MIDDLE CLICK (" << x << ", " << y << ")" << std::endl;
    }
    else if(event == cv::EVENT_MOUSEMOVE)
    {
        std::cout << "MOUSE OVER (" << x << ", " << y << ")" << std::endl;
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
    std::string inputFileName = argv[1];
    cv::Mat imageIn;
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

    /*
    // display the input image
    cv::imshow("imageIn", imageIn);
    cv::waitKey();

    // create a 200 pixel wide region of interest (ROI) on the center of the image
    cv::Point p1(imageIn.cols / 2 - 100, imageIn.rows / 2 - 100);
    cv::Point p2(imageIn.cols / 2 + 100, imageIn.rows / 2 + 100);
    cv::Rect region(p1, p2);

    // extract the ROI into its own image and display
    cv::Mat imageROI = imageIn(region);
    cv::imshow("imageROI", imageROI);
    cv::waitKey();

    // draw a red rectangle around the ROI and update the display
    cv::rectangle(imageIn, region, cv::Scalar(0, 0, 255), 3);
    cv::imshow("imageIn", imageIn);
    cv::waitKey();

    // draw a blue circle around the ROI and update the display
    cv::Point center(imageIn.cols / 2, imageIn.rows / 2);
    cv::circle(imageIn, center, 100, cv::Scalar(255, 0, 0), 3);
    cv::imshow("imageIn", imageIn);
    cv::waitKey();

    // draw black lines from corner to corner and display
    cv::Point cornerTopLeft(0, 0);
    cv::Point cornerTopRight(imageIn.cols - 1, 0);
    cv::Point cornerBottomLeft(0, imageIn.rows - 1);
    cv::Point cornerBottomRight(imageIn.cols - 1, imageIn.rows - 1);
    cv::line(imageIn, cornerTopLeft, cornerBottomRight, cv::Scalar(0, 0, 0), 3);
    cv::line(imageIn, cornerTopRight, cornerBottomLeft, cv::Scalar(0, 0, 0), 3);
    cv::imshow("imageIn", imageIn);
    cv::waitKey();
    */

}
