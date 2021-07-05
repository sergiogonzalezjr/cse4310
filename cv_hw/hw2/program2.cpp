/*
Sergio Gonzalez
CSE 4310 HW2

This program was built on top of the existing 'cv_ellipse'.
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
 * @file cv_ellipse.cpp
 * @brief C++ example of Canny edge detection and ellipse model fitting in OpenCV
 * @author Christopher D. McMurrough
 **********************************************************************************************************************/

// include necessary dependencies
#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"

// configuration parameters
#define NUM_COMNMAND_LINE_ARGUMENTS 1

//int dime_count, penny_count, nickel_count, quarter_count, coin_group = 0;
/*
void coin_ticker()
{
	if(coin_group == 0)
		dime_count++;
	else if(coin_group == 1)
		penny_count++;
	else if(coin_group == 2)
		nickel_count++;
	else if(coin_group == 3)
		quarter_count++;
}*/

void print_count_total(std::vector<std::vector<cv::RotatedRect>> coins_grouped)
{
        std::cout << "Penny - " << coins_grouped[1].size() << std::endl;
        std::cout << "Nickel - " << coins_grouped[2].size() << std::endl;
        std::cout << "Dime - " << coins_grouped[0].size() << std::endl;
        std::cout << "Quarter - " << coins_grouped[3].size() << std::endl;
	std::cout << "Total - $" << (coins_grouped[0].size() * 0.1) + (coins_grouped[1].size() * 0.01) + (coins_grouped[2].size() * 0.05) + (coins_grouped[3].size() * 0.25) << std::endl;
}

// subroutine for sorting ellipse by major semi-axis
bool ellipse_sorter(cv::RotatedRect const& e1, cv::RotatedRect const& e2)
{
	return e1.size.width < e2.size.width;
}

// finds average of vector of ints
int average(std::vector<cv::RotatedRect> list)
{
	int sum = 0;
	
	for(int i = 0; i < list.size(); i++)
		sum += list[i].size.width;
		
	return (sum / list.size());
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
    cv::Mat imageIn;

    // validate and parse the command line arguments
    if(argc != NUM_COMNMAND_LINE_ARGUMENTS + 1)
    {
        std::printf("USAGE: %s <image_path> \n", argv[0]);
        return 0;
    }
    else
    {
        imageIn = cv::imread(argv[1], cv::IMREAD_COLOR);

        // check for file error
        if(!imageIn.data)
        {
            std::cout << "Error while opening file " << argv[1] << std::endl;
            return 0;
        }
    }

    // get the image size
    //std::cout << "image width: " << imageIn.size().width << std::endl;
    //std::cout << "image height: " << imageIn.size().height << std::endl;
    //std::cout << "image channels: " << imageIn.channels() << std::endl;

    // convert the image to grayscale
    cv::Mat imageGray;
    cv::cvtColor(imageIn, imageGray, cv::COLOR_BGR2GRAY);

    // find the image edges
    cv::Mat imageEdges;
    const double cannyThreshold1 = 100;
    const double cannyThreshold2 = 200;
    const int cannyAperture = 3;
    cv::Canny(imageGray, imageEdges, cannyThreshold1, cannyThreshold2, cannyAperture);
    
    // erode and dilate the edges to remove noise
    int morphologySize = 1;
    cv::Mat edgesDilated;
    cv::dilate(imageEdges, edgesDilated, cv::Mat(), cv::Point(-1, -1), morphologySize);
    cv::Mat edgesEroded;
    cv::erode(edgesDilated, edgesEroded, cv::Mat(), cv::Point(-1, -1), morphologySize);
    
    // locate the image contours (after applying a threshold or canny)
    std::vector<std::vector<cv::Point> > contours;
    //std::vector<int> hierarchy;
    cv::findContours(edgesEroded, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    // draw the contours
    cv::Mat imageContours = cv::Mat::zeros(imageEdges.size(), CV_8UC3);
    cv::RNG rand(12345);
    for(int i = 0; i < contours.size(); i++)
    {
        cv::Scalar color = cv::Scalar(rand.uniform(0, 256), rand.uniform(0,256), rand.uniform(0,256));
        cv::drawContours(imageContours, contours, i, color);
    }

    // compute minimum area bounding rectangles
    std::vector<cv::RotatedRect> minAreaRectangles(contours.size());
    for(int i = 0; i < contours.size(); i++)
    {
        // compute a minimum area bounding rectangle for the contour
        minAreaRectangles[i] = cv::minAreaRect(contours[i]);
    }

    // draw the rectangles
    cv::Mat imageRectangles = cv::Mat::zeros(imageEdges.size(), CV_8UC3);
    for(int i = 0; i < contours.size(); i++)
    {
        cv::Scalar color = cv::Scalar(rand.uniform(0, 256), rand.uniform(0,256), rand.uniform(0,256));
        cv::Point2f rectanglePoints[4];
        minAreaRectangles[i].points(rectanglePoints);
        for(int j = 0; j < 4; j++)
        {
            cv::line(imageRectangles, rectanglePoints[j], rectanglePoints[(j+1) % 4], color);
        }
    }

    // fit ellipses to contours containing sufficient inliers
    std::vector<cv::RotatedRect> fittedEllipses(contours.size());
    for(int i = 0; i < contours.size(); i++)
    {
        // compute an ellipse only if the contour has more than 5 points (the minimum for ellipse fitting)
        if(contours.at(i).size() > 5)
        {
            fittedEllipses[i] = cv::fitEllipse(contours[i]);
        }
    }

    // draw the ellipses
    cv::Mat imageEllipse = cv::Mat::zeros(imageEdges.size(), CV_8UC3);
    const int minEllipseInliers = 50;
    for(int i = 0; i < contours.size(); i++)
    {
        // draw any ellipse with sufficient inliers
        if(contours.at(i).size() > minEllipseInliers)
        {
            cv::Scalar color = cv::Scalar(rand.uniform(0, 256), rand.uniform(0,256), rand.uniform(0,256));
            cv::ellipse(imageEllipse, fittedEllipses[i], color, 2);
        }
    }

    // display the images
    //cv::imshow("imageIn", imageIn);
    //cv::imshow("imageGray", imageGray);
    //cv::imshow("imageEdges", imageEdges);
    //cv::imshow("edges dilated", edgesDilated);
    //cv::imshow("edges eroded", edgesEroded);
    //cv::imshow("imageContours", imageContours);
    //cv::imshow("imageRectangles", imageRectangles);
    cv::imshow("imageEllipse", imageEllipse);
        
    cv::waitKey();
    
    // sort by major major semi-axis of ellipses
    std::sort(fittedEllipses.begin(), fittedEllipses.end(), &ellipse_sorter);
    
    //tossing out junk entries
    while(1)
    {
    	if(fittedEllipses[0].size.width > 20)
    	{
    		break;
    	}
    	else
    	{
    		fittedEllipses.erase(fittedEllipses.begin());
    	}
    }
    
    
    
    std::vector<std::vector<cv::RotatedRect>> coins_grouped;  // will be used to hold major semi-axis data
    std::vector<cv::RotatedRect> temp_list;
    int coin_group = 0;
    
    for(int i = 0; i < fittedEllipses.size();  i++)
    {
	if(temp_list.empty())
	{
		temp_list.push_back(fittedEllipses[i]);
		//coin_ticker();
		std::cout << i << " EMPTY\t" << coin_group << "\t" << temp_list.size() << std::endl;
		continue;
	}
	
	if(average(temp_list) * 1.05 >= fittedEllipses[i].size.width)
	{
		temp_list.push_back(fittedEllipses[i]);
		//coin_ticker();
	}
	else
	{
		coins_grouped.push_back(temp_list);
		temp_list.clear();
		coin_group++;
		i--;
	}
	std::cout << i << "      \t" << coin_group << "\t" << temp_list.size() << std::endl;
	
    }
    
    coins_grouped.push_back(temp_list);
    temp_list.clear();
    
    print_count_total(coins_grouped);
    
    //for(int i = 0; i < fittedEllipses.size(); i++)
    	//std::cout << fittedEllipses[i].size.width << std::endl;
}

