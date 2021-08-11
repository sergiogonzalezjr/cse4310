/*
Sergio Gonzalez
CSE 4310 HW4

This program was built using the existing pcl_heaadless as a basis, with code merged in from pcl_plane.
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
/***********************************************************************************************************************
* @file pcl_headless.cpp
* @brief loads a PCD file, makes some changes, and saves an output PCD file
*
* Simple example of loading and saving PCD files, can be used as a template for processing saved data
*
* @author Christopher D. McMurrough
**********************************************************************************************************************/

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/time.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/io.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/extract_clusters.h>

#define NUM_COMMAND_ARGS 2


/***********************************************************************************************************************
* @brief Opens a point cloud file
*
* Opens a point cloud file in either PCD or PLY format
*
* @param[out] cloudOut pointer to opened point cloud
* @param[in] filename path and name of input file
* @return false if an error occurred while opening file
* @author Christopher D. McMurrough
**********************************************************************************************************************/
bool openCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloudOut, std::string fileName)
{
    // handle various file types
    std::string fileExtension = fileName.substr(fileName.find_last_of(".") + 1);
    if(fileExtension.compare("pcd") == 0)
    {
        // attempt to open the file
        if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(fileName, *cloudOut) == -1)
        {
            PCL_ERROR("error while attempting to read pcd file: %s \n", fileName.c_str());
            return false;
        }
        else
        {
            return true;
        }
    }
    else if(fileExtension.compare("ply") == 0)
    {
        // attempt to open the file
        if(pcl::io::loadPLYFile<pcl::PointXYZRGBA>(fileName, *cloudOut) == -1)
        {
            PCL_ERROR("error while attempting to read pcl file: %s \n", fileName.c_str());
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        PCL_ERROR("error while attempting to read unsupported file: %s \n", fileName.c_str());
        return false;
    }
}

/*******************************************************************************************************************//**
 * @brief Saves a point cloud to file
 *
 * Saves a given point cloud to disk in PCD format
 *
 * @param[in] cloudIn pointer to output point cloud
 * @param[in] filename path and name of output file
 * @param[in] binaryMode saves the file in binary form if true (default:false)
 * @return false if an error occured while writing file
 * @author Christopher D. McMurrough
 **********************************************************************************************************************/
bool saveCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloudIn, std::string fileName, bool binaryMode=true)
{
    // if the input cloud is empty, return
    if(cloudIn->points.size() == 0)
    {
        return false;
    }

    // attempt to save the file
    if(pcl::io::savePCDFile<pcl::PointXYZRGBA>(fileName, *cloudIn, binaryMode) == -1)
    {
        PCL_ERROR("error while attempting to save pcd file: %s \n", fileName);
        return false;
    }
    else
    {
        return true;
    }
}


void segmentPlane(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloudIn, pcl::PointIndices::Ptr &inliers, double distanceThreshold, int maxIterations)
{
    // store the model coefficients
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    // Create the segmentation object for the planar model and set the parameters
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloudIn);
    seg.segment(*inliers, *coefficients);
}

/***********************************************************************************************************************
* @brief program entry point
* @param[in] argc number of command line arguments
* @param[in] argv string array of command line arguments
* @returnS return code (0 for normal termination)
* @author Christoper D. McMurrough
**********************************************************************************************************************/
int main(int argc, char** argv)
{
    // validate and parse the command line arguments
    if(argc != NUM_COMMAND_ARGS)
    {
        std::printf("USAGE: %s <file_name>\n", argv[0]);
        return 0;
    }
	std::string inputFilePath(argv[1]);
	std::string outputFilePath = "output.pcd";

    // create a stop watch for measuring time
    //pcl::StopWatch watch;

    // open the point cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGBA>);
    openCloud(cloud_in, inputFilePath);
    
    // segment largest plane
    const float distanceThreshold = 0.0254;
    const int maxIterations = 5000;
    pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
    
    while(1)
    {
    	segmentPlane(cloud_in, plane_inliers, distanceThreshold, maxIterations);
    	
    	if(plane_inliers->indices.size() > 35000)
    	{
    		break;
    	}
    }
    //std::cout << "Segmentation result: " << plane_inliers->indices.size() << " points" << std::endl;
    
    // color the plane plane_inliers green
    for(int i = 0; i < plane_inliers->indices.size(); i++)
    {
        int index = plane_inliers->indices.at(i);
        cloud_in->points.at(index).r = 0;
        cloud_in->points.at(index).g = 0;
        cloud_in->points.at(index).b = 255;
    }

    // extract a cloud sans tabletop
    pcl::ExtractIndices<pcl::PointXYZRGBA> filter;
    filter.setInputCloud(cloud_in);
    filter.setIndices(plane_inliers);
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_clusters(new pcl::PointCloud<pcl::PointXYZRGBA>);
    //pcl::PointIndices::Ptr clusters_inliers(new pcl::PointIndices);
    filter.setNegative(true);
    filter.setKeepOrganized(true);
    filter.filter(*cloud_clusters);
    
    // cloud downsample using voxel grid
    /*const float voxelSize = 0.01;
    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::VoxelGrid<pcl::PointXYZRGBA> voxFilter;
    voxFilter.setInputCloud(cloud_clusters);
    voxFilter.setLeafSize(static_cast<float>(voxelSize), static_cast<float>(voxelSize), static_cast<float>(voxelSize));
    voxFilter.filter(*cloud_clusters);*/
    
    // cluster
    // create the vector of indices lists (each element contains a list of imultiple indices)
    const float clusterDistance = 0.02;
    int minClusterSize = 1000;
    int maxClusterSize = 10000;
    std::vector<pcl::PointIndices> clusters_inliers;

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
    tree->setInputCloud(cloud_clusters);

    // create the euclidian cluster extraction object
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
    ec.setClusterTolerance(clusterDistance);
    ec.setMinClusterSize(minClusterSize);
    ec.setMaxClusterSize(maxClusterSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_clusters);

    // perform the clustering
    ec.extract(clusters_inliers);
    std::cout << "Clusters identified: " << clusters_inliers.size() << std::endl;
    
    // check for planarity/sphericality of identified clusters
    /*
    for(int i = 0; i < clusters_inliers.size(); i++)
    {
    
    	//std::cout << clusters_inliers[i].indices.size() << std::endl;
    
    	pcl::ExtractIndices<pcl::PointXYZRGBA> temp_filter;
        temp_filter.setInputCloud(cloud_clusters);
        
        pcl::PointIndices::Ptr inliers_temp(new pcl::PointIndices);
        *inliers_temp = clusters_inliers[i];
        temp_filter.setIndices(inliers_temp);
    
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGBA>);
        temp_filter.setKeepOrganized(true);
        temp_filter.filter(*cloud_temp);
        
        pcl::PointIndices::Ptr plane_inliers_temp(new pcl::PointIndices);
        segmentPlane(cloud_temp, plane_inliers_temp, distanceThreshold, maxIterations);
        std::cout << "Segmentation result: " << plane_inliers_temp->indices.size() << " points" << std::endl;
        // iterate through the cluster points
        
        for(int j = 0; j < clusters_inliers.at(i).indices.size(); j++)
        {
            cloud_in->points.at(clusters_inliers.at(i).indices.at(j)).r = 0;
            cloud_in->points.at(clusters_inliers.at(i).indices.at(j)).g = 255;
            cloud_in->points.at(clusters_inliers.at(i).indices.at(j)).b = 0;
        }
    }*/

    for(int i = 0; i < clusters_inliers.size(); i++)
    {
        // iterate through the cluster points
        for(int j = 0; j < clusters_inliers.at(i).indices.size(); j++)
        {
            if(i == 0)
            {
            	cloud_in->points.at(clusters_inliers.at(i).indices.at(j)).r = 0;
                cloud_in->points.at(clusters_inliers.at(i).indices.at(j)).g = 255;
                cloud_in->points.at(clusters_inliers.at(i).indices.at(j)).b = 0;
            }
            else if(i == 1)
            {
            	cloud_in->points.at(clusters_inliers.at(i).indices.at(j)).r = 255;
                cloud_in->points.at(clusters_inliers.at(i).indices.at(j)).g = 255;
                cloud_in->points.at(clusters_inliers.at(i).indices.at(j)).b = 0;
            }
            else if(i == 2)
            {
            	cloud_in->points.at(clusters_inliers.at(i).indices.at(j)).r = 255;
                cloud_in->points.at(clusters_inliers.at(i).indices.at(j)).g = 0;
                cloud_in->points.at(clusters_inliers.at(i).indices.at(j)).b = 0;
            }
            else
            {
            	cloud_in->points.at(clusters_inliers.at(i).indices.at(j)).r = 255;
                cloud_in->points.at(clusters_inliers.at(i).indices.at(j)).g = 0;
                cloud_in->points.at(clusters_inliers.at(i).indices.at(j)).b = 255;
            }
        }
    }

	// start timing the processing step
    //watch.reset();
	/*
	// color all of the points random colors
	for(int i = 0; i < cloud->points.size(); i++)
	{
		cloud->points.at(i).r = rand() % 256;
		cloud->points.at(i).g = rand() % 256;
		cloud->points.at(i).b = rand() % 256;
	}
	*/
	
    // get the elapsed time
    //double elapsedTime = watch.getTimeSeconds();
    //std::cout << elapsedTime << " seconds passed " << std::endl;

    // save the point cloud
    saveCloud(cloud_in, outputFilePath);

    // exit program
    return 0;
}
