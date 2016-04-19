//
//    Copyright 2014 Team WALL-E
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
* @file realsense_request.cpp
* @brief C++ example for receiving realsense image frames via a zmq request / response scheme.
* @author Team WALL-E & Christopher D. McMurrough
***********************************************************************************************************************/

#include <string>
#include <iostream>
#include <zmq.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>

// configuration parameters
#define NUM_COMNMAND_LINE_ARGUMENTS 1
#define DISPLAY_WINDOW_NAME "Received Image"

/*******************************************************************************************************************//**
* @brief program entry point
* @param[in] argc number of command line arguments
* @param[in] argv string array of command line arguments
* @return return code (0 for normal termination)
* @author Christoper D. McMurrough
***********************************************************************************************************************/
int main(int argc, char **argv)
{
    // store display parameters
    bool showFrames = false;
    
    // create the cloud viewer object
    pcl::visualization::CloudViewer m_viewer (DISPLAY_WINDOW_NAME);

    // validate and parse the command line arguments
    if(argc != NUM_COMNMAND_LINE_ARGUMENTS + 1)
    {
        std::printf("USAGE: %s <display_mode> \n", argv[0]);
        std::printf("WARNING: Proceeding with default execution parameters... \n");
        showFrames = true;
    }
    else
    {
        showFrames = atoi(argv[1]) > 0;
    }

    // initialize the zmq context and socket
    zmq::context_t context(1);
    zmq::socket_t subscriber(context, ZMQ_SUB);
    
    // connect to the image server
    std::cout << "Connecting to server..." << std::endl;
    
    subscriber.connect ("tcp://129.107.132.24:5555");
    subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);

    // create a request object
    zmq::message_t request(5);
    memcpy(request.data(), "Hello", 5);

    // get new frames until the user presses the 'q' key
    bool getFrames = true;
    while(getFrames)
    {
        // get the reply
        zmq::message_t reply;
        subscriber.recv(&reply);
        //std::vector<uchar> buffer;
        std::cout << "Received reply: " << reply.size() << " bytes" << std::endl;

        // store the reply data into an image structure
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud->height = 480;
        cloud->width = 640;
        cloud->is_dense = false;
        cloud->points.resize(640*480);
        std::memcpy(cloud->points.data(), reply.data(), reply.size());
        //std::cout << cloud->points[0] << std::endl;
        //cv::Mat image(480, 640, CV_8UC3, reply.data());
        
        //cloud->points = reply.data();

        // display the result
        if(showFrames)
        {
            //cv::imshow(DISPLAY_WINDOW_NAME, image);
            m_viewer.showCloud(cloud);
        }

        // check for program termination
        if(m_viewer.wasStopped ())
        {
            getFrames = false;
        }
    }

    // close the subscriber
    subscriber.close();
    return 0;
}
