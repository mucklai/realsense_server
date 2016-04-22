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
#include <ncurses.h>

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
    // character capturing intialization
    initscr();
    int ch;
    nodelay(stdscr, TRUE);
    noecho();
    
    // store display parameters
    bool showFrames = false;
    
    // create the cloud viewer object
    pcl::visualization::CloudViewer m_viewer (DISPLAY_WINDOW_NAME);

    // validate and parse the command line arguments
    if(argc != NUM_COMNMAND_LINE_ARGUMENTS + 1)
    {
        printw("USAGE: %s <display_mode> \n", argv[0]);
        printw("WARNING: Proceeding with default execution parameters... \n");
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
    printw("Connecting to server...\n");
    
    subscriber.connect ("tcp://129.107.132.27:5555");
    
    printw("Press S to stream images and C to capture one image...\n");
    

    // create a request object
    //zmq::message_t request(5);
    //memcpy(request.data(), "Hello", 5);

    // get new frames until the user presses the 'q' key
    bool running = true;
    bool stream = false;
    bool capture = false;
    while(running)
    {
        if ((ch = getch()) == int('s')) { // if presses "stream" button
            if (stream) { // if we were already streaming
                subscriber.setsockopt(ZMQ_UNSUBSCRIBE, "", 0); // UNSUBSCRIBE
                printw("S pressed: Stopping stream.\n");             
            } else { // else
                subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0); //SUBSCRIBE
                printw("S pressed: Starting stream.\n"); 
            }
            stream = !(stream); // toggle stream
        } else if (ch == int('c')) { // else if presses "capture" button
            subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);
            capture = true;
            printw("C pressed: Capturing image.\n"); 
        }
        
        if (stream || capture) {
            //check for subscription
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
            if(showFrames) {
                m_viewer.showCloud(cloud);
            }
            
            if (capture) { // we've received one image and can stop
                subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);
                capture = false;
            }
        } 

        // check for program termination
        if(m_viewer.wasStopped ()) {
            running = false;
        }
    }

    // close the subscriber
    subscriber.close();
    return 0;
}
