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
* @file realsense_server.cpp
* @brief C++ example for acquiring and transmitting realsense image frames via a zmq request / response scheme.
* @author Team WALL-E & Christopher D. McMurrough
***********************************************************************************************************************/

// include necessary dependencies
#include <iostream>
#include <cstdio>
#include <zmq.hpp>
#include <librealsense/rs.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>

#define NUM_COMMAND_LINE_ARGUMENTS 2
#define DISPLAY_WINDOW_NAME "Camera Image"
#define MAX_FRAME_RATE 120
#define DEFAULT_FRAME_RATE 120

/*******************************************************************************************************************//**
* @brief program entry point
* @return return code (0 for normal termination)
* @author Team WALL-E
***********************************************************************************************************************/
int main(int argc, char **argv)
{
    // store video capture parameters
    int frameRate = DEFAULT_FRAME_RATE;
    bool showFrames = true;
    
    // validate and parse the command line arguments
    if(argc != NUM_COMMAND_LINE_ARGUMENTS + 1)
    {
        std::printf("USAGE: %s <frame_rate> <display_mode> \n", argv[0]);
        std::printf("WARNING: Proceeding with default execution parameters... \n");
    }
    else
    {
        if (atoi(argv[1]) < 0 || atoi(argv[1]) > MAX_FRAME_RATE) {
            std::printf("WARNING: Invalid frame rate! Valid frame rates are between 0 and %d, or 0 for any frame rate\nProceeding with default frame rate...\n", MAX_FRAME_RATE);
        } else {
            frameRate = atoi(argv[1]);
        }
        showFrames = atoi(argv[2]) > 0;
    }
    
    // initialize the zmq context and socket
    zmq::context_t context(1);
    zmq::socket_t publisher(context, ZMQ_PUB);
    publisher.bind("tcp://*:5555");
    
    // create the realsense context
    rs::context ctx;
    if(ctx.get_device_count() == 0)
    {
        std::printf("No available RealSense devices, terminating program! \n");
        return 0;
    }
    else
    {
        std::printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
    }
	
	rs::device * dev = ctx.get_device(0);
	printf("\nUsing device 0, an %s\n", dev->get_name());
    printf("    Serial number: %s\n", dev->get_serial());
    printf("    Firmware version: %s\n", dev->get_firmware_version());

	// Configure all streams to run at VGA resolution at 60 frames per second
    dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 60);
    dev->enable_stream(rs::stream::color, 640, 480, rs::format::rgb8, 60);
    dev->start();
    
    // create the cloud viewer object
    pcl::visualization::CloudViewer m_viewer (DISPLAY_WINDOW_NAME);
    
    // create a stop watch for measuring time
    pcl::StopWatch m_stopWatch;
    
    // Retrieve camera parameters for mapping between depth and color
    rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
    rs::extrinsics depth_to_color = dev->get_extrinsics(rs::stream::depth, rs::stream::color);
    rs::intrinsics color_intrin = dev->get_stream_intrinsics(rs::stream::color);
    float scale = dev->get_depth_scale();
    
    bool doCapture = true;
    
    while(doCapture)
    {
        // Wait for new frame data
        dev->wait_for_frames();
        
        // start the timer
        m_stopWatch.reset();
        
        // Retrieve our images
        const uint16_t * depth_image = (const uint16_t *)dev->get_frame_data(rs::stream::depth);
        const uint8_t * color_image = (const uint8_t *)dev->get_frame_data(rs::stream::color);
   
        // Initialize point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        //cloud->header.frame_id = "some_tf_frame";
        //cloud->height = 480;
        //cloud->width = 640;
        //cloud->fields.resize(4);
        //cloud->fields[0].name = "x";
        //cloud->fields[1].name = "y";
        //cloud->fields[2].name = "z";
        //cloud->fields[3].name = "rgb";
        cloud->is_dense = false;
        //cloud->points.resize(640*480);
        
        // We will render our depth data as a set of points in 3D space
        for(int dy=0; dy<depth_intrin.height; ++dy)
        {
            for(int dx=0; dx<depth_intrin.width; ++dx)
            {
                // Retrieve the 16-bit depth value and map it into a depth in meters
                uint16_t depth_value = depth_image[dy * depth_intrin.width + dx];
                float depth_in_meters = depth_value * scale;

                // Skip over pixels with a depth value of zero, which is used to indicate no data
                if(depth_value == 0) continue;

                // Map from pixel coordinates in the depth image to pixel coordinates in the color image
                rs::float2 depth_pixel = {(float)dx, (float)dy};
                rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
                rs::float3 color_point = depth_to_color.transform(depth_point);
                rs::float2 color_pixel = color_intrin.project(color_point);

                // Initialize point                
                pcl::PointXYZRGB point; 

                // Use the color from the nearest color pixel, or pure black if this point falls outside the color image
                const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);
                if(cx < 0 || cy < 0 || cx >= color_intrin.width || cy >= color_intrin.height)
                {
                    point = pcl::PointXYZRGB(0, 0, 0);
                }
                else
                {
                    const uint8_t * colour_array = color_image + (cy * color_intrin.width + cx) * 3;
                    point = pcl::PointXYZRGB(colour_array[0], colour_array[1], colour_array[2]);
                }
                
                // Populate point depth values
                point.x = depth_point.x;
                point.y = depth_point.y;
                point.z = -(depth_point.z);                                            
                
                cloud->points.push_back(point);
                //cloud->points[(dy * depth_intrin.height) + dx] = point;
            }
        }
        
        if (showFrames) {
            // Render Cloud
            m_viewer.showCloud(cloud);
        }
        
        // Log time taken to display frame
        double elapsedTime = m_stopWatch.getTimeSeconds();
        m_stopWatch.reset();
        std::printf("FPS: %f \n", 1/elapsedTime);
        
        if (m_viewer.wasStopped ()) {
            doCapture = false;
        }
        
        // check for image requests
        zmq_msg_t msg;
        if(zmq_msg_init(&msg) != 0)
        {
            std::printf("WARNING: ZMQ error (%d) while creating message... \n", zmq_errno());
        }

        size_t frameSize = sizeof(cloud->points) + cloud->points.size() * sizeof(pcl::PointXYZRGB);
        std::cout << "Sending frame: " << frameSize << " bytes" << std::endl;                
        zmq_send((void*) publisher, cloud->points.data(), frameSize, 0);
    }
    
    return EXIT_SUCCESS;
}

