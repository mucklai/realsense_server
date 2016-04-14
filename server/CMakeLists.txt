project(realsense_node)
cmake_minimum_required(VERSION 3.1)

# explicitly set c++11 
set(CMAKE_CXX_STANDARD 11)

# find thread packages
find_package(Threads)

# configure ZeroMQ
IF(WIN32)
    # set the zeromq directories manually
    message("Windows platform detected, setting ZeroMQ configuration manually...")
    set(ZeroMQ_ROOT_DIR "C:/Program Files/ZeroMQ 4.0.4")
    set(ZeroMQ_INCLUDE_DIR ${ZeroMQ_ROOT_DIR}/include)
    set(ZeroMQ_LIB_DIR ${ZeroMQ_ROOT_DIR}/lib)

    # set the include and link directories
    include_directories(${ZeroMQ_INCLUDE_DIR})
    link_directories(${ZeroMQ_LIB_DIR})
    
    # find all of the libs in ZEROMQ_LIB_DIR
    file(GLOB ZeroMQ_LIBS RELATIVE ${ZeroMQ_LIB_DIR} ${ZeroMQ_LIB_DIR}/*.lib)
    message("Found ZeroMQ libs...")
    message(${ZeroMQ_LIBS})
ELSE(WIN32)
    #find_package (ZeroMQ REQUIRED)
    #set(ZeroMQ_LIBS ${ZeroMQ_LIBRARIES})
 
    # set the zeromq directories
    message("Linux platform detected, setting ZeroMQ configuration manually...")
    find_path(ZeroMQ_ROOT_DIR NAMES include/zmq.h)
    find_library(ZeroMQ_LIBS NAMES zmq libzmq HINTS ${ZeroMQ_ROOT_DIR}/lib)
    find_path(ZeroMQ_INCLUDE_DIR NAMES zmq.h HINTS ${ZeroMQ_ROOT_DIR}/include)
    
    # set the include and link directories
    include_directories(${ZeroMQ_INCLUDE_DIR})
    link_directories(${ZeroMQ_LIB_DIR})
ENDIF(WIN32)

# configure librealsense
IF(WIN32)
    message("Windows platform detected, no librealsense configuration available!")
ELSE(WIN32)
    # set the librealsense directories
    message("Linux platform detected, setting librealsense configuration manually...")
    set(librealsense_LIB_DIR "/usr/local/include")
    set(librealsense_LIBS "librealsense.so")
    set(librealsense_INCLUDE_DIR "~/librealsense/include")
    
    # set the include and link directories
    include_directories(${librealsense_INCLUDE_DIR})
    link_directories(${librealsense_LIB_DIR})
ENDIF(WIN32)

# configure OpenCV
IF(WIN32)
    # set the opencv directories manually
    message("Windows platform detected, setting OpenCV configuration manually...")
    set(OPENCV_BUILD_DIR "C:/OpenCV/build/x64/vc12")
    set(OPENCV_INCLUDE_DIRS "C:/OpenCV/build/include;C:/OpenCV/build/include/opencv;C:/OpenCV/build/include/opencv2")
    set(OPENCV_LIB_DIRS "C:/OpenCV/build/x64/vc12/lib")

    # find all of the subdirectories in OPENCV_LIB_DIRS to include
    include_directories(${OPENCV_BUILD_DIR})
    include_directories(${OPENCV_INCLUDE_DIRS})
    link_directories(${OPENCV_LIB_DIRS})
    
    # find all of the libs in OPENCV_LIB_DIRS
    file(GLOB OpenCV_LIBS RELATIVE ${OPENCV_LIB_DIRS} ${OPENCV_LIB_DIRS}/*2410.lib)
    message("Found OpenCV libs...")
    message(${OpenCV_LIBS})
ELSE(WIN32)
    find_package (OpenCV REQUIRED)
ENDIF(WIN32)

# create individual projects
#add_executable (zmq_cv_request zmq_cv_request.cpp)
#target_link_libraries(zmq_cv_request ${ZeroMQ_LIBS} ${OpenCV_LIBS})

add_executable (realsense_server realsense_server.cpp)
target_link_libraries(realsense_server ${librealsense_LIBS} ${ZeroMQ_LIBS} ${OpenCV_LIBS})
