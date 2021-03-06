project(realsense_node)
cmake_minimum_required(VERSION 3.1)

# explicitly set c++11 
set(CMAKE_CXX_STANDARD 11)

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

# configure PCL
IF(WIN32)
    message("Windows platform detected, try again with Linux!")
ELSE(WIN32)
    find_package(PCL 1.8 REQUIRED COMPONENTS common visualization)

    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
ENDIF(WIN32)

add_executable (realsense_server realsense_server.cpp)
target_link_libraries(realsense_server ${librealsense_LIBS} ${ZeroMQ_LIBS} ${PCL_LIBRARIES})
