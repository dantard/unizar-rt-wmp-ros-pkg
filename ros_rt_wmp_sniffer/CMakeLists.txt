cmake_minimum_required(VERSION 2.4.6)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#rosbuild_init()

add_definitions(-Wno-sign-compare)
set ( CMAKE_CXX_FLAGS -Wno-sign-compare)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_COMPILER g++-4.4)

find_package(PkgConfig)

pkg_check_modules(GTKMM gtkmm-2.4)

link_directories(
    ${GTKMM_LIBRARY_DIRS}  )

include_directories(
    ${GTKMM_INCLUDE_DIRS}  ../ros_rt_wmp/rt-wmp/src src)

add_executable(ros-rt-wmp-sniffer src/wmpSniffer.cc
        src/main_window_glade.cc 
        src/main_window.cc 
        src/area.cc 
        src/bridge.cc 
        src/MyArea.cc 
        src/rec_param_dlg.cc 
        src/rec_param_dlg_glade.cc 
        src/wmp_specific.cc 
        src/marte_layer.cc
        src/window1.cc
        src/window1_glade.cc
        src/window2.cc
        src/window2_glade.cc
        src/pcap_layer.cc
        src/statistics.cc
        src/prim.cc 
        src/buffer_layer.cc 
        src/basic_io.cc 
        src/gct.cc 
        src/enhanced_io.cc 
        src/time.cc   
        src/stats.cc  
        src/Plotter.cc
        src/shmem_layer.cc
        src/window3.cc
        src/window3_glade.cc)



add_custom_command(TARGET ros-rt-wmp-sniffer
        POST_BUILD
        COMMAND sudo chown root:root
	${CMAKE_BINARY_DIR}/../bin/ros-rt-wmp-sniffer
        COMMAND sudo chmod +s ${CMAKE_BINARY_DIR}/../bin/ros-rt-wmp-sniffer
)

target_link_libraries(ros-rt-wmp-sniffer
    ${GTKMM_LIBRARIES}  vte pcap gthread-2.0 rt pthread z)