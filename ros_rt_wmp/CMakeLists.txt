cmake_minimum_required(VERSION 2.4.6)

project(ros_rt_wmp)

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW)
	cmake_policy(SET CMP0015 NEW)
endif(COMMAND cmake_policy)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp std_msgs ros_rt_wmp_msgs roscpp_serialization roscpp_tutorials)
find_package(Boost REQUIRED COMPONENTS thread)
catkin_package(CATKIN_DEPENDS roscpp ros_rt_wmp_msgs roscpp_serialization roscpp_tutorials)

file(APPEND .config "")
file(APPEND .target "")

SET(type ros-rt-wmp)
SET(CMAKE_C_FLAGS "-g -ggdb")

include(${PROJECT_SOURCE_DIR}/.config)
include(${PROJECT_SOURCE_DIR}/.target)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(rt-wmp/src)

add_library(rt-wmp-raw STATIC
rt-wmp/src/core/routing/basic/frames.c
rt-wmp/src/core/routing/basic/handle_frames.c
rt-wmp/src/core/routing/aura.c
rt-wmp/src/core/routing/manage.c
rt-wmp/src/core/task_timing.c
rt-wmp/src/core/interface.c
rt-wmp/src/core/wmp_utils.c
rt-wmp/src/core/RT-WMP.c
rt-wmp/src/core/queue_rx.c
rt-wmp/src/core/queue_tx.c
rt-wmp/src/core/dijkstra.c
rt-wmp/src/core/lqm.c
rt-wmp/src/core/wmp_com.c
rt-wmp/src/core/queues.c
rt-wmp/src/core/ml_com.c
rt-wmp/src/core/flow_control.c
rt-wmp/src/core/dijkstra_alg.c
rt-wmp/src/core/MobileAverage.c
rt-wmp/src/core/frame_compress.c
rt-wmp/src/core/wmp_misc.c
rt-wmp/src/core/nstat.c
rt-wmp/src/core/rssi_average.c
rt-wmp/src/core/queue_core.c
rt-wmp/src/core/prim.c
rt-wmp/src/platforms/linux_us/wmp_misc.c
rt-wmp/src/platforms/linux_us/wmpInit.c
rt-wmp/src/platforms/linux_us/hwi/raw/ll_com_2017.c
rt-wmp/src/platforms/linux_us/hwi/raw/raw_comm.c
rt-wmp/src/platforms/linux_us/hwi/raw/radiotap/radiotap.c

)

target_include_directories(rt-wmp-raw PUBLIC rt-wmp/src/platforms/linux_us)

add_executable(ros-rt-wmp  src/main.cpp
                           src/argon.h
                           src/ServiceManager.h
                           src/TopicManager.h
                           src/WMPServiceManager.h
                           src/config.h
                           src/macros.h
                           src/Manager.h
                           src/TFManager.h
                           src/WMPNodeManager.h
                           src/WMPTopicManager.h)

add_executable(ros-rt-wmp-ip  		src/main.cpp ip/low_level.c)
add_executable(ros-rt-wmp-mh  		src/main.cpp ip/low_level-multi-hop.c)
add_executable(fake_lqm  		src/fake_lqm.cpp)


target_link_libraries(ros-rt-wmp          rt-wmp-raw z pcap rt ${Boost_LIBRARIES}  ${catkin_LIBRARIES})
target_link_libraries(ros-rt-wmp-ip       rt z ${Boost_LIBRARIES}  ${catkin_LIBRARIES})
target_link_libraries(ros-rt-wmp-mh       rt z ${Boost_LIBRARIES}  ${catkin_LIBRARIES})
target_link_libraries(fake_lqm            rt z yaml-cpp ${Boost_LIBRARIES}  ${catkin_LIBRARIES})


add_custom_command(TARGET ros-rt-wmp 
	POST_BUILD
	COMMAND echo "Applying sticky bit to the executable..."
        COMMAND sudo chown root:root ${CMAKE_BINARY_DIR}/../devel/lib/ros_rt_wmp/ros-rt-wmp
        COMMAND sudo chmod +s ${CMAKE_BINARY_DIR}/../devel/lib/ros_rt_wmp/ros-rt-wmp

	#COMMAND sudo chown root:root ${CMAKE_BINARY_DIR}/../devel/lib/ros_rt_wmp/ros-rt-wmp
	#COMMAND sudo chmod +s ${CMAKE_BINARY_DIR}/../devel/lib/ros_rt_wmp/ros-rt-wmp
)

add_custom_command(TARGET ros-rt-wmp-ip 
	POST_BUILD
	COMMAND echo "Applying sticky bit to the executable..."
        COMMAND sudo chown root:root ${CMAKE_BINARY_DIR}/../devel/lib/ros_rt_wmp/ros-rt-wmp-ip
        COMMAND sudo chmod +s ${CMAKE_BINARY_DIR}/../devel/lib/ros_rt_wmp/ros-rt-wmp-ip

        #COMMAND mv /tmp/ros-rt-wmp-ip ${CMAKE_BINARY_DIR}/../devel/lib/ros_rt_wmp/ros-rt-wmp-ip 
	#COMMAND sudo chown root:root ${CMAKE_BINARY_DIR}/../devel/lib/ros_rt_wmp/ros-rt-wmp-ip
	#COMMAND sudo chmod +s ${CMAKE_BINARY_DIR}/../devel/lib/ros_rt_wmp/ros-rt-wmp-ip
)

