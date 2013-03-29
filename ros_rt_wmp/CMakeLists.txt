cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

file(APPEND .config "")
file(APPEND .target "")

SET(type ros-rt-wmp)

include(${PROJECT_SOURCE_DIR}/.config)
include(${PROJECT_SOURCE_DIR}/.target)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

link_directories(rt-wmp/lib-linux_us)
link_directories(rt-wmp/lib-linux_ks)

include_directories(rt-wmp/src)

rosbuild_add_executable(ros-rt-wmp EXCLUDE_FROM_ALL src/main.cpp)
rosbuild_add_executable(ros-rt-wmp-us-sock  EXCLUDE_FROM_ALL src/main.cpp)
rosbuild_add_executable(ros-rt-wmp-us-ath5k  EXCLUDE_FROM_ALL src/main.cpp)
rosbuild_add_executable(ros-rt-wmp-ks  EXCLUDE_FROM_ALL src/main.cpp)
rosbuild_add_executable(test  EXCLUDE_FROM_ALL src/main.cpp)


target_link_libraries(ros-rt-wmp          rt-wmp z pcap)
target_link_libraries(ros-rt-wmp-us-sock  rt-wmp z pcap)
target_link_libraries(ros-rt-wmp-us-ath5k rt-wmp z pcap)
target_link_libraries(ros-rt-wmp-ks       rt-wmp-ks z pcap)
target_link_libraries(test                rt-wmp z pcap)

#ADD_DEFINITIONS(-Wno-address -Wno-deprecated -Wno-deprecated-declarations -std=gnu++0x -I../../rt-wmp/platforms/linux_us)

if (NOT CONF MATCHES raw)
add_dependencies(ros-rt-wmp librtwmp)
endif( NOT CONF  MATCHES raw)

if (NOT CONF MATCHES sock)
add_dependencies(ros-rt-wmp-us-sock librtwmp_sock)
endif( NOT CONF  MATCHES sock)


if (NOT CONF MATCHES ath5k)
add_dependencies(ros-rt-wmp-us-ath5k librtwmp_ath5k)
endif( NOT CONF  MATCHES ath5k)


if (NOT CONF MATCHES ks)
add_dependencies(ros-rt-wmp-ks librtwmp_ks)
endif( NOT CONF  MATCHES ks)


add_custom_command(TARGET test
        POST_BUILD
        COMMAND sudo chown root:root ${CMAKE_BINARY_DIR}/../bin/test
        COMMAND sudo chmod +s ${CMAKE_BINARY_DIR}/../bin/test
)

add_custom_command(TARGET ros-rt-wmp 
	POST_BUILD
	COMMAND echo "Applying sticky bit to the executable..."
	COMMAND sudo chown root:root ${CMAKE_BINARY_DIR}/../bin/ros-rt-wmp
	COMMAND sudo chmod +s ${CMAKE_BINARY_DIR}/../bin/ros-rt-wmp
)

add_custom_command(TARGET ros-rt-wmp-us-ath5k
        POST_BUILD
	COMMAND echo "Applying sticky bit to the executable..."
        COMMAND sudo chown root:root ${CMAKE_BINARY_DIR}/../bin/ros-rt-wmp-us-ath5k
        COMMAND sudo chmod +s ${CMAKE_BINARY_DIR}/../bin/ros-rt-wmp-us-ath5k
)

add_custom_command(TARGET ros-rt-wmp-ks
        POST_BUILD
        COMMAND echo "Applying sticky bit to the executable..."
	COMMAND sudo chown root:root ${CMAKE_BINARY_DIR}/../bin/ros-rt-wmp-ks
        COMMAND sudo chmod +s ${CMAKE_BINARY_DIR}/../bin/ros-rt-wmp-ks
)


add_custom_target(
	librtwmp
	COMMAND ./configure --with-llcom=raw
	COMMAND make clean all
	COMMAND echo "\"set("CONF raw")"\" > ${PROJECT_SOURCE_DIR}/.config
	WORKING_DIRECTORY ../rt-wmp
)

add_custom_target(
        librtwmp_sock
        COMMAND ./configure --with-llcom=sock
        COMMAND make clean all
        COMMAND echo "\"set("CONF sock")"\" > ${PROJECT_SOURCE_DIR}/.config
        WORKING_DIRECTORY ../rt-wmp
)

add_custom_target(
        librtwmp_ath5k
        COMMAND ./configure --with-llcom=ath5k_raw
        COMMAND make clean all
        COMMAND echo "\"set("CONF ath5k")"\" > ${PROJECT_SOURCE_DIR}/.config
        WORKING_DIRECTORY ../rt-wmp
)

add_custom_target(
        librtwmp_ks
        COMMAND ./configure --with-platform=linux_ks
        COMMAND make clean all
        COMMAND echo "\"set("CONF ks")"\" > ${PROJECT_SOURCE_DIR}/.config
        WORKING_DIRECTORY ../rt-wmp
)


add_custom_target(compile ALL)
                                   
## Select the desired configuration v here v



add_dependencies(compile ${type})
#add_dependencies(compile ros-rt-wmp)
#add_dependencies(compile ros-rt-wmp-us-sock)
#add_dependencies(compile ros-rt-wmp-us-ath5k)
#add_dependencies(compile ros-rt-wmp-ks)
