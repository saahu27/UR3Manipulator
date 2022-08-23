cmake_minimum_required(VERSION 2.8.3)
project(lab_1)

add_compile_options(-std=c++11)
add_compile_options(-lpthread)

find_package(catkin REQUIRED COMPONENTS	roscpp moveit_wrapper)


# Eigen
find_package(Eigen3 REQUIRED)
if (Eigen3_FOUND)
	message(STATUS "Eigen3 found")
else ()
	message(FATAL_ERROR "Eigen3 not found")
endif ()

include_directories(
		include
		${catkin_INCLUDE_DIRS}
		${EIGEN3_INCLUDE_DIRS}
)

catkin_package(
		INCLUDE_DIRS include
		CATKIN_DEPENDS
		roscpp moveit_wrapper
)
#-------------------------------------------------------------------------#

# Tactile Controller
add_executable(square src/square.cpp)
target_link_libraries(square ${catkin_LIBRARIES})
