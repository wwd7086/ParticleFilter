cmake_minimum_required(VERSION 2.8.3)
project(particle_main)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

catkin_package(
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(particle_main_node
	src/Map.cpp
	src/ParticleFilter.cpp
	src/particle_main_node.cpp
)

target_link_libraries(particle_main_node
	${catkin_LIBRARIES} 
)

# add_dependencies(slam_main_node slam_main_generate_messages_cpp)
