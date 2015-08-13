cmake_minimum_required(VERSION 2.8)
project(velma_planners)

find_package(catkin REQUIRED COMPONENTS
  rtt_ros rtt_rosclock std_msgs tf eigen_conversions cmake_modules collision_convex_model kin_dyn_model
  )

find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(OMPL REQUIRED)

link_directories(${orocos_kdl_LIBRARY_DIRS} ${OMPL_LIBRARY_DIRS})

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${USE_OROCOS_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})

add_executable(test_velma_ompl src/test_ompl.cpp src/marker_publisher.cpp src/random_uniform.cpp)

target_link_libraries(test_velma_ompl ${catkin_LIBRARIES} ${OMPL_LIBRARIES})

catkin_package(
    DEPENDS ompl eigen
)

