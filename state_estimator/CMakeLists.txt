cmake_minimum_required(VERSION 3.5)
project(state_estimator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
# set(BUILD_FLAGS "-std=c++11 -Wuninitialized -Wall -Wextra")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package (Boost REQUIRED COMPONENTS system filesystem thread chrono date_time atomic serialization regex timer)
find_package(TBB)


include_directories(
  # include  
  SYSTEM 
  ${GTSAM_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}  
  include/state_estimator  
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Boost_LIBRARY_DIRS} 
  "/usr/local/include"
  "/usr/local/lib"  
)

include_directories(
  SYSTEM 
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(est_node src/StateEstimator.cpp)
target_link_libraries(est_node gtsam ${TBB_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} -ltbb)


ament_target_dependencies(est_node 
tf2
tf2_geometry_msgs
rclcpp 
std_msgs 
# visualization_msgs
geometry_msgs
sensor_msgs
nav_msgs)


install(TARGETS
est_node
  DESTINATION lib/${PROJECT_NAME}  
  )

install(DIRECTORY include/
  DESTINATION include/
)

set(NODE_NAME "est_node")
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


ament_export_dependencies(
  Eigen3
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs  
  tf2_geometry_msgs
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
