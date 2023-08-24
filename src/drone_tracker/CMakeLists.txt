cmake_minimum_required(VERSION 3.8)
project(drone_tracker)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

INCLUDE_DIRECTORIES ( "/usr/include/eigen3" )
INCLUDE_DIRECTORIES ( "/usr/local/include/opencv4 ")
include_directories( ${OpenCV_INCLUDE_DIRS} )

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

# find_package(OpenCV REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
############## GAZEBO CAMERA DRIVER NODE ######################
add_executable(gazebo_camera_driver 
  src/gazebo_camera_driver.cpp)

target_link_libraries(gazebo_camera_driver ${OpenCV_LIBS})
ament_target_dependencies(gazebo_camera_driver rclcpp std_msgs sensor_msgs cv_bridge px4_msgs geometry_msgs  tf2 tf2_ros)
target_include_directories(gazebo_camera_driver PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_compile_features(gazebo_camera_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

############## ARUCO_POSE NODE ######################
add_executable(aruco_pose 
  src/aruco_pose_est.cpp)

target_link_libraries(aruco_pose ${OpenCV_LIBS})
ament_target_dependencies(aruco_pose rclcpp std_msgs sensor_msgs cv_bridge px4_msgs geometry_msgs  tf2 tf2_ros)
target_include_directories(aruco_pose PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_compile_features(aruco_pose PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

############ OPENMV CAMERA NODE ###################
add_executable(openmv_camera 
  src/open_mv_camera.cpp)

target_link_libraries(openmv_camera ${OpenCV_LIBS})
ament_target_dependencies(openmv_camera rclcpp std_msgs sensor_msgs px4_msgs geometry_msgs tf2 tf2_ros)
target_include_directories(openmv_camera PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_compile_features(openmv_camera PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

############## VEHICLE NODE ######################
add_executable(vehicle 
  src/vehicle.cpp)

ament_target_dependencies(vehicle px4_msgs rclcpp std_msgs tf2 tf2_ros)
target_include_directories(vehicle PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_compile_features(vehicle PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

#############  ESTIMATOR NODE #####################
add_executable(estimator
  src/EASN_estimation.cpp)
ament_target_dependencies(estimator px4_msgs rclcpp std_msgs geometry_msgs)

target_include_directories(estimator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(estimator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS estimator vehicle openmv_camera aruco_pose gazebo_camera_driver
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
