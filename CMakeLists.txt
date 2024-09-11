cmake_minimum_required(VERSION 3.8)
project(ocam)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS 2에서 필요한 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

find_package(OpenCV 4.5.2 REQUIRED)


# include 디렉토리 설정
include_directories(
    include
    ${rclcpp_INCLUDE_DIRS}
    /usr/local/include/opencv4
    #${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

# 실행 파일 추가
add_executable(ocam src/oCam.cpp src/withrobot_camera.cpp src/withrobot_utility.cpp)

# 필요한 종속성 추가
ament_target_dependencies(ocam rclcpp std_msgs cv_bridge image_transport camera_info_manager sensor_msgs Boost)

# 라이브러리 링크
target_link_libraries(ocam ${OpenCV_LIBS} Boost::system Boost::thread -lv4l2 -ludev /usr/local/lib/libopencv_core.so /usr/local/lib/libopencv_imgcodecs.so /usr/local/lib/libopencv_imgproc.so)


################################################################################
# Install
################################################################################

install(TARGETS
  ocam
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES config/camera.yaml
  DESTINATION share/${PROJECT_NAME}/config
)


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# 패키지 설정
ament_package()
