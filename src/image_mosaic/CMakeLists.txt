cmake_minimum_required(VERSION 2.8.3)
project(image_mosaic)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge image_transport sensor_msgs roscpp rospy std_msgs message_filters)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_image_transport
#  CATKIN_DEPENDS cv_bridge image_transport sensor_msgs
#  DEPENDS system_lib
)

include_directories(  ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} )


add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


