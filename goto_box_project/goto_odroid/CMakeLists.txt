cmake_minimum_required(VERSION 2.8.3)
project(goto_odroid)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  genmsg
  message_generation
  move_base_msgs
  nav_msgs
  roscpp
  rospy
  rosserial_arduino
  rosserial_client
  sensor_msgs
  std_msgs
  tf
)

 generate_messages(
   DEPENDENCIES
   actionlib_msgs
   move_base_msgs
 )

catkin_package(
   DEPENDS actionlib_msgs)

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY firmware_extensional_module
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}

)

rosserial_configure_client(
  DIRECTORY firmware_wheelchair
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(firmware_extensional_module boxinfo ALL)
rosserial_add_client_target(firmware_extensional_module boxinfo-upload)
rosserial_add_client_target(firmware_wheelchair mega_wheelchair ALL)
rosserial_add_client_target(firmware_wheelchair mega_wheelchair-upload)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pos_pros_node_ src/pos_pros_node_.cpp)
target_link_libraries(pos_pros_node_ 
   ${catkin_LIBRARIES})
add_dependencies(pos_pros_node_ ${${goto_odroid}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(hector_odom_ src/hector_odom_.cpp)
target_link_libraries(hector_odom_ 
   ${catkin_LIBRARIES})
add_dependencies(hector_odom_ ${${goto_odroid}_EXPORTED_TARGETS}${catkin_EXPORTED_TARGETS})

add_executable(tf_to_odom_ src/tf_to_odom_.cpp)
target_link_libraries(tf_to_odom_ 
   ${catkin_LIBRARIES})
add_dependencies(tf_to_odom_ ${${goto_odroid}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(odom_to_tf_ src/odom_to_tf_.cpp)
target_link_libraries(odom_to_tf_
   ${catkin_LIBRARIES})
add_dependencies(odom_to_tf_ ${${goto_odroid}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(loc_ src/loc_.cpp)
target_link_libraries(loc_
   ${catkin_LIBRARIES})
add_dependencies(loc_ ${${goto_odroid}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(amcl_pose_to_odom_ src/amcl_pose_to_odom_.cpp)
target_link_libraries(amcl_pose_to_odom_
   ${catkin_LIBRARIES})
add_dependencies(amcl_pose_to_odom_ ${${goto_odroid}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(convert_scan src/convert_scan.cpp)
target_link_libraries(convert_scan
   ${catkin_LIBRARIES})
add_dependencies(convert_scan ${${goto_odroid}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 install(PROGRAMS
   src/reset_wheelchair_.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
