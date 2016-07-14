cmake_minimum_required(VERSION 2.8.3)
project(goto_box)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  genmsg
  message_generation
  message_runtime
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

 add_service_files(
   FILES Hash.srv
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
   move_base_msgs
 )

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(wc_encoders_lib src/SerialCom.cpp src/WheelEncoder.cpp)

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

add_executable(pos_pros_node src/pos_pros_node.cpp)
target_link_libraries(pos_pros_node 
   ${catkin_LIBRARIES})
add_dependencies(pos_pros_node ${${goto_box}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(nav_talker_rviz src/nav_talker_rviz.cpp)
target_link_libraries(nav_talker_rviz 
   ${catkin_LIBRARIES})
add_dependencies(nav_talker_rviz ${${goto_box}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(joy_to_joy src/joy_to_joy.cpp)
target_link_libraries(joy_to_joy 
   ${catkin_LIBRARIES})
add_dependencies(joy_to_joy ${${goto_box}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(tf_to_odom src/tf_to_odom.cpp)
target_link_libraries(tf_to_odom 
   ${catkin_LIBRARIES})
add_dependencies(tf_to_odom ${${goto_box}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(odom_to_tf src/odom_to_tf.cpp)
target_link_libraries(odom_to_tf
   ${catkin_LIBRARIES})
add_dependencies(odom_to_tf ${${goto_box}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(loc src/loc.cpp)
target_link_libraries(loc
   ${catkin_LIBRARIES})
add_dependencies(loc ${${goto_box}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(wc_encoders_node src/wc_encoders_node.cpp)
add_dependencies(wc_encoders_node ${${goto_box}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wc_encoders_node ${catkin_LIBRARIES} wc_encoders_lib)

add_executable(jt_profile_node src/build_joy_twist_profile.cpp)
add_dependencies(jt_profile_node ${${goto_box}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(jt_profile_node ${catkin_LIBRARIES})

add_executable(amcl_pose_to_odom src/amcl_pose_to_odom.cpp)
add_dependencies(amcl_pose_to_odom ${${goto_box}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(amcl_pose_to_odom ${catkin_LIBRARIES})

 install(PROGRAMS 
   src/jt_inverse_node.py 
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

 install(PROGRAMS
   src/basic_hash_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

 install(PROGRAMS
   src/reset.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

 install(PROGRAMS
   src/reset_wheelchair.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
