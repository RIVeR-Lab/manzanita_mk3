cmake_minimum_required(VERSION 2.8.3)
project(manzanita_mk3_driver)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  device_driver_base
  diagnostic_updater
  dynamic_reconfigure
  sensor_msgs
  message_generation
  std_msgs
)
#find_package(catkin REQUIRED dynamic_reconfigure)
#find_package(Boost REQUIRED COMPONENTS foreach)

catkin_python_setup()

# Declare the message files to be built
add_message_files(FILES BatteryCell.msg BatteryPack.msg)

# Actually generate the language-specific message and service files
generate_messages(DEPENDENCIES std_msgs)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(cfg/MK3.cfg)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sensor_types
  CATKIN_DEPENDS 
    roscpp
    rospy
    device_driver_base
    diagnostic_updater
    dynamic_reconfigure
    sensor_msgs
    rqt_robot_dashboard
    message_runtime
    std_msgs
  DEPENDS Boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(mk3_driver src/mk3_driver.cpp)
add_executable(battery_viewer src/battery_viewer.cpp)

add_dependencies(mk3_driver ${PROJECT_NAME}_gencpp)
add_dependencies(battery_viewer ${PROJECT_NAME}_gencpp)

# make sure configure headers are built before any node using them
add_dependencies(mk3_driver ${PROJECT_NAME}_gencfg)

# make sure the dynamic config has dependencies
add_dependencies(${PROJECT_NAME}_gencfg device_driver_base serial_port)


target_link_libraries(mk3_driver ${catkin_LIBRARIES})# device_driver serial_port)
target_link_libraries(battery_viewer ${catkin_LIBRARIES})
#target_link_libraries(mk3_driver serial_port)

install(PROGRAMS scripts/battery
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
