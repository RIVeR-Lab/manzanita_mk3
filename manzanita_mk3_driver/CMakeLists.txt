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

catkin_python_setup()

# Declare the message files to be built
add_message_files(FILES BatteryCell.msg BatteryPack.msg)

# Actually generate the language-specific message and service files
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mk3_driver
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
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(mk3_driver src/mk3_driver.cpp)
target_link_libraries(mk3_driver ${catkin_LIBRARIES})

add_executable(mk3_driver_node src/mk3_driver_node.cpp)
add_dependencies(mk3_driver_node ${PROJECT_NAME}_gencpp)
target_link_libraries(mk3_driver_node ${catkin_LIBRARIES} mk3_driver)

add_executable(battery_viewer src/battery_viewer.cpp)
add_dependencies(battery_viewer ${PROJECT_NAME}_gencpp)
target_link_libraries(battery_viewer ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS mk3_driver mk3_driver_node battery_viewer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/manzanita_mk3_driver
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
