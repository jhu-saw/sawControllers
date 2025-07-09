#
# (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.16)
project (saw_controllers_ros VERSION 2.3.0)

# first test for ROS1
set (ROS1_depends cisst_ros_crtk roscpp roslib)
find_package (catkin QUIET
  COMPONENTS ${ROS1_depends})

# ROS 1
if (catkin_FOUND)
  set (ROS_VERSION 1)
  catkin_package (
    INCLUDE_DIRS include
    LIBRARIES saw_controllers_ros
    CATKIN_DEPENDS cisst_ros_crtk roscpp roslib)
  include_directories (${catkin_INCLUDE_DIRS})

else (catkin_FOUND)

  # look for ROS2
  find_package (ament_cmake QUIET)
  if (ament_cmake_FOUND)
    find_package (ament_cmake_python REQUIRED)
    set (ROS_VERSION 2)
    set (ROS2_depends
      rclcpp std_msgs std_srvs geometry_msgs sensor_msgs diagnostic_msgs
      tf2_ros tf2_msgs cisst_msgs crtk_msgs cisst_ros_bridge cisst_ros_crtk)
    foreach (_dep ${ROS2_depends})
      find_package (${_dep} REQUIRED)
    endforeach ()

  endif (ament_cmake_FOUND)
endif (catkin_FOUND)

# find cisst and make sure the required libraries have been compiled
set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstCommonQt
     cisstVector
     cisstVectorQt
     cisstOSAbstraction
     cisstMultiTask
     cisstMultiTaskQt
     cisstParameterTypes
     cisstParameterTypesQt
     cisstQt)

find_package (cisst 1.2.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # sawControllers has been compiled within cisst, we should find it automatically
  find_package (sawControllers 2.2.0)

  if (sawControllers_FOUND)

    # sawControllers configuration
    include_directories (
      ${saw_controllers_ros_SOURCE_DIR}/include
      ${sawControllers_INCLUDE_DIR})
    link_directories (${sawControllers_LIBRARY_DIR})

    add_library (
      saw_controllers_ros
      include/saw_controllers_ros/mts_ros_crtk_controllers_pid_bridge.h
      src/mts_ros_crtk_controllers_pid_bridge.cpp)
    target_link_libraries (
      saw_controllers_ros
      ${sawControllers_LIBRARIES}
      ${catkin_LIBRARIES})
    cisst_target_link_libraries (saw_controllers_ros ${REQUIRED_CISST_LIBRARIES})

    install (
      TARGETS saw_controllers_ros
      EXPORT saw_controllers_ros
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib
      INCLUDES DESTINATION include
      )

    install (
      DIRECTORY include/saw_controllers_ros
      DESTINATION include
      )

    if (ROS_VERSION EQUAL 2)
      ament_target_dependencies (
	saw_controllers_ros
        ${ROS2_depends}
        )

      ament_export_targets (saw_controllers_ros HAS_LIBRARY_TARGET)

      ament_package ()
    endif ()

  else (sawControllers_FOUND)
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires sawControllers")
  endif (sawControllers_FOUND)

endif (cisst_FOUND_AS_REQUIRED)
