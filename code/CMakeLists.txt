#
# (C) Copyright 2011-2015 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstCommonXML
     cisstVector
     cisstOSAbstraction
     cisstMultiTask
     cisstParameterTypes
     cisstRobot
     cisstNumerical )

find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_is_catkin_build (sawControllers_IS_CATKIN_BUILT)
  if (sawControllers_IS_CATKIN_BUILT)
    set (sawControllers_CONFIG_FILE_DIR "${CATKIN_DEVEL_PREFIX}/cmake")
    set (LIBRARY_OUTPUT_PATH "${sawControllers_BINARY_DIR}/lib")
  else ()
    set (sawControllers_CONFIG_FILE_DIR "${sawControllers_BINARY_DIR}")
  endif ()

  # create/configure file for find_package (sawControllers)
  set (sawControllers_INCLUDE_DIR "${sawControllers_SOURCE_DIR}/include"
                                  "${sawControllers_BINARY_DIR}/include")
  set (sawControllers_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawControllers_LIBRARIES sawControllers)

  # Set the version number
  set (sawControllers_VERSION_MAJOR "1")
  set (sawControllers_VERSION_MINOR "1")
  set (sawControllers_VERSION_PATCH "0")
  set (sawControllers_VERSION "${sawControllers_VERSION_MAJOR}.${sawControllers_VERSION_MINOR}.${sawControllers_VERSION_PATCH}")

  # Generate sawControllersRevision.h
  configure_file ("${sawControllers_SOURCE_DIR}/code/sawControllersRevision.h.in"
                  "${sawControllers_BINARY_DIR}/include/sawControllers/sawControllersRevision.h")

  include_directories (${sawControllers_INCLUDE_DIR})
  set (sawControllers_HEADER_DIR "${sawControllers_SOURCE_DIR}/include/sawControllers")

  set(
    HEADER_FILES
    ${sawControllers_HEADER_DIR}/osaGravityCompensation.h
    ${sawControllers_HEADER_DIR}/osaPDGC.h
    ${sawControllers_HEADER_DIR}/osaPIDAntiWindup.h

    ${sawControllers_HEADER_DIR}/mtsController.h
    ${sawControllers_HEADER_DIR}/mtsGravityCompensation.h
    ${sawControllers_HEADER_DIR}/mtsPDGC.h
    ${sawControllers_HEADER_DIR}/mtsPID.h
    ${sawControllers_HEADER_DIR}/mtsTeleOperation.h)

  set(
    SOURCE_FILES
    osaGravityCompensation.cpp
    osaPDGC.cpp
    osaPIDAntiWindup.cpp

    mtsController.cpp
    mtsGravityCompensation.cpp
    mtsPDGC.cpp
    mtsPID.cpp
    mtsTeleOperation.cpp)

  add_library (sawControllers ${HEADER_FILES} ${SOURCE_FILES})
  cisst_target_link_libraries (sawControllers ${REQUIRED_CISST_LIBRARIES})
  set_property (TARGET sawControllers PROPERTY FOLDER "sawControllers")

  # add Qt code
  add_subdirectory (Qt)
  set (sawControllers_LIBRARIES ${sawControllers_LIBRARIES} ${sawControllersQt_LIBRARIES})

  configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/sawControllersConfig.cmake.in"
                  "${sawControllers_CONFIG_FILE_DIR}/sawControllersConfig.cmake"
                  @ONLY@)

  cisst_add_config_version ("${sawControllers_CONFIG_FILE_DIR}/sawControllersConfigVersion.cmake"
                            VERSION ${sawControllers_VERSION}
                            DESTINATION ${CISST_CMAKE_INSTALL_SUFFIX}/saw
                            COMPONENT sawControllers)

  # Install targets for headers and library
  install (DIRECTORY
           ${sawControllers_SOURCE_DIR}/include/sawControllers
           ${sawControllers_BINARY_DIR}/include/sawControllers
           DESTINATION include
           PATTERN .svn EXCLUDE)

  install (TARGETS sawControllers
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

  install (FILES "${sawControllers_BINARY_DIR}/sawControllersConfig.cmake"
           DESTINATION "${CISST_CMAKE_INSTALL_SUFFIX}/saw")

endif( cisst_FOUND )
