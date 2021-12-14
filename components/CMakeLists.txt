#
# (C) Copyright 2011-2021 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sawControllers VERSION 2.0.0)

set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_EXTENSIONS OFF)

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstCommonXML
     cisstVector
     cisstOSAbstraction
     cisstMultiTask
     cisstParameterTypes
     cisstRobot
     cisstNumerical)

find_package (cisst 1.1.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_set_output_path ()

  # create/configure file for find_package (sawControllers)
  set (sawControllers_INCLUDE_DIR "${sawControllers_SOURCE_DIR}/include"
                                  "${sawControllers_BINARY_DIR}/include")
  set (sawControllers_HEADER_DIR "${sawControllers_SOURCE_DIR}/include/sawControllers")
  set (sawControllers_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawControllers_LIBRARIES sawControllers)

  include_directories (${sawControllers_INCLUDE_DIR})

  if (CISST_HAS_QT)
    add_subdirectory (code/Qt)
    set (sawControllers_LIBRARIES ${sawControllers_LIBRARIES} sawControllersQt)
  endif ()

  cisst_add_config_files (sawControllers)

  set (HEADER_FILES
       ${sawControllers_HEADER_DIR}/osaGravityCompensation.h
       ${sawControllers_HEADER_DIR}/osaPDGC.h
       ${sawControllers_HEADER_DIR}/osaPIDAntiWindup.h
       ${sawControllers_HEADER_DIR}/osaCartesianImpedanceController.h

       ${sawControllers_HEADER_DIR}/mtsController.h
       ${sawControllers_HEADER_DIR}/mtsGravityCompensation.h
       ${sawControllers_HEADER_DIR}/mtsPDGC.h
       ${sawControllers_HEADER_DIR}/mtsPID.h)

  set (SOURCE_FILES
       code/osaGravityCompensation.cpp
       code/osaPDGC.cpp
       code/osaPIDAntiWindup.cpp
       code/osaCartesianImpedanceController.cpp

       code/mtsController.cpp
       code/mtsGravityCompensation.cpp
       code/mtsPDGC.cpp
       code/mtsPID.cpp)

  add_library (sawControllers ${HEADER_FILES} ${SOURCE_FILES})

  set_target_properties (sawControllers PROPERTIES
                         VERSION ${sawControllers_VERSION}
                         FOLDER "sawControllers")

  cisst_target_link_libraries (sawControllers ${REQUIRED_CISST_LIBRARIES})

  # Install targets for headers and library
  install (DIRECTORY
           ${sawControllers_SOURCE_DIR}/include/sawControllers
           ${sawControllers_BINARY_DIR}/include/sawControllers
           DESTINATION include
           COMPONENT sawControllers-dev
           PATTERN .svn EXCLUDE)

  install (TARGETS sawControllers COMPONENT sawControllers
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

endif( cisst_FOUND )
