#
# $Id$
#
# (C) Copyright 2011-2012 Johns Hopkins University (JHU), All Rights
# Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

set(
  REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstVector
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes
  cisstRobot
  cisstNumerical )

find_package( cisst REQUIRED ${REQUIRED_CISST_LIBRARIES} )

if( cisst_FOUND )

  # load cisst configuration
  include (${CISST_USE_FILE})

  # create/configure file for find_package (sawNDITracker)
  set(sawControllers_INCLUDE_DIR "${sawControllers_SOURCE_DIR}/include")
  set(sawControllers_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set(sawControllers_LIBRARIES sawControllers)

  include_directories (${sawControllers_INCLUDE_DIR})
  set (sawControllers_HEADER_DIR "${sawControllers_INCLUDE_DIR}/sawControllers")

  set(
    HEADER_FILES
    ${sawControllers_HEADER_DIR}/osaGravityCompensation.h
    ${sawControllers_HEADER_DIR}/osaPDGC.h

    ${sawControllers_HEADER_DIR}/mtsController.h
    ${sawControllers_HEADER_DIR}/mtsGravityCompensation.h
    ${sawControllers_HEADER_DIR}/mtsPDGC.h )

  set(
    SOURCE_FILES
    osaGravityCompensation.cpp
    osaPDGC.cpp

    mtsController.cpp
    mtsGravityCompensation.cpp
    mtsPDGC.cpp )

  add_library( sawControllers ${HEADER_FILES} ${SOURCE_FILES} )
  cisst_target_link_libraries( sawControllers ${REQUIRED_CISST_LIBRARIES} )
  set_property (TARGET sawControllers PROPERTY FOLDER "sawControllers")

  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/sawControllersConfig.cmake.in"
    "${sawControllers_BINARY_DIR}/sawControllersConfig.cmake"
    @ONLY@ )

  # Install targets for headers and library
  install (DIRECTORY
           ${sawControllers_INCLUDE_DIR}/sawControllers
           DESTINATION include
           PATTERN .svn EXCLUDE)

  install( TARGETS sawControllers
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)

  install( FILES "${sawControllers_BINARY_DIR}/sawControllersConfig.cmake"
    DESTINATION "${CISST_CMAKE_INSTALL_SUFFIX}/saw")

endif( cisst_FOUND )
