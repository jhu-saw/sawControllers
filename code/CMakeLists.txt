
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
  
  set( 
    HEADER_FILES 
    ../osaGravityCompensation.h
    ../osaPDGC.h
    
    ../mtsController.h
    ../mtsGravityCompensation.h
    ../mtsPDGC.h )
  
  set( 
    SOURCE_FILES 
    osaGravityCompensation.cpp
    osaPDGC.cpp

    mtsController.cpp
    mtsGravityCompensation.cpp
    mtsPDGC.cpp )
  
  add_library( sawControllers ${HEADER_FILES} ${SOURCE_FILES} )
  cisst_target_link_libraries( sawControllers ${REQUIRED_CISST_LIBRARIES} )

  # create/configure file for find_package (sawNDITracker)
  set(sawControllers_INCLUDE_DIR "${sawComponents_SOURCE_DIR}" "${sawComponents_BINARY_DIR}")
  set(sawControllers_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set(sawControllers_LIBRARIES sawBarrett)

  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/sawControllersConfig.cmake.in"
    "${sawComponents_BINARY_DIR}/sawControllersConfig.cmake"
    @ONLY@ )

endif( cisst_FOUND )
