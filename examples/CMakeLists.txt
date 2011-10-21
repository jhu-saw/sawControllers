

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
find_package( sawControllers REQUIRED HINTS "${CISST_BINARY_DIR}/saw/components")
find_package( sawKeyboard REQUIRED HINTS "${CISST_BINARY_DIR}/saw/components")

if( cisst_FOUND AND sawControllers_FOUND AND sawKeyboard_FOUND)

  # load cisst configuration
  include( ${CISST_USE_FILE} )

  add_executable(osaGCExample    osaGCExample.cpp)
  add_executable(osaPDGCExample  osaPDGCExample.cpp)

  add_executable(mtsGCExample    mtsGCExample.cpp)

  target_link_libraries( osaGCExample   sawControllers )
  target_link_libraries( osaPDGCExample sawControllers )

  target_link_libraries( mtsGCExample   sawControllers sawKeyboard )

  cisst_target_link_libraries( osaGCExample   ${REQUIRED_CISST_LIBRARIES} )
  cisst_target_link_libraries( osaPDGCExample ${REQUIRED_CISST_LIBRARIES} )

  cisst_target_link_libraries( mtsGCExample   ${REQUIRED_CISST_LIBRARIES} )

endif( cisst_FOUND AND sawControllers_FOUND AND sawKeyboard_FOUND)
