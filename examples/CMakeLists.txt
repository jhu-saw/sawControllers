

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

if( cisst_FOUND AND sawControllers_FOUND )

  # load cisst configuration
  include( ${CISST_USE_FILE} )

  add_executable(
    osaGravityCompensationExample osaGravityCompesnationExample.cpp)

  target_link_libraries( osaGravityCompensationExample sawControllers )

  cisst_target_link_libraries( 
    osaGravityCompensationExample ${REQUIRED_CISST_LIBRARIES} )

endif( cisst_FOUND AND sawControllers_FOUND )
