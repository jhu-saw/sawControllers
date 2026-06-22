#
# (C) Copyright 2013-2026 Johns Hopkins University (JHU), All Rights Reserved.
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
     cisstVectorQt
     cisstNumerical
     cisstOSAbstraction
     cisstMultiTask
     cisstMultiTaskQt
     cisstParameterTypes
)

# cisst
find_package (cisst REQUIRED COMPONENTS ${REQUIRED_CISST_LIBRARIES})

# for Qt generated header files
include_directories (${CMAKE_CURRENT_BINARY_DIR})
set (CMAKE_AUTOMOC ON)

add_library (sawControllersQt
             ${sawControllers_HEADER_DIR}/mtsPIDQtWidget.h
             mtsPIDQtWidget.cpp)
set_target_properties (sawControllersQt PROPERTIES
                       VERSION ${sawControllers_VERSION}
                       FOLDER "sawControllers")
target_link_libraries (sawControllersQt PUBLIC
  ${cisst_LIBRARIES}
  ${sawControllers_LIBRARIES}
)
 
install (TARGETS sawControllersQt COMPONENT sawControllers-Qt
         RUNTIME DESTINATION bin
         LIBRARY DESTINATION lib
         ARCHIVE DESTINATION lib)
