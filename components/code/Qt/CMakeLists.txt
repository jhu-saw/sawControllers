#
# (C) Copyright 2013-2021 Johns Hopkins University (JHU), All Rights Reserved.
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
     cisstQt)

# cisst
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # cisst
  include (${CISST_USE_FILE})

  # for Qt generated header files
  include_directories (${CMAKE_CURRENT_BINARY_DIR})
  if (CISST_HAS_QT4)
    qt4_wrap_cpp (SAW_CONTROLLERS_QT_WRAP_CPP
                  ${sawControllers_HEADER_DIR}/mtsPIDQtWidget.h)
  else (CISST_HAS_QT4)
    set (CMAKE_AUTOMOC ON)
  endif (CISST_HAS_QT4)

  add_library (sawControllersQt
               ${sawControllers_HEADER_DIR}/mtsPIDQtWidget.h
               mtsPIDQtWidget.cpp
               ${SAW_CONTROLLERS_QT_WRAP_CPP})
  set_target_properties (sawControllersQt PROPERTIES
                         VERSION ${sawControllers_VERSION}
                         FOLDER "sawControllers")
  cisst_target_link_libraries (sawControllersQt ${REQUIRED_CISST_LIBRARIES})

  install (TARGETS sawControllersQt COMPONENT sawControllers-Qt
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
