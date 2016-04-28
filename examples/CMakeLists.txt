#
# (C) Copyright 2011-2016 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 2.8)

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstOSAbstraction
     cisstMultiTask
     cisstParameterTypes
     cisstRobot
     cisstNumerical)

find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  find_package (sawControllers REQUIRED)
  find_package (sawKeyboard REQUIRED)

  if (sawControllers_FOUND AND sawKeyboard_FOUND)

    include_directories (${sawControllers_INCLUDE_DIR}
                         ${sawKeyboard_INCLUDE_DIR})

    link_directories (${sawControllers_LIBRARY_DIR}
                      ${sawKeyboard_LIBRARY_DIR})

    set (sawControllers_EXAMPLES
         osaGCExample
         osaPDGCExample
         mtsGCExample)

    foreach (_example ${sawControllers_EXAMPLES})
      add_executable (${_example} ${_example}.cpp)
      target_link_libraries (${_example} ${sawControllers_LIBRARIES})
      cisst_target_link_libraries (${_example} ${REQUIRED_CISST_LIBRARIES})
      set_property (TARGET ${_example} PROPERTY FOLDER "sawControllers")
    endforeach ()

    # examples that also need sawKeyboard
    target_link_libraries (mtsGCExample ${sawKeyboard_LIBRARIES})

    # copy configuration XML to the executable directory
    # add_custom_target (sawControllers_configPID ALL
    #     COMMAND ${CMAKE_COMMAND} -E copy_if_different
    #             ${CMAKE_CURRENT_SOURCE_DIR}/configPID.xml
    #             ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_CFG_INTDIR}/configPID.xml
    #     SOURCES configPID.xml
    #     COMMENT "Copying configPID.xml to the binary directory")

  endif (sawControllers_FOUND AND sawKeyboard_FOUND)

endif (cisst_FOUND_AS_REQUIRED)
