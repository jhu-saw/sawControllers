/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2025-02-2025

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


// system include
#include <iostream>

// project include
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsManagerLocal.h>

#include <saw_controllers_ros/mts_ros_crtk_controllers_pid_bridge.h>

CMN_IMPLEMENT_SERVICES_DERIVED(mts_ros_crtk_controllers_pid_bridge, mtsComponent);

mts_ros_crtk_controllers_pid_bridge::mts_ros_crtk_controllers_pid_bridge(const std::string & name,
                                                                         cisst_ral::node_ptr_t node_handle,
                                                                         const double period_in_seconds):
    mts_ros_crtk_bridge_provided(name, node_handle, period_in_seconds)
{
}

void mts_ros_crtk_controllers_pid_bridge::Configure(const std::string &) {
}

void mts_ros_crtk_controllers_pid_bridge::Startup(void) {
}

void mts_ros_crtk_controllers_pid_bridge::bridge_interface_provided(const std::string & component_name,
                                                                    const std::string & interface_name,
                                                                    const std::string & ros_namespace,
                                                                    const double publish_period_in_seconds,
                                                                    const double tf_period_in_seconds,
                                                                    const bool read_write)
{
    // call base class method
    mts_ros_crtk_bridge_provided::bridge_interface_provided(component_name, interface_name, ros_namespace,
                                                            publish_period_in_seconds, tf_period_in_seconds, read_write);
    // add non CRTK topics
}
