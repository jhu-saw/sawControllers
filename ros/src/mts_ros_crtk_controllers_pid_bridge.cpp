/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2022-10-12

  (C) Copyright 2022-2025 Johns Hopkins University (JHU), All Rights Reserved.

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
                                                                         const double & ros_period,
                                                                         const double & tf_period):
    mtsComponent(name),
    m_ros_period(ros_period),
    m_tf_period(tf_period)
{
    m_bridge = new mts_ros_crtk_bridge_provided(name + "_actual_bridge", node_handle);

    // This function will make the required interface to be connected with
    // the provided interface of mtsRobotIO1394 named Configure with predefined function names.
    m_configuration_interface = AddInterfaceRequired("RobotConfiguration");
    if (m_configuration_interface) {
        m_configuration_interface->AddFunction("GetRobotNames", m_configuration.GetRobotNames);
        m_configuration_interface->AddFunction("GetDigitalInputNames", m_configuration.GetDigitalInputNames);
        m_configuration_interface->AddFunction("GetName", m_configuration.GetName);
    }
}

void mts_ros_crtk_controllers_pid_bridge::Configure(const std::string &) {
    this->bridge_all();
}

void mts_ros_crtk_controllers_pid_bridge::Startup(void) {
    this->bridge_all();
}

void mts_ros_crtk_controllers_pid_bridge::bridge_all(void)
{
    if (already_bridged) {
        return;
    }
    already_bridged = true;

    if (!m_configuration_interface->GetConnectedInterface()) {
        CMN_LOG_CLASS_INIT_ERROR << "Startup: unable to connect to configuration interface"
                                 << std::endl;
        return;
    }

    // robots, one component per robot with 2 interfaces to be connected
    m_configuration.GetName(m_io_component_name);

    std::vector<std::string> robot_names;
    m_configuration.GetRobotNames(robot_names);
    for (const auto & robot : robot_names) {
        m_bridge->bridge_interface_provided(m_io_component_name, robot,
                                            m_ros_period, m_tf_period);
    }

    std::vector<std::string> input_names;
    m_configuration.GetDigitalInputNames(input_names);
    for (const auto & input : input_names) {
        m_bridge->bridge_interface_provided(m_io_component_name, input,
                                            m_ros_period, m_tf_period);
    }

    m_bridge->Connect();
}
