/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2022-10-12

  (C) Copyright 2022-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mts_ros_crtk_controllers_pid_bridge_h
#define _mts_ros_crtk_controllers_pid_bridge_h

#include <cisstMultiTask/mtsComponent.h>
#include <cisst_ros_crtk/mts_ros_crtk_bridge_provided.h>

class CISST_EXPORT mts_ros_crtk_controllers_pid_bridge: public mtsComponent
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
    mts_ros_crtk_controllers_pid_bridge(const std::string & name,
                                        cisst_ral::node_ptr_t node_handle,
                                        const double & ros_period,
                                        const double & tf_period);

    inline ~mts_ros_crtk_controllers_pid_bridge(void) {}

    void Configure(const std::string & filename = "");
    void Startup(void);

protected:
    void bridge_all(void);
    bool already_bridged = false;
    mtsInterfaceRequired * m_configuration_interface;
    std::string m_io_component_name;
    mts_ros_crtk_bridge_provided * m_bridge = nullptr;
    double m_ros_period, m_tf_period;

    struct ConfigStruct {
        mtsFunctionRead GetRobotNames;
        mtsFunctionRead GetDigitalInputNames;
        mtsFunctionRead GetName;
    } m_configuration;

};

CMN_DECLARE_SERVICES_INSTANTIATION(mts_ros_crtk_controllers_pid_bridge);

#endif // _mts_ros_crtk_controllers_pid_bridge_h
