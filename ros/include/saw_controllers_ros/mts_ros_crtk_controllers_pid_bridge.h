/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2025-02-05

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

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

class CISST_EXPORT mts_ros_crtk_controllers_pid_bridge: public mts_ros_crtk_bridge_provided
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
    mts_ros_crtk_controllers_pid_bridge(const std::string & name,
                                        cisst_ral::node_ptr_t node_handle,
                                        const double period_in_seconds = 5.0 * cmn_ms,
                                        const bool perform_spin = true);

    inline ~mts_ros_crtk_controllers_pid_bridge(void) {}

    void Configure(const std::string & filename = "");
    void Startup(void);

    void bridge_interface_provided(const std::string & component_name,
                                   const std::string & interface_name,
                                   const std::string & ros_namespace,
                                   const double publish_period_in_seconds
                                   = cisst_ros_crtk::bridge_provided_default_publish_period,
                                   const double tf_period_in_seconds
                                   = cisst_ros_crtk::bridge_provided_default_tf_period,
                                   const bool read_write
                                   = true) override;
protected:

};

CMN_DECLARE_SERVICES_INSTANTIATION(mts_ros_crtk_controllers_pid_bridge);

#endif // _mts_ros_crtk_controllers_pid_bridge_h
