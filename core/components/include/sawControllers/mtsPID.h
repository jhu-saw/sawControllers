/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet, Ugur Tumerdem
  Created on: 2013-02-22

  (C) Copyright 2013-2023 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


/*!
  \file
  \brief Basic PID controller
  \ingroup sawControllers

  \todo Add dError filtering
*/


#ifndef _mtsPID_h
#define _mtsPID_h

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstParameterTypes/prmForceTorqueJointSet.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmConfigurationJoint.h>
#include <cisstParameterTypes/prmServoJoint.h>

#include <sawControllers/sawControllersRevision.h>
#include <sawControllers/mtsPIDConfiguration.h>

//! Always include last
#include <sawControllers/sawControllersExport.h>

class CISST_EXPORT mtsPID: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

protected:
    // Required interface
    struct {
        //! Read joint type from robot
        mtsFunctionRead configuration_js;
        mtsFunctionWrite configure_js;

        //! Read joint state from robot
        mtsFunctionRead measured_js;

        //! Write the joint efforts
        mtsFunctionWrite servo_jf;
    } IO;


    size_t m_number_of_joints; // set from the XML file used in Configure method

    mtsPIDConfiguration m_configuration; // PID configuration
    prmConfigurationJoint m_configuration_js; // joint configuration (limits, etc.)

    bool m_enforce_position_limits = false; // whether to check/enforce joint limits
    vctBoolVec mPositionLimitFlagPrevious, mPositionLimitFlag; // whether joint limit was hit

    prmForceTorqueJointSet m_pid_setpoint_jf; // Commanded joint efforts sent to IO level

    //! prm type joint state
    prmStateJoint
        m_measured_js,
        m_measured_js_previous,
        m_setpoint_js;

    prmServoJoint m_joint_command;

    //! Error
    prmStateJoint m_error_state; // position, velocity and effort for disturbance
    vctDoubleVec m_i_error;
    vctDoubleVec m_disturbance_state;

    bool m_use_setpoint_v = true;  // option to ignore user setpoint_v

    //! If cutoff set to 1.0, unfiltered
    vctDoubleVec
        m_setpoint_filtered_v,
        m_setpoint_filtered_v_previous;

    //! Enable mtsPID controller
    bool m_enabled = false;

    //! Enable individal joints
    vctBoolVec m_joints_enabled;

    bool mTrackingErrorEnabled;
    vctDoubleVec mTrackingErrorTolerances;
    vctBoolVec mPreviousTrackingErrorFlag, mTrackingErrorFlag;

    // Flag to determine if this is connected to actual IO/hardware or
    // simulated
    bool m_simulated = false;
    double mCommandTime, mPreviousCommandTime;

    //! Configuration state table
    mtsStateTable mConfigurationStateTable;

    struct {
        //! Enable event
        mtsFunctionWrite enabled;
        //! Position limit event
        mtsFunctionWrite position_limit;
        //! Enabled joints event
        mtsFunctionWrite enabled_joints;
        //! Use setpoint_v
        mtsFunctionWrite use_setpoint_v;
    } Events;

    mtsInterfaceProvided * mInterface;

    /**
     * @brief Reset encoder, clear e/ed/ei value
     *
     */
    void ResetController(void);

    /*! Sets desired setpoint & control mode per joint */
    void servo_js(const prmServoJoint & command);
    /* Convience wrappers around servo_js */
    void servo_jp(const prmPositionJointSet & command);
    void servo_jf(const prmForceTorqueJointSet & command);

    void Init(void);

    void SetupInterfaces(void);

    void enable(const bool & enable);

    void enable_joints(const vctBoolVec & enable);

    void use_setpoint_v(const bool & use);

    void SetTrackingErrorTolerances(const vctDoubleVec & tolerances);

    /*! Retrieve data from the IO component.  This method checks for
      the simulated flag and sets position/effort based on user
      commands if it is simulated.  If the IO component doesn't
      provide the velocity, the method estimates the velocity based on
      the previous measured position. */
    void get_IO_data(void);

    /*! Utility method to convert vector of doubles
      (e.g. mStateJointCommand.Effort()) to cisstParameterType
      prmForceTorqueJointSet and then call the function to sent
      requested efforts to the IO component. */
    void SetEffortLocal(const vctDoubleVec & effort);

    void ErrorEventHandler(const mtsMessage & message);

public:

    /**
     * @brief Main constructor
     *
     * @param taskname   The name of the MTS periodic task
     * @param period     The period of the task
     */
    mtsPID(const std::string & taskname,
           const double period);
    mtsPID(const mtsTaskPeriodicConstructorArg & arg);
    ~mtsPID(){}

    /**
     * @brief Configure PID gains & params
     *
     * @param filename  The name of the configuration file
     */
    void Configure(const std::string& filename);
    void Startup(void);
    void Run(void);
    void Cleanup(void);

    void SetSimulated(void);

protected:

    void enforce_position_limits(const bool & enforce);

    /**
     * @brief Set configuration
     *
     * @param configuration  new configuration
     */
    void configure(const mtsPIDConfiguration & configuration);

    /*! Return true is size mismatch.  Also sends error message and
      disables PID. */
    bool SizeMismatch(const size_t size, const std::string & methodName);

    /*! Sends a warning message if any lower limit is greater or equal
      to upper limit. */
    void CheckLowerUpper(const vctDoubleVec & lower, const vctDoubleVec & upper,
                         const std::string & methodName);

    double clamp(double value, double min, double max) {
        if (value < min) {
          return min;
        } else if (value > max) {
          return max;
        } else {
          return value;
        }
    }
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsPID);

#endif  // _mtsPID_h
