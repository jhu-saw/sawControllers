/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
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


    //! Number of joints, set from the XML file used in Configure method
    size_t m_number_of_joints;

    //! PID Configuration
    mtsPIDConfiguration m_configuration;

    //! Joint configuration
    prmConfigurationJoint m_configuration_js;

    //! Flag whether check joint limit
    bool m_enforce_position_limits = false;
    vctBoolVec mPositionLimitFlagPrevious, mPositionLimitFlag;

    //! Commanded joint efforts sent to IO level
    vctDoubleVec mEffortMeasure;
    prmForceTorqueJointSet m_pid_setpoint_jf;
    //! Feedforward, i.e. effort added to the PID output in position mode
    prmForceTorqueJointSet m_feed_forward_jf;

    //! Desired joint efforts when bypassing PID
    prmForceTorqueJointSet mEffortUserCommand;

    //! prm type joint state
    prmStateJoint
        m_measured_js,
        m_measured_js_previous,
        m_setpoint_js;

    //! Error
    vctDoubleVec m_p_error, m_i_error;

    //! If cutoff set to 1.0, unfiltered
    bool m_has_setpoint_v = false;
    vctDoubleVec
        m_setpoint_filtered_v,
        m_setpoint_filtered_v_previous;

    //! Enable mtsPID controller
    bool mEnabled = false;

    //! Enable individal joints
    vctBoolVec m_joints_enabled;

    //! Enable mtsPID controller
    vctBoolVec m_effort_mode;

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
        mtsFunctionWrite Enabled;
        //! Position limit event
        mtsFunctionWrite PositionLimit;
        //! Enabled joints event
        mtsFunctionWrite EnabledJoints;
    } Events;

    mtsInterfaceProvided * mInterface;

    /**
     * @brief Reset encoder, clear e/ed/ei value
     *
     */
    void ResetController(void);

    /*! Set the desired position, i.e. goal used in the PID
      controller.  See also EnableEffortMode to control with joints
      are controlled in position or effort mode. */
    void servo_jp(const prmPositionJointSet & command);

    /*! Set the effort feed forward for the PID controller.  The
      effort are added to the output of the PID controller.  These
      values are ignored for joints controlled in effort mode. */
    void feed_forward_jf(const prmForceTorqueJointSet & feedForward);

    /*! Set the effort directly, this by-passes the PID controller
      except for the effort limits.  See also EnableEffortMode to
      control with joints are controlled in position or effort
      mode. */
    void servo_jf(const prmForceTorqueJointSet & command);

    void Init(void);

    void SetupInterfaces(void);

    void Enable(const bool & enable);

    void EnableJoints(const vctBoolVec & enable);

    void EnableEffortMode(const vctBoolVec & enable);

    void SetTrackingErrorTolerances(const vctDoubleVec & tolerances);

    /*! Retrieve data from the IO component.  This method checks for
      the simulated flag and sets position/effort based on user
      commands if it is simulated.  If the IO component doesn't
      provide the velocity, the method estimates the velocity based on
      the previous measured position. */
    void GetIOData(void);

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

    /**
     * @brief Set joint configuration.  Size of vector of names must
     * match the current configuration.  Empty vectors are ignored and
     * non empty vectors must match the size of the current
     * configuration.
     *
     * @param configuration
     */
    void configure_js(const prmConfigurationJoint & configuration);

    /*! Return true is size mismatch.  Also sends error message and
      disables PID. */
    bool SizeMismatch(const size_t size, const std::string & methodName);

    /*! Sends a warning message if any lower limit is greater or equal
      to upper limit. */
    void CheckLowerUpper(const vctDoubleVec & lower, const vctDoubleVec & upper,
                         const std::string & methodName);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsPID);

#endif  // _mtsPID_h
