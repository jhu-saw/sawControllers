/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-22

  (C) Copyright 2013-2017 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmVelocityJointGet.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmActuatorJointCoupling.h>

#include <sawControllers/sawControllersRevision.h>

//! Always include last
#include <sawControllers/sawControllersExport.h>

class CISST_EXPORT mtsPID: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

protected:
    // Required interface
    struct InterfaceRobotTorque {
        //! Set actuator/joint coupling
        mtsFunctionWrite SetCoupling;
        //! Read joint type form robot
        mtsFunctionRead GetJointType;
        //! Read joint position from robot
        mtsFunctionRead GetFeedbackPosition;
        //! Read joint velocity from robot
        mtsFunctionRead GetFeedbackVelocity;
        //! Read joint effort from robot
        mtsFunctionRead GetFeedbackEffort;
        //! Write the joint efforts
        mtsFunctionWrite SetEffort;
    } Robot;


    //! Number of joints, set from the XML file used in Configure method
    size_t mNumberOfJoints;

    struct {
        //! Proportional gains
        vctDoubleVec Kp;
        //! Derivative gains
        vctDoubleVec Kd;
        //! Integral gains
        vctDoubleVec Ki;
        //! Offset
        vctDoubleVec Offset;
    } mGains;

    //! Position lower limit
    vctDoubleVec mPositionLowerLimit;
    //! Position upper limit
    vctDoubleVec mPositionUpperLimit;
    //! Flag whether check joint limit
    bool mCheckPositionLimit;
    vctBoolVec mPositionLimitFlagPrevious, mPositionLimitFlag;

    //! Effort lower limit
    vctDoubleVec mEffortLowerLimit;
    //! Effort upper limit
    vctDoubleVec mEffortUpperLimit;
    //! Flag whether to apply effort limit
    bool mApplyEffortLimit;

    //! Commanded joint efforts sent to IO level
    vctDoubleVec mEffortMeasure;
    prmForceTorqueJointSet mEffortPIDCommand;
    //! Desired joint efforts when bypassing PID
    prmForceTorqueJointSet mEffortUserCommand;

    //! prm type feedback positoin
    prmPositionJointGet mPositionMeasure;
    prmPositionJointGet mPositionMeasurePrevious;
    //! prm type feedback velocity
    prmVelocityJointGet mVelocityMeasure;
    //! prm type joint state
    prmStateJoint mStateJointMeasure, mStateJointCommand;

    //! Error
    vctDoubleVec mError;
    vctDoubleVec mIError;

    //! Min/max iError
    vctDoubleVec mIErrorLimitMin;
    vctDoubleVec mIErrorLimitMax;
    //! iError forgetting factor (0 < factor <= 1.0)
    vctDoubleVec mIErrorForgetFactor;

    //! If 0, use regular PID, else use as nonlinear factor
    vctDoubleVec mNonLinear;

    //! Deadband (errors less than this are set to 0)
    vctDoubleVec mDeadBand;

    //! Enable mtsPID controller
    bool mEnabled;

    //! Enable individal joints
    vctBoolVec mJointsEnabled;

    //! Enable mtsPID controller
    vctBoolVec mEffortMode;

    bool mTrackingErrorEnabled;
    vctDoubleVec mTrackingErrorTolerances;
    vctBoolVec mPreviousTrackingErrorFlag, mTrackingErrorFlag;

    // Flag to determine if this is connected to actual IO/hardware or
    // simulated
    bool mIsSimulated;

    // Counter of active joints
    size_t mNumberOfActiveJoints;

    //! Configuration state table
    mtsStateTable mConfigurationStateTable;

    struct {
        //! Enable event
        mtsFunctionWrite Enabled;
        //! Position limit event
        mtsFunctionWrite PositionLimit;
        //! Enabled joints event
        mtsFunctionWrite EnabledJoints;
        //! Coupling changed event
        mtsFunctionWrite Coupling;
    } Events;

    mtsInterfaceProvided * mInterface;

    /**
     * @brief Reset encoder, clear e/ed/ei value
     *
     */
    void ResetController(void);

    /*! See also EnableEffortMode to control with joints are
      controlled in position or effort mode. */
    void SetDesiredPosition(const prmPositionJointSet & command);

    /*! See also EnableEffortMode to control with joints are controlled
      in position or effort mode. */
    void SetDesiredEffort(const prmForceTorqueJointSet & command);

    void Init(void);

    void SetupInterfaces(void);

    void Enable(const bool & enable);

    void EnableJoints(const vctBoolVec & enable);

    void EnableEffortMode(const vctBoolVec & enable);

    void SetTrackingErrorTolerances(const vctDoubleVec & tolerances);

    void SetCoupling(const prmActuatorJointCoupling & coupling);

    /*! Retrieve data from the IO component.  This method checks for
      the simulated flag and sets position/effort based on user
      commands if it is simulated.  If the IO component doesn't
      provide the velocity, the method estimates the velocity based on
      the previous measured position.  When changing coupling, the
      previous position might be irrelevant so the user can skip
      velocity computation.  In this case, velocity is set to 0. */
    void GetIOData(const bool computeVelocity);

    /*! Utility method to convert vector of doubles
      (e.g. mStateJointCommand.Effort()) to cisstParameterType
      prmForceTorqueJointSet and then call the function to sent
      requested efforts to the IO component. */ 
    void SetEffortLocal(const vctDoubleVec & effort);

    void CouplingEventHandler(const prmActuatorJointCoupling & coupling);

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
    /**
     * @brief Set controller P gains
     *
     * @param pgain   new P gains
     */
    void SetPGain(const vctDoubleVec & pgain);

    /**
     * @brief Set controller D gains
     *
     * @param dgain   new D gains
     */
    void SetDGain(const vctDoubleVec & dgain);

    /**
     * @brief Set controller I gains
     *
     * @param igain  new I gains
     */
    void SetIGain(const vctDoubleVec & igain);

    /**
     * @brief Set joint position lower limit
     *
     * @param lowerLimit  new joint position lower limit
     */
    void SetPositionLowerLimit(const vctDoubleVec & lowerLimit);

    /**
     * @brief Set joint position upper limit
     *
     * @param upperLimit  new joint position upper limit
     */
    void SetPositionUpperLimit(const vctDoubleVec & upperLimit);

    /**
     * @brief Set joint effort lower limit
     *
     * @param lowerLimit  new joint effort lower limit
     */
    void SetEffortLowerLimit(const vctDoubleVec & lowerLimit);

    /**
     * @brief Set joint effort upper limit
     *
     * @param upperLimit  new joint effort upper limit
     */
    void SetEffortUpperLimit(const vctDoubleVec & upperLimit);

    /**
     * @brief Set minimum iError limit
     *
     * @param iminlim  minmum iError limit
     */
    void SetMinIErrorLimit(const vctDoubleVec & iminlim);


    /**
     * @brief Set maximum iError limit
     *
     * @param imaxlim  maximum iError limit
     */
    void SetMaxIErrorLimit(const vctDoubleVec & imaxlim);

    /**
     * @brief Set error integral forgetting factor
     *
     * @param forget  iError forgetting factor
     */
    void SetForgetIError(const double & forget);

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsPID);

#endif  // _mtsPID_h
