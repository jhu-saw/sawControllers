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
#include <cisstParameterTypes/prmJointType.h>
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
        //! Read joint torque from robot
        mtsFunctionRead GetFeedbackTorque;
        //! Write the joint torques
        mtsFunctionWrite SetTorque;
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

    //! Joint lower limit
    vctDoubleVec JointLowerLimit;
    //! Joint upper limit
    vctDoubleVec JointUpperLimit;
    //! Flag whether check joint limit
    bool CheckJointLimit;
    vctBoolVec mPreviousJointLimitFlag, mJointLimitFlag;

    //! Torque lower limit
    vctDoubleVec TorqueLowerLimit;
    //! Torque upper limit
    vctDoubleVec TorqueUpperLimit;
    //! Flag whether to apply torque limit
    bool ApplyTorqueLimit;

    //! Desired joint positions
    vctDoubleVec DesiredPosition;
    //! mMeasured joint positions

    vctDoubleVec mTorqueMeasure;
    //! Commanded joint torques sent to IO level
    prmForceTorqueJointSet mTorqueCommand;
    //! Desired joint torques when bypassing PID
    prmForceTorqueJointSet mTorqueUserCommand;

    //! prm type joint type
    prmJointTypeVec JointType;
    //! prm type feedback positoin
    prmPositionJointGet mPositionMeasure;
    prmPositionJointGet mPositionMeasurePrevious;
    //! prm type desired position
    prmPositionJointSet mPositionCommand;
    //! prm type feedback velocity
    prmVelocityJointGet mVelocityMeasure;
    //! prm type joint state
    prmStateJoint mStateJointMeasure, mStateJointCommand;

    //! Error
    vctDoubleVec Error;
    vctDoubleVec ErrorAbsolute;

    //! Error derivative
    vctDoubleVec dError;
    //! Error integral
    vctDoubleVec iError;

    //! Min iError
    vctDoubleVec minIErrorLimit;
    //! Max iError
    vctDoubleVec maxIErrorLimit;

    //! iError forgetting factor
    vctDoubleVec forgetIError;

    //! If 0, use regular PID, else use as nonlinear factor
    vctDoubleVec nonlinear;

    //! Deadband (errors less than this are set to 0)
    vctDoubleVec DeadBand;

    //! Enable mtsPID controller
    bool Enabled;

    //! Enable individal joints
    vctBoolVec mJointsEnabled;

    //! Enable mtsPID controller
    vctBoolVec TorqueMode;

    bool mEnableTrackingError;
    vctDoubleVec mTrackingErrorTolerances;
    vctBoolVec mPreviousTrackingErrorFlag, mTrackingErrorFlag;
    
    // Flag to determine if this is connected to actual IO/hardware or
    // simulated
    bool mIsSimulated;

    // Counter of active joints
    size_t mNumberOfActiveJoints;

    //! Configuration state table
    mtsStateTable ConfigurationStateTable;

    struct {
        //! Enable event
        mtsFunctionWrite Enabled;
        // !Joint limit event
        mtsFunctionWrite JointLimit;
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

    /**
     * @brief Set desired joint position
     *
     * @param prmPos   The desired position
     */
    void SetDesiredPosition(const prmPositionJointSet & prmPos);
    void SetDesiredTorque(const prmForceTorqueJointSet& prmTrq);

    void SetupInterfaces(void);

    void Enable(const bool & enable);

    void EnableJoints(const vctBoolVec & enable);

    void EnableTorqueMode(const vctBoolVec & enable);

    void SetTrackingErrorTolerances(const vctDoubleVec & tolerances);

    void SetCoupling(const prmActuatorJointCoupling & coupling);

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
     * @brief Set joint lower limit
     *
     * @param lowerLimit  new joint lower limit
     */
    void SetJointLowerLimit(const vctDoubleVec & lowerLimit);

    /**
     * @brief Set joint upper limit
     *
     * @param upperLimit  new joint upper limit
     */
    void SetJointUpperLimit(const vctDoubleVec & upperLimit);

    /**
     * @brief Set torque lower limit
     *
     * @param lowerLimit  new torque lower limit
     */
    void SetTorqueLowerLimit(const vctDoubleVec & lowerLimit);

    /**
     * @brief Set torque upper limit
     *
     * @param upperLimit  new torque upper limit
     */
    void SetTorqueUpperLimit(const vctDoubleVec & upperLimit);

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
