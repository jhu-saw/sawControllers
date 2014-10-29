/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen
  Created on: 2013-02-22

  (C) Copyright 2013-2014 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstParameterTypes/prmJointType.h>

#include <sawControllers/sawControllersExport.h>

class CISST_EXPORT mtsPID : public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

protected:
    // Required interface
    struct InterfaceRobotTorque {
        //! Read joint type form robot
        mtsFunctionRead GetJointType;
        //! Read joint position from robot
        mtsFunctionRead GetFeedbackPosition;
        //! Read joint velocity from robot
        mtsFunctionRead GetFeedbackVelocity;
        //! Write the joint torques
        mtsFunctionWrite SetTorque;
    } Robot;

    //! Counter for internal use
    int Counter;

    //! Proportional gains
    vctDoubleVec Kp;
    //! Derivative gains
    vctDoubleVec Kd;
    //! Integral gains
    vctDoubleVec Ki;
    //! Offset
    vctDoubleVec Offset;
    //! Joint lower limit
    vctDoubleVec JointLowerLimit;
    //! Joint upper limit
    vctDoubleVec JointUpperLimit;
    //! Flag whether check joint limit
    bool CheckJointLimit;


    // TODO: change to prmPositionJointGet
    //! Feedback joint positions
    vctDoubleVec FeedbackPosition;
    //! Desired joint positions
    vctDoubleVec DesiredPosition;
    //! Desired joint positions
    vctDoubleVec DesiredTorque;
    //! Feedback joint velocities
    vctDoubleVec FeedbackVelocity;
    //! Desired joint velocities
    vctDoubleVec DesiredVelocity;
    //! Torque set to robot
    vctDoubleVec Torque;

    //! prm type joint type
    prmJointTypeVec JointType;
    //! prm type feedback positoin
    prmPositionJointGet FeedbackPositionParam;
    prmPositionJointGet FeedbackPositionPreviousParam;
    //! prm type desired position
    prmPositionJointSet DesiredPositionParam;
    //! prm type desired torque
    prmForceTorqueJointSet prmDesiredTrq;
    //! prm type feedback velocity
    prmVelocityJointGet FeedbackVelocityParam;
    //! prm type set torque
    prmForceTorqueJointSet TorqueParam;

    //! Error
    vctDoubleVec Error;
    vctDoubleVec ErrorAbsolute;

    //! Error derivative
    vctDoubleVec dError;
    //! Error integral
    vctDoubleVec iError;

    //! Previous error
    vctDoubleVec oldError;
    //! Previous desiredPos
    vctDoubleVec oldDesiredPos;

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

    //! Enable mtsPID controller
    vctBoolVec TorqueMode;

    bool mEnableTrackingError;
    vctDoubleVec mTrackingErrorTolerances;

    //! Configuration state table
    mtsStateTable ConfigurationStateTable;

    //! Enable event
    mtsFunctionWrite EventPIDEnable;

    // !Tracking error event
    mtsFunctionVoid EventTrackingError;

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
    void SetDesiredPositions(const prmPositionJointSet & prmPos);
    void SetDesiredTorques(const prmForceTorqueJointSet& prmTrq);

    void SetupInterfaces(void);

    void Enable(const bool & enable);

    void EnableTorqueMode(const vctBoolVec & enable);

    void SetTrackingErrorTolerances(const vctDoubleVec & tolerances);

public:

    /**
     * @brief Main constructor
     *
     * @param taskname   The name of the MTS periodic task
     * @param period     The period of the task
     */
    mtsPID(const std::string & taskname,
           double period);
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
