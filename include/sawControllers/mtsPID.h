/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */
/*
  $Id$

  Author(s):  Zihan Chen
  Created on: 2013-02-22

  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.

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

#include <cisstVector.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstParameterTypes/prmForceTorqueJointSet.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmVelocityJointGet.h>

#include <sawControllers/sawControllersExport.h>

class CISST_EXPORT mtsPID : public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

private:
    // Required interface
    struct InterfaceRobotTorque {
        //! Read joint position from robot
        mtsFunctionRead GetFeedbackPosition;
        //! Read joint velocity from robot
        mtsFunctionRead GetFeedbackVelocity;
        //! Write the joint torques
        mtsFunctionWrite SetTorque;
    }Robot;

    //! Proportional gains
    vctDoubleVec Kp;
    //! Derivative gains
    vctDoubleVec Kd;
    //! Integral gains
    vctDoubleVec Ki;
    //! Offset
    vctDoubleVec Offset;

    // TODO: change to prmPositionJointGet
    //! Feedback joint positions
    vctDoubleVec feedbackPos;
    //! Desired joint positions
    vctDoubleVec desiredPos;
    //! Feedback joint velocities
    vctDoubleVec feedbackVel;
    //! Desired joint velocities
    vctDoubleVec desiredVel;
    //! Torque set to robot
    vctDoubleVec torque;

    //! prm type feedback positoin
    prmPositionJointGet prmFeedbackPos;
    //! prm type desired position
    prmPositionJointSet prmDesiredPos;
    //! prm type feedback velocity
    prmVelocityJointGet prmFeedbackVel;
    //! prm type set torque
    prmForceTorqueJointSet prmTorque;

    //! Error
    vctDoubleVec Error;
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
    //! Error limit
    vctDoubleVec errorLimit;

    //! iError forgetting factor
    vctDoubleVec forgetIError;

    //! Enable mtsPID controller
    bool enabled;


protected:

    //! Error limit event trigger
    mtsFunctionVoid EventErrorLimit;

    /**
     * @brief Reset encoder, clear e/ed/ei value
     *
     */
    void ResetController();

    /**
     * @brief Set desired joint position
     *
     * @param prmPos   The desired position
     */
    void SetDesiredPositions(const prmPositionJointSet &prmPos);

    void SetupInterfaces(void);

    void Enable(const mtsBool &ena);

public:

    /**
     * @brief Main constructor
     *
     * @param taskname   The name of the MTS periodic task
     * @param period     The period of the task
     */
    mtsPID( const std::string& taskname,
            double period );
    mtsPID( const mtsTaskPeriodicConstructorArg &arg);
    ~mtsPID(){}

    /**
     * @brief Configure PID gains & params
     *
     * @param filename  The name of the configuration file
     */
    void Configure( const std::string& filename );
    void Startup();
    void Run();
    void Cleanup();

    /**
     * @brief Set controller P gains
     *
     * @param pgain   new P gains
     */
    void SetPGain(const vctDoubleVec &pgain);

    /**
     * @brief Set controller D gains
     *
     * @param dgain   new D gains
     */
    void SetDGain(const vctDoubleVec &dgain);

    /**
     * @brief Set controller I gains
     *
     * @param igain  new I gains
     */
    void SetIGain(const vctDoubleVec &igain);

    /**
     * @brief Set minimum iError limit
     *
     * @param iminlim  minmum iError limit
     */
    void SetMinIErrorLimit(const vctDoubleVec &iminlim);


    /**
     * @brief Set maximum iError limit
     *
     * @param imaxlim  maximum iError limit
     */
    void SetMaxIErrorLimit(const vctDoubleVec &imaxlim);

    /**
     * @brief Set error integral forgetting factor
     *
     * @param forget  iError forgetting factor
     */
    void SetForgetIError(const double &forget);

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsPID);

#endif  // _mtsPID_h
