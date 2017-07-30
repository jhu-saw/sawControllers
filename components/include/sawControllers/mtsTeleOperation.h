/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-03-06

  (C) Copyright 2013-2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsTeleOperation_h
#define _mtsTeleOperation_h

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstRobot/robManipulator.h>

//! Always include last
#include <sawControllers/sawControllersExport.h>

/**
 * @brief  teleoperation component
 *
 *    position: translation + rotation (vctFrm4x4)
 *    translation: 3D x,y,z (vct3)
 *    rotation: 3x3 rotation (vctMatRot3)
 *
 * \todo
 *
 */
class CISST_EXPORT mtsTeleOperation: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsTeleOperation(const std::string & componentName, const double periodInSeconds);
    mtsTeleOperation(const mtsTaskPeriodicConstructorArg & arg);
    ~mtsTeleOperation(){}

    void Configure(const std::string & filename = "");
    void Startup(void);
    void Run(void);
    void Cleanup(void);

    void ConfigureMaster(const std::string & filename);
    void ConfigureSlave(const std::string & filename);

    void SetScale(const double & scale);
    void SetRegistrationRotation(const vctMatRot3 & rotation);
    void LockRotation(const bool & lock);
    void LockTranslation(const bool & lock);

private:

    void Init(void);

    // Event Handler
    void MasterErrorEventHandler(const std::string & message);
    void SlaveErrorEventHandler(const std::string & message);

    void SlaveClutchEventHandler(const prmEventButton & button);
    void CameraClutchEventHandler(const prmEventButton & button);
    void StartAlignMaster(void);

    void ClutchEventHandler(const prmEventButton & button);
    void OperatorPresentEventHandler(const prmEventButton & button);

    // Functions for events
    struct {
        mtsFunctionWrite Status;
        mtsFunctionWrite Warning;
        mtsFunctionWrite Error;
        mtsFunctionWrite Enabled;
    } MessageEvents;

    struct {
        mtsFunctionWrite Scale;
        mtsFunctionWrite RotationLocked;
        mtsFunctionWrite TranslationLocked;
    } ConfigurationEvents;

    void Enable(const bool & enable);

    /**
     * @brief Set MTM control states based on teleop component state
     *        and control input device (cluch & operatorPresent).
     *
     *  WARNING: should only be called by event handlers
     */
    void SetMasterControlState(void);

protected:

    class RobotMaster {
    public:
        mtsFunctionRead GetPositionCartesian;
        mtsFunctionWrite SetPositionCartesian;
        mtsFunctionWrite SetPositionGoalCartesian;
        mtsFunctionWrite SetRobotControlState;

        mtsFunctionRead GetGripperPosition;

        prmPositionCartesianGet PositionCartesianCurrent;
        prmPositionCartesianSet PositionCartesianDesired;
        vctFrm4x4 CartesianPrevious;
    };
    RobotMaster Master;

    class RobotSlave {
    public:
        mtsFunctionRead GetPositionCartesian;
        mtsFunctionWrite SetPositionCartesian;
        mtsFunctionWrite SetRobotControlState;

        mtsFunctionWrite SetJawPosition;

        prmPositionCartesianGet PositionCartesianCurrent;
        prmPositionCartesianSet PositionCartesianDesired;
        vctFrm4x4 CartesianPrevious;
        bool IsManipClutched;
        bool IsSUJClutched;
    };
    RobotSlave Slave;

private:

    int Counter;
    double Scale;
    vctMatRot3 RegistrationRotation;
    vctFrm3 Offset;
    vct3 MasterLockTranslation;

    bool IsClutched;
    bool IsOperatorPresent;
    bool IsEnabled;
    bool RotationLocked;
    bool TranslationLocked;

    vctMatRot3 MasterClutchedOrientation;

    mtsStateTable * ConfigurationStateTable;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsTeleOperation);

#endif // _mtsTeleOperation_h
