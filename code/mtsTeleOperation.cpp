/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-20

  (C) Copyright 2013-2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


// system include
#include <iostream>

// cisst
#include <sawControllers/mtsTeleOperation.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>


CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsTeleOperation, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

mtsTeleOperation::mtsTeleOperation(const std::string & componentName, const double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds)
{
    Init();
}

mtsTeleOperation::mtsTeleOperation(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg)
{
    Init();
}

void mtsTeleOperation::Init(void)
{
    Counter = 0;
    Scale = 0.2;

    // Initialize states
    this->IsClutched = false;
    this->IsOperatorPresent = false;
    this->IsEnabled = false;
    Slave.IsManipClutched = false;
    Slave.IsSUJClutched = false;

    this->StateTable.AddData(Master.PositionCartesianCurrent, "MasterCartesianPosition");
    this->StateTable.AddData(Slave.PositionCartesianCurrent, "SlaveCartesianPosition");

    this->ConfigurationStateTable = new mtsStateTable(100, "Configuration");
    this->ConfigurationStateTable->SetAutomaticAdvance(false);
    this->AddStateTable(this->ConfigurationStateTable);
    this->ConfigurationStateTable->AddData(this->RegistrationRotation, "RegistrationRotation");

    // Setup CISST Interface
    mtsInterfaceRequired * masterRequired = AddInterfaceRequired("Master");
    if (masterRequired) {
        masterRequired->AddFunction("GetPositionCartesian", Master.GetPositionCartesian);
        masterRequired->AddFunction("SetPositionCartesian", Master.SetPositionCartesian);
        masterRequired->AddFunction("SetPositionGoalCartesian", Master.SetPositionGoalCartesian);
        masterRequired->AddFunction("GetGripperPosition", Master.GetGripperPosition);
        masterRequired->AddFunction("SetRobotControlState", Master.SetRobotControlState);
        masterRequired->AddEventHandlerWrite(&mtsTeleOperation::MasterErrorEventHandler, this, "Error");
    }

    mtsInterfaceRequired * slaveRequired = AddInterfaceRequired("Slave");
    if (slaveRequired) {
        slaveRequired->AddFunction("GetPositionCartesian", Slave.GetPositionCartesian);
        slaveRequired->AddFunction("SetPositionCartesian", Slave.SetPositionCartesian);
        slaveRequired->AddFunction("SetJawPosition", Slave.SetJawPosition);
        slaveRequired->AddFunction("SetRobotControlState", Slave.SetRobotControlState);

        slaveRequired->AddEventHandlerWrite(&mtsTeleOperation::SlaveErrorEventHandler, this, "Error");
        slaveRequired->AddEventHandlerWrite(&mtsTeleOperation::ManipClutchEventHandler, this, "ManipClutch");
        slaveRequired->AddEventHandlerWrite(&mtsTeleOperation::SUJClutchEventHandler, this, "SUJClutch");
    }

    // Footpedal events
    mtsInterfaceRequired * clutchRequired = AddInterfaceRequired("Clutch");
    if (clutchRequired) {
        clutchRequired->AddEventHandlerWrite(&mtsTeleOperation::ClutchEventHandler, this, "Button");
    }

    mtsInterfaceRequired * headRequired = AddInterfaceRequired("OperatorPresent");
    if (headRequired) {
        headRequired->AddEventHandlerWrite(&mtsTeleOperation::OperatorPresentEventHandler, this, "Button");
    }

    mtsInterfaceProvided * providedSettings = AddInterfaceProvided("Setting");
    if (providedSettings) {
        // commands
        providedSettings->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                              "GetPeriodStatistics"); // mtsIntervalStatistics

        providedSettings->AddCommandWrite(&mtsTeleOperation::Enable, this, "Enable", false);
        providedSettings->AddCommandWrite(&mtsTeleOperation::SetScale, this, "SetScale", 0.5);
        providedSettings->AddCommandWrite(&mtsTeleOperation::SetRegistrationRotation, this,
                                          "SetRegistrationRotation", vctMatRot3());
        providedSettings->AddCommandReadState(*(this->ConfigurationStateTable), RegistrationRotation, "GetRegistrationRotation");

        providedSettings->AddCommandReadState(this->StateTable, Master.PositionCartesianCurrent, "GetPositionCartesianMaster");
        providedSettings->AddCommandReadState(this->StateTable, Slave.PositionCartesianCurrent, "GetPositionCartesianSlave");
        // events
        providedSettings->AddEventWrite(MessageEvents.Status, "Status", std::string(""));
        providedSettings->AddEventWrite(MessageEvents.Warning, "Warning", std::string(""));
        providedSettings->AddEventWrite(MessageEvents.Error, "Error", std::string(""));
        providedSettings->AddEventWrite(MessageEvents.Enabled, "Enabled", false);
    }
}

void mtsTeleOperation::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsTeleOperation::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Startup" << std::endl;
}

void mtsTeleOperation::Run(void)
{
    ProcessQueuedCommands();
    ProcessQueuedEvents();

    // increment counter
    Counter++;

    // get master Cartesian position
    mtsExecutionResult executionResult;
    executionResult = Master.GetPositionCartesian(Master.PositionCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to Master.GetPositionCartesian failed \""
                                << executionResult << "\"" << std::endl;
        MessageEvents.Error(this->GetName() + ": unable to get cartesian position from master");
        this->Enable(false);
    }
    vctFrm4x4 masterPosition(Master.PositionCartesianCurrent.Position());

    // get slave Cartesian position
    executionResult = Slave.GetPositionCartesian(Slave.PositionCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to Slave.GetPositionCartesian failed \""
                                << executionResult << "\"" << std::endl;
        MessageEvents.Error(this->GetName() + ": unable to get cartesian position from slave");
        this->Enable(false);
    }

    /*!
      mtsTeleOperation can run in 4 control modes, which is controlled by
      footpedal Clutch & OperatorPresent.

      Mode 1: OperatorPresent = False, Clutch = False
              MTM and PSM stop at their current position. If PSM ManipClutch is
              pressed, then the user can manually move PSM.
              NOTE: MTM always tries to allign its orientation with PSM's orientation

      Mode 2/3: OperatorPresent = False/True, Clutch = True
              MTM can move freely in workspace, however its orientation is locked
              PSM can not move

      Mode 4: OperatorPresent = True, Clutch = False
              PSM follows MTM motion
    */
    if (IsEnabled
        && Master.PositionCartesianCurrent.Valid()
        && Slave.PositionCartesianCurrent.Valid()) {
        // follow mode
        if (!IsClutched && IsOperatorPresent) {
            // compute master Cartesian motion
            vctFrm4x4 masterCartesianMotion;
            masterCartesianMotion = Master.CartesianPrevious.Inverse() * masterPosition;

            // translation
            vct3 masterTranslation;
            vct3 slaveTranslation;
            masterTranslation = (masterPosition.Translation() - Master.CartesianPrevious.Translation());
            slaveTranslation = masterTranslation * this->Scale;
            slaveTranslation = RegistrationRotation * slaveTranslation + Slave.CartesianPrevious.Translation();

            // rotation
            vctMatRot3 slaveRotation;
            slaveRotation = RegistrationRotation * masterPosition.Rotation();

            // compute desired slave position
            vctFrm4x4 slaveCartesianDesired;
            slaveCartesianDesired.Translation().Assign(slaveTranslation);
            slaveCartesianDesired.Rotation().FromNormalized(slaveRotation);
            Slave.PositionCartesianDesired.Goal().FromNormalized(slaveCartesianDesired);

            // Slave go this cartesian position
            Slave.SetPositionCartesian(Slave.PositionCartesianDesired);

            // Gripper
            if (Master.GetGripperPosition.IsValid()) {
                double gripperPosition;
                Master.GetGripperPosition(gripperPosition);
                Slave.SetJawPosition(gripperPosition);
            } else {
                Slave.SetJawPosition(5.0 * cmnPI_180);
            }
        } else if (!IsClutched && !IsOperatorPresent) {
            // Do nothing
        }
    } else {
        CMN_LOG_CLASS_RUN_DEBUG << "mtsTeleOperation disabled" << std::endl;
    }
}

void mtsTeleOperation::Cleanup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Cleanup" << std::endl;
}

void mtsTeleOperation::MasterErrorEventHandler(const std::string & message)
{
    this->Enable(false);
    MessageEvents.Error(this->GetName() + ": received from master [" + message + "]");
}

void mtsTeleOperation::SlaveErrorEventHandler(const std::string & message)
{
    this->Enable(false);
    MessageEvents.Error(this->GetName() + ": received from slave [" + message + "]");
}

void mtsTeleOperation::ManipClutchEventHandler(const prmEventButton & button)
{
    if (button.Type() == prmEventButton::PRESSED) {
        Slave.IsManipClutched = true;
        MessageEvents.Status(this->GetName() + ": slave clutch pressed");
        CMN_LOG_CLASS_RUN_DEBUG << "EventHandlerManipClutch: ManipClutch pressed" << std::endl;
    } else {
        Slave.IsManipClutched = false;
        MessageEvents.Status(this->GetName() + ": slave clutch released");
        CMN_LOG_CLASS_RUN_DEBUG << "EventHandlerManipClutch: ManipClutch released" << std::endl;
    }

    // Slave State
    if (IsEnabled && !IsOperatorPresent && Slave.IsManipClutched) {
        Slave.SetRobotControlState(mtsStdString("Manual"));
    } else if (IsEnabled) {
        Slave.SetRobotControlState(mtsStdString("Teleop"));
    }

    // Master
    if (IsEnabled && !Slave.IsManipClutched) {
        vctFrm4x4 masterCartesianDesired;
        masterCartesianDesired.Translation().Assign(MasterLockTranslation);
        vctMatRot3 masterRotation;
        masterRotation = RegistrationRotation.Inverse() * Slave.PositionCartesianCurrent.Position().Rotation();
        masterCartesianDesired.Rotation().FromNormalized(masterRotation);

        // Send Master command position
        Master.SetRobotControlState(mtsStdString("DVRK_POSITION_GOAL_CARTESIAN"));
        Master.PositionCartesianDesired.Goal().FromNormalized(masterCartesianDesired);
        Master.SetPositionGoalCartesian(Master.PositionCartesianDesired);
    }
}

void mtsTeleOperation::SUJClutchEventHandler(const prmEventButton & button)
{
    if (button.Type() == prmEventButton::PRESSED) {
        Slave.IsSUJClutched = true;
        CMN_LOG_CLASS_RUN_DEBUG << "EventHandlerSUJClutch: SUJClutch pressed" << std::endl;
    } else {
        Slave.IsSUJClutched = false;
        CMN_LOG_CLASS_RUN_DEBUG << "EventHandlerSUJClutch: SUJClutch released" << std::endl;
    }
}

void mtsTeleOperation::ClutchEventHandler(const prmEventButton & button)
{
    mtsExecutionResult executionResult;
    executionResult = Master.GetPositionCartesian(Master.PositionCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "EventHandlerClutched: call to Master.GetPositionCartesian failed \""
                                << executionResult << "\"" << std::endl;
    }
    executionResult = Slave.GetPositionCartesian(Slave.PositionCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "EventHandlerClutched: call to Slave.GetPositionCartesian failed \""
                                << executionResult << "\"" << std::endl;
    }

    if (button.Type() == prmEventButton::PRESSED) {
        this->IsClutched = true;
        Master.PositionCartesianDesired.Goal().Rotation().FromNormalized(
                    Slave.PositionCartesianCurrent.Position().Rotation());
        Master.PositionCartesianDesired.Goal().Translation().Assign(
                    Master.PositionCartesianCurrent.Position().Translation());
        MessageEvents.Status(this->GetName() + ": master clutch pressed");
    } else {
        this->IsClutched = false;
        MessageEvents.Status(this->GetName() + ": master clutch released");
    }
    SetMasterControlState();
}

void mtsTeleOperation::OperatorPresentEventHandler(const prmEventButton & button)
{
    if (button.Type() == prmEventButton::PRESSED) {
        this->IsOperatorPresent = true;
        MessageEvents.Status(this->GetName() + ": operator present");
        CMN_LOG_CLASS_RUN_DEBUG << "EventHandlerOperatorPresent: OperatorPresent pressed" << std::endl;
    } else {
        this->IsOperatorPresent = false;
        MessageEvents.Status(this->GetName() + ": operator not present");
        CMN_LOG_CLASS_RUN_DEBUG << "EventHandlerOperatorPresent: OperatorPresent released" << std::endl;
    }
    SetMasterControlState();
}

void mtsTeleOperation::Enable(const bool & enable)
{
    IsEnabled = enable;

    if (IsEnabled) {
        // Set Master/Slave to Teleop (Cartesian Position Mode)
        SetMasterControlState();
        Slave.SetRobotControlState(mtsStdString("Teleop"));

        // Orientate Master with Slave
        vctFrm4x4 masterCartesianDesired;
        masterCartesianDesired.Translation().Assign(MasterLockTranslation);
        vctMatRot3 masterRotation;
        masterRotation = RegistrationRotation.Inverse() * Slave.PositionCartesianCurrent.Position().Rotation();
        masterCartesianDesired.Rotation().FromNormalized(masterRotation);

        // Send Master command postion
        Master.SetRobotControlState(mtsStdString("DVRK_POSITION_GOAL_CARTESIAN"));
        Master.PositionCartesianDesired.Goal().FromNormalized(masterCartesianDesired);
        Master.SetPositionGoalCartesian(Master.PositionCartesianDesired);
    }

    // Send event for GUI
    MessageEvents.Enabled(IsEnabled);
}

void mtsTeleOperation::SetScale(const double & scale)
{
    this->Scale = scale;
}

void mtsTeleOperation::SetRegistrationRotation(const vctMatRot3 & rotation)
{
    this->ConfigurationStateTable->Start();
    this->RegistrationRotation = rotation;
    this->ConfigurationStateTable->Advance();
}

void mtsTeleOperation::SetMasterControlState(void)
{
    if (IsEnabled == false) {
        CMN_LOG_CLASS_RUN_WARNING << "TeleOperation is NOT enabled" << std::endl;
        return;
    }

    if (IsClutched) {
        Master.SetRobotControlState(mtsStdString("Clutch"));
    } else {
        if (IsOperatorPresent) {
            Master.SetRobotControlState(mtsStdString("Gravity"));
        } else {
            MasterLockTranslation.Assign(Master.PositionCartesianCurrent.Position().Translation());
            Master.SetRobotControlState(mtsStdString("DVRK_POSITION_CARTESIAN"));
            Master.PositionCartesianDesired.SetGoal(Master.PositionCartesianCurrent.Position());
            Master.SetPositionCartesian(Master.PositionCartesianDesired);
        }
    }

    // Update MTM/PSM previous position
    Master.CartesianPrevious.From(Master.PositionCartesianCurrent.Position());
    Slave.CartesianPrevious.From(Slave.PositionCartesianCurrent.Position());
}
