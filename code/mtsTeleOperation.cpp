/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Zihan Chen
  Created on: 2013-02-20

  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.

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


bool mtsTeleOperation::Robot::Configure(const std::string & filename)
{
    robManipulator::Errno result;
    result = this->Manipulator.LoadRobot(filename);
    if (result == robManipulator::EFAILURE) {
        return false;
    }
    return true;
}

mtsTeleOperation::mtsTeleOperation(const std::string & taskName, const double period)
    : mtsTaskPeriodic(taskName, period)
{

    Init();
}

mtsTeleOperation::mtsTeleOperation(const mtsTaskPeriodicConstructorArg &arg)
    : mtsTaskPeriodic(arg)
{
    Init();
}

void mtsTeleOperation::Init(void)
{
    this->StateTable.AddData(Master.CartesianCurrent, "MasterCartesianPosition");
    this->StateTable.AddData(Slave.CartesianCurrent, "MasterCartesianPosition");

    // Setup CISST Interface
    mtsInterfaceRequired *req = AddInterfaceRequired("Master");
    if (req) {
        req->AddFunction("GetPositionJoint", Master.GetPositionJoint);
        // req->AddFunction("SetCartesianPosition", Master.SetCartesianPosition);
    }

    std::cerr << CMN_LOG_DETAILS << " this need to be required when I have a working slave" << std::endl;
    req = AddInterfaceRequired("Slave", MTS_OPTIONAL);
    if (req) {
        req->AddFunction("GetPositionJoint", Slave.GetPositionJoint);
        // req->AddFunction("SetCartesianPosition", Slave.SetPositionCartesian);
    }

    mtsInterfaceProvided *prov = AddInterfaceProvided("Setting");
    if (prov) {
        prov->AddCommandWrite(&mtsTeleOperation::SetScale, this, "SetScale", mtsDouble());
        prov->AddCommandWrite(&mtsTeleOperation::SetRegistrationRotation, this,
                              "SetRegistrationRoation", vctMatRot3());

        prov->AddCommandVoid(&mtsTeleOperation::AllignMasterToSlave, this, "AllignMasterToSlave");
        prov->AddCommandReadState(this->StateTable, Master.CartesianCurrent, "GetPositionCartesianMaster");
        prov->AddCommandReadState(this->StateTable, Slave.CartesianCurrent, "GetCartesianPositionSlave");
    }
}

void mtsTeleOperation::Configure(const std::string &filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsTeleOperation::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsPIDQt::Startup" << std::endl;

    // Set desired pos to cur pos

    // align master to align slave position on startup();

}

void mtsTeleOperation::Run(void)
{
    mtsExecutionResult executionResult;
    executionResult = Master.GetPositionJoint(Master.JointCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Call to Master.GetJointPosition failed \""
                                << executionResult << "\"" << std::endl;
    }
    vctFrm4x4 masterPosition;
    masterPosition = Master.Manipulator.ForwardKinematics(Master.JointCurrent.Position());
    Master.CartesianCurrent.Position().From(masterPosition);

    // Clutch
    // Follow mode

    if (IsClutched) {
        /*
        Master.GetCartesianPosition(Master.PositionCurrent);
        Master.PositionCurrent.Position().Rotation().Assign(MasterClutchedOrientation);

        Master.PositionDesired.SetGoal(Master.PositionCurrent.Position());
        Master.SetCartesianPosition(Master.PositionDesired);
*/
    } else {
/*
        Master.GetCartesianPosition(Master.PositionCurrent);
        vctFrm3 mPos;
        ComputeMasterToSlaveFrame(Master.PositionCurrent.Position(), mPos);

        Slave.PositionDesired.SetGoal(Offset.ApplyTo(mPos));
        Slave.SetCartesianPosition(Slave.PositionDesired);
        */
    }
}

void mtsTeleOperation::Cleanup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Cleanup" << std::endl;
}

void mtsTeleOperation::EventHandlerClutched(const prmEventButton &button)
{

    if (button.Type() == prmEventButton::PRESSED) {
        this->IsClutched = true;
        /*
        Master.GetCartesianPosition(Master.PositionCurrent);
        MasterClutchedOrientation.Assign(Master.PositionCurrent.Position().Rotation());
        */
    } else {
        this->IsClutched = false;
/*
        //! \todo compute clutch offset

        Master.GetCartesianPosition(Master.PositionCurrent);
        Slave.GetCartesianPosition(Slave.PositionCurrent);

        vctFrm3 Rt_ms;
        ComputeMasterToSlaveFrame(Master.PositionCurrent.Position(), Rt_ms);

        // update Offset
        Offset = Rt_ms * Slave.PositionCurrent.Position().Inverse();

        //! \todo add a flag
        // set rot to identity()
        Offset.Rotation().Assign(vctMatRot3::Identity());
        */
    }
}

void mtsTeleOperation::AllignMasterToSlave(void)
{
    //! \todo Reverse this procedure

//    vctFrm3 Rt_ms;
//    Rt_ms = ComputeMasterToSlaveFrame(masterPos.GetPosition());

//    // update Offset
//    Offset = Rt_ms * slavePos.Position().Inverse();

//    //! \todo add a flag
//    // set rot to identity()
//    Offset.Rotation().Assign(vctMatRot3::Identity());
}

void mtsTeleOperation::ComputeMasterToSlaveFrame(const vctFrm3 & mPos,
                                                 vctFrm3 & sPos)
{
//    sPos.Translation().Multiply(this->Scale);
}

void mtsTeleOperation::SetScale(const mtsDouble & scale)
{
    this->Scale = scale;
}

void mtsTeleOperation::SetRegistrationRotation(const vctMatRot3 & rot)
{
    this->RegistrationRotation = rot;
}

void mtsTeleOperation::ConfigureMaster(const std::string & filename)
{
    if (!Master.Configure(filename)) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureMaster: failed to load file \"" << filename << "\"" << std::endl;
    }
}

void mtsTeleOperation::ConfigureSlave(const std::string & filename)
{
    if (!Slave.Configure(filename)) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureSlave: failed to load file \"" << filename << "\"" << std::endl;
    }
}
