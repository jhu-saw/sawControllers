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

mtsTeleOperation::mtsTeleOperation(const std::string & taskName, const double period):
    mtsTaskPeriodic(taskName, period),
    Scale(0.2)
{
    Init();
}

mtsTeleOperation::mtsTeleOperation(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg),
    Scale(0.2)
{
    Init();
}

void mtsTeleOperation::Init(void)
{
    this->IsClutched = true;

    this->StateTable.AddData(Master.CartesianCurrent, "MasterCartesianPosition");
    this->StateTable.AddData(Slave.CartesianCurrent, "SlaveCartesianPosition");

    // Setup CISST Interface
    mtsInterfaceRequired * req;
    req = AddInterfaceRequired("Master");
    if (req) {
        req->AddFunction("GetPositionJoint", Master.GetPositionJoint);
        // req->AddFunction("SetCartesianPosition", Master.SetCartesianPosition);
    }

    req = AddInterfaceRequired("Slave");
    if (req) {
        req->AddFunction("GetPositionJoint", Slave.GetPositionJoint);
        req->AddFunction("SetPositionJoint", Slave.SetPositionJoint);
    }

    req = AddInterfaceRequired("Clutch");
    if (req) {
        req->AddEventHandlerWrite(&mtsTeleOperation::EventHandlerClutched, this, "Button");
        // req->AddFunction("SetCartesianPosition", Slave.SetPositionCartesian);
    }

    mtsInterfaceProvided *prov = AddInterfaceProvided("Setting");
    if (prov) {
        prov->AddCommandWrite(&mtsTeleOperation::SetScale, this, "SetScale", mtsDouble());
        prov->AddCommandWrite(&mtsTeleOperation::SetRegistrationRotation, this,
                              "SetRegistrationRotation", vctMatRot3());

        prov->AddCommandVoid(&mtsTeleOperation::AllignMasterToSlave, this, "AllignMasterToSlave");
        prov->AddCommandReadState(this->StateTable, Master.CartesianCurrent, "GetPositionCartesianMaster");
        prov->AddCommandReadState(this->StateTable, Slave.CartesianCurrent, "GetPositionCartesianSlave");
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
    ProcessQueuedCommands();
    ProcessQueuedEvents();

    mtsExecutionResult executionResult;
    executionResult = Master.GetPositionJoint(Master.JointCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Call to Master.GetJointPosition failed \""
                                << executionResult << "\"" << std::endl;
    }
    vctFrm4x4 masterPosition;
    masterPosition = Master.Manipulator.ForwardKinematics(Master.JointCurrent.Position());
    masterPosition.Rotation().NormalizedSelf();
    Master.CartesianCurrent.Position().From(masterPosition);

    executionResult = Slave.GetPositionJoint(Slave.JointCurrent);
    Slave.JointCurrent.Position()[2] = Slave.JointCurrent.Position()[2] * cmn180_PI / 1000.0; // ugly hack to convert radians to degrees to meters
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Call to Slave.GetJointPosition failed \""
                                << executionResult << "\"" << std::endl;
    }
    vctFrm4x4 slavePosition;
    slavePosition = Slave.Manipulator.ForwardKinematics(Slave.JointCurrent.Position());
    slavePosition.Rotation().NormalizedSelf();
    Slave.CartesianCurrent.Position().From(slavePosition);

    // follow mode
    if (!IsClutched) {
        // computer master motion
        vctFrm4x4 masterCartesianMotion;
        // Master.CartesianPrevious.ApplyInverseTo(masterPosition, masterCartesianMotion);
        masterCartesianMotion = masterPosition - Master.CartesianPrevious;
        // masterCartesianMotion = Master.CartesianPrevious.Inverse() * masterPosition;
        // compute desired slave motion
        vctFrm4x4 slaveCartesianMotion;
        slaveCartesianMotion.Translation().ProductOf(masterCartesianMotion.Translation(), this->Scale);
        slaveCartesianMotion.Rotation().Assign(masterCartesianMotion.Rotation());
        // compute desired slave position
        vctFrm4x4 slaveCartesianDesired;
        slaveCartesianMotion.ApplyTo(Slave.CartesianPrevious, slaveCartesianDesired);
        vctDoubleVec slaveJointDesired(Slave.JointCurrent.Position());
        slaveJointDesired.resize(6);
        Slave.Manipulator.InverseKinematics(slaveJointDesired, slaveCartesianDesired);
        slaveJointDesired[2] = slaveJointDesired[2] / cmn180_PI * 1000.0;
        slaveJointDesired.resize(7);
        slaveJointDesired.Element(6) = 0.5;
        // apply desired slave position
        Slave.JointDesired.Goal().ForceAssign(slaveJointDesired);
        Slave.SetPositionJoint(Slave.JointDesired);

//        std::cerr << "=== master cartesian motion ===" << std::endl;
//        std::cerr << masterCartesianMotion << std::endl;
//        std::cerr << "=== slave cartesian motion ===" << std::endl;
//        std::cerr << slaveCartesianMotion << std::endl;

//        std::cerr << "=== slave joint current ===" << std::endl;
//        std::cerr << Slave.JointCurrent.Position() << std::endl;
//        std::cerr << "=== slave joint desired ===" << std::endl;
//        std::cerr << slaveJointDesired << std::endl;

//        std::cerr << "=== slave cart position ===" << std::endl;
//        std::cerr << slavePosition << std::endl;

//        std::cerr << "=== slave cart desired ===" << std::endl;
//        slaveJointDesired[2] = slaveJointDesired[2] * cmn180_PI / 1000.0;
//        std::cerr << Slave.Manipulator.ForwardKinematics(slaveJointDesired) << std::endl;

//        std::cerr << "=== slave cart desired ===" << std::endl;
//        std::cerr << slavePosition - Slave.Manipulator.ForwardKinematics(slaveJointDesired) << std::endl;

//        //        std::cerr << "=== slave joint diff ===" << std::endl;
//        //        std::cerr << slaveJointDesired - Slave.JointCurrent.Position() << std::endl;

//        std::cerr << "==============================" << std::endl;
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
    } else {
        this->IsClutched = false;

        mtsExecutionResult executionResult;
        executionResult = Master.GetPositionJoint(Master.JointCurrent);
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_ERROR << "Call to Master.GetJointPosition failed \""
                                    << executionResult << "\"" << std::endl;
        }
        vctFrm4x4 masterPosition;
        masterPosition = Master.Manipulator.ForwardKinematics(Master.JointCurrent.Position());
        masterPosition.Rotation().NormalizedSelf();
        Master.CartesianCurrent.Position().From(masterPosition);

        executionResult = Slave.GetPositionJoint(Slave.JointCurrent);
        Slave.JointCurrent.Position()[2] = Slave.JointCurrent.Position()[2] * cmn180_PI / 1000.0; // ugly hack to convert radians to degrees to meters
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_ERROR << "Call to Slave.GetJointPosition failed \""
                                    << executionResult << "\"" << std::endl;
        }
        vctFrm4x4 slavePosition;
        slavePosition = Slave.Manipulator.ForwardKinematics(Slave.JointCurrent.Position());
        slavePosition.Rotation().NormalizedSelf();
        Slave.CartesianCurrent.Position().From(slavePosition);

        Master.CartesianPrevious.Assign(masterPosition);
        Slave.CartesianPrevious.Assign(slavePosition);
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
