/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Zihan Chen, Anton Deguet
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

mtsTeleOperation::mtsTeleOperation(const std::string & componentName, const double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds),
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

    this->StateTable.AddData(Master.PositionCartesianCurrent, "MasterCartesianPosition");
    this->StateTable.AddData(Slave.PositionCartesianCurrent, "SlaveCartesianPosition");

    // Setup CISST Interface
    mtsInterfaceRequired * req;
    req = AddInterfaceRequired("Master");
    if (req) {
        req->AddFunction("GetPositionCartesian", Master.GetPositionCartesian);
        req->AddFunction("SetPositionCartesian", Master.SetPositionCartesian);
    }

    req = AddInterfaceRequired("Slave");
    if (req) {
        req->AddFunction("GetPositionCartesian", Slave.GetPositionCartesian);
        req->AddFunction("SetPositionCartesian", Slave.SetPositionCartesian);
    }

    req = AddInterfaceRequired("Clutch");
    if (req) {
        req->AddEventHandlerWrite(&mtsTeleOperation::EventHandlerClutched, this, "Button");
    }

    mtsInterfaceProvided * prov = AddInterfaceProvided("Setting");
    if (prov) {
        prov->AddCommandWrite(&mtsTeleOperation::SetScale, this, "SetScale", mtsDouble());
        prov->AddCommandWrite(&mtsTeleOperation::SetRegistrationRotation, this,
                              "SetRegistrationRotation", vctMatRot3());

        prov->AddCommandVoid(&mtsTeleOperation::AllignMasterToSlave, this, "AllignMasterToSlave");
        prov->AddCommandReadState(this->StateTable, Master.PositionCartesianCurrent, "GetPositionCartesianMaster");
        prov->AddCommandReadState(this->StateTable, Slave.PositionCartesianCurrent, "GetPositionCartesianSlave");
    }
}

void mtsTeleOperation::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsTeleOperation::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsPIDQt::Startup" << std::endl;
}

void mtsTeleOperation::Run(void)
{
    ProcessQueuedCommands();
    ProcessQueuedEvents();

    mtsExecutionResult executionResult;
    executionResult = Master.GetPositionCartesian(Master.PositionCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Call to Master.GetPositionCartesian failed \""
                                << executionResult << "\"" << std::endl;
    }
    vctFrm4x4 masterPosition(Master.PositionCartesianCurrent.Position());

    executionResult = Slave.GetPositionCartesian(Slave.PositionCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Call to Slave.GetPositionCartesian failed \""
                                << executionResult << "\"" << std::endl;
    }
    vctFrm4x4 slavePosition(Slave.PositionCartesianCurrent.Position());

    // follow mode
    if (!IsClutched) {
        // computer master motion
        vctFrm4x4 masterCartesianMotion;
        masterCartesianMotion = Master.CartesianPrevious.Inverse() * masterPosition;
        // compute desired slave motion
        vctFrm4x4 slaveCartesianMotion;

        vct3 zcTranslation;
        zcTranslation = (masterPosition.Translation() - Master.CartesianPrevious.Translation()) * this->Scale;


        vctAxAnRot3 masterRotation;
        masterRotation.FromNormalized(masterCartesianMotion.Rotation());
        vctFrm4x4 masterWRTslave;
        masterWRTslave = Slave.CartesianPrevious.Inverse() * Master.CartesianPrevious;
//        Slave.CartesianPrevious.ApplyInverseTo(Master.CartesianPrevious, masterWRTslave);

        vct3 slaveRotationAxis;
        slaveRotationAxis = masterWRTslave.Rotation() * masterRotation.Axis();
        vctMatRot3 slaveRotation(vctAxAnRot3(slaveRotationAxis, masterRotation.Angle()));


        // compute desired slave position
        vctFrm4x4 slaveCartesianDesired;

        // slaveCartesianDesired = slaveCartesianPrevious * slaveCartesianMotion
        // output = motion * input -> motion.ApplyTo(input, output)
        slaveRotation = Slave.CartesianPrevious.Rotation() * slaveRotation;

        slaveCartesianDesired.Rotation().FromNormalized(slaveRotation);

        // Translation
        slaveCartesianDesired.Translation() = Slave.CartesianPrevious.Translation() + zcTranslation;


        // ZC: I think this is wrong
        // apply desired slave position
        Slave.PositionCartesianDesired.Goal().FromNormalized(slaveCartesianDesired);

        // Slave go this cartesian position
        Slave.SetPositionCartesian(Slave.PositionCartesianDesired);
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
        executionResult = Master.GetPositionCartesian(Master.PositionCartesianCurrent);
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_ERROR << "Call to Master.GetPositionCartesian failed \""
                                    << executionResult << "\"" << std::endl;
        }
        executionResult = Slave.GetPositionCartesian(Slave.PositionCartesianCurrent);
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_ERROR << "Call to Slave.GetPositionCartesian failed \""
                                    << executionResult << "\"" << std::endl;
        }
        Master.CartesianPrevious.From(Master.PositionCartesianCurrent.Position());
        Slave.CartesianPrevious.From(Slave.PositionCartesianCurrent.Position());
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
