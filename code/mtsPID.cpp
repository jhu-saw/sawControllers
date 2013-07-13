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

#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstCommonXML.h>
#include <sawControllers/mtsPID.h>


CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsPID, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);


mtsPID::mtsPID(const std::string &taskname, double period):
    mtsTaskPeriodic(taskname, period),
    CheckJointLimit(true),
    Enabled(false)
{
}


mtsPID::mtsPID(const mtsTaskPeriodicConstructorArg &arg):
    mtsTaskPeriodic(arg),
    CheckJointLimit(true),
    Enabled(false)
{
}


void mtsPID::SetupInterfaces(void)
{
    // require RobotJointTorque interface
    mtsInterfaceRequired * req = AddInterfaceRequired("RobotJointTorqueInterface");
    if (req) {
        req->AddFunction("GetJointType", Robot.GetJointType);
        req->AddFunction("GetPositionJoint", Robot.GetFeedbackPosition);
        req->AddFunction("GetVelocityJoint", Robot.GetFeedbackVelocity, MTS_OPTIONAL);
        req->AddFunction("SetTorqueJoint", Robot.SetTorque);
    }

    // this should go in "write" state table
    StateTable.AddData(Torque, "RequestedTorque");
    // this should go in a "read" state table
    StateTable.AddData(FeedbackPositionParam, "prmFeedbackPos");
    // this should go in a configuration state table with occasional start/advance
    StateTable.AddData(JointType, "jointType");
    StateTable.AddData(Kp, "Kp");
    StateTable.AddData(Kd, "Kd");
    StateTable.AddData(Ki, "Ki");
    StateTable.AddData(JointLowerLimit, "JointLowerLimit");
    StateTable.AddData(JointUpperLimit, "JointUpperLimit");
    StateTable.AddData(CheckJointLimit, "IsCheckJointLimit");

    // provide SetDesiredPositions
    mtsInterfaceProvided * prov = AddInterfaceProvided("Controller");
    if (prov) {
        prov->AddCommandVoid(&mtsPID::ResetController, this, "ResetController");
        prov->AddCommandWrite(&mtsPID::Enable, this, "Enable", mtsBool());
        prov->AddCommandWrite(&mtsPID::SetDesiredPositions, this, "SetPositionJoint", DesiredPositionParam);
        prov->AddCommandReadState(StateTable, FeedbackPositionParam, "GetPositionJoint");
        prov->AddCommandReadState(StateTable, Torque, "GetEffortJoint");
        prov->AddCommandReadState(StateTable, JointType, "GetJointType");

        // Get PID gains
        prov->AddCommandReadState(StateTable, Kp, "GetPGain");
        prov->AddCommandReadState(StateTable, Kd, "GetDGain");
        prov->AddCommandReadState(StateTable, Ki, "GetIGain");
        // Get joint limits
        prov->AddCommandReadState(StateTable, JointLowerLimit, "GetJointLowerLimit");
        prov->AddCommandReadState(StateTable, JointUpperLimit, "GetJointUpperLimit");
        // Set check limits
        prov->AddCommandWriteState(StateTable, CheckJointLimit, "SetCheckJointLimit");

        // Set PID gains
        prov->AddCommandWrite(&mtsPID::SetPGain, this, "SetPGain", Kp);
        prov->AddCommandWrite(&mtsPID::SetDGain, this, "SetDGain", Kd);
        prov->AddCommandWrite(&mtsPID::SetIGain, this, "SetIGain", Ki);
        // Set joint limits
        prov->AddCommandWrite(&mtsPID::SetJointLowerLimit, this, "SetJointLowerLimit", JointLowerLimit);
        prov->AddCommandWrite(&mtsPID::SetJointUpperLimit, this, "SetJointUpperLimit", JointUpperLimit);

        // Events
        prov->AddEventVoid(this->EventErrorLimit, "EventErrorLimit");
    }
}

void mtsPID::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: using " << filename << std::endl;
    cmnXMLPath config;
    config.SetInputSource(filename);

    // check type, interface and number of joints
    std::string type, interface;
    int numJoints = -1;
    config.GetXMLValue("/controller", "@type", type, "");
    config.GetXMLValue("/controller", "@interface", interface, "");
    config.GetXMLValue("/controller", "@numofjoints", numJoints, -1);
    if (type != "PID") {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: wrong controller type" << std::endl;
        return;
    } else if (interface != "JointTorqueInterface") {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: wrong interface. Require JointTorqueInterface" << std::endl;
        return;
    } else if (numJoints < 0) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: invalid number of joints" << std::endl;
        return;
    }

    // set dynamic var size
    Kp.SetSize(numJoints);
    Kd.SetSize(numJoints);
    Ki.SetSize(numJoints);
    Offset.SetSize(numJoints);
    JointLowerLimit.SetSize(numJoints);
    JointLowerLimit.SetAll(0.0);
    JointUpperLimit.SetSize(numJoints);
    JointUpperLimit.SetAll(0.0);

    // feedback
    FeedbackPosition.SetSize(numJoints);
    DesiredPosition.SetSize(numJoints);
    FeedbackVelocity.SetSize(numJoints);
    DesiredVelocity.SetSize(numJoints);
    Torque.SetSize(numJoints);
    Torque.SetAll(0.0);

    FeedbackPositionParam.SetSize(numJoints);
    DesiredPositionParam.SetSize(numJoints);
    FeedbackVelocityParam.SetSize(numJoints);
    TorqueParam.SetSize(numJoints);

    // errors
    Error.SetSize(numJoints);
    dError.SetSize(numJoints);
    iError.SetSize(numJoints);
    oldError.SetSize(numJoints);
    oldDesiredPos.SetSize(numJoints);
    ResetController();

    minIErrorLimit.SetSize(numJoints);
    minIErrorLimit.SetAll(-100.0);
    maxIErrorLimit.SetSize(numJoints);
    maxIErrorLimit.SetAll(100.0);
    errorLimit.SetSize(numJoints);
    errorLimit.SetAll(100.0);

    // default 1.0: no effect
    forgetIError.SetSize(numJoints);
    forgetIError.SetAll(1.0);

    DeadBand.SetSize(numJoints);
    DeadBand.SetAll(0.0);

    // read data from xml file
    char context[64];
    for (int i = 0; i < numJoints; i++) {
        // joint
        sprintf(context, "controller/joints/joint[%d]", i+1);
        // pid
        config.GetXMLValue(context, "pid/@PGain", Kp[i]);
        config.GetXMLValue(context, "pid/@DGain", Kd[i]);
        config.GetXMLValue(context, "pid/@IGain", Ki[i]);
        config.GetXMLValue(context, "pid/@OffsetTorque", Offset[i]);
        config.GetXMLValue(context, "pid/@Forget", forgetIError[i]);

        // limit
        config.GetXMLValue(context, "limit/@MinILimit", minIErrorLimit[i]);
        config.GetXMLValue(context, "limit/@MaxILimit", maxIErrorLimit[i]);
        config.GetXMLValue(context, "limit/@ErrorLimit", errorLimit[i]);
        config.GetXMLValue(context, "limit/@Deadband", DeadBand[i]);

        // joint limit
        CheckJointLimit = true;
        std::string tmpUnits;
        bool ret = false;
        ret = config.GetXMLValue(context, "pos/@Units", tmpUnits);
        if (ret){
            config.GetXMLValue(context, "pos/@LowerLimit", JointLowerLimit[i]);
            config.GetXMLValue(context, "pos/@UpperLimit", JointUpperLimit[i]);
            if(tmpUnits == "deg"){
                JointLowerLimit[i] *= cmnPI_180;
                JointUpperLimit[i] *= cmnPI_180;
            }else if (tmpUnits == "mm"){
                JointLowerLimit[i] *= cmn_mm;
                JointUpperLimit[i] *= cmn_mm;
            }
        }else{
            CheckJointLimit = false;
        }
    }


    // Convert from degrees to radians
    // TODO: Decide whether to use degrees or radians in XML file
    // TODO: Also do this for other parameters (not just Deadband)
    // TODO: Only do this for revolute joints
    DeadBand.Multiply(cmnPI_180);

    CMN_LOG_CLASS_INIT_VERBOSE << "Kp: " << Kp << std::endl;
    CMN_LOG_CLASS_INIT_VERBOSE << "Kd: " << Kd << std::endl;
    CMN_LOG_CLASS_INIT_VERBOSE << "Ki: " << Ki << std::endl;
    CMN_LOG_CLASS_INIT_VERBOSE << "Offset: " << Offset << std::endl;
    CMN_LOG_CLASS_INIT_VERBOSE << "JointLowerLimit" << JointLowerLimit << std::endl;
    CMN_LOG_CLASS_INIT_VERBOSE << "JointUpperLimit" << JointUpperLimit << std::endl;
    CMN_LOG_CLASS_INIT_VERBOSE << "Deadband: " << DeadBand << std::endl;
    CMN_LOG_CLASS_INIT_VERBOSE << "minLimit: " << minIErrorLimit << std::endl;
    CMN_LOG_CLASS_INIT_VERBOSE << "maxLimit: " << maxIErrorLimit << std::endl;
    CMN_LOG_CLASS_INIT_VERBOSE << "elimit: " << errorLimit << std::endl;
    CMN_LOG_CLASS_INIT_VERBOSE << "forget: " << forgetIError << std::endl;

    // now that we know the sizes of vectors, create interfaces
    this->SetupInterfaces();
}

void mtsPID::Startup(void)
{
    // startup
    mtsExecutionResult result;
    result = Robot.GetJointType(JointType);
    if (!result) {
        CMN_LOG_CLASS_INIT_ERROR << "Startup: Robot interface isn't connected properly, unable to get joint type.  Function call returned: "
                                 << result << std::endl;
    }
}

void mtsPID::Run(void)
{
    ProcessQueuedEvents();
    ProcessQueuedCommands();

    Robot.GetFeedbackPosition(FeedbackPositionParam);
    FeedbackPositionParam.GetPosition(FeedbackPosition);

    // compute torque
    if (Enabled) {
        // compute error
        Error.DifferenceOf(DesiredPosition, FeedbackPosition);
        size_t i;
        for (i = 0; i < Error.size(); i++) {
            if ((Error[i] <= DeadBand[i]) && (Error[i] >= -DeadBand[i]))
                Error[i] = 0.0;
        }

        // compute error derivative
        if (Robot.GetFeedbackVelocity.IsValid()) {
            Robot.GetFeedbackVelocity(FeedbackVelocityParam);
            FeedbackVelocityParam.GetVelocity(FeedbackVelocity);
            dError.DifferenceOf(DesiredVelocity, FeedbackVelocity);
        } else {
            // ZC: TODO add dError filtering
            // compute error derivative
            double dt = StateTable.Period;
            if (dt > 0) {
                dError.DifferenceOf(Error, oldError);
                dError.Divide(dt);
            }
            else {
                dError.SetAll(0.0);
            }
        }

        // compute error integral
        iError.ElementwiseMultiply(forgetIError);
        iError.Add(Error);

        // check error limit & clamp iError
        bool isOutOfLimit = false;
        for (i = 0; i < iError.size(); i++) {
            // error limit
            if (fabs(Error[i]) > errorLimit[i])
                isOutOfLimit = true;

            // iError clamping
            if (iError.at(i) > maxIErrorLimit.at(i))
                iError.at(i) = maxIErrorLimit.at(i);
            else if (iError.at(i) < minIErrorLimit.at(i))
                iError.at(i) = minIErrorLimit[i];
        }

        // send EventErrorLimit
        if (isOutOfLimit) EventErrorLimit();

        // save Error to oldError
        oldError.Assign(Error);
        oldDesiredPos.Assign(DesiredPosition);

        // compute torque
        Torque.ElementwiseProductOf(Kp, Error);
        Torque.AddElementwiseProductOf(Kd, dError);
        Torque.AddElementwiseProductOf(Ki, iError);
        Torque.Add(Offset);

        // write torque to robot
        TorqueParam.SetForceTorque(Torque);
        Robot.SetTorque(TorqueParam);

    } else {
        // set torque to 0
//        torque.SetAll(0.0);
//        std::cerr << "disable " << StateTable.GetIndexReader().Index() << std::endl;
    }

}


void mtsPID::Cleanup(void)
{
    // cleanup
}


void mtsPID::SetPGain(const vctDoubleVec & pgain)
{
    if (pgain.size() != Kp.size()) {
        CMN_LOG_CLASS_INIT_ERROR << "SetPGain: size mismatch" << std::endl;
    } else {
        Kp.Assign(pgain);
    }
}

void mtsPID::SetDGain(const vctDoubleVec & dgain)
{
    if (dgain.size() != Kd.size()) {
        CMN_LOG_CLASS_INIT_ERROR << "SetDGain: size mismatch" << std::endl;
    } else {
        Kd.Assign(dgain);
    }
}

void mtsPID::SetIGain(const vctDoubleVec & igain)
{
    if (igain.size() != Ki.size()) {
        CMN_LOG_CLASS_INIT_ERROR << "SetIGain: size mismatch" << std::endl;
    } else {
        Ki.Assign(igain);
    }
}

void mtsPID::SetJointLowerLimit(const vctDoubleVec &lowerLimit)
{
    if (lowerLimit.size() != JointLowerLimit.size()) {
        CMN_LOG_CLASS_INIT_ERROR << "SetJointLowerLimit: size mismatch" << std::endl;
    } else {
        JointLowerLimit.Assign(lowerLimit);
    }
}

void mtsPID::SetJointUpperLimit(const vctDoubleVec &upperLimit)
{
    if (upperLimit.size() != JointUpperLimit.size()) {
        CMN_LOG_CLASS_INIT_ERROR << "SetJointUpperLimit: size mismatch" << std::endl;
    } else {
        JointUpperLimit.Assign(upperLimit);
    }
}


void mtsPID::SetMinIErrorLimit(const vctDoubleVec & iminlim)
{
    minIErrorLimit = iminlim;
}

void mtsPID::SetMaxIErrorLimit(const vctDoubleVec & imaxlim)
{
    maxIErrorLimit = imaxlim;
}

void mtsPID::SetForgetIError(const double & forget)
{
    forgetIError = forget;
}


//------------- Protected Method ---------------

void mtsPID::ResetController(void)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "Reset Controller" << std::endl;

    Error.SetAll(0.0);
    oldError.SetAll(0.0);
    dError.SetAll(0.0);
    iError.SetAll(0.0);

    prmPositionJointSet setPrmPos;
    setPrmPos.SetSize(Error.size());
    setPrmPos.SetGoal(FeedbackPosition);
    SetDesiredPositions(setPrmPos);

    DesiredVelocity.SetAll(0.0);
}

void mtsPID::SetDesiredPositions(const prmPositionJointSet & positionParam)
{
    DesiredPositionParam = positionParam;
    DesiredPositionParam.GetGoal(DesiredPosition);

    if (CheckJointLimit) {
        // limit check: clip the desired position
        DesiredPosition.ElementwiseMin(JointUpperLimit);
        DesiredPosition.ElementwiseMax(JointLowerLimit);
    }

    double dt = StateTable.Period;
    DesiredVelocity = (DesiredPosition - oldDesiredPos) / dt;
}


void mtsPID::Enable(const mtsBool & enable)
{
    Enabled = enable.Data;

    // set torque to 0
    Torque.SetAll(0.0);

    // write torque to robot
    TorqueParam.SetForceTorque(Torque);
    Robot.SetTorque(TorqueParam);
}
