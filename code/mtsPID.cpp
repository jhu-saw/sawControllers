/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen
  Created on: 2013-02-22

  (C) Copyright 2013-2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstCommon/cmnXMLPath.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

#include <sawControllers/mtsPID.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsPID, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);


mtsPID::mtsPID(const std::string &componentName, double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds),
    Counter(0),
    CheckJointLimit(true),
    Enabled(false),
    ConfigurationStateTable(100, "Configuration")
{
    AddStateTable(&ConfigurationStateTable);
    ConfigurationStateTable.SetAutomaticAdvance(false);
}


mtsPID::mtsPID(const mtsTaskPeriodicConstructorArg &arg):
    mtsTaskPeriodic(arg),
    Counter(0),
    CheckJointLimit(true),
    Enabled(false),
    ConfigurationStateTable(100, "Configuration")
{
    AddStateTable(&ConfigurationStateTable);
    ConfigurationStateTable.SetAutomaticAdvance(false);
}


void mtsPID::SetupInterfaces(void)
{
    // require RobotJointTorque interface
    mtsInterfaceRequired * requiredInterface = AddInterfaceRequired("RobotJointTorqueInterface");
    if (requiredInterface) {
        requiredInterface->AddFunction("GetJointType", Robot.GetJointType);
        requiredInterface->AddFunction("GetPositionJoint", Robot.GetFeedbackPosition);
        requiredInterface->AddFunction("GetTorqueJoint", Robot.GetFeedbackTorque);
        requiredInterface->AddFunction("GetVelocityJoint", Robot.GetFeedbackVelocity, MTS_OPTIONAL);
        requiredInterface->AddFunction("SetTorqueJoint", Robot.SetTorque);
        // event handlers
        requiredInterface->AddEventHandlerWrite(&mtsPID::ErrorEventHandler, this, "Error");
    }

    // this should go in "write" state table
    StateTable.AddData(Torque, "RequestedTorque");
    // this should go in a "read" state table
    StateTable.AddData(FeedbackPositionParam, "prmFeedbackPos");
    StateTable.AddData(FeedbackVelocityParam, "prmFeedbackVel");
    StateTable.AddData(FeedbackTorque, "FeedbackTorque");
    StateTable.AddData(DesiredPosition, "DesiredPosition");
    StateTable.AddData(CheckJointLimit, "IsCheckJointLimit");
    StateTable.AddData(Offset, "TorqueOffset");
    StateTable.AddData(mStateJoint, "StateJoint");
    StateTable.AddData(mStateJointDesired, "DesiredStateJoint");

    // configuration state table with occasional start/advance
    ConfigurationStateTable.AddData(Kp, "Kp");
    ConfigurationStateTable.AddData(Kd, "Kd");
    ConfigurationStateTable.AddData(Ki, "Ki");
    ConfigurationStateTable.AddData(JointLowerLimit, "JointLowerLimit");
    ConfigurationStateTable.AddData(JointUpperLimit, "JointUpperLimit");
    ConfigurationStateTable.AddData(JointType, "JointType");
    ConfigurationStateTable.AddData(mEnableTrackingError, "EnableTrackingError");
    ConfigurationStateTable.AddData(mTrackingErrorTolerances, "TrackingErrorTolerances");

    // provide SetDesiredPositions
    mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("Controller");
    if (interfaceProvided) {
        interfaceProvided->AddCommandVoid(&mtsPID::ResetController, this, "ResetController");
        interfaceProvided->AddCommandWrite(&mtsPID::Enable, this, "Enable", false);
        interfaceProvided->AddCommandWrite(&mtsPID::EnableTorqueMode, this, "EnableTorqueMode", TorqueMode);
        interfaceProvided->AddCommandWrite(&mtsPID::SetDesiredPositions, this, "SetPositionJoint", DesiredPositionParam);
        interfaceProvided->AddCommandWrite(&mtsPID::SetDesiredTorques, this, "SetTorqueJoint", prmDesiredTrq);
        interfaceProvided->AddCommandReadState(StateTable, FeedbackPositionParam, "GetPositionJoint");
        interfaceProvided->AddCommandReadState(StateTable, FeedbackVelocityParam, "GetVelocityJoint");
        interfaceProvided->AddCommandReadState(StateTable, FeedbackTorque, "GetTorqueJoint");
        interfaceProvided->AddCommandReadState(StateTable, DesiredPosition, "GetPositionJointDesired");
        interfaceProvided->AddCommandReadState(StateTable, Torque, "GetEffortJointDesired");
        // ROS compatible joint state
        interfaceProvided->AddCommandReadState(StateTable, mStateJoint, "GetStateJoint");
        interfaceProvided->AddCommandReadState(StateTable, mStateJointDesired, "GetStateJointDesired");

        // Set check limits
        interfaceProvided->AddCommandWriteState(StateTable, CheckJointLimit, "SetCheckJointLimit");
        interfaceProvided->AddCommandWriteState(StateTable, Offset, "SetTorqueOffset");

        // Get PID gains
        interfaceProvided->AddCommandReadState(ConfigurationStateTable, Kp, "GetPGain");
        interfaceProvided->AddCommandReadState(ConfigurationStateTable, Kd, "GetDGain");
        interfaceProvided->AddCommandReadState(ConfigurationStateTable, Ki, "GetIGain");
        // Get joint limits
        interfaceProvided->AddCommandReadState(ConfigurationStateTable, JointLowerLimit, "GetJointLowerLimit");
        interfaceProvided->AddCommandReadState(ConfigurationStateTable, JointUpperLimit, "GetJointUpperLimit");
        interfaceProvided->AddCommandReadState(ConfigurationStateTable, JointType, "GetJointType");

        // Error tracking
        interfaceProvided->AddCommandWriteState(ConfigurationStateTable, mEnableTrackingError, "EnableTrackingError");
        interfaceProvided->AddCommandWrite(&mtsPID::SetTrackingErrorTolerances, this, "SetTrackingErrorTolerances");

        // Set PID gains
        interfaceProvided->AddCommandWrite(&mtsPID::SetPGain, this, "SetPGain", Kp);
        interfaceProvided->AddCommandWrite(&mtsPID::SetDGain, this, "SetDGain", Kd);
        interfaceProvided->AddCommandWrite(&mtsPID::SetIGain, this, "SetIGain", Ki);
        // Set joint limits
        interfaceProvided->AddCommandWrite(&mtsPID::SetJointLowerLimit, this, "SetJointLowerLimit", JointLowerLimit);
        interfaceProvided->AddCommandWrite(&mtsPID::SetJointUpperLimit, this, "SetJointUpperLimit", JointUpperLimit);

        // Events
        interfaceProvided->AddEventWrite(Events.Enabled, "Enabled", false);
        interfaceProvided->AddEventWrite(Events.JointLimit, "JointLimit", vctBoolVec());
        interfaceProvided->AddEventWrite(MessageEvents.Status, "Status", std::string(""));
        interfaceProvided->AddEventWrite(MessageEvents.Warning, "Warning", std::string(""));
        interfaceProvided->AddEventWrite(MessageEvents.Error, "Error", std::string(""));
    }
}

void mtsPID::Configure(const std::string & filename)
{
    ConfigurationStateTable.Start();

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
        ConfigurationStateTable.Advance();
        return;
    } else if (interface != "JointTorqueInterface") {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: wrong interface. Require JointTorqueInterface" << std::endl;
        ConfigurationStateTable.Advance();
        return;
    } else if (numJoints < 0) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: invalid number of joints" << std::endl;
        ConfigurationStateTable.Advance();
        return;
    }

    // set dynamic var size
    Kp.SetSize(numJoints);
    Kd.SetSize(numJoints);
    Ki.SetSize(numJoints);
    Offset.SetSize(numJoints);
    Offset.SetAll(0.0);
    JointLowerLimit.SetSize(numJoints);
    JointLowerLimit.SetAll(0.0);
    JointUpperLimit.SetSize(numJoints);
    JointUpperLimit.SetAll(0.0);
    mJointLimitFlag.SetSize(numJoints);
    mJointLimitFlag.SetAll(false);
    mPreviousJointLimitFlag.ForceAssign(mJointLimitFlag);

    // feedback
    FeedbackPosition.SetSize(numJoints);
    DesiredPosition.SetSize(numJoints);
    FeedbackTorque.SetSize(numJoints);
    DesiredTorque.SetSize(numJoints);
    DesiredTorque.SetAll(0.0);
    FeedbackVelocity.SetSize(numJoints);
    DesiredVelocity.SetSize(numJoints);
    Torque.SetSize(numJoints);
    Torque.SetAll(0.0);
    FeedbackPositionParam.SetSize(numJoints);
    FeedbackPositionPreviousParam.SetSize(numJoints);
    DesiredPositionParam.SetSize(numJoints);
    DesiredPositionParam.Goal().SetAll(0.0);
    FeedbackVelocityParam.SetSize(numJoints);
    TorqueParam.SetSize(numJoints);

    // errors
    Error.SetSize(numJoints);
    ErrorAbsolute.SetSize(numJoints);
    dError.SetSize(numJoints);
    iError.SetSize(numJoints);
    oldError.SetSize(numJoints);
    oldDesiredPos.SetSize(numJoints);
    ResetController();

    minIErrorLimit.SetSize(numJoints);
    minIErrorLimit.SetAll(-100.0);
    maxIErrorLimit.SetSize(numJoints);
    maxIErrorLimit.SetAll(100.0);

    // default 1.0: no effect
    forgetIError.SetSize(numJoints);
    forgetIError.SetAll(1.0);

    // default: use regular PID
    nonlinear.SetSize(numJoints);
    nonlinear.SetAll(0.0);

    DeadBand.SetSize(numJoints);
    DeadBand.SetAll(0.0);

    // torque mode
    TorqueMode.SetSize(numJoints);
    TorqueMode.SetAll(false);

    // tracking error
    mEnableTrackingError = false;
    mTrackingErrorTolerances.SetSize(numJoints);
    mTrackingErrorTolerances.SetAll(0.0);
    mTrackingErrorFlag.SetSize(numJoints);
    mTrackingErrorFlag.SetAll(false);
    mPreviousTrackingErrorFlag.ForceAssign(mTrackingErrorFlag);

    // names in joint states
    mStateJoint.Name().SetSize(numJoints);
    mStateJointDesired.Name().SetSize(numJoints);

    // read data from xml file
    char context[64];
    for (int i = 0; i < numJoints; i++) {
        // joint
        sprintf(context, "controller/joints/joint[%d]", i+1);

        // name
        std::string name;
        config.GetXMLValue(context, "@name", name);
        mStateJoint.Name().at(i) = name;
        mStateJointDesired.Name().at(i) = name;

        // pid
        config.GetXMLValue(context, "pid/@PGain", Kp[i]);
        config.GetXMLValue(context, "pid/@DGain", Kd[i]);
        config.GetXMLValue(context, "pid/@IGain", Ki[i]);
        config.GetXMLValue(context, "pid/@OffsetTorque", Offset[i]);
        config.GetXMLValue(context, "pid/@Forget", forgetIError[i]);
        config.GetXMLValue(context, "pid/@Nonlinear", nonlinear[i]);

        // limit
        config.GetXMLValue(context, "limit/@MinILimit", minIErrorLimit[i]);
        config.GetXMLValue(context, "limit/@MaxILimit", maxIErrorLimit[i]);
        config.GetXMLValue(context, "limit/@ErrorLimit", mTrackingErrorTolerances[i]);
        config.GetXMLValue(context, "limit/@Deadband", DeadBand[i]);

        // joint limit
        CheckJointLimit = true;
        std::string tmpUnits;
        bool ret = false;
        ret = config.GetXMLValue(context, "pos/@Units", tmpUnits);
        if (ret) {
            config.GetXMLValue(context, "pos/@LowerLimit", JointLowerLimit[i]);
            config.GetXMLValue(context, "pos/@UpperLimit", JointUpperLimit[i]);
            if (tmpUnits == "deg") {
                JointLowerLimit[i] *= cmnPI_180;
                JointUpperLimit[i] *= cmnPI_180;
            } else if (tmpUnits == "mm") {
                JointLowerLimit[i] *= cmn_mm;
                JointUpperLimit[i] *= cmn_mm;
            }
        } else {
            CheckJointLimit = false;
        }
    }

    // Convert from degrees to radians
    // TODO: Decide whether to use degrees or radians in XML file
    // TODO: Also do this for other parameters (not just Deadband)
    // TODO: Only do this for revolute joints
    DeadBand.Multiply(cmnPI_180);

    CMN_LOG_CLASS_INIT_VERBOSE << "Kp: " << Kp << std::endl
                               << "Kd: " << Kd << std::endl
                               << "Ki: " << Ki << std::endl
                               << "Offset: " << Offset << std::endl
                               << "JntLowerLimit" << JointLowerLimit << std::endl
                               << "JntUpperLimit" << JointUpperLimit << std::endl
                               << "Deadband: " << DeadBand << std::endl
                               << "minLimit: " << minIErrorLimit << std::endl
                               << "maxLimit: " << maxIErrorLimit << std::endl
                               << "elimit: " << mTrackingErrorTolerances << std::endl
                               << "forget: " << forgetIError << std::endl;

    ConfigurationStateTable.Advance();

    // now that we know the sizes of vectors, create interfaces
    this->SetupInterfaces();
}

void mtsPID::Startup(void)
{
    // get joint type and store in configuration state table
    mtsExecutionResult result;
    ConfigurationStateTable.Start();
    result = Robot.GetJointType(JointType);
    ConfigurationStateTable.Advance();
    if (!result) {
        CMN_LOG_CLASS_INIT_ERROR << "Startup: Robot interface isn't connected properly, unable to get joint type.  Function call returned: "
                                 << result << std::endl;
    }
}

void mtsPID::Run(void)
{
    ProcessQueuedEvents();
    ProcessQueuedCommands();

    // increment counter
    Counter++;

    // update states
    Robot.GetFeedbackPosition(FeedbackPositionParam);
    FeedbackPositionParam.GetPosition(FeedbackPosition);
    Robot.GetFeedbackTorque(FeedbackTorque);

    // compute error
    Error.DifferenceOf(DesiredPosition, FeedbackPosition);
    for (size_t i = 0; i < Error.size(); i++) {
        if ((Error[i] <= DeadBand[i]) && (Error[i] >= -DeadBand[i]))
            Error[i] = 0.0;
    }

    // update velocities from robot
    if (Robot.GetFeedbackVelocity.IsValid()) {
        Robot.GetFeedbackVelocity(FeedbackVelocityParam);
        FeedbackVelocityParam.GetVelocity(FeedbackVelocity);
    } else {
        // or compute an estimate from position
        double dt = FeedbackPositionParam.Timestamp() - FeedbackPositionPreviousParam.Timestamp();
        if (dt > 0) {
            FeedbackVelocity.DifferenceOf(FeedbackPositionParam.Position(),
                                          FeedbackPositionPreviousParam.Position());
            FeedbackVelocity.Divide(dt);
        } else {
            FeedbackVelocity.SetAll(0.0);
        }
        // set param so data in state table is accurate
        FeedbackVelocityParam.SetVelocity(FeedbackVelocity);
    }

    // update ROS message
    mStateJoint.Position().ForceAssign(FeedbackPosition);
    mStateJoint.Velocity().ForceAssign(FeedbackVelocity);
    mStateJoint.Effort().ForceAssign(FeedbackTorque);

    // compute torque
    if (Enabled) {

        // check for tracking error
        if (mEnableTrackingError) {
            vctDoubleVec::const_iterator error = Error.begin();
            vctDoubleVec::const_iterator tolerance = mTrackingErrorTolerances.begin();
            vctBoolVec::iterator limitFlag = mJointLimitFlag.begin();
            vctBoolVec::iterator trackingErrorFlag = mTrackingErrorFlag.begin();
            vctBoolVec::iterator previousTrackingErrorFlag = mPreviousTrackingErrorFlag.begin();
            bool anyTrackingError = false;
            bool newTrackingError = false;
            const vctDoubleVec::iterator end = Error.end();

            // detect tracking errors
            for (; error != end;
                 ++error, ++tolerance, ++limitFlag, ++trackingErrorFlag, ++previousTrackingErrorFlag) {
                double errorAbsolute = fabs(*error);
                // trigger error if the error is too high AND the last request was not outside joint limit
                if ((errorAbsolute > *tolerance) && !(*limitFlag)) {
                    anyTrackingError = true;
                    *trackingErrorFlag = true;
                    if (*trackingErrorFlag != *previousTrackingErrorFlag) {
                        newTrackingError = true;
                        *previousTrackingErrorFlag = *trackingErrorFlag;
                    }
                } else {
                    *trackingErrorFlag = false;
                }
            }
            // act on errors
            if (anyTrackingError) {
                Enable(false);
                if (newTrackingError) {
                    std::string message = this->Name + ": tracking error, mask: ";
                    message.append(mTrackingErrorFlag.ToString());
                    MessageEvents.Error(message);
                    CMN_LOG_CLASS_RUN_ERROR << message
                                            << ", errors: " << Error
                                            << ", tolerances: " << mTrackingErrorTolerances
                                            << std::endl;
                }
            }
        }

        // compute error derivative
        dError.Assign(FeedbackVelocity);
        dError.Multiply(-1.0);

        // compute error integral
        iError.ElementwiseMultiply(forgetIError);
        iError.Add(Error);

        // check error limit & clamp iError
        for (size_t i = 0; i < iError.size(); i++) {
            // iError clamping
            if (iError.at(i) > maxIErrorLimit.at(i)) {
                iError.at(i) = maxIErrorLimit.at(i);
            }
            else if (iError.at(i) < minIErrorLimit.at(i)) {
                iError.at(i) = minIErrorLimit[i];
            }
        }

        // save Error to oldError
        oldError.Assign(Error);
        oldDesiredPos.Assign(DesiredPosition);

        // compute torque
        Torque.ElementwiseProductOf(Kp, Error);
        Torque.AddElementwiseProductOf(Kd, dError);
        Torque.AddElementwiseProductOf(Ki, iError);

        // nonlinear control mode
        for (size_t i = 0; i < Error.size(); i++) {
            if ((nonlinear[i] > 0)
                && (fabs(Error[i]) < nonlinear[i])) {
                Torque[i] = Torque[i] * fabs(Error[i]) / nonlinear[i];
            }
        }

        // Add torque (e.g. gravity compensation)
        Torque.Add(Offset);

        // Set Torque to DesiredTorque if
        for (size_t i = 0; i < Torque.size(); i++) {
            if (TorqueMode[i]) {
                Torque[i] = DesiredTorque[i];
            }
        }

        // write torque to robot
        TorqueParam.SetForceTorque(Torque);
        Robot.SetTorque(TorqueParam);
    }
    else {
        Torque.SetAll(0.0);
        TorqueParam.SetForceTorque(Torque);
        Robot.SetTorque(TorqueParam);
    }

    // save previous position
    FeedbackPositionPreviousParam = FeedbackPositionParam;
}


void mtsPID::Cleanup(void)
{
    // cleanup
    Torque.SetAll(0.0);
    TorqueParam.SetForceTorque(Torque);
    Robot.SetTorque(TorqueParam);
}


void mtsPID::SetPGain(const vctDoubleVec & pgain)
{
    if (pgain.size() != Kp.size()) {
        CMN_LOG_CLASS_INIT_ERROR << "SetPGain: size mismatch" << std::endl;
    } else {
        ConfigurationStateTable.Start();
        Kp.Assign(pgain);
        ConfigurationStateTable.Advance();
    }
}

void mtsPID::SetDGain(const vctDoubleVec & dgain)
{
    if (dgain.size() != Kd.size()) {
        CMN_LOG_CLASS_INIT_ERROR << "SetDGain: size mismatch" << std::endl;
    } else {
        ConfigurationStateTable.Start();
        Kd.Assign(dgain);
        ConfigurationStateTable.Advance();
    }
}

void mtsPID::SetIGain(const vctDoubleVec & igain)
{
    if (igain.size() != Ki.size()) {
        CMN_LOG_CLASS_INIT_ERROR << "SetIGain: size mismatch" << std::endl;
    } else {
        ConfigurationStateTable.Start();
        Ki.Assign(igain);
        ConfigurationStateTable.Advance();
    }
}

void mtsPID::SetJointLowerLimit(const vctDoubleVec &lowerLimit)
{
    if (lowerLimit.size() != JointLowerLimit.size()) {
        CMN_LOG_CLASS_INIT_ERROR << "SetJointLowerLimit: size mismatch" << std::endl;
    } else {
        ConfigurationStateTable.Start();
        JointLowerLimit.Assign(lowerLimit);
        ConfigurationStateTable.Advance();
    }
}

void mtsPID::SetJointUpperLimit(const vctDoubleVec &upperLimit)
{
    if (upperLimit.size() != JointUpperLimit.size()) {
        CMN_LOG_CLASS_INIT_ERROR << "SetJointUpperLimit: size mismatch" << std::endl;
    } else {
        ConfigurationStateTable.Start();
        JointUpperLimit.Assign(upperLimit);
        ConfigurationStateTable.Advance();
    }
}


void mtsPID::SetMinIErrorLimit(const vctDoubleVec & iminlim)
{
    if (iminlim.size() != minIErrorLimit.size()) {
        CMN_LOG_CLASS_INIT_ERROR << "SetMinIErrorLimit: size mismatch" << std::endl;
    } else {
        ConfigurationStateTable.Start();
        minIErrorLimit.Assign(iminlim);
        ConfigurationStateTable.Advance();
    }
}

void mtsPID::SetMaxIErrorLimit(const vctDoubleVec & imaxlim)
{
    if (imaxlim.size() != maxIErrorLimit.size()) {
        CMN_LOG_CLASS_INIT_ERROR << "SetMaxIErrorLimit: size mismatch" << std::endl;
    } else {
        ConfigurationStateTable.Start();
        maxIErrorLimit.Assign(imaxlim);
        ConfigurationStateTable.Advance();
    }
}

void mtsPID::SetForgetIError(const double & forget)
{
    forgetIError = forget;
}

void mtsPID::ResetController(void)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "Reset Controller" << std::endl;
    Error.SetAll(0.0);
    oldError.SetAll(0.0);
    dError.SetAll(0.0);
    iError.SetAll(0.0);
    Enable(false);
}

void mtsPID::SetDesiredTorques(const prmForceTorqueJointSet & prmTrq)
{
    prmDesiredTrq = prmTrq;
    prmDesiredTrq.GetForceTorque( DesiredTorque );
}

void mtsPID::SetDesiredPositions(const prmPositionJointSet & positionParam)
{
    DesiredPositionParam = positionParam;
    DesiredPositionParam.GetGoal(DesiredPosition);

    if (CheckJointLimit) {
        bool limitReached = false;
        vctDoubleVec::const_iterator upper = JointUpperLimit.begin();
        vctDoubleVec::const_iterator lower = JointLowerLimit.begin();
        vctBoolVec::iterator flags = mJointLimitFlag.begin();
        vctDoubleVec::iterator desired = DesiredPosition.begin();
        const vctDoubleVec::iterator end = DesiredPosition.end();
        for (; desired != end; ++desired, ++upper, ++lower, ++flags) {
            if (*desired > *upper) {
                limitReached = true;
                *desired = *upper;
                *flags = true;
            } else if (*desired < *lower) {
                limitReached = true;
                *desired = *lower;
                *flags = true;
            } else {
                *flags = false;
            }
        }
        if (limitReached) {
            if (mPreviousJointLimitFlag.NotEqual(mJointLimitFlag)) {
                mPreviousJointLimitFlag.Assign(mJointLimitFlag);
                Events.JointLimit(mJointLimitFlag);
                std::string message = this->Name + ": joint limit, mask: ";
                message.append(mJointLimitFlag.ToString());
                MessageEvents.Warning(message);
                CMN_LOG_CLASS_RUN_WARNING << message
                                          << ", \n requested: " << DesiredPositionParam.Goal()
                                          << ", \n lower limits: " << JointLowerLimit
                                          << ", \n upper limits: " << JointUpperLimit
                                          << std::endl;
            }
        }
    }

    double dt = StateTable.Period;
    DesiredVelocity = (DesiredPosition - oldDesiredPos) / dt;
}


void mtsPID::Enable(const bool & enable)
{
    Enabled = enable;

    // set torque to 0
    Torque.SetAll(0.0);

    // write torque to robot
    TorqueParam.SetForceTorque(Torque);
    Robot.SetTorque(TorqueParam);

    // reset error flags
    mPreviousTrackingErrorFlag.SetAll(false);
    mTrackingErrorFlag.SetAll(false);
    mPreviousJointLimitFlag.SetAll(false);
    mJointLimitFlag.SetAll(false);

    // trigger Enabled
    Events.Enabled(Enabled);
}

void mtsPID::EnableTorqueMode(const vctBoolVec &ena)
{
    if (TorqueMode.size() != ena.size()) {
        CMN_LOG_CLASS_RUN_ERROR << "EnableTorqueMode size mismatch" << std::endl;
        return;
    } else {
        TorqueMode.Assign(ena);
    }

    // set torque to 0
    Torque.SetAll(0.0);

    // write torque to robot
    TorqueParam.SetForceTorque(Torque);
    Robot.SetTorque(TorqueParam);
}

void mtsPID::SetTrackingErrorTolerances(const vctDoubleVec & tolerances)
{
    if (tolerances.size() == mTrackingErrorTolerances.size()) {
        mTrackingErrorTolerances.Assign(tolerances);
    } else {
        std::string message = this->Name + ": incorrect vector size for SetTrackingErrorTolerances";
        cmnThrow(message);
    }
}

void mtsPID::ErrorEventHandler(const std::string & message) {
    this->Enable(false);
    MessageEvents.Error(this->GetName() + ": received [" + message + "]");
}
