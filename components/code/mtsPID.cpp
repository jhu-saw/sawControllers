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


mtsPID::mtsPID(const std::string & componentName, const double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds),
    Counter(0),
    CheckJointLimit(true),
    Enabled(false),
    mIsSimulated(false),
    ConfigurationStateTable(100, "Configuration")
{
    AddStateTable(&ConfigurationStateTable);
    ConfigurationStateTable.SetAutomaticAdvance(false);
}


mtsPID::mtsPID(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg),
    Counter(0),
    CheckJointLimit(true),
    Enabled(false),
    mIsSimulated(false),
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
        interfaceProvided->AddCommandWrite(&mtsPID::EnableJoints, this, "EnableJoints", mJointsEnabled);
        interfaceProvided->AddCommandWrite(&mtsPID::EnableTorqueMode, this, "EnableTorqueMode", TorqueMode);
        interfaceProvided->AddCommandWrite(&mtsPID::SetDesiredPosition, this, "SetPositionJoint", DesiredPositionParam);
        interfaceProvided->AddCommandWrite(&mtsPID::SetDesiredTorque, this, "SetTorqueJoint", prmDesiredTrq);
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
        interfaceProvided->AddEventWrite(Events.EnabledJoints, "EnabledJoints", vctBoolVec());
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
    config.GetXMLValue("/controller", "@type", type, "");
    config.GetXMLValue("/controller", "@interface", interface, "");
    int numberOfJoints;
    config.GetXMLValue("/controller", "@numofjoints", numberOfJoints, -1);
    if (type != "PID") {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: wrong controller type" << std::endl;
        ConfigurationStateTable.Advance();
        return;
    } else if (interface != "JointTorqueInterface") {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: wrong interface. Require JointTorqueInterface" << std::endl;
        ConfigurationStateTable.Advance();
        return;
    } else if (numberOfJoints < 0) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: invalid number of joints" << std::endl;
        ConfigurationStateTable.Advance();
        return;
    }
    mNumberOfJoints = static_cast<size_t>(numberOfJoints);

    // set dynamic var size
    JointType.SetSize(mNumberOfJoints);
    Kp.SetSize(mNumberOfJoints);
    Kd.SetSize(mNumberOfJoints);
    Ki.SetSize(mNumberOfJoints);
    Offset.SetSize(mNumberOfJoints);
    Offset.SetAll(0.0);
    JointLowerLimit.SetSize(mNumberOfJoints);
    JointLowerLimit.SetAll(0.0);
    JointUpperLimit.SetSize(mNumberOfJoints);
    JointUpperLimit.SetAll(0.0);
    mJointLimitFlag.SetSize(mNumberOfJoints);
    mJointLimitFlag.SetAll(false);
    mPreviousJointLimitFlag.ForceAssign(mJointLimitFlag);
    mJointsEnabled.SetSize(mNumberOfJoints);
    mJointsEnabled.SetAll(true);

    // feedback
    FeedbackPosition.SetSize(mNumberOfJoints);
    DesiredPosition.SetSize(mNumberOfJoints);
    FeedbackTorque.SetSize(mNumberOfJoints);
    DesiredTorque.SetSize(mNumberOfJoints);
    DesiredTorque.SetAll(0.0);
    FeedbackVelocity.SetSize(mNumberOfJoints);
    Torque.SetSize(mNumberOfJoints);
    Torque.SetAll(0.0);
    FeedbackPositionParam.SetSize(mNumberOfJoints);
    FeedbackPositionPreviousParam.SetSize(mNumberOfJoints);
    DesiredPositionParam.SetSize(mNumberOfJoints);
    DesiredPositionParam.Goal().SetAll(0.0);
    FeedbackVelocityParam.SetSize(mNumberOfJoints);
    TorqueParam.SetSize(mNumberOfJoints);

    mStateJoint.Position().SetSize(mNumberOfJoints);
    mStateJoint.Velocity().SetSize(mNumberOfJoints);
    mStateJoint.Effort().SetSize(mNumberOfJoints);
    mStateJointDesired.Position().SetSize(mNumberOfJoints);
    mStateJointDesired.Velocity().SetSize(0); // we don't support desired velocity
    mStateJointDesired.Effort().SetSize(mNumberOfJoints);

    // errors
    Error.SetSize(mNumberOfJoints);
    ErrorAbsolute.SetSize(mNumberOfJoints);
    dError.SetSize(mNumberOfJoints);
    iError.SetSize(mNumberOfJoints);
    ResetController();

    minIErrorLimit.SetSize(mNumberOfJoints);
    minIErrorLimit.SetAll(-100.0);
    maxIErrorLimit.SetSize(mNumberOfJoints);
    maxIErrorLimit.SetAll(100.0);

    // default 1.0: no effect
    forgetIError.SetSize(mNumberOfJoints);
    forgetIError.SetAll(1.0);

    // default: use regular PID
    nonlinear.SetSize(mNumberOfJoints);
    nonlinear.SetAll(0.0);

    DeadBand.SetSize(mNumberOfJoints);
    DeadBand.SetAll(0.0);

    // torque mode
    TorqueMode.SetSize(mNumberOfJoints);
    TorqueMode.SetAll(false);

    // tracking error
    mEnableTrackingError = false;
    mTrackingErrorTolerances.SetSize(mNumberOfJoints);
    mTrackingErrorTolerances.SetAll(0.0);
    mTrackingErrorFlag.SetSize(mNumberOfJoints);
    mTrackingErrorFlag.SetAll(false);
    mPreviousTrackingErrorFlag.ForceAssign(mTrackingErrorFlag);

    // names in joint states
    mStateJoint.Name().SetSize(mNumberOfJoints);
    mStateJointDesired.Name().SetSize(mNumberOfJoints);

    // read data from xml file
    char context[64];
    for (int i = 0; i < numberOfJoints; i++) {
        // joint
        sprintf(context, "controller/joints/joint[%d]", i + 1);

        // name
        std::string name;
        config.GetXMLValue(context, "@name", name);
        mStateJoint.Name().at(i) = name;
        mStateJointDesired.Name().at(i) = name;

        // type
        std::string type;
        config.GetXMLValue(context, "@type", type);
        if (type == "Revolute") {
            JointType.at(i) = PRM_REVOLUTE;
        } else if (type == "Prismatic") {
            JointType.at(i) = PRM_PRISMATIC;
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: joint " << i << " in file: "
                                     << filename
                                     << " needs a \"type\", either \"Revolute\" or \"Prismatic\""
                                     << std::endl;
        }

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
    // get joint type from IO and check against values from PID config file
    if (!mIsSimulated) {
        mtsExecutionResult result;
        prmJointTypeVec jointType;
        result = Robot.GetJointType(jointType);
        if (!result) {
            CMN_LOG_CLASS_INIT_ERROR << "Startup: Robot interface isn't connected properly, unable to get joint type.  Function call returned: "
                                     << result << std::endl;
        } else {
            if (jointType != JointType) {
                std::string message =  "Startup: joint types from IO don't match types from configuration files for " + this->GetName();
                CMN_LOG_CLASS_INIT_ERROR << message << std::endl
                                         << "From IO:     " << jointType << std::endl
                                         << "From config: " << JointType << std::endl;
                cmnThrow("PID::" + message);
            }
        }
    }
}

void mtsPID::Run(void)
{
    ProcessQueuedEvents();
    ProcessQueuedCommands();

    // increment counter
    Counter++;

    // update states
    if (mIsSimulated) {
        FeedbackPositionParam.SetValid(true);
        FeedbackPositionParam.SetPosition(FeedbackPositionPreviousParam.Position());
        FeedbackPositionParam.GetPosition(FeedbackPosition);
        FeedbackTorque.Assign(Torque);
    } else {
        Robot.GetFeedbackPosition(FeedbackPositionParam);
        FeedbackPositionParam.GetPosition(FeedbackPosition);
        Robot.GetFeedbackTorque(FeedbackTorque);
    }

    // compute error
    Error.DifferenceOf(DesiredPosition, FeedbackPosition);
    for (size_t i = 0; i < mNumberOfJoints; i++) {
        if ((Error[i] <= DeadBand[i]) && (Error[i] >= -DeadBand[i]))
            Error[i] = 0.0;
    }

    // update velocities from robot
    if (!mIsSimulated && Robot.GetFeedbackVelocity.IsValid()) {
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

    // update state data
    mStateJoint.Position().ForceAssign(FeedbackPosition);
    mStateJoint.Velocity().ForceAssign(FeedbackVelocity);
    mStateJoint.Effort().ForceAssign(FeedbackTorque);
    mStateJointDesired.Position().ForceAssign(DesiredPosition);

    // compute torque
    if (Enabled) {

        // check for tracking error
        if (mEnableTrackingError) {
            vctDoubleVec::const_iterator error = Error.begin();
            vctDoubleVec::const_iterator tolerance = mTrackingErrorTolerances.begin();
            vctBoolVec::iterator limitFlag = mJointLimitFlag.begin();
            vctBoolVec::iterator trackingErrorFlag = mTrackingErrorFlag.begin();
            vctBoolVec::iterator previousTrackingErrorFlag = mPreviousTrackingErrorFlag.begin();
            vctBoolVec::iterator jointsEnabled = mJointsEnabled.begin();
            bool anyTrackingError = false;
            bool newTrackingError = false;
            const vctDoubleVec::iterator end = Error.end();

            // detect tracking errors
            for (; error != end;
                 ++error, ++tolerance, ++limitFlag, ++jointsEnabled,
                 ++trackingErrorFlag, ++previousTrackingErrorFlag) {
                double errorAbsolute = fabs(*error);
                // joint is enabled
                // AND trigger error if the error is too high
                // AND the last request was not outside joint limit
                if ((*jointsEnabled) && (errorAbsolute > *tolerance) && !(*limitFlag)) {
                    anyTrackingError = true;
                    *trackingErrorFlag = true;
                    if (*trackingErrorFlag != *previousTrackingErrorFlag) {
                        newTrackingError = true;
                    }
                } else {
                    *trackingErrorFlag = false;
                }
                *previousTrackingErrorFlag = *trackingErrorFlag;
            }
            // act on errors
            if (anyTrackingError) {
                Enable(false);
                if (newTrackingError) {
                    std::string message = this->Name + ": tracking error, mask (1 for error): ";
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
        for (size_t i = 0; i < mNumberOfJoints; i++) {
            // iError clamping
            if (iError.at(i) > maxIErrorLimit.at(i)) {
                iError.at(i) = maxIErrorLimit.at(i);
            }
            else if (iError.at(i) < minIErrorLimit.at(i)) {
                iError.at(i) = minIErrorLimit[i];
            }
        }

        // compute torque
        Torque.ElementwiseProductOf(Kp, Error);
        Torque.AddElementwiseProductOf(Kd, dError);
        Torque.AddElementwiseProductOf(Ki, iError);

        // nonlinear control mode
        for (size_t i = 0; i < mNumberOfJoints; i++) {
            if ((nonlinear[i] > 0)
                && (fabs(Error[i]) < nonlinear[i])) {
                Torque[i] = Torque[i] * fabs(Error[i]) / nonlinear[i];
            }
        }

        // set torque to zero if that joint was not enabled
        for (size_t i = 0; i < mNumberOfJoints; i++) {
            if (!mJointsEnabled[i]) {
                Torque[i] = 0.0;
            }
        }

        // Add torque (e.g. gravity compensation)
        Torque.Add(Offset);

        // Set Torque to DesiredTorque if
        for (size_t i = 0; i < mNumberOfJoints; i++) {
            if (TorqueMode[i]) {
                Torque[i] = DesiredTorque[i];
                // since we assume nobody sent a desired position,
                // update desired from current
                mStateJointDesired.Position()[i] = FeedbackPosition[i];
                DesiredPosition[i] = FeedbackPosition[i];                
            }
        }

        // write torque to robot
        TorqueParam.SetForceTorque(Torque);
        if (!mIsSimulated) {
            Robot.SetTorque(TorqueParam);
        }
    }
    else {
        Torque.SetAll(0.0);
        TorqueParam.SetForceTorque(Torque);
        if (!mIsSimulated) {
            Robot.SetTorque(TorqueParam);
        }
    }

    // for simulated mode
    if (mIsSimulated) {
        FeedbackPositionParam.SetPosition(DesiredPosition);
        FeedbackPositionParam.SetValid(true);
        FeedbackPosition.Assign(DesiredPosition);
        FeedbackTorque.Assign(Torque);
    }

    // update state data
    mStateJointDesired.Effort().ForceAssign(Torque);

    // save previous position
    FeedbackPositionPreviousParam = FeedbackPositionParam;
}


void mtsPID::Cleanup(void)
{
    // cleanup
    Torque.SetAll(0.0);
    TorqueParam.SetForceTorque(Torque);
    if (!mIsSimulated) {
        Robot.SetTorque(TorqueParam);
    }
}

void mtsPID::SetSimulated(void)
{
    mIsSimulated = true;
    // in simulation mode, we don't need IO
    RemoveInterfaceRequired("RobotJointTorqueInterface");
}

void mtsPID::SetPGain(const vctDoubleVec & pgain)
{
    if (pgain.size() != mNumberOfJoints) {
        CMN_LOG_CLASS_INIT_ERROR << "SetPGain: size mismatch" << std::endl;
    } else {
        ConfigurationStateTable.Start();
        Kp.Assign(pgain);
        ConfigurationStateTable.Advance();
    }
}

void mtsPID::SetDGain(const vctDoubleVec & dgain)
{
    if (dgain.size() != mNumberOfJoints) {
        CMN_LOG_CLASS_INIT_ERROR << "SetDGain: size mismatch" << std::endl;
    } else {
        ConfigurationStateTable.Start();
        Kd.Assign(dgain);
        ConfigurationStateTable.Advance();
    }
}

void mtsPID::SetIGain(const vctDoubleVec & igain)
{
    if (igain.size() != mNumberOfJoints) {
        CMN_LOG_CLASS_INIT_ERROR << "SetIGain: size mismatch" << std::endl;
    } else {
        ConfigurationStateTable.Start();
        Ki.Assign(igain);
        ConfigurationStateTable.Advance();
    }
}

void mtsPID::SetJointLowerLimit(const vctDoubleVec &lowerLimit)
{
    if (lowerLimit.size() != mNumberOfJoints) {
        CMN_LOG_CLASS_INIT_ERROR << "SetJointLowerLimit: size mismatch" << std::endl;
    } else {
        ConfigurationStateTable.Start();
        JointLowerLimit.Assign(lowerLimit);
        ConfigurationStateTable.Advance();
    }
}

void mtsPID::SetJointUpperLimit(const vctDoubleVec &upperLimit)
{
    if (upperLimit.size() != mNumberOfJoints) {
        CMN_LOG_CLASS_INIT_ERROR << "SetJointUpperLimit: size mismatch" << std::endl;
    } else {
        ConfigurationStateTable.Start();
        JointUpperLimit.Assign(upperLimit);
        ConfigurationStateTable.Advance();
    }
}


void mtsPID::SetMinIErrorLimit(const vctDoubleVec & iminlim)
{
    if (iminlim.size() != mNumberOfJoints) {
        CMN_LOG_CLASS_INIT_ERROR << "SetMinIErrorLimit: size mismatch" << std::endl;
    } else {
        ConfigurationStateTable.Start();
        minIErrorLimit.Assign(iminlim);
        ConfigurationStateTable.Advance();
    }
}

void mtsPID::SetMaxIErrorLimit(const vctDoubleVec & imaxlim)
{
    if (imaxlim.size() != mNumberOfJoints) {
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
    dError.SetAll(0.0);
    iError.SetAll(0.0);
    Enable(false);
}

void mtsPID::SetDesiredTorque(const prmForceTorqueJointSet & prmTrq)
{
    prmDesiredTrq = prmTrq;
    prmDesiredTrq.GetForceTorque( DesiredTorque );
}

void mtsPID::SetDesiredPosition(const prmPositionJointSet & positionParam)
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
                std::string message = this->Name + ": joint limit, mask (1 for limit): ";
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
}


void mtsPID::Enable(const bool & enable)
{
    if (enable == Enabled) {
        return;
    }

    Enabled = enable;

    // set torque to 0
    Torque.SetAll(0.0);
    TorqueParam.SetForceTorque(Torque);
    if (!mIsSimulated) {
        Robot.SetTorque(TorqueParam);
    }
    // reset error flags
    if (enable) {
        mPreviousTrackingErrorFlag.SetAll(false);
        mTrackingErrorFlag.SetAll(false);
        mPreviousJointLimitFlag.SetAll(false);
        mJointLimitFlag.SetAll(false);
    }
    // trigger Enabled
    Events.Enabled(Enabled);
}

void mtsPID::EnableJoints(const vctBoolVec & enable)
{
    if (enable.size() == mNumberOfJoints) {
        mJointsEnabled.Assign(enable);
        Events.EnabledJoints(enable);
    } else {
        const std::string message = this->Name + ": incorrect vector size for EnableJoints";
        cmnThrow(message);
    }
}

void mtsPID::EnableTorqueMode(const vctBoolVec & enable)
{
    if (enable.size() != mNumberOfJoints) {
        CMN_LOG_CLASS_RUN_ERROR << "EnableTorqueMode size mismatch" << std::endl;
        return;
    } else {
        TorqueMode.Assign(enable);
    }

    // set torque to 0
    Torque.SetAll(0.0);

    // write torque to robot
    TorqueParam.SetForceTorque(Torque);
    if (!mIsSimulated) {
        Robot.SetTorque(TorqueParam);
    }
}

void mtsPID::SetTrackingErrorTolerances(const vctDoubleVec & tolerances)
{
    if (tolerances.size() == mNumberOfJoints) {
        mTrackingErrorTolerances.Assign(tolerances);
    } else {
        std::string message = this->Name + ": incorrect vector size for SetTrackingErrorTolerances";
        cmnThrow(message);
    }
}

void mtsPID::ErrorEventHandler(const std::string & message)
{
    if (this->Enabled) {
        this->Enable(false);
        MessageEvents.Error(this->GetName() + ": received [" + message + "]");
    } else {
        MessageEvents.Status(this->GetName() + ": received [" + message + "]");
    }
}
