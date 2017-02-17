/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen
  Created on: 2013-02-22

  (C) Copyright 2013-2017 Johns Hopkins University (JHU), All Rights Reserved.

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
        requiredInterface->AddFunction("SetCoupling", Robot.SetCoupling);
        requiredInterface->AddFunction("GetJointType", Robot.GetJointType);
        requiredInterface->AddFunction("GetPositionJoint", Robot.GetFeedbackPosition);
        requiredInterface->AddFunction("GetTorqueJoint", Robot.GetFeedbackTorque);
        requiredInterface->AddFunction("GetVelocityJoint", Robot.GetFeedbackVelocity, MTS_OPTIONAL);
        requiredInterface->AddFunction("SetTorqueJoint", Robot.SetTorque);
        // event handlers
        requiredInterface->AddEventHandlerWrite(&mtsPID::CouplingEventHandler, this, "Coupling");
        requiredInterface->AddEventHandlerWrite(&mtsPID::ErrorEventHandler, this, "Error");
    }

    // this should go in "write" state table
    StateTable.AddData(mTorqueCommand, "TorqueCommand");
    // this should go in a "read" state table
    StateTable.AddData(mPositionMeasure, "PositionMeasure");
    StateTable.AddData(mVelocityMeasure, "VelocityMeasure");
    StateTable.AddData(mTorqueMeasure, "TorqueMeasure");
    StateTable.AddData(mPositionCommand, "PositionCommand");
    StateTable.AddData(CheckJointLimit, "IsCheckJointLimit");
    StateTable.AddData(mGains.Offset, "TorqueOffset");
    StateTable.AddData(mStateJointMeasure, "StateJointMeasure");
    StateTable.AddData(mStateJointCommand, "StateJointCommand");

    // configuration state table with occasional start/advance
    ConfigurationStateTable.AddData(mGains.Kp, "Kp");
    ConfigurationStateTable.AddData(mGains.Kd, "Kd");
    ConfigurationStateTable.AddData(mGains.Ki, "Ki");
    ConfigurationStateTable.AddData(JointLowerLimit, "JointLowerLimit");
    ConfigurationStateTable.AddData(JointUpperLimit, "JointUpperLimit");
    ConfigurationStateTable.AddData(JointType, "JointType");
    ConfigurationStateTable.AddData(mEnableTrackingError, "EnableTrackingError");
    ConfigurationStateTable.AddData(mTrackingErrorTolerances, "TrackingErrorTolerances");

    mInterface = AddInterfaceProvided("Controller");
    mInterface->AddMessageEvents();
    if (mInterface) {
        mInterface->AddCommandVoid(&mtsPID::ResetController, this, "ResetController");
        mInterface->AddCommandWrite(&mtsPID::Enable, this, "Enable", false);
        mInterface->AddCommandWrite(&mtsPID::EnableJoints, this, "EnableJoints", mJointsEnabled);
        mInterface->AddCommandWrite(&mtsPID::EnableTorqueMode, this, "EnableTorqueMode", TorqueMode);
        mInterface->AddCommandWrite(&mtsPID::SetDesiredPosition, this, "SetPositionJoint", mPositionCommand);
        mInterface->AddCommandWrite(&mtsPID::SetDesiredTorque, this, "SetTorqueJoint", prmForceTorqueJointSet());
        mInterface->AddCommandReadState(StateTable, mPositionMeasure, "GetPositionJoint");
        mInterface->AddCommandReadState(StateTable, mVelocityMeasure, "GetVelocityJoint");
        mInterface->AddCommandReadState(StateTable, mTorqueMeasure, "GetTorqueJoint");
        mInterface->AddCommandReadState(StateTable, mPositionCommand, "GetPositionJointDesired");
        mInterface->AddCommandReadState(StateTable, mTorqueCommand, "GetEffortJointDesired");

        // ROS compatible joint state
        mInterface->AddCommandReadState(StateTable, mStateJointMeasure, "GetStateJoint");
        mInterface->AddCommandReadState(StateTable, mStateJointCommand, "GetStateJointDesired");

        // coupling
        mInterface->AddCommandWrite(&mtsPID::SetCoupling, this, "SetCoupling", prmActuatorJointCoupling());
        mInterface->AddEventWrite(Events.Coupling, "Coupling", prmActuatorJointCoupling());

        // Set check limits
        mInterface->AddCommandWriteState(StateTable, CheckJointLimit, "SetCheckJointLimit");
        mInterface->AddCommandWriteState(StateTable, mGains.Offset, "SetTorqueOffset");

        // Get PID gains
        mInterface->AddCommandReadState(ConfigurationStateTable, mGains.Kp, "GetPGain");
        mInterface->AddCommandReadState(ConfigurationStateTable, mGains.Kd, "GetDGain");
        mInterface->AddCommandReadState(ConfigurationStateTable, mGains.Ki, "GetIGain");

        // Get joint limits
        mInterface->AddCommandReadState(ConfigurationStateTable, JointLowerLimit, "GetJointLowerLimit");
        mInterface->AddCommandReadState(ConfigurationStateTable, JointUpperLimit, "GetJointUpperLimit");
        mInterface->AddCommandReadState(ConfigurationStateTable, JointType, "GetJointType");

        // Error tracking
        mInterface->AddCommandWriteState(ConfigurationStateTable, mEnableTrackingError, "EnableTrackingError");
        mInterface->AddCommandWrite(&mtsPID::SetTrackingErrorTolerances, this, "SetTrackingErrorTolerances");

        // Set PID gains
        mInterface->AddCommandWrite(&mtsPID::SetPGain, this, "SetPGain", mGains.Kp);
        mInterface->AddCommandWrite(&mtsPID::SetDGain, this, "SetDGain", mGains.Kd);
        mInterface->AddCommandWrite(&mtsPID::SetIGain, this, "SetIGain", mGains.Ki);

        // Set joint limits
        mInterface->AddCommandWrite(&mtsPID::SetJointLowerLimit, this, "SetJointLowerLimit", JointLowerLimit);
        mInterface->AddCommandWrite(&mtsPID::SetJointUpperLimit, this, "SetJointUpperLimit", JointUpperLimit);

        // Events
        mInterface->AddEventWrite(Events.Enabled, "Enabled", false);
        mInterface->AddEventWrite(Events.EnabledJoints, "EnabledJoints", vctBoolVec());
        mInterface->AddEventWrite(Events.JointLimit, "JointLimit", vctBoolVec());
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
    mGains.Kp.SetSize(mNumberOfJoints);
    mGains.Kd.SetSize(mNumberOfJoints);
    mGains.Ki.SetSize(mNumberOfJoints);
    mGains.Offset.SetSize(mNumberOfJoints);
    mGains.Offset.SetAll(0.0);
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
    mPositionMeasure.SetSize(mNumberOfJoints);
    mTorqueMeasure.SetSize(mNumberOfJoints);
    mTorqueCommand.SetSize(mNumberOfJoints);
    mTorqueCommand.ForceTorque().SetAll(0.0);
    mTorqueUserCommand.SetSize(mNumberOfJoints);
    mTorqueUserCommand.ForceTorque().SetAll(0.0);
    mVelocityMeasure.SetSize(mNumberOfJoints);
    mPositionMeasure.SetSize(mNumberOfJoints);
    mPositionMeasurePrevious.SetSize(mNumberOfJoints);
    mPositionCommand.SetSize(mNumberOfJoints);
    mPositionCommand.Goal().SetAll(0.0);
    mVelocityMeasure.SetSize(mNumberOfJoints);

    mStateJointMeasure.Position().SetSize(mNumberOfJoints);
    mStateJointMeasure.Velocity().SetSize(mNumberOfJoints);
    mStateJointMeasure.Effort().SetSize(mNumberOfJoints);
    mStateJointCommand.Position().SetSize(mNumberOfJoints);
    mStateJointCommand.Velocity().SetSize(0); // we don't support desired velocity
    mStateJointCommand.Effort().SetSize(mNumberOfJoints);

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
    mStateJointMeasure.Name().SetSize(mNumberOfJoints);
    mStateJointCommand.Name().SetSize(mNumberOfJoints);

    // read data from xml file
    char context[64];
    for (int i = 0; i < numberOfJoints; i++) {
        // joint
        sprintf(context, "controller/joints/joint[%d]", i + 1);

        // name
        std::string name;
        config.GetXMLValue(context, "@name", name);
        mStateJointMeasure.Name().at(i) = name;
        mStateJointCommand.Name().at(i) = name;

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
        config.GetXMLValue(context, "pid/@PGain", mGains.Kp[i]);
        config.GetXMLValue(context, "pid/@DGain", mGains.Kd[i]);
        config.GetXMLValue(context, "pid/@IGain", mGains.Ki[i]);
        config.GetXMLValue(context, "pid/@OffsetTorque", mGains.Offset[i]);
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

    CMN_LOG_CLASS_INIT_VERBOSE << "Kp: " << mGains.Kp << std::endl
                               << "Kd: " << mGains.Kd << std::endl
                               << "Ki: " << mGains.Ki << std::endl
                               << "Offset: " << mGains.Offset << std::endl
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

    // update states
    if (mIsSimulated) {
        mPositionMeasure.SetValid(true);
        mPositionMeasure.Position().ForceAssign(mPositionMeasurePrevious.Position());
        mTorqueMeasure.Assign(mTorqueCommand.ForceTorque());
    } else {
        Robot.GetFeedbackPosition(mPositionMeasure);
        Robot.GetFeedbackTorque(mTorqueMeasure);
    }

    // compute error
    Error.DifferenceOf(mPositionCommand.Goal(), mPositionMeasure.Position());
    for (size_t i = 0; i < mNumberOfJoints; i++) {
        if ((Error[i] <= DeadBand[i]) && (Error[i] >= -DeadBand[i]))
            Error[i] = 0.0;
    }

    // update velocities from robot
    if (!mIsSimulated && Robot.GetFeedbackVelocity.IsValid()) {
        Robot.GetFeedbackVelocity(mVelocityMeasure);
    } else {
        // or compute an estimate from position
        double dt = mPositionMeasure.Timestamp() - mPositionMeasurePrevious.Timestamp();
        if (dt > 0) {
            mVelocityMeasure.Velocity().DifferenceOf(mPositionMeasure.Position(),
                                                      mPositionMeasurePrevious.Position());
            mVelocityMeasure.Velocity().Divide(dt);
        } else {
            mVelocityMeasure.Velocity().SetAll(0.0);
        }
    }

    // update state data
    mStateJointMeasure.Position().ForceAssign(mPositionMeasure.Position());
    mStateJointMeasure.Velocity().ForceAssign(mVelocityMeasure.Velocity());
    mStateJointMeasure.Effort().ForceAssign(mTorqueMeasure);
    mStateJointCommand.Position().ForceAssign(mPositionCommand.Goal());

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
                    mInterface->SendError(message);
                    CMN_LOG_CLASS_RUN_ERROR << message << std::endl
                                            << "errors:     " << Error << std::endl
                                            << "tolerances: " << mTrackingErrorTolerances << std::endl
                                            << "measure:    " << mPositionMeasure.Position() << std::endl
                                            << "command:    " << mPositionCommand.Goal() << std::endl;
                }
            }
        }

        // compute error derivative
        dError.Assign(mVelocityMeasure.Velocity());
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
        mTorqueCommand.ForceTorque().ElementwiseProductOf(mGains.Kp, Error);
        mTorqueCommand.ForceTorque().AddElementwiseProductOf(mGains.Kd, dError);
        mTorqueCommand.ForceTorque().AddElementwiseProductOf(mGains.Ki, iError);

        // nonlinear control mode
        for (size_t i = 0; i < mNumberOfJoints; i++) {
            if ((nonlinear[i] > 0)
                && (fabs(Error[i]) < nonlinear[i])) {
                mTorqueCommand.ForceTorque()[i] *= fabs(Error[i]) / nonlinear[i];
            }
        }

        // set torque to zero if that joint was not enabled
        for (size_t i = 0; i < mNumberOfJoints; i++) {
            if (!mJointsEnabled[i]) {
                mTorqueCommand.ForceTorque()[i] = 0.0;
                mStateJointCommand.Position()[i] = mPositionMeasure.Position()[i];
            }
        }

        // Add torque (e.g. gravity compensation)
        mTorqueCommand.ForceTorque().Add(mGains.Offset);

        // Set Torque to DesiredTorque if
        for (size_t i = 0; i < mNumberOfJoints; i++) {
            if (TorqueMode[i]) {
                mTorqueCommand.ForceTorque()[i] = mTorqueUserCommand.ForceTorque()[i];
                // since we assume nobody sent a desired position,
                // update desired from current
                mStateJointCommand.Position()[i] = mPositionMeasure.Position()[i];
                mPositionCommand.Goal()[i] = mPositionMeasure.Position()[i];
            }
        }

        // write torque to robot
        if (!mIsSimulated) {
            Robot.SetTorque(mTorqueCommand);
        }
    }
    else {
        mTorqueCommand.ForceTorque().SetAll(0.0);
        mStateJointMeasure.Position() = mPositionMeasure.Position();
        if (!mIsSimulated) {
            Robot.SetTorque(mTorqueCommand);
        }
    }

    // for simulated mode
    if (mIsSimulated) {
        mPositionMeasure.SetValid(true);
        mPositionMeasure.Position().Assign(mPositionCommand.Goal());
        mTorqueMeasure.Assign(mTorqueCommand.ForceTorque());
    }

    // update state data
    mStateJointCommand.Effort().ForceAssign(mTorqueCommand.ForceTorque());

    // save previous position
    mPositionMeasurePrevious = mPositionMeasure;
}


void mtsPID::Cleanup(void)
{
    // cleanup
    mTorqueCommand.ForceTorque().SetAll(0.0);
    if (!mIsSimulated) {
        Robot.SetTorque(mTorqueCommand);
    }
}

void mtsPID::SetSimulated(void)
{
    mIsSimulated = true;
    // in simulation mode, we don't need IO
    RemoveInterfaceRequired("RobotJointTorqueInterface");
}

void mtsPID::SetPGain(const vctDoubleVec & gain)
{
    if (gain.size() != mNumberOfJoints) {
        CMN_LOG_CLASS_INIT_ERROR << "SetPGain: size mismatch" << std::endl;
    } else {
        ConfigurationStateTable.Start();
        mGains.Kp.Assign(gain);
        ConfigurationStateTable.Advance();
    }
}

void mtsPID::SetDGain(const vctDoubleVec & gain)
{
    if (gain.size() != mNumberOfJoints) {
        CMN_LOG_CLASS_INIT_ERROR << "SetDGain: size mismatch" << std::endl;
    } else {
        ConfigurationStateTable.Start();
        mGains.Kd.Assign(gain);
        ConfigurationStateTable.Advance();
    }
}

void mtsPID::SetIGain(const vctDoubleVec & gain)
{
    if (gain.size() != mNumberOfJoints) {
        CMN_LOG_CLASS_INIT_ERROR << "SetIGain: size mismatch" << std::endl;
    } else {
        ConfigurationStateTable.Start();
        mGains.Ki.Assign(gain);
        ConfigurationStateTable.Advance();
    }
}

void mtsPID::SetJointLowerLimit(const vctDoubleVec & lowerLimit)
{
    if (lowerLimit.size() != mNumberOfJoints) {
        CMN_LOG_CLASS_INIT_ERROR << "SetJointLowerLimit: size mismatch" << std::endl;
    } else {
        ConfigurationStateTable.Start();
        JointLowerLimit.Assign(lowerLimit);
        ConfigurationStateTable.Advance();
    }
}

void mtsPID::SetJointUpperLimit(const vctDoubleVec & upperLimit)
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

void mtsPID::SetDesiredTorque(const prmForceTorqueJointSet & command)
{
    mTorqueUserCommand = command;
}

void mtsPID::SetDesiredPosition(const prmPositionJointSet & command)
{
    mPositionCommand = command;

    if (CheckJointLimit) {
        bool limitReached = false;
        vctDoubleVec::const_iterator upper = JointUpperLimit.begin();
        vctDoubleVec::const_iterator lower = JointLowerLimit.begin();
        vctBoolVec::iterator flags = mJointLimitFlag.begin();
        vctDoubleVec::iterator desired = mPositionCommand.Goal().begin();
        const vctDoubleVec::iterator end = mPositionCommand.Goal().end();
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
                mInterface->SendWarning(message);
                CMN_LOG_CLASS_RUN_WARNING << message
                                          << ", \n requested: " << mPositionCommand.Goal()
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
    mTorqueCommand.ForceTorque().SetAll(0.0);
    if (!mIsSimulated) {
        Robot.SetTorque(mTorqueCommand);
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
    mTorqueCommand.ForceTorque().SetAll(0.0);
    if (!mIsSimulated) {
        Robot.SetTorque(mTorqueCommand);
    }
}

void mtsPID::SetCoupling(const prmActuatorJointCoupling & coupling)
{
    // we assume the user has disabled whatever joints needed to be
    // disabled so we just past the request to IO level
    Robot.SetCoupling(coupling);
}

void mtsPID::CouplingEventHandler(const prmActuatorJointCoupling & coupling)
{
    // force recalculating everything based on new IO data
    // update states
    if (mIsSimulated) {
        mPositionMeasure.SetValid(true);
        mPositionMeasure.SetPosition(mPositionMeasurePrevious.Position());
        mTorqueMeasure.Assign(mTorqueCommand.ForceTorque());
    } else {
        Robot.GetFeedbackPosition(mPositionMeasure);
        std::cerr << "PID post coupling: " << mPositionMeasure << std::endl;
        Robot.GetFeedbackTorque(mTorqueMeasure);
    }
    Events.Coupling(coupling);
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

void mtsPID::ErrorEventHandler(const mtsMessage & message)
{
    if (this->Enabled) {
        this->Enable(false);
        mInterface->SendError(this->GetName() + ": received [" + message.Message + "]");
    } else {
        mInterface->SendStatus(this->GetName() + ": received [" + message.Message + "]");
    }
}
