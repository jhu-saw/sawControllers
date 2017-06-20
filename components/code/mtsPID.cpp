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
    mConfigurationStateTable(100, "Configuration")
{
    Init();
}


mtsPID::mtsPID(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg),
    mConfigurationStateTable(100, "Configuration")
{
    Init();
}


void mtsPID::Init(void)
{
    mCheckPositionLimit = true,
    mApplyEffortLimit = false,
    mEnabled = false,
    mIsSimulated = false,
    mNumberOfActiveJoints = 0,
    AddStateTable(&mConfigurationStateTable);
    mConfigurationStateTable.SetAutomaticAdvance(false);
}


void mtsPID::SetupInterfaces(void)
{
    // require RobotJointTorque interface
    mtsInterfaceRequired * requiredInterface = AddInterfaceRequired("RobotJointTorqueInterface");
    if (requiredInterface) {
        requiredInterface->AddFunction("SetCoupling", Robot.SetCoupling);
        requiredInterface->AddFunction("GetJointType", Robot.GetJointType);
        requiredInterface->AddFunction("GetPositionJoint", Robot.GetFeedbackPosition);
        requiredInterface->AddFunction("GetTorqueJoint", Robot.GetFeedbackEffort);
        requiredInterface->AddFunction("GetVelocityJoint", Robot.GetFeedbackVelocity, MTS_OPTIONAL);
        requiredInterface->AddFunction("SetTorqueJoint", Robot.SetEffort);
        // event handlers
        requiredInterface->AddEventHandlerWrite(&mtsPID::CouplingEventHandler, this, "Coupling");
        requiredInterface->AddEventHandlerWrite(&mtsPID::ErrorEventHandler, this, "Error");
    }

    // this should go in "write" state table
    StateTable.AddData(mEffortUserCommand, "EffortUserCommand");
    // this should go in a "read" state table
    StateTable.AddData(mPositionMeasure, "PositionMeasure");
    StateTable.AddData(mVelocityMeasure, "VelocityMeasure");
    StateTable.AddData(mEffortMeasure, "EffortMeasure");
    StateTable.AddData(mCheckPositionLimit, "CheckPositionLimit");
    StateTable.AddData(mGains.Offset, "EffortOffset");
    StateTable.AddData(mStateJointMeasure, "StateJointMeasure");
    StateTable.AddData(mStateJointCommand, "StateJointCommand");

    // configuration state table with occasional start/advance
    mConfigurationStateTable.AddData(mGains.Kp, "Kp");
    mConfigurationStateTable.AddData(mGains.Kd, "Kd");
    mConfigurationStateTable.AddData(mGains.Ki, "Ki");
    mConfigurationStateTable.AddData(mPositionLowerLimit, "PositionLowerLimit");
    mConfigurationStateTable.AddData(mPositionUpperLimit, "PositionUpperLimit");
    mConfigurationStateTable.AddData(mJointType, "JointType");
    mConfigurationStateTable.AddData(mEnableTrackingError, "EnableTrackingError");
    mConfigurationStateTable.AddData(mTrackingErrorTolerances, "TrackingErrorTolerances");

    mInterface = AddInterfaceProvided("Controller");
    mInterface->AddMessageEvents();
    if (mInterface) {
        mInterface->AddCommandVoid(&mtsPID::ResetController, this, "ResetController");
        mInterface->AddCommandWrite(&mtsPID::Enable, this, "Enable", false);
        mInterface->AddCommandWrite(&mtsPID::EnableJoints, this, "EnableJoints", mJointsEnabled);
        mInterface->AddCommandWrite(&mtsPID::EnableEffortMode, this, "EnableTorqueMode", mEffortMode);
        mInterface->AddCommandWrite(&mtsPID::SetDesiredPosition, this, "SetPositionJoint", prmPositionJointSet());
        mInterface->AddCommandWrite(&mtsPID::SetDesiredEffort, this, "SetTorqueJoint", prmForceTorqueJointSet());

        // ROS compatible joint state
        mInterface->AddCommandReadState(StateTable, mStateJointMeasure, "GetStateJoint");
        mInterface->AddCommandReadState(StateTable, mStateJointCommand, "GetStateJointDesired");

        // coupling
        mInterface->AddCommandWrite(&mtsPID::SetCoupling, this, "SetCoupling", prmActuatorJointCoupling());
        mInterface->AddEventWrite(Events.Coupling, "Coupling", prmActuatorJointCoupling());

        // Set check limits
        mInterface->AddCommandWriteState(StateTable, mCheckPositionLimit, "SetCheckPositionLimit");
        mInterface->AddCommandWriteState(StateTable, mGains.Offset, "SetTorqueOffset");

        // Get PID gains
        mInterface->AddCommandReadState(mConfigurationStateTable, mGains.Kp, "GetPGain");
        mInterface->AddCommandReadState(mConfigurationStateTable, mGains.Kd, "GetDGain");
        mInterface->AddCommandReadState(mConfigurationStateTable, mGains.Ki, "GetIGain");

        // Get joint limits
        mInterface->AddCommandReadState(mConfigurationStateTable, mPositionLowerLimit, "GetPositionLowerLimit");
        mInterface->AddCommandReadState(mConfigurationStateTable, mPositionUpperLimit, "GetPositionUpperLimit");
        mInterface->AddCommandReadState(mConfigurationStateTable, mJointType, "GetJointType");

        // Error tracking
        mInterface->AddCommandWriteState(mConfigurationStateTable, mEnableTrackingError, "EnableTrackingError");
        mInterface->AddCommandWrite(&mtsPID::SetTrackingErrorTolerances, this, "SetTrackingErrorTolerances");

        // Set PID gains
        mInterface->AddCommandWrite(&mtsPID::SetPGain, this, "SetPGain", mGains.Kp);
        mInterface->AddCommandWrite(&mtsPID::SetDGain, this, "SetDGain", mGains.Kd);
        mInterface->AddCommandWrite(&mtsPID::SetIGain, this, "SetIGain", mGains.Ki);

        // Set joint limits
        mInterface->AddCommandWrite(&mtsPID::SetPositionLowerLimit, this, "SetPositionLowerLimit", mPositionLowerLimit);
        mInterface->AddCommandWrite(&mtsPID::SetPositionUpperLimit, this, "SetPositionUpperLimit", mPositionUpperLimit);

        // Set effort limits
        mInterface->AddCommandWrite(&mtsPID::SetEffortLowerLimit, this, "SetTorqueLowerLimit", mEffortLowerLimit);
        mInterface->AddCommandWrite(&mtsPID::SetEffortUpperLimit, this, "SetTorqueUpperLimit", mEffortUpperLimit);

        // Events
        mInterface->AddEventWrite(Events.Enabled, "Enabled", false);
        mInterface->AddEventWrite(Events.EnabledJoints, "EnabledJoints", vctBoolVec());
        mInterface->AddEventWrite(Events.PositionLimit, "PositionLimit", vctBoolVec());
    }
}

void mtsPID::Configure(const std::string & filename)
{
    mConfigurationStateTable.Start();

    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: using " << filename << std::endl;
    cmnXMLPath config;
    config.SetInputSource(filename);
    mNumberOfActiveJoints = 0;
    bool hasInactiveJoints = false;

    // check type, interface and number of joints
    std::string type, interface;
    config.GetXMLValue("/controller", "@type", type, "");
    config.GetXMLValue("/controller", "@interface", interface, "");
    int numberOfJoints;
    config.GetXMLValue("/controller", "@numofjoints", numberOfJoints, -1);
    if (type != "PID") {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: wrong controller type" << std::endl;
        mConfigurationStateTable.Advance();
        return;
    } else if (interface != "JointTorqueInterface") {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: wrong interface. Require JointTorqueInterface" << std::endl;
        mConfigurationStateTable.Advance();
        return;
    } else if (numberOfJoints < 0) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: invalid number of joints" << std::endl;
        mConfigurationStateTable.Advance();
        return;
    }
    mNumberOfJoints = static_cast<size_t>(numberOfJoints);

    // set dynamic var size
    mJointType.SetSize(mNumberOfJoints);
    mGains.Kp.SetSize(mNumberOfJoints);
    mGains.Kd.SetSize(mNumberOfJoints);
    mGains.Ki.SetSize(mNumberOfJoints);
    mGains.Offset.SetSize(mNumberOfJoints, 0.0);
    mPositionLowerLimit.SetSize(mNumberOfJoints, 0.0);
    mPositionUpperLimit.SetSize(mNumberOfJoints, 0.0);
    mEffortLowerLimit.SetSize(mNumberOfJoints, 0.0);
    mEffortUpperLimit.SetSize(mNumberOfJoints, 0.0);
    mPositionLimitFlag.SetSize(mNumberOfJoints);
    mPositionLimitFlag.SetAll(false);
    mPositionLimitFlagPrevious.ForceAssign(mPositionLimitFlag);
    mJointsEnabled.SetSize(mNumberOfJoints);
    mJointsEnabled.SetAll(true);

    // feedback
    mPositionMeasure.Position().SetSize(mNumberOfJoints, 0.0);
    mEffortMeasure.SetSize(mNumberOfJoints, 0.0);
    mEffortPIDCommand.ForceTorque().SetSize(mNumberOfJoints, 0.0);
    mEffortUserCommand.ForceTorque().SetSize(mNumberOfJoints, 0.0);
    mVelocityMeasure.Velocity().SetSize(mNumberOfJoints, 0.0);
    mPositionMeasurePrevious.Position().SetSize(mNumberOfJoints, 0.0);

    // errors
    mError.SetSize(mNumberOfJoints);
    mIError.SetSize(mNumberOfJoints);
    ResetController();

    mIErrorLimitMin.SetSize(mNumberOfJoints, cmnTypeTraits<double>::MinNegativeValue());
    mIErrorLimitMax.SetSize(mNumberOfJoints, cmnTypeTraits<double>::MaxPositiveValue());

    // default 1.0: no effect
    mIErrorForgetFactor.SetSize(mNumberOfJoints);
    mIErrorForgetFactor.SetAll(1.0);

    // default: use regular PID
    mNonLinear.SetSize(mNumberOfJoints);
    mNonLinear.SetAll(0.0);

    mDeadBand.SetSize(mNumberOfJoints);
    mDeadBand.SetAll(0.0);

    // effort mode
    mEffortMode.SetSize(mNumberOfJoints);
    mEffortMode.SetAll(false);

    // tracking error
    mEnableTrackingError = false;
    mTrackingErrorTolerances.SetSize(mNumberOfJoints, 0.0);
    mTrackingErrorFlag.SetSize(mNumberOfJoints, false);
    mPreviousTrackingErrorFlag.ForceAssign(mTrackingErrorFlag);

    // read data from xml file
    char context[64];
    for (int i = 0; i < numberOfJoints; i++) {
        // joint
        sprintf(context, "controller/joints/joint[%d]", i + 1);

        // type
        std::string type;
        config.GetXMLValue(context, "@type", type);
        if (type == "Revolute") {
            mJointType.at(i) = PRM_REVOLUTE;
        } else if (type == "Prismatic") {
            mJointType.at(i) = PRM_PRISMATIC;
        } else if (type == "Inactive") {
            mJointType.at(i) = PRM_INACTIVE;
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: joint " << i << " in file: "
                                     << filename
                                     << " needs a \"type\", either \"Revolute\" or \"Prismatic\""
                                     << std::endl;
            mConfigurationStateTable.Advance();
            return;
        }

        // make sure we update the number of active joints
        if (mJointType.at(i) == PRM_INACTIVE) {
            hasInactiveJoints = true;
        } else {
            // we found an inactive joint after an active one, this is not supported
            if (hasInactiveJoints) {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: joint " << i << " in file: "
                                         << filename
                                         << " has is not \"Inactive\" but is defined after an \"Inactive\" joint, this is not supported"
                                         << std::endl;
                mConfigurationStateTable.Advance();
                return;
            }
            mNumberOfActiveJoints++;
        }

        if (!hasInactiveJoints) {
            // name
            std::string name;
            config.GetXMLValue(context, "@name", name);
            // names in joint states
            mStateJointMeasure.Name().resize(mNumberOfActiveJoints);
            mStateJointCommand.Name().resize(mNumberOfActiveJoints);
            mStateJointMeasure.Name().at(i) = name;
            mStateJointCommand.Name().at(i) = name;

            // pid
            config.GetXMLValue(context, "pid/@PGain", mGains.Kp.at(i));
            config.GetXMLValue(context, "pid/@DGain", mGains.Kd.at(i));
            config.GetXMLValue(context, "pid/@IGain", mGains.Ki.at(i));
            config.GetXMLValue(context, "pid/@OffsetTorque", mGains.Offset.at(i));
            config.GetXMLValue(context, "pid/@Forget", mIErrorForgetFactor.at(i));
            config.GetXMLValue(context, "pid/@Nonlinear", mNonLinear.at(i));

            // limit
            config.GetXMLValue(context, "limit/@MinILimit", mIErrorLimitMin.at(i));
            config.GetXMLValue(context, "limit/@MaxILimit", mIErrorLimitMax.at(i));
            config.GetXMLValue(context, "limit/@ErrorLimit", mTrackingErrorTolerances.at(i));
            config.GetXMLValue(context, "limit/@Deadband", mDeadBand.at(i));

            // joint limit
            mCheckPositionLimit = true;
            std::string tmpUnits;
            bool ret = false;
            ret = config.GetXMLValue(context, "pos/@Units", tmpUnits);
            if (ret) {
                config.GetXMLValue(context, "pos/@LowerLimit", mPositionLowerLimit.at(i));
                config.GetXMLValue(context, "pos/@UpperLimit", mPositionUpperLimit.at(i));
                if (tmpUnits == "deg") {
                    mPositionLowerLimit.at(i) *= cmnPI_180;
                    mPositionUpperLimit.at(i) *= cmnPI_180;
                } else if (tmpUnits == "mm") {
                    mPositionLowerLimit.at(i) *= cmn_mm;
                    mPositionUpperLimit.at(i) *= cmn_mm;
                }
            } else {
                mCheckPositionLimit = false;
            }
        }
    }

    // Convert from degrees to radians
    // TODO: Decide whether to use degrees or radians in XML file
    // TODO: Also do this for other parameters (not just Deadband)
    // TODO: Only do this for revolute joints
    mDeadBand.Multiply(cmnPI_180);

    CMN_LOG_CLASS_INIT_VERBOSE << "Kp: " << mGains.Kp << std::endl
                               << "Kd: " << mGains.Kd << std::endl
                               << "Ki: " << mGains.Ki << std::endl
                               << "Offset: " << mGains.Offset << std::endl
                               << "JntLowerLimit" << mPositionLowerLimit << std::endl
                               << "JntUpperLimit" << mPositionUpperLimit << std::endl
                               << "Deadband: " << mDeadBand << std::endl
                               << "minILimit: " << mIErrorLimitMin << std::endl
                               << "maxILimit: " << mIErrorLimitMax << std::endl
                               << "elimit: " << mTrackingErrorTolerances << std::endl
                               << "forget: " << mIErrorForgetFactor << std::endl;

    mConfigurationStateTable.Advance();

    mStateJointMeasure.Position().SetSize(mNumberOfActiveJoints, 0.0);
    mStateJointMeasure.Velocity().SetSize(mNumberOfActiveJoints, 0.0);
    mStateJointMeasure.Effort().SetSize(mNumberOfActiveJoints, 0.0);
    mStateJointCommand.Position().SetSize(mNumberOfActiveJoints, 0.0);
    mStateJointCommand.Velocity().SetSize(0); // we don't support desired velocity
    mStateJointCommand.Effort().SetSize(mNumberOfActiveJoints, 0.0);

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
            for (size_t index = 0;
                 index < mNumberOfActiveJoints;
                 ++index) {
                if (jointType.at(index) != mJointType.at(index)) {
                    std::string message =  "Startup: joint types from IO don't match types from configuration files for " + this->GetName();
                    CMN_LOG_CLASS_INIT_ERROR << message << std::endl
                                             << "From IO:     " << jointType << std::endl
                                             << "From config: " << mJointType << std::endl;
                    cmnThrow("PID::" + message);
                }
            }
        }
    }
}

void mtsPID::Run(void)
{
    ProcessQueuedEvents();
    ProcessQueuedCommands();

    // get data from IO if not in simulated mode
    GetIOData(true); // compute velocity if needed

    // initialize variables
    bool anyTrackingError = false;
    bool newTrackingError = false;

    // loop on all joints
    vctBoolVec::const_iterator enabled = mJointsEnabled.begin();
    vctDoubleVec::const_iterator measurePosition = mStateJointMeasure.Position().begin();
    vctDoubleVec::const_iterator measureVelocity = mStateJointMeasure.Velocity().begin();
    vctDoubleVec::iterator commandPosition = mStateJointCommand.Position().begin();
    vctDoubleVec::iterator commandEffort = mStateJointCommand.Effort().begin();
    vctBoolVec::const_iterator effortMode = mEffortMode.begin();
    vctDoubleVec::const_iterator effortUserCommand = mEffortUserCommand.ForceTorque().begin();
    vctDoubleVec::iterator error = mError.begin();
    vctDoubleVec::const_iterator deadBand = mDeadBand.begin();
    vctDoubleVec::const_iterator tolerance = mTrackingErrorTolerances.begin();
    vctBoolVec::iterator limitFlag = mPositionLimitFlag.begin();
    vctBoolVec::iterator trackingErrorFlag = mTrackingErrorFlag.begin();
    vctBoolVec::iterator previousTrackingErrorFlag = mPreviousTrackingErrorFlag.begin();
    vctDoubleVec::iterator iError = mIError.begin();
    vctDoubleVec::const_iterator iErrorForgetFactor = mIErrorForgetFactor.begin();
    vctDoubleVec::const_iterator iErrorLimitMin = mIErrorLimitMin.begin();
    vctDoubleVec::const_iterator iErrorLimitMax = mIErrorLimitMax.begin();
    vctDoubleVec::const_iterator kP = mGains.Kp.begin();
    vctDoubleVec::const_iterator kI = mGains.Ki.begin();
    vctDoubleVec::const_iterator kD = mGains.Kd.begin();
    vctDoubleVec::const_iterator offset = mGains.Offset.begin();
    vctDoubleVec::const_iterator nonLinear = mNonLinear.begin();
    vctDoubleVec::const_iterator effortLowerLimit = mEffortLowerLimit.begin();
    vctDoubleVec::const_iterator effortUpperLimit = mEffortUpperLimit.begin();

    // loop on all active joints using iterators
    for (size_t i = 0;
         i < mNumberOfActiveJoints;
         ++i,
             // make sure you increment all iterators declared above!
             ++enabled,
             ++measurePosition,
             ++measureVelocity,
             ++commandPosition,
             ++commandEffort,
             ++effortMode,
             ++effortUserCommand,
             ++error,
             ++deadBand,
             ++tolerance,
             ++limitFlag,
             ++trackingErrorFlag,
             ++previousTrackingErrorFlag,
             ++iError,
             ++iErrorForgetFactor,
             ++iErrorLimitMin,
             ++iErrorLimitMax,
             ++kP,
             ++kI,
             ++kD,
             ++offset,
             ++nonLinear,
             ++effortLowerLimit,
             ++effortUpperLimit
         ) {

        // first check if the controller and this joint is enabled
        if (!mEnabled || !(*enabled)) {
            *commandEffort = 0.0;
            *commandPosition = *measurePosition;
        } else {
            // the PID controller is enabled and this joint is actively controlled
            // check the mode, i.e. position or effort pass-through
            if (*effortMode) {
                *commandEffort = *effortUserCommand;
                *commandPosition = *measurePosition;
            } else {
                // PID mode
                *error = *commandPosition - *measurePosition;
                // apply dead band
                if ((*error <= *deadBand) && (*error >= -(*deadBand))) {
                    *error = 0.0;
                }
                // check for tracking errors
                if (mEnableTrackingError) {
                    double errorAbsolute = fabs(*error);
                    // trigger error if the error is too high
                    // AND the last request was not outside joint limit
                    if ((errorAbsolute > *tolerance) && !(*limitFlag)) {
                        anyTrackingError = true;
                        *trackingErrorFlag = true;
                        if (*trackingErrorFlag != *previousTrackingErrorFlag) {
                            newTrackingError = true;
                        }
                    } else {
                        *trackingErrorFlag = false;
                    }
                    *previousTrackingErrorFlag = *trackingErrorFlag;
                } // end of tracking error

                // compute error derivative
                double dError =  -1.0 * (*measureVelocity);

                // compute error integral
                *iError *= *iErrorForgetFactor;
                *iError += *error;
                if (*iError > *iErrorLimitMax) {
                    *iError = *iErrorLimitMax;
                }
                else if (*iError < *iErrorLimitMin) {
                    *iError = *iErrorLimitMin;
                }

                // compute effort
                *commandEffort =
                    *kP * (*error) + *kD * dError + *kI * (*iError);

                // nonlinear control mode
                if (*nonLinear > 0.0) {
                    const double absError = fabs(*error);
                    if (absError < *nonLinear) {
                        *commandEffort *= (absError / *nonLinear);
                    }
                }

                // add constant offsets in PID mode only and after non-linear scaling
                *commandEffort += *offset;

            } // end of PID mode

            // apply effort limits if needed
            if (mApplyEffortLimit) {
                if (*commandEffort > *effortUpperLimit) {
                    *commandEffort = *effortUpperLimit;
                } else if (*commandEffort < *effortLowerLimit) {
                    *commandEffort = *effortLowerLimit;
                }
            }
        } // end of enabled
    }

    // report errors (tracking)
    if (mEnableTrackingError && anyTrackingError) {
        Enable(false);
        if (newTrackingError) {
            std::string message = this->Name + ": tracking error, mask (1 for error): ";
            message.append(mTrackingErrorFlag.ToString());
            mInterface->SendError(message);
            CMN_LOG_CLASS_RUN_ERROR << message << std::endl
                                    << "errors:     " << mError << std::endl
                                    << "tolerances: " << mTrackingErrorTolerances << std::endl
                                    << "measure:    " << mPositionMeasure.Position() << std::endl
                                    << "command:    " << mStateJointCommand.Position() << std::endl;
        }
    }

    // save previous position with timestamp
    mPositionMeasurePrevious = mPositionMeasure;

    // for simulated mode
    if (mIsSimulated) {
        mPositionMeasure.SetValid(true);
        mPositionMeasure.Position().Assign(mStateJointCommand.Position(), mNumberOfActiveJoints);
        mEffortMeasure.Assign(mStateJointCommand.Effort(), mNumberOfActiveJoints);
    } else {
        if (mEnabled) {
            SetEffortLocal(mStateJointCommand.Effort());
        }
    }

    // update state data
    // outdated, now use state to compute effort
    // mStateJointCommand.Effort().Assign(mStateJointCommand.Effort(), mNumberOfActiveJoints);

    // save previous position
    // PORTED mPositionMeasurePrevious = mPositionMeasure;
}


void mtsPID::Cleanup(void)
{
    // cleanup
    mStateJointCommand.Effort().SetAll(0.0);
    if (!mIsSimulated) {
        SetEffortLocal(mStateJointCommand.Effort());
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
    if (gain.size() != mNumberOfActiveJoints) {
        CMN_LOG_CLASS_INIT_ERROR << "SetPGain: size mismatch" << std::endl;
        return;
    }
    mConfigurationStateTable.Start();
    mGains.Kp.Assign(gain, mNumberOfActiveJoints);
    mConfigurationStateTable.Advance();
}

void mtsPID::SetDGain(const vctDoubleVec & gain)
{
    if (gain.size() != mNumberOfActiveJoints) {
        CMN_LOG_CLASS_INIT_ERROR << "SetDGain: size mismatch" << std::endl;
        return;
    }
    mConfigurationStateTable.Start();
    mGains.Kd.Assign(gain, mNumberOfActiveJoints);
    mConfigurationStateTable.Advance();
}

void mtsPID::SetIGain(const vctDoubleVec & gain)
{
    if (gain.size() != mNumberOfActiveJoints) {
        CMN_LOG_CLASS_INIT_ERROR << "SetIGain: size mismatch" << std::endl;
        return;
    }
    mConfigurationStateTable.Start();
    mGains.Ki.Assign(gain, mNumberOfActiveJoints);
    mConfigurationStateTable.Advance();
}

void mtsPID::SetPositionLowerLimit(const vctDoubleVec & lowerLimit)
{
    if (lowerLimit.size() != mNumberOfActiveJoints) {
        CMN_LOG_CLASS_INIT_ERROR << "SetPositionLowerLimit: size mismatch" << std::endl;
        return;
    }
    mConfigurationStateTable.Start();
    mPositionLowerLimit.Assign(lowerLimit, mNumberOfActiveJoints);
    mConfigurationStateTable.Advance();
}

void mtsPID::SetPositionUpperLimit(const vctDoubleVec & upperLimit)
{
    if (upperLimit.size() != mNumberOfActiveJoints) {
        CMN_LOG_CLASS_INIT_ERROR << "SetPositionUpperLimit: size mismatch" << std::endl;
        return;
    }
    mConfigurationStateTable.Start();
    mPositionUpperLimit.Assign(upperLimit, mNumberOfActiveJoints);
    mConfigurationStateTable.Advance();
}

void mtsPID::SetEffortLowerLimit(const vctDoubleVec & lowerLimit)
{
    if (lowerLimit.size() != mNumberOfActiveJoints) {
        CMN_LOG_CLASS_INIT_ERROR << "SetEffortLowerLimit: size mismatch" << std::endl;
        return;
    }
    mConfigurationStateTable.Start();
    mEffortLowerLimit.Assign(lowerLimit, mNumberOfActiveJoints);
    mConfigurationStateTable.Advance();

    mApplyEffortLimit = mEffortLowerLimit.Any() && mEffortUpperLimit.Any();
}

void mtsPID::SetEffortUpperLimit(const vctDoubleVec & upperLimit)
{
    if (upperLimit.size() != mNumberOfActiveJoints) {
        CMN_LOG_CLASS_INIT_ERROR << "SetEffortUpperLimit: size mismatch" << std::endl;
        return;
    }
    mConfigurationStateTable.Start();
    mEffortUpperLimit.Assign(upperLimit, mNumberOfActiveJoints);
    mConfigurationStateTable.Advance();

    mApplyEffortLimit = mEffortLowerLimit.Any() && mEffortUpperLimit.Any();
}

void mtsPID::SetMinIErrorLimit(const vctDoubleVec & iminlim)
{
    if (iminlim.size() != mNumberOfActiveJoints) {
        CMN_LOG_CLASS_INIT_ERROR << "SetMinIErrorLimit: size mismatch" << std::endl;
        return;
    }
    mConfigurationStateTable.Start();
    mIErrorLimitMin.Assign(iminlim, mNumberOfActiveJoints);
    mConfigurationStateTable.Advance();
}

void mtsPID::SetMaxIErrorLimit(const vctDoubleVec & imaxlim)
{
    if (imaxlim.size() != mNumberOfActiveJoints) {
        CMN_LOG_CLASS_INIT_ERROR << "SetMaxIErrorLimit: size mismatch" << std::endl;
        return;
    }
    mConfigurationStateTable.Start();
    mIErrorLimitMax.Assign(imaxlim, mNumberOfActiveJoints);
    mConfigurationStateTable.Advance();
}

void mtsPID::SetForgetIError(const double & forget)
{
    mIErrorForgetFactor.SetAll(forget);
}

void mtsPID::ResetController(void)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "Reset Controller" << std::endl;
    mError.SetAll(0.0);
    mIError.SetAll(0.0);
    Enable(false);
}

void mtsPID::SetDesiredEffort(const prmForceTorqueJointSet & command)
{
    mEffortUserCommand = command;
    if (command.ForceTorque().size() != mNumberOfActiveJoints) {
        CMN_LOG_CLASS_INIT_ERROR << "SetDesiredEffort: size mismatch" << std::endl;
        return;
    }
}

void mtsPID::SetDesiredPosition(const prmPositionJointSet & command)
{

    if (command.Goal().size() != mNumberOfActiveJoints) {
        CMN_LOG_CLASS_INIT_ERROR << "SetDesiredPosition: size mismatch" << std::endl;
        return;
    }

    mStateJointCommand.Position().Assign(command.Goal(), mNumberOfActiveJoints);

    if (mCheckPositionLimit) {
        bool limitReached = false;
        vctDoubleVec::const_iterator upper = mPositionUpperLimit.begin();
        vctDoubleVec::const_iterator lower = mPositionLowerLimit.begin();
        vctBoolVec::iterator flags = mPositionLimitFlag.begin();
        vctDoubleVec::iterator desired = mStateJointCommand.Position().begin();
        const vctDoubleVec::iterator end = mStateJointCommand.Position().end();
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
            if (mPositionLimitFlagPrevious.NotEqual(mPositionLimitFlag)) {
                mPositionLimitFlagPrevious.Assign(mPositionLimitFlag);
                Events.PositionLimit(mPositionLimitFlag);
                std::string message = this->Name + ": position limit, mask (1 for limit): ";
                message.append(mPositionLimitFlag.ToString());
                mInterface->SendWarning(message);
                CMN_LOG_CLASS_RUN_WARNING << message
                                          << ", \n requested: " << mStateJointCommand.Position()
                                          << ", \n lower limits: " << mPositionLowerLimit
                                          << ", \n upper limits: " << mPositionUpperLimit
                                          << std::endl;
            }
        }
    }
}


void mtsPID::Enable(const bool & enable)
{
    if (enable == mEnabled) {
        return;
    }

    mEnabled = enable;

    // set effort to 0
    mStateJointCommand.Effort().SetAll(0.0);
    if (!mIsSimulated) {
        SetEffortLocal(mStateJointCommand.Effort());
    }
    // reset error flags
    if (enable) {
        mPreviousTrackingErrorFlag.SetAll(false);
        mTrackingErrorFlag.SetAll(false);
        mPositionLimitFlagPrevious.SetAll(false);
        mPositionLimitFlag.SetAll(false);
    }
    // trigger Enabled
    Events.Enabled(mEnabled);
}


void mtsPID::EnableJoints(const vctBoolVec & enable)
{
    if (enable.size() == mNumberOfActiveJoints) {
        mJointsEnabled.Assign(enable, mNumberOfActiveJoints);
        Events.EnabledJoints(enable);
    } else {
        const std::string message = this->Name + ": incorrect vector size for EnableJoints";
        cmnThrow(message);
    }
}


void mtsPID::EnableEffortMode(const vctBoolVec & enable)
{
    if (enable.size() != mNumberOfActiveJoints) {
        CMN_LOG_CLASS_RUN_ERROR << "EnableEffortMode size mismatch" << std::endl;
        return;
    }
    // save preference
    mEffortMode.Assign(enable, mNumberOfActiveJoints);
    // reset effort to 0
    mStateJointCommand.Effort().SetAll(0.0);
}

void mtsPID::SetCoupling(const prmActuatorJointCoupling & coupling)
{
    // we assume the user has disabled whatever joints needed to be
    // disabled so we just past the request to IO level
    Robot.SetCoupling(coupling);
}

void mtsPID::GetIOData(const bool computeVelocity)
{
    // get data from IO if not in simulated mode
    if (mIsSimulated) {
        mPositionMeasure.SetValid(true);
        // set current position/effort based on goal
        mPositionMeasure.Position().Assign(mStateJointCommand.Position(), mNumberOfActiveJoints);
        mPositionMeasure.SetTimestamp(StateTable.GetTic());
        // measured effort
        mEffortMeasure.Assign(mEffortUserCommand.ForceTorque());
    } else {
        Robot.GetFeedbackPosition(mPositionMeasure);
        Robot.GetFeedbackEffort(mEffortMeasure);
    }

    // update velocities from robot
    if (!mIsSimulated && Robot.GetFeedbackVelocity.IsValid()) {
        Robot.GetFeedbackVelocity(mVelocityMeasure);
    } else {
        if (computeVelocity) {
            // or compute an estimate from position
            double dt = mPositionMeasure.Timestamp() - mPositionMeasurePrevious.Timestamp();
            if (dt > 0) {
                vctDoubleVec::const_iterator currentPosition = mPositionMeasure.Position().begin();
                vctDoubleVec::const_iterator previousPosition = mPositionMeasurePrevious.Position().begin();
                vctDoubleVec::iterator velocity;
                const vctDoubleVec::iterator end = mVelocityMeasure.Velocity().end();
                for (velocity = mVelocityMeasure.Velocity().begin();
                     velocity != end;
                     ++velocity) {
                    *velocity = (*currentPosition - *previousPosition) / dt;
                }
            } else {
                // dt is useless set to zero to be safe
                mVelocityMeasure.Velocity().SetAll(0.0);
            }
        } else {
            // user requested to not compute velocities, likely
            // because previousPosition can't be trusted (e.g. after
            // coupling change)
            mVelocityMeasure.Velocity().SetAll(0.0);
        }
    }

    // update state data
    mStateJointMeasure.Position().Assign(mPositionMeasure.Position(), mNumberOfActiveJoints);
    mStateJointMeasure.Velocity().Assign(mVelocityMeasure.Velocity(), mNumberOfActiveJoints);
    mStateJointMeasure.Effort().Assign(mEffortMeasure, mNumberOfActiveJoints);
}

void mtsPID::SetEffortLocal(const vctDoubleVec & effort)
{
    mEffortPIDCommand.ForceTorque().Assign(effort, mNumberOfActiveJoints);
    Robot.SetEffort(mEffortPIDCommand);
}

void mtsPID::CouplingEventHandler(const prmActuatorJointCoupling & coupling)
{
    // force recalculating everything based on new IO data
    // update states
    StateTable.Start(); {
        GetIOData(false); // don't estimate velocity based on previous
                          // position since that position might not be
                          // computed using same coupling
        // reset commanded based on measured to have reasonable defaults
        mStateJointCommand.Position().Assign(mStateJointMeasure.Position());
        mStateJointCommand.Effort().Assign(mStateJointMeasure.Effort());
    } StateTable.Advance();

    Events.Coupling(coupling);
}

void mtsPID::SetTrackingErrorTolerances(const vctDoubleVec & tolerances)
{
    if (tolerances.size() == mNumberOfActiveJoints) {
        mTrackingErrorTolerances.Assign(tolerances, mNumberOfActiveJoints);
    } else {
        std::string message = this->Name + ": incorrect vector size for SetTrackingErrorTolerances";
        cmnThrow(message);
    }
}

void mtsPID::ErrorEventHandler(const mtsMessage & message)
{
    if (this->mEnabled) {
        this->Enable(false);
        mInterface->SendError(this->GetName() + ": received [" + message.Message + "]");
    } else {
        mInterface->SendStatus(this->GetName() + ": received [" + message.Message + "]");
    }
}
