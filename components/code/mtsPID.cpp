/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen
  Created on: 2013-02-22

  (C) Copyright 2013-2019 Johns Hopkins University (JHU), All Rights Reserved.

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
    mApplyEffortLimit = false,
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
    StateTable.AddData(mEnabled, "Enabled");
    StateTable.AddData(mJointsEnabled, "JointsEnabled");

    // measures are timestamped by the IO level
    mPositionMeasure.SetAutomaticTimestamp(false);
    StateTable.AddData(mPositionMeasure, "PositionMeasure");
    mVelocityMeasure.SetAutomaticTimestamp(false);
    StateTable.AddData(mVelocityMeasure, "VelocityMeasure");
    StateTable.AddData(mEffortMeasure, "EffortMeasure");
    StateTable.AddData(mCheckPositionLimit, "CheckPositionLimit");
    StateTable.AddData(mGains.Offset, "EffortOffset");

    mStateJointMeasure.SetAutomaticTimestamp(false);
    StateTable.AddData(mStateJointMeasure, "StateJointMeasure");
    StateTable.AddData(mStateJointCommand, "StateJointCommand");

    // configuration state table with occasional start/advance
    mConfigurationStateTable.AddData(mGains.Kp, "Kp");
    mConfigurationStateTable.AddData(mGains.Kd, "Kd");
    mConfigurationStateTable.AddData(mGains.Ki, "Ki");
    mConfigurationStateTable.AddData(mConfigurationJoint, "ConfigurationJoint");
    StateTable.AddData(mTrackingErrorEnabled, "EnableTrackingError"); // that table advances automatically
    mConfigurationStateTable.AddData(mTrackingErrorTolerances, "TrackingErrorTolerances");

    mInterface = AddInterfaceProvided("Controller");
    mInterface->AddMessageEvents();
    if (mInterface) {
        mInterface->AddCommandVoid(&mtsPID::ResetController, this, "ResetController");
        mInterface->AddCommandWrite(&mtsPID::Enable, this, "Enable", false);
        mInterface->AddCommandWrite(&mtsPID::EnableJoints, this, "EnableJoints", mJointsEnabled);
        mInterface->AddCommandWrite(&mtsPID::EnableEffortMode, this, "EnableTorqueMode", mEffortMode);
        mInterface->AddCommandReadState(StateTable, mEnabled, "Enabled");
        mInterface->AddCommandReadState(StateTable, mJointsEnabled, "JointsEnabled");

        // set goals
        mInterface->AddCommandWrite(&mtsPID::SetDesiredPosition, this, "SetPositionJoint", prmPositionJointSet());
        mInterface->AddCommandWrite(&mtsPID::SetFeedForward, this, "SetFeedForwardJoint", prmForceTorqueJointSet());
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

        // Get joint configuration
        mInterface->AddCommandReadState(mConfigurationStateTable, mConfigurationJoint, "GetConfigurationJoint");

        // Error tracking
        mInterface->AddCommandWriteState(StateTable, mTrackingErrorEnabled, "EnableTrackingError");
        mInterface->AddCommandReadState(StateTable, mTrackingErrorEnabled, "TrackingErrorEnabled");
        mInterface->AddCommandWrite(&mtsPID::SetTrackingErrorTolerances, this, "SetTrackingErrorTolerances");

        // Set PID gains
        mInterface->AddCommandWrite(&mtsPID::SetPGain, this, "SetPGain", mGains.Kp);
        mInterface->AddCommandWrite(&mtsPID::SetDGain, this, "SetDGain", mGains.Kd);
        mInterface->AddCommandWrite(&mtsPID::SetIGain, this, "SetIGain", mGains.Ki);

        // Set joint configuration
        mInterface->AddCommandWrite(&mtsPID::SetConfigurationJoint, this, "SetConfigurationJoint", mConfigurationJoint);

        // Events
        mInterface->AddEventWrite(Events.Enabled, "Enabled", false);
        mInterface->AddEventWrite(Events.EnabledJoints, "EnabledJoints", vctBoolVec());
        mInterface->AddEventWrite(Events.PositionLimit, "PositionLimit", vctBoolVec());
    }
}

void mtsPID::Configure(const std::string & filename)
{
    mConfigurationStateTable.Start();

    CMN_LOG_CLASS_INIT_VERBOSE << this->GetName() << " Configure: using " << filename << std::endl;
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
        CMN_LOG_CLASS_INIT_ERROR << this->GetName() << " Configure: wrong controller type" << std::endl;
        mConfigurationStateTable.Advance();
        return;
    } else if (interface != "JointTorqueInterface") {
        CMN_LOG_CLASS_INIT_ERROR << this->GetName() << " Configure: wrong interface. Require JointTorqueInterface" << std::endl;
        mConfigurationStateTable.Advance();
        return;
    } else if (numberOfJoints < 0) {
        CMN_LOG_CLASS_INIT_ERROR << this->GetName() << " Configure: invalid number of joints" << std::endl;
        mConfigurationStateTable.Advance();
        return;
    }
    mNumberOfJoints = static_cast<size_t>(numberOfJoints);

    // feedback
    mPositionMeasure.Position().SetSize(mNumberOfJoints, 0.0);
    mEffortMeasure.SetSize(mNumberOfJoints, 0.0);
    mEffortPIDCommand.ForceTorque().SetSize(mNumberOfJoints, 0.0);
    mVelocityMeasure.Velocity().SetSize(mNumberOfJoints, 0.0);
    mPositionMeasurePrevious.Position().SetSize(mNumberOfJoints, 0.0);
    mFeedForward.ForceTorque().SetSize(mNumberOfJoints, 0.0);

    // read data from xml file
    char context[64];

    mConfigurationJoint.Type().SetSize(numberOfJoints);

    // first loop to find inactive joints
    for (int i = 0; i < numberOfJoints; i++) {
        // joint
        sprintf(context, "controller/joints/joint[%d]", i + 1);

        // type
        std::string type;
        prmJointType jointType;
        config.GetXMLValue(context, "@type", type);
        if (type == "Revolute") {
            jointType = PRM_JOINT_REVOLUTE;
        } else if (type == "Prismatic") {
            jointType = PRM_JOINT_PRISMATIC;
        } else if (type == "Inactive") {
            jointType = PRM_JOINT_INACTIVE;
        } else {
            CMN_LOG_CLASS_INIT_ERROR << this->GetName() << " Configure: joint " << i << " in file: "
                                     << filename
                                     << " needs a \"type\", either \"Revolute\" or \"Prismatic\""
                                     << std::endl;
            mConfigurationStateTable.Advance();
            return;
        }

        // make sure we update the number of active joints
        if (jointType == PRM_JOINT_INACTIVE) {
            hasInactiveJoints = true;
        } else {
            // we found an inactive joint after an active one, this is not supported
            if (hasInactiveJoints) {
                CMN_LOG_CLASS_INIT_ERROR << this->GetName() << " Configure: joint " << i << " in file: "
                                         << filename
                                         << " has is not \"Inactive\" but is defined after an \"Inactive\" joint, this is not supported"
                                         << std::endl;
                mConfigurationStateTable.Advance();
                return;
            }
            mNumberOfActiveJoints++;
        }
        // save joint type in joint state
        mConfigurationJoint.Type().at(i) = jointType;
    }

    // resize Types so they will match size of all other vectors in States
    mConfigurationJoint.Type().resize(mNumberOfActiveJoints);
    mEffortUserCommand.ForceTorque().SetSize(mNumberOfActiveJoints, 0.0);

    // size all vectors specific to active joints
    mGains.Kp.SetSize(mNumberOfActiveJoints);
    mGains.Kd.SetSize(mNumberOfActiveJoints);
    mGains.Ki.SetSize(mNumberOfActiveJoints);
    mGains.Offset.SetSize(mNumberOfActiveJoints, 0.0);
    mConfigurationJoint.PositionMin().SetSize(mNumberOfActiveJoints, 0.0);
    mConfigurationJoint.PositionMax().SetSize(mNumberOfActiveJoints, 0.0);
    mConfigurationJoint.EffortMin().SetSize(mNumberOfActiveJoints, 0.0);
    mConfigurationJoint.EffortMax().SetSize(mNumberOfActiveJoints, 0.0);
    mPositionLimitFlag.SetSize(mNumberOfActiveJoints);
    mPositionLimitFlag.SetAll(false);
    mPositionLimitFlagPrevious.ForceAssign(mPositionLimitFlag);
    mJointsEnabled.SetSize(mNumberOfActiveJoints);
    mJointsEnabled.SetAll(true);

    // errors
    mError.SetSize(mNumberOfActiveJoints);
    mIError.SetSize(mNumberOfActiveJoints);
    ResetController();

    mIErrorLimitMin.SetSize(mNumberOfActiveJoints, cmnTypeTraits<double>::MinNegativeValue());
    mIErrorLimitMax.SetSize(mNumberOfActiveJoints, cmnTypeTraits<double>::MaxPositiveValue());

    // default 1.0: no effect
    mIErrorForgetFactor.SetSize(mNumberOfActiveJoints);
    mIErrorForgetFactor.SetAll(1.0);

    // default: use regular PID
    mNonLinear.SetSize(mNumberOfActiveJoints);
    mNonLinear.SetAll(0.0);

    mDeadBand.SetSize(mNumberOfActiveJoints);
    mDeadBand.SetAll(0.0);

    // effort mode
    mEffortMode.SetSize(mNumberOfActiveJoints);
    mEffortMode.SetAll(false);

    // tracking error
    mTrackingErrorEnabled = false;
    mTrackingErrorTolerances.SetSize(mNumberOfActiveJoints, 0.0);
    mTrackingErrorFlag.SetSize(mNumberOfActiveJoints, false);
    mPreviousTrackingErrorFlag.ForceAssign(mTrackingErrorFlag);

    // loop to get configuration data except type
    for (int i = 0; i < mNumberOfActiveJoints; i++) {
        // joint
        sprintf(context, "controller/joints/joint[%d]", i + 1);

        // name
        std::string name;
        config.GetXMLValue(context, "@name", name);
        // names in joint states
        mStateJointMeasure.Name().resize(mNumberOfActiveJoints);
        mStateJointCommand.Name().resize(mNumberOfActiveJoints);
        mConfigurationJoint.Name().resize(mNumberOfActiveJoints);
        mStateJointMeasure.Name().at(i) = name;
        mStateJointCommand.Name().at(i) = name;
        mConfigurationJoint.Name().at(i) = name;

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
            config.GetXMLValue(context, "pos/@LowerLimit", mConfigurationJoint.PositionMin().at(i));
            config.GetXMLValue(context, "pos/@UpperLimit", mConfigurationJoint.PositionMax().at(i));
            if (tmpUnits == "deg") {
                mConfigurationJoint.PositionMin().at(i) *= cmnPI_180;
                mConfigurationJoint.PositionMax().at(i) *= cmnPI_180;
            } else if (tmpUnits == "mm") {
                mConfigurationJoint.PositionMin().at(i) *= cmn_mm;
                mConfigurationJoint.PositionMax().at(i) *= cmn_mm;
            }
        } else {
            mCheckPositionLimit = false;
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
                               << "JntLowerLimit" << mConfigurationJoint.PositionMin() << std::endl
                               << "JntUpperLimit" << mConfigurationJoint.PositionMax() << std::endl
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
            CMN_LOG_CLASS_INIT_ERROR << this->GetName() << " Startup: Robot interface isn't connected properly, unable to get joint type.  Function call returned: "
                                     << result << std::endl;
        } else {
            for (size_t index = 0;
                 index < mNumberOfActiveJoints;
                 ++index) {
                if (jointType.at(index) != mConfigurationJoint.Type().at(index)) {
                    std::string message = this->GetName() + " Startup: joint types from IO don't match types from configuration files for " + this->GetName();
                    CMN_LOG_CLASS_INIT_ERROR << message << std::endl
                                             << "From IO:     " << jointType << std::endl
                                             << "From config: " << mConfigurationJoint.Type() << std::endl;
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
    vctDoubleVec::const_iterator feedForward = mFeedForward.ForceTorque().begin();
    vctDoubleVec::const_iterator nonLinear = mNonLinear.begin();
    vctDoubleVec::const_iterator effortLowerLimit = mConfigurationJoint.EffortMin().begin();
    vctDoubleVec::const_iterator effortUpperLimit = mConfigurationJoint.EffortMax().begin();

    CMN_ASSERT(mJointsEnabled.size() >= mNumberOfActiveJoints);
    CMN_ASSERT(mStateJointMeasure.Position().size() >= mNumberOfActiveJoints);
    CMN_ASSERT(mStateJointMeasure.Velocity().size() >= mNumberOfActiveJoints);
    CMN_ASSERT(mStateJointCommand.Position().size() >= mNumberOfActiveJoints);
    CMN_ASSERT(mStateJointCommand.Effort().size() >= mNumberOfActiveJoints);
    CMN_ASSERT(mEffortMode.size() >= mNumberOfActiveJoints);
    CMN_ASSERT(mEffortUserCommand.ForceTorque().size() >= mNumberOfActiveJoints);
    CMN_ASSERT(mError.size() >= mNumberOfActiveJoints);
    CMN_ASSERT(mDeadBand.size() >= mNumberOfActiveJoints);
    CMN_ASSERT(mTrackingErrorTolerances.size() >= mNumberOfActiveJoints);
    CMN_ASSERT(mPositionLimitFlag.size() >= mNumberOfActiveJoints);
    CMN_ASSERT(mTrackingErrorFlag.size() >= mNumberOfActiveJoints);
    CMN_ASSERT(mPreviousTrackingErrorFlag.size() >= mNumberOfActiveJoints);
    CMN_ASSERT(mIError.size() >= mNumberOfActiveJoints);
    CMN_ASSERT(mIErrorForgetFactor.size() >= mNumberOfActiveJoints);
    CMN_ASSERT(mIErrorLimitMin.size() >= mNumberOfActiveJoints);
    CMN_ASSERT(mIErrorLimitMax.size() >= mNumberOfActiveJoints);
    CMN_ASSERT(mGains.Kp.size() >= mNumberOfActiveJoints);
    CMN_ASSERT(mGains.Ki.size() >= mNumberOfActiveJoints);
    CMN_ASSERT(mGains.Kd.size() >= mNumberOfActiveJoints);
    CMN_ASSERT(mGains.Offset.size() >= mNumberOfActiveJoints);
    CMN_ASSERT(mNonLinear.size() >= mNumberOfActiveJoints);
    CMN_ASSERT(mConfigurationJoint.EffortMin().size() >= mNumberOfActiveJoints);
    CMN_ASSERT(mConfigurationJoint.EffortMax().size() >= mNumberOfActiveJoints);

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
             ++feedForward,
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
                if (mTrackingErrorEnabled) {
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

                // finally, add feedForward
                *commandEffort += *feedForward;

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
    if (mTrackingErrorEnabled && anyTrackingError) {
        Enable(false);
        if (newTrackingError) {
            std::string message = this->GetName() + ": tracking error, mask (1 for error): ";
            message.append(mTrackingErrorFlag.ToString());
            mInterface->SendError(message);
            CMN_LOG_CLASS_RUN_ERROR << message << std::endl
                                    << "errors:     " << mError << std::endl
                                    << "tolerances: " << mTrackingErrorTolerances << std::endl
                                    << "measure:    " << mPositionMeasure.Position() << std::endl
                                    << "command:    " << mStateJointCommand.Position() << std::endl;
        }
    }

    // make sure we apply efforts only if enabled
    if (mEnabled) {
        SetEffortLocal(mStateJointCommand.Effort());
    }

    // for simulated mode
    if (mIsSimulated) {
        mPositionMeasure.SetValid(true);
        mPositionMeasure.Position().Assign(mStateJointCommand.Position(), mNumberOfActiveJoints);
        mPositionMeasure.SetTimestamp(StateTable.GetTic());
    }
}


void mtsPID::Cleanup(void)
{
    // cleanup
    mStateJointCommand.Effort().SetAll(0.0);
    SetEffortLocal(mStateJointCommand.Effort());
}

void mtsPID::SetSimulated(void)
{
    mIsSimulated = true;
    // in simulation mode, we don't need IO
    RemoveInterfaceRequired("RobotJointTorqueInterface");
}

void mtsPID::SetPGain(const vctDoubleVec & gain)
{
    if (SizeMismatch(gain.size(), "SetPGain")) {
        return;
    }
    mConfigurationStateTable.Start();
    mGains.Kp.Assign(gain, mNumberOfActiveJoints);
    mConfigurationStateTable.Advance();
}

void mtsPID::SetDGain(const vctDoubleVec & gain)
{
    if (SizeMismatch(gain.size(), "SetDGain")) {
        return;
    }
    mConfigurationStateTable.Start();
    mGains.Kd.Assign(gain, mNumberOfActiveJoints);
    mConfigurationStateTable.Advance();
}

void mtsPID::SetIGain(const vctDoubleVec & gain)
{
    if (SizeMismatch(gain.size(), "SetIGain")) {
        return;
    }
    mConfigurationStateTable.Start();
    mGains.Ki.Assign(gain, mNumberOfActiveJoints);
    mConfigurationStateTable.Advance();
}

void mtsPID::SetConfigurationJoint(const prmConfigurationJoint & configuration)
{
    if (SizeMismatch(configuration.Name().size(), "SetConfigurationJoint")) {
        return;
    }
    mConfigurationStateTable.Start();
    // min position
    if (configuration.PositionMin().size() != 0) {
        if (SizeMismatch(configuration.PositionMin().size(), "SetConfigurationJoint::PositionMin")) {
            return;
        }
        mConfigurationJoint.PositionMin().Assign(configuration.PositionMin());
    }
    // max position
    if (configuration.PositionMax().size() != 0) {
        if (SizeMismatch(configuration.PositionMax().size(), "SetConfigurationJoint::PositionMax")) {
            return;
        }
        mConfigurationJoint.PositionMax().Assign(configuration.PositionMax());
    }
    // min effort
    if (configuration.EffortMin().size() != 0) {
        if (SizeMismatch(configuration.EffortMin().size(), "SetConfigurationJoint::EffortMin")) {
            return;
        }
        mConfigurationJoint.EffortMin().Assign(configuration.EffortMin());
    }
    // max effort
    if (configuration.EffortMax().size() != 0) {
        if (SizeMismatch(configuration.EffortMax().size(), "SetConfigurationJoint::EffortMax")) {
            return;
        }
        mConfigurationJoint.EffortMax().Assign(configuration.EffortMax());
    }
    mConfigurationStateTable.Advance();

    // consistency checks
    CheckLowerUpper(mConfigurationJoint.PositionMin(), mConfigurationJoint.PositionMax(), "SetPositionLowerLimit");
    CheckLowerUpper(mConfigurationJoint.EffortMin(), mConfigurationJoint.EffortMax(), "SetEffortLowerLimit");
    mApplyEffortLimit = mConfigurationJoint.EffortMin().Any() && mConfigurationJoint.EffortMax().Any();

    CMN_LOG_CLASS_INIT_VERBOSE << this->GetName() << "::SetConfigurationJoint: called with "
                               << configuration << std::endl;
}

void mtsPID::SetMinIErrorLimit(const vctDoubleVec & iminlim)
{
    if (SizeMismatch(iminlim.size(), "SetMinIErrorLimit")) {
        return;
    }
    mConfigurationStateTable.Start();
    mIErrorLimitMin.Assign(iminlim, mNumberOfActiveJoints);
    mConfigurationStateTable.Advance();
}

void mtsPID::SetMaxIErrorLimit(const vctDoubleVec & imaxlim)
{
    if (SizeMismatch(imaxlim.size(), "SetMaxIErrorLimit")) {
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
    CMN_LOG_CLASS_RUN_VERBOSE << this->GetName() << " Reset Controller" << std::endl;
    mError.SetAll(0.0);
    mIError.SetAll(0.0);
    Enable(false);
}

void mtsPID::SetDesiredPosition(const prmPositionJointSet & command)
{
    if (SizeMismatch(command.Goal().size(), "SetDesiredPosition")) {
        return;
    }

    mStateJointCommand.Position().Assign(command.Goal(), mNumberOfActiveJoints);
    mCommandTime = command.Timestamp(); // mStateJointCommand timestamp is set by this class so can't use it later

    if (mCheckPositionLimit) {
        bool limitReached = false;
        vctDoubleVec::const_iterator upper = mConfigurationJoint.PositionMax().begin();
        vctDoubleVec::const_iterator lower = mConfigurationJoint.PositionMin().begin();
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
                std::string message = this->GetName() + ": position limit, mask (1 for limit): ";
                message.append(mPositionLimitFlag.ToString());
                mInterface->SendWarning(message);
                CMN_LOG_CLASS_RUN_WARNING << message
                                          << ", \n requested: " << mStateJointCommand.Position()
                                          << ", \n lower limits: " << mConfigurationJoint.PositionMin()
                                          << ", \n upper limits: " << mConfigurationJoint.PositionMax()
                                          << std::endl;
            }
        }
    }
}


void mtsPID::SetFeedForward(const prmForceTorqueJointSet & feedForward)
{
    if (SizeMismatch(feedForward.ForceTorque().size(), "SetFeedForward")) {
        return;
    }
    mFeedForward.ForceTorque().Assign(feedForward.ForceTorque());
}


void mtsPID::SetDesiredEffort(const prmForceTorqueJointSet & command)
{
    if (SizeMismatch(command.ForceTorque().size(), "SetDesiredEffort")) {
        return;
    }
    mEffortUserCommand.ForceTorque().Assign(command.ForceTorque());
}


void mtsPID::Enable(const bool & enable)
{
    if (enable == mEnabled) {
        // trigger Enabled
        Events.Enabled(mEnabled);
        return;
    }

    mEnabled = enable;

    // set effort to 0
    mStateJointCommand.Effort().SetAll(0.0);
    SetEffortLocal(mStateJointCommand.Effort());

    // reset error flags
    if (enable) {
        mPreviousTrackingErrorFlag.SetAll(false);
        mTrackingErrorFlag.SetAll(false);
        mPositionLimitFlagPrevious.SetAll(false);
        mPositionLimitFlag.SetAll(false);
        mPreviousCommandTime = 0.0;
        mFeedForward.ForceTorque().SetAll(0.0);
    }
    // trigger Enabled
    Events.Enabled(mEnabled);
}


void mtsPID::EnableJoints(const vctBoolVec & enable)
{
    if (SizeMismatch(enable.size(), "EnableJoints")) {
        return;
    }
    mJointsEnabled.Assign(enable, mNumberOfActiveJoints);
    Events.EnabledJoints(enable);
}


void mtsPID::EnableEffortMode(const vctBoolVec & enable)
{
    if (SizeMismatch(enable.size(), "EnableEffortMode")) {
        return;
    }
    // save preference
    mEffortMode.Assign(enable, mNumberOfActiveJoints);
    // reset effort to 0
    mStateJointCommand.Effort().SetAll(0.0);
    mFeedForward.ForceTorque().SetAll(0.0);
}

void mtsPID::SetCoupling(const prmActuatorJointCoupling & coupling)
{
    // we assume the user has disabled whatever joints needed to be
    // disabled so we just past the request to IO level
    Robot.SetCoupling(coupling);
}

void mtsPID::GetIOData(const bool computeVelocity)
{
    // get data from IO if not in simulated mode.  When is simulation
    // mode, position come from the user/client and is found in
    // mStateJointCommand
    if (mIsSimulated) {
        // check that position is not too old
        if ((StateTable.GetTic() - mCommandTime) > 20.0 * cmn_ms) {
            mVelocityMeasure.Velocity().SetAll(0.0);
        } else {
            // evaluate velocity based on positions sent by arm/client
            const double dt = mCommandTime - mPreviousCommandTime;
            if (dt > 0) {
                vctDoubleVec::const_iterator currentPosition = mStateJointCommand.Position().begin();
                vctDoubleVec::const_iterator previousPosition = mPositionMeasurePrevious.Position().begin();
                vctBoolVec::const_iterator effortMode = mEffortMode.begin();
                vctDoubleVec::iterator velocity;
                const vctDoubleVec::iterator end = mVelocityMeasure.Velocity().end();
                for (velocity = mVelocityMeasure.Velocity().begin();
                     velocity != end;
                     ++currentPosition,
                         ++previousPosition,
                         ++effortMode,
                         ++velocity) {
                    if (*effortMode) {
                        *velocity = 0.0;
                    } else {
                        *velocity = (*currentPosition - *previousPosition) / dt;
                    }
                }
            }
        }
        // timestamp using last know position
        mPositionMeasurePrevious.Position().Assign(mStateJointCommand.Position(), mNumberOfActiveJoints);
        mPreviousCommandTime = mCommandTime;
    }

    // talking to actual robot
    else {
        Robot.GetFeedbackPosition(mPositionMeasure);
        Robot.GetFeedbackEffort(mEffortMeasure);

        if (computeVelocity) {
            // update velocities from robot, this should be most cases
            if (Robot.GetFeedbackVelocity.IsValid()) {
                Robot.GetFeedbackVelocity(mVelocityMeasure);
            } else {
                // IO level doesn't provide velocities
                // so estimate it from measured positions.  this
                // estimation is very simple and likely noisy so try to
                // avoid this case as much as possible
                const double dt = mPositionMeasure.Timestamp() - mPositionMeasurePrevious.Timestamp();
                if (dt > 0) {
                    vctDoubleVec::const_iterator currentPosition = mPositionMeasure.Position().begin();
                    vctDoubleVec::const_iterator previousPosition = mPositionMeasurePrevious.Position().begin();
                    vctDoubleVec::iterator velocity;
                    const vctDoubleVec::iterator end = mVelocityMeasure.Velocity().end();
                    for (velocity = mVelocityMeasure.Velocity().begin();
                         velocity != end;
                         ++currentPosition,
                             ++previousPosition,
                             ++velocity) {
                        *velocity = (*currentPosition - *previousPosition) / dt;
                    }
                }
            } // end of software base velocity estimation based on
            // measured positions
        } else {
            // user requested to not compute velocities, likely
            // because previousPosition can't be trusted
            // (e.g. after coupling change)
            mVelocityMeasure.Velocity().SetAll(0.0);
        }

        // save previous position with timestamp
        mPositionMeasurePrevious = mPositionMeasure;
    }

    // update state data
    mStateJointMeasure.Position().Assign(mPositionMeasure.Position(), mNumberOfActiveJoints);
    mStateJointMeasure.Velocity().Assign(mVelocityMeasure.Velocity(), mNumberOfActiveJoints);
    mStateJointMeasure.Effort().Assign(mEffortMeasure, mNumberOfActiveJoints);
    mStateJointMeasure.SetTimestamp(mPositionMeasure.Timestamp());
}

void mtsPID::SetEffortLocal(const vctDoubleVec & effort)
{
    if (!mIsSimulated) {
        mEffortPIDCommand.ForceTorque().Assign(effort, mNumberOfActiveJoints);
        Robot.SetEffort(mEffortPIDCommand);
    }
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
        std::string message = this->GetName() + ": incorrect vector size for SetTrackingErrorTolerances";
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

bool mtsPID::SizeMismatch(const size_t size, const std::string & methodName)
{
    if (size != mNumberOfActiveJoints) {
        CMN_LOG_CLASS_INIT_ERROR << this->GetName() << " " << methodName << ": size mismatch, expected "
                                 << mNumberOfActiveJoints << ", received "
                                 << size << std::endl;
        Enable(false);
        mInterface->SendError(this->GetName() + "::" + methodName + ": size mismatch");
        return true;
    }
    return false;
}

void mtsPID::CheckLowerUpper(const vctDoubleVec & lower, const vctDoubleVec & upper,
                             const std::string & methodName)
{
    // it only makes sense to check if there's at least one limit set
    if (lower.Any() || upper.Any()) {
        if (lower.ElementwiseGreaterOrEqual(upper).Any()) {
            CMN_LOG_CLASS_INIT_ERROR << this->GetName() << " " << methodName
                                     << ": lower limit greater or equal to upper limit " << std::endl
                                     << " - lower: " << lower << std::endl
                                     << " - upper: " << upper << std::endl;
            mInterface->SendWarning(this->GetName() + "::" + methodName + ": lower limit greater or equal to upper limit");
        }
    }
}
