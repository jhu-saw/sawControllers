/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-22

  (C) Copyright 2013-2021 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstParameterTypes/prmPositionJointSet.h>

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
    mApplyEffortLimit = false;
    AddStateTable(&mConfigurationStateTable);
    mConfigurationStateTable.SetAutomaticAdvance(false);
}


void mtsPID::SetupInterfaces(void)
{
    // require RobotJointTorque interface
    mtsInterfaceRequired * requiredInterface = AddInterfaceRequired("RobotJointTorqueInterface");
    if (requiredInterface) {
        requiredInterface->AddFunction("SetCoupling", IO.SetCoupling);
        requiredInterface->AddFunction("ActuatorToJointPosition", IO.ActuatorToJointPosition);
        requiredInterface->AddFunction("JointToActuatorPosition", IO.JointToActuatorPosition);
        requiredInterface->AddFunction("ActuatorToJointEffort", IO.ActuatorToJointEffort);
        requiredInterface->AddFunction("JointToActuatorEffort", IO.JointToActuatorEffort);
        requiredInterface->AddFunction("configuration_js", IO.configuration_js);
        requiredInterface->AddFunction("configure_js", IO.configure_js);
        requiredInterface->AddFunction("measured_js", IO.measured_js);
        requiredInterface->AddFunction("servo_jf", IO.servo_jf);
        // event handlers
        requiredInterface->AddEventHandlerWrite(&mtsPID::CouplingEventHandler, this, "Coupling");
        requiredInterface->AddEventHandlerWrite(&mtsPID::ErrorEventHandler, this, "error");
    }

    // this should go in "write" state table
    StateTable.AddData(mEffortUserCommand, "EffortUserCommand");
    // this should go in a "read" state table
    StateTable.AddData(mEnabled, "Enabled");
    StateTable.AddData(mJointsEnabled, "JointsEnabled");

    // measures are timestamped by the IO level
    StateTable.AddData(mCheckPositionLimit, "CheckPositionLimit");
    StateTable.AddData(mGains.Offset, "EffortOffset");

    m_measured_js.SetAutomaticTimestamp(false);
    StateTable.AddData(m_measured_js, "measured_js");
    StateTable.AddData(m_setpoint_js, "setpoint_js");

    // configuration state table with occasional start/advance
    mConfigurationStateTable.AddData(mGains.Kp, "Kp");
    mConfigurationStateTable.AddData(mGains.Kd, "Kd");
    mConfigurationStateTable.AddData(mGains.Ki, "Ki");
    mConfigurationStateTable.AddData(m_configuration_js, "configuration_js");
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
        mInterface->AddCommandWrite(&mtsPID::servo_jp, this, "servo_jp", prmPositionJointSet());
        mInterface->AddCommandWrite(&mtsPID::feed_forward_jf, this, "feed_forward_jf", prmForceTorqueJointSet());
        mInterface->AddCommandWrite(&mtsPID::servo_jf, this, "servo_jf", prmForceTorqueJointSet());

        // ROS compatible joint state
        mInterface->AddCommandReadState(StateTable, m_measured_js, "measured_js");
        mInterface->AddCommandReadState(StateTable, m_setpoint_js, "setpoint_js");

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
        mInterface->AddCommandReadState(mConfigurationStateTable, m_configuration_js, "configuration_js");

        // Error tracking
        mInterface->AddCommandWriteState(StateTable, mTrackingErrorEnabled, "EnableTrackingError");
        mInterface->AddCommandReadState(StateTable, mTrackingErrorEnabled, "TrackingErrorEnabled");
        mInterface->AddCommandWrite(&mtsPID::SetTrackingErrorTolerances, this, "SetTrackingErrorTolerances");

        // Set PID gains
        mInterface->AddCommandWrite(&mtsPID::SetPGain, this, "SetPGain", mGains.Kp);
        mInterface->AddCommandWrite(&mtsPID::SetDGain, this, "SetDGain", mGains.Kd);
        mInterface->AddCommandWrite(&mtsPID::SetIGain, this, "SetIGain", mGains.Ki);

        // Set joint configuration
        mInterface->AddCommandWrite(&mtsPID::configure_js, this, "configure_js", m_configuration_js);

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

    // check type, interface and number of joints
    std::string type, interface;
    config.GetXMLValue("/controller", "@type", type, "");
    config.GetXMLValue("/controller", "@interface", interface, "");
    int numberOfJoints;
    config.GetXMLValue("/controller", "@numofjoints", numberOfJoints, -1);
    if (type != "PID") {
        CMN_LOG_CLASS_INIT_ERROR << this->GetName() << " Configure: wrong controller type" << std::endl;
        exit(EXIT_FAILURE);
    } else if (interface != "JointTorqueInterface") {
        CMN_LOG_CLASS_INIT_ERROR << this->GetName() << " Configure: wrong interface. Require JointTorqueInterface" << std::endl;
        exit(EXIT_FAILURE);
    } else if (numberOfJoints < 0) {
        CMN_LOG_CLASS_INIT_ERROR << this->GetName() << " Configure: invalid number of joints" << std::endl;
        exit(EXIT_FAILURE);
    }
    mNumberOfJoints = static_cast<size_t>(numberOfJoints);

    // actuator setpoint
    m_pre_coupling_setpoint_ap.SetSize(mNumberOfJoints);
    m_pre_coupling_setpoint_af.SetSize(mNumberOfJoints);

    // feedback
    mEffortPIDCommand.ForceTorque().SetSize(mNumberOfJoints, 0.0);
    m_feed_forward_jf.ForceTorque().SetSize(mNumberOfJoints, 0.0);
    mEffortUserCommand.ForceTorque().SetSize(mNumberOfJoints, 0.0);

    // size all vectors
    mGains.Kp.SetSize(mNumberOfJoints);
    mGains.Kd.SetSize(mNumberOfJoints);
    mGains.Ki.SetSize(mNumberOfJoints);
    mGains.Offset.SetSize(mNumberOfJoints, 0.0);

    m_configuration_js.Name().SetSize(mNumberOfJoints);
    m_configuration_js.Type().SetSize(mNumberOfJoints);
    m_configuration_js.PositionMin().SetSize(mNumberOfJoints, 0.0);
    m_configuration_js.PositionMax().SetSize(mNumberOfJoints, 0.0);
    m_configuration_js.EffortMin().SetSize(mNumberOfJoints, 0.0);
    m_configuration_js.EffortMax().SetSize(mNumberOfJoints, 0.0);

    m_measured_js.Name().SetSize(mNumberOfJoints);
    m_measured_js.Position().SetSize(mNumberOfJoints, 0.0);
    m_measured_js.Velocity().SetSize(mNumberOfJoints, 0.0);
    m_measured_js.Effort().SetSize(mNumberOfJoints, 0.0);

    m_setpoint_js.Name().SetSize(mNumberOfJoints);
    m_setpoint_js.Position().SetSize(mNumberOfJoints, 0.0);
    m_setpoint_js.Velocity().SetSize(0); // we don't support desired velocity
    m_setpoint_js.Effort().SetSize(mNumberOfJoints, 0.0);

    mPositionLimitFlag.SetSize(mNumberOfJoints);
    mPositionLimitFlag.SetAll(false);
    mPositionLimitFlagPrevious.ForceAssign(mPositionLimitFlag);
    mJointsEnabled.SetSize(mNumberOfJoints);
    mJointsEnabled.SetAll(true);

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
    mTrackingErrorEnabled = false;
    mTrackingErrorTolerances.SetSize(mNumberOfJoints, 0.0);
    mTrackingErrorFlag.SetSize(mNumberOfJoints, false);
    mPreviousTrackingErrorFlag.ForceAssign(mTrackingErrorFlag);

    // xml context
    char context[128];

    // loop to get configuration data except type
    for (size_t i = 0; i < mNumberOfJoints; i++) {
        // joint
        sprintf(context, "controller/joints/joint[%zu]", i + 1);

        // name
        std::string name;
        config.GetXMLValue(context, "@name", name);
        // names in joint states
        m_measured_js.Name().at(i) = name;
        m_setpoint_js.Name().at(i) = name;
        m_configuration_js.Name().at(i) = name;

        // type
        std::string type;
        config.GetXMLValue(context, "@type", type);
        if (type == "Revolute") {
            m_configuration_js.Type().at(i) = PRM_JOINT_REVOLUTE;
        } else if (type == "Prismatic") {
            m_configuration_js.Type().at(i) = PRM_JOINT_PRISMATIC;
        } else {
            CMN_LOG_CLASS_INIT_ERROR << this->GetName() << " Configure: joint " << i << " in file: "
                                     << filename
                                     << " needs a \"type\", either \"Revolute\" or \"Prismatic\".  Note: type \"Inactive\" is not supported anymore."
                                     << std::endl;
            exit(EXIT_FAILURE);
        }

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
            config.GetXMLValue(context, "pos/@LowerLimit", m_configuration_js.PositionMin().at(i));
            config.GetXMLValue(context, "pos/@UpperLimit", m_configuration_js.PositionMax().at(i));
            if (tmpUnits == "deg") {
                m_configuration_js.PositionMin().at(i) *= cmnPI_180;
                m_configuration_js.PositionMax().at(i) *= cmnPI_180;
            } else if (tmpUnits == "mm") {
                m_configuration_js.PositionMin().at(i) *= cmn_mm;
                m_configuration_js.PositionMax().at(i) *= cmn_mm;
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
                               << "JntLowerLimit" << m_configuration_js.PositionMin() << std::endl
                               << "JntUpperLimit" << m_configuration_js.PositionMax() << std::endl
                               << "Deadband: " << mDeadBand << std::endl
                               << "minILimit: " << mIErrorLimitMin << std::endl
                               << "maxILimit: " << mIErrorLimitMax << std::endl
                               << "elimit: " << mTrackingErrorTolerances << std::endl
                               << "forget: " << mIErrorForgetFactor << std::endl;

    mConfigurationStateTable.Advance();

    // now that we know the sizes of vectors, create interfaces
    this->SetupInterfaces();
}

void mtsPID::Startup(void)
{
    // get joint type from IO and check against values from PID config file
    if (!mIsSimulated) {
        mtsExecutionResult result;
        prmConfigurationJoint io_configuration_js;
        result = IO.configuration_js(io_configuration_js);
        if (!result) {
            CMN_LOG_CLASS_INIT_ERROR << this->GetName() << " Startup: Robot interface isn't connected properly, unable to get joint configuration.  Function call returned: "
                                     << result << std::endl;
        } else {
            for (size_t index = 0;
                 index < mNumberOfJoints;
                 ++index) {
                if (io_configuration_js.Type().at(index) != m_configuration_js.Type().at(index)) {
                    std::string message = this->GetName() + " Startup: joint types from IO don't match types from configuration files for " + this->GetName();
                    CMN_LOG_CLASS_INIT_ERROR << message << std::endl
                                             << "From IO:     " << io_configuration_js.Type() << std::endl
                                             << "From config: " << m_configuration_js.Type() << std::endl;
                    cmnThrow("PID::" + message);
                }
            }
            // check that names are also set properly
            if (io_configuration_js.Name().size() == m_configuration_js.Name().size()) {
                if (io_configuration_js.Name() != m_configuration_js.Name()) {
                    std::string message = this->GetName() + " Startup: joint names from IO don't match names from configuration files for " + this->GetName();
                    CMN_LOG_CLASS_INIT_ERROR << message << std::endl
                                             << "From IO:     " << io_configuration_js.Name() << std::endl
                                             << "From config: " << m_configuration_js.Name() << std::endl;
                    cmnThrow("PID::" + message);
                }
            } else if (io_configuration_js.Name().size() == 0) {
                // IO doesn't have proper names, let's configure them
                IO.configure_js(m_configuration_js);
            }
        }
    }
}

void mtsPID::Run(void)
{
    // first process events from IO (likely errors)
    ProcessQueuedEvents();

    // get data from IO if not in simulated mode
    GetIOData(true); // compute velocity if needed

    // now that we have recent data (e.g. position after
    // calibration/coupling...), deal with user commands
    ProcessQueuedCommands();

    // initialize variables
    bool anyTrackingError = false;
    bool newTrackingError = false;

    // loop on all joints
    vctBoolVec::const_iterator enabled = mJointsEnabled.begin();
    vctDoubleVec::const_iterator measurePosition = m_measured_js.Position().begin();
    vctDoubleVec::const_iterator measureVelocity = m_measured_js.Velocity().begin();
    vctDoubleVec::iterator commandPosition = m_setpoint_js.Position().begin();
    vctDoubleVec::iterator commandEffort = m_setpoint_js.Effort().begin();
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
    vctDoubleVec::const_iterator feedForward = m_feed_forward_jf.ForceTorque().begin();
    vctDoubleVec::const_iterator nonLinear = mNonLinear.begin();
    vctDoubleVec::const_iterator effortLowerLimit = m_configuration_js.EffortMin().begin();
    vctDoubleVec::const_iterator effortUpperLimit = m_configuration_js.EffortMax().begin();

    CMN_ASSERT(mJointsEnabled.size() == mNumberOfJoints);
    CMN_ASSERT(m_measured_js.Position().size() == mNumberOfJoints);
    CMN_ASSERT(m_measured_js.Velocity().size() == mNumberOfJoints);
    CMN_ASSERT(m_setpoint_js.Position().size() == mNumberOfJoints);
    CMN_ASSERT(m_setpoint_js.Effort().size() == mNumberOfJoints);
    CMN_ASSERT(mEffortMode.size() == mNumberOfJoints);
    CMN_ASSERT(mEffortUserCommand.ForceTorque().size() == mNumberOfJoints);
    CMN_ASSERT(mError.size() == mNumberOfJoints);
    CMN_ASSERT(mDeadBand.size() == mNumberOfJoints);
    CMN_ASSERT(mTrackingErrorTolerances.size() == mNumberOfJoints);
    CMN_ASSERT(mPositionLimitFlag.size() == mNumberOfJoints);
    CMN_ASSERT(mTrackingErrorFlag.size() == mNumberOfJoints);
    CMN_ASSERT(mPreviousTrackingErrorFlag.size() == mNumberOfJoints);
    CMN_ASSERT(mIError.size() == mNumberOfJoints);
    CMN_ASSERT(mIErrorForgetFactor.size() == mNumberOfJoints);
    CMN_ASSERT(mIErrorLimitMin.size() == mNumberOfJoints);
    CMN_ASSERT(mIErrorLimitMax.size() == mNumberOfJoints);
    CMN_ASSERT(mGains.Kp.size() == mNumberOfJoints);
    CMN_ASSERT(mGains.Ki.size() == mNumberOfJoints);
    CMN_ASSERT(mGains.Kd.size() == mNumberOfJoints);
    CMN_ASSERT(mGains.Offset.size() == mNumberOfJoints);
    CMN_ASSERT(mNonLinear.size() == mNumberOfJoints);
    CMN_ASSERT(m_configuration_js.EffortMin().size() == mNumberOfJoints);
    CMN_ASSERT(m_configuration_js.EffortMax().size() == mNumberOfJoints);

    // loop on all joints using iterators
    for (size_t i = 0;
         i < mNumberOfJoints;
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
                                    << "measured:   " << m_measured_js.Position() << std::endl
                                    << "setpoint:   " << m_setpoint_js.Position() << std::endl;
        }
    }

    // make sure we apply efforts only if enabled
    if (mEnabled) {
        SetEffortLocal(m_setpoint_js.Effort());
    }

    // for simulated mode
    if (mIsSimulated) {
        m_measured_js.SetValid(true);
        m_measured_js.Position().Assign(m_setpoint_js.Position());
        m_measured_js.SetTimestamp(StateTable.GetTic());
    }
}


void mtsPID::Cleanup(void)
{
    // cleanup
    m_setpoint_js.Effort().SetAll(0.0);
    SetEffortLocal(m_setpoint_js.Effort());
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
    mGains.Kp.Assign(gain);
    mConfigurationStateTable.Advance();
}

void mtsPID::SetDGain(const vctDoubleVec & gain)
{
    if (SizeMismatch(gain.size(), "SetDGain")) {
        return;
    }
    mConfigurationStateTable.Start();
    mGains.Kd.Assign(gain);
    mConfigurationStateTable.Advance();
}

void mtsPID::SetIGain(const vctDoubleVec & gain)
{
    if (SizeMismatch(gain.size(), "SetIGain")) {
        return;
    }
    mConfigurationStateTable.Start();
    mGains.Ki.Assign(gain);
    mConfigurationStateTable.Advance();
}

void mtsPID::configure_js(const prmConfigurationJoint & configuration)
{
    if (SizeMismatch(configuration.Name().size(), "configure_js.Name")) {
        return;
    }
    mConfigurationStateTable.Start();
    // min position
    if (configuration.PositionMin().size() != 0) {
        if (SizeMismatch(configuration.PositionMin().size(), "configure_js.PositionMin")) {
            return;
        }
        m_configuration_js.PositionMin().Assign(configuration.PositionMin());
    }
    // max position
    if (configuration.PositionMax().size() != 0) {
        if (SizeMismatch(configuration.PositionMax().size(), "configure_js.PositionMax")) {
            return;
        }
        m_configuration_js.PositionMax().Assign(configuration.PositionMax());
    }
    // min effort
    if (configuration.EffortMin().size() != 0) {
        if (SizeMismatch(configuration.EffortMin().size(), "configure_js.EffortMin")) {
            return;
        }
        m_configuration_js.EffortMin().Assign(configuration.EffortMin());
    }
    // max effort
    if (configuration.EffortMax().size() != 0) {
        if (SizeMismatch(configuration.EffortMax().size(), "configure_js.EffortMax")) {
            return;
        }
        m_configuration_js.EffortMax().Assign(configuration.EffortMax());
    }
    mConfigurationStateTable.Advance();

    // consistency checks
    CheckLowerUpper(m_configuration_js.PositionMin(), m_configuration_js.PositionMax(), "SetPositionLowerLimit");
    CheckLowerUpper(m_configuration_js.EffortMin(), m_configuration_js.EffortMax(), "SetEffortLowerLimit");
    mApplyEffortLimit = m_configuration_js.EffortMin().Any() && m_configuration_js.EffortMax().Any();


    CMN_LOG_CLASS_INIT_VERBOSE << this->GetName() << "::configure_js: called with "
                               << configuration << std::endl;
}

void mtsPID::SetMinIErrorLimit(const vctDoubleVec & iminlim)
{
    if (SizeMismatch(iminlim.size(), "SetMinIErrorLimit")) {
        return;
    }
    mConfigurationStateTable.Start();
    mIErrorLimitMin.Assign(iminlim);
    mConfigurationStateTable.Advance();
}

void mtsPID::SetMaxIErrorLimit(const vctDoubleVec & imaxlim)
{
    if (SizeMismatch(imaxlim.size(), "SetMaxIErrorLimit")) {
        return;
    }
    mConfigurationStateTable.Start();
    mIErrorLimitMax.Assign(imaxlim);
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

void mtsPID::servo_jp(const prmPositionJointSet & command)
{
    if (SizeMismatch(command.Goal().size(), "servo_jp")) {
        return;
    }

    m_setpoint_js.Position().Assign(command.Goal());
    mCommandTime = command.Timestamp(); // m_setpoint_js timestamp is set by this class so can't use it later

    if (mCheckPositionLimit) {
        bool limitReached = false;
        vctDoubleVec::const_iterator upper = m_configuration_js.PositionMax().begin();
        vctDoubleVec::const_iterator lower = m_configuration_js.PositionMin().begin();
        vctBoolVec::iterator flags = mPositionLimitFlag.begin();
        vctDoubleVec::iterator desired = m_setpoint_js.Position().begin();
        const vctDoubleVec::iterator end = m_setpoint_js.Position().end();
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
                                          << ", \n requested: " << m_setpoint_js.Position()
                                          << ", \n lower limits: " << m_configuration_js.PositionMin()
                                          << ", \n upper limits: " << m_configuration_js.PositionMax()
                                          << std::endl;
            }
        }
    }
}


void mtsPID::feed_forward_jf(const prmForceTorqueJointSet & feedForward)
{
    if (SizeMismatch(feedForward.ForceTorque().size(), "SetFeedForward")) {
        return;
    }
    m_feed_forward_jf.ForceTorque().Assign(feedForward.ForceTorque());
}


void mtsPID::servo_jf(const prmForceTorqueJointSet & command)
{
    if (SizeMismatch(command.ForceTorque().size(), "servo_jf")) {
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
    m_setpoint_js.Effort().SetAll(0.0);
    SetEffortLocal(m_setpoint_js.Effort());

    // reset error flags
    if (enable) {
        mPreviousTrackingErrorFlag.SetAll(false);
        mTrackingErrorFlag.SetAll(false);
        mPositionLimitFlagPrevious.SetAll(false);
        mPositionLimitFlag.SetAll(false);
        mPreviousCommandTime = 0.0;
        m_feed_forward_jf.ForceTorque().SetAll(0.0);
    }
    // trigger Enabled
    Events.Enabled(mEnabled);
}


void mtsPID::EnableJoints(const vctBoolVec & enable)
{
    if (SizeMismatch(enable.size(), "EnableJoints")) {
        return;
    }
    mJointsEnabled.Assign(enable);
    Events.EnabledJoints(enable);
}


void mtsPID::EnableEffortMode(const vctBoolVec & enable)
{
    if (SizeMismatch(enable.size(), "EnableEffortMode")) {
        return;
    }
    // save preference
    mEffortMode.Assign(enable);
    // reset effort to 0
    m_setpoint_js.Effort().SetAll(0.0);
    m_feed_forward_jf.ForceTorque().SetAll(0.0);
}

void mtsPID::SetCoupling(const prmActuatorJointCoupling & coupling)
{
    // we assume the user has disabled whatever joints needed to be
    // disabled so we just past the request to IO level

    // first save the setpoint in actuator space
    IO.JointToActuatorPosition(m_setpoint_js.Position(), m_pre_coupling_setpoint_ap);
    IO.JointToActuatorEffort(m_setpoint_js.Effort(), m_pre_coupling_setpoint_af);

    // then request coupling
    IO.SetCoupling(coupling);
}

void mtsPID::GetIOData(const bool computeVelocity)
{
    // get data from IO if not in simulated mode.  When is simulation
    // mode, position come from the user/client and is found in
    // m_setpoint_js
    if (mIsSimulated) {
        // check that position is not too old
        if ((StateTable.GetTic() - mCommandTime) > 20.0 * cmn_ms) {
            m_measured_js.Velocity().SetAll(0.0);
        } else {
            // evaluate velocity based on positions sent by arm/client
            const double dt = mCommandTime - mPreviousCommandTime;
            if (dt > 0) {
                vctDoubleVec::const_iterator currentPosition = m_setpoint_js.Position().begin();
                vctDoubleVec::const_iterator previousPosition = m_measured_js_previous.Position().begin();
                vctBoolVec::const_iterator effortMode = mEffortMode.begin();
                vctDoubleVec::iterator velocity;
                const vctDoubleVec::iterator end = m_measured_js.Velocity().end();
                for (velocity = m_measured_js.Velocity().begin();
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
        m_measured_js_previous = m_setpoint_js;
        mPreviousCommandTime = mCommandTime;
    }

    // talking to actual robot
    else {
        IO.measured_js(m_measured_js);
        if (computeVelocity) {
            // velocities are not provided by the robot
            if (m_measured_js.Velocity().size() == 0) {
                // IO level doesn't provide velocities
                // so estimate it from measured positions.  this
                // estimation is very simple and likely noisy so try to
                // avoid this case as much as possible
                const double dt = m_measured_js.Timestamp() - m_measured_js_previous.Timestamp();
                if (dt > 0) {
                    vctDoubleVec::const_iterator currentPosition = m_measured_js.Position().begin();
                    vctDoubleVec::const_iterator previousPosition = m_measured_js_previous.Position().begin();
                    vctDoubleVec::iterator velocity;
                    const vctDoubleVec::iterator end = m_measured_js.Velocity().end();
                    for (velocity = m_measured_js.Velocity().begin();
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
            m_measured_js.Velocity().SetAll(0.0);
        }

        // save previous position with timestamp
        m_measured_js_previous = m_measured_js;
    }
}

void mtsPID::SetEffortLocal(const vctDoubleVec & effort)
{
    if (!mIsSimulated) {
        mEffortPIDCommand.ForceTorque().Assign(effort);
        IO.servo_jf(mEffortPIDCommand);
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
        IO.ActuatorToJointPosition(m_pre_coupling_setpoint_ap, m_setpoint_js.Position());
        IO.ActuatorToJointEffort(m_pre_coupling_setpoint_af, m_setpoint_js.Effort());
    } StateTable.Advance();

    Events.Coupling(coupling);
}

void mtsPID::SetTrackingErrorTolerances(const vctDoubleVec & tolerances)
{
    if (tolerances.size() == mNumberOfJoints) {
        mTrackingErrorTolerances.Assign(tolerances);
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
    if (size != mNumberOfJoints) {
        CMN_LOG_CLASS_INIT_ERROR << this->GetName() << " " << methodName << ": size mismatch, expected "
                                 << mNumberOfJoints << ", received "
                                 << size << std::endl;
        Enable(false);
        mInterface->SendError(this->GetName() + "::" + methodName + ": size mismatch (check cisstLog)");
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
