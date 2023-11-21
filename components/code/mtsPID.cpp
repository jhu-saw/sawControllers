/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-22

  (C) Copyright 2013-2023 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

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
    AddStateTable(&mConfigurationStateTable);
    mConfigurationStateTable.SetAutomaticAdvance(false);
}


void mtsPID::SetupInterfaces(void)
{
    // require RobotJointTorque interface
    mtsInterfaceRequired * requiredInterface = AddInterfaceRequired("RobotJointTorqueInterface");
    if (requiredInterface) {
        requiredInterface->AddFunction("configuration_js", IO.configuration_js);
        requiredInterface->AddFunction("configure_js", IO.configure_js);
        requiredInterface->AddFunction("measured_js", IO.measured_js);
        requiredInterface->AddFunction("servo_jf", IO.servo_jf);
        // event handlers
        requiredInterface->AddEventHandlerWrite(&mtsPID::ErrorEventHandler, this, "error");
    }

    // this should go in "write" state table
    StateTable.AddData(mEffortUserCommand, "EffortUserCommand");
    // this should go in a "read" state table
    StateTable.AddData(m_enabled, "enabled");
    StateTable.AddData(m_joints_enabled, "joints_enabled");
    StateTable.AddData(m_use_setpoint_v, "use_setpoint_v");

    // measures are timestamped by the IO level
    StateTable.AddData(m_enforce_position_limits, "enforce_position_limits");

    m_measured_js.SetAutomaticTimestamp(false);
    StateTable.AddData(m_measured_js, "measured_js");
    StateTable.AddData(m_setpoint_js, "setpoint_js");
    StateTable.AddData(m_error_state, "error_state");

    // configuration state table with occasional start/advance
    mConfigurationStateTable.AddData(m_configuration, "configuration");
    mConfigurationStateTable.AddData(m_configuration_js, "configuration_js");
    StateTable.AddData(mTrackingErrorEnabled, "EnableTrackingError"); // that table advances automatically
    mConfigurationStateTable.AddData(mTrackingErrorTolerances, "TrackingErrorTolerances");

    mInterface = AddInterfaceProvided("Controller");
    mInterface->AddMessageEvents();
    if (mInterface) {
        mInterface->AddCommandVoid(&mtsPID::ResetController, this, "ResetController");
        mInterface->AddCommandWrite(&mtsPID::enable, this, "Enable", m_enabled);
        mInterface->AddCommandWrite(&mtsPID::enable_joints, this, "EnableJoints", m_joints_enabled);
        mInterface->AddCommandWrite(&mtsPID::use_setpoint_v, this, "use_setpoint_v", m_use_setpoint_v);
        mInterface->AddCommandWrite(&mtsPID::EnableEffortMode, this, "EnableTorqueMode", m_effort_mode);
        mInterface->AddCommandReadState(StateTable, m_enabled, "Enabled");
        mInterface->AddCommandReadState(StateTable, m_joints_enabled, "JointsEnabled");
        mInterface->AddCommandReadState(StateTable, m_use_setpoint_v, "use_setpoint_v");

        // set goals
        mInterface->AddCommandWrite(&mtsPID::servo_jp, this, "servo_jp", prmPositionJointSet());
        mInterface->AddCommandWrite(&mtsPID::feed_forward_jf, this, "feed_forward_jf", prmForceTorqueJointSet());
        mInterface->AddCommandWrite(&mtsPID::servo_jf, this, "servo_jf", prmForceTorqueJointSet());

        // ROS compatible joint state
        mInterface->AddCommandReadState(StateTable, m_measured_js, "measured_js");
        mInterface->AddCommandReadState(StateTable, m_setpoint_js, "setpoint_js");

        // disturbance
        mInterface->AddCommandReadState(StateTable, m_error_state, "error_state/measured_js");

        // Get joint configuration
        mInterface->AddCommandReadState(mConfigurationStateTable, m_configuration, "configuration");
        mInterface->AddCommandReadState(mConfigurationStateTable, m_configuration_js, "configuration_js");

        // Set enforce position limits
        mInterface->AddCommandWrite(&mtsPID::enforce_position_limits, this, "enforce_position_limits", m_enforce_position_limits);
        mInterface->AddCommandReadState(StateTable, m_enforce_position_limits, "position_limits_enforced");

        // Error tracking
        mInterface->AddCommandWriteState(StateTable, mTrackingErrorEnabled, "EnableTrackingError");
        mInterface->AddCommandReadState(StateTable, mTrackingErrorEnabled, "TrackingErrorEnabled");
        mInterface->AddCommandWrite(&mtsPID::SetTrackingErrorTolerances, this, "SetTrackingErrorTolerances");

        // Set joint configuration
        mInterface->AddCommandWrite(&mtsPID::configure, this, "configure", m_configuration);

        // Events
        mInterface->AddEventWrite(Events.enabled, "Enabled", m_enabled);
        mInterface->AddEventWrite(Events.enabled_joints, "EnabledJoints", vctBoolVec());
        mInterface->AddEventWrite(Events.use_setpoint_v, "use_setpoint_v", m_use_setpoint_v);
        mInterface->AddEventWrite(Events.position_limit, "PositionLimit", vctBoolVec());
    }

    auto interfaceMonitoring = AddInterfaceProvided("Monitoring");
    if (interfaceMonitoring) {
        // ROS compatible joint state
        interfaceMonitoring->AddCommandReadState(StateTable, m_measured_js, "measured_js");
        interfaceMonitoring->AddCommandReadState(StateTable, m_setpoint_js, "setpoint_js");
        // disturbance
        interfaceMonitoring->AddCommandReadState(StateTable, m_error_state, "error_state/measured_js");
    }
}


void mtsPID::enforce_position_limits(const bool & enforce)
{
    // make sure we have proper joint limits
    if (enforce) {
        // if any value is different from 0, we assume we have proper limits
        if (m_configuration_js.PositionMin().Any() || m_configuration_js.PositionMax().Any()) {
            m_enforce_position_limits = enforce;
            return;
        } else {
            mInterface->SendWarning(this->GetName() + ": unable to enforce position limits since the limits are not set properly");
            CMN_LOG_CLASS_INIT_VERBOSE << this->GetName() << " enforce_position_limits: using configuration " << m_configuration_js << std::endl;
        }
    }
    m_enforce_position_limits = false;
}


void mtsPID::Configure(const std::string & filename)
{
    mConfigurationStateTable.Start();
    CMN_LOG_CLASS_INIT_VERBOSE << this->GetName() << " Configure: using " << filename << std::endl;

    try {
        std::ifstream jsonStream;
        Json::Value jsonConfig;
        Json::Reader jsonReader;

        jsonStream.open(filename.c_str());
        if (!jsonReader.parse(jsonStream, jsonConfig)) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                     << ": failed to parse configuration file \""
                                     << filename << "\"\n"
                                     << jsonReader.getFormattedErrorMessages();
            exit(EXIT_FAILURE);
        }

        CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << this->GetName()
                                   << " using file \"" << filename << "\"" << std::endl
                                   << "----> content of configuration file: " << std::endl
                                   << jsonConfig << std::endl
                                   << "<----" << std::endl;

        const auto jsonPID = jsonConfig["pid"];
        if (jsonPID.isNull()) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                     << ": failed to find \"pid\" in configuration file \""
                                     << filename << "\"\n" << std::endl;
            exit(EXIT_FAILURE);
        }
        cmnDataJSON<mtsPIDConfiguration>::DeSerializeText(m_configuration, jsonPID);

        // post parsing checks
        size_t index = 0;
        for (const auto & c : m_configuration) {
            if (c.index != index) {
                CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                         << ": index " << c.index << " doesn't match the position "
                                         << index << " in configuration file \""
                                         << filename << "\"\n" << std::endl;
                exit(EXIT_FAILURE);
            }
            ++index;
            if ((c.v_low_pass_cutoff <= 0.0) || (c.v_low_pass_cutoff > 1.0)) {
                CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                         << ": velocity low pass filter cut off value for " << c.index
                                         << " must be in ]0, 1] range, found "
                                         << c.v_low_pass_cutoff << " in configuration file \""
                                         << filename << "\"\n" << std::endl;
                exit(EXIT_FAILURE);
            }
        }

    } catch (std::exception & e) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName() << ": parsing file \""
                                 << filename << "\", got error: " << e.what() << std::endl;
        exit(EXIT_FAILURE);
    } catch (...) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName() << ": make sure the file \""
                                 << filename << "\" is in JSON format" << std::endl;
        exit(EXIT_FAILURE);
    }

    m_number_of_joints = m_configuration.size();
    if (m_number_of_joints < 1) {
        CMN_LOG_CLASS_INIT_ERROR << this->GetName()
                                 << " Configure: invalid number of joints: "
                                 << m_number_of_joints << std::endl;
        exit(EXIT_FAILURE);
    }

    // feedback
    m_pid_setpoint_jf.ForceTorque().SetSize(m_number_of_joints, 0.0);
    m_feed_forward_jf.ForceTorque().SetSize(m_number_of_joints, 0.0);
    mEffortUserCommand.ForceTorque().SetSize(m_number_of_joints, 0.0);

    // size all vectors
    m_configuration_js.Name().SetSize(m_number_of_joints);
    m_configuration_js.Type().SetSize(m_number_of_joints);
    m_configuration_js.PositionMin().SetSize(m_number_of_joints, 0.0);
    m_configuration_js.PositionMax().SetSize(m_number_of_joints, 0.0);
    m_configuration_js.EffortMin().SetSize(m_number_of_joints, 0.0);
    m_configuration_js.EffortMax().SetSize(m_number_of_joints, 0.0);

    m_measured_js.Name().SetSize(m_number_of_joints);
    m_measured_js.Position().SetSize(m_number_of_joints, 0.0);
    m_measured_js.Velocity().SetSize(m_number_of_joints, 0.0);
    m_measured_js.Effort().SetSize(m_number_of_joints, 0.0);

    m_setpoint_js.Name().SetSize(m_number_of_joints);
    m_setpoint_js.Position().SetSize(m_number_of_joints, 0.0);
    m_setpoint_js.Velocity().SetSize(m_number_of_joints, 0.0);
    m_setpoint_js.Effort().SetSize(m_number_of_joints, 0.0);

    mPositionLimitFlag.SetSize(m_number_of_joints);
    mPositionLimitFlag.SetAll(false);
    mPositionLimitFlagPrevious.ForceAssign(mPositionLimitFlag);
    m_joints_enabled.SetSize(m_number_of_joints);
    m_joints_enabled.SetAll(true);

    // errors
    m_error_state.Name().SetSize(m_number_of_joints);
    m_error_state.Position().SetSize(m_number_of_joints);
    m_error_state.Velocity().SetSize(m_number_of_joints);
    m_error_state.Effort().SetSize(m_number_of_joints); // disturbance
    m_i_error.SetSize(m_number_of_joints);
    // disturbance observer
    m_disturbance_state.SetSize(m_number_of_joints);
    m_disturbance_state.SetAll(0.0);
    ResetController();

    // default: 1 so there's no filtering
    m_setpoint_filtered_v.SetSize(m_number_of_joints, 0.0);
    m_setpoint_filtered_v_previous.SetSize(m_number_of_joints, 0.0);

    // effort mode
    m_effort_mode.SetSize(m_number_of_joints);
    m_effort_mode.SetAll(false);

    // tracking error
    mTrackingErrorEnabled = false;
    mTrackingErrorTolerances.SetSize(m_number_of_joints, 0.0);
    mTrackingErrorFlag.SetSize(m_number_of_joints, false);
    mPreviousTrackingErrorFlag.ForceAssign(mTrackingErrorFlag);

    // copy data from configuration, name and type
    size_t index = 0;
    for (const auto & c : m_configuration) {
        m_configuration_js.Name().at(index) = c.name;
        m_configuration_js.Type().at(index) = c.type;
        m_measured_js.Name().at(index) = c.name;
        m_setpoint_js.Name().at(index) = c.name;
        m_error_state.Name().at(index) = c.name;
        ++index;
    }
    mConfigurationStateTable.Advance();

    // now that we know the sizes of vectors, create interfaces
    this->SetupInterfaces();
}


void mtsPID::Startup(void)
{
    // get joint type from IO and check against values from PID config file
    if (!m_simulated) {
        mtsExecutionResult result;
        prmConfigurationJoint io_configuration_js;
        result = IO.configuration_js(io_configuration_js);
        if (!result) {
            CMN_LOG_CLASS_INIT_ERROR << this->GetName() << " Startup: Robot interface isn't connected properly, unable to get joint configuration.  Function call returned: "
                                     << result << std::endl;
        } else {
            // check the types
            for (size_t index = 0;
                 index < m_number_of_joints;
                 ++index) {
                if (io_configuration_js.Type().at(index) != m_configuration_js.Type().at(index)) {
                    std::string message = this->GetName() + " Startup: joint types from IO don't match types from configuration files for " + this->GetName();
                    CMN_LOG_CLASS_INIT_ERROR << message << std::endl
                                             << "From IO:     " << io_configuration_js.Type() << std::endl
                                             << "From config: " << m_configuration_js.Type() << std::endl;
                    cmnThrow("PID::" + message);
                }
            }

            // get the position min/max based on IO level.  Provided
            // from IO level based on max current and position to
            // current scale
            if ((io_configuration_js.PositionMin().size() != m_configuration_js.Name().size())
                || (io_configuration_js.PositionMax().size() != m_configuration_js.Name().size())) {
                std::string message = this->GetName() + " Startup: min and max position vectors from IO don't have the correct size for " + this->GetName();
                CMN_LOG_CLASS_INIT_ERROR << message << std::endl
                                         << "IO configuration position min: " << io_configuration_js.PositionMin() << std::endl
                                         << "IO configuration position max: " << io_configuration_js.PositionMax() << std::endl;
                cmnThrow("PID::" + message);
            } else {
                m_configuration_js.PositionMin().ForceAssign(io_configuration_js.PositionMin());
                m_configuration_js.PositionMax().ForceAssign(io_configuration_js.PositionMax());
                CheckLowerUpper(m_configuration_js.PositionMin(), m_configuration_js.PositionMax(), "Startup, position");
            }

            // get the effort min/max based on IO level.  Provided
            // from IO level based on max current and effort to
            // current scale
            if ((io_configuration_js.EffortMin().size() != m_configuration_js.Name().size())
                || (io_configuration_js.EffortMax().size() != m_configuration_js.Name().size())) {
                std::string message = this->GetName() + " Startup: min and max effort vectors from IO don't have the correct size for " + this->GetName();
                CMN_LOG_CLASS_INIT_ERROR << message << std::endl
                                         << "IO configuration effort min: " << io_configuration_js.EffortMin() << std::endl
                                         << "IO configuration effort max: " << io_configuration_js.EffortMax() << std::endl;
                cmnThrow("PID::" + message);
            } else {
                m_configuration_js.EffortMin().ForceAssign(io_configuration_js.EffortMin());
                m_configuration_js.EffortMax().ForceAssign(io_configuration_js.EffortMax());
                CheckLowerUpper(m_configuration_js.EffortMin(), m_configuration_js.EffortMax(), "Startup, effort");
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
            CMN_LOG_CLASS_INIT_VERBOSE << "Startup: configuration_js:" << std::endl
                                       << m_configuration_js << std::endl;
        }
    }
}


void mtsPID::Run(void)
{
    // first process events from IO (likely errors)
    ProcessQueuedEvents();

    // get data from IO if not in simulated mode
    get_IO_data();

    // now that we have recent data, deal with user commands
    ProcessQueuedCommands();

    // initialize variables
    bool anyTrackingError = false;
    bool newTrackingError = false;

    // loop on all joints
    vctBoolVec::const_iterator enabled = m_joints_enabled.begin();
    vctDoubleVec::const_iterator measured_p = m_measured_js.Position().begin();
    vctDoubleVec::const_iterator measured_v = m_measured_js.Velocity().begin();
    vctDoubleVec::iterator setpoint_p = m_setpoint_js.Position().begin();
    vctDoubleVec::iterator setpoint_f = m_setpoint_js.Effort().begin();
    vctBoolVec::const_iterator effortMode = m_effort_mode.begin();
    vctDoubleVec::const_iterator effortUserCommand = mEffortUserCommand.ForceTorque().begin();
    vctDoubleVec::iterator p_error = m_error_state.Position().begin();
    vctDoubleVec::iterator v_error = m_error_state.Velocity().begin();
    vctDoubleVec::const_iterator tolerance = mTrackingErrorTolerances.begin();
    vctBoolVec::iterator limitFlag = mPositionLimitFlag.begin();
    vctBoolVec::iterator trackingErrorFlag = mTrackingErrorFlag.begin();
    vctBoolVec::iterator previousTrackingErrorFlag = mPreviousTrackingErrorFlag.begin();
    vctDoubleVec::iterator i_error = m_i_error.begin();
    vctDoubleVec::iterator disturbance = m_error_state.Effort().begin();
    vctDoubleVec::iterator disturbance_state = m_disturbance_state.begin();
    auto c = m_configuration.cbegin();
    vctDoubleVec::const_iterator feed_forward = m_feed_forward_jf.ForceTorque().begin();
    vctDoubleVec::iterator filtered_setpoint_v = m_setpoint_filtered_v.begin();
    vctDoubleVec::iterator filtered_setpoint_v_previous = m_setpoint_filtered_v_previous.begin();
    vctDoubleVec::const_iterator effortLowerLimit = m_configuration_js.EffortMin().begin();
    vctDoubleVec::const_iterator effortUpperLimit = m_configuration_js.EffortMax().begin();

    // check if the setpoint has velocities
    vctDoubleVec::iterator setpoint_v;
    if (m_has_setpoint_v) {
        setpoint_v = m_setpoint_js.Velocity().begin();
    }

    CMN_ASSERT(m_joints_enabled.size() == m_number_of_joints);
    CMN_ASSERT(m_measured_js.Position().size() == m_number_of_joints);
    CMN_ASSERT(m_measured_js.Velocity().size() == m_number_of_joints);
    CMN_ASSERT(m_setpoint_js.Position().size() == m_number_of_joints);
    CMN_ASSERT(m_setpoint_js.Effort().size() == m_number_of_joints);
    CMN_ASSERT(m_setpoint_filtered_v.size() == m_number_of_joints);
    CMN_ASSERT(m_setpoint_filtered_v_previous.size() == m_number_of_joints);
    CMN_ASSERT(m_effort_mode.size() == m_number_of_joints);
    CMN_ASSERT(mEffortUserCommand.ForceTorque().size() == m_number_of_joints);
    CMN_ASSERT(m_configuration.size() == m_number_of_joints);
    CMN_ASSERT(mTrackingErrorTolerances.size() == m_number_of_joints);
    CMN_ASSERT(mPositionLimitFlag.size() == m_number_of_joints);
    CMN_ASSERT(mTrackingErrorFlag.size() == m_number_of_joints);
    CMN_ASSERT(mPreviousTrackingErrorFlag.size() == m_number_of_joints);
    CMN_ASSERT(m_i_error.size() == m_number_of_joints);
    CMN_ASSERT(m_error_state.Effort().size() == m_number_of_joints);
    CMN_ASSERT(m_disturbance_state.size() == m_number_of_joints);
    CMN_ASSERT(m_configuration.size() == m_number_of_joints);
    CMN_ASSERT(m_configuration_js.EffortMin().size() == m_number_of_joints);
    CMN_ASSERT(m_configuration_js.EffortMax().size() == m_number_of_joints);

    // loop on all joints using iterators
    for (size_t i = 0;
         i < m_number_of_joints;
         ++i,
             // make sure you increment all iterators declared above!
             ++enabled,
             ++measured_p,
             ++measured_v,
             ++setpoint_p,
             ++setpoint_f,
             ++effortMode,
             ++effortUserCommand,
             ++p_error,
             ++v_error,
             ++tolerance,
             ++limitFlag,
             ++trackingErrorFlag,
             ++previousTrackingErrorFlag,
             ++i_error,
             ++disturbance,
             ++disturbance_state,
             ++c,
             ++feed_forward,
             ++filtered_setpoint_v,
             ++filtered_setpoint_v_previous,
             ++effortLowerLimit,
             ++effortUpperLimit
         ) {

        // first check if the controller and this joint is enabled
        if (!m_enabled || !(*enabled)) {
            *setpoint_f = 0.0;
            *setpoint_p = *measured_p;
        } else {
            // the PID controller is enabled and this joint is actively controlled
            // check the mode, i.e. position or effort pass-through
            if (*effortMode) {
                *setpoint_f = *effortUserCommand;
                *setpoint_p = *measured_p;
            } else {
                // PID mode
                *p_error = *setpoint_p - *measured_p;
                // deadband
                bool in_deadband = false;
                if (*p_error > 0.0) {
                    if (*p_error < c->p_deadband) {
                        *p_error = 0.0;
                        in_deadband = true;
                    } else {
                        *p_error -= c->p_deadband;
                    }
                } else if (*p_error < 0.0) {
                    if (*p_error > -c->p_deadband) {
                        *p_error = 0.0;
                        in_deadband = true;
                    } else {
                        *p_error += c->p_deadband;
                    }
                }
                // check for tracking errors
                if (mTrackingErrorEnabled) {
                    double errorAbsolute = fabs(*p_error);
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
                *v_error = 0.0;
                if (!in_deadband) {
                    double _setpoint_v = 0.0;
                    if (m_has_setpoint_v && m_use_setpoint_v) {
                        // apply filter on setpoint_v only
                        *filtered_setpoint_v = (1.0 - c->v_low_pass_cutoff) * *filtered_setpoint_v_previous + c->v_low_pass_cutoff * *setpoint_v;
                        _setpoint_v = *filtered_setpoint_v;
                        *filtered_setpoint_v_previous = *filtered_setpoint_v;
                    }
                    *v_error = _setpoint_v - (*measured_v);
                }

                // compute error integral
                *i_error *= c->i_forget_rate;
                *i_error += *p_error;
                if (*i_error > c->i_limit) {
                    *i_error = c->i_limit;
                }
                else if (*i_error < -c->i_limit) {
                    *i_error = -c->i_limit;
                }


                // compute disturbance observer before setpoint_f is changed
                if (c->use_disturbance_observer) {
                    const double d_t = this->GetPeriodicity();
                    const double dis_tmp = *setpoint_f
                        + c->nominal_mass * c->disturbance_cutoff * *measured_v;
                    *disturbance_state += (dis_tmp - *disturbance_state)
                        * c->disturbance_cutoff * d_t;
                    *disturbance = *disturbance_state
                        - c->nominal_mass * c->disturbance_cutoff * *measured_v;
                    *setpoint_f = *disturbance;
                } else {
                    *disturbance = 0.0;
                    *setpoint_f = 0.0;
                }

                // compute PID effort
                *setpoint_f +=
                    c->p_gain * (*p_error) + c->d_gain * (*v_error) + c->i_gain * (*i_error);

                // add constant offsets in PID mode only and after non-linear scaling
                *setpoint_f += c->offset;

                // add feedForward
                *setpoint_f += *feed_forward;

            } // end of PID mode

            // apply effort limits if needed
            if (*effortLowerLimit != *effortUpperLimit) {
                if (*setpoint_f > *effortUpperLimit) {
                    *setpoint_f = *effortUpperLimit;
                } else if (*setpoint_f < *effortLowerLimit) {
                    *setpoint_f = *effortLowerLimit;
                }
            }
        } // end of enabled

        // increment setpoint_v if it's used
        if (m_has_setpoint_v) {
            ++setpoint_v;
        }
    }

    // report errors (tracking)
    if (mTrackingErrorEnabled && anyTrackingError) {
        this->enable(false);
        if (newTrackingError) {
            std::string message = this->GetName() + ": tracking error, mask (1 for error): ";
            message.append(mTrackingErrorFlag.ToString());
            mInterface->SendError(message);
            CMN_LOG_CLASS_RUN_ERROR << message << std::endl
                                    << "errors:     " << m_error_state.Position() << std::endl
                                    << "tolerances: " << mTrackingErrorTolerances << std::endl
                                    << "measured:   " << m_measured_js.Position() << std::endl
                                    << "setpoint:   " << m_setpoint_js.Position() << std::endl;
        }
    }

    // make sure we apply efforts only if enabled
    if (m_enabled) {
        SetEffortLocal(m_setpoint_js.Effort());
    }

    // for simulated mode
    if (m_simulated) {
        m_measured_js.SetValid(true);
        m_measured_js.Position().Assign(m_setpoint_js.Position());
        m_measured_js.SetTimestamp(StateTable.GetTic());
    }
}


void mtsPID::Cleanup(void)
{
    // cleanup
    m_setpoint_js.Effort().Zeros();
    SetEffortLocal(m_setpoint_js.Effort());
}


void mtsPID::SetSimulated(void)
{
    m_simulated = true;
    // in simulation mode, we don't need IO
    RemoveInterfaceRequired("RobotJointTorqueInterface");
}


void mtsPID::configure(const mtsPIDConfiguration & configuration)
{
    if (SizeMismatch(configuration.size(), "configure")) {
        return;
    }
    mConfigurationStateTable.Start();
    m_configuration = configuration;
    mConfigurationStateTable.Advance();
}


void mtsPID::ResetController(void)
{
    CMN_LOG_CLASS_RUN_VERBOSE << this->GetName() << " Reset Controller" << std::endl;
    m_error_state.Position().SetAll(0.0);
    m_error_state.Velocity().SetAll(0.0);
    m_error_state.Effort().SetAll(0.0); // disturbance
    m_i_error.SetAll(0.0);
    this->enable(false);
}


void mtsPID::servo_jp(const prmPositionJointSet & command)
{
    // set position within limits
    if (SizeMismatch(command.Goal().size(), "servo_jp")) {
        return;
    }

    m_setpoint_js.Position().Assign(command.Goal());
    mCommandTime = command.Timestamp(); // m_setpoint_js timestamp is set by this class so can't use it later

    if (m_enforce_position_limits) {
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
                Events.position_limit(mPositionLimitFlag);
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

    // set and resize setpoint_v as needed
    m_has_setpoint_v = false;
    if (command.Velocity().size() > 0) {
        if (SizeMismatch(command.Velocity().size(), "servo_jp")) {
            m_setpoint_js.Velocity().Zeros();
            return;
        }
        m_has_setpoint_v = true;
        m_setpoint_js.Velocity().Assign(command.Velocity());
    } else {
        m_setpoint_js.Velocity().Zeros();
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


void mtsPID::enable(const bool & enable)
{
    if (enable == m_enabled) {
        // trigger enabled
        Events.enabled(m_enabled);
        return;
    }

    m_enabled = enable;

    // set effort to 0
    m_setpoint_js.Effort().Zeros();
    SetEffortLocal(m_setpoint_js.Effort());

    // reset error flags
    if (enable) {
        mPreviousTrackingErrorFlag.SetAll(false);
        mTrackingErrorFlag.SetAll(false);
        mPositionLimitFlagPrevious.SetAll(false);
        mPositionLimitFlag.SetAll(false);
        mPreviousCommandTime = 0.0;
        m_feed_forward_jf.ForceTorque().Zeros();
        m_has_setpoint_v = false;
        m_setpoint_filtered_v_previous.Zeros();
        // set valid flags
        m_error_state.SetValid(true);
    } else {
        m_error_state.SetValid(false);
    }

    // trigger Enabled
    Events.enabled(m_enabled);
}


void mtsPID::enable_joints(const vctBoolVec & enable)
{
    if (SizeMismatch(enable.size(), "EnableJoints")) {
        return;
    }
    m_joints_enabled.Assign(enable);
    Events.enabled_joints(enable);
}


void mtsPID::use_setpoint_v(const bool & use)
{
    m_use_setpoint_v = use;
    Events.use_setpoint_v(m_use_setpoint_v);
    if (m_use_setpoint_v) {
        m_setpoint_filtered_v_previous.Zeros();
    }
}


void mtsPID::EnableEffortMode(const vctBoolVec & enable)
{
    if (SizeMismatch(enable.size(), "EnableEffortMode")) {
        return;
    }
    // save preference
    m_effort_mode.Assign(enable);
    // reset effort to 0
    m_setpoint_js.Effort().Zeros();
    m_feed_forward_jf.ForceTorque().Zeros();
}


void mtsPID::get_IO_data(void)
{
    // get data from IO if not in simulated mode.  When is simulation
    // mode, position come from the user/client and is found in
    // m_setpoint_js
    if (m_simulated) {
        // check that position is not too old
        if ((StateTable.GetTic() - mCommandTime) > 20.0 * cmn_ms) {
            m_measured_js.Velocity().Zeros();
        } else {
            // evaluate velocity based on positions sent by arm/client
            const double dt = mCommandTime - mPreviousCommandTime;
            if (dt > 0) {
                vctDoubleVec::const_iterator currentPosition = m_setpoint_js.Position().begin();
                vctDoubleVec::const_iterator previousPosition = m_measured_js_previous.Position().begin();
                vctBoolVec::const_iterator effortMode = m_effort_mode.begin();
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

        // save previous position with timestamp
        m_measured_js_previous = m_measured_js;
    }
}

void mtsPID::SetEffortLocal(const vctDoubleVec & effort)
{
    if (!m_simulated) {
        m_pid_setpoint_jf.ForceTorque().Assign(effort);
        IO.servo_jf(m_pid_setpoint_jf);
    }
}

void mtsPID::SetTrackingErrorTolerances(const vctDoubleVec & tolerances)
{
    if (tolerances.size() == m_number_of_joints) {
        mTrackingErrorTolerances.Assign(tolerances);
    } else {
        std::string message = this->GetName() + ": incorrect vector size for SetTrackingErrorTolerances";
        cmnThrow(message);
    }
}

void mtsPID::ErrorEventHandler(const mtsMessage & message)
{
    if (this->m_enabled) {
        this->enable(false);
        mInterface->SendError(this->GetName() + ": received [" + message.Message + "]");
    } else {
        mInterface->SendStatus(this->GetName() + ": received [" + message.Message + "]");
    }
}

bool mtsPID::SizeMismatch(const size_t size, const std::string & methodName)
{
    if (size != m_number_of_joints) {
        CMN_LOG_CLASS_INIT_ERROR << this->GetName() << " " << methodName << ": size mismatch, expected "
                                 << m_number_of_joints << ", received "
                                 << size << std::endl;
        this->enable(false);
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
