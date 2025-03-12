/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-22

  (C) Copyright 2013-2025 Johns Hopkins University (JHU), All Rights Reserved.

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

#include <cmath>

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

    // this should go in a "read" state table
    StateTable.AddData(m_enabled, "enabled");
    StateTable.AddData(m_joints_enabled, "joints_enabled");
    StateTable.AddData(m_setpoint_v_used, "setpoint_v_used");

    // measures are timestamped by the IO level
    StateTable.AddData(m_enforce_position_limits, "enforce_position_limits");

    m_measured_js.SetAutomaticTimestamp(false);
    StateTable.AddData(m_measured_js, "measured_js");
    StateTable.AddData(m_setpoint_js, "setpoint_js");
    StateTable.AddData(m_error_state, "error_state");

    // configuration state table with occasional start/advance
    mConfigurationStateTable.AddData(m_configuration, "configuration");
    mConfigurationStateTable.AddData(m_configuration_js, "configuration_js");
    StateTable.AddData(m_measured_setpoint_check, "enable_measured_setpoint_check"); // that table advances automatically
    mConfigurationStateTable.AddData(m_measured_setpoint_tolerance, "measured to setpoint tolerance");

    mInterface = AddInterfaceProvided("Controller");
    mInterface->AddMessageEvents();
    if (mInterface) {
        mInterface->AddCommandVoid(&mtsPID::reset_controller, this, "reset_controller");
        mInterface->AddCommandWrite(&mtsPID::enable, this, "enable", m_enabled);
        mInterface->AddCommandWrite(&mtsPID::enable_joints, this, "enable_joints", m_joints_enabled);
        mInterface->AddCommandWrite(&mtsPID::use_setpoint_v, this, "use_setpoint_v", m_setpoint_v_used);
        mInterface->AddCommandReadState(StateTable, m_enabled, "enabled");
        mInterface->AddCommandReadState(StateTable, m_joints_enabled, "joints_enabled");
        mInterface->AddCommandReadState(StateTable, m_setpoint_v_used, "setpoint_v_used");

        // set goals
        mInterface->AddCommandWrite(&mtsPID::servo_js, this, "servo_js", prmServoJoint());
        mInterface->AddCommandWrite(&mtsPID::servo_jp, this, "servo_jp", prmPositionJointSet());
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

        // measured vs setpoint check
        mInterface->AddCommandWriteState(StateTable, m_measured_setpoint_check, "enable_measured_setpoint_check");
        mInterface->AddCommandReadState(StateTable, m_measured_setpoint_check, "measured_setpoint_check_enabled");
        mInterface->AddCommandWrite(&mtsPID::set_measured_setpoint_tolerance, this, "set_measured_setpoint_tolerance");

        // Set joint configuration
        mInterface->AddCommandWrite(&mtsPID::configure, this, "configure", m_configuration);

        // Events
        mInterface->AddEventWrite(Events.enabled, "enabled", m_enabled);
        mInterface->AddEventWrite(Events.enabled_joints, "enabled_joints", vctBoolVec());
        mInterface->AddEventWrite(Events.setpoint_v_used, "setpoint_v_used", m_setpoint_v_used);
        mInterface->AddEventWrite(Events.position_limit, "position_limit", vctBoolVec());
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

    // size all vectors
    m_pid_setpoint_jf.ForceTorque().SetSize(m_number_of_joints, 0.0);

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

    m_servo_js.Name().SetSize(m_number_of_joints);
    m_servo_js.Position().SetSize(m_number_of_joints, 0.0);
    m_servo_js.Velocity().SetSize(m_number_of_joints, 0.0);
    m_servo_js.Effort().SetSize(m_number_of_joints, 0.0);
    m_servo_js.Mode().SetSize(m_number_of_joints, prmSetpointMode::NONE);
    m_servo_js.PositionProjection().SetSize(m_number_of_joints, m_number_of_joints, 0.0);

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
    reset_controller();

    m_setpoint_filtered_v.SetSize(m_number_of_joints, 0.0);
    m_setpoint_filtered_v_previous.SetSize(m_number_of_joints, 0.0);

    // tracking error
    m_measured_setpoint_check = false;
    m_measured_setpoint_tolerance.SetSize(m_number_of_joints, 0.0);
    m_measured_setpoint_error.SetSize(m_number_of_joints, false);
    m_previous_measured_setpoint_error.ForceAssign(m_measured_setpoint_error);

    // copy data from configuration, name and type
    size_t index = 0;
    for (const auto & c : m_configuration) {
        m_configuration_js.Name().at(index) = c.name;
        m_configuration_js.Type().at(index) = c.type;
        m_measured_js.Name().at(index) = c.name;
        m_setpoint_js.Name().at(index) = c.name;
        m_servo_js.Name().at(index) = c.name;
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

    // loop on all joints
    vctBoolVec::const_iterator enabled = m_joints_enabled.begin();
    vctDoubleVec::const_iterator measured_p = m_measured_js.Position().begin();
    vctDoubleVec::const_iterator measured_v = m_measured_js.Velocity().begin();

    vctDoubleVec::iterator setpoint_p = m_setpoint_js.Position().begin();
    vctDoubleVec::iterator setpoint_v = m_setpoint_js.Velocity().begin();
    vctDoubleVec::iterator setpoint_f = m_setpoint_js.Effort().begin();
    vctDynamicVector<prmSetpointMode>::const_iterator mode = m_servo_js.Mode().begin();

    vctDoubleVec::iterator p_error = m_error_state.Position().begin();
    vctDoubleVec::iterator v_error = m_error_state.Velocity().begin();

    vctDoubleVec::iterator disturbance = m_error_state.Effort().begin();
    vctDoubleVec::iterator disturbance_state = m_disturbance_state.begin();
    vctDoubleVec::iterator i_error = m_i_error.begin();

    vctDoubleVec::iterator filtered_setpoint_v = m_setpoint_filtered_v.begin();
    vctDoubleVec::iterator filtered_setpoint_v_previous = m_setpoint_filtered_v_previous.begin();

    vctBoolVec::iterator limitFlag = mPositionLimitFlag.begin();
    vctDoubleVec::const_iterator tolerance = m_measured_setpoint_tolerance.begin();
    vctBoolVec::iterator trackingErrorFlag = m_measured_setpoint_error.begin();
    vctBoolVec::iterator previousTrackingErrorFlag = m_previous_measured_setpoint_error.begin();

    auto c = m_configuration.cbegin();

    CMN_ASSERT(m_joints_enabled.size() == m_number_of_joints);
    CMN_ASSERT(m_measured_js.Position().size() == m_number_of_joints);
    CMN_ASSERT(m_measured_js.Velocity().size() == m_number_of_joints);

    CMN_ASSERT(m_setpoint_js.Position().size() == m_number_of_joints);
    CMN_ASSERT(m_setpoint_js.Effort().size() == m_number_of_joints);
    CMN_ASSERT(m_servo_js.Effort().size() == m_number_of_joints);
    CMN_ASSERT(m_servo_js.Mode().size() == m_number_of_joints);

    CMN_ASSERT(m_setpoint_js.Position().size() == m_number_of_joints);
    CMN_ASSERT(m_setpoint_js.Velocity().size() == m_number_of_joints);
    CMN_ASSERT(m_setpoint_js.Effort().size() == m_number_of_joints);
    CMN_ASSERT(m_setpoint_filtered_v.size() == m_number_of_joints);
    CMN_ASSERT(m_setpoint_filtered_v_previous.size() == m_number_of_joints);

    CMN_ASSERT(m_error_state.Position().size() == m_number_of_joints);
    CMN_ASSERT(m_error_state.Velocity().size() == m_number_of_joints);
    CMN_ASSERT(m_error_state.Effort().size() == m_number_of_joints);

    CMN_ASSERT(m_disturbance_state.size() == m_number_of_joints);
    CMN_ASSERT(m_i_error.size() == m_number_of_joints);

    CMN_ASSERT(m_setpoint_filtered_v.size() == m_number_of_joints);
    CMN_ASSERT(m_setpoint_filtered_v_previous.size() == m_number_of_joints);

    CMN_ASSERT(mPositionLimitFlag.size() == m_number_of_joints);
    CMN_ASSERT(m_measured_setpoint_tolerance.size() == m_number_of_joints);
    CMN_ASSERT(m_measured_setpoint_error.size() == m_number_of_joints);
    CMN_ASSERT(m_previous_measured_setpoint_error.size() == m_number_of_joints);

    CMN_ASSERT(m_configuration.size() == m_number_of_joints);
    CMN_ASSERT(m_configuration_js.EffortMin().size() == m_number_of_joints);
    CMN_ASSERT(m_configuration_js.EffortMax().size() == m_number_of_joints);

    const double dt = this->GetPeriodicity();

    // loop on all joints using iterators
    for (size_t i = 0;
         i < m_number_of_joints;
         ++i,
             // make sure you increment all iterators declared above!
             ++enabled,
             ++measured_p,
             ++measured_v,
             ++setpoint_p,
             ++setpoint_v,
             ++setpoint_f,
             ++mode,
             ++p_error,
             ++v_error,
             ++i_error,
             ++disturbance,
             ++disturbance_state,
             ++filtered_setpoint_v,
             ++filtered_setpoint_v_previous,
             ++limitFlag,
             ++tolerance,
             ++trackingErrorFlag,
             ++previousTrackingErrorFlag,
             ++c
         ) {

        // first check if the controller and this joint is enabled
        if (!m_enabled || !(*enabled)) {
            *setpoint_f = 0.0;
            *setpoint_v = 0.0;
            *setpoint_p = *measured_p;

            *p_error = 0.0;
            *v_error = 0.0;
            *i_error = 0.0;
            *disturbance_state = 0.0;
            *disturbance = 0.0;
        } else {
            // for disturbance observer, should really be pulled from m_pid_setpoint_jf
            double current_effort = *setpoint_f;

            const bool velocity_mode = (*mode) & prmSetpointMode::VELOCITY;
            // currently velocity mode implementation requires position mode
            const bool position_mode = velocity_mode || ((*mode) & prmSetpointMode::POSITION);
            const bool psuedo_position_mode = velocity_mode && !((*mode) & prmSetpointMode::POSITION);

            if (psuedo_position_mode) {
                // note: not measured_p + dt * setpoint_v since that "drifts" too much
                *setpoint_p += *setpoint_v * dt;
                // TODO: check position limits!!!!!
            }

            bool in_deadband = false; // needs to be acessible for velocity mode
            *p_error = 0.0;
            if (position_mode) {
                *p_error = *setpoint_p - *measured_p;
                // deadband
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

                // compute error integral
                *i_error *= c->i_forget_rate;
                *i_error += *p_error * dt;
                *i_error = clamp(*i_error, -c->i_limit, c->i_limit);

                *disturbance = 0.0;
                // TODO: need to reset when joint enters/leaves position mode
                if (c->use_disturbance_observer) {
                    const double dis_tmp = current_effort + c->nominal_mass * c->disturbance_cutoff * *measured_v;
                    *disturbance_state += (dis_tmp - *disturbance_state) * c->disturbance_cutoff * dt;
                    *disturbance = *disturbance_state - c->nominal_mass * c->disturbance_cutoff * *measured_v;
                } else {
                    *disturbance_state = 0.0;
                    *disturbance = 0.0;
                }
            } else {
                *setpoint_p = *measured_p;
                *disturbance_state = 0.0;
                *disturbance = 0.0;
            }

            *v_error = 0.0;
            if (velocity_mode) {
                // compute error derivative
                // TODO: only zero-out velocity if in deadband AND v-error is very small (e.g. dt * v-error < deadband size)?
                // maybe we always zero velocity if dt*v-error < deadband size?
                if (!in_deadband) {
                    double _setpoint_v = 0.0;
                    if (m_setpoint_v_used) {
                        // apply filter on setpoint_v only
                        *filtered_setpoint_v = (1.0 - c->v_low_pass_cutoff) * *filtered_setpoint_v_previous + c->v_low_pass_cutoff * *setpoint_v;
                        _setpoint_v = *filtered_setpoint_v;
                        *filtered_setpoint_v_previous = *filtered_setpoint_v;
                    }
                    *v_error = _setpoint_v - (*measured_v);
                }
            } else if (position_mode) {
                // if we don't have a velocity setpoint, assume zero
                *v_error = 0.0 - (*measured_v);
            }
        } // end of enabled
    }

    m_error_state.Position() = m_servo_js.PositionProjection() * m_error_state.Position();
    m_error_state.Velocity() = m_servo_js.PositionProjection() * m_error_state.Velocity();
    m_setpoint_js.Position() = m_measured_js.Position() + m_error_state.Position();

    for (size_t i = 0; i < m_number_of_joints; i++) {
        if (m_enabled && m_joints_enabled[i]) {
            const auto& cfg = m_configuration[i];
            double pid = cfg.p_gain * m_error_state.Position()[i] +
                         cfg.i_gain * m_i_error[i] +
                         cfg.d_gain * m_error_state.Velocity()[i];
            // total effort = offset + pid effort + disturance estimate + effort command
            m_setpoint_js.Effort()[i] = cfg.offset + pid + m_error_state.Effort()[i] + m_servo_js.Effort()[i];
        }
    }

    measured_setpoint_check();

    // apply effort limits if needed
    // TODO: why inequality test?
    const auto& lower_limits = m_configuration_js.EffortMin();
    const auto& upper_limits = m_configuration_js.EffortMax();
    for (size_t i = 0; i < m_number_of_joints; i++) {
        if (lower_limits[i] != upper_limits[i]) {
            m_setpoint_js.Effort()[i] = clamp(m_setpoint_js.Effort()[i], lower_limits[i], upper_limits[i]);
        }
    }

    // make sure we apply efforts only if enabled
    if (m_enabled) {
        servo_jf_local(m_setpoint_js.Effort());
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
    servo_jf_local(m_setpoint_js.Effort());
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


void mtsPID::reset_controller(void)
{
    CMN_LOG_CLASS_RUN_VERBOSE << this->GetName() << " reset_controller" << std::endl;
    m_error_state.Position().SetAll(0.0);
    m_error_state.Velocity().SetAll(0.0);
    m_error_state.Effort().SetAll(0.0); // disturbance
    m_i_error.SetAll(0.0);
    this->enable(false);
}


void mtsPID::servo_js(const prmServoJoint & command)
{
    if (command.Position().size() > 0 && SizeMismatch(command.Position().size(), "servo_js:position")) { return; }
    if (command.Velocity().size() > 0 && SizeMismatch(command.Velocity().size(), "servo_js:velocity")) { return; }
    if (command.Effort().size() > 0 && SizeMismatch(command.Effort().size(), "servo_js:effort")) { return; }
    if (command.Mode().size() > 0 && SizeMismatch(command.Mode().size(), "servo_js:mode")) { return; }

    if (command.Position().size() > 0) {
        m_setpoint_js.Position().Assign(command.Position());
    } else {
        m_setpoint_js.Position().Assign(m_measured_js.Position());
    }

    if (command.Velocity().size() > 0) {
        m_setpoint_js.Velocity().Assign(command.Velocity());
    } else {
        m_setpoint_js.Velocity().Zeros();
    }

    // m_setpoint_js.Effort() already used to hold controller output
    if (command.Effort().size() > 0) {
        m_servo_js.Effort().Assign(command.Effort());
    } else {
        m_servo_js.Effort().Zeros();
    }

    m_servo_js.Mode().Assign(command.Mode());
    m_command_time = command.Timestamp(); // m_setpoint_js timestamp is set by this class so can't use it later

    if (m_servo_js.Mode().size() == 0) {
        m_servo_js.Mode().SetAll(prmSetpointMode::NONE);
    }

    vctDoubleMat::size_type rows = command.PositionProjection().rows();
    vctDoubleMat::size_type cols = command.PositionProjection().cols();
    if (rows != 0 || cols != 0) {
        if (SizeMismatch(rows, "servo_js:projection:rows") || SizeMismatch(cols, "servo_js:projection:cols")) {
            return;
        }

        m_servo_js.PositionProjection().Assign(command.PositionProjection());
    } else {
        m_servo_js.PositionProjection().Zeros();
        for (vctDoubleMat::size_type i = 0; i < m_number_of_joints; i++) {
            bool pid_control = (m_servo_js.Mode()[i] & prmSetpointMode::POSITION) || (m_servo_js.Mode()[i] & prmSetpointMode::VELOCITY);
            m_servo_js.PositionProjection()(i, i) = pid_control ? 1 : 0;
        }
    }

    if (m_enforce_position_limits && command.Position().size() > 0) {
        bool limitReached = false;
        vctDynamicVector<prmSetpointMode>::const_iterator mode = m_servo_js.Mode().begin();
        vctDoubleVec::const_iterator upper = m_configuration_js.PositionMax().begin();
        vctDoubleVec::const_iterator lower = m_configuration_js.PositionMin().begin();
        vctBoolVec::iterator flags = mPositionLimitFlag.begin();
        vctDoubleVec::iterator desired = m_setpoint_js.Position().begin();
        const vctDoubleVec::iterator end = m_setpoint_js.Position().end();
        for (; desired != end; ++desired, ++upper, ++lower, ++flags, ++mode) {
            if (!(*mode & prmSetpointMode::POSITION)) {
                *flags = false;
                continue; // only enforce for joints under position control
            }

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
}

void mtsPID::servo_jp(const prmPositionJointSet & command) {
    prmServoJoint js;
    js.Position().Assign(command.Goal());
    js.Velocity().Assign(command.Velocity());
    js.Effort().Zeros();

    js.Mode().SetSize(command.Goal().size());
    js.Mode().SetAll(prmSetpointMode::POSITION);

    js.PositionProjection().Assign(vctDoubleMat::Eye(m_number_of_joints));
}

void mtsPID::servo_jf(const prmForceTorqueJointSet & command) {
    prmServoJoint js;
    if (SizeMismatch(command.ForceTorque().size(), "servo_jf")) {
        return;
    }

    js.Position().Assign(m_measured_js.Position());
    js.Velocity().Zeros();
    js.Effort().Assign(command.ForceTorque());

    js.Mode().SetSize(command.ForceTorque().size());
    js.Mode().SetAll(prmSetpointMode::EFFORT);

    js.PositionProjection().Zeros();
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
    servo_jf_local(m_setpoint_js.Effort());

    // reset error flags
    if (enable) {
        m_previous_measured_setpoint_error.SetAll(false);
        m_measured_setpoint_error.SetAll(false);
        mPositionLimitFlagPrevious.SetAll(false);
        mPositionLimitFlag.SetAll(false);
        m_previous_command_time = 0.0;
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
    if (SizeMismatch(enable.size(), "enable_joints")) {
        return;
    }
    m_joints_enabled.Assign(enable);
    Events.enabled_joints(enable);
}


void mtsPID::use_setpoint_v(const bool & use)
{
    m_setpoint_v_used = use;
    std::cerr << "used: " << use << std::endl;
    Events.setpoint_v_used(m_setpoint_v_used);
    if (m_setpoint_v_used) {
        m_setpoint_filtered_v_previous.Zeros();
    }
}


void mtsPID::get_IO_data(void)
{
    // get data from IO if not in simulated mode.  When is simulation
    // mode, position come from the user/client and is found in
    // m_setpoint_js
    if (m_simulated) {
        // check that position is not too old
        if ((StateTable.GetTic() - m_command_time) > 20.0 * cmn_ms) {
            m_measured_js.Velocity().Zeros();
        } else {
            // evaluate velocity based on positions sent by arm/client
            const double dt = m_command_time - m_previous_command_time;
            if (dt > 0) {
                vctDoubleVec::const_iterator currentPosition = m_setpoint_js.Position().begin();
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
        }
        // timestamp using last know position
        m_measured_js_previous = m_setpoint_js;
        m_previous_command_time = m_command_time;
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
                vctDoubleVec::iterator velocity = m_measured_js.Velocity().begin();
                const vctDoubleVec::iterator end = m_measured_js.Velocity().end();
                for (; velocity != end;
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

void mtsPID::servo_jf_local(const vctDoubleVec & effort)
{
    m_pid_setpoint_jf.ForceTorque().Assign(effort);

    if (!m_simulated) {
        IO.servo_jf(m_pid_setpoint_jf);
    }
}

void mtsPID::set_measured_setpoint_tolerance(const vctDoubleVec & tolerances)
{
    if (tolerances.size() == m_number_of_joints) {
        m_measured_setpoint_tolerance.Assign(tolerances);
    } else {
        std::string message = this->GetName() + ": incorrect vector size for set_measured_setpoint_tolerance";
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

bool mtsPID::measured_setpoint_check()
{
    if (!m_measured_setpoint_check) {
        return true;
    }

    bool any_tracking_error = false;
    bool new_tracking_error = false;
    const auto& position_error = m_error_state.Position();
    for (size_t i = 0; i < m_number_of_joints; i++) {
        if (std::abs(position_error[i]) > m_measured_setpoint_tolerance[i]) {
            any_tracking_error = true;
            m_measured_setpoint_error[i] = true;
            if (!m_previous_measured_setpoint_error[i]) {
                new_tracking_error = true;
            }
        } else {
            m_measured_setpoint_error[i] = false;
        }
    }

    m_previous_measured_setpoint_error = m_measured_setpoint_error;

    // report errors (tracking)
    if (any_tracking_error) {
        this->enable(false);
        if (new_tracking_error) {
            std::string message = this->GetName() + ": tracking error, mask (1 for error): ";
            message.append(m_measured_setpoint_error.ToString());
            mInterface->SendError(message);

            std::stringstream full_message;
            full_message << message << "\n"
                         << std::endl
                         << "errors:     " << m_error_state.Position() << std::endl
                         << "tolerances: " << m_measured_setpoint_tolerance << std::endl
                         << "measured:   " << m_measured_js.Position() << std::endl
                         << "setpoint:   " << m_setpoint_js.Position() << std::endl;
            CMN_LOG_CLASS_RUN_ERROR << full_message.str();
        }
    }

    return any_tracking_error;
}
