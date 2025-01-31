/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet, Ugur Tumerdem
  Created on: 2013-02-20

  (C) Copyright 2013-2023 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsPIDQtWidget_h
#define _mtsPIDQtWidget_h

#include <cisstCommon/cmnXMLPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstVector/vctPlot2DOpenGLQtWidget.h>
#include <cisstMultiTask/mtsComponent.h>
#include <cisstVector/vctQtWidgetDynamicVector.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmConfigurationJoint.h>
#include <cisstParameterTypes/prmPositionJointSet.h>

#include <QCheckBox>
#include <QSpinBox>
#include <QPushButton>

#include <sawControllers/mtsPIDConfiguration.h>
#include <sawControllers/sawControllersQtExport.h>

class CISST_EXPORT mtsPIDQtWidget: public QWidget, public mtsComponent
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsPIDQtWidget(const std::string & componentName, unsigned int numberOfAxis,
                   double periodInSeconds = 50.0 * cmn_ms);
    mtsPIDQtWidget(const mtsComponentConstructorNameAndUInt &arg);
    ~mtsPIDQtWidget(){}

    void Configure(const std::string & filename = "");
    void Startup(void);
    void Cleanup(void);

protected:
    void Init(void);
    void GetConfiguration(void);
    virtual void closeEvent(QCloseEvent * event);

signals:
    void SignalEnable(bool enable);
    void SignalUseSetpointV(bool use);

private slots:
    //! slot enable/disable mtsPID controller
    void SlotEnable(bool toggle);
    void SlotEnabledJointsChanged(void);
    void SlotEnableTrackingError(bool toggle);
    void SlotEnforcePositionLimits(bool toggle);
    void SlotConfigurationChanged(void);
    //! slot send setpoint pos when input changed
    void SlotPositionChanged(void);
    //! slot reset setpoint pos to measured pos
    void SlotMaintainPosition(void);
    //! slot to save
    void SlotSave(void);
    //! slot to select which axis to plot
    void SlotPlotIndex(int newAxis);
    //! slot to change Enable Checkbox
    void SlotEnableEventHandler(bool enable);
    void SlotEnableDirectControl(bool toggle);

    void SlotUseSetpointV(bool use);
    void SlotUseSetpointVEventHandler(bool use);

    //! timer event to update GUI
    void timerEvent(QTimerEvent * event);

private:
    //! setup PID controller GUI
    void setupUi(void);
    int TimerPeriodInMilliseconds;

    void ErrorEventHandler(const mtsMessage & message);
    void EnableEventHandler(const bool & enable);
    void UseSetpointVEventHandler(const bool & use);

protected:

    struct ControllerPIDStruct {
        mtsFunctionVoid  ResetController;
        mtsFunctionWrite Enable;
        mtsFunctionWrite EnableJoints;
        mtsFunctionRead  JointsEnabled;
        mtsFunctionWrite UseSetpointV;
        mtsFunctionRead  UsingSetpointV;
        mtsFunctionWrite EnableTrackingError;
        mtsFunctionRead  TrackingErrorEnabled;
        mtsFunctionWrite enforce_position_limits;
        mtsFunctionRead  position_limits_enforced;
        mtsFunctionRead  configuration;
        mtsFunctionRead  configuration_js;
        mtsFunctionWrite configure;
        mtsFunctionWrite servo_jp;
        mtsFunctionRead  measured_js;
        mtsFunctionRead  setpoint_js;
        mtsFunctionRead error_state_measured_js;

        mtsPIDConfiguration m_configuration;
        prmConfigurationJoint m_configuration_js;
        prmStateJoint m_measured_js;
        prmStateJoint m_setpoint_js;
        prmStateJoint m_error_state;
    } PID;

private:
    bool DirectControl;

    //! SetPosition
    vctBoolVec JointsEnabled;
    vctDoubleVec SetpointPosition;
    prmPositionJointSet SetpointPositionParam;
    vctDoubleVec UnitFactor;

    size_t m_number_of_joints;

    // GUI: Commands
    QCheckBox * QCBEnableDirectControl;
    QCheckBox * QCBEnable;
    QCheckBox * QCBEnableTrackingError;
    QCheckBox * QCBEnforcePositionLimits;
    QCheckBox * QCBUseSetpointV;
    QPushButton * QPBMaintainPosition;
    QPushButton * QPBSave;
    vctQtWidgetDynamicVectorBoolWrite * QVWJointsEnabled;
    vctQtWidgetDynamicVectorDoubleWrite * QVWSetpointPosition;
    vctQtWidgetDynamicVectorDoubleRead * QVRMeasuredPosition;
    vctQtWidgetDynamicVectorDoubleRead * QVWSetpointEffort;
    vctQtWidgetDynamicVectorDoubleRead * QVRMeasuredEffort;
    vctQtWidgetDynamicVectorDoubleWrite * QVWPGain;
    vctQtWidgetDynamicVectorDoubleWrite * QVWDGain;
    vctQtWidgetDynamicVectorDoubleWrite * QVWIGain;
    vctQtWidgetDynamicVectorDoubleWrite * QVWDeadband;
    vctQtWidgetDynamicVectorDoubleWrite * QVWCutoff;

    // GUI: plot
    vctPlot2DOpenGLQtWidget * QVPlot;
    vctPlot2DBase::Signal * signal_measured_p;
    vctPlot2DBase::Signal * signal_setpoint_p;
    vctPlot2DBase::Signal * signal_measured_v;
    vctPlot2DBase::Signal * signal_setpoint_v;
    vctPlot2DBase::Signal * signal_measured_f;
    vctPlot2DBase::Signal * signal_setpoint_f;
    vctPlot2DBase::Signal * signal_disturbance;
    QSpinBox * QSBPlotIndex;
    int PlotIndex;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsPIDQtWidget);

#endif // _mtsPIDQtWidget_h
