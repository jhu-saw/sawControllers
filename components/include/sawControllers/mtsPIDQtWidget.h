/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-20

  (C) Copyright 2013-2017 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstParameterTypes/prmPositionJointSet.h>

#include <QCheckBox>
#include <QSpinBox>
#include <QPushButton>
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
    virtual void closeEvent(QCloseEvent * event);

signals:
    void SignalEnable(bool enable);

private slots:
    //! slot enable/disable mtsPID controller
    void SlotEnable(bool toggle);
    void SlotEnabledJointsChanged(void);
    void SlotEnableTrackingError(bool toggle);
    //! slot send desired pos when input changed
    void SlotPositionChanged(void);
    void SlotPGainChanged(void);
    void SlotDGainChanged(void);
    void SlotIGainChanged(void);
    //! slot reset desired pos to current pos
    void SlotMaintainPosition(void);
    //! slot reset pid gain to current gain
    void SlotResetPIDGain(void);
    //! slot to select which axis to plot
    void SlotPlotIndex(int newAxis);
    //! slot to change Enable Checkbox
    void SlotEnableEventHandler(bool enable);
    void SlotEnableDirectControl(bool toggle);

    //! timer event to update GUI
    void timerEvent(QTimerEvent * event);

private:
    //! setup PID controller GUI
    void setupUi(void);
    int TimerPeriodInMilliseconds;

    void ErrorEventHandler(const mtsMessage & message);
    void EnableEventHandler(const bool & enable);

protected:

    struct ControllerPIDStruct {
        mtsFunctionVoid  ResetController;
        mtsFunctionWrite Enable;
        mtsFunctionWrite EnableJoints;
        mtsFunctionRead  JointsEnabled;
        mtsFunctionWrite EnableTrackingError;
        mtsFunctionRead  TrackingErrorEnabled;
        mtsFunctionWrite SetPositionJoint;
        mtsFunctionRead  GetStateJoint;
        mtsFunctionRead  GetStateJointDesired;

        prmStateJoint    StateJoint;
        prmStateJoint    StateJointDesired;

        mtsFunctionRead  GetPGain;
        mtsFunctionRead  GetDGain;
        mtsFunctionRead  GetIGain;

        mtsFunctionWrite SetPGain;
        mtsFunctionWrite SetDGain;
        mtsFunctionWrite SetIGain;
    } PID;

private:
    bool DirectControl;

    //! SetPosition
    vctBoolVec JointsEnabled;
    vctDoubleVec DesiredPosition;
    prmPositionJointSet DesiredPositionParam;
    vctDoubleVec UnitFactor;

    size_t NumberOfAxis;

    // GUI: Commands
    QCheckBox * QCBEnableDirectControl;
    QCheckBox * QCBEnable;
    QCheckBox * QCBEnableTrackingError;
    QPushButton * QPBMaintainPosition;
    QPushButton * QPBResetPIDGain;
    vctQtWidgetDynamicVectorBoolWrite * QVWJointsEnabled;
    vctQtWidgetDynamicVectorDoubleWrite * QVWDesiredPosition;
    vctQtWidgetDynamicVectorDoubleRead * QVRCurrentPosition;
    vctQtWidgetDynamicVectorDoubleRead * QVWDesiredEffort;
    vctQtWidgetDynamicVectorDoubleRead * QVRCurrentEffort;
    vctQtWidgetDynamicVectorDoubleWrite * QVWPGain;
    vctQtWidgetDynamicVectorDoubleWrite * QVWDGain;
    vctQtWidgetDynamicVectorDoubleWrite * QVWIGain;

    // GUI: plot
    vctPlot2DOpenGLQtWidget * QVPlot;
    vctPlot2DBase::Signal * CurrentPositionSignal;
    vctPlot2DBase::Signal * DesiredPositionSignal;
    vctPlot2DBase::Signal * CurrentVelocitySignal;
    vctPlot2DBase::Signal * DesiredEffortSignal;
    vctPlot2DBase::Signal * CurrentEffortSignal;
    QSpinBox * QSBPlotIndex;
    int PlotIndex;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsPIDQtWidget);

#endif // _mtsPIDQtWidget_h
