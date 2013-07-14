/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-20

  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsPIDQtWidget_h
#define _mtsPIDQtWidget_h

#include <cisstCommon/cmnXMLPath.h>
#include <cisstVector/vctPlot2DOpenGLQtWidget.h>
#include <cisstMultiTask/mtsComponent.h>
#include <cisstVector/vctQtWidgetDynamicVector.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionJointSet.h>

#include <QtCore>
#include <QtGui>


class mtsPIDQtWidget: public QWidget, public mtsComponent
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsPIDQtWidget(const std::string & componentName, unsigned int numberOfAxis);
    mtsPIDQtWidget(const mtsComponentConstructorNameAndUInt &arg);
    ~mtsPIDQtWidget(){}

    void Configure(const std::string & filename = "");
    void Startup();
    void Cleanup();

protected:
    void Init(void);
    virtual void closeEvent(QCloseEvent * event);

private slots:
    //! slot enable/disable mtsPID controller
    void SlotEnablePID(bool toggle);
    void SlotEnableTrqMode(bool toggle);
    //! slot send desired pos when input changed
    void SlotPositionChanged(void);
    void SlotPGainChanged(void);
    void SlotDGainChanged(void);
    void SlotIGainChanged(void);
    //! slot reset desired pos to current pos
    void SlotMaintainPosition(void);
    //! go to zero position
    void SlotZeroPosition(void);
    //! slot reset pid gain to current gain
    void SlotResetPIDGain(void);
    //! slot to select which axis to plot
    void SlotPlotIndex(int newAxis);

    void timerEvent(QTimerEvent * event);

private:
    //! setup PID controller GUI
    void setupUi(void);
    void EventErrorLimitHandler(void);

protected:

    struct ControllerPIDStruct {
        mtsFunctionVoid  ResetController;
        mtsFunctionWrite Enable;
        mtsFunctionWrite EnableTrqMode;
        mtsFunctionWrite SetPositionJoint;
        mtsFunctionRead  GetPositionJoint;
        mtsFunctionRead  GetEffortJoint;

        prmPositionJointGet PositionJointGetParam;
        vctDoubleVec EffortJoint;

        mtsFunctionRead  GetJointType;
        mtsFunctionRead  GetPGain;
        mtsFunctionRead  GetDGain;
        mtsFunctionRead  GetIGain;

        mtsFunctionWrite SetPGain;
        mtsFunctionWrite SetDGain;
        mtsFunctionWrite SetIGain;
    } PID;

private:
    //! SetPosition
    vctDoubleVec DesiredPosition;
    prmPositionJointSet DesiredPositionParam;
    vctDoubleVec UnitFactor;

    int NumberOfAxis;

    // GUI: Commands
    QCheckBox * QCBEnablePID;
    QCheckBox * QCBEnableTrqMode;
    vctQtWidgetDynamicVectorDoubleWrite * QVWDesiredPositionWidget;
    vctQtWidgetDynamicVectorDoubleWrite * QVWPGainWidget;
    vctQtWidgetDynamicVectorDoubleWrite * QVWDGainWidget;
    vctQtWidgetDynamicVectorDoubleWrite * QVWIGainWidget;
    vctQtWidgetDynamicVectorDoubleRead * QVRCurrentPositionWidget;
    vctQtWidgetDynamicVectorDoubleRead * QVRCurrentEffortWidget;

    // GUI: plot
    vctPlot2DOpenGLQtWidget * QVPlot;
    vctPlot2DBase::Signal * CurrentPositionSignal;
    vctPlot2DBase::Signal * DesiredPositionSignal;
    QSpinBox * QSBPlotIndex;
    int PlotIndex;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsPIDQtWidget);

#endif // _mtsPIDQtWidget_h
