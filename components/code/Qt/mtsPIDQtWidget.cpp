/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-20

  (C) Copyright 2013-2019 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


// system include
#include <iostream>

// Qt include
#include <QMessageBox>
#include <QGridLayout>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QCloseEvent>
#include <QCoreApplication>

// cisst
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstCommon/cmnConstants.h>
#include <cisstCommon/cmnUnits.h>

#include <sawControllers/mtsPIDQtWidget.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsPIDQtWidget, mtsComponent, mtsComponentConstructorNameAndUInt)

mtsPIDQtWidget::mtsPIDQtWidget(const std::string & componentName,
                               unsigned int numberOfAxis,
                               double periodInSeconds):
    mtsComponent(componentName),
    TimerPeriodInMilliseconds(periodInSeconds * 1000), // Qt timer are in milliseconds
    NumberOfAxis(numberOfAxis)
{
    Init();
}

mtsPIDQtWidget::mtsPIDQtWidget(const mtsComponentConstructorNameAndUInt & arg):
    mtsComponent(arg.Name),
    TimerPeriodInMilliseconds(50),
    NumberOfAxis(arg.Arg)
{
    Init();
}

void mtsPIDQtWidget::Init(void)
{
    PID.StateJoint.Position().SetSize(NumberOfAxis);
    PID.StateJoint.Velocity().SetSize(NumberOfAxis);
    PID.StateJoint.Effort().SetSize(NumberOfAxis);
    PID.StateJointDesired.Position().SetSize(NumberOfAxis);
    PID.StateJointDesired.Velocity().SetSize(0);
    PID.StateJointDesired.Effort().SetSize(NumberOfAxis);

    DesiredPosition.SetSize(NumberOfAxis);
    DesiredPosition.SetAll(0.0);
    UnitFactor.SetSize(NumberOfAxis);
    UnitFactor.SetAll(1.0);

    DirectControl = false;
    PlotIndex = 0;

    // Setup cisst interface
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("Controller");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("ResetController", PID.ResetController);
        interfaceRequired->AddFunction("Enable", PID.Enable);
        interfaceRequired->AddFunction("EnableJoints", PID.EnableJoints);
        interfaceRequired->AddFunction("JointsEnabled", PID.JointsEnabled);
        interfaceRequired->AddFunction("EnableTrackingError", PID.EnableTrackingError);
        interfaceRequired->AddFunction("TrackingErrorEnabled", PID.TrackingErrorEnabled);
        interfaceRequired->AddFunction("SetPositionJoint", PID.SetPositionJoint);
        interfaceRequired->AddFunction("GetStateJoint", PID.GetStateJoint);
        interfaceRequired->AddFunction("GetStateJointDesired", PID.GetStateJointDesired);
        interfaceRequired->AddFunction("GetPGain", PID.GetPGain);
        interfaceRequired->AddFunction("GetDGain", PID.GetDGain);
        interfaceRequired->AddFunction("GetIGain", PID.GetIGain);
        interfaceRequired->AddFunction("SetPGain", PID.SetPGain);
        interfaceRequired->AddFunction("SetDGain", PID.SetDGain);
        interfaceRequired->AddFunction("SetIGain", PID.SetIGain);
        // Events
        interfaceRequired->AddEventHandlerWrite(&mtsPIDQtWidget::ErrorEventHandler, this, "Error");
        interfaceRequired->AddEventHandlerWrite(&mtsPIDQtWidget::EnableEventHandler, this, "Enabled");
    }
    setupUi();
    startTimer(TimerPeriodInMilliseconds); // ms
}

void mtsPIDQtWidget::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsPIDQtWidget::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsPIDQtWidget::Startup" << std::endl;
    // get joint state just to compute conversion factors
    SlotResetPIDGain();
    mtsExecutionResult result = PID.GetStateJoint(PID.StateJoint);
    if (!result) {
        CMN_LOG_CLASS_INIT_ERROR << "Startup: Robot interface isn't connected properly, unable to get joint type.  Function call returned: "
                                 << result << std::endl;
        UnitFactor.SetAll(0.0);
    } else {
        // set unitFactor;
        prmJointTypeToFactor(PID.StateJoint.Type(), 1.0 / cmn_mm, cmn180_PI, UnitFactor);
    }

    // Show the GUI
    if (!parent()) {
        show();
    }
}

void mtsPIDQtWidget::Cleanup(void)
{
    this->hide();
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsPIDQtWidget::Cleanup" << std::endl;
}

//---------- Protected --------------------------
void mtsPIDQtWidget::closeEvent(QCloseEvent * event)
{
    int answer = QMessageBox::warning(this, tr("mtsPIDQtWidget"),
                                      tr("Do you really want to quit this application?"),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::Yes) {
        event->accept();
        QCoreApplication::exit();
    } else {
        event->ignore();
    }
}

void mtsPIDQtWidget::SlotEnable(bool toggle)
{
    PID.Enable(toggle);
}

void mtsPIDQtWidget::SlotEnabledJointsChanged(void)
{
    vctBoolVec enable(NumberOfAxis, false);
    QVWJointsEnabled->GetValue(enable);
    PID.EnableJoints(enable);
}

void mtsPIDQtWidget::SlotEnableTrackingError(bool toggle)
{
    PID.EnableTrackingError(toggle);
}

void mtsPIDQtWidget::SlotPositionChanged(void)
{
    DesiredPosition.SetAll(0.0);
    QVWDesiredPosition->GetValue(DesiredPosition);
    DesiredPositionParam.SetGoal(DesiredPosition);
    DesiredPositionParam.Goal().ElementwiseDivide(UnitFactor);
    PID.SetPositionJoint(DesiredPositionParam);
}

void mtsPIDQtWidget::SlotPGainChanged(void)
{
    vctDoubleVec pgain(NumberOfAxis, 0.0);
    QVWPGain->GetValue(pgain);
    PID.SetPGain(pgain);
}

void mtsPIDQtWidget::SlotDGainChanged(void)
{
    vctDoubleVec dgain(NumberOfAxis, 0.0);
    QVWDGain->GetValue(dgain);
    PID.SetDGain(dgain);
}

void mtsPIDQtWidget::SlotIGainChanged(void)
{
    vctDoubleVec igain(NumberOfAxis, 0.0);
    QVWIGain->GetValue(igain);
    PID.SetIGain(igain);
}

void mtsPIDQtWidget::SlotMaintainPosition(void)
{
    // reset desired position
    vctDoubleVec goal(PID.StateJoint.Position());
    QVWDesiredPosition->SetValue(goal);
    SlotPositionChanged();
}

void mtsPIDQtWidget::SlotResetPIDGain(void)
{
    // get gains
    vctDoubleVec gain;
    gain.SetSize(NumberOfAxis);
    // PGain
    PID.GetPGain(gain);
    QVWPGain->SetValue(gain);
    // DGain
    PID.GetDGain(gain);
    QVWDGain->SetValue(gain);
    // IGain
    PID.GetIGain(gain);
    QVWIGain->SetValue(gain);
}

void mtsPIDQtWidget::SlotPlotIndex(int newAxis)
{
    PlotIndex = newAxis;
    QVPlot->SetContinuousExpandYResetSlot();
}

void mtsPIDQtWidget::SlotEnableEventHandler(bool enable)
{
    QCBEnable->setChecked(enable);
}

void mtsPIDQtWidget::SlotEnableDirectControl(bool toggle)
{
    if (toggle) {
        int answer = QMessageBox::warning(this, tr("mtsPIDQtWidget"),
                                          tr("In direct control mode you can potentially harm your robot.\nAre you sure you want to continue?"),
                                          QMessageBox::No | QMessageBox::Yes);
        if (answer == QMessageBox::No) {
            toggle = false;
        }
    }
    QCBEnableDirectControl->setChecked(toggle);
    DirectControl = toggle;
    // if checked in DIRECT_CONTROL mode
    QVWJointsEnabled->setEnabled(toggle);
    QVWDesiredPosition->setEnabled(toggle);
    QVWPGain->setEnabled(toggle);
    QVWIGain->setEnabled(toggle);
    QVWDGain->setEnabled(toggle);
    QCBEnable->setEnabled(toggle);
    QCBEnableTrackingError->setEnabled(toggle);
    QPBMaintainPosition->setEnabled(toggle);
    QPBResetPIDGain->setEnabled(toggle);
}

void mtsPIDQtWidget::timerEvent(QTimerEvent * CMN_UNUSED(event))
{
    // make sure we should update the display
    if (this->isHidden()) {
        return;
    }

    // get data from the PID
    PID.JointsEnabled(JointsEnabled);
    PID.GetStateJoint(PID.StateJoint);
    PID.StateJoint.Position().ElementwiseMultiply(UnitFactor);
    PID.StateJoint.Velocity().ElementwiseMultiply(UnitFactor);
    PID.GetStateJointDesired(PID.StateJointDesired);
    PID.StateJointDesired.Position().ElementwiseMultiply(UnitFactor);
    bool trackingErrorEnabled;
    PID.TrackingErrorEnabled(trackingErrorEnabled);

    // update GUI
    QVWJointsEnabled->SetValue(JointsEnabled);
    QVRCurrentPosition->SetValue(PID.StateJoint.Position());
    QVRCurrentEffort->SetValue(PID.StateJoint.Effort());
    QCBEnableTrackingError->setChecked(trackingErrorEnabled);

    // display requested joint positions when we are not trying to set it using GUI
    if (!DirectControl) {
        QVWDesiredPosition->SetValue(PID.StateJointDesired.Position());
        QVWDesiredEffort->SetValue(PID.StateJointDesired.Effort());
    }

    // plot
    CurrentPositionSignal->AppendPoint(vctDouble2(PID.StateJoint.Timestamp(),
                                                  PID.StateJoint.Position().Element(PlotIndex)));
    DesiredPositionSignal->AppendPoint(vctDouble2(PID.StateJointDesired.Timestamp(),
                                                  PID.StateJointDesired.Position().Element(PlotIndex)));
    CurrentVelocitySignal->AppendPoint(vctDouble2(PID.StateJoint.Timestamp(),
                                                  PID.StateJoint.Velocity().Element(PlotIndex)));
    // negate effort to plot the same direction
    CurrentEffortSignal->AppendPoint(vctDouble2(PID.StateJoint.Timestamp(),
                                                -PID.StateJoint.Effort().Element(PlotIndex)));
    DesiredEffortSignal->AppendPoint(vctDouble2(PID.StateJointDesired.Timestamp(),
                                                -PID.StateJointDesired.Effort().Element(PlotIndex)));
    QVPlot->update();
}

////------------ Private Methods ----------------
void mtsPIDQtWidget::setupUi(void)
{
    QFont font;
    font.setBold(true);
    font.setPointSize(12);

    const double maximum = 30000.0;

    QGridLayout * gridLayout = new QGridLayout();
    gridLayout->setSpacing(1);

    int row = 0;

    QLabel * jointsEnabledLabel = new QLabel("Joints enabled");
    jointsEnabledLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(jointsEnabledLabel, row, 0);
    QVWJointsEnabled = new vctQtWidgetDynamicVectorBoolWrite();
    gridLayout->addWidget(QVWJointsEnabled, row, 1);
    row++;

    QLabel * currentPosLabel = new QLabel("Current position (deg)");
    currentPosLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(currentPosLabel, row, 0);
    QVRCurrentPosition = new vctQtWidgetDynamicVectorDoubleRead();
    QVRCurrentPosition->SetPrecision(3);
    gridLayout->addWidget(QVRCurrentPosition, row, 1);
    row++;

    QLabel * desiredPosLabel = new QLabel("Desired position (deg)");
    desiredPosLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(desiredPosLabel, row, 0);
    QVWDesiredPosition = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SPINBOX_WIDGET);
    QVWDesiredPosition->SetStep(0.1);
    QVWDesiredPosition->SetRange(-360.0, 360.0);
    gridLayout->addWidget(QVWDesiredPosition, row, 1);
    row++;

    QLabel * currentEffortLabel = new QLabel("Current effort (Nm)");
    currentEffortLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(currentEffortLabel, row, 0);
    QVRCurrentEffort = new vctQtWidgetDynamicVectorDoubleRead();
    QVRCurrentEffort->SetPrecision(3);
    gridLayout->addWidget(QVRCurrentEffort, row, 1);
    row++;

    QLabel * desiredEffortLabel = new QLabel("Desired effort (Nm)");
    desiredEffortLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(desiredEffortLabel, row, 0);
    QVWDesiredEffort = new vctQtWidgetDynamicVectorDoubleRead();
    QVWDesiredEffort->SetPrecision(3);
    gridLayout->addWidget(QVWDesiredEffort, row, 1);
    row++;

    QLabel * pLabel = new QLabel("PGain");
    pLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(pLabel);
    QVWPGain = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SPINBOX_WIDGET);
    QVWPGain->SetStep(0.01);
    QVWPGain->SetPrecision(3);
    QVWPGain->SetRange(-maximum, maximum);
    gridLayout->addWidget(QVWPGain, row, 1);
    row++;

    QLabel * dLabel = new QLabel("DGain");
    dLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(dLabel);
    QVWDGain = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SPINBOX_WIDGET);
    QVWDGain->SetStep(0.01);
    QVWDGain->SetPrecision(3);
    QVWDGain->SetRange(-maximum, maximum);
    gridLayout->addWidget(QVWDGain, row, 1);
    row++;

    QLabel * iLabel = new QLabel("IGain");
    iLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(iLabel);
    QVWIGain = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SPINBOX_WIDGET);
    QVWIGain->SetStep(0.001);
    QVWIGain->SetPrecision(5);
    QVWIGain->SetRange(-maximum, maximum);
    gridLayout->addWidget(QVWIGain, row, 1);
    row++;

    // plot
    QHBoxLayout * plotLayout = new QHBoxLayout;
    // plot control
    QVBoxLayout * plotButtonsLayout = new QVBoxLayout;
    // - pick axis to display
    QLabel * plotIndexLabel = new QLabel("Index");
    plotButtonsLayout->addWidget(plotIndexLabel);
    QSBPlotIndex = new QSpinBox();
    QSBPlotIndex->setRange(0, (NumberOfAxis > 0) ? (NumberOfAxis - 1) : 0);
    plotButtonsLayout->addWidget(QSBPlotIndex);
    // legend
    QLabel * label;
    QPalette palette;
    palette.setColor(QPalette::Window, Qt::black);
    label = new QLabel("Current position");
    label->setAutoFillBackground(true);
    palette.setColor(QPalette::WindowText, Qt::red);
    label->setPalette(palette);
    plotButtonsLayout->addWidget(label);
    label = new QLabel("Desired position");
    label->setAutoFillBackground(true);
    palette.setColor(QPalette::WindowText, Qt::green);
    label->setPalette(palette);
    plotButtonsLayout->addWidget(label);
    label = new QLabel("Current velocity");
    label->setAutoFillBackground(true);
    palette.setColor(QPalette::WindowText, Qt::gray);
    label->setPalette(palette);
    plotButtonsLayout->addWidget(label);
    label = new QLabel("Current effort");
    label->setAutoFillBackground(true);
    palette.setColor(QPalette::WindowText, Qt::cyan);
    label->setPalette(palette);
    plotButtonsLayout->addWidget(label);
    label = new QLabel("Desired effort");
    label->setAutoFillBackground(true);
    palette.setColor(QPalette::WindowText, Qt::white);
    label->setPalette(palette);
    plotButtonsLayout->addWidget(label);
    plotButtonsLayout->addStretch();
    plotLayout->addLayout(plotButtonsLayout);
    // plotting area
    QVPlot = new vctPlot2DOpenGLQtWidget();
    vctPlot2DBase::Scale * scalePosition = QVPlot->AddScale("positions");
    CurrentPositionSignal = scalePosition->AddSignal("current");
    CurrentPositionSignal->SetColor(vctDouble3(1.0, 0.0, 0.0));
    DesiredPositionSignal = scalePosition->AddSignal("desired");
    DesiredPositionSignal->SetColor(vctDouble3(0.0, 1.0, 0.0));
    vctPlot2DBase::Scale * scaleVelocity = QVPlot->AddScale("velocities");
    CurrentVelocitySignal = scaleVelocity->AddSignal("current");
    CurrentVelocitySignal->SetColor(vctDouble3(0.5, 0.5, 0.5));
    vctPlot2DBase::Scale * scaleEffort = QVPlot->AddScale("efforts");
    CurrentEffortSignal = scaleEffort->AddSignal("-current");
    CurrentEffortSignal->SetColor(vctDouble3(0.0, 1.0, 1.0));
    DesiredEffortSignal = scaleEffort->AddSignal("-desired");
    DesiredEffortSignal->SetColor(vctDouble3(1.0, 1.0, 1.0));
    QVPlot->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));
    plotLayout->addWidget(QVPlot);

    // control
    QCBEnableDirectControl = new QCheckBox("Direct control");
    QCBEnable = new QCheckBox("Enable PID");
    QCBEnableTrackingError = new QCheckBox("Enable tracking error");
    QPBMaintainPosition = new QPushButton("Maintain position");
    QPBResetPIDGain = new QPushButton("Reset PID gains");
    QHBoxLayout * controlLayout = new QHBoxLayout;
    controlLayout->addWidget(QCBEnableDirectControl);
    controlLayout->addWidget(QCBEnable);
    controlLayout->addWidget(QCBEnableTrackingError);
    controlLayout->addWidget(QPBMaintainPosition);
    controlLayout->addWidget(QPBResetPIDGain);
    QGroupBox * controlGroupBox = new QGroupBox("Control");
    controlGroupBox->setLayout(controlLayout);

    connect(QCBEnableDirectControl, SIGNAL(toggled(bool)), this, SLOT(SlotEnableDirectControl(bool)));
    connect(QCBEnable, SIGNAL(clicked(bool)), this, SLOT(SlotEnable(bool)));
    connect(this, SIGNAL(SignalEnable(bool)), this, SLOT(SlotEnableEventHandler(bool)));
    connect(QCBEnableTrackingError, SIGNAL(clicked(bool)), this, SLOT(SlotEnableTrackingError(bool)));
    connect(QPBMaintainPosition, SIGNAL(clicked()), this, SLOT(SlotMaintainPosition()));
    connect(QPBResetPIDGain, SIGNAL(clicked()), this, SLOT(SlotResetPIDGain()));
    connect(QSBPlotIndex, SIGNAL(valueChanged(int)), this, SLOT(SlotPlotIndex(int)));

    // main layout
    QVBoxLayout * mainLayout = new QVBoxLayout;
    mainLayout->addLayout(gridLayout);
    mainLayout->addLayout(plotLayout);
    mainLayout->addWidget(controlGroupBox);

    setLayout(mainLayout);

    setWindowTitle(this->GetName().c_str());
    resize(sizeHint());

    // connect signals & slots
    connect(QVWJointsEnabled, SIGNAL(valueChanged()), this, SLOT(SlotEnabledJointsChanged()));
    connect(QVWDesiredPosition, SIGNAL(valueChanged()), this, SLOT(SlotPositionChanged()));
    connect(QVWPGain, SIGNAL(valueChanged()), this, SLOT(SlotPGainChanged()));
    connect(QVWDGain, SIGNAL(valueChanged()), this, SLOT(SlotDGainChanged()));
    connect(QVWIGain, SIGNAL(valueChanged()), this, SLOT(SlotIGainChanged()));

    // set initial values
    QCBEnableDirectControl->setChecked(DirectControl);
    SlotEnableDirectControl(DirectControl);
}

void mtsPIDQtWidget::ErrorEventHandler(const mtsMessage & message)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "ErrorEventHandler: " << message << std::endl;
}

void mtsPIDQtWidget::EnableEventHandler(const bool & enable)
{
    emit SignalEnable(enable);
}
