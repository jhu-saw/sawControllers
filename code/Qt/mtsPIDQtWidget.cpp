/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-20

  (C) Copyright 2013-2015 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstParameterTypes/prmJointType.h>
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
        interfaceRequired->AddFunction("EnableTorqueMode", PID.EnableTorqueMode);
        interfaceRequired->AddFunction("SetPositionJoint", PID.SetPositionJoint);
        interfaceRequired->AddFunction("GetStateJoint", PID.GetStateJoint);
        interfaceRequired->AddFunction("GetStateJointDesired", PID.GetStateJointDesired);
        interfaceRequired->AddFunction("GetJointType", PID.GetJointType);
        interfaceRequired->AddFunction("GetPGain", PID.GetPGain);
        interfaceRequired->AddFunction("GetDGain", PID.GetDGain);
        interfaceRequired->AddFunction("GetIGain", PID.GetIGain);
        interfaceRequired->AddFunction("SetPGain", PID.SetPGain);
        interfaceRequired->AddFunction("SetDGain", PID.SetDGain);
        interfaceRequired->AddFunction("SetIGain", PID.SetIGain);
        // Events
        interfaceRequired->AddEventHandlerWrite(&mtsPIDQtWidget::JointLimitEventHandler, this, "JointLimit");
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
    // Set desired pos to cur pos
    SlotResetPIDGain();
    SlotMaintainPosition();
    mtsExecutionResult result;
    prmJointTypeVec jointType;
    result = PID.GetJointType(jointType);
    if (!result) {
        CMN_LOG_CLASS_INIT_ERROR << "Startup: Robot interface isn't connected properly, unable to get joint type.  Function call returned: "
                                 << result << std::endl;
        UnitFactor.SetAll(0.0);
    } else {
        // set unitFactor;
        for (size_t i = 0; i < this->NumberOfAxis; i++) {
            if (jointType[i] == PRM_REVOLUTE) {
                UnitFactor[i] = cmn180_PI;
            } else if (jointType[i] == PRM_PRISMATIC) {
                UnitFactor[i] = 1.0 / cmn_mm;
            } else {
                cmnThrow("mtsRobotIO1394QtWidget: Unknown joint type");
            }
        }
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

void mtsPIDQtWidget::SlotEnablePID(bool toggle)
{
    PID.Enable(toggle);
}

void mtsPIDQtWidget::SlotEnableTorqueMode(bool toggle)
{
    vctBoolVec torqueMode(NumberOfAxis, toggle);
    PID.EnableTorqueMode(torqueMode);
}

void mtsPIDQtWidget::SlotPositionChanged(void)
{
    DesiredPosition.SetAll(0.0);
    QVWDesiredPositionWidget->GetValue(DesiredPosition);
    DesiredPositionParam.SetGoal(DesiredPosition);
    DesiredPositionParam.Goal().ElementwiseDivide(UnitFactor);
    PID.SetPositionJoint(DesiredPositionParam);
}

void mtsPIDQtWidget::SlotPGainChanged(void)
{
    vctDoubleVec pgain(NumberOfAxis, 0.0);
    QVWPGainWidget->GetValue(pgain);
    PID.SetPGain(pgain);
}

void mtsPIDQtWidget::SlotDGainChanged(void)
{
    vctDoubleVec dgain(NumberOfAxis, 0.0);
    QVWDGainWidget->GetValue(dgain);
    PID.SetDGain(dgain);
}

void mtsPIDQtWidget::SlotIGainChanged(void)
{
    vctDoubleVec igain(NumberOfAxis, 0.0);
    QVWIGainWidget->GetValue(igain);
    PID.SetIGain(igain);
}

void mtsPIDQtWidget::SlotMaintainPosition(void)
{
    // reset desired position
    PID.StateJoint.Position().ElementwiseMultiply(UnitFactor);
    QVWDesiredPositionWidget->SetValue(PID.StateJoint.Position());
    PID.ResetController();
    SlotPositionChanged();
}

void mtsPIDQtWidget::SlotZeroPosition(void)
{
    // reset desired position
    DesiredPosition.SetAll(0.0);
    QVWDesiredPositionWidget->SetValue(DesiredPosition);
    PID.ResetController();
    SlotPositionChanged();
}

void mtsPIDQtWidget::SlotResetPIDGain(void)
{
    // get gains
    vctDoubleVec gain;
    gain.SetSize(NumberOfAxis);
    // PGain
    PID.GetPGain(gain);
    QVWPGainWidget->SetValue(gain);
    // DGain
    PID.GetDGain(gain);
    QVWDGainWidget->SetValue(gain);
    // IGain
    PID.GetIGain(gain);
    QVWIGainWidget->SetValue(gain);
}

void mtsPIDQtWidget::SlotPlotIndex(int newAxis)
{
    PlotIndex = newAxis;
    QVPlot->SetContinuousExpandYResetSlot();
}

void mtsPIDQtWidget::SlotEnableEventHandler(bool enable)
{
    QCBEnablePID->setChecked(enable);
}

void mtsPIDQtWidget::SlotEnableDirectControl(bool toggle)
{
    DirectControl = toggle;
    // if checked in DIRECT_CONTROL mode
    QVWDesiredPositionWidget->setEnabled(toggle);
    QVWPGainWidget->setEnabled(toggle);
    QVWIGainWidget->setEnabled(toggle);
    QVWDGainWidget->setEnabled(toggle);
    QCBEnablePID->setEnabled(toggle);
    QCBEnableTorqueMode->setEnabled(toggle);
    QPBMaintainPosition->setEnabled(toggle);
    QPBZeroPosition->setEnabled(toggle);
    QPBResetPIDGain->setEnabled(toggle);
}

void mtsPIDQtWidget::timerEvent(QTimerEvent * CMN_UNUSED(event))
{
    // make sure we should update the display
    if (this->isHidden()) {
        return;
    }

    // get data from the PID
    PID.GetStateJoint(PID.StateJoint);
    PID.StateJoint.Position().ElementwiseMultiply(UnitFactor);
    PID.StateJoint.Velocity().ElementwiseMultiply(UnitFactor);
    PID.StateJoint.Effort().ElementwiseMultiply(UnitFactor);
    PID.GetStateJointDesired(PID.StateJointDesired);
    PID.StateJointDesired.Position().ElementwiseMultiply(UnitFactor);
    PID.StateJointDesired.Effort().ElementwiseMultiply(UnitFactor);

    // update GUI
    QVRCurrentPositionWidget->SetValue(PID.StateJoint.Position());
    QVRCurrentEffortWidget->SetValue(PID.StateJoint.Effort());

    // display requested joint positions when we are not trying to set it using GUI
    if (!DirectControl) {
        QVWDesiredPositionWidget->SetValue(PID.StateJointDesired.Position());
        QVWDesiredEffortWidget->SetValue(PID.StateJointDesired.Effort());
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
    QVPlot->updateGL();
}

////------------ Private Methods ----------------
void mtsPIDQtWidget::setupUi(void)
{
    QFont font;
    font.setBold(true);
    font.setPointSize(12);

    const double maximum = 30000;

    QGridLayout * gridLayout = new QGridLayout();
    gridLayout->setSpacing(1);

    int row = 0;
    QLabel * currentPosLabel = new QLabel("Current position (deg)");
    currentPosLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(currentPosLabel, row, 0);
    QVRCurrentPositionWidget = new vctQtWidgetDynamicVectorDoubleRead();
    QVRCurrentPositionWidget->SetPrecision(3);
    gridLayout->addWidget(QVRCurrentPositionWidget, row, 1);
    row++;

    QLabel * desiredPosLabel = new QLabel("Desired position (deg)");
    desiredPosLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(desiredPosLabel, row, 0);
    QVWDesiredPositionWidget = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SPINBOX_WIDGET);
    QVWDesiredPositionWidget->SetStep(0.1);
    QVWDesiredPositionWidget->SetRange(-360.0, 360.0);
    gridLayout->addWidget(QVWDesiredPositionWidget, row, 1);
    row++;

    QLabel * currentEffortLabel = new QLabel("Current effort (Nm)");
    currentEffortLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(currentEffortLabel, row, 0);
    QVRCurrentEffortWidget = new vctQtWidgetDynamicVectorDoubleRead();
    QVRCurrentEffortWidget->SetPrecision(3);
    gridLayout->addWidget(QVRCurrentEffortWidget, row, 1);
    row++;

    QLabel * desiredEffortLabel = new QLabel("Desired effort (Nm)");
    desiredEffortLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(desiredEffortLabel, row, 0);
    QVWDesiredEffortWidget = new vctQtWidgetDynamicVectorDoubleRead();
    QVWDesiredEffortWidget->SetPrecision(3);
    gridLayout->addWidget(QVWDesiredEffortWidget, row, 1);
    row++;

    QLabel * pLabel = new QLabel("PGain");
    pLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(pLabel);
    QVWPGainWidget = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SPINBOX_WIDGET);
    QVWPGainWidget->SetStep(0.01);
    QVWPGainWidget->SetPrecision(3);
    QVWPGainWidget->SetRange(-maximum, maximum);
    gridLayout->addWidget(QVWPGainWidget, row, 1);
    row++;

    QLabel * dLabel = new QLabel("DGain");
    dLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(dLabel);
    QVWDGainWidget = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SPINBOX_WIDGET);
    QVWDGainWidget->SetStep(0.01);
    QVWDGainWidget->SetPrecision(3);
    QVWDGainWidget->SetRange(-maximum, maximum);
    gridLayout->addWidget(QVWDGainWidget, row, 1);
    row++;

    QLabel * iLabel = new QLabel("IGain");
    iLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(iLabel);
    QVWIGainWidget = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SPINBOX_WIDGET);
    QVWIGainWidget->SetStep(0.001);
    QVWIGainWidget->SetPrecision(5);
    QVWIGainWidget->SetRange(-maximum, maximum);
    gridLayout->addWidget(QVWIGainWidget, row, 1);
    row++;

    // plot
    QHBoxLayout * plotLayout = new QHBoxLayout;
    // plot control
    QVBoxLayout * plotButtonsLayout = new QVBoxLayout;
    // - pick axis to display
    QLabel * plotIndexLabel = new QLabel("Index");
    plotButtonsLayout->addWidget(plotIndexLabel);
    QSBPlotIndex = new QSpinBox();
    QSBPlotIndex->setRange(0, NumberOfAxis);
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
    QCBEnablePID = new QCheckBox("Enable PID");
    QCBEnableTorqueMode = new QCheckBox("Enable torque mode");
    QPBMaintainPosition = new QPushButton("Maintain position");
    QPBZeroPosition = new QPushButton("Zero position");
    QPBResetPIDGain = new QPushButton("Reset PID gains");
    QHBoxLayout * controlLayout = new QHBoxLayout;
    controlLayout->addWidget(QCBEnableDirectControl);
    controlLayout->addWidget(QCBEnablePID);
    controlLayout->addWidget(QCBEnableTorqueMode);
    controlLayout->addWidget(QPBMaintainPosition);
    controlLayout->addWidget(QPBZeroPosition);
    controlLayout->addWidget(QPBResetPIDGain);
    controlLayout->addStretch();
    QGroupBox * controlGroupBox = new QGroupBox("Control");
    controlGroupBox->setLayout(controlLayout);

    connect(QCBEnableDirectControl, SIGNAL(toggled(bool)), this, SLOT(SlotEnableDirectControl(bool)));
    connect(QCBEnablePID, SIGNAL(clicked(bool)), this, SLOT(SlotEnablePID(bool)));
    connect(this, SIGNAL(SignalEnablePID(bool)), this, SLOT(SlotEnableEventHandler(bool)));
    connect(QCBEnableTorqueMode, SIGNAL(toggled(bool)), this, SLOT(SlotEnableTorqueMode(bool)));
    connect(QPBMaintainPosition, SIGNAL(clicked()), this, SLOT(SlotMaintainPosition()));
    connect(QPBZeroPosition, SIGNAL(clicked()), this, SLOT(SlotZeroPosition()));
    connect(QPBResetPIDGain, SIGNAL(clicked()), this, SLOT(SlotResetPIDGain()));
    connect(QSBPlotIndex, SIGNAL(valueChanged(int)), this, SLOT(SlotPlotIndex(int)));

    // main layout
    QVBoxLayout * mainLayout = new QVBoxLayout;
    mainLayout->addLayout(gridLayout);
    mainLayout->addLayout(plotLayout);
    mainLayout->addWidget(controlGroupBox);

    setLayout(mainLayout);

    setWindowTitle(this->GetName().c_str());
    setMinimumWidth(750);
    resize(sizeHint());

    // connect signals & slots
    connect(QVWDesiredPositionWidget, SIGNAL(valueChanged()), this, SLOT(SlotPositionChanged()));
    connect(QVWPGainWidget, SIGNAL(valueChanged()), this, SLOT(SlotPGainChanged()));
    connect(QVWDGainWidget, SIGNAL(valueChanged()), this, SLOT(SlotDGainChanged()));
    connect(QVWIGainWidget, SIGNAL(valueChanged()), this, SLOT(SlotIGainChanged()));

    // set initial values
    QCBEnableDirectControl->setChecked(DirectControl);
    SlotEnableDirectControl(DirectControl);
}

void mtsPIDQtWidget::JointLimitEventHandler(const vctBoolVec & flags)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "JointLimitEventHandler: " << flags << std::endl;
}

void mtsPIDQtWidget::ErrorEventHandler(const std::string & message)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "ErrorEventHandler: " << message << std::endl;
}

void mtsPIDQtWidget::EnableEventHandler(const bool & enable)
{
    emit SignalEnablePID(enable);
}
