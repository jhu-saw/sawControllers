/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-20

  (C) Copyright 2013-2022 Johns Hopkins University (JHU), All Rights Reserved.

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
    m_number_of_joints(numberOfAxis)
{
    Init();
}


mtsPIDQtWidget::mtsPIDQtWidget(const mtsComponentConstructorNameAndUInt & arg):
    mtsComponent(arg.Name),
    TimerPeriodInMilliseconds(50),
    m_number_of_joints(arg.Arg)
{
    Init();
}


void mtsPIDQtWidget::Init(void)
{
    PID.m_measured_js.Position().SetSize(m_number_of_joints);
    PID.m_measured_js.Velocity().SetSize(m_number_of_joints);
    PID.m_measured_js.Effort().SetSize(m_number_of_joints);
    PID.m_setpoint_js.Position().SetSize(m_number_of_joints);
    PID.m_setpoint_js.Velocity().SetSize(0);
    PID.m_setpoint_js.Effort().SetSize(m_number_of_joints);

    DesiredPosition.SetSize(m_number_of_joints);
    DesiredPosition.SetAll(0.0);
    UnitFactor.SetSize(m_number_of_joints);
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
        interfaceRequired->AddFunction("configuration", PID.configuration);
        interfaceRequired->AddFunction("configuration_js", PID.configuration_js);
        interfaceRequired->AddFunction("configure", PID.configure);
        interfaceRequired->AddFunction("servo_jp", PID.servo_jp);
        interfaceRequired->AddFunction("measured_js", PID.measured_js);
        interfaceRequired->AddFunction("setpoint_js", PID.setpoint_js);
        // Events
        interfaceRequired->AddEventHandlerWrite(&mtsPIDQtWidget::ErrorEventHandler, this, "error");
        interfaceRequired->AddEventHandlerWrite(&mtsPIDQtWidget::EnableEventHandler, this, "Enabled");
    }
    setupUi();
    startTimer(TimerPeriodInMilliseconds); // ms
}


void mtsPIDQtWidget::GetConfiguration(void)
{
    // get configuration
    PID.configuration(PID.m_configuration);
    // convert to vectors
    vctDoubleVec pg, ig, dg, co;
    pg.SetSize(m_number_of_joints);
    ig.SetSize(m_number_of_joints);
    dg.SetSize(m_number_of_joints);
    co.SetSize(m_number_of_joints);
    size_t index = 0;
    for (const auto & v : PID.m_configuration) {
        pg.at(index) = v.p_gain;
        ig.at(index) = v.i_gain;
        dg.at(index) = v.d_gain;
        co.at(index) = v.v_low_pass_cutoff;
        ++index;
    }
    // update widgets
    QVWPGain->SetValue(pg);
    QVWIGain->SetValue(ig);
    QVWDGain->SetValue(dg);
    QVWCutoff->SetValue(co);
}


void mtsPIDQtWidget::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}


void mtsPIDQtWidget::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsPIDQtWidget::Startup" << std::endl;
    // get joint state just to compute conversion factors
    GetConfiguration();
    mtsExecutionResult result = PID.configuration_js(PID.m_configuration_js);
    if (!result) {
        CMN_LOG_CLASS_INIT_ERROR << "Startup: Robot interface isn't connected properly, unable to get joint type.  Function call returned: "
                                 << result << std::endl;
        UnitFactor.SetAll(0.0);
    } else {
        // set unitFactor;
        prmJointTypeToFactor(PID.m_configuration_js.Type(), 1.0 / cmn_mm, cmn180_PI, UnitFactor);
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
    vctBoolVec enable(m_number_of_joints, false);
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
    PID.servo_jp(DesiredPositionParam);
}


void mtsPIDQtWidget::SlotConfigurationChanged(void)
{
    // make sure we have the latest configuration
    PID.configuration(PID.m_configuration);
    // get values from GUI
    vctDoubleVec pg(m_number_of_joints);
    QVWPGain->GetValue(pg);
    vctDoubleVec dg(m_number_of_joints);
    QVWDGain->GetValue(dg);
    vctDoubleVec ig(m_number_of_joints);
    QVWIGain->GetValue(ig);
    vctDoubleVec co(m_number_of_joints);
    QVWCutoff->GetValue(co);
    // copy to configuration
    size_t index = 0;
    for (auto & v : PID.m_configuration) {
        v.p_gain = pg.at(index);
        v.i_gain = ig.at(index);
        v.d_gain = dg.at(index);
        v.v_low_pass_cutoff = co.at(index);
        ++index;
    }
    // set
    PID.configure(PID.m_configuration);
}


void mtsPIDQtWidget::SlotMaintainPosition(void)
{
    // reset desired position
    vctDoubleVec goal(PID.m_measured_js.Position());
    QVWDesiredPosition->SetValue(goal);
    SlotPositionChanged();
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
    QVWJointsEnabled->setEnabled(toggle);
    QVWDesiredPosition->setEnabled(toggle);
    QVWPGain->setEnabled(toggle);
    QVWDGain->setEnabled(toggle);
    QVWIGain->setEnabled(toggle);
    QVWCutoff->setEnabled(toggle);
    QCBEnable->setEnabled(toggle);
    QCBEnableTrackingError->setEnabled(toggle);
    QPBMaintainPosition->setEnabled(toggle);
}


void mtsPIDQtWidget::timerEvent(QTimerEvent * CMN_UNUSED(event))
{
    // make sure we should update the display
    if (this->isHidden()) {
        return;
    }

    // get data from the PID
    PID.JointsEnabled(JointsEnabled);
    PID.measured_js(PID.m_measured_js);
    PID.m_measured_js.Position().ElementwiseMultiply(UnitFactor);
    PID.m_measured_js.Velocity().ElementwiseMultiply(UnitFactor);
    PID.setpoint_js(PID.m_setpoint_js);
    PID.m_setpoint_js.Position().ElementwiseMultiply(UnitFactor);
    bool trackingErrorEnabled;
    PID.TrackingErrorEnabled(trackingErrorEnabled);

    // update GUI
    QVWJointsEnabled->SetValue(JointsEnabled);
    QVRCurrentPosition->SetValue(PID.m_measured_js.Position());
    QVRCurrentEffort->SetValue(PID.m_measured_js.Effort());
    QCBEnableTrackingError->setChecked(trackingErrorEnabled);

    // display requested joint positions when we are not trying to set it using GUI
    if (!DirectControl) {
        QVWDesiredPosition->SetValue(PID.m_setpoint_js.Position());
        QVWDesiredEffort->SetValue(PID.m_setpoint_js.Effort());
    }

    // plot
    CurrentPositionSignal->AppendPoint(vctDouble2(PID.m_measured_js.Timestamp(),
                                                  PID.m_measured_js.Position().Element(PlotIndex)));
    DesiredPositionSignal->AppendPoint(vctDouble2(PID.m_setpoint_js.Timestamp(),
                                                  PID.m_setpoint_js.Position().Element(PlotIndex)));
    CurrentVelocitySignal->AppendPoint(vctDouble2(PID.m_measured_js.Timestamp(),
                                                  PID.m_measured_js.Velocity().Element(PlotIndex)));
    // negate effort to plot the same direction
    CurrentEffortSignal->AppendPoint(vctDouble2(PID.m_measured_js.Timestamp(),
                                                -PID.m_measured_js.Effort().Element(PlotIndex)));
    DesiredEffortSignal->AppendPoint(vctDouble2(PID.m_setpoint_js.Timestamp(),
                                                -PID.m_setpoint_js.Effort().Element(PlotIndex)));
    QVPlot->update();
}


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

    QLabel * cLabel = new QLabel("Cutoff");
    cLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(cLabel);
    QVWCutoff = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SPINBOX_WIDGET);
    QVWCutoff->SetStep(0.000001);
    QVWCutoff->SetPrecision(7);
    QVWCutoff->SetRange(-maximum, maximum);
    gridLayout->addWidget(QVWCutoff, row, 1);
    row++;

    // plot
    QHBoxLayout * plotLayout = new QHBoxLayout;
    // plot control
    QVBoxLayout * plotButtonsLayout = new QVBoxLayout;
    // - pick axis to display
    QLabel * plotIndexLabel = new QLabel("Index");
    plotButtonsLayout->addWidget(plotIndexLabel);
    QSBPlotIndex = new QSpinBox();
    QSBPlotIndex->setRange(0, (m_number_of_joints > 0) ? (m_number_of_joints - 1) : 0);
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
    QHBoxLayout * controlLayout = new QHBoxLayout;
    controlLayout->setContentsMargins(1, 1, 1, 1);
    controlLayout->addWidget(QCBEnableDirectControl);
    controlLayout->addWidget(QCBEnable);
    controlLayout->addWidget(QCBEnableTrackingError);
    controlLayout->addWidget(QPBMaintainPosition);
    QFrame * controlFrame = new QFrame();
    controlFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    controlFrame->setLayout(controlLayout);

    connect(QCBEnableDirectControl, SIGNAL(toggled(bool)), this, SLOT(SlotEnableDirectControl(bool)));
    connect(QCBEnable, SIGNAL(clicked(bool)), this, SLOT(SlotEnable(bool)));
    connect(this, SIGNAL(SignalEnable(bool)), this, SLOT(SlotEnableEventHandler(bool)));
    connect(QCBEnableTrackingError, SIGNAL(clicked(bool)), this, SLOT(SlotEnableTrackingError(bool)));
    connect(QPBMaintainPosition, SIGNAL(clicked()), this, SLOT(SlotMaintainPosition()));
    connect(QSBPlotIndex, SIGNAL(valueChanged(int)), this, SLOT(SlotPlotIndex(int)));

    // main layout
    QVBoxLayout * mainLayout = new QVBoxLayout;
    mainLayout->addLayout(gridLayout);
    mainLayout->addLayout(plotLayout);
    mainLayout->addWidget(controlFrame);

    setLayout(mainLayout);

    setWindowTitle(this->GetName().c_str());
    resize(sizeHint());

    // connect signals & slots
    connect(QVWJointsEnabled, SIGNAL(valueChanged()), this, SLOT(SlotEnabledJointsChanged()));
    connect(QVWDesiredPosition, SIGNAL(valueChanged()), this, SLOT(SlotPositionChanged()));
    connect(QVWPGain, SIGNAL(valueChanged()), this, SLOT(SlotConfigurationChanged()));
    connect(QVWDGain, SIGNAL(valueChanged()), this, SLOT(SlotConfigurationChanged()));
    connect(QVWIGain, SIGNAL(valueChanged()), this, SLOT(SlotConfigurationChanged()));
    connect(QVWCutoff, SIGNAL(valueChanged()), this, SLOT(SlotConfigurationChanged()));

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
