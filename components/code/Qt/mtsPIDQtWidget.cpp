/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-20

  (C) Copyright 2013-2023 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <QFileDialog>

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
    PID.m_error_state.Position().SetSize(m_number_of_joints);
    PID.m_error_state.Velocity().SetSize(m_number_of_joints);
    PID.m_error_state.Effort().SetSize(m_number_of_joints);

    SetpointPosition.SetSize(m_number_of_joints);
    SetpointPosition.SetAll(0.0);
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
        interfaceRequired->AddFunction("use_setpoint_v", PID.UseSetpointV);
        interfaceRequired->AddFunction("use_setpoint_v", PID.UsingSetpointV);
        interfaceRequired->AddFunction("EnableTrackingError", PID.EnableTrackingError);
        interfaceRequired->AddFunction("TrackingErrorEnabled", PID.TrackingErrorEnabled);
        interfaceRequired->AddFunction("enforce_position_limits", PID.enforce_position_limits);
        interfaceRequired->AddFunction("position_limits_enforced", PID.position_limits_enforced);
        interfaceRequired->AddFunction("configuration", PID.configuration);
        interfaceRequired->AddFunction("configuration_js", PID.configuration_js);
        interfaceRequired->AddFunction("configure", PID.configure);
        interfaceRequired->AddFunction("servo_jp", PID.servo_jp);
        interfaceRequired->AddFunction("measured_js", PID.measured_js);
        interfaceRequired->AddFunction("setpoint_js", PID.setpoint_js);
        interfaceRequired->AddFunction("error_state/measured_js", PID.error_state_measured_js);
        // Events
        interfaceRequired->AddEventHandlerWrite(&mtsPIDQtWidget::ErrorEventHandler, this, "error");
        interfaceRequired->AddEventHandlerWrite(&mtsPIDQtWidget::EnableEventHandler, this, "Enabled");
        interfaceRequired->AddEventHandlerWrite(&mtsPIDQtWidget::UseSetpointVEventHandler, this, "use_setpoint_v");
    }
    setupUi();
    startTimer(TimerPeriodInMilliseconds); // ms
}


void mtsPIDQtWidget::GetConfiguration(void)
{
    // get configuration
    PID.configuration(PID.m_configuration);
    // convert to vectors
    vctDoubleVec pg, ig, dg, db, co;
    pg.SetSize(m_number_of_joints);
    ig.SetSize(m_number_of_joints);
    dg.SetSize(m_number_of_joints);
    db.SetSize(m_number_of_joints);
    co.SetSize(m_number_of_joints);
    size_t index = 0;
    for (const auto & v : PID.m_configuration) {
        pg.at(index) = v.p_gain;
        ig.at(index) = v.i_gain;
        dg.at(index) = v.d_gain;
        db.at(index) = v.p_deadband;
        co.at(index) = v.v_low_pass_cutoff;
        ++index;
    }
    // update widgets
    QVWPGain->SetValue(pg);
    QVWIGain->SetValue(ig);
    QVWDGain->SetValue(dg);
    QVWDeadband->SetValue(db);
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

    // get other configuration from PID
    bool flag;
    PID.UsingSetpointV(flag);
    QCBUseSetpointV->setChecked(flag);

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


void mtsPIDQtWidget::SlotEnforcePositionLimits(bool toggle)
{
    PID.enforce_position_limits(toggle);
}


void mtsPIDQtWidget::SlotPositionChanged(void)
{
    SetpointPosition.SetAll(0.0);
    QVWSetpointPosition->GetValue(SetpointPosition);
    SetpointPositionParam.SetGoal(SetpointPosition);
    SetpointPositionParam.Goal().ElementwiseDivide(UnitFactor);
    PID.servo_jp(SetpointPositionParam);
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
    vctDoubleVec db(m_number_of_joints);
    QVWDeadband->GetValue(db);
    vctDoubleVec co(m_number_of_joints);
    QVWCutoff->GetValue(co);
    // copy to configuration
    size_t index = 0;
    for (auto & v : PID.m_configuration) {
        v.p_gain = pg.at(index);
        v.i_gain = ig.at(index);
        v.d_gain = dg.at(index);
        v.p_deadband = db.at(index);
        v.v_low_pass_cutoff = co.at(index);
        ++index;
    }
    // set
    PID.configure(PID.m_configuration);
}


void mtsPIDQtWidget::SlotMaintainPosition(void)
{
    // reset setpoint position
    vctDoubleVec goal(PID.m_measured_js.Position());
    QVWSetpointPosition->SetValue(goal);
    SlotPositionChanged();
}


void mtsPIDQtWidget::SlotSave(void)
{
    QString filename
        = QFileDialog::getSaveFileName(this,
                                       "Save PID configuration",
                                       QDir::currentPath(),
                                       "sawControllersPID JSON (sawControllersPID*.json)");
    if (!filename.isNull()) {
        // save
        std::string message;
        std::string actualFile = filename.toStdString();
        try {
            std::ofstream rawFile;
            rawFile.open(actualFile);
            if (rawFile.is_open()) {
                Json::Value jsonValue;
                cmnDataJSON<mtsPIDConfiguration>::SerializeText(PID.m_configuration, jsonValue);
                rawFile << jsonValue;
                rawFile.close();
                message = "Configuration saved to:\n" + actualFile;
            } else {
                message = "Failed to open:\n" + actualFile;
            }
        } catch (const std::runtime_error & e) {
            message = "Failed to save configuration to:\n" +  actualFile
                + "\nException:\n" + e.what();
        }
        // confirmation message
        QMessageBox * msgBox = new QMessageBox(this);
        msgBox->setAttribute(Qt::WA_DeleteOnClose);
        msgBox->setStandardButtons(QMessageBox::Ok);
        msgBox->setWindowTitle("Information");
        msgBox->setText(message.c_str());
        msgBox->setModal(true);
        msgBox->show();
    }
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
    QVWSetpointPosition->setEnabled(toggle);
    QVWPGain->setEnabled(toggle);
    QVWDGain->setEnabled(toggle);
    QVWIGain->setEnabled(toggle);
    QVWDeadband->setEnabled(toggle);
    QVWCutoff->setEnabled(toggle);
    QCBEnable->setEnabled(toggle);
    QCBEnableTrackingError->setEnabled(toggle);
    QCBEnforcePositionLimits->setEnabled(toggle);
    QCBUseSetpointV->setEnabled(toggle);
    QPBMaintainPosition->setEnabled(toggle);
    QPBSave->setEnabled(toggle);
}


void mtsPIDQtWidget::SlotUseSetpointV(bool use)
{
    PID.UseSetpointV(use);
}


void mtsPIDQtWidget::SlotUseSetpointVEventHandler(bool use)
{
    QCBUseSetpointV->setChecked(use);
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
    bool has_setpoint_v = (PID.m_setpoint_js.Position().size() == PID.m_setpoint_js.Velocity().size());
    if (has_setpoint_v) {
        PID.m_setpoint_js.Velocity().ElementwiseMultiply(UnitFactor);
    }
    PID.error_state_measured_js(PID.m_error_state);
    bool trackingErrorEnabled;
    PID.TrackingErrorEnabled(trackingErrorEnabled);
    bool positionLimitsEnforced;
    PID.position_limits_enforced(positionLimitsEnforced);

    // update GUI
    QVWJointsEnabled->SetValue(JointsEnabled);
    QVRMeasuredPosition->SetValue(PID.m_measured_js.Position());
    QVRMeasuredEffort->SetValue(PID.m_measured_js.Effort());
    QCBEnableTrackingError->setChecked(trackingErrorEnabled);
    QCBEnforcePositionLimits->setChecked(positionLimitsEnforced);

    // display requested joint positions when we are not trying to set it using GUI
    if (!DirectControl) {
        QVWSetpointPosition->SetValue(PID.m_setpoint_js.Position());
        QVWSetpointEffort->SetValue(PID.m_setpoint_js.Effort());
    }

    // plot
    signal_measured_p->AppendPoint(vctDouble2(PID.m_measured_js.Timestamp(),
                                              PID.m_measured_js.Position().Element(PlotIndex)));
    signal_setpoint_p->AppendPoint(vctDouble2(PID.m_setpoint_js.Timestamp(),
                                              PID.m_setpoint_js.Position().Element(PlotIndex)));
    signal_measured_v->AppendPoint(vctDouble2(PID.m_measured_js.Timestamp(),
                                              PID.m_measured_js.Velocity().Element(PlotIndex)));
    if (has_setpoint_v) {
        signal_setpoint_v->AppendPoint(vctDouble2(PID.m_setpoint_js.Timestamp(),
                                                  PID.m_setpoint_js.Velocity().Element(PlotIndex)));
    } else {
        signal_setpoint_v->AppendPoint(vctDouble2(PID.m_setpoint_js.Timestamp(),
                                                  0.0));
    }
    // negate effort to plot the same direction
    signal_measured_f->AppendPoint(vctDouble2(PID.m_measured_js.Timestamp(),
                                              -PID.m_measured_js.Effort().Element(PlotIndex)));
    signal_setpoint_f->AppendPoint(vctDouble2(PID.m_setpoint_js.Timestamp(),
                                              -PID.m_setpoint_js.Effort().Element(PlotIndex)));
    signal_disturbance->AppendPoint(vctDouble2(PID.m_error_state.Timestamp(),
                                               -PID.m_error_state.Position().Element(PlotIndex)));
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

    QLabel * measuredPosLabel = new QLabel("Measured position (deg)");
    measuredPosLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(measuredPosLabel, row, 0);
    QVRMeasuredPosition = new vctQtWidgetDynamicVectorDoubleRead();
    QVRMeasuredPosition->SetPrecision(3);
    gridLayout->addWidget(QVRMeasuredPosition, row, 1);
    row++;

    QLabel * setpointPosLabel = new QLabel("Setpoint position (deg)");
    setpointPosLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(setpointPosLabel, row, 0);
    QVWSetpointPosition = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SPINBOX_WIDGET);
    QVWSetpointPosition->SetStep(0.1);
    QVWSetpointPosition->SetRange(-360.0, 360.0);
    gridLayout->addWidget(QVWSetpointPosition, row, 1);
    row++;

    QLabel * measuredEffortLabel = new QLabel("Measured effort (Nm)");
    measuredEffortLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(measuredEffortLabel, row, 0);
    QVRMeasuredEffort = new vctQtWidgetDynamicVectorDoubleRead();
    QVRMeasuredEffort->SetPrecision(3);
    gridLayout->addWidget(QVRMeasuredEffort, row, 1);
    row++;

    QLabel * setpointEffortLabel = new QLabel("Setpoint effort (Nm)");
    setpointEffortLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(setpointEffortLabel, row, 0);
    QVWSetpointEffort = new vctQtWidgetDynamicVectorDoubleRead();
    QVWSetpointEffort->SetPrecision(3);
    gridLayout->addWidget(QVWSetpointEffort, row, 1);
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

    QLabel * dbLabel = new QLabel("Deadband");
    dbLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(dbLabel);
    QVWDeadband = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SPINBOX_WIDGET);
    QVWDeadband->SetStep(0.001);
    QVWDeadband->SetPrecision(5);
    QVWDeadband->SetRange(-maximum, maximum);
    gridLayout->addWidget(QVWDeadband, row, 1);
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
    // --
    label = new QLabel("Measured position");
    label->setAutoFillBackground(true);
    palette.setColor(QPalette::WindowText, Qt::red);
    label->setPalette(palette);
    plotButtonsLayout->addWidget(label);
    // --
    label = new QLabel("Setpoint position");
    label->setAutoFillBackground(true);
    palette.setColor(QPalette::WindowText, Qt::green);
    label->setPalette(palette);
    plotButtonsLayout->addWidget(label);
    // --
    label = new QLabel("Measured velocity");
    label->setAutoFillBackground(true);
    palette.setColor(QPalette::WindowText, QColor(static_cast<int>(0.7 * 255), 0, 0));
    label->setPalette(palette);
    plotButtonsLayout->addWidget(label);
    label = new QLabel("Setpoint velocity");
    label->setAutoFillBackground(true);
    palette.setColor(QPalette::WindowText, QColor(0, static_cast<int>(0.7 * 255), 0));
    label->setPalette(palette);
    plotButtonsLayout->addWidget(label);
    // --
    label = new QLabel("Measured effort");
    label->setAutoFillBackground(true);
    palette.setColor(QPalette::WindowText, Qt::cyan);
    label->setPalette(palette);
    plotButtonsLayout->addWidget(label);
    // --
    label = new QLabel("Setpoint effort");
    label->setAutoFillBackground(true);
    palette.setColor(QPalette::WindowText, Qt::white);
    label->setPalette(palette);
    plotButtonsLayout->addWidget(label);
    // --
    label = new QLabel("Disturbance");
    label->setAutoFillBackground(true);
    palette.setColor(QPalette::WindowText, Qt::magenta);
    label->setPalette(palette);
    plotButtonsLayout->addWidget(label);
    // --
    plotButtonsLayout->addStretch();
    plotLayout->addLayout(plotButtonsLayout);
    // plotting area
    QVPlot = new vctPlot2DOpenGLQtWidget();

    vctPlot2DBase::Scale * scalePosition = QVPlot->AddScale("positions");
    signal_measured_p = scalePosition->AddSignal("measured");
    signal_measured_p->SetColor(vctDouble3(1.0, 0.0, 0.0));
    signal_setpoint_p = scalePosition->AddSignal("setpoint");
    signal_setpoint_p->SetColor(vctDouble3(0.0, 1.0, 0.0));

    vctPlot2DBase::Scale * scaleVelocity = QVPlot->AddScale("velocities");
    signal_measured_v = scaleVelocity->AddSignal("measured");
    signal_measured_v->SetColor(vctDouble3(0.7, 0.0, 0.0));
    signal_setpoint_v = scaleVelocity->AddSignal("setpoint");
    signal_setpoint_v->SetColor(vctDouble3(0.0, 0.7, 0.0));

    vctPlot2DBase::Scale * scaleEffort = QVPlot->AddScale("efforts");
    signal_measured_f = scaleEffort->AddSignal("-measured");
    signal_measured_f->SetColor(vctDouble3(0.0, 1.0, 1.0));
    signal_setpoint_f = scaleEffort->AddSignal("-setpoint");
    signal_setpoint_f->SetColor(vctDouble3(1.0, 1.0, 1.0));
    signal_disturbance = scaleEffort->AddSignal("-disturbance");
    signal_disturbance->SetColor(vctDouble3(1.0, 0.0, 1.0));
    QVPlot->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));
    plotLayout->addWidget(QVPlot);

    // control
    QCBEnableDirectControl = new QCheckBox("Direct control");
    QCBEnable = new QCheckBox("Enable PID");
    QCBEnableTrackingError = new QCheckBox("Tracking error");
    QCBUseSetpointV = new QCheckBox("Use setpoint_v");
    QCBEnforcePositionLimits = new QCheckBox("Position limits");
    QPBMaintainPosition = new QPushButton("Maintain position");
    QPBSave = new QPushButton("Save configuration");
    QHBoxLayout * controlLayout = new QHBoxLayout;
    controlLayout->setContentsMargins(1, 1, 1, 1);
    controlLayout->addWidget(QCBEnableDirectControl);
    controlLayout->addWidget(QCBEnable);
    controlLayout->addWidget(QCBEnableTrackingError);
    controlLayout->addWidget(QCBEnforcePositionLimits);
    controlLayout->addWidget(QCBUseSetpointV);
    controlLayout->addWidget(QPBMaintainPosition);
    controlLayout->addWidget(QPBSave);
    QFrame * controlFrame = new QFrame();
    controlFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    controlFrame->setLayout(controlLayout);

    connect(QCBEnableDirectControl, SIGNAL(toggled(bool)), this, SLOT(SlotEnableDirectControl(bool)));
    connect(QCBEnable, SIGNAL(clicked(bool)), this, SLOT(SlotEnable(bool)));
    connect(this, SIGNAL(SignalEnable(bool)), this, SLOT(SlotEnableEventHandler(bool)));
    connect(QCBEnableTrackingError, SIGNAL(clicked(bool)), this, SLOT(SlotEnableTrackingError(bool)));
    connect(QCBEnforcePositionLimits, SIGNAL(clicked(bool)), this, SLOT(SlotEnforcePositionLimits(bool)));
    connect(QCBUseSetpointV, SIGNAL(clicked(bool)), this, SLOT(SlotUseSetpointV(bool)));
    connect(this, SIGNAL(SignalUseSetpointV(bool)), this, SLOT(SlotUseSetpointVEventHandler(bool)));
    connect(QPBMaintainPosition, SIGNAL(clicked()), this, SLOT(SlotMaintainPosition()));
    connect(QPBSave, SIGNAL(clicked()), this, SLOT(SlotSave()));
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
    connect(QVWSetpointPosition, SIGNAL(valueChanged()), this, SLOT(SlotPositionChanged()));
    connect(QVWPGain, SIGNAL(valueChanged()), this, SLOT(SlotConfigurationChanged()));
    connect(QVWDGain, SIGNAL(valueChanged()), this, SLOT(SlotConfigurationChanged()));
    connect(QVWIGain, SIGNAL(valueChanged()), this, SLOT(SlotConfigurationChanged()));
    connect(QVWDeadband, SIGNAL(valueChanged()), this, SLOT(SlotConfigurationChanged()));
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

void mtsPIDQtWidget::UseSetpointVEventHandler(const bool & use)
{
    emit SignalUseSetpointV(use);
}
