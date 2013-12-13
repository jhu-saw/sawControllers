/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Zihan Chen
  Created on: 2013-02-20

  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


// system include
#include <iostream>

// Qt include
#include <QString>
#include <QtGui>

// cisst
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionJointSet.h>

#include <sawControllers/mtsTeleOperationQtWidget.h>

#define SWITCH 0

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsTeleOperationQtWidget, mtsComponent, std::string);

mtsTeleOperationQtWidget::mtsTeleOperationQtWidget(const std::string & componentName, double periodInSeconds):
    mtsComponent(componentName),
    TimerPeriodInMilliseconds(periodInSeconds * 1000) // Qt timers are in milliseconds
{
    // Setup CISST Interface
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("TeleOperation");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("Enable", TeleOperation.Enable);
        interfaceRequired->AddFunction("SetScale", TeleOperation.SetScale);
        interfaceRequired->AddFunction("GetPositionCartesianMaster", TeleOperation.GetPositionCartesianMaster);
        interfaceRequired->AddFunction("GetPositionCartesianSlave", TeleOperation.GetPositionCartesianSlave);
        interfaceRequired->AddFunction("GetPeriodStatistics", TeleOperation.GetPeriodStatistics);
    }
    setupUi();
    startTimer(TimerPeriodInMilliseconds);
}

void mtsTeleOperationQtWidget::Configure(const std::string &filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsTeleOperationQtWidget::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsTeleOperationQtWidget::Startup" << std::endl;
    if (!parent()) {
        show();
    }
}

void mtsTeleOperationQtWidget::Cleanup(void)
{
    this->hide();
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsTeleOperationQtWidget::Cleanup" << std::endl;
}

void mtsTeleOperationQtWidget::closeEvent(QCloseEvent * event)
{
    int answer = QMessageBox::warning(this, tr("mtsTeleOperationQtWidget"),
                                      tr("Do you really want to quit this application?"),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::Yes) {
        event->accept();
        QCoreApplication::exit();
    } else {
        event->ignore();
    }
}

void mtsTeleOperationQtWidget::timerEvent(QTimerEvent * event)
{
    // make sure we should update the display
    if (this->isHidden()) {
        return;
    }

    mtsExecutionResult executionResult;
    executionResult = TeleOperation.GetPositionCartesianMaster(PositionMaster);
    if (!executionResult) {
        CMN_LOG_CLASS_RUN_ERROR << "TeleOperation.GetPositionCartesianMaster failed, \""
                                << executionResult << "\"" << std::endl;
    }
    executionResult = TeleOperation.GetPositionCartesianSlave(PositionSlave);
    if (!executionResult) {
        CMN_LOG_CLASS_RUN_ERROR << "TeleOperation.GetPositionCartesianSlave failed, \""
                                << executionResult << "\"" << std::endl;
    }
    QFRPositionMasterWidget->SetValue(PositionMaster.Position());
    QFRPositionSlaveWidget->SetValue(PositionSlave.Position());

    TeleOperation.GetPeriodStatistics(IntervalStatistics);
    QMIntervalStatistics->SetValue(IntervalStatistics);
}


void mtsTeleOperationQtWidget::SlotEnableTeleop(bool state)
{
    TeleOperation.Enable(mtsBool(state));
}

void mtsTeleOperationQtWidget::SlotSetScale(double scale)
{
    TeleOperation.SetScale(scale);
}

void mtsTeleOperationQtWidget::setupUi(void)
{
    QFont font;
    font.setBold(true);
    font.setPointSize(12);

    QGridLayout * cmdTitleLayout = new QGridLayout;
    QSpacerItem * cmdTitleLeftSpacer = new QSpacerItem(341, 20, QSizePolicy::Expanding);
    QSpacerItem * cmdTitleRightSpacer = new QSpacerItem(341, 20, QSizePolicy::Expanding);
    cmdTitleLayout->addItem(cmdTitleLeftSpacer, 0, 0);
    cmdTitleLayout->addItem(cmdTitleRightSpacer, 0, 2);

    QFrame * cmdTitleLeftLine = new QFrame;
    cmdTitleLeftLine->setFrameShape(QFrame::HLine);
    cmdTitleLeftLine->setFrameShadow(QFrame::Sunken);
    QFrame * cmdTitleRightLine = new QFrame;
    cmdTitleRightLine->setFrameShape(QFrame::HLine);
    cmdTitleRightLine->setFrameShadow(QFrame::Sunken);
    QLabel * cmdTitleLabel = new QLabel("TeleOperation Controller");
    cmdTitleLabel->setFont(font);
    cmdTitleLabel->setAlignment(Qt::AlignCenter);

    cmdTitleLayout->addWidget(cmdTitleLeftLine, 1, 0);
    cmdTitleLayout->addWidget(cmdTitleLabel, 1, 1);
    cmdTitleLayout->addWidget(cmdTitleRightLine, 1, 2);

    QGridLayout * frameLayout = new QGridLayout;
    QFRPositionMasterWidget = new vctQtWidgetFrameDoubleRead(vctQtWidgetRotationDoubleRead::OPENGL_WIDGET);
    frameLayout->addWidget(QFRPositionMasterWidget, 0, 0);
    QFRPositionSlaveWidget = new vctQtWidgetFrameDoubleRead(vctQtWidgetRotationDoubleRead::OPENGL_WIDGET);
    frameLayout->addWidget(QFRPositionSlaveWidget, 1, 0);


    QVBoxLayout * controlLayout = new QVBoxLayout;

    QLabel * instructionsLabel = new QLabel("To start tele-operation you must first insert the tool past the cannula tip (push tool clutch button and manually insert tool).\nYou must keep your right foot on the COAG/MONO pedal to operate.\nYou can use the clutch pedal to re-position your masters.");
    controlLayout->addWidget(instructionsLabel);

    // enable/disable teleoperation
    QCheckBox * enableCheckbox = new QCheckBox("Enable");
    controlLayout->addWidget(enableCheckbox);

    // scale
    QDoubleSpinBox * scaleSpinbox = new QDoubleSpinBox();
    scaleSpinbox->setRange(0.1, 0.5);
    scaleSpinbox->setSingleStep(0.1);
    scaleSpinbox->setPrefix("scale ");
    scaleSpinbox->setValue(0.2);
    controlLayout->addWidget(scaleSpinbox);

    // Timing
    QMIntervalStatistics = new mtsQtWidgetIntervalStatistics();
    controlLayout->addWidget(QMIntervalStatistics);

    controlLayout->addStretch();

    QHBoxLayout * mainLayout = new QHBoxLayout;
    mainLayout->addLayout(frameLayout);
    mainLayout->addLayout(controlLayout);

    setLayout(mainLayout);

    setWindowTitle("TeleOperation Controller");
    resize(sizeHint());

    // setup Qt Connection
    connect(enableCheckbox, SIGNAL(clicked(bool)), this, SLOT(SlotEnableTeleop(bool)));
    connect(scaleSpinbox, SIGNAL(valueChanged(double)), this, SLOT(SlotSetScale(double)));
}
