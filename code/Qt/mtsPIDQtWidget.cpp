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


// system include
#include <iostream>

// Qt include
#include <QString>
#include <QtGui>

// cisst
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmJointType.h>
#include <cisstCommon/cmnConstants.h>
#include <cisstCommon/cmnUnits.h>

#include <sawControllers/mtsPIDQtWidget.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsPIDQtWidget, mtsComponent, mtsComponentConstructorNameAndUInt)

mtsPIDQtWidget::mtsPIDQtWidget(const std::string &taskName,
                               unsigned int numberOfAxis)
    : mtsComponent(taskName), NumberOfAxis(numberOfAxis)
{
    Init();
}

mtsPIDQtWidget::mtsPIDQtWidget(const mtsComponentConstructorNameAndUInt &arg)
    : mtsComponent(arg.Name), NumberOfAxis(arg.Arg)
{
    Init();
}

void mtsPIDQtWidget::Init(void)
{
    DesiredPosition.SetSize(NumberOfAxis);
    DesiredPosition.SetAll(0.0);
    UnitFactor.SetSize(NumberOfAxis);
    UnitFactor.SetAll(1.0);

    // Setup cisst interface
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("Controller");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("ResetController", PID.ResetController);
        interfaceRequired->AddFunction("Enable", PID.Enable);
        interfaceRequired->AddFunction("SetPositionJoint", PID.SetPositionJoint);
        interfaceRequired->AddFunction("GetPositionJoint", PID.GetPositionJoint);
        interfaceRequired->AddFunction("GetJointType", PID.GetJointType);
        interfaceRequired->AddFunction("GetPGain", PID.GetPGain);
        interfaceRequired->AddFunction("GetDGain", PID.GetDGain);
        interfaceRequired->AddFunction("GetIGain", PID.GetIGain);
        interfaceRequired->AddFunction("SetPGain", PID.SetPGain);
        interfaceRequired->AddFunction("SetDGain", PID.SetDGain);
        interfaceRequired->AddFunction("SetIGain", PID.SetIGain);
        // Events
        interfaceRequired->AddEventHandlerVoid(&mtsPIDQtWidget::EventErrorLimitHandler, this, "EventErrorLimit");
    }
    setupUi();
    startTimer(50); // ms
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
                UnitFactor[i] = cmn_mm;
            } else {
                cmnThrow("mtsRobotIO1394QtWidget: Unknown joint type");
            }
        }
    }

    // Show the GUI
    show();
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

//----------- Private Slot ------------------------------

void mtsPIDQtWidget::SlotEnablePID(bool toggle)
{
    PID.Enable(toggle);
}

void mtsPIDQtWidget::SlotPositionChanged(void)
{
    DesiredPosition.SetAll(0.0);
    QVWDesiredPositionWidget->GetValue(DesiredPosition);
    DesiredPosition.ElementwiseDivide(UnitFactor);
    prmPositionJointSet prmDesiredPos;
    prmDesiredPos.SetGoal(DesiredPosition);
    PID.SetPositionJoint(prmDesiredPos);
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
    prmPositionJointGet prmFeedbackPos;
    prmFeedbackPos.SetSize(NumberOfAxis);
    PID.GetPositionJoint(prmFeedbackPos);
    prmFeedbackPos.Position().ElementwiseMultiply(UnitFactor);
    QVWDesiredPositionWidget->SetValue(prmFeedbackPos.Position());
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

void mtsPIDQtWidget::timerEvent(QTimerEvent * event)
{
    prmPositionJointGet prmFeedbackPos;
    prmFeedbackPos.SetSize(NumberOfAxis);
    PID.GetPositionJoint(prmFeedbackPos);
    prmFeedbackPos.Position().ElementwiseMultiply(UnitFactor);
    QVRCurrentPositionWidget->SetValue(prmFeedbackPos.Position());
}

////------------ Private Methods ----------------
void mtsPIDQtWidget::setupUi(void)
{
    QFont font;
    font.setBold(true);
    font.setPointSize(12);

    QGridLayout * gridLayout = new QGridLayout();

    int row = 0;
    QLabel * currentPosLabel = new QLabel("Current position (deg)");
    currentPosLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(currentPosLabel, row, 0);
    QVRCurrentPositionWidget = new vctQtWidgetDynamicVectorDoubleRead();
    gridLayout->addWidget(QVRCurrentPositionWidget, row, 1);
    row++;

    QLabel * desiredPosLabel = new QLabel("Desired position (deg)");
    desiredPosLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(desiredPosLabel, row, 0);
    QVWDesiredPositionWidget = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SPINBOX_WIDGET);
    // DesiredPositionWidget->SetDecimals(2);
    QVWDesiredPositionWidget->SetStep(0.1);
    QVWDesiredPositionWidget->SetRange(-360.0, 360.0);
    gridLayout->addWidget(QVWDesiredPositionWidget, row, 1);
    row++;

    QLabel * pLabel = new QLabel("PGain");
    pLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(pLabel);
    QVWPGainWidget = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SPINBOX_WIDGET);
    QVWPGainWidget->SetStep(0.01);
    QVWPGainWidget->SetPrecision(3);
    QVWPGainWidget->SetRange(-1000.0, 1000.0);
    gridLayout->addWidget(QVWPGainWidget, row, 1);
    row++;

    QLabel * dLabel = new QLabel("DGain");
    dLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(dLabel);
    QVWDGainWidget = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SPINBOX_WIDGET);
    QVWDGainWidget->SetStep(0.01);
    QVWDGainWidget->SetPrecision(3);
    QVWDGainWidget->SetRange(-1000.0, 1000.0);
    gridLayout->addWidget(QVWDGainWidget, row, 1);
    row++;

    QLabel * iLabel = new QLabel("IGain");
    iLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(iLabel);
    QVWIGainWidget = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SPINBOX_WIDGET);
    QVWIGainWidget->SetStep(0.001);
    QVWIGainWidget->SetPrecision(5);
    QVWIGainWidget->SetRange(-1000.0, 1000.0);
    gridLayout->addWidget(QVWIGainWidget, row, 1);
    row++;

    //------------ Test --------------------
    QCBEnablePID = new QCheckBox("Enable PID");
    QPushButton * qpbMaintainPosition = new QPushButton("Maintain position");
    QPushButton * qpbZeroPosition = new QPushButton("Zero position");
    QPushButton * qpbResetPIDGain = new QPushButton("Reset PID gains");
    QHBoxLayout * testLayout = new QHBoxLayout;
    testLayout->addWidget(QCBEnablePID);
    testLayout->addWidget(qpbMaintainPosition);
    testLayout->addWidget(qpbZeroPosition);
    testLayout->addWidget(qpbResetPIDGain);
    testLayout->addStretch();
    QGroupBox * testGroupBox = new QGroupBox("Control");
    testGroupBox->setLayout(testLayout);

    connect(QCBEnablePID, SIGNAL(toggled(bool)), this, SLOT(SlotEnablePID(bool)));
    connect(qpbMaintainPosition, SIGNAL(clicked()), this, SLOT(SlotMaintainPosition()));
    connect(qpbZeroPosition, SIGNAL(clicked()), this, SLOT(SlotZeroPosition()));
    connect(qpbResetPIDGain, SIGNAL(clicked()), this, SLOT(SlotResetPIDGain()));

    //------------ main layout -------------
    QVBoxLayout * mainLayout = new QVBoxLayout;
    mainLayout->addLayout(gridLayout);
    mainLayout->addWidget(testGroupBox);

    setLayout(mainLayout);

    setWindowTitle(this->GetName().c_str());
    setMinimumWidth(750);
    resize(sizeHint());

    // connect signals & slots
    connect(QVWDesiredPositionWidget, SIGNAL(valueChanged()), this, SLOT(SlotPositionChanged()));
    connect(QVWPGainWidget, SIGNAL(valueChanged()), this, SLOT(SlotPGainChanged()));
    connect(QVWDGainWidget, SIGNAL(valueChanged()), this, SLOT(SlotDGainChanged()));
    connect(QVWIGainWidget, SIGNAL(valueChanged()), this, SLOT(SlotIGainChanged()));
}

void mtsPIDQtWidget::EventErrorLimitHandler(void)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "EventErrorLimitHandler" << std::endl;
}
