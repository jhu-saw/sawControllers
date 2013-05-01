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
#include <cisstCommon/cmnConstants.h>

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
    lastEnableState.SetSize(NumberOfAxis);
    lastEnableState.SetAll(false);
    analogIn.SetSize(NumberOfAxis);
    motorFeedbackCurrent.SetSize(NumberOfAxis);

    // ZC
    desiredPos.SetSize(NumberOfAxis);
    desiredPos.SetAll(0.0);

    // Setup CISST Interface
    mtsInterfaceRequired *req = AddInterfaceRequired("Controller");
    if (req) {
        req->AddFunction("ResetController", PID.ResetController);
        req->AddFunction("Enable", PID.Enable);
        req->AddFunction("SetPositionJoint", PID.SetPositionJoint);
        req->AddFunction("GetPositionJoint", PID.GetPositionJoint);
        req->AddFunction("GetPGain", PID.GetPGain);
        req->AddFunction("GetDGain", PID.GetDGain);
        req->AddFunction("GetIGain", PID.GetIGain);
        req->AddFunction("SetPGain", PID.SetPGain);
        req->AddFunction("SetDGain", PID.SetDGain);
        req->AddFunction("SetIGain", PID.SetIGain);
        // Events
        req->AddEventHandlerVoid(&mtsPIDQtWidget::EventErrorLimitHandler, this, "EventErrorLimit");
    }
    setupUi();
    startTimer(50); // ms
}

void mtsPIDQtWidget::Configure(const std::string &filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsPIDQtWidget::Startup()
{
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsPIDQtWidget::Startup" << std::endl;
    // Set desired pos to cur pos
    slot_ResetPIDGain();
    slot_MaintainPosition();
    // Show the GUI
    show();
}

void mtsPIDQtWidget::Cleanup()
{
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsPIDQtWidget::Cleanup" << std::endl;
}

//---------- Protected --------------------------
void mtsPIDQtWidget::closeEvent(QCloseEvent * event)
{
//    int ret = QMessageBox::warning(this, tr("ExitBox"),
//                                 tr("Please power off the robot before quit. \n"
//                                    "Continue?"),
//                                 QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
//    if(ret == QMessageBox::Yes){
////        Robot.DisablePower();

//        event->accept();
//    }else {
//        event->ignore();
//    }
    event->accept();
}

//----------- Private Slot ------------------------------

void mtsPIDQtWidget::slot_qcbEnablePID(bool toggle)
{
    PID.Enable(toggle);
}

void mtsPIDQtWidget::slot_PositionChanged(void)
{
    desiredPos.SetAll(0.0);
    DesiredPositionWidget->GetValue(desiredPos);
    desiredPos.Multiply(cmnPI_180); // all UI is in degrees, all internals are in radians
    prmPositionJointSet prmDesiredPos;
    prmDesiredPos.SetGoal(desiredPos);
    PID.SetPositionJoint(prmDesiredPos);
}

void mtsPIDQtWidget::slot_PGainChanged(void)
{
    vctDoubleVec pgain(NumberOfAxis, 0.0);
    PGainWidget->GetValue(pgain);
    PID.SetPGain(pgain);
}

void mtsPIDQtWidget::slot_DGainChanged(void)
{
    vctDoubleVec dgain(NumberOfAxis, 0.0);
    DGainWidget->GetValue(dgain);
    PID.SetDGain(dgain);
}

void mtsPIDQtWidget::slot_IGainChanged(void)
{
    vctDoubleVec igain(NumberOfAxis, 0.0);
    IGainWidget->GetValue(igain);
    PID.SetIGain(igain);
}

void mtsPIDQtWidget::slot_MaintainPosition(void)
{
    // reset desired position
    prmPositionJointGet prmFeedbackPos;
    prmFeedbackPos.SetSize(NumberOfAxis);
    PID.GetPositionJoint(prmFeedbackPos);
    prmFeedbackPos.Position().Multiply(cmn180_PI);
    DesiredPositionWidget->SetValue(prmFeedbackPos.Position());
    PID.ResetController();
    slot_PositionChanged();
}

void mtsPIDQtWidget::slot_ZeroPosition(void)
{
    // reset desired position
    desiredPos.SetAll(0.0);
    DesiredPositionWidget->SetValue(desiredPos);
    PID.ResetController();
    slot_PositionChanged();
}

void mtsPIDQtWidget::slot_ResetPIDGain(void)
{
    // get gains
    vctDoubleVec gain;
    gain.SetSize(NumberOfAxis);
    // PGain
    PID.GetPGain(gain);
    PGainWidget->SetValue(gain);
    // DGain
    PID.GetDGain(gain);
    DGainWidget->SetValue(gain);
    // IGain
    PID.GetIGain(gain);
    IGainWidget->SetValue(gain);
}

void mtsPIDQtWidget::timerEvent(QTimerEvent * event)
{
    prmPositionJointGet prmFeedbackPos;
    prmFeedbackPos.SetSize(NumberOfAxis);
    PID.GetPositionJoint(prmFeedbackPos);
    prmFeedbackPos.Position().Multiply(cmn180_PI);
    CurrentPositionWidget->SetValue(prmFeedbackPos.Position());
}

////------------ Private Methods ----------------
void mtsPIDQtWidget::setupUi()
{
    QFont font;
    font.setBold(true);
    font.setPointSize(12);

    QGridLayout * gridLayout = new QGridLayout();

    int row = 0;
    QLabel * currentPosLabel = new QLabel("Current position (deg)");
    currentPosLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(currentPosLabel, row, 0);
    CurrentPositionWidget = new vctQtWidgetDynamicVectorDoubleRead();
    gridLayout->addWidget(CurrentPositionWidget, row, 1);
    row++;

    QLabel * desiredPosLabel = new QLabel("Desired position (deg)");
    desiredPosLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(desiredPosLabel, row, 0);
    DesiredPositionWidget = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SPINBOX_WIDGET);
    // DesiredPositionWidget->SetDecimals(2);
    DesiredPositionWidget->SetStep(0.1);
    DesiredPositionWidget->SetRange(-360.0, 360.0);
    gridLayout->addWidget(DesiredPositionWidget, row, 1);
    row++;

    QLabel* pLabel = new QLabel("PGain");
    pLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(pLabel);
    PGainWidget = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SPINBOX_WIDGET);
    PGainWidget->SetStep(0.01);
    PGainWidget->SetRange(-1000.0, 1000.0);
    gridLayout->addWidget(PGainWidget, row, 1);
    row++;

    QLabel* dLabel = new QLabel("DGain");
    dLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(dLabel);
    DGainWidget = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SPINBOX_WIDGET);
    DGainWidget->SetStep(0.01);
    DGainWidget->SetRange(-1000.0, 1000.0);
    gridLayout->addWidget(DGainWidget, row, 1);
    row++;

    QLabel* iLabel = new QLabel("IGain");
    iLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(iLabel);
    IGainWidget = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SPINBOX_WIDGET);
    IGainWidget->SetStep(0.01);
    IGainWidget->SetRange(-1000.0, 1000.0);
    gridLayout->addWidget(IGainWidget, row, 1);
    row++;

    //------------ Test --------------------
    qcbEnablePID = new QCheckBox("Enable PID");
    QPushButton * qpbMaintainPosition = new QPushButton("Maintain position");
    QPushButton * qpbZeroPosition = new QPushButton("Zero position");
    QPushButton * qpbResetPIDGain = new QPushButton("Reset PID gains");
    QHBoxLayout * testLayout = new QHBoxLayout;
    testLayout->addWidget(qcbEnablePID);
    testLayout->addWidget(qpbMaintainPosition);
    testLayout->addWidget(qpbZeroPosition);
    testLayout->addWidget(qpbResetPIDGain);
    testLayout->addStretch();
    QGroupBox * testGroupBox = new QGroupBox("Control");
    testGroupBox->setLayout(testLayout);

    connect(qcbEnablePID, SIGNAL(toggled(bool)), this, SLOT(slot_qcbEnablePID(bool)));
    connect(qpbMaintainPosition, SIGNAL(clicked()), this, SLOT(slot_MaintainPosition()));
    connect(qpbZeroPosition, SIGNAL(clicked()), this, SLOT(slot_ZeroPosition()));
    connect(qpbResetPIDGain, SIGNAL(clicked()), this, SLOT(slot_ResetPIDGain()));

    //------------ main layout -------------
    QVBoxLayout * mainLayout = new QVBoxLayout;
    mainLayout->addLayout(gridLayout);
    mainLayout->addWidget(testGroupBox);

    setLayout(mainLayout);

    setWindowTitle(this->GetName().c_str());
    resize(sizeHint());

    // connect signals & slots
    connect(DesiredPositionWidget, SIGNAL(valueChanged()), this, SLOT(slot_PositionChanged()));
    connect(PGainWidget, SIGNAL(valueChanged()), this, SLOT(slot_PGainChanged()));
    connect(DGainWidget, SIGNAL(valueChanged()), this, SLOT(slot_DGainChanged()));
    connect(IGainWidget, SIGNAL(valueChanged()), this, SLOT(slot_IGainChanged()));
}

void mtsPIDQtWidget::EventErrorLimitHandler(void)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "EventErrorLimitHandler" << std::endl;
}
