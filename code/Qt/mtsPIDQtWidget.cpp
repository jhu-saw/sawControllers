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

#define SWITCH 0

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsPIDQtWidget, mtsComponent, std::string);

mtsPIDQtWidget::mtsPIDQtWidget(const std::string & taskName)
    :mtsComponent(taskName)
{
    numOfAxis = 8;
    tmpStatic = 0;
    lastEnableState.SetSize(numOfAxis);
    lastEnableState.SetAll(false);
    analogIn.SetSize(numOfAxis);
    motorFeedbackCurrent.SetSize(numOfAxis);

    // ZC
    desiredPos.SetSize(numOfAxis);
    desiredPos.SetAll(0.0);

    // Setup CISST Interface
    mtsInterfaceRequired *req = AddInterfaceRequired("Controller");
    if (req) {
        req->AddFunction("ResetController", PID.ResetController);
        req->AddFunction("Enable", PID.Enable);
        req->AddFunction("SetDesiredPositions", PID.SetDesiredPositions);
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
}

void mtsPIDQtWidget::Configure(const std::string &filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsPIDQtWidget::Startup()
{
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsPIDQtWidget::Startup" << std::endl;
    // Set desired pos to cur pos
    slot_qpbResetPIDGain();
    slot_qpbResetDesiredPosition();
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

void mtsPIDQtWidget::slot_qdsbPosition(double position)
{
    desiredPos.SetAll(0.0);
    for (size_t i = 0; i < numOfAxis; i++) {
        desiredPos.at(i) = qdsbPosition[i]->value();
    }
    desiredPos.Multiply(cmnPI_180); // all UI is in degrees, all internals are in radians
    prmPositionJointSet prmDesiredPos;
    prmDesiredPos.SetGoal(desiredPos);
    PID.SetDesiredPositions(prmDesiredPos);
}

void mtsPIDQtWidget::slot_qdsbPGain(double val)
{
    vctDoubleVec pgain;
    pgain.SetSize(numOfAxis);
    pgain.SetAll(0.0);
    for (size_t i = 0; i < numOfAxis; i++) {
        pgain[i] = qdsbPGain[i]->value();
    }
    PID.SetPGain(pgain);
}

void mtsPIDQtWidget::slot_qdsbDGain(double val)
{
    vctDoubleVec dgain;
    dgain.SetSize(numOfAxis);
    dgain.SetAll(0.0);
    for (size_t i = 0; i < numOfAxis; i++) {
        dgain[i] = qdsbDGain[i]->value();
    }
    PID.SetDGain(dgain);
}

void mtsPIDQtWidget::slot_qdsbIGain(double val)
{
    vctDoubleVec igain;
    igain.SetSize(numOfAxis);
    igain.SetAll(0.0);
    for (size_t i = 0; i < numOfAxis; i++) {
        igain[i] = qdsbIGain[i]->value();
    }
    PID.SetIGain(igain);
    CMN_LOG_CLASS_RUN_DEBUG << igain << std::endl;
}

void mtsPIDQtWidget::slot_qpbResetDesiredPosition()
{
    // reset desired position
    prmPositionJointGet prmFeedbackPos;
    prmFeedbackPos.SetSize(numOfAxis);
    PID.GetPositionJoint(prmFeedbackPos);
    PID.GetPositionJoint(prmFeedbackPos);
    size_t i;
    for(i = 0; i < numOfAxis; i++){
        qdsbPosition[i]->blockSignals(true);
        qdsbPosition[i]->setValue(prmFeedbackPos.Position().at(i) * cmn180_PI);
        qdsbPosition[i]->blockSignals(false);
    }
    PID.ResetController();
}

void mtsPIDQtWidget::slot_qpbResetPIDGain()
{
    // get gains
    vctDoubleVec gain;
    gain.SetSize(numOfAxis);
    size_t i;
    // PGain
    PID.GetPGain(gain);
    for(i = 0; i < numOfAxis; i++){
        qdsbPGain[i]->blockSignals(true);
        qdsbPGain[i]->setValue(gain[i]);
        qdsbPGain[i]->blockSignals(false);
    }
    // DGain
    PID.GetDGain(gain);
    for(i = 0; i < numOfAxis; i++){
        qdsbDGain[i]->blockSignals(true);
        qdsbDGain[i]->setValue(gain[i]);
        qdsbDGain[i]->blockSignals(false);
    }
    // IGain
    PID.GetIGain(gain);
    for(i = 0; i < numOfAxis; i++){
        qdsbIGain[i]->blockSignals(true);
        qdsbIGain[i]->setValue(gain[i]);
        qdsbIGain[i]->blockSignals(false);
    }
}

////------------ Private Methods ----------------
void mtsPIDQtWidget::setupUi()
{
    QFont font;
    font.setBold(true);
    font.setPointSize(12);

    //----------------- Command -------------------------------
    // Commands Title
    // spacer          spacer
    // -----  Commands ------
    QGridLayout* cmdTitleLayout = new QGridLayout;
    QSpacerItem* cmdTitleLeftSpacer = new QSpacerItem(341, 20, QSizePolicy::Expanding);
    QSpacerItem* cmdTitleRightSpacer = new QSpacerItem(341, 20, QSizePolicy::Expanding);
    cmdTitleLayout->addItem(cmdTitleLeftSpacer, 0, 0);
    cmdTitleLayout->addItem(cmdTitleRightSpacer, 0, 2);

    QFrame* cmdTitleLeftLine = new QFrame;
    cmdTitleLeftLine->setFrameShape(QFrame::HLine);
    cmdTitleLeftLine->setFrameShadow(QFrame::Sunken);
    QFrame* cmdTitleRightLine = new QFrame;
    cmdTitleRightLine->setFrameShape(QFrame::HLine);
    cmdTitleRightLine->setFrameShadow(QFrame::Sunken);
    QLabel* cmdTitleLabel = new QLabel("PID Controller");
    cmdTitleLabel->setFont(font);
    cmdTitleLabel->setAlignment(Qt::AlignCenter);

    cmdTitleLayout->addWidget(cmdTitleLeftLine, 1, 0);
    cmdTitleLayout->addWidget(cmdTitleLabel, 1, 1);
    cmdTitleLayout->addWidget(cmdTitleRightLine, 1, 2);

    // Commands Label
    // [] Enable PID
    // Desired Positions
    //    QLabel
    QVBoxLayout* cmdLabelLayout = new QVBoxLayout;
    QFrame* cmdLabelFrame = new QFrame;
    qcbEnablePID = new QCheckBox("Enable");
    QLabel* desiredPosLabel = new QLabel("Position");
    desiredPosLabel->setAlignment(Qt::AlignRight);
    QLabel* pLabel = new QLabel("PGain");
    pLabel->setAlignment(Qt::AlignRight);
    QLabel* dLabel = new QLabel("DGain");
    dLabel->setAlignment(Qt::AlignRight);
    QLabel* iLabel = new QLabel("IGain");
    iLabel->setAlignment(Qt::AlignRight);

    cmdLabelLayout->addWidget(qcbEnablePID);
    cmdLabelLayout->addWidget(desiredPosLabel);
    cmdLabelLayout->addWidget(pLabel);
    cmdLabelLayout->addWidget(dLabel);
    cmdLabelLayout->addWidget(iLabel);
    cmdLabelFrame->setLayout(cmdLabelLayout);
    cmdLabelFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);

    // Commands Info
    // [] Enable Axis i
    //   xx.xx mA  double spin box
    //   --|--     slider
    QVBoxLayout** cmdInfoLayout = new QVBoxLayout*[numOfAxis];
    QFrame** cmdInfoFrame = new QFrame*[numOfAxis];
//    qcbEnable = new QCheckBox*[numOfAxis];
    qdsbPosition = new QDoubleSpinBox*[numOfAxis];
    qdsbPGain = new QDoubleSpinBox*[numOfAxis];
    qdsbDGain = new QDoubleSpinBox*[numOfAxis];
    qdsbIGain = new QDoubleSpinBox*[numOfAxis];

    for (int i = 0; i < numOfAxis; i++) {
        QLabel *blank = new QLabel("Joint " + QString::number(i+1));
        blank->setAlignment(Qt::AlignCenter);
        qdsbPosition[i] = new QDoubleSpinBox;
        qdsbPosition[i]->setSuffix(" deg");
        qdsbPosition[i]->setDecimals(2);
        qdsbPosition[i]->setSingleStep(0.1);
        qdsbPosition[i]->setMinimum(-360);
        qdsbPosition[i]->setMaximum(360);
        qdsbPosition[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

        double range = 1000.0;
        qdsbPGain[i] = new QDoubleSpinBox;
        qdsbPGain[i]->setDecimals(3);
        qdsbPGain[i]->setSingleStep(0.001);
        qdsbPGain[i]->setMinimum(-range);
        qdsbPGain[i]->setMaximum(range);
        qdsbPGain[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

        qdsbDGain[i] = new QDoubleSpinBox;
        qdsbDGain[i]->setDecimals(3);
        qdsbDGain[i]->setSingleStep(0.001);
        qdsbDGain[i]->setMinimum(-range);
        qdsbDGain[i]->setMaximum(range);
        qdsbDGain[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

        range = 100.0;
        qdsbIGain[i] = new QDoubleSpinBox;
        qdsbIGain[i]->setDecimals(3);
        qdsbIGain[i]->setSingleStep(0.001);
        qdsbIGain[i]->setMinimum(-range);
        qdsbIGain[i]->setMaximum(range);
        qdsbIGain[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

        cmdInfoLayout[i] = new QVBoxLayout;
        cmdInfoLayout[i]->addWidget(blank);
        cmdInfoLayout[i]->addWidget(qdsbPosition[i]);
        cmdInfoLayout[i]->addWidget(qdsbPGain[i]);
        cmdInfoLayout[i]->addWidget(qdsbDGain[i]);
        cmdInfoLayout[i]->addWidget(qdsbIGain[i]);

        cmdInfoFrame[i] = new QFrame;
        cmdInfoFrame[i]->setLayout(cmdInfoLayout[i]);
        cmdInfoFrame[i]->setFrameShape(QFrame::StyledPanel);
        cmdInfoFrame[i]->setFrameShadow(QFrame::Sunken);
    }

    // Commands lower layout
    // cmdLabel | cmdInfo1 | cmdInfo2 |...
    QHBoxLayout* cmdLowerLayout = new QHBoxLayout;
    cmdLowerLayout->addWidget(cmdLabelFrame);
    for (int i = 0; i < numOfAxis; i++) {
        cmdLowerLayout->addWidget(cmdInfoFrame[i]);
    }

    // Commands layout
    // cmdTitleLayout
    // cmdLowerLayout
    QVBoxLayout* cmdLayout = new QVBoxLayout;
    cmdLayout->addLayout(cmdTitleLayout);
    cmdLayout->addLayout(cmdLowerLayout);

    //------------ Test --------------------
    QPushButton* qpbResetDesiredPosition = new QPushButton("Reset Position");
    QPushButton* qpbResetPIDGain = new QPushButton("Reset PIDGain");
    QHBoxLayout* testLayout = new QHBoxLayout;
    testLayout->addWidget(qpbResetDesiredPosition);
    testLayout->addWidget(qpbResetPIDGain);
    testLayout->addStretch();
    QGroupBox* testGroupBox = new QGroupBox("Function Test");
    testGroupBox->setLayout(testLayout);

    connect(qpbResetDesiredPosition, SIGNAL(clicked()), this, SLOT(slot_qpbResetDesiredPosition()));
    connect(qpbResetPIDGain, SIGNAL(clicked()), this, SLOT(slot_qpbResetPIDGain()));

    //------------ main layout -------------
    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->addLayout(cmdLayout);
    mainLayout->addWidget(testGroupBox);

    setLayout(mainLayout);

////    setFixedWidth(750);
////    setFixedHeight(sizeHint().height());

    setWindowTitle("PID Controller");
    resize(sizeHint());

    // connect signals & slots
    // Commands
    connect(qcbEnablePID, SIGNAL(toggled(bool)), this, SLOT(slot_qcbEnablePID(bool)));
    for (int i = 0; i < numOfAxis; i++) {
        connect(qdsbPosition[i], SIGNAL(valueChanged(double)),
                this, SLOT(slot_qdsbPosition(double)));
        connect(qdsbPGain[i], SIGNAL(valueChanged(double)),
                this, SLOT(slot_qdsbPGain(double)));
        connect(qdsbDGain[i], SIGNAL(valueChanged(double)),
                this, SLOT(slot_qdsbDGain(double)));
        connect(qdsbIGain[i], SIGNAL(valueChanged(double)),
                this, SLOT(slot_qdsbIGain(double)));
    }

}

void mtsPIDQtWidget::EventErrorLimitHandler()
{
    CMN_LOG_CLASS_RUN_VERBOSE << "EventErrorLimitHandler" << std::endl;
}
