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

mtsTeleOperationQtWidget::mtsTeleOperationQtWidget(const std::string & componentName)
    :mtsComponent(componentName)
{
    // Setup CISST Interface
    mtsInterfaceRequired * rinterfaceRequired = AddInterfaceRequired("TeleOperation");
    if (rinterfaceRequired) {
        rinterfaceRequired->AddFunction("GetPositionCartesianMaster", TeleOperation.GetPositionCartesianMaster);
        rinterfaceRequired->AddFunction("GetPositionCartesianSlave", TeleOperation.GetPositionCartesianSlave);
    }
    setupUi();
    startTimer(50); // ms
}

void mtsTeleOperationQtWidget::Configure(const std::string &filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsTeleOperationQtWidget::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsTeleOperationQtWidget::Startup" << std::endl;
    show();
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
    QFRPositionMasterWidget = new vctQtWidgetFrameDoubleRead;
    frameLayout->addWidget(QFRPositionMasterWidget, 0, 0);
    QFRPositionSlaveWidget = new vctQtWidgetFrameDoubleRead;
    frameLayout->addWidget(QFRPositionSlaveWidget, 1, 0);
    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->addLayout(frameLayout);

    setLayout(mainLayout);

    setWindowTitle("TeleOperation Controller");
    resize(sizeHint());
}
