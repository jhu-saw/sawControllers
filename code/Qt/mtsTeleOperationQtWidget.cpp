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

mtsTeleOperationQtWidget::mtsTeleOperationQtWidget(const std::string & taskName)
    :mtsComponent(taskName)
{
    // Setup CISST Interface
    mtsInterfaceRequired *req = AddInterfaceRequired("TeleOperation");
    if (req) {
        req->AddFunction("GetPositionCartesianMaster", TeleOperation.GetPositionCartesianMaster);
    }
    setupUi();
    startTimer(50); // ms
}

void mtsTeleOperationQtWidget::Configure(const std::string &filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsTeleOperationQtWidget::Startup()
{
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsTeleOperationQtWidget::Startup" << std::endl;
    show();
}

void mtsTeleOperationQtWidget::Cleanup()
{
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsTeleOperationQtWidget::Cleanup" << std::endl;
}

//---------- Protected --------------------------
void mtsTeleOperationQtWidget::closeEvent(QCloseEvent * event)
{
    event->accept();
}

//----------- Private Slot ------------------------------

void mtsTeleOperationQtWidget::slot_qcbClutch(bool toggle)
{
    // TeleOperation.Enable(toggle);
}

void mtsTeleOperationQtWidget::timerEvent(QTimerEvent *event)
{
    mtsExecutionResult executionResult;
    executionResult = TeleOperation.GetPositionCartesianMaster(PositionMaster);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "TeleOperation.GetPositionCartesianMaster failed, \""
                                << executionResult << "\"" << std::endl;
    }
    PositionMasterWidget->SetValue(PositionMaster.Position());
}

////------------ Private Methods ----------------
void mtsTeleOperationQtWidget::setupUi()
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
    QLabel* cmdTitleLabel = new QLabel("TeleOperation Controller");
    cmdTitleLabel->setFont(font);
    cmdTitleLabel->setAlignment(Qt::AlignCenter);

    cmdTitleLayout->addWidget(cmdTitleLeftLine, 1, 0);
    cmdTitleLayout->addWidget(cmdTitleLabel, 1, 1);
    cmdTitleLayout->addWidget(cmdTitleRightLine, 1, 2);

    QGridLayout * frameLayout = new QGridLayout;
    PositionMasterWidget = new vctQtWidgetFrameDoubleRead;
    frameLayout->addWidget(PositionMasterWidget, 0, 0);

    //------------ main layout -------------
    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->addLayout(frameLayout);

    setLayout(mainLayout);

    setWindowTitle("TeleOperation Controller");
    resize(sizeHint());

    //connect(qcbEnableTeleOperation, SIGNAL(toggled(bool)), this, SLOT(slot_qcbEnableTeleOperation(bool)));
}
