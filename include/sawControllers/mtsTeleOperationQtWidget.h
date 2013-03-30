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


#ifndef _mtsTeleOperationQtWidget_h
#define _mtsTeleOperationQtWidget_h

#include <cisstCommon/cmnXMLPath.h>
#include <cisstOSAbstraction/osaTimeServer.h>
#include <cisstMultiTask/mtsComponent.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>

#include <cisstVector/vctQtWidgetFrame4x4.h>

#include <QtCore>
#include <QtGui>


class mtsTeleOperationQtWidget : public QWidget, public mtsComponent
{
    Q_OBJECT
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsTeleOperationQtWidget(const std::string& taskName);
    ~mtsTeleOperationQtWidget(){}

    void Configure(const std::string &filename = "");
    void Startup();
    void Cleanup();

protected:
    virtual void closeEvent(QCloseEvent *event);

private slots:
    //! qslot to clutch
    void slot_qcbClutch(bool toggle);

    void timerEvent(QTimerEvent * event);

private:
    //! setup TeleOperation controller GUI
    void setupUi();

protected:

    struct TeleOperationStruct {
        mtsFunctionRead GetPositionCartesianMaster;
    } TeleOperation;

private:

    prmPositionCartesianGet PositionMaster;
    vctQtWidgetFrame4x4DoubleRead * PositionMasterWidget;

    // Control
    QPushButton* quitButton;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsTeleOperationQtWidget);

#endif // _mtsTeleOperationQtWidget_h
