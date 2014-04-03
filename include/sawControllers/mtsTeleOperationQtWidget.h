/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*

  Author(s):  Zihan Chen, Anton Deguet
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

#include <cisstVector/vctQtWidgetFrame.h>
#include <cisstMultiTask/mtsComponent.h>
#include <cisstMultiTask/mtsQtWidgetIntervalStatistics.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>

#include <QtCore>
#include <QtGui>


class mtsTeleOperationQtWidget: public QWidget, public mtsComponent
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsTeleOperationQtWidget(const std::string & componentName, double periodInSeconds = 50.0 * cmn_ms);
    ~mtsTeleOperationQtWidget(){}

    void Configure(const std::string & filename = "");
    void Startup(void);
    void Cleanup(void);

protected:
    virtual void closeEvent(QCloseEvent * event);

private slots:
    void timerEvent(QTimerEvent * event);
    void SlotEnableTeleop(bool state);
    void SlotSetScale(double scale);

private:
    //! setup TeleOperation controller GUI
    void setupUi(void);
    int TimerPeriodInMilliseconds;

protected:
    struct TeleOperationStruct {
        mtsFunctionWrite Enable;
        mtsFunctionWrite SetScale;
        mtsFunctionRead GetPositionCartesianMaster;
        mtsFunctionRead GetPositionCartesianSlave;
        mtsFunctionRead GetPeriodStatistics;
    } TeleOperation;

private:
    prmPositionCartesianGet PositionMaster;
    vctQtWidgetFrameDoubleRead * QFRPositionMasterWidget;
    prmPositionCartesianGet PositionSlave;
    vctQtWidgetFrameDoubleRead * QFRPositionSlaveWidget;
    // GUI: timing
    mtsIntervalStatistics IntervalStatistics;
    mtsQtWidgetIntervalStatistics * QMIntervalStatistics;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsTeleOperationQtWidget);

#endif // _mtsTeleOperationQtWidget_h
