/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: mtsPIDQt.h 3536 2013-02-20 02:25:59 zchen24 $

  Author(s):  Zihan Chen
  Created on: 2013-02-20

  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsPIDQtWidget_h
#define _mtsPIDQtWidget_h

#include <cisstCommon/cmnXMLPath.h>
#include <cisstOSAbstraction/osaTimeServer.h>
#include <cisstMultiTask/mtsComponent.h>

#include <QtCore>
#include <QtGui>


class mtsPIDQtWidget : public QWidget, public mtsComponent
{
    Q_OBJECT
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsPIDQtWidget(const std::string& taskName);
    ~mtsPIDQtWidget(){}

    void Configure(const std::string &filename = "");
    void Startup();
    void Cleanup();

protected:
    virtual void closeEvent(QCloseEvent *event);

private slots:
    //! qslot enable/disable mtsPID controller
    void slot_qcbEnablePID(bool toggle);
    //! qslot send desired pos when input changed
    void slot_qdsbPosition(double position);
    void slot_qdsbPGain(double val);
    void slot_qdsbDGain(double val);
    void slot_qdsbIGain(double val);
    //! qslot reset desired pos to current pos
    void slot_qpbResetDesiredPosition();
    //! qslot reset pid gain to current gain
    void slot_qpbResetPIDGain();

private:
    //! setup PID controller GUI
    void setupUi();
    void EventErrorLimitHandler();

protected:

    struct ControllerPIDStruct {
        mtsFunctionVoid  ResetController;
        mtsFunctionWrite Enable;
        mtsFunctionWrite SetDesiredPositions;
        mtsFunctionRead  GetPositionJoint;

        mtsFunctionRead  GetPGain;
        mtsFunctionRead  GetDGain;
        mtsFunctionRead  GetIGain;

        mtsFunctionWrite SetPGain;
        mtsFunctionWrite SetDGain;
        mtsFunctionWrite SetIGain;
    } PID;

private:

    //! SetPosition
    vctDoubleVec desiredPos;

    int numOfAxis;
    double curFBOffset;

    vctDoubleVec analogIn;
    vctDoubleVec motorFeedbackCurrent;
    vctDoubleVec motorControlCurrent;
    vctBoolVec ampEnable;
    vctBoolVec ampStatus;
    bool powerStatus;
    unsigned short safetyRelay;

    // Interface
    double tmpStatic;
    vctDynamicVector<bool> lastEnableState;
    double startTime;

    // GUI: Commands
    QCheckBox* qcbEnablePID;
    QDoubleSpinBox** qdsbPosition;
    QDoubleSpinBox** qdsbPGain;
    QDoubleSpinBox** qdsbDGain;
    QDoubleSpinBox** qdsbIGain;

    // Control
    QPushButton* quitButton;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsPIDQtWidget);

#endif // _mtsPIDQtWidget_h
