/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Preetham Chalasani
  Created on: 2016-11-07

  (C) Copyright 2016-2023 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#include <sawControllers/osaCartesianImpedanceController.h>

osaCartesianImpedanceController::osaCartesianImpedanceController(void)
{
    ResetGoal();
}

void osaCartesianImpedanceController::SetGoal(const prmCartesianImpedance & goal)
{
    m_goal = goal;
}

void osaCartesianImpedanceController::ResetGoal(void)
{
    m_goal.ForceOrientation.Identity();
    m_goal.TorqueOrientation.Identity();

    m_goal.PositionPositive.Deadband.Zeros();
    m_goal.PositionNegative.Deadband.Zeros();
    m_goal.PositionPositive.P.Zeros();
    m_goal.PositionNegative.P.Zeros();
    m_goal.PositionPositive.D.Zeros();
    m_goal.PositionNegative.D.Zeros();
    m_goal.PositionPositive.Bias.Zeros();
    m_goal.PositionNegative.Bias.Zeros();

    m_goal.OrientationPositive.Deadband.Zeros();
    m_goal.OrientationNegative.Deadband.Zeros();
    m_goal.OrientationPositive.P.Zeros();
    m_goal.OrientationNegative.P.Zeros();
    m_goal.OrientationPositive.D.Zeros();
    m_goal.OrientationNegative.D.Zeros();
    m_goal.OrientationPositive.Bias.Zeros();
    m_goal.OrientationNegative.Bias.Zeros();
}

void osaCartesianImpedanceController::Update(const prmPositionCartesianGet & pose,
                                             const prmVelocityCartesianGet & twist,
                                             prmForceCartesianSet & wrenchBody,
                                             const bool needWrenchInBody)
{
    // ---- FORCE ----
    vct3 force(0.0);
    vct3 kpForce(0.0);
    vct3 kdForce(0.0);
    vct3 biasForce(0.0);

    // In phantom frame
    vct3 errPos = m_goal.ForceOrientation.Inverse() * (pose.Position().Translation() - m_goal.ForcePosition);
    vct3 velPos = m_goal.ForceOrientation.Inverse() * twist.VelocityLinear();

    for (size_t i = 0; i < 3; i++) {
        if (errPos[i] > 0) {
            errPos[i] = errPos[i] - fabs(m_goal.PositionPositive.Deadband.at(i));
            if (errPos[i] < 0.0) {
                errPos[i] = 0.0;
            }
            kpForce[i] = errPos[i] * m_goal.PositionPositive.P.at(i);
            kdForce[i] = velPos[i] * m_goal.PositionPositive.D.at(i);
            biasForce[i] = m_goal.PositionPositive.Bias.at(i);
        } else {
            errPos[i] = errPos[i] + fabs(m_goal.PositionNegative.Deadband.at(i));
            if (errPos[i] > 0.0) {
                errPos[i] = 0.0;
            }
            kpForce[i] = errPos[i] * m_goal.PositionNegative.P.at(i);
            kdForce[i] = velPos[i] * m_goal.PositionNegative.D.at(i);
            biasForce[i] = m_goal.PositionNegative.Bias.at(i);
        }
    }

    force = kpForce + kdForce + biasForce;
    force = m_goal.ForceOrientation * force;   // Force in absolute Frame
    if (needWrenchInBody) {
        force = pose.Position().Rotation().Inverse() * force;   // Force in body frame
    }

    // ---- TORQUE ----
    vct3 torque(0.0);
    vct3 kpTorque;
    vct3 kdTorque;
    vct3 biasTorque;

    // error theta
    vctRot3 tempRot;
    tempRot = m_goal.TorqueOrientation.Inverse() * pose.Position().Rotation();
    vctAxAnRot3 errAxAnRot;
    errAxAnRot.FromNormalized(tempRot);
    vct3 errRot = errAxAnRot.Angle() * errAxAnRot.Axis();
    vct3 velRot = m_goal.TorqueOrientation.Inverse() * twist.VelocityAngular();

    for (size_t i = 0; i < 3; i++) {
        if (errRot[i] > 0.0) {
            errRot[i] = errRot[i] - fabs(m_goal.OrientationPositive.Deadband.at(i));
            if (errRot[i] < 0.0) {
                errRot[i] = 0.0;
            }
            kpTorque[i] = errRot[i] * m_goal.OrientationPositive.P.at(i);
            kdTorque[i] = velRot[i] * m_goal.OrientationPositive.D.at(i);
            biasTorque[i] = m_goal.OrientationPositive.Bias.at(i);
        } else {
            errRot[i] = errRot[i] + fabs(m_goal.OrientationNegative.Deadband.at(i));
            if (errRot[i] > 0.0) {
                errRot[i] = 0.0;
            }
            kpTorque[i] = errRot[i] * m_goal.OrientationNegative.P.at(i);
            kdTorque[i] = velRot[i] * m_goal.OrientationNegative.D.at(i);
            biasTorque[i] = m_goal.OrientationNegative.Bias.at(i);
        }
    }

    torque = kpTorque + kdTorque + biasTorque;
    torque = m_goal.TorqueOrientation * torque;   // Torque in absolute Frame
    if (needWrenchInBody) {
        torque = pose.Position().Rotation().Inverse() * torque;     // Torque in Body Frame
    }

    std::copy(force.begin(), force.begin() + 3, wrenchBody.Force().begin());
    std::copy(torque.begin(), torque.begin() + 3, wrenchBody.Force().begin() + 3);
}
