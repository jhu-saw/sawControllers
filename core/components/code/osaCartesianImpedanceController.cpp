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


#include <Eigen/Geometry>
#include <sawControllers/osaCartesianImpedanceController.h>

osaCartesianImpedanceController::osaCartesianImpedanceController()
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

    m_goal.PositionPositive.Deadband.setZero();
    m_goal.PositionNegative.Deadband.setZero();
    m_goal.PositionPositive.P.setZero();
    m_goal.PositionNegative.P.setZero();
    m_goal.PositionPositive.D.setZero();
    m_goal.PositionNegative.D.setZero();
    m_goal.PositionPositive.Bias.setZero();
    m_goal.PositionNegative.Bias.setZero();

    m_goal.OrientationPositive.Deadband.setZero();
    m_goal.OrientationNegative.Deadband.setZero();
    m_goal.OrientationPositive.P.setZero();
    m_goal.OrientationNegative.P.setZero();
    m_goal.OrientationPositive.D.setZero();
    m_goal.OrientationNegative.D.setZero();
    m_goal.OrientationPositive.Bias.setZero();
    m_goal.OrientationNegative.Bias.setZero();
}

void osaCartesianImpedanceController::Update(const prmPositionCartesianGet & pose,
                                             const prmVelocityCartesianGet & twist,
                                             prmForceCartesianSet & wrenchBody,
                                             const bool needWrenchInBody)
{
    // ---- FORCE ----
    Eigen::Vector3d force = Eigen::Vector3d::Zero();
    Eigen::Vector3d kpForce = Eigen::Vector3d::Zero();
    Eigen::Vector3d kdForce = Eigen::Vector3d::Zero();
    Eigen::Vector3d biasForce = Eigen::Vector3d::Zero();

    // In phantom frame
    Eigen::Vector3d errPos = m_goal.ForceOrientation.transpose() * (pose.Position().translation() - m_goal.ForcePosition);
    Eigen::Vector3d velPos = m_goal.ForceOrientation.transpose() * twist.VelocityLinear();

    for (size_t i = 0; i < 3; i++) {
        if (errPos[i] > 0) {
            errPos[i] = errPos[i] - fabs(m_goal.PositionPositive.Deadband(i));
            if (errPos[i] < 0.0) {
                errPos[i] = 0.0;
            }
            kpForce[i] = errPos[i] * m_goal.PositionPositive.P(i);
            kdForce[i] = velPos[i] * m_goal.PositionPositive.D(i);
            biasForce[i] = m_goal.PositionPositive.Bias(i);
        } else {
            errPos[i] = errPos[i] + fabs(m_goal.PositionNegative.Deadband(i));
            if (errPos[i] > 0.0) {
                errPos[i] = 0.0;
            }
            kpForce[i] = errPos[i] * m_goal.PositionNegative.P(i);
            kdForce[i] = velPos[i] * m_goal.PositionNegative.D(i);
            biasForce[i] = m_goal.PositionNegative.Bias(i);
        }
    }

    force = kpForce + kdForce + biasForce;
    force = m_goal.ForceOrientation * force;   // Force in absolute Frame
    if (needWrenchInBody) {
        force = pose.Position().rotation().inverse() * force;   // Force in body frame
    }

    // ---- TORQUE ----
    Eigen::Vector3d torque = Eigen::Vector3d::Zero();
    Eigen::Vector3d kpTorque = Eigen::Vector3d::Zero();
    Eigen::Vector3d kdTorque = Eigen::Vector3d::Zero();
    Eigen::Vector3d biasTorque = Eigen::Vector3d::Zero();

    // error theta
    Eigen::Matrix3d temp_rot = m_goal.TorqueOrientation.transpose() * pose.Position().rotation();
    Eigen::AngleAxisd errAxAnRot(temp_rot);
    Eigen::Vector3d errRot = errAxAnRot.angle() * errAxAnRot.axis();
    Eigen::Vector3d velRot = m_goal.TorqueOrientation.transpose() * twist.VelocityAngular();

    for (size_t i = 0; i < 3; i++) {
        if (errRot[i] > 0.0) {
            errRot[i] = errRot[i] - fabs(m_goal.OrientationPositive.Deadband(i));
            if (errRot[i] < 0.0) {
                errRot[i] = 0.0;
            }
            kpTorque[i] = errRot[i] * m_goal.OrientationPositive.P(i);
            kdTorque[i] = velRot[i] * m_goal.OrientationPositive.D(i);
            biasTorque[i] = m_goal.OrientationPositive.Bias(i);
        } else {
            errRot[i] = errRot[i] + fabs(m_goal.OrientationNegative.Deadband(i));
            if (errRot[i] > 0.0) {
                errRot[i] = 0.0;
            }
            kpTorque[i] = errRot[i] * m_goal.OrientationNegative.P(i);
            kdTorque[i] = velRot[i] * m_goal.OrientationNegative.D(i);
            biasTorque[i] = m_goal.OrientationNegative.Bias(i);
        }
    }

    torque = kpTorque + kdTorque + biasTorque;
    torque = m_goal.TorqueOrientation * torque;   // Torque in absolute Frame
    if (needWrenchInBody) {
        torque = pose.Position().rotation().inverse() * torque;     // Torque in Body Frame
    }

    std::copy(force.begin(), force.begin() + 3, wrenchBody.Force().begin());
    std::copy(torque.begin(), torque.begin() + 3, wrenchBody.Force().begin() + 3);
}
