/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Preetham Chalasani
  Created on: 2016-11-07

  (C) Copyright 2016-2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#include <sawControllers/osaCartesianImpedanceController.h>

osaCartesianImpedanceController::osaCartesianImpedanceController(void)
{
    ResetGains();
}

void osaCartesianImpedanceController::SetGains(const prmCartesianImpedanceGains & gains)
{
    mGains = gains;
}

void osaCartesianImpedanceController::ResetGains(void)
{
    mGains.ForceOrientation().Identity();
    mGains.TorqueOrientation().Identity();

    mGains.PositionDeadbandPos().Zeros();
    mGains.PositionDeadbandNeg().Zeros();
    mGains.PositionStiffnessPos().Zeros();
    mGains.PositionStiffnessNeg().Zeros();
    mGains.PositionDampingPos().Zeros();
    mGains.PositionDampingNeg().Zeros();
    mGains.ForceBiasPos().Zeros();
    mGains.ForceBiasNeg().Zeros();
    
    mGains.OrientationDeadbandPos().Zeros();
    mGains.OrientationDeadbandNeg().Zeros();
    mGains.OrientationStiffnessPos().Zeros();
    mGains.OrientationStiffnessNeg().Zeros();
    mGains.OrientationDampingPos().Zeros();
    mGains.OrientationDampingNeg().Zeros();
    mGains.TorqueBiasPos().Zeros();
    mGains.TorqueBiasNeg().Zeros();
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
    vct3 errPos = mGains.ForceOrientation().Inverse() * (pose.Position().Translation() - mGains.ForcePosition());
    vct3 velPos = mGains.ForceOrientation().Inverse() * twist.VelocityLinear();

    for (size_t i = 0; i < 3; i++) {
        if (errPos[i] > 0) {
            errPos[i] = errPos[i] - fabs(mGains.PositionDeadbandPos().at(i));
            if (errPos[i] < 0.0) {
                errPos[i] = 0.0;
            }
            kpForce[i] = errPos[i] * mGains.PositionStiffnessPos().at(i);
            kdForce[i] = velPos[i] * mGains.PositionDampingPos().at(i);
            biasForce[i] = mGains.ForceBiasPos().at(i);
        } else {
            errPos[i] = errPos[i] + fabs(mGains.PositionDeadbandNeg().at(i));
            if (errPos[i] > 0.0) {
                errPos[i] = 0.0;
            }
            kpForce[i] = errPos[i] * mGains.PositionStiffnessNeg().at(i);
            kdForce[i] = velPos[i] * mGains.PositionDampingNeg().at(i);
            biasForce[i] = mGains.ForceBiasNeg().at(i);
        }
    }

    force = kpForce + kdForce + biasForce;
    force = mGains.ForceOrientation() * force;   // Force in absolute Frame
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
    tempRot = mGains.TorqueOrientation().Inverse() * pose.Position().Rotation();
    vctAxAnRot3 errAxAnRot;
    errAxAnRot.FromNormalized(tempRot);
    vct3 errRot = errAxAnRot.Angle() * errAxAnRot.Axis();
    vct3 velRot = mGains.TorqueOrientation().Inverse() * twist.VelocityAngular();

    for (size_t i = 0; i < 3; i++) {
        if (errRot[i] > 0.0) {
            errRot[i] = errRot[i] - fabs(mGains.OrientationDeadbandPos().at(i));
            if (errRot[i] < 0.0) {
                errRot[i] = 0.0;
            }
            kpTorque[i] = errRot[i] * mGains.OrientationStiffnessPos().at(i);
            kdTorque[i] = velRot[i] * mGains.OrientationDampingPos().at(i);
            biasTorque[i] = mGains.TorqueBiasPos().at(i);
        } else {
            errRot[i] = errRot[i] + fabs(mGains.OrientationDeadbandNeg().at(i));
            if (errRot[i] > 0.0) {
                errRot[i] = 0.0;
            }
            kpTorque[i] = errRot[i] * mGains.OrientationStiffnessNeg().at(i);
            kdTorque[i] = velRot[i] * mGains.OrientationDampingNeg().at(i);
            biasTorque[i] = mGains.TorqueBiasNeg().at(i);
        }
    }

    torque = kpTorque + kdTorque + biasTorque;
    torque = mGains.TorqueOrientation() * torque;   // Torque in absolute Frame
    if (needWrenchInBody) {
        torque = pose.Position().Rotation().Inverse() * torque;     // Torque in Body Frame
    }

    std::copy(force.begin(), force.begin() + 3, wrenchBody.Force().begin());
    std::copy(torque.begin(), torque.begin() + 3, wrenchBody.Force().begin() + 3);
}
