#include <sawControllers/osaImpedanceController.h>

osaImpedanceController::osaImpedanceController()
{

}

void osaImpedanceController::SetGains(const prmFixtureGainCartesianSet &gains)
{
    mImpedanceGains = gains;
}

void osaImpedanceController::Update(const prmPositionCartesianGet & pose, const prmVelocityCartesianGet & twist, prmForceCartesianSet & wrenchBody)
{
    // ---- FORCE ----
    vct3 force(0.0);
    vct3 kpForce(0.0);
    vct3 kdForce(0.0);
    vct3 biasForce(0.0);

    // In phantom frame
    vct3 errPos = mImpedanceGains.ForceOrientation().Inverse() * (pose.Position().Translation() - mImpedanceGains.ForcePosition());
    vct3 velPos = mImpedanceGains.ForceOrientation().Inverse() * twist.VelocityLinear();

    for (size_t i = 0; i < 3; i++) {
        if (errPos[i] > 0) {
            kpForce[i] = errPos[i] * mImpedanceGains.PositionStiffnessPos().at(i);
            kdForce[i] = velPos[i] * mImpedanceGains.PositionDampingPos().at(i);
            biasForce[i] = mImpedanceGains.ForceBiasPos().at(i);
        } else {
            kpForce[i] = errPos[i] * mImpedanceGains.PositionStiffnessNeg().at(i);
            kdForce[i] = velPos[i] * mImpedanceGains.PositionDampingNeg().at(i);
            biasForce[i] = mImpedanceGains.ForceBiasNeg().at(i);
        }
    }

    force = kpForce + kdForce + biasForce;
    force = pose.Position().Rotation().Inverse() * mImpedanceGains.ForceOrientation() * force;   // Force in Body Frame


    // ---- TORQUE ----
    vct3 torque(0.0);
    vct3 kpTorque;
    vct3 kdTorque;
    vct3 biasTorque;

    // error theta
    vctRot3 tempRot;
    tempRot = mImpedanceGains.TorqueOrientation().Inverse() * pose.Position().Rotation();
    vctAxAnRot3 errAxAnRot;
    errAxAnRot.FromNormalized(tempRot);
    vct3 errRot = errAxAnRot.Angle() * errAxAnRot.Axis();
    vct3 velRot = mImpedanceGains.ForceOrientation().Inverse() * twist.VelocityAngular();

    for (size_t i = 0; i < 3; i++) {
        if (errRot[i] > 0) {
            kpTorque[i] = errRot[i] * mImpedanceGains.OrientationStiffnessPos().at(i);
            kdTorque[i] = velRot[i] * mImpedanceGains.OrientationDampingPos().at(i);
            biasTorque[i] = mImpedanceGains.TorqueBiasPos().at(i);
        } else  {
            kpTorque[i] = errRot[i] * mImpedanceGains.OrientationStiffnessNeg().at(i);
            kdTorque[i] = velRot[i] * mImpedanceGains.OrientationDampingNeg().at(i);
            biasTorque[i] = mImpedanceGains.TorqueBiasNeg().at(i);
        }
    }

    torque = kpTorque + kdTorque + biasTorque;
    torque = pose.Position().Rotation().Inverse() * mImpedanceGains.TorqueOrientation() * torque;   // Torque in Body Frame

    std::copy(force.begin(), force.begin()+3, wrenchBody.Force().begin());
    std::copy(torque.begin(), torque.begin()+3, wrenchBody.Force().begin()+3);
}
