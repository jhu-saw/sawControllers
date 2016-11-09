#ifndef _osaImpedanceController_h
#define _osaImpedanceController_h

#include <cisstParameterTypes/prmFixtureGainCartesianSet.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstVector/vctFrame4x4.h>
#include <sawControllers/sawControllersExport.h>

class CISST_EXPORT osaImpedanceController
{
public:
    osaImpedanceController();
    ~osaImpedanceController(){}

    void Update(const prmPositionCartesianGet & pose, const prmVelocityCartesianGet & twist,
                prmForceCartesianSet & wrenchBody, bool needWrenchInBody = false);
    void SetGains(const prmFixtureGainCartesianSet & gains);
    void ResetGains();

private:
    prmFixtureGainCartesianSet mImpedanceGains;
};
#endif // _osaImpedanceController_h
