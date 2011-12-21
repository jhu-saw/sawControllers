
#ifndef _mtsController_h
#define _mtsController_h

#include <cisstOSAbstraction/osaCPUAffinity.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <sawControllers/sawControllersExport.h>

class CISST_EXPORT mtsController : public mtsTaskPeriodic {

 protected:

  osaCPUMask cpumask;

  mtsInterfaceProvided* ctl;

  mtsBool mtsEnabled;

 protected:

  bool IsEnabled() const { return mtsEnabled; }

 public:

  mtsController( const std::string& taskname,
		 double period,
		 osaCPUMask cpumask = OSA_CPUANY );

  ~mtsController();

};

#endif
