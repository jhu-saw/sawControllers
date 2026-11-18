
#ifndef _mtsController_h
#define _mtsController_h

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <sawControllers/sawControllersExport.h>

class CISST_EXPORT mtsController : public mtsTaskPeriodic {

 protected:
  mtsInterfaceProvided* ctl;

  mtsBool mtsEnabled;

 protected:

  bool IsEnabled() const { return mtsEnabled; }

 public:

  mtsController( const std::string& taskname,
		 double period
  );

  ~mtsController();

};

#endif
