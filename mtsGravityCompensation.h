
#ifndef _mtsGravityCompensation_h
#define _mtsGravityCompensation_h

#include <sawControllers/mtsController.h>
#include <sawControllers/osaGravityCompensation.h>
#include <sawControllers/sawControllersExport.h>

class CISST_EXPORT mtsGravityCompensation : public mtsController {

 private:

  osaGravityCompensation* GC;

  mtsFunctionRead  GetPositions;
  mtsFunctionWrite SetTorques;

 public:

  mtsGravityCompensation( const std::string& taskname,
			  double period,
			  const std::string& robfile,
			  const vctFrame4x4<double>& Rtwb,
			  osaCPUMask cpumask = OSA_CPUANY );

  ~mtsGravityCompensation();

  void Configure( const std::string& argv );

  void Startup();
  void Run();
  void Cleanup();      

};

#endif
