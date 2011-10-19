
#ifndef _osaGravityCompensation_h
#define _osaGravityCompensation_h

#include <cisstRobot/robManipulator.h>
#include <sawControllers/sawControllersExport.h>

class CISST_EXPORT osaGravityCompensation : public robManipulator {

 public:

  enum Errno{ ESUCCESS, EFAILURE };

 public:

  osaGravityCompensation( const std::string& robfile,
			  const vctFrame4x4<double>& Rtwb );
  
  osaGravityCompensation::Errno
    Evaluate( const vctDynamicVector<double>& q,
	      vctDynamicVector<double>& tau );

};

#endif
