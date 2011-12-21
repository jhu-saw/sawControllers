#include <sawControllers/osaGravityCompensation.h>

osaGravityCompensation::osaGravityCompensation(const std::string& robfile,
					       const vctFrame4x4<double>& Rtw0):
  robManipulator( robfile, Rtw0 ){}

osaGravityCompensation::Errno
osaGravityCompensation::Evaluate
( const vctDynamicVector<double>& q,
  vctDynamicVector<double>& tau ){
  
  if( q.size() != links.size() ){
    CMN_LOG_RUN_ERROR << "size(q) = " << q.size() << " "
		      << "N = " << links.size() << std::endl;
    return osaGravityCompensation::EFAILURE;
  }

  vctDynamicVector<double> qd( links.size(), 0.0 );   // zero velocity
  vctDynamicVector<double> qdd( links.size(), 0.0 );  // zero acceleration

  // inverse dynamics 
  tau =  InverseDynamics( q, qd, qdd );

  return osaGravityCompensation::ESUCCESS;

}

