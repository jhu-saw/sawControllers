#include <sawControllers/osaPDGC.h>

osaPDGC::osaPDGC( const std::string& robfile, 
		  const vctFrame4x4<double>& Rtw0,
		  const vctDynamicMatrix<double>& Kp,
		  const vctDynamicMatrix<double>& Kd,
		  const vctDynamicVector<double>& qinit ):
  robManipulator( robfile, Rtw0 ),
  Kp( Kp ),
  Kd( Kd ),
  qold( qinit ),
  eold( qinit.size(), 0.0 ){

  if( Kp.rows() != links.size() || Kp.cols() != links.size() ){
    CMN_LOG_RUN_ERROR << "size(Kp) = [" << Kp.rows() 
		      << ", "           << Kp.cols() << " ] "
		      << "NxN = ["      << links.size() 
		      << ", "           << links.size() << " ]"
		      << std::endl;
  }

  if( Kd.rows() != links.size() || Kd.cols() != links.size() ){
    CMN_LOG_RUN_ERROR << "size(Kd) = [" << Kd.rows() 
		      << ", "           << Kd.cols() << " ] "
		      << "NxN = ["      << links.size() 
		      << ", "           << links.size() << " ]"
		      << std::endl;
  }

  if( qold.size() != links.size() ){
    CMN_LOG_RUN_ERROR << "size(qold) = " << qold.size() << " "
		      << "N = "          << links.size() << std::endl;
  }

}

osaPDGC::Errno
osaPDGC::Evaluate
( const vctDynamicVector<double>& qs,
  const vctDynamicVector<double>& q,
  vctDynamicVector<double>& tau,
  double dt ){
  
  if( qs.size() != links.size() ){
    CMN_LOG_RUN_ERROR << "size(qs) = " << qs.size() << " "
		      << "N = "        << links.size() << std::endl;
    return osaPDGC::EFAILURE;
  }

  if( q.size() != links.size() ){
    CMN_LOG_RUN_ERROR << "size(q) = " << q.size() << " "
		      << "N = "       << links.size() << std::endl;
    return osaPDGC::EFAILURE;
  }

  // evaluate the velocity
  vctDynamicVector<double> qd( links.size(), 0.0 );
  if( 0 < dt ) qd = (q - qold)/dt;
    
  // error = current - desired
  vctDynamicVector<double> e = q - qs;
    
  // error time derivative
  vctDynamicVector<double> ed( links.size(), 0.0 );
  if( 0 < dt ) ed = (e - eold)/dt;      
    
  // Compute the coriolis+gravity load
  vctDynamicVector<double> ccg = 
    CCG( q, vctDynamicVector<double>( links.size(), 0.0 ) );
    
  tau = ccg - Kp*e - Kd*ed;
    
  eold = e;
  qold = q;

  return osaPDGC::ESUCCESS;

}

