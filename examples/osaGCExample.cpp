#include <cisstCommon/cmnPath.h>
#include <sawControllers/osaGravityCompensation.h>

int main(){

  cmnLogger::SetMask( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskFunction( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskDefaultLog( CMN_LOG_ALLOW_ALL );

  cmnPath path;
  path.AddRelativeToCisstShare("/models/WAM");
  std::string fname = path.Find("wam7.rob", cmnPath::READ);

  // Rotate the base
  vctMatrixRotation3<double> Rw0(  0.0,  0.0, -1.0,
                                   0.0,  1.0,  0.0,
                                   1.0,  0.0,  0.0 );
  vctFixedSizeVector<double,3> tw0(0.0);
  vctFrame4x4<double> Rtw0( Rw0, tw0 );

  osaGravityCompensation GC( fname, Rtw0 );


  vctDynamicVector<double> q( 7, 0.0 );
  q[1] = -cmnPI_2;
  q[3] =  cmnPI;

  vctDynamicVector<double> tau;

  if( GC.Evaluate( q, tau ) != osaGravityCompensation::ESUCCESS ){
    CMN_LOG_RUN_ERROR << "Failed to evaluate gravity compensation" << std::endl;
    return -1;
  }

  std::cout << tau << std::endl;

  return 0;

}
