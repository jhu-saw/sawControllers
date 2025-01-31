#include <cisstCommon/cmnPath.h>
#include <sawControllers/osaPDGC.h>

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

  vctDynamicVector<double> qinit( 7, 0.0 );
  qinit[1] = -cmnPI_2;
  qinit[3] =  cmnPI;

  // Gain matrices
  vctDynamicMatrix<double> Kp(7, 7, 0.0), Kd(7, 7, 0.0);
  Kp[0][0] = 250;     Kd[0][0] = 3.0;
  Kp[1][1] = 250;     Kd[1][1] = 3.0;
  Kp[2][2] = 250;     Kd[2][2] = 3.0;
  Kp[3][3] = 200;     Kd[3][3] = 3;
  Kp[4][4] = 50;      Kd[4][4] = 0.8;
  Kp[5][5] = 50;      Kd[5][5] = 0.8;
  Kp[6][6] = 10;      Kd[6][6] = .1;

  osaPDGC PDGC( fname, Rtw0, Kp, Kd, qinit );


  vctDynamicVector<double> tau;

  if( PDGC.Evaluate( qinit, qinit, tau, 0.002 ) != osaPDGC::ESUCCESS ){
    CMN_LOG_RUN_ERROR << "Failed to evaluate gravity compensation" << std::endl;
    return -1;
  }

  std::cout << tau << std::endl;

  return 0;

}
