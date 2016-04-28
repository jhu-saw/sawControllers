
#ifndef _osaPDGC_h
#define _osaPDGC_h

#include <cisstRobot/robManipulator.h>
#include <sawControllers/sawControllersExport.h>

class CISST_EXPORT osaPDGC : public robManipulator {

 public:

  enum Errno{ ESUCCESS, EFAILURE };

 private:

  //! Proportional gains
  vctDynamicMatrix<double> Kp;
  //! Derivative gains
  vctDynamicMatrix<double> Kd;

  //! Old joint positions
  vctDynamicVector<double> qold;
  //! Old error
  vctDynamicVector<double> eold;

 public:

  //! Main constructor
  /**
     \param[in] robfilename File containing the kinematics and dynamics 
                            parameters
     \param[in] Rtwb        Position and orientation of the robot with resepct 
                            to world frame
     \param[in] Kp          NxN matrix of proportional gains
     \param[in] Kd          NxN matrix of derivative gains
     \param[in] qinit       Initial joint positions
  */
  osaPDGC( const std::string& robfilename, 
	   const vctFrame4x4<double>& Rtwb,
	   const vctDynamicMatrix<double>& Kp,
	   const vctDynamicMatrix<double>& Kd,
	   const vctDynamicVector<double>& qinit );
  
  //! Evaluate the control law
  /**
     \param[in]  qs  Desired joint positions
     \param[in]  q   Current joint positions
     \param[out] tau Joint forces/torques
     \param      dt  Time interval
     \return     ESUCCESS if the evaluation was successful. EFAILURE otherwise
  */
  osaPDGC::Errno
    Evaluate( const vctDynamicVector<double>& qs,
	      const vctDynamicVector<double>& q,
	      vctDynamicVector<double>& tau,
	      double dt );

};

#endif
