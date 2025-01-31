
#ifndef _osaGravityCompensation_h
#define _osaGravityCompensation_h

#include <cisstRobot/robManipulator.h>
#include <sawControllers/sawControllersExport.h>

class CISST_EXPORT osaGravityCompensation : public robManipulator {

 public:

  enum Errno{ ESUCCESS, EFAILURE };

 public:

  //! Main constructor
  /**
     \param[in] robfilename File containing the kinematics and dynamics 
                            parameters
     \param[in] Rtwb        Position and orientation of the robot with resepct 
                            to world frame
  */
  osaGravityCompensation( const std::string& robfile,
			  const vctFrame4x4<double>& Rtwb );
  

  //! Evaluate the control law
  /**
     \param[in]  qs  Desired joint positions
     \param[out] tau Joint forces/torques
     \return     ESUCCESS if the evaluation was successful. EFAILURE otherwise
  */
  osaGravityCompensation::Errno
    Evaluate( const vctDynamicVector<double>& q,
	      vctDynamicVector<double>& tau );

};

#endif
