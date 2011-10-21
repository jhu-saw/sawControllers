
#ifndef _mtsGravityCompensation_h
#define _mtsGravityCompensation_h

#include <sawControllers/mtsController.h>
#include <sawControllers/osaGravityCompensation.h>
#include <sawControllers/sawControllersExport.h>

class CISST_EXPORT mtsGravityCompensation : public mtsController {

 private:

  //! Pointer to a gravity compensation controller
  osaGravityCompensation* GC;

  //! Read the joint positions
  mtsFunctionRead  GetPositions;

  //! Write the joint torques
  mtsFunctionWrite SetTorques;

 public:

  //! Main constructor
  /**
     \param[in] taskname    The name of the MTS periodic task
     \param     period      The period of the task
     \param[in] robfilename File containing the kinematics and dynamics 
                            parameters
     \param[in] Rtwb        Position and orientation of the robot with resepct 
                            to world frame
     \param     cpumask     The mask of the allowed CPU for the task
		
  */
  mtsGravityCompensation( const std::string& taskname,
			  double period,
			  const std::string& robfilename,
			  const vctFrame4x4<double>& Rtwb,
			  osaCPUMask cpumask = OSA_CPUANY );

  ~mtsGravityCompensation();

  void Configure( const std::string& argv );

  void Startup();
  void Run();
  void Cleanup();      

};

#endif
