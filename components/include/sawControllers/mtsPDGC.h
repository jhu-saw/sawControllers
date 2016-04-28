
#ifndef _mtsPDGC_h
#define _mtsPDGC_h

#include <sawControllers/mtsController.h>
#include <sawControllers/osaPDGC.h>
#include <sawControllers/sawControllersExport.h>

class CISST_EXPORT mtsPDGC : public mtsController {

 private:

  //! Pointer to a gravity compensation controller
  osaPDGC* PDGC;

  //! Read the joint positions from the robot
  mtsFunctionRead  GetFeedbackPositions;

  //! Read the joint desired positions from a trajectory
  mtsFunctionRead  GetDesiredPositions;

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
     \param[in] Kp          NxN matrix of proportional gains
     \param[in] Kd          NxN matrix of derivative gains
     \param[in] qinit       Initial joint positions
     \param     cpumask     The mask of the allowed CPU for the task
  */
  mtsPDGC( const std::string& taskname,
	   double period,
	   const std::string& robfile,
	   const vctFrame4x4<double>& Rtwb,
	   const vctDynamicMatrix<double>& K,
	   const vctDynamicMatrix<double>& Kd,
	   const vctDynamicVector<double>& qinit,
	   osaCPUMask cpumask = OSA_CPUANY );

  ~mtsPDGC();

  void Configure( const std::string& argv );

  void Startup();
  void Run();
  void Cleanup();      

};

#endif
