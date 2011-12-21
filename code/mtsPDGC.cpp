#include <sawControllers/mtsPDGC.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmForceTorqueJointSet.h>

mtsPDGC::mtsPDGC( const std::string& taskname,
		  double period,
		  const std::string& robfile,
		  const vctFrame4x4<double>& Rtwb,
		  const vctDynamicMatrix<double>& Kp,
		  const vctDynamicMatrix<double>& Kd,
		  const vctDynamicVector<double>& qinit,
		  osaCPUMask cpumask ) :
  mtsController( taskname, period, cpumask ),
  PDGC( NULL ){
  
  PDGC = new osaPDGC( robfile, Rtwb, Kp, Kd, qinit );

  mtsInterfaceRequired* input    = AddInterfaceRequired( "Input", MTS_OPTIONAL);
  mtsInterfaceRequired* output   = AddInterfaceRequired( "Output" );
  mtsInterfaceRequired* feedback = AddInterfaceRequired( "Feedback" );
  
  input->AddFunction(    "GetPositionJoint", GetDesiredPositions );
  feedback->AddFunction( "GetPositionJoint", GetFeedbackPositions );
  output->AddFunction(   "SetTorqueJoint",  SetTorques );

}

mtsPDGC::~mtsPDGC(){

  if( PDGC != NULL )
    { delete PDGC; }

}

void mtsPDGC::Configure( const std::string& ){}

void mtsPDGC::Startup(){

  osaCPUSetAffinity( OSA_CPU3 );

}

void mtsPDGC::Run(){
  ProcessQueuedCommands();

  prmPositionJointGet prmqs;
  GetDesiredPositions( prmqs );

  prmPositionJointGet prmq;
  GetFeedbackPositions( prmq );

  vctDynamicVector<double> tau( prmq.Position().size(), 0.0 );
  if( PDGC != NULL && IsEnabled() ){

    vctDynamicVector<double> qs = prmqs.Position();
    vctDynamicVector<double> q  = prmq.Position();
    //std::cout << "qs: " << qs << std::endl
    //      << "q : " << q << std::endl << std::endl;

    double dt = GetPeriodicity();
    if( PDGC->Evaluate( qs, q, tau, dt ) != osaPDGC::ESUCCESS )
      { CMN_LOG_RUN_ERROR << "Faile to evaluate the controller" << std::endl; }

    prmForceTorqueJointSet t;
    t.ForceTorque() = tau;
    SetTorques( t );
    
  }

}

void mtsPDGC::Cleanup(){}
