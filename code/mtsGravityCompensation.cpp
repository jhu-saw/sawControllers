#include <sawControllers/mtsGravityCompensation.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmForceTorqueJointSet.h>

mtsGravityCompensation::mtsGravityCompensation( const std::string& taskname,
						double period,
						const std::string& robfilename,
						const vctFrame4x4<double>& Rtwb,
						osaCPUMask cpumask ) :
  mtsController( taskname, period, cpumask ),
  GC( NULL ){
  
  GC = new osaGravityCompensation( robfilename, Rtwb );

  mtsInterfaceRequired* input = AddInterfaceRequired( "Input" );
  mtsInterfaceRequired* output = AddInterfaceRequired( "Output" );
  
  input->AddFunction( "GetPositionJoint", GetPositions );
  output->AddFunction( "SetTorqueJoint",  SetTorques );

}

mtsGravityCompensation::~mtsGravityCompensation(){

  if( GC != NULL )
    { delete GC; }

}

void mtsGravityCompensation::Configure( const std::string& ){}

void mtsGravityCompensation::Startup(){

}

void mtsGravityCompensation::Run(){
  ProcessQueuedCommands();

  prmPositionJointGet prmq;
  GetPositions( prmq );

  vctDynamicVector<double> tau( prmq.Position().size(), 0.0 );
  if( GC != NULL && IsEnabled() ){

    if( GC->Evaluate( prmq.Position(), tau ) != 
      osaGravityCompensation::ESUCCESS ){
      CMN_LOG_RUN_ERROR << "Faile to evaluate the controller" << std::endl;
    }

    prmForceTorqueJointSet t;
    t.ForceTorque() = tau;
    SetTorques( t );
    
  }

}

void mtsGravityCompensation::Cleanup(){}
