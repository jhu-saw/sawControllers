#include <sawControllers/mtsController.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

mtsController::mtsController( const std::string& taskname,
			      double period,
			      osaCPUMask cpumask ) :
  mtsTaskPeriodic( taskname, period, true ),
  cpumask( cpumask ),
  ctl( NULL ),
  mtsEnabled( false ) {

  ctl = AddInterfaceProvided( "Control" );
  if( ctl ){
    
    StateTable.AddData( mtsEnabled, "Enabled" );
    ctl->AddCommandWriteState( StateTable, mtsEnabled, "Enable" );

  }
  else{
    CMN_LOG_RUN_ERROR << "Failed to create interface Control for " << GetName()
		      << std::endl;
  }

  
}

mtsController::~mtsController(){}

