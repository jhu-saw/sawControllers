
#include <cisstCommon/cmnPortability.h>
#if (CISST_OS == CISST_WINDOWS)
#include <windows.h>   // for WaitMessage
#endif

#include <sawKeyboard/mtsKeyboard.h>
#include <sawControllers/mtsGravityCompensation.h>

#include <cisstCommon/cmnPath.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsTaskManager.h>

#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmForceTorqueJointSet.h>

class Robot : public mtsTaskPeriodic{

private:

  prmForceTorqueJointSet tin;
  prmPositionJointGet qout;

public:

  Robot( const vctDynamicVector<double>& q ) :
    mtsTaskPeriodic( "Robot", 0.01, true ){

    qout.Position() = q;

    mtsInterfaceProvided* input = AddInterfaceProvided( "Input" );
    if( input ){

      StateTable.AddData( tin, "TorquesInput" );
      input->AddCommandWriteState( StateTable, tin, "SetTorqueJoint" );

    }
    else{
      CMN_LOG_RUN_ERROR << "Failed to create interface Input for " << GetName()
			<< std::endl;
    }

    mtsInterfaceProvided* output = AddInterfaceProvided( "Output" );
    if( output ){

      StateTable.AddData( qout, "PositionsOutput" );
      output->AddCommandReadState( StateTable, qout, "GetPositionJoint" );

    }
    else{
      CMN_LOG_RUN_ERROR << "Failed to create interface Output for " << GetName()
			<< std::endl;
    }

  }

  void Configure( const std::string& ){}
  void Startup(){}
  void Run(){
    ProcessQueuedCommands();
    for( size_t i=0; i<qout.Position().size(); i++ )
      { qout.Position()[i] += 0.001; }
    std::cout << "q:   " << qout.Position() << std::endl;
    std::cout << "tau: " << tin.ForceTorque() << std::endl;
  }
  void Cleanup(){}

};

int main(){

  mtsTaskManager* taskManager = mtsTaskManager::GetInstance();

  cmnLogger::SetMask( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskFunction( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskDefaultLog( CMN_LOG_ALLOW_ALL );

  mtsKeyboard kb;
  kb.SetQuitKey( 'q' );
  kb.AddKeyWriteFunction( 'G', "GCEnable", "Enable", true );
  taskManager->AddComponent( &kb );

  cmnPath path;
  path.AddRelativeToCisstShare("/models/WAM");
  std::string fname = path.Find("wam7.rob", cmnPath::READ);

  // Rotate the base
  vctMatrixRotation3<double> Rw0(  0.0,  0.0, -1.0,
                                   0.0,  1.0,  0.0,
                                   1.0,  0.0,  0.0 );
  vctFixedSizeVector<double,3> tw0(0.0);
  vctFrame4x4<double> Rtw0( Rw0, tw0 );

  mtsGravityCompensation GC( "GC",
			     0.01,
			     fname,
			     Rtw0 );
  taskManager->AddComponent( &GC );

  vctDynamicVector<double> q( 7, 0.0 );
  q[1] = -cmnPI_2;
  q[3] =  cmnPI;
  Robot robot( q );
  taskManager->AddComponent( &robot );

  if( !taskManager->Connect( kb.GetName(), "GCEnable",GC.GetName(),"Control") ){
    std::cout << "Failed to connect: "
	      << kb.GetName() << "::GCEnable to "
	      << kb.GetName()  << "::Control" << std::endl;
    return -1;
  }

  if( !taskManager->Connect(robot.GetName(), "Input", GC.GetName(), "Output") ){
    std::cout << "Failed to connect: "
	      << robot.GetName() << "::Input to "
	      << GC.GetName()    << "::Output" << std::endl;
    return -1;
  }

  if( !taskManager->Connect(robot.GetName(), "Output", GC.GetName(), "Input") ){
    std::cout << "Failed to connect: "
	      << robot.GetName() << "::Output to "
	      << GC.GetName()    << "::Input" << std::endl;
    return -1;
  }

  taskManager->CreateAll();
  taskManager->StartAll();

#if (CISST_OS == CISST_WINDOWS)
  WaitMessage();
#else
  pause();
#endif

  taskManager->KillAll();
  taskManager->Cleanup();

  return 0;

}
