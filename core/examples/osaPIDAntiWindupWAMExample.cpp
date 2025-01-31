/*-*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-   */
/*ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab:*/

/*
  $Id: osaFTControlWAMExample.cpp 3181 2011-11-15 15:41:28Z sleonard $

  Author(s):  Simon Leonard
  Created on: 2013-07-22

  (C) Copyright 2012 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

/*
  This program moves the joints between an initial position qi to a final
  position qf = qi + 1.0 (rads). Make sure to move to a qi that will avoids
  interference with the workspace and the WAM (self-collision).
*/

#include <cisstCommon/cmnPath.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstOSAbstraction/osaGetTime.h>

#include <sawCANBus/osaSocketCAN.h>
#include <sawBarrett/osaWAM.h>

#include <sawControllers/osaPIDAntiWindup.h>
#include <sawControllers/osaGravityCompensation.h>

#include <stdio.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

int getch(){

    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt );
    newt = oldt;
    newt.c_lflag &= ~(ICANON);
    newt.c_lflag &= ~(ECHO);
    newt.c_cc[VMIN] = 0;
    newt.c_cc[VTIME] = 0;
    tcsetattr( STDIN_FILENO, TCSANOW, &newt );
    
    int c = getchar();
    
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
    
    return c;
    
}

int main( int argc, char** argv ){

    cmnLogger::SetMask( CMN_LOG_ALLOW_ALL );
    cmnLogger::SetMaskFunction( CMN_LOG_ALLOW_ALL );
    cmnLogger::SetMaskDefaultLog( CMN_LOG_ALLOW_ALL );
    
    if( argc != 3 ){
        std::cout << "Usage: " << argv[0] << " can[0-1] robfile" << std::endl;
        return -1;
    }

    osaSocketCAN can( argv[1], osaCANBus::RATE_1000 );
    if( can.Open() != osaCANBus::ESUCCESS ){
        CMN_LOG_RUN_ERROR << argv[0] << "Failed to open " << argv[1]
                          << std::endl;
        return -1;
    }
  
    osaWAM osaWAM( &can );
    if( osaWAM.Initialize() != osaWAM::ESUCCESS ){
        CMN_LOG_RUN_ERROR << "Failed to initialize WAM" << std::endl;
        return -1;
    }

    // WAM initial joint values
    vctDynamicVector<double> qinit( 7, 0.0 );
    qinit[1] = -cmnPI_2;
    qinit[3] =  cmnPI;
    if( osaWAM.SetPositions( qinit ) != osaWAM::ESUCCESS ){
        CMN_LOG_RUN_ERROR << "Failed to set position: " << qinit << std::endl;
        return -1;
    }

    // Rotation of the base
    vctMatrixRotation3<double> Rw0(  0.0,  0.0, -1.0,
                                     0.0,  1.0,  0.0,
                                     1.0,  0.0,  0.0 );
    vctFixedSizeVector<double,3> tw0(0.0);
    vctFrame4x4<double> Rtw0( Rw0, tw0 );

    // Gravity compensation controller
    osaGravityCompensation GC( argv[2], Rtw0 );

    // Goal joint values
    vctDynamicVector<double> qs( qinit );

    // gains (proportional, integral, derivative, anti-windup)
    vctDynamicVector<double> Kp(7,1800.0,1800.0,1800.0,1800.0,180.0,180.0,40.0);
    vctDynamicVector<double> Ki(7,3200.0,3200.0,3200.0,3200.0,200.0,200.0,80.0);
    vctDynamicVector<double> Kd(7, 10.0, 10.0, 10.0, 10.0, 0.8, 0.8, 0.2);
    vctDynamicVector<double> Kt(7, 5.0);
    // torque limits
    vctDynamicVector<double> limit(7, 40.0, 40.0, 40.0, 40.0, 15.0, 15.0, 6.5);
    osaPIDAntiWindup pid( Kp, Ki, Kd, Kt, limit, qinit );
                    
    char c = 'a';
    double t1 = osaGetTime();
    double t = 0.0;
    
    while( c != 'q' ){
        
        char tmp = getch();
        if( tmp == 'g' || tmp == 'm' || tmp == 'q' ) { c = tmp; }
        
        // Get the positions
        vctDynamicVector<double> q;
        if( osaWAM.GetPositions( q ) != osaWAM::ESUCCESS ){
            CMN_LOG_RUN_ERROR << "Failed to get positions" << std::endl;
            return -1;
        }

        double t2 = osaGetTime();
        double dt = t2 - t1;
        t1 = t2;

        switch( c ){
        case 'g':
            {
                vctDynamicVector<double> tau( q.size(), 0.0 );
                if( GC.Evaluate( q, tau ) != osaGravityCompensation::ESUCCESS ){
                    CMN_LOG_RUN_ERROR << "Failed to evaluate controller"
                                      << std::endl;
                    return -1;
                }
                if( osaWAM.SetTorques( tau ) != osaWAM::ESUCCESS ){
                    CMN_LOG_RUN_ERROR << "Failed to set torques" << std::endl;
                    return -1;
                }
                qs = q;	
                t=0.0;
            }
            break;
            
        case 'm':
            {
                t += 0.0008;
                vctDynamicVector<double> qst( qs );
                for( size_t i=0; i<qst.size(); i++ )
                    { qst(i) += 1.0 * sin(t-M_PI_2) + 1.0; }

                vctDynamicVector<double> tau;
                pid.Evaluate( qst, q, tau, dt );
                if( osaWAM.SetTorques( tau ) != osaWAM::ESUCCESS ){
                    CMN_LOG_RUN_ERROR << "Failed to set torques" << std::endl;
                    return -1;
                }
            }
            break;
            
        default:
            {
                vctDynamicVector<double> tau( q.size(), 0.0 );
                if( osaWAM.SetTorques( tau ) != osaWAM::ESUCCESS ){
                    CMN_LOG_RUN_ERROR << "Failed to set torques" << std::endl;
                    return -1;
                }
            }
        }

    }

    return 0;

}
