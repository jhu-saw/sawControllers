/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-  */
/*ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab:*/

/*
  $Id: osaPIDAntiWindup.cpp 4416 2013-08-20 01:45:09Z sleonar7 $
  
  Author(s):  Simon Leonard
  Created on: 2013-11-30
  
  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.
  
  --- begin cisst license - do not edit ---
  
  This software is provided "as is" under an open source license, with
  no warranty.  The complete license can be found in license.txt and
  http://www.cisst.org/cisst/license.txt.
  
--- end cisst license ---
*/

#include <sawControllers/osaPIDAntiWindup.h>
#include <cisstOSAbstraction/osaGetTime.h>

osaPIDAntiWindup::osaPIDAntiWindup( const vctDynamicVector<double>& Kp,
                                    const vctDynamicVector<double>& Ki,
                                    const vctDynamicVector<double>& Kd,
                                    const vctDynamicVector<double>& Kt,
                                    const vctDynamicVector<double>& limits,
                                    const vctDynamicVector<double>& qinit ) :
    Kp( Kp ),
    Ki( Ki ),
    Kd( Kd ),
    Kt( Kt ),
    I( Ki.size(), 0.0 ),
    commands( limits.size(), 0.0 ),
    outputs( limits.size(), 0.0 ),
    limits( limits.size(), 0.0 ),
    qold( qinit ),
    eold( qinit.size(), 0.0 ){

    if( this->Kp.size() != this->Kd.size() || 
        this->Kp.size() != this->Ki.size() || 
        this->Kp.size() != this->Kt.size() ){
        std::cerr << "Size mismatch:" 
                          << " size(Kp) = " << this->Kp.size()
                          << " size(Ki) = " << this->Ki.size()
                          << " size(Kd) = " << this->Kd.size()
                          << " size(Kt) = " << this->Kt.size()
                          << std::endl;
    }

    if( this->Kp.size() != this->limits.size() ){
        std::cerr << "Size mismatch: initializing " << this->Kp.size() 
                          << " controllers with " << this->limits.size() 
                          << " limit values."
                          << std::endl;
    }

    if( this->Kp.size() != this->qold.size() ){
        std::cerr << "Size mismatch: initializing " << this->Kp.size() 
                          << " controllers with " << this->qold.size() 
                          << " values."
                          << std::endl;
    }

    for( size_t i=0; i<this->limits.size(); i++ )
        { this->limits[i] = fabs( limits[i] ); }

}

osaPIDAntiWindup::Errno
osaPIDAntiWindup::Evaluate
( const vctDynamicVector<double>& qs,
  const vctDynamicVector<double>& q,
  vctDynamicVector<double>& tau,
  double dt ){

    if( dt <= 0 ){
        std::cerr << "Invalid time increment: " << dt << std::endl;
        return osaPIDAntiWindup::EFAILURE;
    }
    
    if( qs.size() != Kp.size() ){
        std::cerr << "size(qs) = " << qs.size() << " "
                          << "N = "        << Kp.size() << std::endl;
        return osaPIDAntiWindup::EFAILURE;
    }
    
    if( q.size() != Kp.size() ){
        std::cerr << "size(q) = " << q.size() << " "
                          << "N = "       << Kp.size() << std::endl;
        return osaPIDAntiWindup::EFAILURE;
    }

    // error = desired - current
    vctDynamicVector<double> e = qs - q;

    // evaluate the velocity
    vctDynamicVector<double> qd = (q - qold)/dt;
    
    // error time derivative
    vctDynamicVector<double> ed = (e - eold)/dt;      
    
    // command with anti windup
    for( size_t i=0; i<commands.size(); i++ ){
        I[i] += ( Ki[i]*e[i] + Kt[i]*(outputs[i]-commands[i]) ) * dt;
        commands[i] = Kp[i]*e[i] + Kd[i]*ed[i] + I[i];
    }

    // set the output to the command
    outputs = commands;

    // saturate the output
    for( size_t i=0; i<outputs.size(); i++ ){
        if( outputs[i] < -limits[i] ) { outputs[i] = -limits[i]; }
        if( limits[i]  < outputs[i] ) { outputs[i] =  limits[i]; }
    }
    
    tau = outputs;

    eold = e;
    qold = q;
    
    return osaPIDAntiWindup::ESUCCESS;
    
}

