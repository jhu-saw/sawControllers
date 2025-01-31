/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */
/*

  Author(s):  Simon Leonard
  Created on: 2012

  (C) Copyright 2012 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawControllers/osaGravityCompensation.h>

osaGravityCompensation::osaGravityCompensation(const std::string& robfile,
					       const vctFrame4x4<double>& Rtw0):
    robManipulator( robfile, Rtw0 ){}

osaGravityCompensation::Errno
osaGravityCompensation::Evaluate
( const vctDynamicVector<double>& q,
  vctDynamicVector<double>& tau ){

    if( q.size() != links.size() ){
	CMN_LOG_RUN_ERROR << "size(q) = " << q.size() << " "
			  << "N = " << links.size() << std::endl;
	return osaGravityCompensation::EFAILURE;
    }

    vctDynamicVector<double> qd( links.size(), 0.0 );   // zero velocity
    vctDynamicVector<double> qdd( links.size(), 0.0 );  // zero acceleration

    // inverse dynamics
    tau =  InverseDynamics( q, qd, qdd );

    return osaGravityCompensation::ESUCCESS;

}
