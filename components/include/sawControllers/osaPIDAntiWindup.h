/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-  */
/*ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab:*/

/*
  $Id: osaPIDAntiWindup.h 4416 2013-08-20 01:45:09Z sleonar7 $
  
  Author(s):  Simon Leonard
  Created on: 2013-11-30
  
  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.
  
  --- begin cisst license - do not edit ---
  
  This software is provided "as is" under an open source license, with
  no warranty.  The complete license can be found in license.txt and
  http://www.cisst.org/cisst/license.txt.
  
--- end cisst license ---
*/

#ifndef _osaPIDAntiWindup_h
#define _osaPIDAntiWindup_h

#include <cisstVector/vctDynamicVector.h>
#include <sawControllers/sawControllersExport.h>

class CISST_EXPORT osaPIDAntiWindup {

 public:

  enum Errno{ ESUCCESS, EFAILURE };

 private:

  //! Vector of proportional gains
  vctDynamicVector<double> Kp; 

  //! Vector of integral gains
  vctDynamicVector<double> Ki;

  //! Vector of derivative gains
  vctDynamicVector<double> Kd;

  //! Vector of anti-windup gains
  vctDynamicVector<double> Kt;

  //! Integral accumulator
  vctDynamicVector<double> I;

  //! Controller command
  vctDynamicVector<double> commands;

  //! Controller output
  vctDynamicVector<double> outputs;

  //! Plant limits
  vctDynamicVector<double> limits;
  
  //! Old position
  vctDynamicVector<double> qold;

  //! Old error
  vctDynamicVector<double> eold;

 protected:


 public:

  //! Main constructor
  /**
     \param[in]  Kp Vector of proportional gains
     \param[in]  Ki Vector of integral gains
     \param[in]  Kd Vector of derivative gains
     \param[in]  Kt Vector of anti-windup gains
     \param[in]  limits Vector of plant limits
     \param[in]  qinit Initial positions
  */
  osaPIDAntiWindup( const vctDynamicVector<double>& Kp,
                    const vctDynamicVector<double>& Ki,
                    const vctDynamicVector<double>& Kd,
                    const vctDynamicVector<double>& Kt,
                    const vctDynamicVector<double>& limits,
                    const vctDynamicVector<double>& qinit );

  //! Evaluate the control law
  /**
     \param[in]  qs  Desired joint positions
     \param[in]  q   Current joint positions
     \param[out] tau Joint forces/torques
     \param      dt  Time interval
     \return     ESUCCESS if the evaluation was successful. EFAILURE otherwise
  */
  osaPIDAntiWindup::Errno Evaluate( const vctDynamicVector<double>& qs,
                                    const vctDynamicVector<double>& q,
                                    vctDynamicVector<double>& tau,
                                    double dt );
  
};

#endif
