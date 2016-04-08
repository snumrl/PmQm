// +-------------------------------------------------------------------------
// | pm.h
// | 
// | Author: Jehee Lee
// +-------------------------------------------------------------------------
// | COPYRIGHT:
// |    Copyright Jehee Lee 2013
// |    See the included COPYRIGHT.txt file for further details.
// |    
// |    This file is part of the PmQm library.
// |    PmQm library is free software: you can redistribute it and/or modify
// |    it under the terms of the MIT License.
// |
// |    You should have received a copy of the MIT License
// |    along with PmQm library.  If not, see <mit-license.org>.
// +-------------------------------------------------------------------------
﻿/** 
@page Notes 주의할 점 

@section First What is First? 

하나면 하나지 둘이겠느냐... 

@section Second What is Second? 

둘이면 둘이지 셋이겠느냐? 
*/ 

#ifndef	__PM_H__
#define	__PM_H__

#include "MATHCLASS/mathclass.h"

#include "qmComplex.h"

#include "pmMask.h"
#include "pmConstraint.h"
#include "pmPosture.h"
#include "pmHuman.h"
#include "pmVector.h"
#include "pmVectorArray.h"
#include "pmMotion.h"
#include "pmLinearMotion.h"
#include "pmMRmotion.h"
#include "pmVariables.h"

// for creating experimental results which compare
// quaternion-based and euler-angles-based approaches.
// #define __EULER_ANGLES__

#endif
