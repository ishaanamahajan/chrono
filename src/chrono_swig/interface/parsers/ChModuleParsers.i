//////////////////////////////////////////////////
//
//   ChModuleParser.i
//
//   SWIG configuration file.
//   This is processed by SWIG to create the C::E
//   wrapper for Python.
//
///////////////////////////////////////////////////

// Define the module to be used in Python when typing
//  'import pychrono.parsers'

%module(directors="1") parsers

// Turn on the documentation of members, for more intuitive IDE typing

%feature("autodoc", "1");
%feature("flatnested", "1");

// Turn on the exception handling to intercept C++ exceptions
%include "exception.i"

%exception {
  try {
    $action
  } catch (const std::exception& e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}

// For optional downcasting of polymorphic objects:
%include "../chrono_downcast.i" 

// For supporting shared pointers:
%include <std_shared_ptr.i>

%{
/* Includes the header in the wrapper code */
#include "chrono/core/ChFrame.h"
#include "chrono_parsers/ChParserURDF.h"
#include "chrono_parsers/ChRobotActuation.h"

#include "chrono/motion_functions/ChFunction.h"
#include "chrono/motion_functions/ChFunctionRotation.h"
#include "chrono/motion_functions/ChFunctionRotation_axis.h"
#include "chrono/motion_functions/ChFunctionRotation_ABCfunctions.h"
#include "chrono/motion_functions/ChFunctionRotation_setpoint.h"
#include "chrono/motion_functions/ChFunctionRotation_spline.h"
#include "chrono/motion_functions/ChFunctionRotation_SQUAD.h"
#include "chrono/motion_functions/ChFunctionPosition.h"
#include "chrono/motion_functions/ChFunctionPosition_line.h"
#include "chrono/motion_functions/ChFunctionPosition_setpoint.h"
#include "chrono/motion_functions/ChFunctionPosition_XYZfunctions.h"

using namespace chrono;
using namespace chrono::parsers;
%}

%shared_ptr(chrono::ChFrame<double>)
%shared_ptr(chrono::ChFunction)  

// Undefine ChApiParsers otherwise SWIG gives a syntax error
#define ChApi 
#define ChApiParsers
#define EIGEN_MAKE_ALIGNED_OPERATOR_NEW

// Include other .i configuration files for SWIG. 
// These are divided in many .i files, each per a
// different c++ class, when possible.
%include "std_string.i"
%include "std_vector.i"
%include "std_array.i"
%include "stdint.i"
%include "typemaps.i"
%include "cpointer.i"
%include "cdata.i"

%import(module = "pychrono.core")  "chrono_swig/interface/core/ChClassFactory.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChSystem.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChFrame.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChBodyFrame.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChBody.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChBodyAuxRef.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChLinkMotor.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChFunction.i"

%template(Actuation) std::vector<double>;

/* Parse the header file to generate wrappers */
%include "../../../chrono/core/ChFrame.h"    
%include "../../../chrono_parsers/ChParserURDF.h"
// note: unignore these if tinyxml2/urdfdom can be wrapped
%ignore chrono::parsers::ChParserURDF::CustomProcess;
%ignore chrono::parsers::ChParserURDF::CustomProcessor;
%ignore chrono::parsers::ChParserURDF::GetModelTree;
%include "../../../chrono_parsers/ChRobotActuation.h"

%include "../../../chrono/motion_functions/ChFunction.h"
%include "../../../chrono/motion_functions/ChFunctionRotation.h"
%include "../../../chrono/motion_functions/ChFunctionRotation_axis.h"
%include "../../../chrono/motion_functions/ChFunctionRotation_ABCfunctions.h"
%include "../../../chrono/motion_functions/ChFunctionRotation_setpoint.h"
%include "../../../chrono/motion_functions/ChFunctionRotation_spline.h"
%include "../../../chrono/motion_functions/ChFunctionRotation_SQUAD.h"
%include "../../../chrono/motion_functions/ChFunctionPosition.h"
%include "../../../chrono/motion_functions/ChFunctionPosition_line.h"
%include "../../../chrono/motion_functions/ChFunctionPosition_setpoint.h"
%include "../../../chrono/motion_functions/ChFunctionPosition_XYZfunctions.h"