%module qpOASES

%{
//#define SWIG_FILE_WITH_INIT
//#define SWIG_JAVA_EXTRA_NATIVE_CONTAINERS

#include "qpOASES.hpp"
using namespace qpOASES;

%}

%include "std_string.i"
%include "exception.i"
%include "enums.swg"



/*
%include "carrays.i"
%array_class(double, doubleArray);
%array_class(int, intArray);
*/

%include "typemaps.i"
%include "arrays_java.i"
%include "stdint.i"

%apply double[] {double*};
%apply int[] {int*};
%apply int[] {int&};
%apply long[] {long*};
%apply float[] {float*};
%apply long[] {unsigned long*};



%include "qpOASES/Types.hpp"
%include <qpOASES/MessageHandling.hpp>

namespace qpOASES {
   enum returnValue;
}

/*%include "qpOASES/Bounds.hpp"
%include <qpOASES/MessageHandling.hpp>
%include <qpOASES/Constraints.hpp>
%include <qpOASES/ConstraintProduct.hpp>
%include "qpOASES/Bounds.hpp"*/

%include "qpOASES/Options.hpp"
%include "qpOASES/QProblemB.hpp"
%include "qpOASES/QProblem.hpp"
%include "qpOASES/SQProblem.hpp"




