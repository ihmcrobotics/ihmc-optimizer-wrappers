%module qpOASES

%{
#include "qpOASES.hpp"
using namespace qpOASES;
%}

%include "std_string.i"
%include "exception.i"
%include "enums.swg"



%include "typemaps.i"
%include "arrays_java.i"
%include "stdint.i"

%apply double[] {double*};
%apply int[] {int*};
%apply int[] {int&};
%apply long[] {long*};
%apply float[] {float*};
%apply long[] {unsigned long*};

%ignore qpOASES::Matrix::full;
%ignore qpOASES::Matrix::createDiagInfo;
%ignore dgemm_;
%ignore sgemm_;
%ignore dsyr_;
%ignore ssyr_;
%ignore dsyr2_;
%ignore ssyr2_;
%ignore dpotrf_;
%ignore spotrf_;

%include "qpOASES/Types.hpp"
%include <qpOASES/MessageHandling.hpp>
%include <qpOASES/Indexlist.hpp>
%include <qpOASES/Matrices.hpp>
%include <qpOASES/SubjectTo.hpp>
%include "qpOASES/Bounds.hpp"
%include <qpOASES/Constraints.hpp>
%include <qpOASES/ConstraintProduct.hpp>
%include "qpOASES/Options.hpp"
%include "qpOASES/QProblemB.hpp"
%include "qpOASES/QProblem.hpp"
%include "qpOASES/SQProblem.hpp"



