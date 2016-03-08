#include <qpOASES.hpp>
#include <iostream>
#include <limits>
#include <jni.h>

namespace ihmc_optimizer_wrappers
{
    class QpOASESSolverHandle
    {
        public:
            QpOASESSolverHandle();
            virtual ~QpOASESSolverHandle();
    }
}