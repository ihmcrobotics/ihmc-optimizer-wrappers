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
      ~QpOASESSolverHandle();

      int getNVar();
      int getNCon();
      void setupQPOASES(int nvar, int ncon);
      void setupQuadraticProgramBuffers(JNIEnv *env);

      jobject getABuffer();
      jobject getXBuffer();
      jobject getHessianBuffer();
      jobject getGradientBuffer();
      jobject getLowerBoundBuffer();
      jobject getUpperBoundBuffer();
      jobject getLowerBoundABuffer();
      jobject getUpperBoundABuffer();

    private:
      int nvar;
      int ncon;
      qpOASES::SQProblem *sqProblem;
      qpOASES::Options *options;

      // Quadratic Program formulation data
      double *A;
      double *x;
      double *hessian;
      double *gradient;
      double *lowerBound;
      double *upperBound;
      double *lowerBoundA;
      double *upperBoundA;
//      double *numberOfWorkingSetChanges;
//      double *cpuTime;
//      double *objVal;

      // Quadratic Program Java direct byte buffers, for getters.
      jobject ABuffer;
      jobject xBuffer;
      jobject hessianBuffer;
      jobject gradientBuffer;
      jobject lowerBoundBuffer;
      jobject upperBoundBuffer;
      jobject lowerBoundABuffer;
      jobject upperBoundABuffer;

      void deleteBuffers();
   };
}