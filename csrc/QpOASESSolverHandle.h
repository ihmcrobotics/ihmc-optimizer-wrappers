#include <qpOASES.hpp>
#include <iostream>
#include <limits>
#include <jni.h>

#define HST_ZERO_ORDINAL 0
#define HST_IDENTITY_ORDINAL 1
#define HST_POSDEF_ORDINAL 2
#define HST_POSDEF_NULLSPACE_ORDINAL 3
#define HST_SEMIDEF_ORDINAL 4
#define HST_INDEF_ORDINAL 5
#define HST_UNKNOWN_ORDINAL 6

#define RELIABLE_OPTION_ORDINAL 0
#define FAST_OPTION_ORDINAL 1
#define MPC_OPTION_ORDINAL 2
#define DEFAULT_OPTION_ORDINAL 3

namespace ihmc_optimizer_wrappers
{
   class QpOASESSolverHandle
   {
    public:
      QpOASESSolverHandle(int hessianTypeOrdinal, int solverOptionOrdinal);
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
      int hessianTypeOrdinal;
      int solverOptionOrdinal;
      int nvar;
      int ncon;
      qpOASES::SQProblem *sqProblem;
      qpOASES::Options *options;

      // Quadratic Program formulation data
      double *A;
      double *x;
      double *H;
      double *g;
      double *lb;
      double *ub;
      double *lbA;
      double *ubA;
//      double *numberOfWorkingSetChanges;
//      double *cpuTime;
//      double *objVal;

      // Quadratic Program Java direct byte buffers, for getters.
      jobject ABuffer;
      jobject xBuffer;
      jobject HBuffer;
      jobject gBuffer;
      jobject lbBuffer;
      jobject ubBuffer;
      jobject lbABuffer;
      jobject ubABuffer;

      void deleteBuffers();
   };
}