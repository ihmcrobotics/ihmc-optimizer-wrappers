#include "QpOASESSolverHandle.h"

namespace ihmc_optimizer_wrappers
{
   QpOASESSolverHandle::QpOASESSolverHandle()
   {
   }

   QpOASESSolverHandle::~QpOASESSolverHandle()
   {
      delete this->sqProblem;

      // Not sure if this needs to be done, I'll play with this later.
//      delete this->options;

      this->deleteBuffers();
   }

   int QpOASESSolverHandle::getNCon()
   {
      return this->ncon;
   }

   int QpOASESSolverHandle::getNVar()
   {
      return this->nvar;
   }

   void QpOASESSolverHandle::setupQPOASES(int nvar, int ncon)
   {
      this->nvar = nvar;
      this->ncon = ncon;

      this->sqProblem = new qpOASES::SQProblem(this->nvar, this->ncon, qpOASES::HST_SEMIDEF);
      this->options = new qpOASES::Options;

      this->options->setToReliable();
      this->options->printLevel = qpOASES::PL_LOW;

      this->sqProblem->setOptions(*(this->options));
   }

   void QpOASESSolverHandle::setupQuadraticProgramBuffers(JNIEnv *env)
   {
      this->A = new double[this->nvar * this->ncon];
      this->x = new double[this->nvar];
      this->hessian = new double[this->nvar * this->nvar];
      this->gradient = new double[this->nvar];
      this->lowerBound = new double[this->nvar];
      this->upperBound = new double[this->nvar];
      this->lowerBoundA = new double[this->nvar];
      this->upperBoundA = new double[this->nvar];

      this->ABuffer = env->NewDirectByteBuffer(A, (jlong) (sizeof(double) * (this->nvar * this->ncon)));
      this->xBuffer = env->NewDirectByteBuffer(x, (jlong) (sizeof(double) * (this->nvar)));
      this->hessianBuffer = env->NewDirectByteBuffer(hessian, (jlong) (sizeof(double) * (this->nvar * this->nvar)));
      this->gradientBuffer = env->NewDirectByteBuffer(gradient, (jlong) (sizeof(double) * (this->nvar)));
      this->lowerBoundBuffer = env->NewDirectByteBuffer(lowerBound, (jlong) (sizeof(double) * (this->nvar)));
      this->upperBoundBuffer = env->NewDirectByteBuffer(upperBound, (jlong) (sizeof(double) * (this->nvar)));
      this->lowerBoundABuffer = env->NewDirectByteBuffer(lowerBoundA, (jlong) (sizeof(double) * (this->nvar)));
      this->upperBoundABuffer = env->NewDirectByteBuffer(upperBoundA, (jlong) (sizeof(double) * (this->nvar)));
   }

   jobject QpOASESSolverHandle::getABuffer()
   {
      return this->ABuffer;
   }

   jobject QpOASESSolverHandle::getXBuffer()
   {
      return this->xBuffer;
   }

   jobject QpOASESSolverHandle::getHessianBuffer()
   {
      return this->hessianBuffer;
   }

   jobject QpOASESSolverHandle::getGradientBuffer()
   {
      return this->gradientBuffer;
   }

   jobject QpOASESSolverHandle::getLowerBoundBuffer()
   {
      return this->lowerBoundBuffer;
   }

   jobject QpOASESSolverHandle::getUpperBoundBuffer()
   {
      return this->upperBoundBuffer;
   }

   jobject QpOASESSolverHandle::getLowerBoundABuffer()
   {
      return this->lowerBoundABuffer;
   }

   jobject QpOASESSolverHandle::getUpperBoundABuffer()
   {
      return this->upperBoundABuffer;
   }

   void QpOASESSolverHandle::deleteBuffers()
   {
      delete this->A;
      delete this->x;
      delete this->hessian;
      delete this->gradient;
      delete this->lowerBound;
      delete this->upperBound;
      delete this->lowerBoundA;
      delete this->upperBoundA;
   }
}