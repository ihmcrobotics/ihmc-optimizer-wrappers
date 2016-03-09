#include "QpOASESSolverHandle.h"

namespace ihmc_optimizer_wrappers
{
   QpOASESSolverHandle::QpOASESSolverHandle()
   {
      this->isInitialized = false;
   }

   QpOASESSolverHandle::~QpOASESSolverHandle()
   {
      delete this->sqProblem;

      // Not sure if this needs to be done, I'll play with this later.
      delete this->options;

      this->deleteBuffers();
   }

   void QpOASESSolverHandle::setupQPOASES(int nvar, int ncon)
   {
      this->nvar = nvar;
      this->ncon = ncon;

      this->sqProblem = new qpOASES::SQProblem(this->nvar, this->ncon,
                                               static_cast<qpOASES::HessianType>(this->hessianTypeOrdinal));
      this->options = new qpOASES::Options;

      switch (this->solverOptionOrdinal)
      {
         case RELIABLE_OPTION_ORDINAL:
            this->options->setToReliable();
            break;
         case FAST_OPTION_ORDINAL:
            this->options->setToFast();
            break;
         case MPC_OPTION_ORDINAL:
            this->options->setToMPC();
            break;
         case DEFAULT_OPTION_ORDINAL:
         default:
            this->options->setToDefault();
      }

      this->options->printLevel = qpOASES::PL_LOW;

      this->sqProblem->setOptions(*(this->options));

      this->isInitialized = true;
   }

   void QpOASESSolverHandle::setupQuadraticProgramBuffers(JNIEnv *env)
   {
      this->A = new double[this->nvar * this->ncon];
      this->x = new double[this->nvar];
      this->H = new double[this->nvar * this->nvar];
      this->g = new double[this->nvar];
      this->lb = new double[this->nvar];
      this->ub = new double[this->nvar];
      this->lbA = new double[this->nvar];
      this->ubA = new double[this->nvar];

      this->ABuffer = env->NewDirectByteBuffer(A, (jlong) (sizeof(double) * (this->nvar * this->ncon)));
      this->xBuffer = env->NewDirectByteBuffer(x, (jlong) (sizeof(double) * (this->nvar)));
      this->HBuffer = env->NewDirectByteBuffer(H, (jlong) (sizeof(double) * (this->nvar * this->nvar)));
      this->gBuffer = env->NewDirectByteBuffer(g, (jlong) (sizeof(double) * (this->nvar)));
      this->lbBuffer = env->NewDirectByteBuffer(lb, (jlong) (sizeof(double) * (this->nvar)));
      this->ubBuffer = env->NewDirectByteBuffer(ub, (jlong) (sizeof(double) * (this->nvar)));
      this->lbABuffer = env->NewDirectByteBuffer(lbA, (jlong) (sizeof(double) * (this->nvar)));
      this->ubABuffer = env->NewDirectByteBuffer(ubA, (jlong) (sizeof(double) * (this->nvar)));
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
      return this->HBuffer;
   }

   jobject QpOASESSolverHandle::getGradientBuffer()
   {
      return this->gBuffer;
   }

   jobject QpOASESSolverHandle::getLowerBoundBuffer()
   {
      return this->lbBuffer;
   }

   jobject QpOASESSolverHandle::getUpperBoundBuffer()
   {
      return this->ubBuffer;
   }

   jobject QpOASESSolverHandle::getLowerBoundABuffer()
   {
      return this->lbABuffer;
   }

   jobject QpOASESSolverHandle::getUpperBoundABuffer()
   {
      return this->ubABuffer;
   }

   int QpOASESSolverHandle::getNumberOfWorkingSetChanges()
   {
      return this->numberOfWorkingSetChanges;
   }

   double QpOASESSolverHandle::getCPUTime()
   {
      return this->cpuTime;
   }

   double QpOASESSolverHandle::getObjValue()
   {
      return this->objValue;
   }

   void QpOASESSolverHandle::deleteBuffers()
   {
      delete this->A;
      delete this->x;
      delete this->H;
      delete this->g;
      delete this->lb;
      delete this->ub;
      delete this->lbA;
      delete this->ubA;
   }

   int QpOASESSolverHandle::hotstart()
   {
      if (!this->sqProblem)
      {
         std::cerr << "OASES::call initializeNative() first" << std::endl;
         return -1;
      }

      int qpOASESHotstartRet = this->sqProblem->hotstart(H, g, A, lb, ub, lbA, ubA, this->numberOfWorkingSetChanges,
                                                         &(this->cpuTime));
      if (qpOASESHotstartRet != qpOASES::SUCCESSFUL_RETURN)
      {
         std::cerr << "OASES::hotstart failed: retVal= " << qpOASESHotstartRet << std::endl;
         return qpOASESHotstartRet;
      }

      int qpOASESPrimalRet = this->sqProblem->getPrimalSolution(x);
      if (qpOASESPrimalRet != qpOASES::SUCCESSFUL_RETURN)
      {
         std::cerr << "OASES::getPrimalSolution failed: retVal= " << qpOASESPrimalRet << std::endl;
         return qpOASESPrimalRet;
      }

      this->objValue = this->sqProblem->getObjVal();

      return 0;
   }

   int QpOASESSolverHandle::solve()
   {
      if (!this->sqProblem)
      {
         std::cerr << "OASES::call initializeNative() first" << std::endl;
         return -1;
      }

      int qpOASESInitRet = this->sqProblem->init(H, g, A, lb, ub, lbA, ubA, this->numberOfWorkingSetChanges,
                                                 &(this->cpuTime));
      if (qpOASESInitRet != qpOASES::SUCCESSFUL_RETURN)
      {
         std::cerr << "OASES::init failed: retVal= " << qpOASESInitRet << std::endl;
         return qpOASESInitRet;
      }

      int qpOASESPrimalRet = this->sqProblem->getPrimalSolution(x);
      if (qpOASESPrimalRet != qpOASES::SUCCESSFUL_RETURN)
      {
         std::cerr << "OASES::getPrimalSolution failed: retVal= " << qpOASESPrimalRet << std::endl;
         return qpOASESPrimalRet;
      }

      this->objValue = this->sqProblem->getObjVal();

      return 0;
   }

   void QpOASESSolverHandle::setNumberOfWorkingSetChanges(int numberOfWorkingSetChanges)
   {
      this->numberOfWorkingSetChanges = numberOfWorkingSetChanges;
   }

   void QpOASESSolverHandle::setCPUTime(double cpuTime)
   {
      this->cpuTime = cpuTime;
   }

   void QpOASESSolverHandle::setObjValue(double objValue)
   {
      this->objValue = objValue;
   }

   void QpOASESSolverHandle::setHessianTypeOrdinal(int hessianTypeOrdinal)
   {
      this->hessianTypeOrdinal = hessianTypeOrdinal;
   }

   void QpOASESSolverHandle::setSolverOptionOrdinal(int solverOptionOrdinal)
   {
      this->solverOptionOrdinal = solverOptionOrdinal;
   }

   bool QpOASESSolverHandle::isSolverInitialized()
   {
      return this->isInitialized;
   }

   void QpOASESSolverHandle::reinitializeSolverOptions()
   {
      this->sqProblem->setHessianType(static_cast<qpOASES::HessianType>(this->hessianTypeOrdinal));

      switch (this->solverOptionOrdinal)
      {
         case RELIABLE_OPTION_ORDINAL:
            this->options->setToReliable();
            break;
         case FAST_OPTION_ORDINAL:
            this->options->setToFast();
            break;
         case MPC_OPTION_ORDINAL:
            this->options->setToMPC();
            break;
         case DEFAULT_OPTION_ORDINAL:
         default:
            this->options->setToDefault();
      }

      this->options->printLevel = qpOASES::PL_LOW;

      this->sqProblem->setOptions(*(this->options));
   }
}
