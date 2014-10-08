
#include <qpOASES.hpp>
#include <iostream>
#include <limits>
#include <jni.h>

extern "C"
{
	qpOASES::SQProblem *qp=NULL;
	qpOASES::Options *options=NULL;
	JNIEXPORT void initializeNative (int nvar, int ncon)
	{
		if(qp) delete qp;
		qp = new qpOASES::SQProblem(nvar, ncon,qpOASES::HST_SEMIDEF);
		//qp = new qpOASES::SQProblem(nvar, ncon,qpOASES::HST_POSDEF);

		if(options==NULL) 
			options = new qpOASES::Options;
		options->setToReliable( );
		//options->setToMPC( );
		options->printLevel = qpOASES::PL_LOW;
		qp->setOptions(*options);
	}

	/*	
	 *	http://www.qpoases.org/doxygen/manual.pdf
	 *
	 *	 min_x 0.5 x' H x + g' x
	 *
	 *	 st lbA <= Ax <= ubA
	 *	     lb <=  x <= ub
	 *
	 */
	JNIEXPORT int hotstartNative(double*H, double* g, double* A, double* lb, double* ub, double* lbA, double* ubA, int* nWSR, double* cputime, double* x)
	{
		if(!qp)
		{
			std::cerr << "OASES::call initializeNative() first" << std::endl;
			return -1;
		}
		int retHotstart=qp->hotstart(H, g, A, lb, ub, lbA, ubA, *nWSR, cputime);
		if(retHotstart!=qpOASES::SUCCESSFUL_RETURN)
		{
			std::cerr << "OASES::hotstart failed: retVal= " << retHotstart << std::endl;
			return retHotstart;
		}
		int retPrimal=qp->getPrimalSolution(x);
		if(retPrimal!=qpOASES::SUCCESSFUL_RETURN)
		{
			std::cerr << "OASES::getPrimalSolution failed: retVal= " << retPrimal<< std::endl;
			return retPrimal;
		}
		return 0;
	}

	JNIEXPORT int solveNative(double*H, double* g, double* A, double* lb, double* ub, double* lbA, double* ubA, int* nWSR, double* cputime, double* x)
	{

		if(!qp)
		{
			std::cerr << "OASES::call initializeNative() first" << std::endl;
			return -1;
		}

		int retInit=qp->init(H, g, A, lb, ub, lbA, ubA, *nWSR, cputime);
		if(retInit!=qpOASES::SUCCESSFUL_RETURN)
		{
			std::cerr << "OASES::init failed: retVal= " << retInit<< std::endl;
			return retInit;
		}

		int retPrimal=qp->getPrimalSolution(x);
		if(retPrimal!=qpOASES::SUCCESSFUL_RETURN)
		{
			std::cerr << "OASES::getPrimalSolution failed: retVal= " << retPrimal<< std::endl;
			return retPrimal;
		}
		return 0;
	}

	JNIEXPORT double getObjVal()
	{
		if(qp)
			return qp->getObjVal();
		else
		{
			return std::numeric_limits<double>::quiet_NaN();
		}
	}
}
