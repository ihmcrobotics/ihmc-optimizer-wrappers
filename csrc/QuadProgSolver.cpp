
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include "uQuadProg++.hh"
#include <iostream>
#include <limits>
#include <cstring>
#include <cmath>
#ifdef _WIN32
#define CLASS_DECLSPEC    __declspec(dllexport)
#else
#define CLASS_DECLSPEC
#endif

extern "C"
{

	/*
		The problem is in the form:

		min 0.5 * x G x + g0 x
		s.t.
			CE^T x + ce0 = 0
			CI^T x + ci0 >= 0
			 
		 The matrix and vectors dimensions are as follows:
			 G: n * n
				g0: n
						
				CE: n * p
			 ce0: p
						
			  CI: n * m
		   ci0: m

			 x: n
	*/

	namespace ublas = boost::numeric::ublas;
	ublas::matrix<double> G, CE, CI;
	ublas::vector<double> g0, ce0, ci0, x;

	int nvar=-1, ne=-1, ni=-1;

	CLASS_DECLSPEC void initializeNative (int _nvar, int _ne, int _ni)
	{
		nvar = _nvar;
		ne = _ne;
		ni = _ni;
		std::cout << "Initialize QuadProg (nvar ne ni) = " << nvar << " " << ne << " " << ni << std::endl;

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
	void assignMatrix(ublas::matrix<double>& m, int s1, int s2, double data[])
	{
		m.resize(s1 ,s2);
		for(int i=0;i<s1*s2;i++)
			m.data()[i]=data[i];
	}

	void assignVector(ublas::vector<double>& v, int s, double data[])
	{
		v.resize(s);
		for(int i=0;i<s;i++)
			v.data()[i]=data[i];
	}


	CLASS_DECLSPEC double solveNative(double* _G, double* _g0, double* _CE, double* _ce0, double* _CI, double* _ci0, double* _x, int* iter, char* errMsg)
	{

		if(nvar<0 || ne<0 || ni<0)
		{
			std::cerr << "uQuadProg++::call initializeNative() first" << std::endl;
			*iter=-1;
			return 0;
		}   
		     

		assignMatrix(G, nvar, nvar, _G);
		assignVector(g0, nvar, _g0);
		assignMatrix(CE, nvar, ne, _CE);
		assignVector(ce0, ne, _ce0);
		assignMatrix(CI, nvar, ni, _CI);
		assignVector(ci0, ni, _ci0);
		assignVector(x, nvar, _x);

#if 0
		std::cout << "G" << G << std::endl;
		std::cout << "g0" << g0 << std::endl;
		std::cout << "CE" << CE << std::endl;
		std::cout << "ce0" << ce0 << std::endl;
		std::cout << "CI" << CI << std::endl;
		std::cout << "ci0" << ci0 << std::endl;
		std::cout << "x" << x << std::endl;
#endif
		double ret=NAN;
		try{
			ret=uQuadProgPP::solve_quadprog(G, g0, CE, ce0, CI, ci0, x, *iter);
		}
		catch(const std::runtime_error& e)
		{
			strncpy(errMsg, e.what(), 512);
			*iter=-1;
			return NAN;
		}

		for(int i=0;i<x.size();i++)
				_x[i] = x.data()[i];
		return ret;
	}

}
