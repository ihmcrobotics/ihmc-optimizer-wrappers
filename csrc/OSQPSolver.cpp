#include <osqp/osqp.h>
#include <cstdlib>

#ifdef _WIN32
#define CLASS_DECLSPEC    __declspec(dllexport)
#else
#define CLASS_DECLSPEC
#endif

extern "C" {

OSQPSolver* solver;
OSQPSettings* settings;

CLASS_DECLSPEC int setupC(int nvar, int ncon,
                          double *P_, int P_numnonzero, int *P_i, int *P_p, double *g_, 
                          double *A_, int A_numnonzero, int *A_i, int *A_p, double *lb_, double *ub_,
                          bool verbose, double epsPrimal, double epsDual, int maxIters, int itersBetweenChecks)
{
    /**
     * Populates a Compressed-Column-Sparse matrix from existing arrays
     (just assigns the pointers - no malloc or copying is done)
    * @param  M     Matrix pointer
    * @param  m     First dimension
    * @param  n     Second dimension
    * @param  nzmax Maximum number of nonzero elements
    * @param  x     Vector of data
    * @param  i     Vector of row indices
    * @param  p     Vector of column pointers
    */

    // void csc_set_data(OSQPCscMatrix* M,
    //               OSQPInt        m,
    //               OSQPInt        n,
    //               OSQPInt        nzmax,
    //               OSQPFloat*     x,
    //               OSQPInt*       i,
    //               OSQPInt*       p) {

    OSQPCscMatrix* P = (OSQPCscMatrix*)malloc(sizeof(OSQPCscMatrix));
    OSQPCscMatrix* A = (OSQPCscMatrix*)malloc(sizeof(OSQPCscMatrix));
    settings = (OSQPSettings*)malloc(sizeof(OSQPSettings));

    /* Populate matrices */
    csc_set_data(P, nvar, nvar, P_numnonzero, P_, (long long*)P_i, (long long*)P_p);
    csc_set_data(A, ncon, nvar, A_numnonzero, A_, (long long*)A_i, (long long*)A_p);

    settings->verbose = verbose;
    settings->eps_prim_inf = epsPrimal;
    settings->eps_dual_inf = epsDual;
    settings->max_iter = maxIters;
    settings->check_termination = itersBetweenChecks;
    return (int)osqp_setup(&solver, P, g_, A, lb_, ub_, ncon, nvar, settings);
}

CLASS_DECLSPEC int updateC(double *gNew, double *lbNew, double *ubNew)
{
    return (int)osqp_update_data_vec(solver, gNew, lbNew, ubNew);
}

CLASS_DECLSPEC int solveC(double *x, int nvar, double *objVal)
{
    OSQPInt ret = osqp_solve(solver);
    for (int i=0; i<nvar; i++) {
        x[i] = (double)solver->solution->x[i];
    }
    *objVal = (double)solver->info->obj_val;
    return (int)ret;
}

} // extern "C"