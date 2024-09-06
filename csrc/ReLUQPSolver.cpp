#include <reluqp.hpp>
#include <cstdlib>
#include <iostream>

#ifdef _WIN32
#define CLASS_DECLSPEC    __declspec(dllexport)
#else
#define CLASS_DECLSPEC
#endif

extern "C" {

ReLU_QP *solver = NULL;

CLASS_DECLSPEC void setupC(int nx, int nc,
                          double *H_data, double *g_data, double *A_data, double *l_data, double *u_data,
                          bool verbose, double epsPrimal, double epsDual, int maxIters, int itersBetweenChecks)
{
    if (solver) delete solver;
    Settings settings;
    settings.verbose = verbose;
    settings.eps_primal = epsPrimal;
    settings.eps_dual = epsDual;
    settings.max_iters = maxIters;
    settings.iters_between_checks = itersBetweenChecks;
    MatrixXd H = Map<Matrix<double, -1, -1, RowMajor>>(H_data, nx, nx);
    MatrixXd g = Map<Matrix<double, -1, -1, RowMajor>>(g_data, nx, 1);
    MatrixXd A = Map<Matrix<double, -1, -1, RowMajor>>(A_data, nc, nx);
    MatrixXd l = Map<Matrix<double, -1, -1, RowMajor>>(l_data, nc, 1);
    MatrixXd u = Map<Matrix<double, -1, -1, RowMajor>>(u_data, nc, 1);
    solver = new ReLU_QP(H, g, A, l, u, settings);
}

CLASS_DECLSPEC int updateC(double *g_new_data, double *lb_new_data, double *ub_new_data)
{
    if (!solver)
    {
        std::cerr << "ReLUQP error in updateC: call setupNative() first" << std::endl;
        return -1;
    }
    MatrixXd g_new = Map<Matrix<double, -1, -1, RowMajor>>(g_new_data, solver->qp.nx, 1);
    MatrixXd lb_new = Map<Matrix<double, -1, -1, RowMajor>>(lb_new_data, solver->qp.nc, 1);
    MatrixXd ub_new = Map<Matrix<double, -1, -1, RowMajor>>(ub_new_data, solver->qp.nc, 1);
    ReLUQPError retUpdate = solver->update(g_new, lb_new, ub_new);
    if (retUpdate != SUCCESS)
    {
        std::cerr << "ReLUQP error in updateC: update failed: retVal= " << retUpdate << std::endl;
        return retUpdate;
    }
    return SUCCESS;
}

CLASS_DECLSPEC int solveC(double *x, double *objVal, int *iters)
{
    if (!solver)
    {
        std::cerr << "ReLUQP error in solveC: call setupNative() first" << std::endl;
        return -1;
    }
    Results results = solver->solve();
    if (results.info.iters == solver->settings.max_iters)
    {
        std::clog << "ReLUQP warning in solveC: max iters reached" << std::endl;
    }
    for (int i=0; i<solver->qp.nx; i++)
    {
        x[i] = results.x(i, 0);
    }
    *objVal = results.info.obj_val;
    *iters = results.info.iters;
    return SUCCESS;
}

} // extern "C"