package us.ihmc.convexOptimization;

//~--- non-JDK imports --------------------------------------------------------

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
//~--- JDK imports ------------------------------------------------------------

public class QpOASESWrapper {

    /*
     *  minimizex (1/2)x'Qx+f'x
     *  s.t.
     *  Ain x <= bin
     *  Aeq x = beq,
     */
    static {
        NativeLibraryLoader.loadLibraryFromClassPath(
            NativeLibraryLoader.getOSDependentName("OASESConstrainedQPSolver"), QpOASESWrapper.class);
    }

    double[]               cputime = new double[1];
    int[]                  nWSR    = new int[1];
    double[]               objVal  = new double[1];
    private DenseMatrix64F A       = null;
    private DenseMatrix64F lbA     = null;
    private DenseMatrix64F ubA     = null;
    int                    nvar, ncon;
    boolean                coldStart;

    public QpOASESWrapper() {
        this(1, 1);
    }

    /**
     *
     * @param nvar - hint the number of variables
     * @param ncon - hint the number of constraints
     */
    public QpOASESWrapper(int nvar, int ncon) {
        initialize(nvar, ncon);
    }

    private static native void initializeNative(int nvar, int ncon);

    /**
     *
     * min 0.5*x'Hx + g'x
     *
     * st lbA <= Ax <= ubA
     *     lb <= x <= ub
     *
     * matrices are row-major
     *
     * @param nWSR - number of working set re-calculation
     * @param cputime - maximum cputime, null
     * @param x - initial and return variable to be optimized
     * @return returnCode from C-API
     */
    public static native int solveNative(double[] H, double[] g, double[] A, double[] lb, double[] ub, double[] lbA,
            double[] ubA, int[] nWSR, double[] cputime, double[] x, double[] objVal);

    public static native int hotstartNative(double[] H, double[] g, double[] A, double[] lb, double[] ub, double[] lbA,
            double[] ubA, int[] nWSR, double[] cputime, double[] x, double[] objVal);

    private void initialize(int nvar, int ncon) {
        if ((this.nvar != nvar) || (this.ncon != ncon)) {
            initializeNative(nvar, ncon);
            this.nvar  = nvar;
            this.ncon  = ncon;
            A          = new DenseMatrix64F(this.ncon, this.nvar);
            lbA        = new DenseMatrix64F(ncon, 1);
            ubA        = new DenseMatrix64F(ncon, 1);
            coldStart  = true;
            cputime[0] = Double.MAX_VALUE;
            nWSR[0]    = Integer.MAX_VALUE;
        }
    }

    public int solve(DenseMatrix64F Q, DenseMatrix64F f, DenseMatrix64F Aeq, DenseMatrix64F beq, DenseMatrix64F Ain,
                     DenseMatrix64F bin, DenseMatrix64F lb, DenseMatrix64F ub, DenseMatrix64F x, boolean initialize) {
        if ((Aeq.numCols != Ain.numCols) || (Aeq.numCols != x.numRows)) {
            throw new RuntimeException("inconsistent constraints");
        }

        if (initialize) {
            coldStart = true;
        }

        initialize(Aeq.numCols, Aeq.numRows + Ain.numRows);
        CommonOps.insert(Aeq, A, 0, 0);
        CommonOps.insert(Ain, A, Aeq.numRows, 0);
        CommonOps.fill(lbA, Double.NEGATIVE_INFINITY);
        CommonOps.insert(beq, lbA, 0, 0);
        CommonOps.insert(beq, ubA, 0, 0);
        CommonOps.insert(bin, ubA, beq.numRows, 0);

        // see C-code for retCode explanation
        double[] lbData  = (lb == null)
                           ? null
                           : lb.getData(),
                 ubData  = (ub == null)
                           ? null
                           : ub.getData();
        int      retCode = 0;

        if (coldStart) {
            retCode = solveNative(Q.getData(), f.getData(), A.getData(), lbData, ubData, lbA.getData(), ubA.getData(),
                                  nWSR, cputime, x.getData(), objVal);
        } else {
            retCode = hotstartNative(Q.getData(), f.getData(), A.getData(), lbData, ubData, lbA.getData(),
                                     ubA.getData(), nWSR, cputime, x.getData(), objVal);
        }

        coldStart = false;

        return retCode;
    }

    public double getLastCpuTime() {
        return cputime[0];
    }

    public void setMaxCpuTime(double maxCpuTime) {
        cputime[0] = maxCpuTime;
    }

    public int getLastWorkingSetChanges() {
        return nWSR[0];
    }

    public void setMaxWorkingSetChanges(int maxWorkingSetChanges) {
        nWSR[0] = maxWorkingSetChanges;
    }

    public boolean isColdStart() {
        return coldStart;
    }

    public double getOptVal() {
        return objVal[0];
    }

    public static void main(String[] arg) {
        int            nin    = 1,
                       neq    = 1,
                       nv     = 2;
        DenseMatrix64F Q      = new DenseMatrix64F(nv, nv, true, 1, 0, 0, 1);
        DenseMatrix64F f      = new DenseMatrix64F(nv, 1, true, 1, 0);
        DenseMatrix64F Aeq    = new DenseMatrix64F(neq, nv, true, 1, 1);
        DenseMatrix64F beq    = new DenseMatrix64F(neq, 1, true, 0);
        DenseMatrix64F Ain    = new DenseMatrix64F(nin, nv, true, 2, 1);
        DenseMatrix64F bin    = new DenseMatrix64F(nin, 1, true, 0);
        DenseMatrix64F x      = new DenseMatrix64F(nv, 1, true, -1, 1);
        QpOASESWrapper solver = new QpOASESWrapper();
        int            iter   = solver.solve(Q, f, Aeq, beq, Ain, bin, null, null, x, true);

        System.out.println("xopt=" + x + "iter=" + iter + " optVal=" + solver.getOptVal());
    }
}
