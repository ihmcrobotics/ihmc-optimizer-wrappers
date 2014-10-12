package us.ihmc.convexOptimization;

//~--- non-JDK imports --------------------------------------------------------

import org.ejml.data.DenseMatrix64F;
//~--- JDK imports ------------------------------------------------------------

public class QuadProgWrapper {
    static {
        NativeLibraryLoader.loadLibraryFromClassPath(NativeLibraryLoader.getOSDependentName("uQuadProg"),
                QuadProgWrapper.class);
    }

    int[]   nIteration = new int[1];
    double  objVal     = 0;
    int     nVariables, nEqualityConstraints, nInequalityConstraints;
    boolean coldStart;

    public QuadProgWrapper() {
        this(1, 1, 1);
    }

    /**
     *
     * @param nvar - hint the number of variables
     * @param ncon - hint the number of constraints
     */
    public QuadProgWrapper(int nVariables, int nEqualityConstraints, int nInequalityConstraints) {
        initialize(nVariables, nEqualityConstraints, nInequalityConstraints);
    }

    public static native void initializeNative(int nVariables, int nEqualityConstraints, int nIneqalityConstraints);

    /*
     * The problem is in the form:
     *
     * min 0.5 * x G x + g0 x
     * s.t.
     *   CE^T x + ce0 = 0
     *   CI^T x + ci0 >= 0
     *
     * The matrix and vectors dimensions are as follows:
     *    G: n * n
     *       g0: n
     *
     *       CE: n * p
     *    ce0: p
     *
     *     CI: n * m
     *  ci0: m
     *
     *    x: n
     *    
     *    Both EJML and ublas are row-major
     */
    public static native double solveNative(double[] G, double[] g0, double[] CE, double[] ce0, double[] CI,
            double[] ci0, double[] x, int[] nIteration);

    private void initialize(int nVariables, int nEqualityConstraints, int nIneqalityConstraints) {
        if ((this.nVariables != nVariables) || (this.nEqualityConstraints != nEqualityConstraints)
                || (this.nInequalityConstraints != nIneqalityConstraints)) {
            initializeNative(nVariables, nEqualityConstraints, nIneqalityConstraints);
            this.nVariables             = nVariables;
            this.nEqualityConstraints   = nEqualityConstraints;
            this.nInequalityConstraints = nIneqalityConstraints;
            coldStart                   = true;
        }
    }

    public int solve(DenseMatrix64F Q, DenseMatrix64F f, DenseMatrix64F Aeq, DenseMatrix64F beq, DenseMatrix64F Ain,
                     DenseMatrix64F bin, DenseMatrix64F x, boolean initialize) {
        if ((Aeq.numCols != Ain.numCols) || (Aeq.numCols != x.numRows)) {
            throw new RuntimeException("inconsistent constraints");
        }

        if (initialize) {
            coldStart = true;
        }

        initialize(Aeq.numCols, Aeq.numRows, Ain.numRows);
        objVal = solveNative(Q.getData(), f.getData(), Aeq.getData(), beq.getData(), Ain.getData(), bin.getData(),
                             x.getData(), nIteration);
        coldStart = false;

        return nIteration[0];
    }

    public boolean isColdStart() {
        return coldStart;
    }

    public double getObjVal() {
        return objVal;
    }

    public static void main(String[] arg) {
        int             nin    = 1,
                        neq    = 1,
                        nv     = 2;
        DenseMatrix64F  Q      = new DenseMatrix64F(nv, nv, true, 1, 0, 0, 1);
        DenseMatrix64F  f      = new DenseMatrix64F(nv, 1, true, 1, 0);
        DenseMatrix64F  Aeq    = new DenseMatrix64F(neq, nv, true, -1, -1);
        DenseMatrix64F  beq    = new DenseMatrix64F(neq, 1, true, 0);
        DenseMatrix64F  Ain    = new DenseMatrix64F(nin, nv, true, -2, -1);
        DenseMatrix64F  bin    = new DenseMatrix64F(nin, 1, true, 0);
        DenseMatrix64F  x      = new DenseMatrix64F(nv, 1, true, 0, 0);
        QuadProgWrapper solver = new QuadProgWrapper();
        int             iter   = solver.solve(Q, f, Aeq, beq, Ain, bin, x, false);

        System.out.println("xopt=" + x + "iter=" + iter + " optVal=" + solver.getObjVal());
    }
}
