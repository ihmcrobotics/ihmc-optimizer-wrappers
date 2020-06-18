package us.ihmc.convexOptimization;

//~--- non-JDK imports --------------------------------------------------------

import org.ejml.data.DMatrixRMaj;
//~--- JDK imports ------------------------------------------------------------

import com.sun.jna.Native;

import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

public class QuadProgWrapper {
    static {
       String library = NativeLibraryLoader.extractLibrary("us.ihmc.convexOptimization", "uQuadProg_rel");
       Native.register(QuadProgWrapper.class, library);
    }

    int[]   nIteration = new int[1];
    byte[]  errMsg = new byte[512];
    double  objVal     = 0;
    static int     nVariables, nEqualityConstraints, nInequalityConstraints;
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
            double[] ci0, double[] x, int[] nIteration, byte[] errMsg);

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

    public int solve(DMatrixRMaj Q, DMatrixRMaj f, DMatrixRMaj Aeq, DMatrixRMaj beq, DMatrixRMaj Ain,
                     DMatrixRMaj bin, DMatrixRMaj x, boolean initialize) {
        if ((Aeq.numRows!= Ain.numRows) || (Aeq.numRows!= x.numRows)) {
            throw new RuntimeException("inconsistent constraints");
        }

        if (initialize) {
            coldStart = true;
        }

        initialize(Aeq.numRows, Aeq.numCols, Ain.numCols);
        objVal = solveNative(Q.getData(), f.getData(), Aeq.getData(), beq.getData(), Ain.getData(), bin.getData(),
                             x.getData(), nIteration, errMsg);
        if(nIteration[0]<0)
        	throw new RuntimeException(new String(errMsg));
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
        DMatrixRMaj  Q      = new DMatrixRMaj(nv, nv, true, 1, 0, 0, 1);
        DMatrixRMaj  f      = new DMatrixRMaj(nv, 1, true, 1, 0);
        DMatrixRMaj  Aeq    = new DMatrixRMaj(nv, neq, true, -1, -1);
        DMatrixRMaj  beq    = new DMatrixRMaj(neq, 1, true, 0);
        DMatrixRMaj  Ain    = new DMatrixRMaj(nv, nin, true, -2, -1);
        DMatrixRMaj  bin    = new DMatrixRMaj(nin, 1, true, 0);
        DMatrixRMaj  x      = new DMatrixRMaj(nv, 1, true, 0, 0);
        QuadProgWrapper solver = new QuadProgWrapper();
        int             iter   = solver.solve(Q, f, Aeq, beq, Ain, bin, x, false);

        System.out.println("xopt=" + x + "iter=" + iter + " optVal=" + solver.getObjVal());
    }
}
