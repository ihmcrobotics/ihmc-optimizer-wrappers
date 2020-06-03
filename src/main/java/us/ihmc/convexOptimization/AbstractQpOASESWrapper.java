package us.ihmc.convexOptimization;

//~--- non-JDK imports --------------------------------------------------------

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

public abstract class AbstractQpOASESWrapper {
	
    protected abstract int solveNative(double[] H, double[] g, double[] A, double[] lb, double[] ub, double[] lbA,
            double[] ubA, int[] nWSR, double[] cputime, double[] x, double[] objVal);

    protected abstract int hotstartNative(double[] H, double[] g, double[] A, double[] lb, double[] ub, double[] lbA,
            double[] ubA, int[] nWSR, double[] cputime, double[] x, double[] objVal);

    protected abstract void initializeNative(int nvar, int ncon);

    /*
     *  minimizex (1/2)x'Qx+f'x
     *  s.t.
     *  Ain x <= bin
     *  Aeq x = beq,
     */

    double[]               cputime = new double[1];
    int[]                  nWSR    = new int[1];
    double[]               objVal  = new double[1];
    private DMatrixRMaj A       = null;
    private DMatrixRMaj lbA     = null;
    private DMatrixRMaj ubA     = null;
    int                    nvar, ncon;
    boolean                coldStart;

    public AbstractQpOASESWrapper() {
        this(1, 1);
    }

    /**
     *
     * @param nvar - hint the number of variables
     * @param ncon - hint the number of constraints
     */
    public AbstractQpOASESWrapper(int nvar, int ncon) {
        initialize(nvar, ncon);
    }

    private void initialize(int nvar, int ncon) {
        if ((this.nvar != nvar) || (this.ncon != ncon)) {
            initializeNative(nvar, ncon);
            this.nvar  = nvar;
            this.ncon  = ncon;
            A          = new DMatrixRMaj(this.ncon, this.nvar);
            lbA        = new DMatrixRMaj(ncon, 1);
            ubA        = new DMatrixRMaj(ncon, 1);
            coldStart  = true;
            cputime[0] = Double.MAX_VALUE;
            nWSR[0]    = Integer.MAX_VALUE;
        }
    }

    public int solve(DMatrixRMaj Q, DMatrixRMaj f, DMatrixRMaj Aeq, DMatrixRMaj beq, DMatrixRMaj Ain,
                     DMatrixRMaj bin, DMatrixRMaj lb, DMatrixRMaj ub, DMatrixRMaj x, boolean initialize) {
        if ((Aeq.numCols != Ain.numCols) || (Aeq.numCols != x.numRows)) {
            throw new RuntimeException("inconsistent constraints");
        }

        if (initialize) {
            coldStart = true;
        }

        initialize(Aeq.numCols, Aeq.numRows + Ain.numRows);
        CommonOps_DDRM.insert(Aeq, A, 0, 0);
        CommonOps_DDRM.insert(Ain, A, Aeq.numRows, 0);
        CommonOps_DDRM.fill(lbA, Double.NEGATIVE_INFINITY);
        CommonOps_DDRM.insert(beq, lbA, 0, 0);
        CommonOps_DDRM.insert(beq, ubA, 0, 0);
        CommonOps_DDRM.insert(bin, ubA, beq.numRows, 0);

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


}
