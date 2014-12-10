package us.ihmc.convexOptimization;

public class QpOASESCWrapper extends AbstractQpOASESWrapper{
	 static {
	        NativeLibraryLoader.loadJNALibraryFromClassPath("us.ihmc.convexOptimization",
	            NativeLibraryLoader.getOSDependentName("OASESConstrainedQPSolver"), QpOASESCWrapper.class);
	    }

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
    public static native int solveC(double[] H, double[] g, double[] A, double[] lb, double[] ub, double[] lbA,
            double[] ubA, int[] nWSR, double[] cputime, double[] x, double[] objVal);

    public static native int hotstartC(double[] H, double[] g, double[] A, double[] lb, double[] ub, double[] lbA,
            double[] ubA, int[] nWSR, double[] cputime, double[] x, double[] objVal);

    private static native void initializeC(int nvar, int ncon);

	@Override
	protected int solveNative(double[] H, double[] g, double[] A, double[] lb,
			double[] ub, double[] lbA, double[] ubA, int[] nWSR,
			double[] cputime, double[] x, double[] objVal) {
		int ret = solveC(H, g, A, lb, ub, lbA, ubA, nWSR, cputime, x, objVal);
		System.out.println("cputimeC:"+cputime[0]);
		return ret;
	}

	@Override
	protected int hotstartNative(double[] H, double[] g, double[] A,
			double[] lb, double[] ub, double[] lbA, double[] ubA, int[] nWSR,
			double[] cputime, double[] x, double[] objVal) {
		return hotstartC(H, g, A, lb, ub, lbA, ubA, nWSR, cputime, x, objVal);
	}

	@Override
	protected void initializeNative(int nvar, int ncon) {
		initializeC(nvar, ncon);
	}



}
