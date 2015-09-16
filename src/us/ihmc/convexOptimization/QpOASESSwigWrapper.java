package us.ihmc.convexOptimization;

import us.ihmc.convexOptimization.qpOASES.SQProblem;
import us.ihmc.convexOptimization.qpOASES.returnValue;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

public class QpOASESSwigWrapper extends AbstractQpOASESWrapper {
	static{
	   NativeLibraryLoader.loadLibrary("us.ihmc.convexOptimization", "qpOASESSwig_rel");
	}

	SQProblem qProblem=null;
	public QpOASESSwigWrapper() {
		super();
	}
	public QpOASESSwigWrapper(int nvar, int ncon) {
		super(nvar, ncon);
	}

	@Override
	protected int solveNative(double[] H, double[] g, double[] A, double[] lb,
			double[] ub, double[] lbA, double[] ubA, int[] nWSR,
			double[] cputime, double[] x, double[] objVal) {

		returnValue ret;
		
		ret = qProblem.init(H, g, A, lb, ub, lbA, ubA, nWSR, cputime);
		if(ret!=returnValue.SUCCESSFUL_RETURN)
			throw new RuntimeException(ret.name());
		
		ret = qProblem.getPrimalSolution(x);
		objVal[0]=qProblem.getObjVal();
		if(ret!=returnValue.SUCCESSFUL_RETURN)
			throw new RuntimeException(ret.name());
		return 0;
	}

	@Override
	protected int hotstartNative(double[] H, double[] g, double[] A,
			double[] lb, double[] ub, double[] lbA, double[] ubA, int[] nWSR,
			double[] cputime, double[] x, double[] objVal) {
		returnValue ret;
		
		ret = qProblem.hotstart(H, g, A, lb, ub, lbA, ubA, nWSR, cputime);
		if(ret!=returnValue.SUCCESSFUL_RETURN)
			throw new RuntimeException(ret.name());
		return 0;
	}

	@Override
	protected void initializeNative(int nvar, int ncon) {
		qProblem = new SQProblem(nvar, ncon);
	}

}
