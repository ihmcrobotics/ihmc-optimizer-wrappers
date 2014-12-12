package us.ihmc.convexOptimization;

import static org.junit.Assert.assertArrayEquals;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;

public class qpOASESTest {

	@Test
	public void testQpOASES() {
		//setup problem
		int nin = 1, neq = 1, nv = 2;
		DenseMatrix64F Q = new DenseMatrix64F(nv, nv, true, 1, 0, 0, 1);
		DenseMatrix64F f = new DenseMatrix64F(nv, 1, true, 1, 0);
		DenseMatrix64F Aeq = new DenseMatrix64F(neq, nv, true, 1, 1);
		DenseMatrix64F beq = new DenseMatrix64F(neq, 1, true, 0);
		DenseMatrix64F Ain = new DenseMatrix64F(nin, nv, true, 2, 1);
		DenseMatrix64F bin = new DenseMatrix64F(nin, 1, true, 0);
		DenseMatrix64F x = new DenseMatrix64F(nv, 1, true, -1, 1);
		DenseMatrix64F lb = new DenseMatrix64F(nv, 1, true,
				Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);
		DenseMatrix64F ub = new DenseMatrix64F(nv, 1, true,
				Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
		DenseMatrix64F solX = new DenseMatrix64F(nv, 1, true, -0.5, 0.5);
		double epsilon = 1e-10;

		//qpOASES C wrapper
		QpOASESCWrapper solverC = new QpOASESCWrapper();
		x.zero();
		solverC.solve(Q, f, Aeq, beq, Ain, bin, lb, ub, x, true);
		System.out.println("OASES CWrapper xopt=" + x + "iter="
				+ solverC.getLastWorkingSetChanges() + " optVal="
				+ solverC.getOptVal() + " cputime:"+ solverC.getLastCpuTime());
		assertArrayEquals(solX.data, x.data, epsilon);

		
		//qpOASES Swig wrapper
		QpOASESSwigWrapper solverSwig = new QpOASESSwigWrapper();
		x.zero();
		solverSwig.solve(Q, f, Aeq, beq, Ain, bin, lb, ub, x, true);
		System.out.println("OASES SwigWrapper xopt=" + x + "iter="
				+ solverSwig.getLastWorkingSetChanges() + " optVal="
				+ solverSwig.getOptVal() + " cputime:"+ solverSwig.getLastCpuTime());
		assertArrayEquals(solX.data, x.data, epsilon);

		//QuadProg
		QuadProgWrapper qprog = new QuadProgWrapper();
		DenseMatrix64F AeqNegT = new DenseMatrix64F(Aeq.numCols, Aeq.numRows);
		CommonOps.transpose(Aeq, AeqNegT);
		CommonOps.scale(-1, AeqNegT);
		DenseMatrix64F AinNegT = new DenseMatrix64F(Ain.numCols, Ain.numRows);
		CommonOps.transpose(Ain, AinNegT);
		CommonOps.scale(-1, AinNegT);
		x.zero();

		int iter = qprog.solve(Q, f, AeqNegT, beq, AinNegT, bin, x, true);
		System.out.println("QuadProg SwigWrapper xopt=" + x + "iter=" + iter
				+ " optVal=" + solverSwig.getOptVal());
		assertArrayEquals(solX.data, x.data, epsilon);
	}
	
	public static void main(String[] arg)
	{
		(new qpOASESTest()).testQpOASES();
	}

}
