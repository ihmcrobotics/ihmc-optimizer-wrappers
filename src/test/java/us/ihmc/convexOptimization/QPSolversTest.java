package us.ihmc.convexOptimization;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

public class QPSolversTest {

	@Test
	public void simpleTests() {
		//setup problem
		int nin = 1, neq = 1, nv = 2;
		DMatrixRMaj Q = new DMatrixRMaj(nv, nv, true, 1, 0, 0, 1);
		DMatrixRMaj f = new DMatrixRMaj(nv, 1, true, 1, 0);
		DMatrixRMaj Aeq = new DMatrixRMaj(neq, nv, true, 1, 1);
		DMatrixRMaj beq = new DMatrixRMaj(neq, 1, true, 0);
		DMatrixRMaj Ain = new DMatrixRMaj(nin, nv, true, 2, 1);
		DMatrixRMaj bin = new DMatrixRMaj(nin, 1, true, 0);
		DMatrixRMaj x = new DMatrixRMaj(nv, 1, true, -1, 1);
		DMatrixRMaj lb = new DMatrixRMaj(nv, 1, true,
				Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);
		DMatrixRMaj ub = new DMatrixRMaj(nv, 1, true,
				Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
		DMatrixRMaj solX = new DMatrixRMaj(nv, 1, true, -0.5, 0.5);
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
		DMatrixRMaj AeqNegT = new DMatrixRMaj(Aeq.numCols, Aeq.numRows);
		CommonOps_DDRM.transpose(Aeq, AeqNegT);
		CommonOps_DDRM.scale(-1, AeqNegT);
		DMatrixRMaj AinNegT = new DMatrixRMaj(Ain.numCols, Ain.numRows);
		CommonOps_DDRM.transpose(Ain, AinNegT);
		CommonOps_DDRM.scale(-1, AinNegT);
		x.zero();

		int iter = qprog.solve(Q, f, AeqNegT, beq, AinNegT, bin, x, true);
		System.out.println("QuadProg SwigWrapper xopt=" + x + "iter=" + iter
				+ " optVal=" + solverSwig.getOptVal());
		assertArrayEquals(solX.data, x.data, epsilon);
	}
	
	public static void main(String[] arg)
	{
		(new QPSolversTest()).simpleTests();
	}

}
