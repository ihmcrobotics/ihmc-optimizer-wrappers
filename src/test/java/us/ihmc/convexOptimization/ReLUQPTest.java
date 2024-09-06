package us.ihmc.convexOptimization;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class ReLUQPTest
{

	@Test
	public void simpleTest() {
		double epsilon = 1e-5;

		// Set up problem
		int nx = 5, nc = 6;
		DMatrixRMaj H = new DMatrixRMaj(nx, nx, true, 6.71383, 0.71247, 3.75614, 0.886201, 0.74104, 0.71247, 3.54515, 0.835424, 0.532941, 1.42631, 3.75614, 0.835424, 5.22627, 0.317399, 1.3052, 0.886201, 0.532941, 0.317399, 4.51379, -0.836364, 0.74104, 1.42631, 1.3052, -0.836364, 4.98613);
		DMatrixRMaj g = new DMatrixRMaj(nx, 1, true, 2.78913, 2.34454, 0.846321, 4.11682, -2.9479);
		DMatrixRMaj A = new DMatrixRMaj(nc, nx, true, 0.84794, 0.821944, 0.840257, -0.136093, -0.385084, -0.203127, -0.0350187, -0.70468, 0.239193, -0.105933, 0.629534, -0.56835, 0.762124, -0.437881, -0.547787, 0.368437, 0.900505, 0.282161, 0.572004, -0.624934, -0.447531, -0.166997, 0.813608, -0.747849, 0.52095, 0.112888, -0.660786, 0.793658, -0.00911187, 0.969503);
		DMatrixRMaj lb = new DMatrixRMaj(nc, 1, true, -0.73883, -0.440346, 0.433656, -1.47864, 1.58857, 0.118917);
		DMatrixRMaj ub = new DMatrixRMaj(nc, 1, true, -0.73883, -0.440346, 0.433656, -1.47864, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
		DMatrixRMaj xSolution = new DMatrixRMaj(nx, 1); // Initialize guess to zeros
		DMatrixRMaj xExpected = new DMatrixRMaj(nx, 1, true, -5.1583E-01, -6.9028E-01, 4.5770E-01, -7.6141E-01, 5.7715E-01);

		ReLUQPWrapper reluqp = new ReLUQPWrapper();
		reluqp.setup(H, g, A, lb, ub, false, 1e-4, 1e-4, 4000, 1);
		reluqp.solve();

		xSolution.set(reluqp.getSolution());

		System.out.println("ReLUQP JNA wrapper: solution = " + xSolution +
		                   ", objective value = " + reluqp.getObjectiveValue() +
		                   ", iters = " + reluqp.getIters());
		assertArrayEquals(xExpected.data, xSolution.data, epsilon);
	}
	
	public static void main(String[] arg)
	{
		(new ReLUQPTest()).simpleTest();
	}

}
