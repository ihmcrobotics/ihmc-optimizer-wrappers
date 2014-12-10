package us.ihmc.convexOptimization;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

public class qpOASESTest {
	
	@Test
	public void testQpOASES() {
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
	        DenseMatrix64F lb 	  = new DenseMatrix64F(nv, 1,true, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);
	        DenseMatrix64F ub 	  = new DenseMatrix64F(nv, 1,true, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
	        int iter=0;
	        
	        
	        
	        QpOASESCWrapper solverC = new QpOASESCWrapper();
	        iter   = solverC.solve(Q, f, Aeq, beq, Ain, bin, null, null, x, true);
	        System.out.println("OASES CWrapper xopt=" + x + "iter=" + iter + " optVal=" + solverC.getOptVal());
	        
	        QpOASESSwigWrapper solverSwig = new QpOASESSwigWrapper();
	        iter   = solverSwig.solve(Q, f, Aeq, beq, Ain, bin, lb, ub, x, true);
	        System.out.println("OASES SwigWrapper xopt=" + x + "iter=" + iter + " optVal=" + solverSwig.getOptVal());
	        
	        
	}

}
