package us.ihmc.convexOptimization;

import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

public class QpOASESJNISolver extends AbstractQpOASESWrapper
{
   static
   {
      NativeLibraryLoader.loadLibrary("us.ihmc.convexOptimization", "QPOASESJNIWrapper");
   }

   private final long solverID;

   public QpOASESJNISolver(int nvar, int ncon)
   {
      super(nvar, ncon);
      this.solverID = this.createSolver();
   }

   public QpOASESJNISolver()
   {
      super();
      this.solverID = this.createSolver();
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
   private native int solveJNI(long solverID);

   private native int hotstartJNI(long solverID);

   private native void initializeJNI(int nvar, int ncon, long solverID);

   private native long createSolver();

   @Override protected int solveNative(double[] H, double[] g, double[] A, double[] lb, double[] ub, double[] lbA, double[] ubA, int[] nWSR, double[] cputime,
         double[] x, double[] objVal)
   {
//      int ret = solveC(H, g, A, lb, ub, lbA, ubA, nWSR, cputime, x, objVal);
//      System.out.println("cputimeC:" + cputime[0]);
//      return ret;
      return this.solveJNI(this.solverID);
   }

   @Override protected int hotstartNative(double[] H, double[] g, double[] A, double[] lb, double[] ub, double[] lbA, double[] ubA, int[] nWSR,
         double[] cputime, double[] x, double[] objVal)
   {
//      return hotstartC(H, g, A, lb, ub, lbA, ubA, nWSR, cputime, x, objVal);
      return this.hotstartJNI(this.solverID);
   }

   @Override protected void initializeNative(int nvar, int ncon)
   {
      this.initializeJNI(nvar, ncon, this.solverID);
   }

}
