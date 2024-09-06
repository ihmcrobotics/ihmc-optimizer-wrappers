package us.ihmc.convexOptimization;

import com.sun.jna.Native;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

public class ReLUQPWrapper
{
   static
   {
      String library = NativeLibraryLoader.extractLibrary("us.ihmc.convexOptimization", "ReLUQPSolver_rel");
      Native.register(ReLUQPWrapper.class, library);
   }

   public static native void setupC(int nx, int nc,
                                   double[] H, double[] g, double[] A, double[] l, double[] u,
                                   boolean verbose, double epsPrimal, double epsDual, int maxIters, int itersBetweenChecks);

   public static native int updateC(double[] gNew, double[] lbNew, double[] ubNew);

   public static native int solveC(double[] x, double[] objVal, int[] iters);

   private double[] objVal = new double[1]; // Objective value
   private int[] iters = new int[1];
   private DMatrixRMaj x = null; // Solution

   public ReLUQPWrapper() {}

   public void setup(DMatrixRMaj H, DMatrixRMaj g, DMatrixRMaj A, DMatrixRMaj lb, DMatrixRMaj ub,
                    boolean verbose, double epsPrimal, double epsDual, int maxIters, int itersBetweenChecks)
   {
      int nx = H.getNumRows();
      int nc = A.getNumRows();
      x = new DMatrixRMaj(nx, 1);
      setupC(nx, nc,
             H.getData(), g.getData(), A.getData(), lb.getData(), ub.getData(),
             verbose, epsPrimal, epsDual, maxIters, itersBetweenChecks);
   }

   public int update(DMatrixRMaj gNew, DMatrixRMaj lbNew, DMatrixRMaj ubNew)
   {
      return updateC(gNew.getData(), lbNew.getData(), ubNew.getData());
   }

   public int solve()
   {
      if (x == null)
      {
         System.err.println("Error in ReLUQPWrapper::solve(): call setup() first");
         return -1;
      }
      return solveC(x.getData(), objVal, iters);
   }

   public DMatrixRMaj getSolution()
   {
      return x;
   }

   public double getObjectiveValue()
   {
      return objVal[0];
   }

   public int getIters()
   {
      return iters[0];
   }

}
