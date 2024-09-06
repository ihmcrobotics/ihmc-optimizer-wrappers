package us.ihmc.convexOptimization;

import com.sun.jna.Native;

import org.ejml.data.DMatrixRMaj;
import org.ejml.data.DMatrixSparseCSC;
import org.ejml.ops.ConvertDMatrixStruct;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

public class OSQPCWrapper
{
   static
   {
      String library = NativeLibraryLoader.extractLibrary("us.ihmc.convexOptimization", "OSQPSolver_rel");
      Native.register(OSQPCWrapper.class, library);
   }

   public static native int setupC(int nvar, int ncon,
                                   double[] H, int numNonZerosH, int[] rowsH, int[] colRangesH, double[] g,
                                   double[] A, int numNonZerosA, int[] rowsA, int[] colRangesA, double[] lb, double[] ub,
                                   boolean verbose, double epsPrimal, double epsDual, int maxIters, int itersBetweenChecks);

   public static native int updateC(double[] gNew, double[] lbNew, double[] ubNew);

   public static native int solveC(double[] x, double[] objVal);

   private double[] objVal = new double[1];
   private DMatrixRMaj x = null;
   private int nvar;
   private int ncon;

   public OSQPCWrapper(int nVariables, int nConstraints)
   {
      this.nvar = nVariables;
      this.ncon = nConstraints;
      this.x = new DMatrixRMaj(nVariables, 1);
   }

   public int setup(DMatrixRMaj H, DMatrixRMaj g, DMatrixRMaj A, DMatrixRMaj lb, DMatrixRMaj ub,
                    boolean verbose, double epsPrimal, double epsDual, int maxIters, int itersBetweenChecks)
   {
      DMatrixSparseCSC sparseH = ConvertDMatrixStruct.convert(H, (DMatrixSparseCSC)null, 1e-14);
      DMatrixSparseCSC sparseA = ConvertDMatrixStruct.convert(A, (DMatrixSparseCSC)null, 1e-14);
      return setupC(nvar, ncon,
                    sparseH.nz_values, sparseH.nz_length, sparseH.nz_rows, sparseH.col_idx, g.getData(),
                    sparseA.nz_values, sparseA.nz_length, sparseA.nz_rows, sparseA.col_idx, lb.getData(), ub.getData(),
                    verbose, epsPrimal, epsDual, maxIters, itersBetweenChecks);
   }

   public int update(DMatrixRMaj gNew, DMatrixRMaj lbNew, DMatrixRMaj ubNew)
   {
      return updateC(gNew.getData(), lbNew.getData(), ubNew.getData());
   }

   public int solve()
   {
      return solveC(x.getData(), objVal);
   }

   public DMatrixRMaj getSolution()
   {
      return x;
   }

   public double getObjectiveValue()
   {
      return objVal[0];
   }

}
