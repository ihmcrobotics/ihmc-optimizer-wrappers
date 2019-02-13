package us.ihmc.convexOptimization;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.DoubleBuffer;

import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

public class QpOASESJNISolver extends AbstractQpOASESWrapper
{
   static
   {
      NativeLibraryLoader.loadLibrary("us.ihmc.convexOptimization", "IHMCOASESConstrainedQPSolver_rel");
   }

   // We typically use HST_SEMIDEF
   public enum QPOASESHessianType
   {
      HST_ZERO, HST_IDENTITY, HST_POSDEF, HST_POSDEF_NULLSPACE, HST_SEMIDEF, HST_INDEF, HST_UNKNOWN
   }

   // We typically use RELIABLE
   public enum QPOASESSolverOptions
   {
      RELIABLE, FAST, MPC, DEFAULT
   }

   private long solverID;

   private DoubleBuffer A_Buffer;
   private DoubleBuffer x_Buffer;
   private DoubleBuffer H_Buffer;
   private DoubleBuffer g_Buffer;
   private DoubleBuffer lb_Buffer;
   private DoubleBuffer ub_Buffer;
   private DoubleBuffer lbA_Buffer;
   private DoubleBuffer ubA_Buffer;

   public QpOASESJNISolver(int nvar, int ncon, QPOASESHessianType hessianType, QPOASESSolverOptions solverOption)
   {
      super(nvar, ncon);
      setHessianTypeOrdinal(hessianType.ordinal(), this.solverID);
      setSolverOptionOrdinal(solverOption.ordinal(), this.solverID);
      this.reinitializeSolverOptions(this.solverID);
   }

   public QpOASESJNISolver(int nvar, int ncon)
   {
      this(nvar, ncon, QPOASESHessianType.HST_SEMIDEF, QPOASESSolverOptions.RELIABLE);
   }

   public QpOASESJNISolver()
   {
      this(1, 1);
   }

   private native int solveJNI(int numberOfWorkingSetChanges, double cpuTime, long solverID);

   private native int hotstartJNI(int numberOfWorkingSetChanges, double cpuTime, long solverID);

   private native void initializeJNI(int nvar, int ncon, long solverID);

   private native long createSolver();

   private native void setHessianTypeOrdinal(int hessianTypeOrdinal, long solverID);

   private native void setSolverOptionOrdinal(int solverOptionOrdinal, long solverID);

   private native ByteBuffer get_A_Buffer(long solverID);

   private native ByteBuffer get_x_Buffer(long solverID);

   private native ByteBuffer get_H_Buffer(long solverID);

   private native ByteBuffer get_g_Buffer(long solverID);

   private native ByteBuffer get_lb_Buffer(long solverID);

   private native ByteBuffer get_ub_Buffer(long solverID);

   private native ByteBuffer get_lbA_Buffer(long solverID);

   private native ByteBuffer get_ubA_Buffer(long solverID);

   private native int getNumberOfWorkingSetChangesFromNative(long solverID);

   private native double getCPUTimeFromNative(long solverID);

   private native double getObjValueFromNative(long solverID);

   private native void reinitializeSolverOptions(long solverID);

   @Override protected int solveNative(double[] H, double[] g, double[] A, double[] lb, double[] ub, double[] lbA, double[] ubA, int[] nWSR, double[] cputime,
         double[] x, double[] objVal)
   {
      updateBuffers(H, g, A, lb, ub, lbA, ubA, x);
      int ret = this.solveJNI(nWSR[0], cputime[0], this.solverID);
      updateArrays(H, g, A, lb, ub, lbA, ubA, x);
      //      System.out.println("cputimeC:" + cputime[0]);
      nWSR[0] = getNumberOfWorkingSetChangesFromNative(this.solverID);
      cputime[0] = getCPUTimeFromNative(this.solverID);
      objVal[0] = getObjValueFromNative(this.solverID);
      return ret;
   }

   @Override protected int hotstartNative(double[] H, double[] g, double[] A, double[] lb, double[] ub, double[] lbA, double[] ubA, int[] nWSR,
         double[] cputime, double[] x, double[] objVal)
   {
      updateBuffers(H, g, A, lb, ub, lbA, ubA, x);
      int ret = this.hotstartJNI(nWSR[0], cputime[0], this.solverID);
      updateArrays(H, g, A, lb, ub, lbA, ubA, x);
      //      System.out.println("cputimeC:" + cputime[0]);
      nWSR[0] = getNumberOfWorkingSetChangesFromNative(this.solverID);
      cputime[0] = getCPUTimeFromNative(this.solverID);
      objVal[0] = getObjValueFromNative(this.solverID);
      return ret;
   }

   @Override protected void initializeNative(int nvar, int ncon)
   {
      this.solverID = this.createSolver();
      this.initializeJNI(nvar, ncon, this.solverID);
      setupBuffers();
   }

   private void setupBuffers()
   {
      ByteBuffer A_Buffer_asByteBuffer = this.get_A_Buffer(this.solverID);
      A_Buffer_asByteBuffer.order(ByteOrder.nativeOrder());
      this.A_Buffer = A_Buffer_asByteBuffer.asDoubleBuffer();

      ByteBuffer x_Buffer_asByteBuffer = this.get_x_Buffer(this.solverID);
      x_Buffer_asByteBuffer.order(ByteOrder.nativeOrder());
      this.x_Buffer = x_Buffer_asByteBuffer.asDoubleBuffer();

      ByteBuffer H_Buffer_asByteBuffer = this.get_H_Buffer(this.solverID);
      H_Buffer_asByteBuffer.order(ByteOrder.nativeOrder());
      this.H_Buffer = H_Buffer_asByteBuffer.asDoubleBuffer();

      ByteBuffer g_Buffer_asByteBuffer = this.get_g_Buffer(this.solverID);
      g_Buffer_asByteBuffer.order(ByteOrder.nativeOrder());
      this.g_Buffer = g_Buffer_asByteBuffer.asDoubleBuffer();

      ByteBuffer lb_Buffer_asByteBuffer = this.get_lb_Buffer(this.solverID);
      lb_Buffer_asByteBuffer.order(ByteOrder.nativeOrder());
      this.lb_Buffer = lb_Buffer_asByteBuffer.asDoubleBuffer();

      ByteBuffer ub_Buffer_asByteBuffer = this.get_ub_Buffer(this.solverID);
      ub_Buffer_asByteBuffer.order(ByteOrder.nativeOrder());
      this.ub_Buffer = ub_Buffer_asByteBuffer.asDoubleBuffer();

      ByteBuffer lbA_Buffer_asByteBuffer = this.get_lbA_Buffer(this.solverID);
      lbA_Buffer_asByteBuffer.order(ByteOrder.nativeOrder());
      this.lbA_Buffer = lbA_Buffer_asByteBuffer.asDoubleBuffer();

      ByteBuffer ubA_Buffer_asByteBuffer = this.get_ubA_Buffer(this.solverID);
      ubA_Buffer_asByteBuffer.order(ByteOrder.nativeOrder());
      this.ubA_Buffer = ubA_Buffer_asByteBuffer.asDoubleBuffer();
   }

   private void updateBuffers(double[] H, double[] g, double[] A, double[] lb, double[] ub, double[] lbA, double[] ubA, double[] x)
   {
      H_Buffer.clear();
      H_Buffer.put(H);

      g_Buffer.clear();
      g_Buffer.put(g);

      A_Buffer.clear();
      A_Buffer.put(A);

      x_Buffer.clear();
      x_Buffer.put(x);

      lb_Buffer.clear();
      lb_Buffer.put(lb);

      ub_Buffer.clear();
      ub_Buffer.put(ub);

      lbA_Buffer.clear();
      lbA_Buffer.put(lbA);

      ubA_Buffer.clear();
      ubA_Buffer.put(ubA);
   }

   private void updateArrays(double[] H, double[] g, double[] A, double[] lb, double[] ub, double[] lbA, double[] ubA, double[] x)
   {
      H_Buffer.clear();
      g_Buffer.clear();
      A_Buffer.clear();
      x_Buffer.clear();
      lb_Buffer.clear();
      ub_Buffer.clear();
      lbA_Buffer.clear();
      ubA_Buffer.clear();

      H_Buffer.get(H);
      g_Buffer.get(g);
      A_Buffer.get(A);
      x_Buffer.get(x);
      lb_Buffer.get(lb);
      ub_Buffer.get(ub);
      lbA_Buffer.get(lbA);
      ubA_Buffer.get(ubA);
   }

}
