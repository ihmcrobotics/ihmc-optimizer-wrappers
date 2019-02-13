/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 2.0.12
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package us.ihmc.convexOptimization.qpOASES;

public class SymSparseMat extends SymmetricMatrix {
  private long swigCPtr;

  protected SymSparseMat(long cPtr, boolean cMemoryOwn) {
    super(qpOASESJNI.SymSparseMat_SWIGUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  protected static long getCPtr(SymSparseMat obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        qpOASESJNI.delete_SymSparseMat(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

  public SymSparseMat() {
    this(qpOASESJNI.new_SymSparseMat__SWIG_0(), true);
  }

  public SymSparseMat(int nr, int nc, int[] r, int[] c, double[] v) {
    this(qpOASESJNI.new_SymSparseMat__SWIG_1(nr, nc, r, c, v), true);
  }

  public SymSparseMat(int nr, int nc, int ld, double[] v) {
    this(qpOASESJNI.new_SymSparseMat__SWIG_2(nr, nc, ld, v), true);
  }

  public Matrix duplicate() {
    long cPtr = qpOASESJNI.SymSparseMat_duplicate(swigCPtr, this);
    return (cPtr == 0) ? null : new Matrix(cPtr, false);
  }

  public SymmetricMatrix duplicateSym() {
    long cPtr = qpOASESJNI.SymSparseMat_duplicateSym(swigCPtr, this);
    return (cPtr == 0) ? null : new SymmetricMatrix(cPtr, false);
  }

  public returnValue bilinear(Indexlist icols, int xN, double[] x, int xLD, double[] y, int yLD) {
    return returnValue.swigToEnum(qpOASESJNI.SymSparseMat_bilinear(swigCPtr, this, Indexlist.getCPtr(icols), icols, xN, x, xLD, y, yLD));
  }

}
