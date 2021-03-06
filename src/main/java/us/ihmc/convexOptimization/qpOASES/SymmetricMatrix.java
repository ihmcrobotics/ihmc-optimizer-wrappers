/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 2.0.12
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package us.ihmc.convexOptimization.qpOASES;

public class SymmetricMatrix extends Matrix {
  private long swigCPtr;

  protected SymmetricMatrix(long cPtr, boolean cMemoryOwn) {
    super(qpOASESJNI.SymmetricMatrix_SWIGUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  protected static long getCPtr(SymmetricMatrix obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        qpOASESJNI.delete_SymmetricMatrix(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

  public SymmetricMatrix duplicateSym() {
    long cPtr = qpOASESJNI.SymmetricMatrix_duplicateSym(swigCPtr, this);
    return (cPtr == 0) ? null : new SymmetricMatrix(cPtr, false);
  }

  public returnValue bilinear(Indexlist icols, int xN, double[] x, int xLD, double[] y, int yLD) {
    return returnValue.swigToEnum(qpOASESJNI.SymmetricMatrix_bilinear(swigCPtr, this, Indexlist.getCPtr(icols), icols, xN, x, xLD, y, yLD));
  }

}
