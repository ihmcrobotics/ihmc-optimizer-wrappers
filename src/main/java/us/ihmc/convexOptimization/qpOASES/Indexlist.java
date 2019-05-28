/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 2.0.12
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package us.ihmc.convexOptimization.qpOASES;

public class Indexlist {
  private long swigCPtr;
  protected boolean swigCMemOwn;

  protected Indexlist(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(Indexlist obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        qpOASESJNI.delete_Indexlist(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public Indexlist() {
    this(qpOASESJNI.new_Indexlist__SWIG_0(), true);
  }

  public Indexlist(int n) {
    this(qpOASESJNI.new_Indexlist__SWIG_1(n), true);
  }

  public Indexlist(Indexlist rhs) {
    this(qpOASESJNI.new_Indexlist__SWIG_2(Indexlist.getCPtr(rhs), rhs), true);
  }

  public returnValue init(int n) {
    return returnValue.swigToEnum(qpOASESJNI.Indexlist_init__SWIG_0(swigCPtr, this, n));
  }

  public returnValue init() {
    return returnValue.swigToEnum(qpOASESJNI.Indexlist_init__SWIG_1(swigCPtr, this));
  }

  public returnValue getNumberArray(SWIGTYPE_p_p_int numberarray) {
    return returnValue.swigToEnum(qpOASESJNI.Indexlist_getNumberArray(swigCPtr, this, SWIGTYPE_p_p_int.getCPtr(numberarray)));
  }

  public returnValue getISortArray(SWIGTYPE_p_p_int iSortArray) {
    return returnValue.swigToEnum(qpOASESJNI.Indexlist_getISortArray(swigCPtr, this, SWIGTYPE_p_p_int.getCPtr(iSortArray)));
  }

  public int getIndex(int givennumber) {
    return qpOASESJNI.Indexlist_getIndex(swigCPtr, this, givennumber);
  }

  public int getNumber(int physicalindex) {
    return qpOASESJNI.Indexlist_getNumber(swigCPtr, this, physicalindex);
  }

  public int getLength() {
    return qpOASESJNI.Indexlist_getLength(swigCPtr, this);
  }

  public int getLastNumber() {
    return qpOASESJNI.Indexlist_getLastNumber(swigCPtr, this);
  }

  public returnValue addNumber(int addnumber) {
    return returnValue.swigToEnum(qpOASESJNI.Indexlist_addNumber(swigCPtr, this, addnumber));
  }

  public returnValue removeNumber(int removenumber) {
    return returnValue.swigToEnum(qpOASESJNI.Indexlist_removeNumber(swigCPtr, this, removenumber));
  }

  public returnValue swapNumbers(int number1, int number2) {
    return returnValue.swigToEnum(qpOASESJNI.Indexlist_swapNumbers(swigCPtr, this, number1, number2));
  }

  public BooleanType isMember(int _number) {
    return BooleanType.swigToEnum(qpOASESJNI.Indexlist_isMember(swigCPtr, this, _number));
  }

}