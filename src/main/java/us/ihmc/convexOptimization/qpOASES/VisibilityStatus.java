/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 2.0.12
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package us.ihmc.convexOptimization.qpOASES;

public enum VisibilityStatus {
  VS_HIDDEN,
  VS_VISIBLE;

  public final int swigValue() {
    return swigValue;
  }

  public static VisibilityStatus swigToEnum(int swigValue) {
    VisibilityStatus[] swigValues = VisibilityStatus.class.getEnumConstants();
    if (swigValue < swigValues.length && swigValue >= 0 && swigValues[swigValue].swigValue == swigValue)
      return swigValues[swigValue];
    for (VisibilityStatus swigEnum : swigValues)
      if (swigEnum.swigValue == swigValue)
        return swigEnum;
    throw new IllegalArgumentException("No enum " + VisibilityStatus.class + " with value " + swigValue);
  }

  @SuppressWarnings("unused")
  private VisibilityStatus() {
    this.swigValue = SwigNext.next++;
  }

  @SuppressWarnings("unused")
  private VisibilityStatus(int swigValue) {
    this.swigValue = swigValue;
    SwigNext.next = swigValue+1;
  }

  @SuppressWarnings("unused")
  private VisibilityStatus(VisibilityStatus swigEnum) {
    this.swigValue = swigEnum.swigValue;
    SwigNext.next = this.swigValue+1;
  }

  private final int swigValue;

  private static class SwigNext {
    private static int next = 0;
  }
}

