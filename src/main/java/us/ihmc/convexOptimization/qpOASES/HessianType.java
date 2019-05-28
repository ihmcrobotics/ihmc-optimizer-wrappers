/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 2.0.12
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package us.ihmc.convexOptimization.qpOASES;

public enum HessianType {
  HST_ZERO,
  HST_IDENTITY,
  HST_POSDEF,
  HST_POSDEF_NULLSPACE,
  HST_SEMIDEF,
  HST_INDEF,
  HST_UNKNOWN;

  public final int swigValue() {
    return swigValue;
  }

  public static HessianType swigToEnum(int swigValue) {
    HessianType[] swigValues = HessianType.class.getEnumConstants();
    if (swigValue < swigValues.length && swigValue >= 0 && swigValues[swigValue].swigValue == swigValue)
      return swigValues[swigValue];
    for (HessianType swigEnum : swigValues)
      if (swigEnum.swigValue == swigValue)
        return swigEnum;
    throw new IllegalArgumentException("No enum " + HessianType.class + " with value " + swigValue);
  }

  @SuppressWarnings("unused")
  private HessianType() {
    this.swigValue = SwigNext.next++;
  }

  @SuppressWarnings("unused")
  private HessianType(int swigValue) {
    this.swigValue = swigValue;
    SwigNext.next = swigValue+1;
  }

  @SuppressWarnings("unused")
  private HessianType(HessianType swigEnum) {
    this.swigValue = swigEnum.swigValue;
    SwigNext.next = this.swigValue+1;
  }

  private final int swigValue;

  private static class SwigNext {
    private static int next = 0;
  }
}
