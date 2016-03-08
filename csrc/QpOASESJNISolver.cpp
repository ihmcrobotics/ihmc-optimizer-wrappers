/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
#include "QpOASESSolverHandle.h"
/* Header for class us_ihmc_convexOptimization_QpOASESJNISolver */

#ifndef _Included_us_ihmc_convexOptimization_QpOASESJNISolver
#define _Included_us_ihmc_convexOptimization_QpOASESJNISolver
#ifdef __cplusplus
extern "C" {
#endif

/*
 * Class:     us_ihmc_convexOptimization_QpOASESJNISolver
 * Method:    solveJNI
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_us_ihmc_convexOptimization_QpOASESJNISolver_solveJNI
        (JNIEnv *env, jobject object, jlong solverId)
{
   ihmc_optimizer_wrappers::QpOASESSolverHandle *solverHandle = (ihmc_optimizer_wrappers::QpOASESSolverHandle *) solverId;
}

/*
 * Class:     us_ihmc_convexOptimization_QpOASESJNISolver
 * Method:    hotstartJNI
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_us_ihmc_convexOptimization_QpOASESJNISolver_hotstartJNI
        (JNIEnv *env, jobject object, jlong solverId)
{
   ihmc_optimizer_wrappers::QpOASESSolverHandle *solverHandle = (ihmc_optimizer_wrappers::QpOASESSolverHandle *) solverId;
}
/*
 * Class:     us_ihmc_convexOptimization_QpOASESJNISolver
 * Method:    initializeJNI
 * Signature: (IIJ)V
 */
JNIEXPORT void JNICALL Java_us_ihmc_convexOptimization_QpOASESJNISolver_initializeJNI
        (JNIEnv *env, jobject object, jint nvar, jint ncon, jlong solverId)
{
   ihmc_optimizer_wrappers::QpOASESSolverHandle *solverHandle = (ihmc_optimizer_wrappers::QpOASESSolverHandle *) solverId;

   solverHandle->setupQPOASES(nvar, ncon);
   solverHandle->setupQuadraticProgramBuffers(env);
}
/*
 * Class:     us_ihmc_convexOptimization_QpOASESJNISolver
 * Method:    createSolver
 * Signature: ()J
 */
JNIEXPORT jlong JNICALL Java_us_ihmc_convexOptimization_QpOASESJNISolver_createSolver
        (JNIEnv *env, jobject object, jint hessianTypeOrdinal, jint solverTypeOrdinal)
{
   return (jlong) (new ihmc_optimizer_wrappers::QpOASESSolverHandle::QpOASESSolverHandle());
}

/*
 * Class:     us_ihmc_convexOptimization_QpOASESJNISolver
 * Method:    get_A_Buffer
 * Signature: (J)Ljava/nio/ByteBuffer;
 */
JNIEXPORT jobject JNICALL Java_us_ihmc_convexOptimization_QpOASESJNISolver_get_1A_1Buffer
        (JNIEnv *env, jobject object, jlong solverID)
{
   return ((ihmc_optimizer_wrappers::QpOASESSolverHandle *) solverID)->getABuffer();
}

/*
 * Class:     us_ihmc_convexOptimization_QpOASESJNISolver
 * Method:    get_x_Buffer
 * Signature: (J)Ljava/nio/ByteBuffer;
 */
JNIEXPORT jobject JNICALL Java_us_ihmc_convexOptimization_QpOASESJNISolver_get_1x_1Buffer
        (JNIEnv *env, jobject object, jlong solverID)
{
   return ((ihmc_optimizer_wrappers::QpOASESSolverHandle *) solverID)->getXBuffer();
}

/*
 * Class:     us_ihmc_convexOptimization_QpOASESJNISolver
 * Method:    get_H_Buffer
 * Signature: (J)Ljava/nio/ByteBuffer;
 */
JNIEXPORT jobject JNICALL Java_us_ihmc_convexOptimization_QpOASESJNISolver_get_1H_1Buffer
        (JNIEnv *env, jobject object, jlong solverID)
{
   return ((ihmc_optimizer_wrappers::QpOASESSolverHandle *) solverID)->getHessianBuffer();
}

/*
 * Class:     us_ihmc_convexOptimization_QpOASESJNISolver
 * Method:    get_g_Buffer
 * Signature: (J)Ljava/nio/ByteBuffer;
 */
JNIEXPORT jobject JNICALL Java_us_ihmc_convexOptimization_QpOASESJNISolver_get_1g_1Buffer
        (JNIEnv *env, jobject object, jlong solverID)
{
   return ((ihmc_optimizer_wrappers::QpOASESSolverHandle *) solverID)->getGradientBuffer();
}

/*
 * Class:     us_ihmc_convexOptimization_QpOASESJNISolver
 * Method:    get_lb_Buffer
 * Signature: (J)Ljava/nio/ByteBuffer;
 */
JNIEXPORT jobject JNICALL Java_us_ihmc_convexOptimization_QpOASESJNISolver_get_1lb_1Buffer
        (JNIEnv *env, jobject object, jlong solverID)
{
   return ((ihmc_optimizer_wrappers::QpOASESSolverHandle *) solverID)->getLowerBoundBuffer();
}

/*
 * Class:     us_ihmc_convexOptimization_QpOASESJNISolver
 * Method:    get_ub_Buffer
 * Signature: (J)Ljava/nio/ByteBuffer;
 */
JNIEXPORT jobject JNICALL Java_us_ihmc_convexOptimization_QpOASESJNISolver_get_1ub_1Buffer
        (JNIEnv *env, jobject object, jlong solverID)
{
   return ((ihmc_optimizer_wrappers::QpOASESSolverHandle *) solverID)->getUpperBoundBuffer();
}

/*
 * Class:     us_ihmc_convexOptimization_QpOASESJNISolver
 * Method:    get_lbA_Buffer
 * Signature: (J)Ljava/nio/ByteBuffer;
 */
JNIEXPORT jobject JNICALL Java_us_ihmc_convexOptimization_QpOASESJNISolver_get_1lbA_1Buffer
        (JNIEnv *env, jobject object, jlong solverID)
{
   return ((ihmc_optimizer_wrappers::QpOASESSolverHandle *) solverID)->getLowerBoundABuffer();
}

/*
 * Class:     us_ihmc_convexOptimization_QpOASESJNISolver
 * Method:    get_ubA_Buffer
 * Signature: (J)Ljava/nio/ByteBuffer;
 */
JNIEXPORT jobject JNICALL Java_us_ihmc_convexOptimization_QpOASESJNISolver_get_1ubA_1Buffer
        (JNIEnv *env, jobject object, jlong solverID)
{
   return ((ihmc_optimizer_wrappers::QpOASESSolverHandle *) solverID)->getUpperBoundABuffer();
}

/*
 * Class:     us_ihmc_convexOptimization_QpOASESJNISolver
 * Method:    getNumberOfWorkingSetChangesFromNative
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_us_ihmc_convexOptimization_QpOASESJNISolver_getNumberOfWorkingSetChangesFromNative
        (JNIEnv *env, jobject object, jlong solverID)
{
   return ((ihmc_optimizer_wrappers::QpOASESSolverHandle *) solverID)->getNumberOfWorkingSetChanges();
}

/*
 * Class:     us_ihmc_convexOptimization_QpOASESJNISolver
 * Method:    getCPUTimeFromNative
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL Java_us_ihmc_convexOptimization_QpOASESJNISolver_getCPUTimeFromNative
        (JNIEnv *env, jobject object, jlong solverID)
{
   return ((ihmc_optimizer_wrappers::QpOASESSolverHandle *) solverID)->getCPUTime();
}

/*
 * Class:     us_ihmc_convexOptimization_QpOASESJNISolver
 * Method:    getObjValueFromNative
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL Java_us_ihmc_convexOptimization_QpOASESJNISolver_getObjValueFromNative
        (JNIEnv *env, jobject object, jlong solverID)
{
   return ((ihmc_optimizer_wrappers::QpOASESSolverHandle *) solverID)->getObjValue();
}

#ifdef __cplusplus
}
#endif
#endif
