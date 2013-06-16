/*
 * Nav.hpp
 *
 *  Created on: Jun 1, 2013
 *      Author: tracy
 */

#ifndef NAV_JNI_HPP_
#define NAV_JNI_HPP_

#include <jni.h>

#include "Nav.hpp"

#define NAV_FN(fn) Java_com_github_twadleigh_nav_Nav_ ## fn

// initialization
extern "C" void NAV_FN(initialize)(JNIEnv*, jobject, long t0, double lon, double lat, double alt, float err);

// measurement updates
extern "C" void NAV_FN(updateGps)(JNIEnv*, jobject, long t, double lon, double lat, double alt, float err);
extern "C" void NAV_FN(updateAccelerometer)(JNIEnv*, jobject, long t, jfloatArray a);
extern "C" void NAV_FN(updateGyroscope)(JNIEnv*, jobject, long t, jfloatArray o);
extern "C" void NAV_FN(updateMagnetometer)(JNIEnv*, jobject, long t, jfloatArray f);

// state query
extern "C" void NAV_FN(getState)(JNIEnv*, jobject, long t, jdoubleArray s);

#endif /* NAV_JNI_HPP_ */
