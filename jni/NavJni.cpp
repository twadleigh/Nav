#include "Nav.hpp"
#include "NavJni.hpp"

Nav* g_navPtr = 0;


// helpers

inline double t_S(long t_nS) { return double(t_nS)*1.0e-9; }

inline cvec3 get3Floats(JNIEnv* env, jfloatArray a) {
	float* ptr = env->GetFloatArrayElements(a, 0);
	float buf[3];
	memcpy(&buf, ptr, 3*sizeof(float));
	env->ReleaseFloatArrayElements(a, ptr, 0);
	return cvec3(buf[0],buf[1],buf[2]);
}


// initialization

void NAV_FN(initialize)(JNIEnv*, jobject, long t0, double lon, double lat, double alt, float err) {
	if(g_navPtr != 0) delete g_navPtr;
	cLLA lla(lon, lat, alt);
	g_navPtr = new Nav(t_S(t0), lla);
}


// measurement updates

void NAV_FN(updateGps)(JNIEnv*, jobject, long t, double lon, double lat, double alt, float err) {
	g_navPtr->updateGps(t_S(t), cLLA(lon,lat,alt));
}

void NAV_FN(updateAccelerometer)(JNIEnv* env, jobject, long t, jfloatArray a) {
	g_navPtr->updateAccelerometer(t_S(t), get3Floats(env, a));
}

void NAV_FN(updateGyroscope)(JNIEnv* env, jobject, long t, jfloatArray o) {
	g_navPtr->updateGyroscope(t_S(t), get3Floats(env, o));
}

void NAV_FN(updateMagnetometer)(JNIEnv* env, jobject, long t, jfloatArray f) {
	g_navPtr->updateMagnetometer(t_S(t), get3Floats(env, f));
}


// state query
void NAV_FN(getState)(JNIEnv* env, jobject, long t, jdoubleArray s) {
	g_navPtr->update(t_S(t));
	double* ptr = env->GetDoubleArrayElements(s, 0);
	memcpy(ptr, &(g_navPtr->state.data), Nav::State::SIZE*sizeof(double));
	env->ReleaseDoubleArrayElements(s, ptr, 0);
}
