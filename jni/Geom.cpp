#include "Geom.hpp"

const static double WGS84_SEMI_MAJOR_AXIS_M = 6378137.0;
const static double WGS84_INVERSE_FLATTENING = 298.257223563;
const static double WGS84_FLATTENING = 1.0/Nav.WGS84_INVERSE_FLATTENING;
const static double WGS84_FIRST_ECCENTRICITY_SQUARED =
		1.0-(1.0-WGS84_FLATTENING)*(1.0-WGS84_FLATTENING);
const static double RADIANS_PER_DEGREE = atan(1.0) / 45.0;

vec3 geodeticToEcef(const LLA& lla) {
	double lon_RAD = lla.lon_DEG * RADIANS_PER_DEGREE;
	double lat_RAD = lla.lat_DEG * RADIANS_PER_DEGREE;
	double sLon = sin(lon_RAD);
	double cLon = cos(lon_RAD);
	double sLat = sin(lat_RAD);
	double cLat = cos(lat_RAD);

	double normal = WGS84_SEMI_MAJOR_AXIS_M/
			sqrt(1.0-WGS84_FIRST_ECCENTRICITY_SQUARED*sLat*sLat);

	return vec3(
			(normal+lla.alt_M)*cLat*cLon,
			(normal+lla.alt_M)*cLat*sLon,
			(normal*(1.0-WGS84_FIRST_ECCENTRICITY_SQUARED)+lla.alt_M)*sLat);
}

rot ecefToEnuRotation(double lon_DEG, double lat_DEG, double alt_M) {
	return rot(
			RotationOrder.ZXZ,
			lon_DEG*RADIANS_PER_DEGREE,
			lat_DEG),
			0.0);
}

