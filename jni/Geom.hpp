/*
 * Coords.hpp
 *
 *  Created on: Jun 1, 2013
 *      Author: tracy
 */

#ifndef COORDS_HPP_
#define COORDS_HPP_

#include <Eigen/Eigen>
#include <math.h>

typedef Eigen::Vector3d vec3;
typedef Eigen::Vector4d vec4;
typedef Eigen::Quaternion<double> rot;
struct LLA { double lat_DEG; double lon_DEG; double alt_M; };

vec3 geodeticToEcef(const LLA&);
rot ecefToEnuRotation(const LLA&);

#endif /* COORDS_HPP_ */
