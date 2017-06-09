// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#ifndef AP_MATH_H
#define AP_MATH_H

// Assorted useful math operations for ArduPilot(Mega)

#define PI 3.1415926535
#define M_PI      (3.141592653589793f)


inline bool isnan(double x)
{
   return x != x;
}

#include "StdAfx.h"
#include <cmath>
#include <limits>
#include <stdint.h>
#include <type_traits>

//#include <AP_Common.h>
//#include <stdint.h>
#include "rotations.h"
#include "vector2.h"
#include "vector3.h"
#include "matrix3.h"
#include "quaternion.h"
#include "polygon.h"

/*
template <class Arithmetic1, class Arithmetic2>
typename std::enable_if<std::is_integral<typename std::common_type<Arithmetic1, Arithmetic2>::type>::value ,bool>::type
is_equal(const Arithmetic1 v_1, const Arithmetic2 v_2);

template <class Arithmetic1, class Arithmetic2>
typename std::enable_if<std::is_floating_point<typename std::common_type<Arithmetic1, Arithmetic2>::type>::value, bool>::type
is_equal(const Arithmetic1 v_1, const Arithmetic2 v_2);
*/

inline bool is_equal(double d1,double d2){
	return abs(d1-d2)< std::numeric_limits<double>::epsilon();

}

#define is_zero(v) is_equal(v,0)

inline long round (double value)
{
   return (long) floor(value + 0.5);
}


template <class T>
T constrain_value(const T amt, const T low, const T high);

inline double constrain_float(const double amt, const double low, const double high)
{
    //return constrain_value(amt, low, high);
	if(amt<low) return low;
	if(amt>high) return high;
	return amt;
}

inline int16_t constrain_int16(const int16_t amt, const int16_t low, const int16_t high)
{
    return constrain_value(amt, low, high);
}

inline int32_t constrain_int32(const int32_t amt, const int32_t low, const int32_t high)
{
    return constrain_value(amt, low, high);
}

// define AP_Param types AP_Vector3f and Ap_Matrix3f
//AP_PARAMDEFV(Matrix3f, Matrix3f, AP_PARAM_MATRIX3F);
//AP_PARAMDEFV(Vector3f, Vector3f, AP_PARAM_VECTOR3F);

// a varient of asin() that always gives a valid answer.
double safe_asin(double v);

// a varient of sqrt() that always gives a valid answer.
double safe_sqrt(double v);

// find a rotation that is the combination of two other
// rotations. This is used to allow us to add an overall board
// rotation to an existing rotation of a sensor such as the compass
enum Rotation rotation_combination(enum Rotation r1, enum Rotation r2, bool *found = NULL);


/* simple 16 bit random number generator */
uint16_t get_random16(void);

// matrix algebra
bool inverse(double x[], double y[], uint8_t dim);

using namespace std;

inline int isinf(double x) { return !isnan(x) && isnan(x - x); }

#endif
