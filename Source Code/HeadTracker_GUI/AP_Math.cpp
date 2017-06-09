#include "AP_Math.h"
#include "StdAfx.h"


// a varient of asin() that checks the input ranges and ensures a
// valid angle as output. If nan is given as input then zero is
// returned.
double safe_asin(double v)
{
	if (isnan(v)) {
		return 0.0;
	}
	if (v >= 1.0) {
		return PI/2;
	}
	if (v <= -1.0) {
		return -PI/2;
	}
	return asin(v);
}

// a varient of sqrt() that checks the input ranges and ensures a
// valid value as output. If a negative number is given then 0 is
// returned. The reasoning is that a negative number for sqrt() in our
// code is usually caused by small numerical rounding errors, so the
// real input should have been zero
double safe_sqrt(double v)
{
	double ret = sqrt(v);
	if (isnan(ret)) {
		return 0;
	}
	return ret;
}


// find a rotation that is the combination of two other
// rotations. This is used to allow us to add an overall board
// rotation to an existing rotation of a sensor such as the compass
// Note that this relies the set of rotations being complete. The
// optional 'found' parameter is for the test suite to ensure that it is.
enum Rotation rotation_combination(enum Rotation r1, enum Rotation r2, bool *found)
{
	Vector3f tv1, tv2;
	enum Rotation r;
	tv1(1,2,3);
	tv1.rotate(r1);
	tv1.rotate(r2);

	for (r=ROTATION_NONE; r<ROTATION_MAX; r = (enum Rotation)((int)r+1)) {
		Vector3f diff;
		tv2(1,2,3);
		tv2.rotate(r);
		diff = tv1 - tv2;
		if (diff.length() < 1.0e-6) {
			// we found a match
			if (found) {
				*found = true;
			}
			return r;
		}
	}

	// we found no matching rotation. Someone has edited the
	// rotations list and broken its completeness property ...
	if (found) {
		*found = false;
	}
	return ROTATION_NONE;
}



/*
  simple 16 bit random number generator
 */
uint16_t get_random16(void)
{
    static uint32_t m_z = 1234;
    static uint32_t m_w = 76542;
    m_z = 36969 * (m_z & 0xFFFFu) + (m_z >> 16);
    m_w = 18000 * (m_w & 0xFFFFu) + (m_w >> 16);
    return ((m_z << 16) + m_w) & 0xFFFF;
}





