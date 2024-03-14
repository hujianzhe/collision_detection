//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/math_quat.h"
#include <math.h>

#ifdef	__cplusplus
extern "C" {
#endif

CCTNum_t* mathQuatSet(CCTNum_t q[4], CCTNum_t x, CCTNum_t y, CCTNum_t z, CCTNum_t w) {
	q[0] = x;
	q[1] = y;
	q[2] = z;
	q[3] = w;
	return q;
}

CCTNum_t* mathQuatNormalized(CCTNum_t r[4], const CCTNum_t q[4]) {
	CCTNum_t m = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
	if (m > CCTNum(0.0)) {
		m = CCTNum(1.0) / CCTNum_sqrt(m);
		r[0] = q[0] * m;
		r[1] = q[1] * m;
		r[2] = q[2] * m;
		r[3] = q[3] * m;
	}
	return r;
}

CCTNum_t* mathQuatFromEuler(CCTNum_t q[4], const CCTNum_t e[3], const char order[3]) {
	CCTNum_t pitch_x = e[0];
	CCTNum_t yaw_y = e[1];
	CCTNum_t roll_z = e[2];

	CCTNum_t c1 = CCTNum_cos(pitch_x * CCTNum(0.5));
	CCTNum_t c2 = CCTNum_cos(yaw_y * CCTNum(0.5));
	CCTNum_t c3 = CCTNum_cos(roll_z * CCTNum(0.5));
	CCTNum_t s1 = CCTNum_sin(pitch_x * CCTNum(0.5));
	CCTNum_t s2 = CCTNum_sin(yaw_y * CCTNum(0.5));
	CCTNum_t s3 = CCTNum_sin(roll_z * CCTNum(0.5));

	if (order[0] == 'X' && order[1] == 'Y' && order[2] == 'Z') {
		q[0] = s1 * c2 * c3 + c1 * s2 * s3;
		q[1] = c1 * s2 * c3 - s1 * c2 * s3;
		q[2] = c1 * c2 * s3 + s1 * s2 * c3;
		q[3] = c1 * c2 * c3 - s1 * s2 * s3;
	}
	else if (order[0] == 'Y' && order[1] == 'X' && order[2] == 'Z') {
		q[0] = s1 * c2 * c3 + c1 * s2 * s3;
		q[1] = c1 * s2 * c3 - s1 * c2 * s3;
		q[2] = c1 * c2 * s3 - s1 * s2 * c3;
		q[3] = c1 * c2 * c3 + s1 * s2 * s3;
	}
	else if (order[0] == 'Z' && order[1] == 'X' && order[2] == 'Y') {
		q[0] = s1 * c2 * c3 - c1 * s2 * s3;
		q[1] = c1 * s2 * c3 + s1 * c2 * s3;
		q[2] = c1 * c2 * s3 + s1 * s2 * c3;
		q[3] = c1 * c2 * c3 - s1 * s2 * s3;
	}
	else if (order[0] == 'Z' && order[1] == 'Y' && order[2] == 'X') {
		q[0] = s1 * c2 * c3 - c1 * s2 * s3;
		q[1] = c1 * s2 * c3 + s1 * c2 * s3;
		q[2] = c1 * c2 * s3 - s1 * s2 * c3;
		q[3] = c1 * c2 * c3 + s1 * s2 * s3;
	}
	else if (order[0] == 'Y' && order[1] == 'Z' && order[2] == 'X') {
		q[0] = s1 * c2 * c3 + c1 * s2 * s3;
		q[1] = c1 * s2 * c3 + s1 * c2 * s3;
		q[2] = c1 * c2 * s3 - s1 * s2 * c3;
		q[3] = c1 * c2 * c3 - s1 * s2 * s3;
	}
	else if (order[0] == 'X' && order[1] == 'Z' && order[2] == 'Y') {
		q[0] = s1 * c2 * c3 - c1 * s2 * s3;
		q[1] = c1 * s2 * c3 - s1 * c2 * s3;
		q[2] = c1 * c2 * s3 + s1 * s2 * c3;
		q[3] = c1 * c2 * c3 + s1 * s2 * s3;
	}
	else {
		q[0] = q[1] = q[2] = CCTNum(0.0);
		q[3] = CCTNum(1.0);
	}
	return q;
}

CCTNum_t* mathQuatFromUnitVec3(CCTNum_t q[4], const CCTNum_t from[3], const CCTNum_t to[3]) {
	CCTNum_t v[3];
	CCTNum_t w = mathVec3Dot(from, to) + CCTNum(1.0);
	if (w < CCTNum(1E-7)) {
		CCTNum_t from_abs_x = CCTNum_abs(from[0]);
		CCTNum_t from_abs_z = CCTNum_abs(from[2]);
		if (from_abs_x > from_abs_z) {
			v[0] = -from[1];
			v[1] = from[0];
			v[2] = CCTNum(0.0);
		}
		else {
			v[0] = CCTNum(0.0);
			v[1] = -from[2];
			v[2] = from[1];
		}
		w = CCTNum(0.0);
	}
	else {
		mathVec3Cross(v, from, to);
	}

	q[0] = v[0];
	q[1] = v[1];
	q[2] = v[2];
	q[3] = w;
	return mathQuatNormalized(q, q);
}

CCTNum_t* mathQuatFromAxisRadian(CCTNum_t q[4], const CCTNum_t axis[3], CCTNum_t radian) {
	const CCTNum_t half_rad = radian * CCTNum(0.5);
	const CCTNum_t s = CCTNum_sin(half_rad);
	q[0] = axis[0] * s;
	q[1] = axis[1] * s;
	q[2] = axis[2] * s;
	q[3] = CCTNum_cos(half_rad);
	return q;
}

void mathQuatToAxisRadian(const CCTNum_t q[4], CCTNum_t axis[3], CCTNum_t* radian) {
	const CCTNum_t qx = q[0], qy = q[1], qz = q[2], qw = q[3];
	const CCTNum_t s2 = qx*qx + qy*qy + qz*qz;
	const CCTNum_t s = CCTNum(1.0) / CCTNum_sqrt(s2);
	axis[0] = qx * s;
	axis[1] = qy * s;
	axis[2] = qz * s;
	*radian = CCTNum_atan2(s2*s, qw) * CCTNum(2.0);
}

int mathQuatIsZero(const CCTNum_t q[4]) {
	return 	q[0] <= CCT_EPSILON && q[1] <= CCT_EPSILON && q[2] <= CCT_EPSILON && q[3] <= CCT_EPSILON &&
			q[0] >= CCT_EPSILON_NEGATE && q[1] >= CCT_EPSILON_NEGATE && q[2] >= CCT_EPSILON_NEGATE && q[3] >= CCT_EPSILON_NEGATE;
}

int mathQuatEqual(const CCTNum_t q1[4], const CCTNum_t q2[4]) {
	CCTNum_t delta;

	delta = q1[0] - q2[0];
	if (delta > CCT_EPSILON || delta < CCT_EPSILON_NEGATE) {
		return 0;
	}
	delta = q1[1] - q2[1];
	if (delta > CCT_EPSILON || delta < CCT_EPSILON_NEGATE) {
		return 0;
	}
	delta = q1[2] - q2[2];
	if (delta > CCT_EPSILON || delta < CCT_EPSILON_NEGATE) {
		return 0;
	}
	delta = q1[3] - q2[3];
	if (delta > CCT_EPSILON || delta < CCT_EPSILON_NEGATE) {
		return 0;
	}
	return 1;
}

CCTNum_t* mathQuatIdentity(CCTNum_t q[4]) {
	q[0] = q[1] = q[2] = CCTNum(0.0);
	q[3] = CCTNum(1.0);
	return q;
}

CCTNum_t mathQuatDot(const CCTNum_t q1[4], const CCTNum_t q2[4]) {
	return q1[0]* q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3];
}

/* r = -q */
CCTNum_t* mathQuatConjugate(CCTNum_t r[4], const CCTNum_t q[4]) {
	r[0] = -q[0];
	r[1] = -q[1];
	r[2] = -q[2];
	r[3] = q[3];
	return r;
}

/* r = q*n */
CCTNum_t* mathQuatMultiplyScalar(CCTNum_t r[4], const CCTNum_t q[4], CCTNum_t n) {
	r[0] = q[0] * n;
	r[1] = q[1] * n;
	r[2] = q[2] * n;
	r[3] = q[3] * n;
	return r;
}

/* r = q/n */
CCTNum_t* mathQuatDivisionScalar(CCTNum_t r[4], const CCTNum_t q[4], CCTNum_t n) {
	if (n != CCTNum(0.0)) {
		CCTNum_t inv = CCTNum(1.0) / n;
		r[0] = q[0] * inv;
		r[1] = q[1] * inv;
		r[2] = q[2] * inv;
		r[3] = q[3] * inv;
	}
	return r;
}

/* r = q1 * q2 */
CCTNum_t* mathQuatMulQuat(CCTNum_t r[4], const CCTNum_t q1[4], const CCTNum_t q2[4]) {
	const CCTNum_t q1x = q1[0], q1y = q1[1], q1z = q1[2], q1w = q1[3];
	const CCTNum_t q2x = q2[0], q2y = q2[1], q2z = q2[2], q2w = q2[3];
	r[0] = q1w*q2x + q2w*q1x + q1y*q2z - q2y*q1z;
	r[1] = q1w*q2y + q2w*q1y + q1z*q2x - q2z*q1x;
	r[2] = q1w*q2z + q2w*q1z + q1x*q2y - q2x*q1y;
	r[3] = q1w*q2w - q2x*q1x - q1y*q2y - q2z*q1z;
	return r;
}

/* r = q * v */
CCTNum_t* mathQuatMulVec3(CCTNum_t r[3], const CCTNum_t q[4], const CCTNum_t v[3]) {
	const CCTNum_t vx = CCTNum(2.0) * v[0];
	const CCTNum_t vy = CCTNum(2.0) * v[1];
	const CCTNum_t vz = CCTNum(2.0) * v[2];
	const CCTNum_t qx = q[0], qy = q[1], qz = q[2], qw = q[3];
	const CCTNum_t qw2 = qw*qw - CCTNum(0.5);
	const CCTNum_t dot2 = qx*vx + qy*vy + qz*vz;
	r[0] = vx*qw2 + (qy * vz - qz * vy)*qw + qx*dot2;
	r[1] = vy*qw2 + (qz * vx - qx * vz)*qw + qy*dot2;
	r[2] = vz*qw2 + (qx * vy - qy * vx)*qw + qz*dot2;
	return r;
}

/* r = q * v */
CCTNum_t* mathQuatMulVec3Inv(CCTNum_t r[3], const CCTNum_t q[4], const CCTNum_t v[3]) {
	const CCTNum_t vx = CCTNum(2.0) * v[0];
	const CCTNum_t vy = CCTNum(2.0) * v[1];
	const CCTNum_t vz = CCTNum(2.0) * v[2];
	const CCTNum_t qx = q[0], qy = q[1], qz = q[2], qw = q[3];
	const CCTNum_t qw2 = qw*qw - CCTNum(0.5);
	const CCTNum_t dot2 = qx*vx + qy*vy + qz*vz;
	r[0] = vx*qw2 - (qy * vz - qz * vy)*qw + qx*dot2;
	r[1] = vy*qw2 - (qz * vx - qx * vz)*qw + qy*dot2;
	r[2] = vz*qw2 - (qx * vy - qy * vx)*qw + qz*dot2;
	return r;
}

#ifdef	__cplusplus
}
#endif
