//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include <math.h>

#ifdef	__cplusplus
extern "C" {
#endif

CCTNum_t* mathCoordinateSystemTransform(const CCTNum_t v[3], const CCTNum_t new_origin[3], const CCTNum_t new_axies[3][3], CCTNum_t new_v[3]) {
	CCTNum_t t[3];
	mathVec3Sub(t, v, new_origin);
	new_v[0] = mathVec3Dot(t, new_axies[0]);
	new_v[1] = mathVec3Dot(t, new_axies[1]);
	new_v[2] = mathVec3Dot(t, new_axies[2]);
	return new_v;
}

int mathVec3IsValid(const CCTNum_t v[3]) {
	if (isinf(v[0]) || isnan(v[0])) {
		return 0;
	}
	if (isinf(v[1]) || isnan(v[1])) {
		return 0;
	}
	if (isinf(v[2]) || isnan(v[2])) {
		return 0;
	}
	return 1;
}

CCTNum_t* mathVec3Set(CCTNum_t r[3], CCTNum_t x, CCTNum_t y, CCTNum_t z) {
	r[0] = x;
	r[1] = y;
	r[2] = z;
	return r;
}

int mathVec3IsZero(const CCTNum_t v[3]) {
	return	v[0] <= CCT_EPSILON && v[1] <= CCT_EPSILON && v[2] <= CCT_EPSILON &&
			v[0] >= CCT_EPSILON_NEGATE && v[1] >= CCT_EPSILON_NEGATE && v[2] >= CCT_EPSILON_NEGATE;
}

int mathVec3Equal(const CCTNum_t v1[3], const CCTNum_t v2[3]) {
	CCTNum_t delta;

	delta = v1[0] - v2[0];
	if (delta > CCT_EPSILON || delta < CCT_EPSILON_NEGATE) {
		return 0;
	}
	delta = v1[1] - v2[1];
	if (delta > CCT_EPSILON || delta < CCT_EPSILON_NEGATE) {
		return 0;
	}
	delta = v1[2] - v2[2];
	if (delta > CCT_EPSILON || delta < CCT_EPSILON_NEGATE) {
		return 0;
	}
	return 1;
}

CCTNum_t mathVec3MinElement(const CCTNum_t v[3]) {
	if (v[0] < v[1]) {
		return v[0] < v[2] ? v[0] : v[2];
	}
	return v[1] < v[2] ? v[1] : v[2];
}

CCTNum_t mathVec3MaxElement(const CCTNum_t v[3]) {
	if (v[0] > v[1]) {
		return v[0] > v[2] ? v[0] : v[2];
	}
	return v[1] > v[2] ? v[1] : v[2];
}

/* r = v */
CCTNum_t* mathVec3Copy(CCTNum_t r[3], const CCTNum_t v[3]) {
	r[0] = v[0];
	r[1] = v[1];
	r[2] = v[2];
	return r;
}

CCTNum_t mathVec3LenSq(const CCTNum_t v[3]) {
	return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
}

CCTNum_t mathVec3Len(const CCTNum_t v[3]) {
	return CCTNum_sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

CCTNum_t mathVec3Normalized(CCTNum_t r[3], const CCTNum_t v[3]) {
	CCTNum_t len_sq = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
	if (len_sq > CCTNum(0.0)) {
		CCTNum_t len = CCTNum_sqrt(len_sq);
		CCTNum_t inv_len = CCTNum(1.0) / len;
		r[0] = v[0] * inv_len;
		if (CCTNum(1.0) - CCT_EPSILON <= r[0] && r[0] <= CCTNum(1.0) + CCT_EPSILON) {
			r[0] = CCTNum(1.0);
			r[1] = r[2] = CCTNum(0.0);
			return len;
		}
		r[1] = v[1] * inv_len;
		if (CCTNum(1.0) - CCT_EPSILON <= r[1] && r[1] <= CCTNum(1.0) + CCT_EPSILON) {
			r[1] = CCTNum(1.0);
			r[0] = r[2] = CCTNum(0.0);
			return len;
		}
		r[2] = v[2] * inv_len;
		if (CCTNum(1.0) - CCT_EPSILON <= r[2] && r[2] <= CCTNum(1.0) + CCT_EPSILON) {
			r[2] = CCTNum(1.0);
			r[0] = r[1] = CCTNum(0.0);
			return len;
		}
		return len;
	}
	else {
		r[0] = r[1] = r[2] = CCTNum(0.0);
		return CCTNum(0.0);
	}
}

CCTNum_t mathVec3Direction(const CCTNum_t end[3], const CCTNum_t start[3], CCTNum_t dir[3]) {
	CCTNum_t v[3];
	if (!dir) {
		dir = v;
	}
	mathVec3Sub(dir, end, start);
	if (mathVec3IsZero(dir)) {
		dir[0] = dir[1] = dir[2] = CCTNum(0.0);
		return CCTNum(0.0);
	}
	return mathVec3Normalized(dir, dir);
}

CCTNum_t mathVec3DistanceSq(const CCTNum_t p1[3], const CCTNum_t p2[3]) {
	CCTNum_t dx = p1[0] - p2[0];
	CCTNum_t dy = p1[1] - p2[1];
	CCTNum_t dz = p1[2] - p2[2];
	return dx * dx + dy * dy + dz * dz;
}

/* r = -v */
CCTNum_t* mathVec3Negate(CCTNum_t r[3], const CCTNum_t v[3]) {
	r[0] = -v[0];
	r[1] = -v[1];
	r[2] = -v[2];
	return r;
}

/* r = v1 + v2 */
CCTNum_t* mathVec3Add(CCTNum_t r[3], const CCTNum_t v1[3], const CCTNum_t v2[3]) {
	r[0] = v1[0] + v2[0];
	r[1] = v1[1] + v2[1];
	r[2] = v1[2] + v2[2];
	return r;
}

/* r += v * n */
CCTNum_t* mathVec3AddScalar(CCTNum_t r[3], const CCTNum_t v[3], CCTNum_t n) {
	r[0] += v[0] * n;
	r[1] += v[1] * n;
	r[2] += v[2] * n;
	return r;
}

/* r -= v * n */
CCTNum_t* mathVec3SubScalar(CCTNum_t r[3], const CCTNum_t v[3], CCTNum_t n) {
	r[0] -= v[0] * n;
	r[1] -= v[1] * n;
	r[2] -= v[2] * n;
	return r;
}

/* r = v1 - v2 */
CCTNum_t* mathVec3Sub(CCTNum_t r[3], const CCTNum_t v1[3], const CCTNum_t v2[3]) {
	r[0] = v1[0] - v2[0];
	r[1] = v1[1] - v2[1];
	r[2] = v1[2] - v2[2];
	return r;
}

/* r = v*n */
CCTNum_t* mathVec3MultiplyScalar(CCTNum_t r[3], const CCTNum_t v[3], CCTNum_t n) {
	r[0] = v[0] * n;
	r[1] = v[1] * n;
	r[2] = v[2] * n;
	return r;
}

/* r = v/n */
CCTNum_t* mathVec3DivisionScalar(CCTNum_t r[3], const CCTNum_t v[3], CCTNum_t n) {
	if (n != CCTNum(0.0)) {
		CCTNum_t inv = CCTNum(1.0) / n;
		r[0] = v[0] * inv;
		r[1] = v[1] * inv;
		r[2] = v[2] * inv;
	}
	return r;
}

/* r = v1 * v2, r = |v1|*|v2|*cos@ */
CCTNum_t mathVec3Dot(const CCTNum_t v1[3], const CCTNum_t v2[3]) {
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

CCTNum_t mathVec3Radian(const CCTNum_t v1[3], const CCTNum_t v2[3]) {
	return CCTNum_acos(mathVec3Dot(v1, v2) / CCTNum_sqrt(mathVec3LenSq(v1) * mathVec3LenSq(v2)));
}

/* r = v1 X v2, r = |v1|*|v2|*sin@ */
CCTNum_t* mathVec3Cross(CCTNum_t r[3], const CCTNum_t v1[3], const CCTNum_t v2[3]) {
	CCTNum_t x = v1[1] * v2[2] - v1[2] * v2[1];
	CCTNum_t y = v1[2] * v2[0] - v1[0] * v2[2];
	CCTNum_t z = v1[0] * v2[1] - v1[1] * v2[0];
	r[0] = x;
	r[1] = y;
	r[2] = z;
	return r;
}

CCTNum_t mathVec3CrossNormalized(CCTNum_t r[3], const CCTNum_t v1[3], const CCTNum_t v2[3]) {
	mathVec3Cross(r, v1, v2);
	if (mathVec3IsZero(r)) {
		r[0] = r[1] = r[2] = CCTNum(0.0);
		return CCTNum(0.0);
	}
	return mathVec3Normalized(r, r);
}

CCTNum_t* mathVec3Reflect(CCTNum_t r[3], const CCTNum_t v[3], const CCTNum_t n[3]) {
	CCTNum_t temp_v[3];
	CCTNum_t dot = mathVec3Dot(v, n);
	mathVec3MultiplyScalar(temp_v, n, dot + dot);
	return mathVec3Sub(r, v, temp_v);
}

CCTNum_t* mathVec3Glide(CCTNum_t r[3], const CCTNum_t v[3], const CCTNum_t n[3]) {
	CCTNum_t temp_v[3];
	CCTNum_t dot = mathVec3Dot(v, n);
	mathVec3MultiplyScalar(temp_v, n, dot);
	return mathVec3Sub(r, v, temp_v);
}

CCTNum_t* mathVec3DelComponent(CCTNum_t r[3], const CCTNum_t v[3], const CCTNum_t dir[3]) {
	CCTNum_t va[3];
	CCTNum_t d = mathVec3Dot(v, dir);
	mathVec3MultiplyScalar(va, dir, d);
	return mathVec3Sub(r, v, va);
}

#ifdef	__cplusplus
}
#endif
