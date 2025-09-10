//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"

#ifdef	__cplusplus
extern "C" {
#endif

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

int mathVec3EqualEps(const CCTNum_t v1[3], const CCTNum_t v2[3], CCTNum_t eps) {
	CCTNum_t delta;
	if (0.0 == eps) {
		return v1[0] == v2[0] && v1[1] == v2[1] && v1[2] == v2[2];
	}

	delta = CCTNum_abs(v1[0] - v2[0]);
	if (delta > eps) {
		return 0;
	}
	delta = CCTNum_abs(v1[1] - v2[1]);
	if (delta > eps) {
		return 0;
	}
	delta = CCTNum_abs(v1[2] - v2[2]);
	if (delta > eps) {
		return 0;
	}
	return 1;
}

CCTNum_t* mathVec3MergeMin(CCTNum_t r[3], const CCTNum_t v1[3], const CCTNum_t v2[3]) {
	r[0] = (v1[0] < v2[0] ? v1[0] : v2[0]);
	r[1] = (v1[1] < v2[1] ? v1[1] : v2[1]);
	r[2] = (v1[2] < v2[2] ? v1[2] : v2[2]);
	return r;
}

CCTNum_t* mathVec3MergeMax(CCTNum_t r[3], const CCTNum_t v1[3], const CCTNum_t v2[3]) {
	r[0] = (v1[0] > v2[0] ? v1[0] : v2[0]);
	r[1] = (v1[1] > v2[1] ? v1[1] : v2[1]);
	r[2] = (v1[2] > v2[2] ? v1[2] : v2[2]);
	return r;
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
		if (CCTNum(-1.0) - CCT_EPSILON <= r[0] && r[0] <= CCTNum(-1.0) + CCT_EPSILON) {
			r[0] = CCTNum(-1.0);
			r[1] = r[2] = CCTNum(0.0);
			return len;
		}

		r[1] = v[1] * inv_len;
		if (CCTNum(1.0) - CCT_EPSILON <= r[1] && r[1] <= CCTNum(1.0) + CCT_EPSILON) {
			r[1] = CCTNum(1.0);
			r[0] = r[2] = CCTNum(0.0);
			return len;
		}
		if (CCTNum(-1.0) - CCT_EPSILON <= r[1] && r[1] <= CCTNum(-1.0) + CCT_EPSILON) {
			r[1] = CCTNum(-1.0);
			r[0] = r[2] = CCTNum(0.0);
			return len;
		}

		r[2] = v[2] * inv_len;
		if (CCTNum(1.0) - CCT_EPSILON <= r[2] && r[2] <= CCTNum(1.0) + CCT_EPSILON) {
			r[2] = CCTNum(1.0);
			r[0] = r[1] = CCTNum(0.0);
			return len;
		}
		if (CCTNum(-1.0) - CCT_EPSILON <= r[2] && r[2] <= CCTNum(-1.0) + CCT_EPSILON) {
			r[2] = CCTNum(-1.0);
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

CCTNum_t mathVec3DistanceSq(const CCTNum_t p1[3], const CCTNum_t p2[3]) {
	CCTNum_t dx = p1[0] - p2[0];
	CCTNum_t dy = p1[1] - p2[1];
	CCTNum_t dz = p1[2] - p2[2];
	return dx * dx + dy * dy + dz * dz;
}

CCTNum_t* mathVec3Midpoint(CCTNum_t mp[3], const CCTNum_t p1[3], const CCTNum_t p2[3]) {
	mp[0] = p1[0] + (p2[0] - p1[0]) * CCTNum(0.5);
	mp[1] = p1[1] + (p2[1] - p1[1]) * CCTNum(0.5);
	mp[2] = p1[2] + (p2[2] - p1[2]) * CCTNum(0.5);
	return mp;
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

int mathVec3IsParallel(const CCTNum_t v1[3], const CCTNum_t v2[3]) {
	CCTNum_t x, y, z;
	x = v1[1] * v2[2] - v1[2] * v2[1];
	if (x > CCT_EPSILON || x < CCT_EPSILON_NEGATE) {
		return 0;
	}
	y = v1[2] * v2[0] - v1[0] * v2[2];
	if (y > CCT_EPSILON || y < CCT_EPSILON_NEGATE) {
		return 0;
	}
	z = v1[0] * v2[1] - v1[1] * v2[0];
	if (z > CCT_EPSILON || z < CCT_EPSILON_NEGATE) {
		return 0;
	}
	return 1;
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

static const CCTNum_t Axis_X[3] = { CCTNums_3(1.0, 0.0, 0.0) };
static const CCTNum_t Axis_Y[3] = { CCTNums_3(0.0, 1.0, 0.0) };
static const CCTNum_t Axis_Z[3] = { CCTNums_3(0.0, 0.0, 1.0) };
static const CCTNum_t Vec3_Zero[3] = { CCTNums_3(0.0, 0.0, 0.0) };

CCTNum_t* mathVec3AnyOtherAxis(CCTNum_t r[3], const CCTNum_t exist_axis[3]) {
	CCTNum_t temp_v[3], len;
	mathVec3Cross(temp_v, exist_axis, Axis_X);
	if (!mathVec3EqualEps(temp_v, Vec3_Zero, CCTNum(1e-2))) {
		len = mathVec3Len(temp_v);
		return mathVec3MultiplyScalar(r, temp_v, CCTNum(1.0) / len);
	}
	mathVec3Cross(r, exist_axis, Axis_Y);
	len = mathVec3Len(r);
	return mathVec3MultiplyScalar(r, r, CCTNum(1.0) / len);
}

#ifdef	__cplusplus
}
#endif
