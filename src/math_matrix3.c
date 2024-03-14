//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/math_matrix3.h"
#include <math.h>

#ifdef	__cplusplus
extern "C" {
#endif

void mathMat44TransformSplit(const CCTNum_t m[16], CCTNum_t T[3], CCTNum_t S[3], CCTNum_t R[9]) {
	if (T) {
		T[0] = m[12];
		T[1] = m[13];
		T[2] = m[14];
	}
	if (R) {
		CCTNum_t s[3];
		if (!S) {
			S = s;
		}
		S[0] = mathVec3Len(m + 0);
		S[1] = mathVec3Len(m + 4);
		S[2] = mathVec3Len(m + 8);
		mathVec3MultiplyScalar(R + 0, m + 0, CCTNum(1.0) / S[0]);
		mathVec3MultiplyScalar(R + 3, m + 4, CCTNum(1.0) / S[1]);
		mathVec3MultiplyScalar(R + 6, m + 8, CCTNum(1.0) / S[2]);
	}
	else if (S) {
		S[0] = mathVec3Len(m + 0);
		S[1] = mathVec3Len(m + 4);
		S[2] = mathVec3Len(m + 8);
	}
}

CCTNum_t* mathMat44SetPositionPart(CCTNum_t m[16], const CCTNum_t p[3]) {
	m[12] = p[0];
	m[13] = p[1];
	m[14] = p[2];
	return m;
}

CCTNum_t* mathMat44Element(const CCTNum_t m[16], unsigned int column, unsigned int row) {
	return (CCTNum_t*)&m[(column << 2) + row];
}

CCTNum_t* mathMat44ToMat33(const CCTNum_t m44[16], CCTNum_t m33[9]) {
	m33[0] = m44[0];
	m33[1] = m44[1];
	m33[2] = m44[2];

	m33[3] = m44[4];
	m33[4] = m44[5];
	m33[5] = m44[6];

	m33[6] = m44[8];
	m33[7] = m44[9];
	m33[8] = m44[10];
	return m33;
}

CCTNum_t* mathMat44Copy(CCTNum_t r[16], const CCTNum_t m[16]) {
	if (r == m) {
		return r;
	}
	r[0] = m[0]; r[1] = m[1]; r[2] = m[2]; r[3] = m[3];
	r[4] = m[4]; r[5] = m[5]; r[6] = m[6]; r[7] = m[7];
	r[8] = m[8]; r[9] = m[9]; r[10] = m[10]; r[11] = m[11];
	r[12] = m[12]; r[13] = m[13]; r[14] = m[14]; r[15] = m[15];
	return r;
}

CCTNum_t* mathMat44Identity(CCTNum_t m[16]) {
	m[0] = CCTNum(1.0);
	m[1] = CCTNum(0.0);
	m[2] = CCTNum(0.0);
	m[3] = CCTNum(0.0);

	m[4] = CCTNum(0.0);
	m[5] = CCTNum(1.0);
	m[6] = CCTNum(0.0);
	m[7] = CCTNum(0.0);

	m[8] = CCTNum(0.0);
	m[9] = CCTNum(0.0);
	m[10] = CCTNum(1.0);
	m[11] = CCTNum(0.0);

	m[12] = CCTNum(0.0);
	m[13] = CCTNum(0.0);
	m[14] = CCTNum(0.0);
	m[15] = CCTNum(1.0);

	return m;
}

CCTNum_t* mathMat44Add(CCTNum_t r[16], const CCTNum_t m1[16], const CCTNum_t m2[16]) {
	int i;
	for (i = 0; i < 16; ++i) {
		r[i] = m1[i] + m2[i];
	}
	return r;
}

CCTNum_t* mathMat44MultiplyScalar(CCTNum_t r[16], const CCTNum_t m[16], CCTNum_t n) {
	int i;
	for (i = 0; i < 16; ++i) {
		r[i] = m[i] * n;
	}
	return r;
}

/* r = m1*m2  */
CCTNum_t* mathMat44MulMat44(CCTNum_t r[16], const CCTNum_t m1[16], const CCTNum_t m2[16]) {
	int i, j;
	for (i = 0; i < 4; ++i) {
		for (j = 0; j < 16; j += 4) {
			r[j+i] = m1[i]*m2[j] + m1[i+4]*m2[1+j] + m1[i+8]*m2[2+j] + m1[i+12]*m2[3+j];
		}
	}
	return r;
}

CCTNum_t* mathMat44Transpose(CCTNum_t r[16], const CCTNum_t m[16]) {
	CCTNum_t t;

	t = m[1];
	r[1] = m[4];
	r[4] = t;

	t = m[2];
	r[2] = m[8];
	r[8] = t;

	t = m[3];
	r[3] = m[12];
	r[12] = t;

	t = m[6];
	r[6] = m[9];
	r[9] = t;

	t = m[7];
	r[7] = m[13];
	r[13] = t;

	t = m[11];
	r[11] = m[14];
	r[14] = t;

	if (r != m) {
		r[0] = m[0];
		r[5] = m[5];
		r[10] = m[10];
		r[15] = m[15];
	}

	return r;
}

CCTNum_t* mathMat44Inverse(CCTNum_t r[16], const CCTNum_t m[16]) {
	CCTNum_t t;

	r[3] = -m[0]*m[12] - m[1]*m[13] - m[2]*m[14];
	r[7] = -m[4]*m[12] - m[5]*m[13] - m[6]*m[14];
	r[11] = -m[8]*m[12] - m[9]*m[13] - m[10]*m[14];

	r[12] = r[3];
	r[13] = r[7];
	r[14] = r[11];
	r[3] = r[7] = r[11] = CCTNum(0.0);
	r[15] = CCTNum(1.0);

	t = m[1];
	r[1] = m[4];
	r[4] = t;

	t = m[2];
	r[2] = m[8];
	r[8] = t;

	t = m[6];
	r[6] = m[9];
	r[9] = t;

	if (r != m) {
		r[0] = m[0];
		r[5] = m[5];
		r[10] = m[10];
	}

	return r;
}

CCTNum_t* mathMat44TransformVec3(CCTNum_t r[3], const CCTNum_t m[16], const CCTNum_t v[3]) {
	CCTNum_t x = v[0], y = v[1], z = v[2];
	r[0] = m[0]*x + m[4]*y + m[8]*z + m[12];
	r[1] = m[1]*x + m[5]*y + m[9]*z + m[13];
	r[2] = m[2]*x + m[6]*y + m[10]*z + m[14];
	return r;
}

CCTNum_t* mathMat44RotateVec3(CCTNum_t r[3], const CCTNum_t m[16], const CCTNum_t v[3]) {
	CCTNum_t x = v[0], y = v[1], z = v[2];
	r[0] = m[0]*x + m[4]*y + m[8]*z;
	r[1] = m[1]*x + m[5]*y + m[9]*z;
	r[2] = m[2]*x + m[6]*y + m[10]*z;
	return r;
}

CCTNum_t* mathMat44FromQuat(CCTNum_t m[16], const CCTNum_t q[4]) {
	CCTNum_t x = q[0];
	CCTNum_t y = q[1];
	CCTNum_t z = q[2];
	CCTNum_t w = q[3];

	CCTNum_t x2 = x + x;
	CCTNum_t y2 = y + y;
	CCTNum_t z2 = z + z;

	CCTNum_t xx = x2 * x;
	CCTNum_t yy = y2 * y;
	CCTNum_t zz = z2 * z;

	CCTNum_t xy = x2 * y;
	CCTNum_t xz = x2 * z;
	CCTNum_t xw = x2 * w;

	CCTNum_t yz = y2 * z;
	CCTNum_t yw = y2 * w;
	CCTNum_t zw = z2 * w;

	/* column 0 */
	m[0] = CCTNum(1.0) - yy - zz;
	m[1] = xy + zw;
	m[2] = xz - yw;
	m[3] = CCTNum(0.0);
	/* column 1 */
	m[4] = xy - zw;
	m[5] = CCTNum(1.0) - xx - zz;
	m[6] = yz + xw;
	m[7] = CCTNum(0.0);
	/* column 2 */
	m[8] = xz + yw;
	m[9] = yz - xw;
	m[10] = CCTNum(1.0) - xx - yy;
	m[11] = CCTNum(0.0);
	/* column 3 */
	m[12] = CCTNum(0.0);
	m[13] = CCTNum(0.0);
	m[14] = CCTNum(0.0);
	m[15] = CCTNum(1.0);
	return m;
}

CCTNum_t* mathMat44ToQuat(const CCTNum_t m[16], CCTNum_t q[4]) {
	CCTNum_t m33[9];
	return mathMat33ToQuat(mathMat44ToMat33(m, m33), q);
}

CCTNum_t* mathMat33ToQuat(const CCTNum_t m[9], CCTNum_t q[4]) {
	CCTNum_t t, s;
	if (m[8] < CCTNum(0.0)) {
		if (m[0] > m[4]) {
			t = 1 + m[0] - m[4] - m[8];
			q[0] = t;
			q[1] = m[1] + m[3];
			q[2] = m[6] + m[2];
			q[3] = m[5] - m[7];
		}
		else {
			t = 1 - m[0] + m[4] - m[8];
			q[0] = m[1] + m[3];
			q[1] = t;
			q[2] = m[5] + m[7];
			q[3] = m[6] - m[2];
		}
	}
	else {
		if (m[0] < -m[4]) {
			t = 1 - m[0] - m[4] + m[8];
			q[0] = m[6] + m[2];
			q[1] = m[5] + m[7];
			q[2] = t;
			q[3] = m[1] - m[3];
		}
		else {
			t = 1 + m[0] + m[4] + m[8];
			q[0] = m[5] - m[7];
			q[1] = m[6] - m[2];
			q[2] = m[1] - m[3];
			q[4] = t;
		}
	}
	s = CCTNum(0.5) / CCTNum_sqrt(t);
	q[0] *= s;
	q[1] *= s;
	q[2] *= s;
	q[3] *= s;
	return q;
}

CCTNum_t* mathMat33FromQuat(CCTNum_t m[9], const CCTNum_t q[4]) {
	CCTNum_t m44[16];
	return mathMat44ToMat33(mathMat44FromQuat(m44, q), m);
}

#ifdef	__cplusplus
}
#endif
