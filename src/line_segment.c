//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/line_segment.h"
#include <math.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

const unsigned int Segment_Indices_Default[2] = { 0, 1 };

CCTNum_t mathPointProjectionLine(const CCTNum_t p[3], const CCTNum_t ls_v[3], const CCTNum_t lsdir[3], CCTNum_t np[3]) {
	CCTNum_t vp[3], dot;
	mathVec3Sub(vp, p, ls_v);
	dot = mathVec3Dot(vp, lsdir);
	mathVec3AddScalar(mathVec3Copy(np, ls_v), lsdir, dot);
	return dot;
}

static void mathLineClosestLine(const CCTNum_t lsv1[3], const CCTNum_t lsdir1[3], const CCTNum_t lsv2[3], const CCTNum_t lsdir2[3], CCTNum_t dir_d[2]) {
	CCTNum_t temp[3], n[3], v[3], nlensq_inv;
	mathVec3Sub(v, lsv2, lsv1);
	mathVec3Cross(n, lsdir1, lsdir2);
	nlensq_inv = CCTNum(1.0) / mathVec3LenSq(n);
	dir_d[0] = mathVec3Dot(mathVec3Cross(temp, v, lsdir2), n) * nlensq_inv;
	dir_d[1] = mathVec3Dot(mathVec3Cross(temp, v, lsdir1), n) * nlensq_inv;
}

CCTNum_t mathSegmentSegmentClosestIndices(const CCTNum_t ls1[2][3], const CCTNum_t ls2[2][3], unsigned int* ls1_indices, unsigned int* ls2_indices) {
	CCTNum_t lensq, min_lensq, v[3];

	mathVec3Sub(v, ls1[0], ls2[0]);
	min_lensq = mathVec3LenSq(v);
	*ls1_indices = 0;
	*ls2_indices = 0;

	mathVec3Sub(v, ls1[0], ls2[1]);
	lensq = mathVec3LenSq(v);
	if (min_lensq > lensq) {
		min_lensq = lensq;
		*ls1_indices = 0;
		*ls2_indices = 1;
	}

	mathVec3Sub(v, ls1[1], ls2[0]);
	lensq = mathVec3Len(v);
	if (min_lensq > lensq) {
		min_lensq = lensq;
		*ls1_indices = 1;
		*ls2_indices = 0;
	}

	mathVec3Sub(v, ls1[1], ls2[1]);
	lensq = mathVec3Len(v);
	if (min_lensq > lensq) {
		min_lensq = lensq;
		*ls1_indices = 1;
		*ls2_indices = 1;
	}

	return min_lensq;
}

int mathSegmentIsSame(const CCTNum_t ls1[2][3], const CCTNum_t ls2[2][3]) {
	if (mathVec3Equal(ls1[0], ls2[0])) {
		return mathVec3Equal(ls1[1], ls2[1]);
	}
	if (mathVec3Equal(ls1[0], ls2[1])) {
		return mathVec3Equal(ls1[1], ls2[0]);
	}
	return 0;
}

void mathSegmentClosestPointTo(const CCTNum_t ls[2][3], const CCTNum_t p[3], CCTNum_t closest_p[3]) {
	CCTNum_t lsdir[3], v[3], lslen, dot;
	mathVec3Sub(v, p, ls[0]);
	mathVec3Sub(lsdir, ls[1], ls[0]);
	dot = mathVec3Dot(v, lsdir);
	if (dot < CCTNum(0.0)) {
		mathVec3Copy(closest_p, ls[0]);
		return;
	}
	lslen = mathVec3Normalized(lsdir, lsdir);
	dot /= lslen;
	if (dot > lslen) {
		mathVec3Copy(closest_p, ls[1]);
		return;
	}
	mathVec3Copy(closest_p, ls[0]);
	mathVec3AddScalar(closest_p, lsdir, dot);
}

void mathSegmentClosestPointTo_v2(const CCTNum_t ls_center_p[3], const CCTNum_t lsdir[3], const CCTNum_t ls_half_len, const CCTNum_t p[3], CCTNum_t closest_p[3]) {
	CCTNum_t v[3], d, abs_d;
	mathVec3Sub(v, p, ls_center_p);
	d = mathVec3Dot(v, lsdir);
	abs_d = CCTNum_abs(d);
	mathVec3Copy(closest_p, ls_center_p);
	if (abs_d > ls_half_len) {
		if (d > CCTNum(0.0)) {
			mathVec3AddScalar(closest_p, lsdir, ls_half_len);
		}
		else {
			mathVec3SubScalar(closest_p, lsdir, ls_half_len);
		}
		return;
	}
	mathVec3AddScalar(closest_p, lsdir, d);
}

#ifdef __cplusplus
}
#endif
