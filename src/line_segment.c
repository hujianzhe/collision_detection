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

int mathLineClosestLine(const CCTNum_t lsv1[3], const CCTNum_t lsdir1[3], const CCTNum_t lsv2[3], const CCTNum_t lsdir2[3], CCTNum_t* min_d, CCTNum_t dir_d[2]) {
	CCTNum_t N[3], n[3], v[3], dot, nlen;
	mathVec3Sub(v, lsv2, lsv1);
	mathVec3Cross(n, lsdir1, lsdir2);
	if (mathVec3IsZero(n)) {
		dot = mathVec3Dot(v, lsdir1);
		dot *= dot;
		nlen = mathVec3LenSq(v);
		if (nlen > dot) {
			if (min_d) {
				*min_d = CCTNum_sqrt(nlen - dot);
			}
			return GEOMETRY_LINE_PARALLEL;
		}
		if (min_d) {
			*min_d = CCTNum(0.0);
		}
		return GEOMETRY_LINE_OVERLAP;
	}
	if (mathVec3IsZero(v)) {
		if (min_d) {
			*min_d = CCTNum(0.0);
		}
		if (dir_d) {
			dir_d[0] = dir_d[1] = CCTNum(0.0);
		}
		return GEOMETRY_LINE_CROSS;
	}
	nlen = mathVec3Normalized(N, n);
	dot = mathVec3Dot(v, N);
	dot = CCTNum_abs(dot);
	if (min_d) {
		*min_d = dot;
	}
	if (dir_d) {
		CCTNum_t cross_v[3], nlensq_inv = CCTNum(1.0) / CCTNum_sq(nlen);
		dir_d[0] = mathVec3Dot(mathVec3Cross(cross_v, v, lsdir2), n) * nlensq_inv;
		dir_d[1] = mathVec3Dot(mathVec3Cross(cross_v, v, lsdir1), n) * nlensq_inv;
	}
	return dot > CCT_EPSILON ? GEOMETRY_LINE_SKEW : GEOMETRY_LINE_CROSS;
}

static int mathSegmentIntersectSegmentWhenInSameLine(const CCTNum_t ls1[2][3], const CCTNum_t ls2[2][3], CCTNum_t p[3]) {
	CCTNum_t dot, lsdir1[3], lsdir2[3];
	mathVec3Sub(lsdir1, ls1[1], ls1[0]);
	mathVec3Sub(lsdir2, ls2[1], ls2[0]);
	if (mathVec3Equal(ls1[0], ls2[0])) {
		dot = mathVec3Dot(lsdir2, lsdir1);
		if (dot <= CCT_EPSILON) {
			if (p) {
				mathVec3Copy(p, ls1[0]);
			}
			return GEOMETRY_SEGMENT_CONTACT;
		}
		return GEOMETRY_SEGMENT_OVERLAP;
	}
	else if (mathVec3Equal(ls1[0], ls2[1])) {
		dot = mathVec3Dot(lsdir2, lsdir1);
		if (dot >= CCT_EPSILON) {
			if (p) {
				mathVec3Copy(p, ls1[0]);
			}
			return GEOMETRY_SEGMENT_CONTACT;
		}
		return GEOMETRY_SEGMENT_OVERLAP;
	}
	else if (mathVec3Equal(ls1[1], ls2[0])) {
		dot = mathVec3Dot(lsdir2, lsdir1);
		if (dot >= CCT_EPSILON) {
			if (p) {
				mathVec3Copy(p, ls1[1]);
			}
			return GEOMETRY_SEGMENT_CONTACT;
		}
		return GEOMETRY_SEGMENT_OVERLAP;
	}
	else if (mathVec3Equal(ls1[1], ls2[1])) {
		dot = mathVec3Dot(lsdir2, lsdir1);
		if (dot <= CCT_EPSILON) {
			if (p) {
				mathVec3Copy(p, ls1[1]);
			}
			return GEOMETRY_SEGMENT_CONTACT;
		}
		return GEOMETRY_SEGMENT_OVERLAP;
	}
	else {
		CCTNum_t v1[3], v2[3];
		int i;
		for (i = 0; i < 2; ++i) {
			mathVec3Sub(v1, ls1[0], ls2[i]);
			mathVec3Sub(v2, ls1[1], ls2[i]);
			dot = mathVec3Dot(v1, v2);
			if (dot < CCT_EPSILON_NEGATE) {
				return GEOMETRY_SEGMENT_OVERLAP;
			}
		}
		for (i = 0; i < 2; ++i) {
			mathVec3Sub(v1, ls2[0], ls1[i]);
			mathVec3Sub(v2, ls2[1], ls1[i]);
			dot = mathVec3Dot(v1, v2);
			if (dot < CCT_EPSILON_NEGATE) {
				return GEOMETRY_SEGMENT_OVERLAP;
			}
		}
		return 0;
	}
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

int mathSegmentClosestSegment(const CCTNum_t ls1[2][3], const CCTNum_t ls2[2][3], CCTNum_t closest_p[2][3]) {
	CCTNum_t dir1[3], dir2[3], lslen1, lslen2, dir_d[2];
	int res, i, has_p;
	unsigned int closest_ls1_indice, closest_ls2_indice;
	mathVec3Sub(dir1, ls1[1], ls1[0]);
	mathVec3Sub(dir2, ls2[1], ls2[0]);
	lslen1 = mathVec3Normalized(dir1, dir1);
	lslen2 = mathVec3Normalized(dir2, dir2);
	res = mathLineClosestLine(ls1[0], dir1, ls2[0], dir2, NULL, dir_d);
	if (GEOMETRY_LINE_PARALLEL == res) {
		for (i = 0; i < 2; ++i) {
			CCTNum_t v[3], dot;
			mathVec3Sub(v, ls1[i], ls2[0]);
			dot = mathVec3Dot(v, dir2);
			if (dot >= CCT_EPSILON_NEGATE && dot <= lslen2 + CCT_EPSILON) {
				mathVec3Copy(closest_p[0], ls1[i]);
				mathVec3Copy(closest_p[1], ls2[0]);
				mathVec3AddScalar(closest_p[1], dir2, dot);
				return 0;
			}
		}
	}
	else if (GEOMETRY_LINE_CROSS == res || GEOMETRY_LINE_SKEW == res) {
		if (dir_d[0] >= CCT_EPSILON && dir_d[0] <= lslen1 + CCT_EPSILON &&
			dir_d[1] >= CCT_EPSILON && dir_d[1] <= lslen2 + CCT_EPSILON)
		{
			mathVec3Copy(closest_p[0], ls1[0]);
			mathVec3AddScalar(closest_p[0], dir1, dir_d[0]);
			mathVec3Copy(closest_p[1], ls2[0]);
			mathVec3AddScalar(closest_p[1], dir2, dir_d[1]);
			return GEOMETRY_LINE_CROSS == res ? GEOMETRY_SEGMENT_CONTACT : 0;
		}
		has_p = 0;
		for (i = 0; i < 2; ++i) {
			CCTNum_t v[3], dot;
			mathVec3Sub(v, ls1[i], ls2[0]);
			dot = mathVec3Dot(v, dir2);
			if (dot < CCT_EPSILON_NEGATE || dot > lslen2 + CCT_EPSILON) {
				continue;
			}
			if (has_p) {
				CCTNum_t dq = mathVec3LenSq(v) - CCTNum_sq(dot);
				mathVec3Sub(v, closest_p[1], closest_p[0]);
				if (dq >= mathVec3LenSq(v)) {
					continue;
				}
			}
			has_p = 1;
			mathVec3Copy(closest_p[0], ls1[i]);
			mathVec3Copy(closest_p[1], ls2[0]);
			mathVec3AddScalar(closest_p[1], dir2, dot);
		}
		for (i = 0; i < 2; ++i) {
			CCTNum_t v[3], dot;
			mathVec3Sub(v, ls2[i], ls1[0]);
			dot = mathVec3Dot(v, dir1);
			if (dot < CCT_EPSILON_NEGATE || dot > lslen1 + CCT_EPSILON) {
				continue;
			}
			if (has_p) {
				CCTNum_t dq = mathVec3LenSq(v) - CCTNum_sq(dot);
				mathVec3Sub(v, closest_p[1], closest_p[0]);
				if (dq >= mathVec3LenSq(v)) {
					continue;
				}
			}
			has_p = 1;
			mathVec3Copy(closest_p[1], ls2[i]);
			mathVec3Copy(closest_p[0], ls1[0]);
			mathVec3AddScalar(closest_p[0], dir1, dot);
		}
		if (has_p) {
			return 0;
		}
	}
	else {
		CCTNum_t p[3];
		res = mathSegmentIntersectSegmentWhenInSameLine(ls1, ls2, p);
		if (GEOMETRY_SEGMENT_CONTACT == res) {
			mathVec3Copy(closest_p[0], p);
			mathVec3Copy(closest_p[1], p);
			return res;
		}
		if (GEOMETRY_SEGMENT_OVERLAP == res) {
			return res;
		}
	}
	mathSegmentSegmentClosestIndices(ls1, ls2, &closest_ls1_indice, &closest_ls2_indice);
	mathVec3Copy(closest_p[0], ls1[closest_ls1_indice]);
	mathVec3Copy(closest_p[1], ls2[closest_ls2_indice]);
	return 0;
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

int Segment_Intersect_Segment(const CCTNum_t ls1[2][3], const CCTNum_t ls2[2][3], CCTNum_t p[3], int* line_mask) {
	int res;
	CCTNum_t lsdir1[3], lsdir2[3], dir_d[2], lslen1, lslen2;
	mathVec3Sub(lsdir1, ls1[1], ls1[0]);
	mathVec3Sub(lsdir2, ls2[1], ls2[0]);
	lslen1 = mathVec3Normalized(lsdir1, lsdir1);
	lslen2 = mathVec3Normalized(lsdir2, lsdir2);
	res = mathLineClosestLine(ls1[0], lsdir1, ls2[0], lsdir2, NULL, dir_d);
	if (line_mask) {
		*line_mask = res;
	}
	if (GEOMETRY_LINE_PARALLEL == res || GEOMETRY_LINE_SKEW == res) {
		return 0;
	}
	if (GEOMETRY_LINE_CROSS == res) {
		if (dir_d[0] < CCT_EPSILON_NEGATE || dir_d[1] < CCT_EPSILON_NEGATE ||
			dir_d[0] > lslen1 + CCT_EPSILON || dir_d[1] > lslen2 + CCT_EPSILON)
		{
			return 0;
		}
		if (p) {
			mathVec3Copy(p, ls1[0]);
			mathVec3AddScalar(p, lsdir1, dir_d[0]);
		}
		return GEOMETRY_SEGMENT_CONTACT;
	}
	return mathSegmentIntersectSegmentWhenInSameLine(ls1, ls2, p);
}

#ifdef __cplusplus
}
#endif
