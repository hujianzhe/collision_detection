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

void mathLineClosestLineOpposite(const CCTNum_t lsv1[3], const CCTNum_t lsdir1[3], const CCTNum_t lsv2[3], const CCTNum_t lsdir2[3], CCTNum_t* lsdir_d1, CCTNum_t* lsdir_d2) {
	CCTNum_t temp[3], n[3], v[3], nlensq_inv;
	mathVec3Sub(v, lsv2, lsv1);
	mathVec3Cross(n, lsdir1, lsdir2);
	nlensq_inv = CCTNum(1.0) / mathVec3LenSq(n);
	*lsdir_d1 = mathVec3Dot(mathVec3Cross(temp, v, lsdir2), n) * nlensq_inv;
	*lsdir_d2 = mathVec3Dot(mathVec3Cross(temp, v, lsdir1), n) * nlensq_inv;
}

CCTNum_t mathLineCrossLine(const CCTNum_t lsv1[3], const CCTNum_t lsdir1[3], const CCTNum_t lsv2[3], const CCTNum_t lsdir2[3]) {
	CCTNum_t v[3], cos_theta, d;
	mathPointProjectionLine(lsv1, lsv2, lsdir2, v);
	if (mathVec3Equal(lsv1, v)) {
		return CCTNum(0.0);
	}
	mathVec3Sub(v, v, lsv1);
	d = mathVec3Normalized(v, v);
	cos_theta = mathVec3Dot(lsdir1, v);
	d /= cos_theta;
	return d;
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
	lensq = mathVec3LenSq(v);
	if (min_lensq > lensq) {
		min_lensq = lensq;
		*ls1_indices = 1;
		*ls2_indices = 0;
	}

	mathVec3Sub(v, ls1[1], ls2[1]);
	lensq = mathVec3LenSq(v);
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

CCTNum_t mathSegmentClosestSegmentDistanceSq(const CCTNum_t ls1[2][3], const CCTNum_t ls1_dir[3], CCTNum_t ls1_len, const CCTNum_t ls2[2][3], const CCTNum_t ls2_dir[3], CCTNum_t ls2_len) {
	CCTNum_t v[3], N[3], d;
	CCTNum_t ls1_dir_temp[3], ls2_dir_temp[3];
	if (!ls1_dir) {
		ls1_dir = ls1_dir_temp;
		mathVec3Sub(ls1_dir_temp, ls1[1], ls1[0]);
	}
	if (!ls2_dir) {
		ls2_dir = ls2_dir_temp;
		mathVec3Sub(ls2_dir_temp, ls2[1], ls2[0]);
	}
	mathVec3Cross(N, ls1_dir, ls2_dir);
	mathVec3Sub(v, ls2[0], ls1[0]);
	if (mathVec3IsZero(N)) {
		unsigned int v_idx[2];
		mathVec3Cross(N, v, ls1_dir);
		if (mathVec3IsZero(N)) {
			/* collinear */
			CCTNum_t l[3], r[3];
			mathVec3Sub(l, ls1[0], ls2[0]);
			mathVec3Sub(r, ls1[1], ls2[0]);
			d = mathVec3Dot(l, r);
			if (d <= CCTNum(0.0)) {
				return CCTNum(0.0);
			}
			mathVec3Sub(l, ls1[0], ls2[1]);
			mathVec3Sub(r, ls1[1], ls2[1]);
			d = mathVec3Dot(l, r);
			if (d <= CCTNum(0.0)) {
				return CCTNum(0.0);
			}
			mathVec3Sub(l, ls2[0], ls1[0]);
			mathVec3Sub(r, ls2[1], ls1[0]);
			d = mathVec3Dot(l, r);
			if (d <= CCTNum(0.0)) {
				return CCTNum(0.0);
			}
			mathVec3Sub(l, ls2[0], ls1[1]);
			mathVec3Sub(r, ls2[1], ls1[1]);
			d = mathVec3Dot(l, r);
			if (d <= CCTNum(0.0)) {
				return CCTNum(0.0);
			}
		}
		else {
			/* parallel */
			int i;
			if (ls1_dir == ls1_dir_temp) {
				ls1_len = mathVec3Normalized(ls1_dir_temp, ls1_dir_temp);
			}
			if (ls2_dir == ls2_dir_temp) {
				ls2_len = mathVec3Normalized(ls2_dir_temp, ls2_dir_temp);
			}
			for (i = 0; i < 2; ++i) {
				mathVec3Sub(v, ls1[i], ls2[0]);
				d = mathVec3Dot(v, ls2_dir);
				if (d < CCTNum(0.0)) {
					continue;
				}
				if (d > ls2_len) {
					continue;
				}
				return mathVec3LenSq(v) - CCTNum_sq(d);
			}
			for (i = 0; i < 2; ++i) {
				mathVec3Sub(v, ls2[i], ls1[0]);
				d = mathVec3Dot(v, ls1_dir);
				if (d < CCTNum(0.0)) {
					continue;
				}
				if (d > ls1_len) {
					continue;
				}
				return mathVec3LenSq(v) - CCTNum_sq(d);
			}
		}
		return mathSegmentSegmentClosestIndices(ls1, ls2, &v_idx[0], &v_idx[1]);
	}
	else {
		int i, lensq_set, set_cnt;
		CCTNum_t lensq;
		if (ls1_dir == ls1_dir_temp) {
			ls1_len = mathVec3Normalized(ls1_dir_temp, ls1_dir_temp);
		}
		if (ls2_dir == ls2_dir_temp) {
			ls2_len = mathVec3Normalized(ls2_dir_temp, ls2_dir_temp);
		}
		d = mathVec3Dot(v, N);
		if (d < CCT_EPSILON_NEGATE || d > CCT_EPSILON) {
			/* opposite */
			CCTNum_t ls1_dir_d, ls2_dir_d;
			mathLineClosestLineOpposite(ls1[0], ls1_dir, ls2[0], ls2_dir, &ls1_dir_d, &ls2_dir_d);
			if ((ls1_dir_d <= ls1_len && ls1_dir_d >= CCTNum(0.0)) &&
				(ls2_dir_d <= ls2_len && ls2_dir_d >= CCTNum(0.0)))
			{
				CCTNum_t closest_p1[3], closest_p2[3];
				mathVec3Copy(closest_p1, ls1[0]);
				mathVec3AddScalar(closest_p1, ls1_dir, ls1_dir_d);
				mathVec3Copy(closest_p2, ls2[0]);
				mathVec3AddScalar(closest_p2, ls2_dir, ls2_dir_d);
				return mathVec3DistanceSq(closest_p1, closest_p2);
			}
		}
		else {
			/* cross */
			CCTNum_t l[3], r[3];
			d = mathLineCrossLine(ls1[0], ls1_dir, ls2[0], ls2_dir);
			mathVec3Copy(v, ls1[0]);
			if (d != CCTNum(0.0)) {
				mathVec3AddScalar(v, ls1_dir, d);
			}
			mathVec3Sub(l, ls1[0], v);
			mathVec3Sub(r, ls1[1], v);
			d = mathVec3Dot(l, r);
			if (d <= CCTNum(0.0)) {
				mathVec3Sub(l, ls2[0], v);
				mathVec3Sub(r, ls2[1], v);
				d = mathVec3Dot(l, r);
				if (d <= CCTNum(0.0)) {
					return CCTNum(0.0);
				}
			}
		}
		lensq_set = 0;
		set_cnt = 0;
		for (i = 0; i < 2; ++i) {
			CCTNum_t l[3], r[3];
			d = mathPointProjectionLine(ls1[i], ls2[0], ls2_dir, v);
			mathVec3Sub(l, ls2[0], v);
			mathVec3Sub(r, ls2[1], v);
			if (mathVec3Dot(l, r) > CCTNum(0.0)) {
				continue;
			}
			set_cnt++;
			d = mathVec3DistanceSq(ls1[i], ls2[0]) - CCTNum_sq(d);
			if (lensq_set && lensq <= d) {
				continue;
			}
			lensq = d;
			lensq_set = 1;
		}
		if (set_cnt >= 2) {
			return lensq;
		}
		set_cnt = 0;
		for (i = 0; i < 2; ++i) {
			CCTNum_t l[3], r[3];
			d = mathPointProjectionLine(ls2[i], ls1[0], ls1_dir, v);
			mathVec3Sub(l, ls1[0], v);
			mathVec3Sub(r, ls1[1], v);
			if (mathVec3Dot(l, r) > CCTNum(0.0)) {
				continue;
			}
			set_cnt++;
			d = mathVec3DistanceSq(ls2[i], ls1[0]) - CCTNum_sq(d);
			if (lensq_set && lensq <= d) {
				continue;
			}
			lensq = d;
			lensq_set = 1;
		}
		if (set_cnt < 2) {
			unsigned int v_idx[2];
			CCTNum_t lensq2 = mathSegmentSegmentClosestIndices(ls1, ls2, &v_idx[0], &v_idx[1]);
			if (lensq_set && lensq < lensq2) {
				return lensq;
			}
			return lensq2;
		}
		return lensq;
	}
}

#ifdef __cplusplus
}
#endif
