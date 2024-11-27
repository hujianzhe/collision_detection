//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/gjk.h"

static void indices_support(const CCTNum_t(*v)[3], const unsigned int* v_indices, unsigned int v_indices_cnt, const CCTNum_t dir[3], CCTNum_t pp[3]) {
	unsigned int i, max_v_idx = v_indices[0];
	CCTNum_t max_d = mathVec3Dot(v[max_v_idx], dir);
	for (i = 1; i < v_indices_cnt; ++i) {
		unsigned int v_idx = v_indices[i];
		CCTNum_t d = mathVec3Dot(v[v_idx], dir);
		if (d > max_d) {
			max_d = d;
			max_v_idx = v_idx;
		}
	}
	mathVec3Copy(pp, v[max_v_idx]);
}

static void vertices_support(const CCTNum_t(*v)[3], unsigned int v_cnt, const CCTNum_t dir[3], CCTNum_t pp[3]) {
	unsigned int i, max_v_idx = 0;
	CCTNum_t max_d = mathVec3Dot(v[max_v_idx], dir);
	for (i = 1; i < v_cnt; ++i) {
		CCTNum_t d = mathVec3Dot(v[i], dir);
		if (d > max_d) {
			max_d = d;
			max_v_idx = i;
		}
	}
	mathVec3Copy(pp, v[max_v_idx]);
}

static void gjk_sub_point(const GeometryConvexGJK_t* geo1, const GeometryConvexGJK_t* geo2, const CCTNum_t dir[3], CCTNum_t sub_p[3]) {
	CCTNum_t neg_dir[3], p1[3], p2[3];
	mathVec3Negate(neg_dir, dir);

	if (geo1->v_indices) {
		indices_support(geo1->v, geo1->v_indices, geo1->v_indices_cnt, dir, p1);
	}
	else {
		vertices_support(geo1->v, geo1->v_cnt, dir, p1);
	}
	if (geo2->v_indices) {
		indices_support(geo2->v, geo2->v_indices, geo2->v_indices_cnt, neg_dir, p2);
	}
	else {
		vertices_support(geo2->v, geo2->v_cnt, neg_dir, p2);
	}
	mathVec3Sub(sub_p, p1, p2);
}

typedef struct Simplex_t {
	CCTNum_t p[4][3];
	CCTNum_t dir[3];
	unsigned int cnt;
} Simplex_t;

static int simplex2(const CCTNum_t a[3], const CCTNum_t b[3], CCTNum_t dir[3]) {
	CCTNum_t N[3], ls_v[3];
	mathVec3Cross(N, a, b);
	if (mathVec3IsZero(N)) {
		return 1;
	}
	mathVec3Sub(ls_v, b, a);
	mathVec3Cross(dir, N, ls_v);
	if (mathVec3Dot(a, dir) > CCTNum(0.0)) {
		mathVec3Negate(dir, dir);
	}
	return 0;
}

static int simplex3(const CCTNum_t a[3], const CCTNum_t b[3], const CCTNum_t c[3], Simplex_t* s) {
	CCTNum_t plane_N[3], ca[3], cb[3], ca_N[3], cb_N[3];
	CCTNum_t dot;

	mathVec3Sub(ca, a, c);
	mathVec3Sub(cb, b, c);
	mathVec3Cross(plane_N, ca, cb);

	mathVec3Cross(ca_N, plane_N, ca);
	if (mathVec3Dot(cb, ca_N) < CCTNum(0.0)) {
		mathVec3Negate(ca_N, ca_N);
	}
	mathVec3Cross(cb_N, plane_N, cb);
	if (mathVec3Dot(ca, cb_N) < CCTNum(0.0)) {
		mathVec3Negate(cb_N, cb_N);
	}

	dot = mathVec3Dot(c, ca_N);
	if (dot > CCTNum(0.0)) {
		mathVec3Copy(s->p[1], c);
		s->cnt = 2;
		mathVec3Negate(s->dir, ca_N);
		return 0;
	}

	dot = mathVec3Dot(c, cb_N);
	if (dot > CCTNum(0.0)) {
		mathVec3Copy(s->p[0], b);
		mathVec3Copy(s->p[1], c);
		s->cnt = 2;
		mathVec3Negate(s->dir, cb_N);
		return 0;
	}

	dot = mathVec3Dot(c, plane_N);
	if (dot > CCT_EPSILON) {
		mathVec3Negate(s->dir, plane_N);
		return 0;
	}
	if (dot < CCT_EPSILON_NEGATE) {
		mathVec3Copy(s->dir, plane_N);
		return 0;
	}
	return 1;
}

static int simplex4(const CCTNum_t a[3], const CCTNum_t b[3], const CCTNum_t c[3], const CCTNum_t d[3], Simplex_t* s) {
	CCTNum_t N[3], e1[3], e2[3], v[3], dot;

	mathVec3Sub(e1, a, d);
	mathVec3Sub(e2, b, d);
	mathVec3Cross(N, e1, e2);
	mathVec3Sub(v, c, a);
	if (mathVec3Dot(v, N) < CCTNum(0.0)) {
		mathVec3Negate(N, N);
	}
	dot = mathVec3Dot(d, N);
	if (dot > CCT_EPSILON) {
		mathVec3Copy(s->p[2], d);
		s->cnt = 3;
		mathVec3Negate(s->dir, N);
		return 0;
	}

	mathVec3Sub(e1, b, d);
	mathVec3Sub(e2, c, d);
	mathVec3Cross(N, e1, e2);
	mathVec3Sub(v, a, c);
	if (mathVec3Dot(v, N) < CCTNum(0.0)) {
		mathVec3Negate(N, N);
	}
	dot = mathVec3Dot(d, N);
	if (dot > CCT_EPSILON) {
		mathVec3Copy(s->p[0], b);
		mathVec3Copy(s->p[1], c);
		mathVec3Copy(s->p[2], d);
		s->cnt = 3;
		mathVec3Negate(s->dir, N);
		return 0;
	}

	mathVec3Sub(e1, a, d);
	mathVec3Sub(e2, c, d);
	mathVec3Cross(N, e1, e2);
	mathVec3Sub(v, b, a);
	if (mathVec3Dot(v, N) < CCTNum(0.0)) {
		mathVec3Negate(N, N);
	}
	dot = mathVec3Dot(d, N);
	if (dot > CCT_EPSILON) {
		mathVec3Copy(s->p[1], c);
		mathVec3Copy(s->p[2], d);
		s->cnt = 3;
		mathVec3Negate(s->dir, N);
		return 0;
	}

	return 1;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
extern "C" {
#endif

int mathGJK(const GeometryConvexGJK_t* geo1, const GeometryConvexGJK_t* geo2, const CCTNum_t dir[3]) {
	unsigned int max_iterator_times = (geo1->v_cnt > geo2->v_cnt ? geo1->v_cnt : geo2->v_cnt);
	Simplex_t s;

	if (!dir || mathVec3IsZero(dir)) {
		mathVec3Set(s.dir, CCTNums_3(1.0, 0.0, 0.0));
		dir = s.dir;
	}
	gjk_sub_point(geo1, geo2, dir, s.p[0]);
	s.cnt = 1;
	mathVec3Negate(s.dir, s.p[0]);
	while (max_iterator_times--) {
		gjk_sub_point(geo1, geo2, s.dir, s.p[s.cnt]);
		if (mathVec3Dot(s.p[s.cnt], s.dir) < CCTNum(0.0)) {
			return 0;
		}
		s.cnt++;
		if (2 == s.cnt) {
			if (simplex2(s.p[0], s.p[1], s.dir)) {
				return 1;
			}
		}
		else if (3 == s.cnt) {
			if (simplex3(s.p[0], s.p[1], s.p[2], &s)) {
				return 1;
			}
		}
		else if (4 == s.cnt) {
			if (simplex4(s.p[0], s.p[1], s.p[2], s.p[3], &s)) {
				return 1;
			}
		}
	}
	return 0;
}

#ifdef __cplusplus
}
#endif
