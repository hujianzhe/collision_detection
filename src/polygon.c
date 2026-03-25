//
// Created by hujianzhe
//

#include "../inc/const_data.h"
#include "../inc/math_vec3.h"
#include "../inc/vertex.h"
#include "../inc/plane.h"
#include "../inc/polygon.h"
#include <stdlib.h>

extern const CCTConstVal_t CCTConstVal_;

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void Polygon_ClearWithoutVertices(GeometryPolygon_t* polygon, const CCTAllocator_t* ac) {
	if (!polygon) {
		return;
	}
	if (polygon->edge_v_ids_flat) {
		ac->fn_free(ac, (void*)polygon->edge_v_ids_flat);
		polygon->edge_v_ids_flat = NULL;
	}
	if (polygon->edge_v_indices_flat) {
		ac->fn_free(ac, (void*)polygon->edge_v_indices_flat);
		polygon->edge_v_indices_flat = NULL;
	}
	polygon->edge_cnt = 0;
	if (polygon->tri_v_indices_flat) {
		ac->fn_free(ac, (void*)polygon->tri_v_indices_flat);
		polygon->tri_v_indices_flat = NULL;
	}
	if (polygon->concave_tri_v_ids_flat) {
		ac->fn_free(ac, (void*)polygon->concave_tri_v_ids_flat);
		polygon->concave_tri_v_ids_flat = NULL;
	}
	if (polygon->concave_tri_edge_ids_flat) {
		ac->fn_free(ac, (void*)polygon->concave_tri_edge_ids_flat);
		polygon->concave_tri_edge_ids_flat = NULL;
	}
	polygon->tri_cnt = 0;
	if (polygon->v_indices) {
		ac->fn_free(ac, (void*)polygon->v_indices);
		polygon->v_indices = NULL;
		polygon->v_indices_cnt = 0;
	}
	if (polygon->mesh_v_ids) {
		ac->fn_free(ac, (void*)polygon->mesh_v_ids);
		polygon->mesh_v_ids = NULL;
	}
	if (polygon->mesh_edge_ids) {
		ac->fn_free(ac, (void*)polygon->mesh_edge_ids);
		polygon->mesh_edge_ids = NULL;
	}
	if (polygon->v_adjacent_infos) {
		ac->fn_free(ac, (void*)polygon->v_adjacent_infos);
		polygon->v_adjacent_infos = NULL;
	}
}

int Polygon_IsConvex(const CCTNum_t(*v)[3], const CCTNum_t normal[3], const unsigned int* edge_v_indices_flat, unsigned int edge_v_indices_cnt, const unsigned int* v_indices, unsigned int v_indices_cnt) {
	unsigned int i;
	if (v_indices_cnt < 3) {
		return 0;
	}
	for (i = 0; i < edge_v_indices_cnt; ) {
		CCTNum_t ls_v[3], N[3];
		int flag_sign = 0;
		unsigned int j, v_idx[2];
		v_idx[0] = edge_v_indices_flat[i++];
		v_idx[1] = edge_v_indices_flat[i++];
		mathVec3Sub(ls_v, v[v_idx[1]], v[v_idx[0]]);
		mathVec3Cross(N, ls_v, normal);
		for (j = 0; j < v_indices_cnt; ++j) {
			CCTNum_t test_v[3], dot;
			if (v_indices[j] == v_idx[0] || v_indices[j] == v_idx[1]) {
				continue;
			}
			mathVec3Sub(test_v, v[v_indices[j]], v[v_idx[0]]);
			dot = mathVec3Dot(test_v, N);
			/* some module needed epsilon */
			if (dot > CCT_EPSILON) {
				if (flag_sign < 0) {
					return 0;
				}
				flag_sign = 1;
			}
			else if (dot < CCT_EPSILON_NEGATE) {
				if (flag_sign > 0) {
					return 0;
				}
				flag_sign = -1;
			}
		}
	}
	return 1;
}

#ifdef	__cplusplus
extern "C" {
#endif

void mathTriangleGetPoint(const CCTNum_t tri_p0[3], const CCTNum_t tri_p1[3], const CCTNum_t tri_p2[3], CCTNum_t u, CCTNum_t v, CCTNum_t p[3]) {
	CCTNum_t v0[3], v1[3], v2[3];
	mathVec3MultiplyScalar(v0, tri_p0, CCTNum(1.0) - u - v);
	mathVec3MultiplyScalar(v1, tri_p1, u);
	mathVec3MultiplyScalar(v2, tri_p2, v);
	mathVec3Add(p, v0, v1);
	mathVec3Add(p, p, v2);
}

int mathTrianglePointUV(const CCTNum_t tri_p0[3], const CCTNum_t tri_p1[3], const CCTNum_t tri_p2[3], const CCTNum_t p[3], CCTNum_t* p_u, CCTNum_t* p_v) {
	CCTNum_t ap[3], ab[3], ac[3], N[3], dot;
	mathVec3Sub(ap, p, tri_p0);
	mathVec3Sub(ab, tri_p1, tri_p0);
	mathVec3Sub(ac, tri_p2, tri_p0);
	mathVec3Cross(N, ab, ac);
	dot = mathVec3Dot(N, ap);
	if (dot > CCT_EPSILON || dot < CCT_EPSILON_NEGATE) {
		return 0;
	}
	else {
		CCTNum_t u, v;
		CCTNum_t dot_ac_ac = mathVec3Dot(ac, ac);
		CCTNum_t dot_ac_ab = mathVec3Dot(ac, ab);
		CCTNum_t dot_ac_ap = mathVec3Dot(ac, ap);
		CCTNum_t dot_ab_ab = mathVec3Dot(ab, ab);
		CCTNum_t dot_ab_ap = mathVec3Dot(ab, ap);
		CCTNum_t inv = CCTNum(1.0) / (dot_ac_ac * dot_ab_ab - dot_ac_ab * dot_ac_ab);
		u = (dot_ab_ab * dot_ac_ap - dot_ac_ab * dot_ab_ap) * inv;
		if (u < CCTNum(0.0) || u > CCTNum(1.0)) {
			return 0;
		}
		v = (dot_ac_ac * dot_ab_ap - dot_ac_ab * dot_ac_ap) * inv;
		if (v < CCTNum(0.0) || v + u > CCTNum(1.0)) {
			return 0;
		}
		if (p_u) {
			*p_u = u;
		}
		if (p_v) {
			*p_v = v;
		}
		return 1;
	}
}

void mathPolygonEdgeNormalOuter(const GeometryPolygon_t* polygon, unsigned int edge_id, CCTNum_t edge_normal[3]) {
	CCTNum_t v[3];
	unsigned int i = edge_id + edge_id;
	unsigned int v_idx[2];
	v_idx[0] = polygon->edge_v_indices_flat[i++];
	v_idx[1] = polygon->edge_v_indices_flat[i++];
	if (i >= polygon->edge_cnt + polygon->edge_cnt) {
		i = 0;
	}
	if (polygon->edge_v_indices_flat[i] == v_idx[0] || polygon->edge_v_indices_flat[i] == v_idx[1]) {
		++i;
	}
	mathVec3Sub(v, polygon->v[v_idx[1]], polygon->v[v_idx[0]]);
	mathVec3Cross(edge_normal, v, polygon->normal);
	mathVec3Sub(v, polygon->v[polygon->edge_v_indices_flat[i]], polygon->v[v_idx[0]]);
	if (mathVec3Dot(edge_normal, v) > CCTNum(0.0)) {
		mathVec3Negate(edge_normal, edge_normal);
	}
}

#ifdef	__cplusplus
}
#endif
