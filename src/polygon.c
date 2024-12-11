//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/vertex.h"
#include "../inc/plane.h"
#include "../inc/polygon.h"
#include <stdlib.h>

static const unsigned int Triangle_Vertice_Indices_Default[3] = { 0, 1, 2 };
static const unsigned int Triangle_Edge_Indices_Default[6] = { 0,1, 1,2, 2,0 };

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

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

void mathTriangleToPolygon(const CCTNum_t tri[3][3], GeometryPolygon_t* polygon) {
	polygon->v = (CCTNum_t(*)[3])tri;
	polygon->v_indices = Triangle_Vertice_Indices_Default;
	polygon->v_indices_cnt = 3;
	polygon->edge_indices = Triangle_Edge_Indices_Default;
	polygon->edge_indices_cnt = 6;
	polygon->tri_indices = Triangle_Vertice_Indices_Default;
	polygon->tri_indices_cnt = 3;
	polygon->is_convex = 1;
	mathPlaneNormalByVertices3(tri[0], tri[1], tri[2], polygon->normal);
	mathVec3Set(polygon->o, CCTNums_3(0.0, 0.0, 0.0));
	mathVertexIndicesAverageXYZ((const CCTNum_t(*)[3])polygon->v, polygon->v_indices, polygon->v_indices_cnt, polygon->center);
}

int mathPolygonIsConvex(const GeometryPolygon_t* polygon) {
	unsigned int i;
	if (polygon->v_indices_cnt < 3) {
		return 0;
	}
	for (i = 0; i < polygon->edge_indices_cnt; ) {
		CCTNum_t ls_v[3], N[3];
		int flag_sign = 0;
		unsigned int j, v_idx[2];
		v_idx[0] = polygon->edge_indices[i++];
		v_idx[1] = polygon->edge_indices[i++];
		mathVec3Sub(ls_v, polygon->v[v_idx[1]], polygon->v[v_idx[0]]);
		mathVec3Cross(N, ls_v, polygon->normal);
		for (j = 0; j < polygon->v_indices_cnt; ++j) {
			CCTNum_t v[3], dot;
			if (polygon->v_indices[j] == v_idx[0] || polygon->v_indices[j] == v_idx[1]) {
				continue;
			}
			mathVec3Sub(v, polygon->v[polygon->v_indices[j]], polygon->v[v_idx[0]]);
			dot = mathVec3Dot(v, N);
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

GeometryPolygon_t* mathPolygonDeepCopy(GeometryPolygon_t* dst, const GeometryPolygon_t* src) {
	unsigned int i, dup_v_cnt = 0;
	CCTNum_t(*dup_v)[3] = NULL;
	unsigned int* dup_v_indices = NULL;
	unsigned int* dup_tri_indices = NULL;
	unsigned int* dup_edge_indices = NULL;
	/* find max vertex index, dup_v_cnt */
	for (i = 0; i < src->v_indices_cnt; ++i) {
		if (src->v_indices[i] >= dup_v_cnt) {
			dup_v_cnt = src->v_indices[i] + 1;
		}
	}
	/* deep copy */
	dup_v = (CCTNum_t(*)[3])malloc(sizeof(dup_v[0]) * dup_v_cnt);
	if (!dup_v) {
		goto err_0;
	}
	dup_v_indices = (unsigned int*)malloc(sizeof(dup_v_indices[0]) * dup_v_cnt);
	if (!dup_v_indices) {
		goto err_0;
	}
	dup_tri_indices = (unsigned int*)malloc(sizeof(dup_tri_indices[0]) * src->tri_indices_cnt);
	if (!dup_tri_indices) {
		goto err_0;
	}
	dup_edge_indices = (unsigned int*)malloc(sizeof(dup_edge_indices[0]) * src->edge_indices_cnt);
	if (!dup_edge_indices) {
		goto err_0;
	}
	for (i = 0; i < src->v_indices_cnt; ++i) {
		unsigned int idx = src->v_indices[i];
		dup_v_indices[i] = idx;
		mathVec3Copy(dup_v[idx], src->v[idx]);
	}
	for (i = 0; i < src->tri_indices_cnt; ++i) {
		dup_tri_indices[i] = src->tri_indices[i];
	}
	for (i = 0; i < src->edge_indices_cnt; ++i) {
		dup_edge_indices[i] = src->edge_indices[i];
	}
	mathVec3Copy(dst->o, src->o);
	mathVec3Copy(dst->center, src->center);
	mathVec3Copy(dst->normal, src->normal);
	dst->tri_indices_cnt = src->tri_indices_cnt;
	dst->v_indices_cnt = src->v_indices_cnt;
	dst->edge_indices_cnt = src->edge_indices_cnt;
	dst->v = dup_v;
	dst->v_indices = dup_v_indices;
	dst->tri_indices = dup_tri_indices;
	dst->edge_indices = dup_edge_indices;
	dst->is_convex = src->is_convex;
	return dst;
err_0:
	free(dup_v);
	free(dup_v_indices);
	free(dup_tri_indices);
	free(dup_edge_indices);
	return NULL;
}

void mathPolygonFreeData(GeometryPolygon_t* polygon) {
	if (!polygon) {
		return;
	}
	if (polygon->edge_indices) {
		free((void*)polygon->edge_indices);
		polygon->edge_indices = NULL;
		polygon->edge_indices_cnt = 0;
	}
	if (polygon->tri_indices) {
		free((void*)polygon->tri_indices);
		polygon->tri_indices = NULL;
		polygon->tri_indices_cnt = 0;
	}
	if (polygon->v_indices) {
		free((void*)polygon->v_indices);
		polygon->v_indices = NULL;
		polygon->v_indices_cnt = 0;
	}
	if (polygon->v) {
		free(polygon->v);
		polygon->v = NULL;
	}
}

void mathPolygonEdgeNormalOuter(const GeometryPolygon_t* polygon, unsigned int edge_idx, CCTNum_t edge_normal[3]) {
	CCTNum_t v[3];
	unsigned int i = edge_idx * 2;
	unsigned int v_idx[2];
	v_idx[0] = polygon->edge_indices[i++];
	v_idx[1] = polygon->edge_indices[i++];
	if (i >= polygon->edge_indices_cnt) {
		i = 0;
	}
	if (polygon->edge_indices[i] == v_idx[0] || polygon->edge_indices[i] == v_idx[1]) {
		i++;
	}
	mathVec3Sub(v, polygon->v[v_idx[1]], polygon->v[v_idx[0]]);
	mathVec3Cross(edge_normal, v, polygon->normal);
	mathVec3Sub(v, polygon->v[polygon->edge_indices[i]], polygon->v[v_idx[0]]);
	if (mathVec3Dot(edge_normal, v) > CCTNum(0.0)) {
		mathVec3Negate(edge_normal, edge_normal);
	}
}

#ifdef	__cplusplus
}
#endif
