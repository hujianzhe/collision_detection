//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/vertex.h"
#include "../inc/plane.h"
#include "../inc/polygon.h"
#include <stdlib.h>

extern int Plane_Contain_Point(const CCTNum_t plane_v[3], const CCTNum_t plane_normal[3], const CCTNum_t p[3]);

static const unsigned int Triangle_Vertice_Indices_Default[3] = { 0, 1, 2 };

GeometryPolygon_t* PolygonCooking_InternalProc(const CCTNum_t(*v)[3], const unsigned int* tri_indices, unsigned int tri_indices_cnt, GeometryPolygon_t* polygon) {
	unsigned int i, s, n, p, last_s, first_s;
	unsigned int* tmp_edge_pair_indices = NULL;
	unsigned int tmp_edge_pair_indices_cnt = 0;
	unsigned int* tmp_edge_indices = NULL;
	unsigned int tmp_edge_indices_cnt = 0;
	unsigned int* ret_v_indices = NULL;
	unsigned int ret_v_indices_cnt = 0;
	CCTNum_t v1[3], v2[3], N[3];

	if (tri_indices_cnt < 3 || tri_indices_cnt % 3 != 0) {
		return NULL;
	}
	mathPlaneNormalByVertices3(v[tri_indices[0]], v[tri_indices[1]], v[tri_indices[2]], N);
	if (mathVec3IsZero(N)) {
		return NULL;
	}
	mathVec3Copy(polygon->normal, N);
	if (tri_indices_cnt == 3) {
		ret_v_indices = (unsigned int*)malloc(3 * sizeof(ret_v_indices[0]));
		if (!ret_v_indices) {
			return NULL;
		}
		ret_v_indices[0] = tri_indices[0];
		ret_v_indices[1] = tri_indices[1];
		ret_v_indices[2] = tri_indices[2];

		polygon->v_indices = ret_v_indices;
		polygon->v_indices_cnt = 3;
		return polygon;
	}
	/* Filters all share triangle edges, leaving all non-shared edges */
	for (i = 0; i < tri_indices_cnt; i += 3) {
		unsigned int ei[6] = {
			tri_indices[i], tri_indices[i + 1],
			tri_indices[i + 1], tri_indices[i + 2],
			tri_indices[i + 2], tri_indices[i]
		};
		int same[3] = { 0 };
		unsigned int j;
		for (j = 0; j < tri_indices_cnt; j += 3) {
			unsigned int k;
			unsigned int ej[6];
			if (i == j) {
				continue;
			}
			for (k = 0; k < 3; ++k) {
				if (!Plane_Contain_Point(v[tri_indices[i]], N, v[tri_indices[j + k]])) {
					free(tmp_edge_pair_indices);
					return NULL;
				}
			}
			ej[0] = tri_indices[j]; ej[1] = tri_indices[j + 1];
			ej[2] = tri_indices[j + 1]; ej[3] = tri_indices[j + 2];
			ej[4] = tri_indices[j + 2]; ej[5] = tri_indices[j];
			for (k = 0; k < 6; k += 2) {
				unsigned int m;
				if (same[k >> 1]) {
					continue;
				}
				for (m = 0; m < 6; m += 2) {
					if (ei[k] == ej[m] || mathVec3Equal(v[ei[k]], v[ej[m]])) {
						same[k >> 1] = (ei[k + 1] == ej[m + 1] || mathVec3Equal(v[ei[k + 1]], v[ej[m + 1]]));
					}
					else if (ei[k] == ej[m + 1] || mathVec3Equal(v[ei[k]], v[ej[m + 1]])) {
						same[k >> 1] = (ei[k + 1] == ej[m] || mathVec3Equal(v[ei[k + 1]], v[ej[m]]));
					}
					else {
						continue;
					}
					if (same[k >> 1]) {
						break;
					}
				}
			}
			if (same[0] && same[1] && same[2]) {
				break;
			}
		}
		if (j != tri_indices_cnt) {
			continue;
		}
		if (!same[0] && !same[1] && !same[2]) {
			free(tmp_edge_pair_indices);
			return NULL;
		}
		for (j = 0; j < 6; j += 2) {
			unsigned int* ptr;
			if (same[j >> 1]) {
				continue;
			}
			tmp_edge_pair_indices_cnt += 2;
			ptr = (unsigned int*)realloc(tmp_edge_pair_indices, tmp_edge_pair_indices_cnt * sizeof(tmp_edge_pair_indices[0]));
			if (!ptr) {
				free(tmp_edge_pair_indices);
				return NULL;
			}
			tmp_edge_pair_indices = ptr;
			tmp_edge_pair_indices[tmp_edge_pair_indices_cnt - 2] = ei[j];
			tmp_edge_pair_indices[tmp_edge_pair_indices_cnt - 1] = ei[j + 1];
		}
	}
	if (!tmp_edge_pair_indices) {
		return NULL;
	}
	/* Calculates the order of edge vertex traversal */
	tmp_edge_indices = (unsigned int*)malloc(sizeof(tmp_edge_indices[0]) * tmp_edge_pair_indices_cnt);
	if (!tmp_edge_indices) {
		free(tmp_edge_pair_indices);
		return NULL;
	}
	s = tmp_edge_pair_indices[0];
	n = tmp_edge_pair_indices[1];
	tmp_edge_indices[tmp_edge_indices_cnt++] = s;
	tmp_edge_indices[tmp_edge_indices_cnt++] = n;
	p = 0;
	for (i = 2; i < tmp_edge_pair_indices_cnt; i += 2) {
		if (i == p) {
			continue;
		}
		if (n == tmp_edge_pair_indices[i] || mathVec3Equal(v[n], v[tmp_edge_pair_indices[i]])) {
			n = tmp_edge_pair_indices[i + 1];
		}
		else if (n == tmp_edge_pair_indices[i + 1] || mathVec3Equal(v[n], v[tmp_edge_pair_indices[i + 1]])) {
			n = tmp_edge_pair_indices[i];
		}
		else {
			continue;
		}
		tmp_edge_indices[tmp_edge_indices_cnt++] = n;
		if (s == n) {
			break;
		}
		if (mathVec3Equal(v[s], v[n])) {
			n = s;
			break;
		}
		p = i;
		i = 0;
	}
	free(tmp_edge_pair_indices);
	if (s != n || tmp_edge_indices_cnt < 2) {
		free(tmp_edge_indices);
		return NULL;
	}
	/* Merge vertices on the same edge */
	--tmp_edge_indices_cnt;
	++ret_v_indices_cnt;
	last_s = 0;
	first_s = -1;
	for (i = 1; i < tmp_edge_indices_cnt; ++i) {
		mathVec3Sub(v1, v[tmp_edge_indices[i]], v[tmp_edge_indices[last_s]]);
		mathVec3Sub(v2, v[tmp_edge_indices[i]], v[tmp_edge_indices[i + 1]]);
		mathVec3Cross(N, v1, v2);
		if (!mathVec3IsZero(N)) {
			last_s = i;
			if (-1 == first_s) {
				first_s = i;
			}
			++ret_v_indices_cnt;
			continue;
		}
		tmp_edge_indices[i] = -1;
	}
	mathVec3Sub(v1, v[tmp_edge_indices[0]], v[tmp_edge_indices[last_s]]);
	mathVec3Sub(v2, v[tmp_edge_indices[0]], v[tmp_edge_indices[first_s]]);
	mathVec3Cross(N, v1, v2);
	if (mathVec3IsZero(N)) {
		tmp_edge_indices[0] = -1;
		--ret_v_indices_cnt;
	}
	ret_v_indices = (unsigned int*)malloc(ret_v_indices_cnt * sizeof(ret_v_indices[0]));
	if (!ret_v_indices) {
		free(tmp_edge_indices);
		return NULL;
	}
	n = 0;
	for (i = 0; i < tmp_edge_indices_cnt && n < ret_v_indices_cnt; ++i) {
		if (-1 == tmp_edge_indices[i]) {
			continue;
		}
		ret_v_indices[n++] = tmp_edge_indices[i];
	}
	free(tmp_edge_indices);
	/* save result */
	polygon->v_indices = ret_v_indices;
	polygon->v_indices_cnt = ret_v_indices_cnt;
	return polygon;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#ifdef	__cplusplus
extern "C" {
#endif

void mathTriangleGetPoint(const CCTNum_t tri[3][3], CCTNum_t u, CCTNum_t v, CCTNum_t p[3]) {
	CCTNum_t v0[3], v1[3], v2[3];
	mathVec3MultiplyScalar(v0, tri[0], CCTNum(1.0) - u - v);
	mathVec3MultiplyScalar(v1, tri[1], u);
	mathVec3MultiplyScalar(v2, tri[2], v);
	mathVec3Add(p, mathVec3Add(p, v0, v1), v2);
}

int mathTrianglePointUV(const CCTNum_t tri[3][3], const CCTNum_t p[3], CCTNum_t* p_u, CCTNum_t* p_v) {
	CCTNum_t ap[3], ab[3], ac[3], N[3], dot;
	mathVec3Sub(ap, p, tri[0]);
	mathVec3Sub(ab, tri[1], tri[0]);
	mathVec3Sub(ac, tri[2], tri[0]);
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

int mathTriangleHasPoint(const CCTNum_t tri[3][3], const CCTNum_t p[3]) {
	return mathTrianglePointUV(tri, p, NULL, NULL);
}

void mathTriangleToPolygon(const CCTNum_t tri[3][3], GeometryPolygon_t* polygon) {
	polygon->v_indices = Triangle_Vertice_Indices_Default;
	polygon->v_indices_cnt = 3;
	polygon->v = (CCTNum_t(*)[3])tri;
	mathPlaneNormalByVertices3(tri[0], tri[1], tri[2], polygon->normal);
}

int mathPolygonIsConvex(const GeometryPolygon_t* polygon) {
	CCTNum_t e1[3], e2[3], test_n[3];
	unsigned int i, has_test_n;
	if (polygon->v_indices_cnt < 3) {
		return 0;
	}
	has_test_n = 0;
	for (i = 0; i < polygon->v_indices_cnt; ++i) {
		CCTNum_t n[3];
		unsigned int v_idx1, v_idx2;
		if (i) {
			v_idx1 = polygon->v_indices[i - 1];
			v_idx2 = polygon->v_indices[i + 1 < polygon->v_indices_cnt ? i + 1 : 0];
		}
		else {
			v_idx1 = polygon->v_indices[polygon->v_indices_cnt - 1];
			v_idx2 = polygon->v_indices[1];
		}
		mathVec3Sub(e1, polygon->v[polygon->v_indices[i]], polygon->v[v_idx1]);
		mathVec3Sub(e2, polygon->v[v_idx2], polygon->v[polygon->v_indices[i]]);
		if (!has_test_n) {
			mathVec3Cross(test_n, e1, e2);
			if (!mathVec3IsZero(test_n)) {
				has_test_n = 1;
			}
			continue;
		}
		mathVec3Cross(n, e1, e2);
		if (mathVec3Dot(test_n, n) < CCTNum(0.0)) {
			return 0;
		}
	}
	return 1;
}

GeometryPolygon_t* mathPolygonCooking(const CCTNum_t(*v)[3], unsigned int v_cnt, const unsigned int* tri_indices, unsigned int tri_indices_cnt, GeometryPolygon_t* polygon) {
	CCTNum_t(*dup_v)[3] = NULL;
	unsigned int dup_v_cnt;
	unsigned int* dup_tri_indices = NULL;

	if (v_cnt < 3 || tri_indices_cnt < 3) {
		return NULL;
	}
	dup_v_cnt = mathVerticesDistinctCount(v, v_cnt);
	if (dup_v_cnt < 3) {
		return NULL;
	}
	dup_v = (CCTNum_t(*)[3])malloc(sizeof(dup_v[0]) * dup_v_cnt);
	if (!dup_v) {
		goto err;
	}
	dup_tri_indices = (unsigned int*)malloc(sizeof(tri_indices[0]) * tri_indices_cnt);
	if (!dup_tri_indices) {
		goto err;
	}
	mathVerticesMerge(v, v_cnt, tri_indices, tri_indices_cnt, dup_v, dup_tri_indices);
	if (!PolygonCooking_InternalProc((const CCTNum_t(*)[3])dup_v, dup_tri_indices, tri_indices_cnt, polygon)) {
		goto err;
	}
	mathVec3Set(polygon->o, CCTNums_3(0.0, 0.0, 0.0));
	polygon->v = dup_v;
	polygon->tri_indices = dup_tri_indices;
	polygon->tri_indices_cnt = tri_indices_cnt;
	return polygon;
err:
	free(dup_v);
	free(dup_tri_indices);
	return NULL;
}

GeometryPolygon_t* mathPolygonDeepCopy(GeometryPolygon_t* dst, const GeometryPolygon_t* src) {
	unsigned int i, dup_v_cnt = 0;
	CCTNum_t(*dup_v)[3] = NULL;
	unsigned int* dup_v_indices = NULL;
	unsigned int* dup_tri_indices = NULL;
	/* find max vertex index, dup_v_cnt */
	for (i = 0; i < src->v_indices_cnt; ++i) {
		if (src->v_indices[i] >= dup_v_cnt) {
			dup_v_cnt = src->v_indices[i] + 1;
		}
	}
	for (i = 0; i < src->tri_indices_cnt; ++i) {
		if (src->tri_indices[i] >= dup_v_cnt) {
			dup_v_cnt = src->tri_indices[i] + 1;
		}
	}
	if (dup_v_cnt < 3) {
		return NULL;
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
	for (i = 0; i < src->v_indices_cnt; ++i) {
		unsigned int idx = src->v_indices[i];
		dup_v_indices[i] = idx;
		mathVec3Copy(dup_v[idx], src->v[idx]);
	}
	for (i = 0; i < src->tri_indices_cnt; ++i) {
		dup_tri_indices[i] = src->tri_indices[i];
	}
	mathVec3Copy(dst->o, src->o);
	mathVec3Copy(dst->normal, src->normal);
	dst->tri_indices_cnt = src->tri_indices_cnt;
	dst->v_indices_cnt = src->v_indices_cnt;
	dst->v = dup_v;
	dst->v_indices = dup_v_indices;
	dst->tri_indices = dup_tri_indices;
	return dst;
err_0:
	free(dup_v);
	free(dup_v_indices);
	free(dup_tri_indices);
	return NULL;
}

void mathPolygonFreeCookingData(GeometryPolygon_t* polygon) {
	if (!polygon) {
		return;
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

#ifdef	__cplusplus
}
#endif
