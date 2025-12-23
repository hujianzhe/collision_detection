//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/vertex.h"
#include "../inc/plane.h"
#include "../inc/cooking.h"
#include <stdlib.h>

extern int Segment_Contain_Point(const CCTNum_t ls0[3], const CCTNum_t ls1[3], const CCTNum_t p[3]);
extern int Plane_Contain_Point(const CCTNum_t plane_v[3], const CCTNum_t plane_normal[3], const CCTNum_t p[3]);
extern int mathPolygonIsConvex(const CCTNum_t(*v)[3], const CCTNum_t normal[3], const unsigned int* edge_v_indices_flat, unsigned int edge_v_indices_cnt, const unsigned int* v_indices, unsigned int v_indices_cnt);

static GeometryPolygon_t* _insert_tri_indices(GeometryPolygon_t* polygon, const unsigned int* tri_v_indices) {
	unsigned int cnt = polygon->tri_cnt * 3;
	unsigned int* new_p = (unsigned int*)realloc((void*)polygon->tri_v_indices_flat, sizeof(polygon->tri_v_indices_flat[0]) * (cnt + 3));
	if (!new_p) {
		return NULL;
	}
	new_p[cnt++] = tri_v_indices[0];
	new_p[cnt++] = tri_v_indices[1];
	new_p[cnt++] = tri_v_indices[2];
	polygon->tri_v_indices_flat = new_p;
	++polygon->tri_cnt;
	return polygon;
}

static GeometryPolygon_t* _init_new_polygon(GeometryPolygon_t* new_pg, const CCTNum_t(*v)[3], const CCTNum_t N[3], const unsigned int* tri_v_indices) {
	new_pg->tri_v_indices_flat = NULL;
	new_pg->tri_cnt = 0;
	if (!_insert_tri_indices(new_pg, tri_v_indices)) {
		return NULL;
	}
	new_pg->concave_tri_v_ids_flat = NULL;
	new_pg->concave_tri_edge_ids_flat = NULL;
	new_pg->v_indices = NULL;
	new_pg->v_indices_cnt = 0;
	new_pg->edge_v_ids_flat = NULL;
	new_pg->edge_v_indices_flat = NULL;
	new_pg->edge_cnt = 0;
	new_pg->is_convex = 0;
	new_pg->v = (CCTNum_t(*)[3])v;
	new_pg->mesh_v_ids = NULL;
	new_pg->mesh_edge_ids = NULL;
	new_pg->v_adjacent_infos = NULL;
	mathVec3Copy(new_pg->normal, N);
	mathVec3Set(new_pg->center, CCTNums_3(0.0, 0.0, 0.0));
	return new_pg;
}

static int _check_tri_valid(const CCTNum_t(*v)[3], const unsigned int* tri_v_indices, CCTNum_t min_edge_len, CCTNum_t cos_min_degree) {
	CCTNum_t e01[3], e02[3], e12[3], cos_d, len;
	mathVec3Sub(e01, v[tri_v_indices[1]], v[tri_v_indices[0]]);
	mathVec3Sub(e02, v[tri_v_indices[2]], v[tri_v_indices[0]]);
	mathVec3Sub(e12, v[tri_v_indices[2]], v[tri_v_indices[1]]);
	len = mathVec3Normalized(e01, e01);
	if (len <= min_edge_len) {
		return 0;
	}
	len = mathVec3Normalized(e02, e02);
	if (len <= min_edge_len) {
		return 0;
	}
	len = mathVec3Normalized(e12, e12);
	if (len <= min_edge_len) {
		return 0;
	}
	cos_d = mathVec3Dot(e01, e02);
	if (cos_d > cos_min_degree) {
		return 0;
	}
	cos_d = mathVec3Dot(e01, e12);
	if (cos_d > cos_min_degree) {
		return 0;
	}
	cos_d = mathVec3Dot(e02, e12);
	if (cos_d > cos_min_degree) {
		return 0;
	}
	return 1;
}

static void _save_invalid_triangle(GeometryCookingOutput_t* output, const CCTNum_t(*v)[3], const unsigned int* tri_v_indices) {
	output->has_invalid_tri = 1;
	mathVec3Copy(output->invalid_tri_v[0], v[tri_v_indices[0]]);
	mathVec3Copy(output->invalid_tri_v[1], v[tri_v_indices[1]]);
	mathVec3Copy(output->invalid_tri_v[2], v[tri_v_indices[2]]);
}

static int _polygon_can_merge_triangle(GeometryPolygon_t* polygon, const CCTNum_t p0[3], const CCTNum_t p1[3], const CCTNum_t p2[3]) {
	unsigned int i, polygon_tri_v_indices_cnt;
	const CCTNum_t* tri_p[] = { p0, p1, p2 };
	for (i = 0; i < 3; ++i) {
		if (!Plane_Contain_Point(polygon->v[polygon->tri_v_indices_flat[0]], polygon->normal, tri_p[i])) {
			return 0;
		}
	}
	polygon_tri_v_indices_cnt = polygon->tri_cnt * 3;
	for (i = 0; i < polygon_tri_v_indices_cnt; i += 3) {
		unsigned int j, n = 0;
		const CCTNum_t* arg_diff_point = NULL, *same_edge_v[2];
		const CCTNum_t* triangle[3] = {
			polygon->v[polygon->tri_v_indices_flat[i]],
			polygon->v[polygon->tri_v_indices_flat[i + 1]],
			polygon->v[polygon->tri_v_indices_flat[i + 2]]
		};
		const CCTNum_t* triangle_diff_point;
		CCTNum_t v[3], N[3], dot;
		for (j = 0; j < 3; ++j) {
			if (mathVec3Equal(tri_p[j], triangle[0])) {
				++n;
				continue;
			}
			if (mathVec3Equal(tri_p[j], triangle[1])) {
				++n;
				continue;
			}
			if (mathVec3Equal(tri_p[j], triangle[2])) {
				++n;
				continue;
			}
			arg_diff_point = tri_p[j];
		}
		if (n >= 3) {
			/* same triangle */
			return 2;
		}
		if (n < 2) {
			/* no same edge */
			continue;
		}
		/* find same and other diff point */
		if (arg_diff_point == p0) {
			same_edge_v[0] = p1;
			same_edge_v[1] = p2;
		}
		else if (arg_diff_point == p1) {
			same_edge_v[0] = p0;
			same_edge_v[1] = p2;
		}
		else {
			same_edge_v[0] = p0;
			same_edge_v[1] = p1;
		}
		triangle_diff_point = NULL;
		for (j = 0; j < 3; ++j) {
			if (mathVec3Equal(triangle[j], same_edge_v[0])) {
				continue;
			}
			if (mathVec3Equal(triangle[j], same_edge_v[1])) {
				continue;
			}
			triangle_diff_point = triangle[j];
			break;
		}
		if (!triangle_diff_point) {
			/* no possible */
			return -1;
		}
		/* check diff point and triangle point in same side */
		mathVec3Sub(v, same_edge_v[1], same_edge_v[0]);
		mathVec3Cross(N, polygon->normal, v);
		mathVec3Sub(v, arg_diff_point, same_edge_v[0]);
		dot = mathVec3Dot(v, N);
		if (dot > CCTNum(0.0)) {
			mathVec3Sub(v, triangle_diff_point, same_edge_v[0]);
			dot = mathVec3Dot(v, N);
			if (dot > CCTNum(0.0)) {
				/* same side, edge is cross */
				return -1;
			}
		}
		else {
			mathVec3Sub(v, triangle_diff_point, same_edge_v[0]);
			dot = mathVec3Dot(v, N);
			if (dot < CCTNum(0.0)) {
				/* same side, edge is cross */
				return -1;
			}
		}
		return 1;
	}
	return 0;
}

int MathCookingStage_DistinctVertices(const CCTNum_t(*v)[3], const unsigned int* tri_v_indices, unsigned int tri_v_indices_cnt, CCTNum_t(**ret_v)[3], unsigned int* ret_v_cnt, unsigned int** ret_tri_v_indices) {
	unsigned int* dup_tri_v_indices = NULL;
	unsigned int dup_v_cnt, i;
	CCTNum_t(*dup_v)[3] = NULL, (*tmp_v)[3] = NULL;

	dup_v = (CCTNum_t(*)[3])malloc(sizeof(dup_v[0]) * tri_v_indices_cnt);
	if (!dup_v) {
		goto err;
	}
	dup_tri_v_indices = (unsigned int*)malloc(sizeof(tri_v_indices[0]) * tri_v_indices_cnt);
	if (!dup_tri_v_indices) {
		goto err;
	}
	dup_v_cnt = mathVerticesMerge(v, tri_v_indices, tri_v_indices_cnt, dup_v, dup_tri_v_indices);
	if (dup_v_cnt < 3) {
		goto err;
	}
	tmp_v = (CCTNum_t(*)[3])malloc(sizeof(tmp_v[0]) * dup_v_cnt);
	if (!tmp_v) {
		goto err;
	}
	for (i = 0; i < dup_v_cnt; ++i) {
		mathVec3Copy(tmp_v[i], dup_v[i]);
	}
	free(dup_v);
	*ret_v = tmp_v;
	*ret_v_cnt = dup_v_cnt;
	*ret_tri_v_indices = dup_tri_v_indices;
	return 1;
err:
	free(dup_v);
	free(dup_tri_v_indices);
	return 0;
}

int MeshCookingStage_SplitFaces(const CCTNum_t(*v)[3], const unsigned int* tri_v_indices, unsigned int tri_v_indices_cnt, GeometryPolygon_t** ret_polygons, unsigned int* ret_polygons_cnt, const GeometryCookingOption_t* opt, GeometryCookingOutput_t* output) {
	unsigned int i, tri_cnt, tmp_polygons_cnt = 0;
	char* tri_merge_bits = NULL;
	GeometryPolygon_t* tmp_polygons = NULL;

	/* check triangles valid */
	tri_cnt = tri_v_indices_cnt / 3;
	if (tri_cnt < 1) {
		goto err;
	}
	if (opt->tri_edge_min_len > CCTNum(0.0) || opt->tri_min_degree > CCTNum(0.0)) {
		CCTNum_t cos_min_degree = CCTNum_cos(opt->tri_min_degree * CCT_RADIAN_PER_DEGREE);
		for (i = 0; i < tri_v_indices_cnt; i += 3) {
			if (!_check_tri_valid(v, tri_v_indices + i, opt->tri_edge_min_len, cos_min_degree)) {
				_save_invalid_triangle(output, v, tri_v_indices + i);
				goto err;
			}
		}
	}

	if (opt->tri_merged) {
		tri_merge_bits = (char*)calloc(1, tri_cnt / 8 + (tri_cnt % 8 ? 1 : 0));
		if (!tri_merge_bits) {
			goto err;
		}
		/* Merge triangles on the same plane */
		for (i = 0; i < tri_v_indices_cnt; i += 3) {
			unsigned int j, tri_idx;
			CCTNum_t N[3];
			GeometryPolygon_t* tmp_parr, * new_pg;

			tri_idx = i / 3;
			if (tri_merge_bits[tri_idx / 8] & (1 << (tri_idx % 8))) {
				continue;
			}
			mathPlaneNormalByVertices3(v[tri_v_indices[i]], v[tri_v_indices[i + 1]], v[tri_v_indices[i + 2]], N);
			if (mathVec3IsZero(N)) {
				goto err;
			}
			tmp_parr = (GeometryPolygon_t*)realloc(tmp_polygons, (tmp_polygons_cnt + 1) * sizeof(GeometryPolygon_t));
			if (!tmp_parr) {
				goto err;
			}
			tmp_polygons = tmp_parr;
			new_pg = tmp_polygons + tmp_polygons_cnt;
			if (!_init_new_polygon(new_pg, v, N, tri_v_indices + i)) {
				goto err;
			}
			++tmp_polygons_cnt;

			tri_merge_bits[tri_idx / 8] |= (1 << (tri_idx % 8));
			for (j = 0; j < tri_v_indices_cnt; j += 3) {
				int ret;
				tri_idx = j / 3;
				if (tri_merge_bits[tri_idx / 8] & (1 << tri_idx % 8)) {
					continue;
				}
				ret = _polygon_can_merge_triangle(new_pg, v[tri_v_indices[j]], v[tri_v_indices[j + 1]], v[tri_v_indices[j + 2]]);
				if (-1 == ret) {
					goto err;
				}
				if (0 == ret) {
					continue;
				}
				tri_merge_bits[tri_idx / 8] |= (1 << (tri_idx % 8));
				if (2 == ret) {
					continue;
				}
				if (!_insert_tri_indices(new_pg, tri_v_indices + j)) {
					goto err;
				}
				j = 0;
			}
		}
		free(tri_merge_bits);
	}
	else {
		tmp_polygons = (GeometryPolygon_t*)malloc(tri_cnt * sizeof(GeometryPolygon_t));
		if (!tmp_polygons) {
			goto err;
		}
		for (i = 0; i < tri_v_indices_cnt; i += 3) {
			CCTNum_t N[3];
			GeometryPolygon_t* new_pg;

			mathPlaneNormalByVertices3(v[tri_v_indices[i]], v[tri_v_indices[i + 1]], v[tri_v_indices[i + 2]], N);
			if (mathVec3IsZero(N)) {
				goto err;
			}
			new_pg = tmp_polygons + tmp_polygons_cnt;
			if (!_init_new_polygon(new_pg, v, N, tri_v_indices + i)) {
				goto err;
			}
			++tmp_polygons_cnt;
		}
	}
	*ret_polygons = tmp_polygons;
	*ret_polygons_cnt = tmp_polygons_cnt;
	return 1;
err:
	if (tmp_polygons) {
		for (i = 0; i < tmp_polygons_cnt; ++i) {
			free((void*)tmp_polygons[i].tri_v_indices_flat);
		}
		free(tmp_polygons);
	}
	free(tri_merge_bits);
	return 0;
}

int MeshCookingStage_GenerateFaceEdges(const CCTNum_t(*v)[3], const unsigned int* tri_v_indices, unsigned int tri_v_indices_cnt, const CCTNum_t plane_n[3], unsigned int** ret_edge_v_indices, unsigned int* ret_edge_v_indices_cnt) {
	unsigned int i, j;
	unsigned int* tmp_edge_pair_indices = NULL;
	unsigned int tmp_edge_pair_indices_cnt = 0;
	unsigned int* tmp_edge_indices = NULL;
	unsigned int tmp_edge_indices_cnt = 0;
	/* Filters all no-border edges */
	if (3 == tri_v_indices_cnt) {
		tmp_edge_indices = (unsigned int*)malloc(sizeof(tmp_edge_indices[0]) * 6);
		if (!tmp_edge_indices) {
			goto err;
		}
		tmp_edge_indices[0] = tri_v_indices[0];
		tmp_edge_indices[1] = tri_v_indices[1];
		tmp_edge_indices[2] = tri_v_indices[1];
		tmp_edge_indices[3] = tri_v_indices[2];
		tmp_edge_indices[4] = tri_v_indices[2];
		tmp_edge_indices[5] = tri_v_indices[0];
		*ret_edge_v_indices = tmp_edge_indices;
		*ret_edge_v_indices_cnt = 6;
		return 1;
	}
	for (i = 0; i < tri_v_indices_cnt; i += 3) {
		unsigned int ei[6] = {
			tri_v_indices[i], tri_v_indices[i + 1],
			tri_v_indices[i + 1], tri_v_indices[i + 2],
			tri_v_indices[i + 2], tri_v_indices[i]
		};
		int same[3] = { 0 }, seperate = 1;
		for (j = 0; j < tri_v_indices_cnt; j += 3) {
			unsigned int k;
			unsigned int ej[6];
			if (i == j) {
				continue;
			}
			ej[0] = tri_v_indices[j]; ej[1] = tri_v_indices[j + 1];
			ej[2] = tri_v_indices[j + 1]; ej[3] = tri_v_indices[j + 2];
			ej[4] = tri_v_indices[j + 2]; ej[5] = tri_v_indices[j];
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
						/* check is border */
						const CCTNum_t* pi, *pj;
						CCTNum_t edge_v[3], edge_n[3];
						CCTNum_t vpi[3], vpj[3], pi_dot, pj_dot;

						seperate = 0;
						mathVec3Sub(edge_v, v[ei[k + 1]], v[ei[k]]);
						mathVec3Cross(edge_n, edge_v, plane_n);
						if (0 == k) {
							pi = v[tri_v_indices[i + 2]];
						}
						else if (2 == k) {
							pi = v[tri_v_indices[i]];
						}
						else {
							pi = v[tri_v_indices[i + 1]];
						}
						mathVec3Sub(vpi, pi, v[ei[k]]);
						pi_dot = mathVec3Dot(vpi, edge_n);
						if (0 == m) {
							pj = v[tri_v_indices[j + 2]];
						}
						else if (2 == m) {
							pj = v[tri_v_indices[j]];
						}
						else {
							pj = v[tri_v_indices[j + 1]];
						}
						mathVec3Sub(vpj, pj, v[ei[k]]);
						pj_dot = mathVec3Dot(vpj, edge_n);
						if (pi_dot > CCTNum(0.0) && pj_dot < CCTNum(0.0)) {
							break;
						}
						if (pi_dot < CCTNum(0.0) && pj_dot > CCTNum(0.0)) {
							break;
						}
						same[k >> 1] = 0;
					}
				}
			}
			if (same[0] && same[1] && same[2]) {
				break;
			}
		}
		if (j != tri_v_indices_cnt) {
			continue;
		}
		if (seperate) {
			goto err;
		}
		for (j = 0; j < 6; j += 2) {
			unsigned int* ptr;
			if (same[j >> 1]) {
				continue;
			}
			tmp_edge_pair_indices_cnt += 2;
			ptr = (unsigned int*)realloc(tmp_edge_pair_indices, tmp_edge_pair_indices_cnt * sizeof(tmp_edge_pair_indices[0]));
			if (!ptr) {
				goto err;
			}
			tmp_edge_pair_indices = ptr;
			tmp_edge_pair_indices[tmp_edge_pair_indices_cnt - 2] = ei[j];
			tmp_edge_pair_indices[tmp_edge_pair_indices_cnt - 1] = ei[j + 1];
		}
	}
	if (!tmp_edge_pair_indices) {
		goto err;
	}
	/* Merge Edge Indices */
	tmp_edge_indices_cnt = tmp_edge_pair_indices_cnt;
	for (i = 0; i < tmp_edge_pair_indices_cnt; i += 2) {
		if (-1 == tmp_edge_pair_indices[i]) {
			continue;
		}
		for (j = 0; j < tmp_edge_pair_indices_cnt; j += 2) {
			if (i == j) {
				continue;
			}
			if (-1 == tmp_edge_pair_indices[j]) {
				continue;
			}
			if (!mathEdgeVertexIndicesMergeEdgeVertexIndices(v, tmp_edge_pair_indices + i, tmp_edge_pair_indices + j, tmp_edge_pair_indices + i)) {
				continue;
			}
			tmp_edge_indices_cnt -= 2;
			tmp_edge_pair_indices[j] = -1;
			j = 0;
		}
	}
	tmp_edge_indices = (unsigned int*)malloc(sizeof(tmp_edge_indices[0]) * tmp_edge_indices_cnt);
	if (!tmp_edge_indices) {
		goto err;
	}
	j = 0;
	for (i = 0; i < tmp_edge_pair_indices_cnt; i += 2) {
		if (-1 == tmp_edge_pair_indices[i]) {
			continue;
		}
		tmp_edge_indices[j++] = tmp_edge_pair_indices[i];
		tmp_edge_indices[j++] = tmp_edge_pair_indices[i + 1];
	}
	free(tmp_edge_pair_indices);
	*ret_edge_v_indices = tmp_edge_indices;
	*ret_edge_v_indices_cnt = tmp_edge_indices_cnt;
	return 1;
err:
	free(tmp_edge_indices);
	free(tmp_edge_pair_indices);
	return 0;
}

int MeshCookingStage_GenerateIndices(const unsigned int* edge_v_indices_flat, unsigned int edge_v_indices_cnt, unsigned int** ret_v_indices, unsigned int* ret_v_indices_cnt, unsigned int** ret_edge_v_ids_flat) {
	unsigned int* tmp_edge_v_ids_flat;
	unsigned int* tmp_v_indices;
	unsigned int tmp_v_indices_cnt, i;
	tmp_edge_v_ids_flat = (unsigned int*)malloc(sizeof(tmp_edge_v_ids_flat[0]) * edge_v_indices_cnt);
	if (!tmp_edge_v_ids_flat) {
		return 0;
	}
	tmp_v_indices = (unsigned int*)malloc(sizeof(tmp_v_indices[0]) * edge_v_indices_cnt);
	if (!tmp_v_indices) {
		free(tmp_edge_v_ids_flat);
		return 0;
	}
	tmp_v_indices_cnt = 0;
	for (i = 0; i < edge_v_indices_cnt; ++i) {
		unsigned int j;
		for (j = 0; j < tmp_v_indices_cnt; ++j) {
			if (edge_v_indices_flat[i] == tmp_v_indices[j]) {
				tmp_edge_v_ids_flat[i] = j;
				break;
			}
		}
		if (j < tmp_v_indices_cnt) {
			continue;
		}
		tmp_edge_v_ids_flat[i] = tmp_v_indices_cnt;
		tmp_v_indices[tmp_v_indices_cnt++] = edge_v_indices_flat[i];
	}
	if (tmp_v_indices_cnt < edge_v_indices_cnt) {
		unsigned int* buf = (unsigned int*)malloc(sizeof(tmp_v_indices[0]) * tmp_v_indices_cnt);
		if (!buf) {
			free(tmp_v_indices);
			return 0;
		}
		for (i = 0; i < tmp_v_indices_cnt; ++i) {
			buf[i] = tmp_v_indices[i];
		}
		free(tmp_v_indices);
		tmp_v_indices = buf;
	}
	*ret_v_indices = tmp_v_indices;
	*ret_v_indices_cnt = tmp_v_indices_cnt;
	*ret_edge_v_ids_flat = tmp_edge_v_ids_flat;
	return 1;
}

static unsigned int find_edge_id(const CCTNum_t(*v)[3], const unsigned int* edge_v_indices_flat, unsigned int edge_v_indices_cnt, const CCTNum_t p0[3], const CCTNum_t p1[3]) {
	unsigned int i;
	for (i = 0; i < edge_v_indices_cnt; ++i) {
		const CCTNum_t* edge_v0 = v[edge_v_indices_flat[i++]];
		const CCTNum_t* edge_v1 = v[edge_v_indices_flat[i]];
		if (Segment_Contain_Point(edge_v0, edge_v1, p0) && Segment_Contain_Point(edge_v0, edge_v1, p1)) {
			return i >> 1;
		}
	}
	return -1;
}

unsigned int* mathCookingConcavePolygonTriangleEdge(const CCTNum_t(*v)[3], const unsigned int* edge_v_indices_flat, unsigned int edge_v_indices_cnt, const unsigned int* tri_v_indices_flat, unsigned int tri_v_indices_cnt) {
	unsigned int i, j;
	unsigned int* tri_edge_ids = (unsigned int*)malloc(sizeof(tri_edge_ids[0]) * tri_v_indices_cnt);
	if (!tri_edge_ids) {
		return NULL;
	}
	for (j = 0, i = 0; i < tri_v_indices_cnt; ) {
		unsigned int edge_id;
		const CCTNum_t* p0 = v[tri_v_indices_flat[i++]];
		const CCTNum_t* p1 = v[tri_v_indices_flat[i++]];
		const CCTNum_t* p2 = v[tri_v_indices_flat[i++]];
		edge_id = find_edge_id(v, edge_v_indices_flat, edge_v_indices_cnt, p0, p1);
		if (edge_id != -1) {
			tri_edge_ids[j++] = edge_id;
		}
		edge_id = find_edge_id(v, edge_v_indices_flat, edge_v_indices_cnt, p0, p2);
		if (edge_id != -1) {
			tri_edge_ids[j++] = edge_id;
		}
		edge_id = find_edge_id(v, edge_v_indices_flat, edge_v_indices_cnt, p1, p2);
		if (edge_id != -1) {
			tri_edge_ids[j++] = edge_id;
		}
		while (j < i) {
			tri_edge_ids[j++] = -1;
		}
	}
	return tri_edge_ids;
}

unsigned int* mathCookingConcavePolygonTriangleVertex(const unsigned int* v_indices, unsigned int v_indices_cnt, const unsigned int* tri_v_indices, unsigned int tri_v_indices_cnt) {
	unsigned int i, j;
	unsigned int* tri_v_ids = (unsigned int*)malloc(sizeof(tri_v_ids[0]) * tri_v_indices_cnt);
	if (!tri_v_ids) {
		return NULL;
	}
	for (i = 0; i < tri_v_indices_cnt; ++i) {
		for (j = 0; j < v_indices_cnt; ++j) {
			if (tri_v_indices[i] == v_indices[j]) {
				tri_v_ids[i] = j;
				break;
			}
		}
		if (j >= v_indices_cnt) {
			tri_v_ids[i] = -1;
		}
	}
	return tri_v_ids;
}

#ifdef	__cplusplus
extern "C" {
#endif

const GeometryCookingOutput_t* mathCookingPolygon(const CCTNum_t(*v)[3], const unsigned int* tri_v_indices, unsigned int tri_v_indices_cnt, const GeometryCookingOption_t* opt, GeometryCookingOutput_t* output) {
	CCTNum_t(*dup_v)[3] = NULL;
	CCTNum_t N[3];
	unsigned int* dup_tri_v_indices = NULL;
	unsigned int* edge_v_indices_flat = NULL, *edge_v_ids_flat = NULL;
	unsigned int* v_indices = NULL;
	unsigned int edge_v_indices_cnt, v_indices_cnt, dup_v_cnt, i;
	unsigned int* concave_tri_edge_ids = NULL, *concave_tri_v_ids = NULL;
	GeometryPolygonVertexAdjacentInfo_t* v_adjacent_infos = NULL;
	GeometryPolygon_t* polygon = output->polygon_ptr;
	output->error_code = 1;
	output->has_invalid_tri = 0;
	/* check */
	if (tri_v_indices_cnt < 3) {
		return output;
	}
	/* merge distinct vertices, rewrite indices */
	if (!MathCookingStage_DistinctVertices(v, tri_v_indices, tri_v_indices_cnt, &dup_v, &dup_v_cnt, &dup_tri_v_indices)) {
		goto err;
	}
	/* check all triangles valid */
	mathPlaneNormalByVertices3(v[tri_v_indices[0]], v[tri_v_indices[1]], v[tri_v_indices[2]], N);
	for (i = 0; i < dup_v_cnt; ++i) {
		if (!Plane_Contain_Point(v[tri_v_indices[0]], N, dup_v[i])) {
			goto err;
		}
	}
	if (opt->tri_edge_min_len > CCTNum(0.0) || opt->tri_min_degree > CCTNum(0.0)) {
		CCTNum_t cos_min_degree = CCTNum_cos(opt->tri_min_degree * CCT_RADIAN_PER_DEGREE);
		for (i = 0; i < tri_v_indices_cnt; i += 3) {
			if (!_check_tri_valid((const CCTNum_t(*)[3])dup_v, dup_tri_v_indices + i, opt->tri_edge_min_len, cos_min_degree)) {
				_save_invalid_triangle(output, (const CCTNum_t(*)[3])dup_v, dup_tri_v_indices + i);
				goto err;
			}
		}
	}
	/* cooking edge */
	if (!MeshCookingStage_GenerateFaceEdges((const CCTNum_t(*)[3])dup_v, dup_tri_v_indices, tri_v_indices_cnt, N, &edge_v_indices_flat, &edge_v_indices_cnt)) {
		goto err;
	}
	/* cooking vertex indice */
	if (!MeshCookingStage_GenerateIndices(edge_v_indices_flat, edge_v_indices_cnt, &v_indices, &v_indices_cnt, &edge_v_ids_flat)) {
		goto err;
	}
	/* cooking vertex adjacent info */
	v_adjacent_infos = (GeometryPolygonVertexAdjacentInfo_t*)malloc(sizeof(v_adjacent_infos[0]) * v_indices_cnt);
	if (!v_adjacent_infos) {
		goto err;
	}
	for (i = 0; i < v_indices_cnt; ++i) {
		if (!mathPolygonVertexAdjacentInfo(edge_v_ids_flat, edge_v_indices_cnt, i, v_adjacent_infos + i)) {
			goto err;
		}
	}
	/* if concave, save triangle edge ids, v ids */
	polygon->is_convex = mathPolygonIsConvex((const CCTNum_t(*)[3])dup_v, N, edge_v_indices_flat, edge_v_indices_cnt, v_indices, v_indices_cnt);
	if (!polygon->is_convex) {
		concave_tri_edge_ids = mathCookingConcavePolygonTriangleEdge((const CCTNum_t(*)[3])dup_v, edge_v_indices_flat, edge_v_indices_cnt, dup_tri_v_indices, tri_v_indices_cnt);
		if (!concave_tri_edge_ids) {
			goto err;
		}
		concave_tri_v_ids = mathCookingConcavePolygonTriangleVertex(v_indices, v_indices_cnt, dup_tri_v_indices, tri_v_indices_cnt);
		if (!concave_tri_v_ids) {
			goto err;
		}
	}
	polygon->concave_tri_edge_ids_flat = concave_tri_edge_ids;
	polygon->concave_tri_v_ids_flat = concave_tri_v_ids;
	/* save result */
	mathVertexIndicesAverageXYZ((const CCTNum_t(*)[3])dup_v, v_indices, v_indices_cnt, polygon->center);
	mathVec3Copy(polygon->normal, N);
	polygon->v = dup_v;
	polygon->v_indices = v_indices;
	polygon->v_indices_cnt = v_indices_cnt;
	polygon->edge_v_ids_flat = edge_v_ids_flat;
	polygon->edge_v_indices_flat = edge_v_indices_flat;
	polygon->edge_cnt = edge_v_indices_cnt / 2;
	polygon->tri_v_indices_flat = dup_tri_v_indices;
	polygon->tri_cnt = tri_v_indices_cnt / 3;
	polygon->mesh_v_ids = NULL;
	polygon->mesh_edge_ids = NULL;
	polygon->v_adjacent_infos = v_adjacent_infos;
	/* finish */
	output->error_code = 0;
	return output;
err:
	free(dup_v);
	free(v_indices);
	free(edge_v_ids_flat);
	free(edge_v_indices_flat);
	free(concave_tri_v_ids);
	free(concave_tri_edge_ids);
	free(dup_tri_v_indices);
	free(v_adjacent_infos);
	return output;
}

#ifdef	__cplusplus
}
#endif
