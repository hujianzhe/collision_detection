//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/vertex.h"
#include "../inc/aabb.h"
#include "../inc/plane.h"
#include "../inc/polygon.h"
#include "../inc/mesh.h"
#include "../inc/cooking.h"

extern void free_data_mesh_vertex_adjacent_info(GeometryMeshVertexAdjacentInfo_t* info, const CCTAllocator_t* ac);
extern void Polygon_ClearWithoutVertices(GeometryPolygon_t* polygon, const CCTAllocator_t* ac);
extern int Segment_Contain_Point(const CCTNum_t ls0[3], const CCTNum_t ls1[3], const CCTNum_t p[3]);
extern int Plane_Contain_Point(const CCTNum_t plane_v[3], const CCTNum_t plane_normal[3], const CCTNum_t p[3]);
extern int Polygon_IsConvex(const CCTNum_t(*v)[3], const CCTNum_t normal[3], const unsigned int* edge_v_indices_flat, unsigned int edge_v_indices_cnt, const unsigned int* v_indices, unsigned int v_indices_cnt);

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

static GeometryPolygon_t* _polygon_insert_tri_indices(GeometryPolygon_t* polygon, const unsigned int tri_v_indices[3], const CCTAllocator_t* ac) {
	unsigned int cnt = polygon->tri_cnt * 3;
	unsigned int* new_p = (unsigned int*)ac->fn_realloc(ac, (void*)polygon->tri_v_indices_flat, sizeof(polygon->tri_v_indices_flat[0]) * (cnt + 3));
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

static GeometryPolygon_t* _polygon_init(GeometryPolygon_t* new_pg, const CCTNum_t(*v)[3], const CCTNum_t N[3], const unsigned int tri_v_indices[3], const CCTAllocator_t* ac) {
	new_pg->tri_v_indices_flat = NULL;
	new_pg->tri_cnt = 0;
	if (!_polygon_insert_tri_indices(new_pg, tri_v_indices, ac)) {
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

static void _save_invalid_triangle(GeometryCookingOutput_t* output, const CCTNum_t(*v)[3], const unsigned int tri_v_indices[3]) {
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
		const CCTNum_t* arg_diff_point = NULL, * same_edge_v[2];
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

static int MathCookingStage_DistinctVertices(const CCTNum_t(*v)[3], const unsigned int* tri_v_indices, unsigned int tri_v_indices_cnt, CCTNum_t(**ret_v)[3], unsigned int* ret_v_cnt, unsigned int** ret_tri_v_indices, const CCTAllocator_t* ac) {
	unsigned int* dup_tri_v_indices = NULL;
	unsigned int dup_v_cnt, i;
	CCTNum_t(*dup_v)[3] = NULL, (*tmp_v)[3] = NULL;

	dup_v = (CCTNum_t(*)[3])ac->fn_malloc(ac, sizeof(dup_v[0]) * tri_v_indices_cnt);
	if (!dup_v) {
		goto err;
	}
	dup_tri_v_indices = (unsigned int*)ac->fn_malloc(ac, sizeof(tri_v_indices[0]) * tri_v_indices_cnt);
	if (!dup_tri_v_indices) {
		goto err;
	}
	dup_v_cnt = mathVerticesMerge(v, tri_v_indices, tri_v_indices_cnt, dup_v, dup_tri_v_indices);
	if (dup_v_cnt < 3) {
		goto err;
	}
	tmp_v = (CCTNum_t(*)[3])ac->fn_malloc(ac, sizeof(tmp_v[0]) * dup_v_cnt);
	if (!tmp_v) {
		goto err;
	}
	for (i = 0; i < dup_v_cnt; ++i) {
		mathVec3Copy(tmp_v[i], dup_v[i]);
	}
	ac->fn_free(ac, dup_v);
	*ret_v = tmp_v;
	*ret_v_cnt = dup_v_cnt;
	*ret_tri_v_indices = dup_tri_v_indices;
	return 1;
err:
	ac->fn_free(ac, dup_v);
	ac->fn_free(ac, dup_tri_v_indices);
	return 0;
}

static int MeshCookingStage_SplitFaces(const CCTNum_t(*v)[3], const unsigned int* tri_v_indices, unsigned int tri_v_indices_cnt, GeometryPolygon_t** ret_polygons, unsigned int* ret_polygons_cnt, const GeometryCookingOption_t* opt, GeometryCookingOutput_t* output, const CCTAllocator_t* ac) {
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
		tri_merge_bits = (char*)ac->fn_calloc(ac, 1, tri_cnt / 8 + (tri_cnt % 8 ? 1 : 0));
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
			tmp_parr = (GeometryPolygon_t*)ac->fn_realloc(ac, tmp_polygons, (tmp_polygons_cnt + 1) * sizeof(GeometryPolygon_t));
			if (!tmp_parr) {
				goto err;
			}
			tmp_polygons = tmp_parr;
			new_pg = tmp_polygons + tmp_polygons_cnt;
			if (!_polygon_init(new_pg, v, N, tri_v_indices + i, ac)) {
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
				if (!_polygon_insert_tri_indices(new_pg, tri_v_indices + j, ac)) {
					goto err;
				}
				j = 0;
			}
		}
		ac->fn_free(ac, tri_merge_bits);
	}
	else {
		tmp_polygons = (GeometryPolygon_t*)ac->fn_malloc(ac, tri_cnt * sizeof(GeometryPolygon_t));
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
			if (!_polygon_init(new_pg, v, N, tri_v_indices + i, ac)) {
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
			ac->fn_free(ac, (void*)tmp_polygons[i].tri_v_indices_flat);
		}
		ac->fn_free(ac, tmp_polygons);
	}
	ac->fn_free(ac, tri_merge_bits);
	return 0;
}

static int MeshCookingStage_GenerateFaceEdges(const CCTNum_t(*v)[3], const unsigned int* tri_v_indices, unsigned int tri_v_indices_cnt, const CCTNum_t plane_n[3], unsigned int** ret_edge_v_indices, unsigned int* ret_edge_v_indices_cnt, const CCTAllocator_t* ac) {
	unsigned int i, j;
	unsigned int* tmp_edge_pair_indices = NULL;
	unsigned int tmp_edge_pair_indices_cnt = 0;
	unsigned int* tmp_edge_indices = NULL;
	unsigned int tmp_edge_indices_cnt = 0;
	/* Filters all no-border edges */
	if (3 == tri_v_indices_cnt) {
		tmp_edge_indices = (unsigned int*)ac->fn_malloc(ac, sizeof(tmp_edge_indices[0]) * 6);
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
						const CCTNum_t* pi, * pj;
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
			ptr = (unsigned int*)ac->fn_realloc(ac, tmp_edge_pair_indices, tmp_edge_pair_indices_cnt * sizeof(tmp_edge_pair_indices[0]));
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
	tmp_edge_indices = (unsigned int*)ac->fn_malloc(ac, sizeof(tmp_edge_indices[0]) * tmp_edge_indices_cnt);
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
	ac->fn_free(ac, tmp_edge_pair_indices);
	*ret_edge_v_indices = tmp_edge_indices;
	*ret_edge_v_indices_cnt = tmp_edge_indices_cnt;
	return 1;
err:
	ac->fn_free(ac, tmp_edge_indices);
	ac->fn_free(ac, tmp_edge_pair_indices);
	return 0;
}

static int MeshCookingStage_GenerateIndices(const unsigned int* edge_v_indices_flat, unsigned int edge_v_indices_cnt, unsigned int** ret_v_indices, unsigned int* ret_v_indices_cnt, unsigned int** ret_edge_v_ids_flat, const CCTAllocator_t* ac) {
	unsigned int* tmp_edge_v_ids_flat;
	unsigned int* tmp_v_indices;
	unsigned int tmp_v_indices_cnt, i;
	tmp_edge_v_ids_flat = (unsigned int*)ac->fn_malloc(ac, sizeof(tmp_edge_v_ids_flat[0]) * edge_v_indices_cnt);
	if (!tmp_edge_v_ids_flat) {
		return 0;
	}
	tmp_v_indices = (unsigned int*)ac->fn_malloc(ac, sizeof(tmp_v_indices[0]) * edge_v_indices_cnt);
	if (!tmp_v_indices) {
		ac->fn_free(ac, tmp_edge_v_ids_flat);
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
		unsigned int* buf = (unsigned int*)ac->fn_malloc(ac, sizeof(tmp_v_indices[0]) * tmp_v_indices_cnt);
		if (!buf) {
			ac->fn_free(ac, tmp_v_indices);
			return 0;
		}
		for (i = 0; i < tmp_v_indices_cnt; ++i) {
			buf[i] = tmp_v_indices[i];
		}
		ac->fn_free(ac, tmp_v_indices);
		tmp_v_indices = buf;
	}
	*ret_v_indices = tmp_v_indices;
	*ret_v_indices_cnt = tmp_v_indices_cnt;
	*ret_edge_v_ids_flat = tmp_edge_v_ids_flat;
	return 1;
}

static unsigned int MeshCookingStage_MergeFaceEdges(unsigned int* edge_v_indices_flat, unsigned int edge_v_indices_cnt, const GeometryPolygon_t* polygon) {
	unsigned int i, polygon_edge_v_indices_cnt = polygon->edge_cnt + polygon->edge_cnt;
	for (i = 0; i < polygon_edge_v_indices_cnt; i += 2) {
		unsigned int j;
		for (j = 0; j < edge_v_indices_cnt; j += 2) {
			if (edge_v_indices_flat[j] == polygon->edge_v_indices_flat[i] && edge_v_indices_flat[j + 1] == polygon->edge_v_indices_flat[i + 1]) {
				break;
			}
			if (edge_v_indices_flat[j] == polygon->edge_v_indices_flat[i + 1] && edge_v_indices_flat[j + 1] == polygon->edge_v_indices_flat[i]) {
				break;
			}
		}
		if (j < edge_v_indices_cnt) {
			continue;
		}
		edge_v_indices_flat[edge_v_indices_cnt++] = polygon->edge_v_indices_flat[i];
		edge_v_indices_flat[edge_v_indices_cnt++] = polygon->edge_v_indices_flat[i + 1];
	}
	return edge_v_indices_cnt;
}

static GeometryPolygonVertexAdjacentInfo_t* Polygon_VertexAdjacentInfo(const unsigned int* edge_v_ids_flat, unsigned int edge_v_indices_cnt, unsigned int v_id, GeometryPolygonVertexAdjacentInfo_t* info) {
	unsigned int i, j = 0;
	for (i = 0; i < edge_v_indices_cnt; ++i) {
		if (edge_v_ids_flat[i++] == v_id) {
			info->v_ids[j] = edge_v_ids_flat[i];
			info->edge_ids[j] = (i >> 1);
			++j;
			if (j >= 2) {
				return info;
			}
		}
		else if (edge_v_ids_flat[i] == v_id) {
			info->v_ids[j] = edge_v_ids_flat[i - 1];
			info->edge_ids[j] = (i >> 1);
			++j;
			if (j >= 2) {
				return info;
			}
		}
	}
	return NULL;
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

static unsigned int* Cooking_ConcavePolygonTriangleEdge(const CCTNum_t(*v)[3], const unsigned int* edge_v_indices_flat, unsigned int edge_v_indices_cnt, const unsigned int* tri_v_indices_flat, unsigned int tri_v_indices_cnt, const CCTAllocator_t* ac) {
	unsigned int i, j;
	unsigned int* tri_edge_ids = (unsigned int*)ac->fn_malloc(ac, sizeof(tri_edge_ids[0]) * tri_v_indices_cnt);
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

static unsigned int* Cooking_ConcavePolygonTriangleVertex(const unsigned int* v_indices, unsigned int v_indices_cnt, const unsigned int* tri_v_indices, unsigned int tri_v_indices_cnt, const CCTAllocator_t* ac) {
	unsigned int i, j;
	unsigned int* tri_v_ids = (unsigned int*)ac->fn_malloc(ac, sizeof(tri_v_ids[0]) * tri_v_indices_cnt);
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

static unsigned int* Polygon_Save_MeshEdgeIds(const GeometryPolygon_t* polygon, const unsigned int* mesh_edge_v_indices, unsigned int mesh_edge_v_indices_cnt, const CCTAllocator_t* ac) {
	unsigned int i, polygon_edge_v_indices_cnt;
	unsigned int* mesh_edge_ids = (unsigned int*)ac->fn_malloc(ac, sizeof(mesh_edge_ids[0]) * polygon->edge_cnt);
	if (!mesh_edge_ids) {
		return NULL;
	}
	polygon_edge_v_indices_cnt = polygon->edge_cnt + polygon->edge_cnt;
	for (i = 0; i < polygon_edge_v_indices_cnt; ++i) {
		unsigned int v_idx[2], edge_id;
		v_idx[0] = polygon->edge_v_indices_flat[i++];
		v_idx[1] = polygon->edge_v_indices_flat[i];
		edge_id = mathFindEdgeIdByVertexIndices(mesh_edge_v_indices, mesh_edge_v_indices_cnt, v_idx[0], v_idx[1]);
		if (-1 == edge_id) {
			ac->fn_free(ac, mesh_edge_ids);
			return NULL;
		}
		mesh_edge_ids[i >> 1] = edge_id;
	}
	return mesh_edge_ids;
}

static unsigned int* Polygon_Save_MeshVertexIds(const GeometryPolygon_t* polygon, const unsigned int* mesh_v_indices, unsigned int mesh_v_indices_cnt, const CCTAllocator_t* ac) {
	unsigned int i;
	unsigned int* mesh_v_ids = (unsigned int*)ac->fn_malloc(ac, sizeof(mesh_v_ids[0]) * polygon->v_indices_cnt);
	if (!mesh_v_ids) {
		return NULL;
	}
	for (i = 0; i < polygon->v_indices_cnt; ++i) {
		unsigned int j;
		for (j = 0; j < mesh_v_indices_cnt; ++j) {
			if (polygon->v_indices[i] == mesh_v_indices[j]) {
				mesh_v_ids[i] = j;
				break;
			}
		}
		if (j < mesh_v_indices_cnt) {
			continue;
		}
		ac->fn_free(ac, mesh_v_ids);
		return NULL;
	}
	return mesh_v_ids;
}

static void ConvexMesh_FacesNormalOut(GeometryMesh_t* mesh) {
	unsigned int i;
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		GeometryPolygon_t* polygon = mesh->polygons + i;
		unsigned int j;
		for (j = 0; j < mesh->v_indices_cnt; ++j) {
			CCTNum_t vj[3], dot;
			mathVec3Sub(vj, mesh->v[mesh->v_indices[j]], polygon->v[polygon->v_indices[0]]);
			dot = mathVec3Dot(polygon->normal, vj);
			/* some module needed epsilon */
			if (dot > CCT_EPSILON) {
				mathVec3Negate(polygon->normal, polygon->normal);
				break;
			}
		}
	}
}

static int Cooking_MeshVertexAdjacentInfo(const GeometryMesh_t* mesh, unsigned int v_id, GeometryMeshVertexAdjacentInfo_t* info, const CCTAllocator_t* ac) {
	unsigned int i, *tmp_p;
	unsigned int mesh_edge_v_indices_cnt;
	unsigned int *v_ids = NULL, *edge_ids = NULL, *face_ids = NULL;
	/* find locate faces */
	info->face_cnt = 0;
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		unsigned int offset = mathFindFaceIdByVertexIndices(mesh->polygons + i, mesh->polygons_cnt - i, &mesh->v_indices[v_id], 1);
		if (-1 == offset) {
			break;
		}
		++info->face_cnt;
		tmp_p = (unsigned int*)ac->fn_realloc(ac, face_ids, info->face_cnt * sizeof(face_ids[0]));
		if (!tmp_p) {
			goto err;
		}
		i += offset;
		face_ids = tmp_p;
		face_ids[info->face_cnt - 1] = i;
	}
	if (info->face_cnt <= 0) {
		/* no possiable */
		goto err;
	}
	/* find locate edge */
	info->v_cnt = info->edge_cnt = 0;
	mesh_edge_v_indices_cnt = mesh->edge_cnt + mesh->edge_cnt;
	for (i = 0; i < mesh_edge_v_indices_cnt; ++i) {
		unsigned int adj_v_id;
		if (mesh->edge_v_ids_flat[i++] == v_id) {
			adj_v_id = mesh->edge_v_ids_flat[i];
		}
		else if (mesh->edge_v_ids_flat[i] == v_id) {
			adj_v_id = mesh->edge_v_ids_flat[i - 1];
		}
		else {
			continue;
		}
		++info->v_cnt;
		tmp_p = (unsigned int*)ac->fn_realloc(ac, v_ids, info->v_cnt * sizeof(v_ids[0]));
		if (!tmp_p) {
			goto err;
		}
		v_ids = tmp_p;
		v_ids[info->v_cnt - 1] = adj_v_id;

		++info->edge_cnt;
		tmp_p = (unsigned int*)ac->fn_realloc(ac, edge_ids, info->edge_cnt * sizeof(edge_ids[0]));
		if (!tmp_p) {
			goto err;
		}
		edge_ids = tmp_p;
		edge_ids[info->edge_cnt - 1] = (i >> 1);
	}
	if (info->v_cnt != info->edge_cnt || info->v_cnt <= 0) {
		/* no possiable */
		goto err;
	}
	info->v_ids = v_ids;
	info->edge_ids = edge_ids;
	info->face_ids = face_ids;
	return 1;
err:
	ac->fn_free(ac, v_ids);
	ac->fn_free(ac, edge_ids);
	ac->fn_free(ac, face_ids);
	return 0;
}

static int Cooking_MeshEdgeAdjacentFace(const GeometryMesh_t* mesh, unsigned int edge_id, unsigned int adjacent_faces_ids[2]) {
	unsigned int i, cnt = 0;
	const unsigned int v_idx[2] = {
		mesh->edge_v_indices_flat[edge_id + edge_id],
		mesh->edge_v_indices_flat[edge_id + edge_id + 1]
	};
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		unsigned int offset = mathFindFaceIdByVertexIndices(mesh->polygons + i, mesh->polygons_cnt - i, v_idx, 2);
		if (-1 == offset) {
			break;
		}
		i += offset;
		adjacent_faces_ids[cnt++] = i;
		if (cnt >= 2) {
			return 1;
		}
	}
	if (1 == cnt) {
		adjacent_faces_ids[1] = -1;
		return 1;
	}
	return 0;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
extern "C" {
#endif

const GeometryCookingOutput_t* mathCookingMesh(const CCTNum_t(*v)[3], const unsigned int* tri_v_indices, unsigned int tri_v_indices_cnt, const GeometryCookingOption_t* opt, GeometryCookingOutput_t* output, const CCTAllocator_t* ac) {
	CCTNum_t(*dup_v)[3] = NULL;
	unsigned int* dup_tri_v_indices = NULL;
	unsigned int dup_v_cnt = 0, i;
	GeometryPolygon_t* tmp_polygons = NULL;
	unsigned int tmp_polygons_cnt = 0;
	unsigned int total_edge_v_indices_cnt;
	unsigned int* edge_v_indices_flat = NULL, *edge_v_ids_flat = NULL;
	unsigned int* v_indices = NULL;
	unsigned int v_indices_cnt;
	GeometryMeshVertexAdjacentInfo_t* v_adjacent_infos = NULL;
	unsigned int* edge_adjacent_face_ids = NULL;
	GeometryMesh_t* mesh = output->mesh_ptr;
	output->error_code = 1;
	output->has_invalid_tri = 0;
	output->has_isolate_vertex = 0;
	if (!ac) {
		ac = CCTAllocator_stdc(NULL);
	}
	/* check */
	if (tri_v_indices_cnt < 3) {
		return output;
	}
	/* merge distinct vertices, rewrite indices */
	if (!MathCookingStage_DistinctVertices(v, tri_v_indices, tri_v_indices_cnt, &dup_v, &dup_v_cnt, &dup_tri_v_indices, ac)) {
		goto err_0;
	}
	/* split all face */
	if (!MeshCookingStage_SplitFaces((const CCTNum_t(*)[3])dup_v, dup_tri_v_indices, tri_v_indices_cnt, &tmp_polygons, &tmp_polygons_cnt, opt, output, ac)) {
		goto err_0;
	}
	ac->fn_free(ac, dup_tri_v_indices);
	dup_tri_v_indices = NULL;
	/* cooking every face */
	total_edge_v_indices_cnt = 0;
	for (i = 0; i < tmp_polygons_cnt; ++i) {
		unsigned int* edge_v_indices_flat = NULL, *v_indices = NULL, *edge_v_ids_flat = NULL;
		GeometryPolygonVertexAdjacentInfo_t* v_adjacent_infos = NULL;
		unsigned int edge_v_indices_cnt, v_indices_cnt, j;
		GeometryPolygon_t* pg = tmp_polygons + i;
		/* cooking edge */
		if (!MeshCookingStage_GenerateFaceEdges((const CCTNum_t(*)[3])pg->v, pg->tri_v_indices_flat, pg->tri_cnt * 3, pg->normal, &edge_v_indices_flat, &edge_v_indices_cnt, ac)) {
			goto err_1;
		}
		pg->edge_cnt = edge_v_indices_cnt / 2;
		pg->edge_v_indices_flat = edge_v_indices_flat;
		/* cooking vertex indice */
		if (!MeshCookingStage_GenerateIndices(edge_v_indices_flat, edge_v_indices_cnt, &v_indices, &v_indices_cnt, &edge_v_ids_flat, ac)) {
			goto err_1;
		}
		total_edge_v_indices_cnt += edge_v_indices_cnt;
		pg->v_indices = v_indices;
		pg->v_indices_cnt = v_indices_cnt;
		pg->edge_v_ids_flat = edge_v_ids_flat;
		/* cooking vertex adjacent infos */
		v_adjacent_infos = (GeometryPolygonVertexAdjacentInfo_t*)ac->fn_malloc(ac, sizeof(v_adjacent_infos[0]) * v_indices_cnt);
		if (!v_adjacent_infos) {
			goto err_1;
		}
		pg->v_adjacent_infos = v_adjacent_infos;
		for (j = 0; j < v_indices_cnt; ++j) {
			if (!Polygon_VertexAdjacentInfo(edge_v_ids_flat, edge_v_indices_cnt, j, v_adjacent_infos + j)) {
				output->has_isolate_vertex = 1;
				mathVec3Copy(output->isolate_v, pg->v[pg->v_indices[j]]);
				goto err_1;
			}
		}
		/* fill polygon other data */
		mathVertexIndicesAverageXYZ((const CCTNum_t(*)[3])pg->v, v_indices, v_indices_cnt, pg->center);
	}
	/* merge face edge */
	edge_v_indices_flat = (unsigned int*)ac->fn_malloc(ac, sizeof(edge_v_indices_flat[0]) * total_edge_v_indices_cnt);
	if (!edge_v_indices_flat) {
		goto err_1;
	}
	total_edge_v_indices_cnt = 0;
	for (i = 0; i < tmp_polygons_cnt; ++i) {
		total_edge_v_indices_cnt = MeshCookingStage_MergeFaceEdges(edge_v_indices_flat, total_edge_v_indices_cnt, tmp_polygons + i);
	}
	/* cooking vertex indice */
	if (!MeshCookingStage_GenerateIndices(edge_v_indices_flat, total_edge_v_indices_cnt, &v_indices, &v_indices_cnt, &edge_v_ids_flat, ac)) {
		goto err_1;
	}
	/* alloc adjacent infos buffer */
	v_adjacent_infos = (GeometryMeshVertexAdjacentInfo_t*)ac->fn_malloc(ac, sizeof(v_adjacent_infos[0]) * v_indices_cnt);
	if (!v_adjacent_infos) {
		goto err_1;
	}
	edge_adjacent_face_ids = (unsigned int*)ac->fn_malloc(ac, sizeof(edge_adjacent_face_ids[0]) * total_edge_v_indices_cnt);
	if (!edge_adjacent_face_ids) {
		goto err_1;
	}
	/* cooking face map relationship data */
	for (i = 0; i < tmp_polygons_cnt; ++i) {
		GeometryPolygon_t* pg = tmp_polygons + i;
		pg->mesh_v_ids = Polygon_Save_MeshVertexIds(pg, v_indices, v_indices_cnt, ac);
		if (!pg->mesh_v_ids) {
			goto err_1;
		}
		pg->mesh_edge_ids = Polygon_Save_MeshEdgeIds(pg, edge_v_indices_flat, total_edge_v_indices_cnt, ac);
		if (!pg->mesh_edge_ids) {
			goto err_1;
		}
	}
	/* save basic data result */
	mathVerticesFindMinMaxXYZ((const CCTNum_t(*)[3])dup_v, dup_v_cnt, mesh->bound_box.min_v, mesh->bound_box.max_v);
	mathAABBFixSize(mesh->bound_box.min_v, mesh->bound_box.max_v);
	mesh->v = dup_v;
	mesh->v_indices = v_indices;
	mesh->v_indices_cnt = v_indices_cnt;
	mesh->edge_v_ids_flat = edge_v_ids_flat;
	mesh->edge_v_indices_flat = edge_v_indices_flat;
	mesh->edge_cnt = total_edge_v_indices_cnt / 2;
	mesh->polygons = tmp_polygons;
	mesh->polygons_cnt = tmp_polygons_cnt;
	/* cooking vertex adjacent infos */
	for (i = 0; i < v_indices_cnt; ++i) {
		if (Cooking_MeshVertexAdjacentInfo(mesh, i, v_adjacent_infos + i, ac)) {
			continue;
		}
		while (i--) {
			free_data_mesh_vertex_adjacent_info(v_adjacent_infos + i, ac);
		}
		goto err_1;
	}
	mesh->v_adjacent_infos = v_adjacent_infos;
	/* cooking edge adjacent infos */
	for (i = 0; i < mesh->edge_cnt; ++i) {
		if (!Cooking_MeshEdgeAdjacentFace(mesh, i, edge_adjacent_face_ids + i + i)) {
			goto err_1;
		}
	}
	mesh->edge_adjacent_face_ids_flat = edge_adjacent_face_ids;
	/* check mesh is convex and closed */
	mesh->is_convex = mathMeshIsConvex(mesh);
	if (mesh->is_convex) {
		for (i = 0; i < mesh->polygons_cnt; ++i) {
			GeometryPolygon_t* polygon = mesh->polygons + i;
			polygon->is_convex = 1;
		}
		ConvexMesh_FacesNormalOut(mesh);
	}
	else {
		for (i = 0; i < mesh->polygons_cnt; ++i) {
			GeometryPolygon_t* polygon = mesh->polygons + i;
			unsigned int polygon_edge_v_indices_cnt = polygon->edge_cnt * 2;
			polygon->is_convex = Polygon_IsConvex(
				(const CCTNum_t(*)[3])polygon->v, polygon->normal,
				polygon->edge_v_indices_flat, polygon_edge_v_indices_cnt,
				polygon->v_indices, polygon->v_indices_cnt
			);
			/* if concave, save triangle edge ids */
			if (!polygon->is_convex) {
				unsigned int* concave_tri_edge_ids, *concave_tri_v_ids;

				concave_tri_edge_ids = Cooking_ConcavePolygonTriangleEdge(
					(const CCTNum_t(*)[3])polygon->v,
					polygon->edge_v_indices_flat, polygon_edge_v_indices_cnt,
					polygon->tri_v_indices_flat, polygon->tri_cnt * 3,
					ac
				);
				if (!concave_tri_edge_ids) {
					goto err_1;
				}
				polygon->concave_tri_edge_ids_flat = concave_tri_edge_ids;

				concave_tri_v_ids = Cooking_ConcavePolygonTriangleVertex(
					polygon->v_indices, polygon->v_indices_cnt,
					polygon->tri_v_indices_flat, polygon->tri_cnt * 3,
					ac
				);
				if (!concave_tri_v_ids) {
					goto err_1;
				}
				polygon->concave_tri_v_ids_flat = concave_tri_v_ids;
			}
		}
	}
	mesh->is_closed = mathMeshIsClosed(mesh);
	mesh->_is_buffer_view = 0;
	/* finish */
	mesh->allocator_ptr = ac;
	output->error_code = 0;
	return output;
err_1:
	for (i = 0; i < tmp_polygons_cnt; ++i) {
		Polygon_ClearWithoutVertices(tmp_polygons + i, ac);
	}
	ac->fn_free(ac, tmp_polygons);
err_0:
	ac->fn_free(ac, dup_v);
	ac->fn_free(ac, v_indices);
	ac->fn_free(ac, edge_v_ids_flat);
	ac->fn_free(ac, edge_v_indices_flat);
	ac->fn_free(ac, dup_tri_v_indices);
	ac->fn_free(ac, v_adjacent_infos);
	ac->fn_free(ac, edge_adjacent_face_ids);
	return output;
}

#ifdef __cplusplus
}
#endif
