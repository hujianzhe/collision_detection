//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/vertex.h"
#include "../inc/aabb.h"
#include "../inc/plane.h"
#include "../inc/polygon.h"
#include "../inc/mesh.h"
#include <stdlib.h>

extern GeometryPolygon_t* PolygonCooking_InternalProc(const CCTNum_t(*v)[3], const unsigned int* tri_indices, unsigned int tri_indices_cnt, GeometryPolygon_t* polygon);
extern int Polygon_Convex_HasPoint_InternalProc(const GeometryPolygon_t* polygon, const CCTNum_t p[3]);

static int Mesh_Cooking_Edge_InternalProc(const CCTNum_t (*v)[3], GeometryMesh_t* mesh) {
	unsigned int i;
	unsigned int* ret_edge_indices = NULL;
	unsigned int ret_edge_indices_cnt = 0;
	const GeometryPolygon_t* polygons = mesh->polygons;
	unsigned int polygons_cnt = mesh->polygons_cnt;

	for (i = 0; i < polygons_cnt; ++i) {
		const GeometryPolygon_t* polygon = polygons + i;
		unsigned int j;
		for (j = 0; j < polygon->v_indices_cnt; ) {
			unsigned int k;
			unsigned int s = polygon->v_indices[j++];
			unsigned int e = polygon->v_indices[j >= polygon->v_indices_cnt ? 0 : j];
			unsigned int* new_p;
			for (k = 0; k < ret_edge_indices_cnt; k += 2) {
				if (s == ret_edge_indices[k] || mathVec3Equal(v[s], v[ret_edge_indices[k]])) {
					if (e == ret_edge_indices[k+1] || mathVec3Equal(v[e], v[ret_edge_indices[k+1]])) {
						break;
					}
				}
				else if (s == ret_edge_indices[k+1] || mathVec3Equal(v[s], v[ret_edge_indices[k+1]])) {
					if ((e == ret_edge_indices[k] || mathVec3Equal(v[e], v[ret_edge_indices[k]]))) {
						break;
					}
				}
			}
			if (k != ret_edge_indices_cnt) {
				continue;
			}
			ret_edge_indices_cnt += 2;
			new_p = (unsigned int*)realloc(ret_edge_indices, ret_edge_indices_cnt * sizeof(ret_edge_indices[0]));
			if (!new_p) {
				free(ret_edge_indices);
				return 0;
			}
			ret_edge_indices = new_p;
			ret_edge_indices[ret_edge_indices_cnt - 2] = s;
			ret_edge_indices[ret_edge_indices_cnt - 1] = e;
		}
	}

	mesh->edge_indices = ret_edge_indices;
	mesh->edge_indices_cnt = ret_edge_indices_cnt;
	return 1;
}

static GeometryPolygon_t* _insert_tri_indices(GeometryPolygon_t* polygon, const unsigned int* tri_indices) {
	unsigned int cnt = polygon->tri_indices_cnt;
	unsigned int* new_p = (unsigned int*)realloc((void*)polygon->tri_indices, sizeof(polygon->tri_indices[0]) * (cnt + 3));
	if (!new_p) {
		return NULL;
	}
	new_p[cnt++] = tri_indices[0];
	new_p[cnt++] = tri_indices[1];
	new_p[cnt++] = tri_indices[2];
	polygon->tri_indices = new_p;
	polygon->tri_indices_cnt = cnt;
	return polygon;
}

static int _polygon_can_merge_triangle(GeometryPolygon_t* polygon, const CCTNum_t p0[3], const CCTNum_t p1[3], const CCTNum_t p2[3]) {
	unsigned int i, n = 0;
	const CCTNum_t* tri_p[] = { p0, p1, p2 };
	for (i = 0; i < 3; ++i) {
		unsigned int j;
		if (!Plane_Contain_Point(polygon->v[polygon->tri_indices[0]], polygon->normal, tri_p[i])) {
			return 0;
		}
		if (n >= 2) {
			continue;
		}
		for (j = 0; j < polygon->tri_indices_cnt; ++j) {
			const CCTNum_t* pv = polygon->v[polygon->tri_indices[j]];
			if (mathVec3Equal(pv, tri_p[i])) {
				++n;
				break;
			}
		}
	}
	return n >= 2;
}

static int Mesh_Cooking_Polygen_InternalProc(const CCTNum_t (*v)[3], const unsigned int* tri_indices, unsigned int tri_indices_cnt, GeometryMesh_t* mesh) {
	unsigned int i;
	GeometryPolygon_t* tmp_polygons = NULL;
	unsigned int tmp_polygons_cnt = 0;
	unsigned int tri_cnt;
	char* tri_merge_bits = NULL;

	tri_cnt = tri_indices_cnt / 3;
	if (tri_cnt < 1) {
		goto err;
	}
	tri_merge_bits = (char*)calloc(1, tri_cnt / 8 + (tri_cnt % 8 ? 1 : 0));
	if (!tri_merge_bits) {
		goto err;
	}
	/* Merge triangles on the same plane */
	for (i = 0; i < tri_indices_cnt; i += 3) {
		unsigned int j, tri_idx;
		CCTNum_t N[3];
		GeometryPolygon_t* tmp_parr, * new_pg;

		tri_idx = i / 3;
		if (tri_merge_bits[tri_idx / 8] & (1 << (tri_idx % 8))) {
			continue;
		}
		mathPlaneNormalByVertices3(v[tri_indices[i]], v[tri_indices[i + 1]], v[tri_indices[i + 2]], N);
		if (mathVec3IsZero(N)) {
			goto err;
		}
		tmp_parr = (GeometryPolygon_t*)realloc(tmp_polygons, (tmp_polygons_cnt + 1) * sizeof(GeometryPolygon_t));
		if (!tmp_parr) {
			goto err;
		}
		tmp_polygons = tmp_parr;
		new_pg = tmp_polygons + tmp_polygons_cnt;
		tmp_polygons_cnt++;

		new_pg->v_indices = NULL;
		new_pg->v_indices_cnt = 0;
		new_pg->tri_indices = NULL;
		new_pg->tri_indices_cnt = 0;
		if (!_insert_tri_indices(new_pg, tri_indices + i)) {
			goto err;
		}
		new_pg->v = (CCTNum_t(*)[3])v;
		mathVec3Copy(new_pg->normal, N);

		tri_merge_bits[tri_idx / 8] |= (1 << (tri_idx % 8));
		for (j = 0; j < tri_indices_cnt; j += 3) {
			tri_idx = j / 3;
			if (tri_merge_bits[tri_idx / 8] & (1 << tri_idx % 8)) {
				continue;
			}
			if (!_polygon_can_merge_triangle(new_pg,
				v[tri_indices[j]], v[tri_indices[j + 1]], v[tri_indices[j + 2]]))
			{
				continue;
			}
			if (!_insert_tri_indices(new_pg, tri_indices + j)) {
				goto err;
			}
			tri_merge_bits[tri_idx / 8] |= (1 << (tri_idx % 8));
			j = 0;
		}
	}
	free(tri_merge_bits);
	tri_merge_bits = NULL;
	/* Cooking all polygen */
	for (i = 0; i < tmp_polygons_cnt; ++i) {
		GeometryPolygon_t* polygon = tmp_polygons + i;
		if (!PolygonCooking_InternalProc((const CCTNum_t(*)[3])polygon->v, polygon->tri_indices, polygon->tri_indices_cnt, polygon)) {
			goto err;
		}
	}
	mesh->polygons = tmp_polygons;
	mesh->polygons_cnt = tmp_polygons_cnt;
	return 1;
err:
	if (tmp_polygons) {
		for (i = 0; i < tmp_polygons_cnt; ++i) {
			tmp_polygons[i].v = NULL;
			mathPolygonFreeCookingData(tmp_polygons + i);
		}
		free(tmp_polygons);
	}
	free(tri_merge_bits);
	return 0;
}

static int ConvexMesh_Contain_Point_InternalProc(const GeometryMesh_t* mesh, const CCTNum_t p[3]) {
	unsigned int i;
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		CCTNum_t v[3], dot;
		const GeometryPolygon_t* polygon = mesh->polygons + i;
		mathVec3Sub(v, p, polygon->v[polygon->v_indices[0]]);
		dot = mathVec3Dot(v, polygon->normal);
		if (dot < CCT_EPSILON_NEGATE) {
			continue;
		}
		if (dot > CCT_EPSILON) {
			return 0;
		}
		return Polygon_Convex_HasPoint_InternalProc(polygon, p);
	}
	return 1;
}

#ifdef __cplusplus
extern "C" {
#endif

GeometryMesh_t* mathMeshCooking(const CCTNum_t (*v)[3], unsigned int v_cnt, const unsigned int* tri_indices, unsigned int tri_indices_cnt, GeometryMesh_t* mesh) {
	CCTNum_t(*dup_v)[3] = NULL;
	CCTNum_t min_v[3], max_v[3];
	unsigned int i, dup_v_cnt;
	unsigned int* dup_v_indices = NULL;
	unsigned int* dup_tri_indices = NULL;

	if (v_cnt < 3 || tri_indices_cnt < 3) {
		goto err_0;
	}
	dup_v_cnt = mathVerticesDistinctCount(v, v_cnt);
	if (dup_v_cnt < 3) {
		goto err_0;
	}
	dup_v = (CCTNum_t(*)[3])malloc(dup_v_cnt * sizeof(dup_v[0]));
	if (!dup_v) {
		goto err_0;
	}
	dup_v_indices = (unsigned int*)malloc(sizeof(dup_v_indices[0]) * dup_v_cnt);
	if (!dup_v_indices) {
		goto err_0;
	}
	dup_tri_indices = (unsigned int*)malloc(sizeof(tri_indices[0]) * tri_indices_cnt);
	if (!dup_tri_indices) {
		goto err_0;
	}
	mathVerticesMerge(v, v_cnt, tri_indices, tri_indices_cnt, dup_v, dup_tri_indices);

	if (!Mesh_Cooking_Polygen_InternalProc((const CCTNum_t(*)[3])dup_v, dup_tri_indices, tri_indices_cnt, mesh)) {
		goto err_0;
	}
	if (!Mesh_Cooking_Edge_InternalProc((const CCTNum_t(*)[3])dup_v, mesh)) {
		goto err_1;
	}
	free(dup_tri_indices);
	for (i = 0; i < dup_v_cnt; ++i) {
		dup_v_indices[i] = i;
	}
	mathVerticesFindMinMaxXYZ((const CCTNum_t(*)[3])dup_v, dup_v_cnt, min_v, max_v);
	mathAABBFromTwoVertice(min_v, max_v, mesh->bound_box.o, mesh->bound_box.half);
	mathVec3Set(mesh->o, CCTNums_3(0.0, 0.0, 0.0));
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		mathVec3Set(mesh->polygons[i].o, CCTNums_3(0.0, 0.0, 0.0));
	}
	mesh->v = dup_v;
	mesh->v_indices = dup_v_indices;
	mesh->v_indices_cnt = dup_v_cnt;
	mathConvexMeshMakeFacesOut(mesh);
	return mesh;
err_1:
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		mesh->polygons[i].v = NULL;
		mathPolygonFreeCookingData(mesh->polygons + i);
	}
	free(mesh->polygons);
err_0:
	free(dup_v);
	free(dup_v_indices);
	free(dup_tri_indices);
	return NULL;
}

GeometryMesh_t* mathMeshDeepCopy(GeometryMesh_t* dst, const GeometryMesh_t* src) {
	unsigned int i, dup_v_cnt = 0;
	CCTNum_t(*dup_v)[3] = NULL;
	unsigned int* dup_v_indices = NULL;
	unsigned int* dup_edge_indices = NULL;
	GeometryPolygon_t* dup_polygons = NULL;
	/* find max vertex index, dup_v_cnt */
	for (i = 0; i < src->v_indices_cnt; ++i) {
		if (src->v_indices[i] >= dup_v_cnt) {
			dup_v_cnt = src->v_indices[i] + 1;
		}
	}
	for (i = 0; i < src->edge_indices_cnt; ++i) {
		if (src->edge_indices[i] >= dup_v_cnt) {
			dup_v_cnt = src->edge_indices[i] + 1;
		}
	}
	for (i = 0; i < src->polygons_cnt; ++i) {
		const GeometryPolygon_t* src_polygon = src->polygons + i;
		unsigned int j;
		for (j = 0; j < src_polygon->v_indices_cnt; ++j) {
			if (src_polygon->v_indices[j] >= dup_v_cnt) {
				dup_v_cnt = src_polygon->v_indices[j] + 1;
			}
		}
		for (j = 0; j < src_polygon->tri_indices_cnt; ++j) {
			if (src_polygon->tri_indices[j] >= dup_v_cnt) {
				dup_v_cnt = src_polygon->tri_indices[j] + 1;
			}
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
	dup_edge_indices = (unsigned int*)malloc(sizeof(dup_edge_indices[0]) * src->edge_indices_cnt);
	if (!dup_edge_indices) {
		goto err_0;
	}
	dup_polygons = (GeometryPolygon_t*)malloc(sizeof(dup_polygons[0]) * src->polygons_cnt);
	if (!dup_polygons) {
		goto err_0;
	}
	for (i = 0; i < src->polygons_cnt; ++i) {
		unsigned int j;
		const GeometryPolygon_t* src_polygon = src->polygons + i;
		unsigned int* dup_v_indices = NULL, *dup_tri_indices = NULL;
		dup_v_indices = (unsigned int*)malloc(sizeof(dup_v_indices[0]) * src_polygon->v_indices_cnt);
		if (!dup_v_indices) {
			goto err_1;
		}
		dup_tri_indices = (unsigned int*)malloc(sizeof(dup_tri_indices[0]) * src_polygon->tri_indices_cnt);
		if (!dup_tri_indices) {
			goto err_1;
		}
		for (j = 0; j < src_polygon->v_indices_cnt; ++j) {
			dup_v_indices[j] = src_polygon->v_indices[j];
		}
		for (j = 0; j < src_polygon->tri_indices_cnt; ++j) {
			dup_tri_indices[j] = src_polygon->tri_indices[j];
		}
		mathVec3Copy(dup_polygons[i].o, src_polygon->o);
		mathVec3Copy(dup_polygons[i].normal, src_polygon->normal);
		dup_polygons[i].v = dup_v;
		dup_polygons[i].v_indices_cnt = src_polygon->v_indices_cnt;
		dup_polygons[i].tri_indices_cnt = src_polygon->tri_indices_cnt;
		dup_polygons[i].v_indices = dup_v_indices;
		dup_polygons[i].tri_indices = dup_tri_indices;
		continue;
	err_1:
		free(dup_v_indices);
		free(dup_tri_indices);
		for (j = 0; j < i; ++j) {
			free((void*)dup_polygons[j].v_indices);
			free((void*)dup_polygons[j].tri_indices);
		}
		goto err_0;
	}
	for (i = 0; i < src->v_indices_cnt; ++i) {
		unsigned int idx = src->v_indices[i];
		dup_v_indices[i] = idx;
		mathVec3Copy(dup_v[idx], src->v[idx]);
	}
	for (i = 0; i < src->edge_indices_cnt; ++i) {
		dup_edge_indices[i] = src->edge_indices[i];
	}
	mathVec3Copy(dst->o, src->o);
	dst->v = dup_v;
	dst->bound_box = src->bound_box;
	dst->polygons_cnt = src->polygons_cnt;
	dst->edge_indices_cnt = src->edge_indices_cnt;
	dst->v_indices_cnt = src->v_indices_cnt;
	dst->polygons = dup_polygons;
	dst->edge_indices = dup_edge_indices;
	dst->v_indices = dup_v_indices;
	return dst;
err_0:
	free(dup_v);
	free(dup_v_indices);
	free(dup_edge_indices);
	free(dup_polygons);
	return NULL;
}

void mathMeshFreeCookingData(GeometryMesh_t* mesh) {
	unsigned int i;
	if (!mesh) {
		return;
	}
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		mesh->polygons[i].v = NULL;
		mathPolygonFreeCookingData(mesh->polygons + i);
	}
	if (mesh->polygons) {
		free(mesh->polygons);
		mesh->polygons = NULL;
		mesh->polygons_cnt = 0;
	}
	if (mesh->edge_indices) {
		free((void*)mesh->edge_indices);
		mesh->edge_indices = NULL;
		mesh->edge_indices_cnt = 0;
	}
	if (mesh->v_indices) {
		free((void*)mesh->v_indices);
		mesh->v_indices = NULL;
		mesh->v_indices_cnt = 0;
	}
	if (mesh->v) {
		free(mesh->v);
		mesh->v = NULL;
	}
}

int mathMeshIsClosed(const GeometryMesh_t* mesh) {
	unsigned int i;
	if (mesh->v_indices_cnt < 3) {
		return 0;
	}
	for (i = 0; i < mesh->v_indices_cnt; ++i) {
		unsigned int j, cnt = 0;
		unsigned int vi = mesh->v_indices[i];
		for (j = 0; j < mesh->polygons_cnt; ++j) {
			unsigned int k;
			const GeometryPolygon_t* polygon = mesh->polygons + j;
			for (k = 0; k < polygon->v_indices_cnt; ++k) {
				unsigned int vk = polygon->v_indices[k];
				if (vi == vk || mathVec3Equal(mesh->v[vi], polygon->v[vk])) {
					++cnt;
					break;
				}
			}
			if (cnt >= 3) {
				break;
			}
		}
		if (cnt < 3) {
			return 0;
		}
	}
	return 1;
}

int mathMeshIsConvex(const GeometryMesh_t* mesh) {
	unsigned int i;
	if (mesh->polygons_cnt < 4) {
		return 0;
	}
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		const GeometryPolygon_t* polygon = mesh->polygons + i;
		unsigned int j, has_test_dot = 0;
		CCTNum_t test_dot;
		for (j = 0; j < mesh->v_indices_cnt; ++j) {
			CCTNum_t vj[3], dot;
			mathVec3Sub(vj, mesh->v[mesh->v_indices[j]], polygon->v[polygon->v_indices[0]]);
			dot = mathVec3Dot(polygon->normal, vj);
			if (dot <= CCT_EPSILON && dot >= CCT_EPSILON_NEGATE) {
				continue;
			}
			if (!has_test_dot) {
				has_test_dot = 1;
				test_dot = dot;
				continue;
			}
			if (test_dot > CCTNum(0.0) && dot < CCTNum(0.0)) {
				return 0;
			}
			if (test_dot < CCTNum(0.0) && dot > CCTNum(0.0)) {
				return 0;
			}
		}
	}
	return 1;
}

void mathConvexMeshMakeFacesOut(GeometryMesh_t* mesh) {
	CCTNum_t p[2][3], po[3];
	unsigned int i;
	for (i = 0; i < 2; ++i) {
		CCTNum_t tri[3][3];
		const GeometryPolygon_t* polygon = mesh->polygons + i;
		mathVec3Copy(tri[0], polygon->v[polygon->v_indices[0]]);
		mathVec3Copy(tri[1], polygon->v[polygon->v_indices[1]]);
		mathVec3Copy(tri[2], polygon->v[polygon->v_indices[2]]);
		mathTriangleGetPoint((const CCTNum_t(*)[3])tri, CCTNum(0.5), CCTNum(0.5), p[i]);
	}
	mathVec3Add(po, p[0], p[1]);
	mathVec3MultiplyScalar(po, po, CCTNum(0.5));
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		CCTNum_t v[3], dot;
		GeometryPolygon_t* polygon = mesh->polygons + i;
		mathVec3Sub(v, po, polygon->v[polygon->v_indices[0]]);
		dot = mathVec3Dot(v, polygon->normal);
		if (dot > CCTNum(0.0)) {
			mathVec3Negate(polygon->normal, polygon->normal);
		}
	}
}

int ConvexMesh_Contain_Point(const GeometryMesh_t* mesh, const CCTNum_t p[3]) {
	if (!mathAABBHasPoint(mesh->bound_box.o, mesh->bound_box.half, p)) {
		return 0;
	}
	return ConvexMesh_Contain_Point_InternalProc(mesh, p);
}

int ConvexMesh_Contain_ConvexMesh(const GeometryMesh_t* mesh1, const GeometryMesh_t* mesh2) {
	unsigned int i;
	if (!AABB_Intersect_AABB(mesh1->bound_box.o, mesh1->bound_box.half, mesh2->bound_box.o, mesh2->bound_box.half)) {
		return 0;
	}
	for (i = 0; i < mesh2->v_indices_cnt; ++i) {
		const CCTNum_t* p = mesh2->v[mesh2->v_indices[i]];
		if (!ConvexMesh_Contain_Point_InternalProc(mesh1, p)) {
			return 0;
		}
	}
	return 1;
}

/*
int ConvexMesh_Contain_Point(const GeometryMesh_t* mesh, const CCTNum_t p[3]) {
	CCTNum_t dir[3];
	unsigned int i, again_flag;
	if (!mathAABBHasPoint(mesh->bound_box.o, mesh->bound_box.half, p)) {
		return 0;
	}
	again_flag = 0;
	dir[0] = CCTNum(1.0); dir[1] = dir[2] = CCTNum(0.0);
again:
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		const GeometryPolygon_t* polygon = mesh->polygons + i;
		CCTNum_t d, cos_theta, cast_p[3];
		mathPointProjectionPlane(p, polygon->v[polygon->v_indices[0]], polygon->normal, NULL, &d);
		if (CCT_EPSILON_NEGATE <= d && d <= CCT_EPSILON) {
			if (Polygon_Convex_HasPoint_InternalProc(polygon, p)) {
				return 1;
			}
			continue;
		}
		cos_theta = mathVec3Dot(dir, polygon->normal);
		if (CCT_EPSILON_NEGATE <= cos_theta && cos_theta <= CCT_EPSILON) {
			continue;
		}
		d /= cos_theta;
		if (d < CCTNum(0.0)) {
			continue;
		}
		mathVec3Copy(cast_p, p);
		mathVec3AddScalar(cast_p, dir, d);
		if (Polygon_Convex_HasPoint_InternalProc(polygon, cast_p)) {
			break;
		}
	}
	if (i >= mesh->polygons_cnt) {
		return 0;
	}
	if (!again_flag) {
		dir[0] = CCTNum(-1.0);
		again_flag = 1;
		goto again;
	}
	return 1;
}
*/

#ifdef __cplusplus
}
#endif
