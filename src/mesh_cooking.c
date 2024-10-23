//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/vertex.h"
#include "../inc/aabb.h"
#include "../inc/plane.h"
#include "../inc/polygon.h"
#include "../inc/mesh.h"
#include "../inc/mesh_cooking.h"
#include <stdlib.h>

extern int Plane_Contain_Point(const CCTNum_t plane_v[3], const CCTNum_t plane_normal[3], const CCTNum_t p[3]);
extern GeometryPolygon_t* mathPolygonCookingDirect(const CCTNum_t(*v)[3], const unsigned int* tri_indices, unsigned int tri_indices_cnt, GeometryPolygon_t* polygon);

static int Mesh_Cooking_Edge_InternalProc(const CCTNum_t(*v)[3], GeometryMesh_t* mesh) {
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
					if (e == ret_edge_indices[k + 1] || mathVec3Equal(v[e], v[ret_edge_indices[k + 1]])) {
						break;
					}
				}
				else if (s == ret_edge_indices[k + 1] || mathVec3Equal(v[s], v[ret_edge_indices[k + 1]])) {
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
	mesh->edge_stride = 2;
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

#ifdef __cplusplus
extern "C" {
#endif

int mathMeshCookingRawPolygons(const CCTNum_t(*v)[3], const unsigned int* tri_indices, unsigned int tri_indices_cnt, GeometryMesh_t* mesh) {
	unsigned int i, tri_cnt, tmp_polygons_cnt = 0;
	char* tri_merge_bits = NULL;
	GeometryPolygon_t* tmp_polygons = NULL;

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
	mesh->polygons = tmp_polygons;
	mesh->polygons_cnt = tmp_polygons_cnt;
	return 1;
err:
	if (tmp_polygons) {
		for (i = 0; i < tmp_polygons_cnt; ++i) {
			free((void*)tmp_polygons[i].tri_indices);
		}
		free(tmp_polygons);
	}
	free(tri_merge_bits);
	return 0;
}

GeometryMesh_t* mathMeshCookingDirect(const CCTNum_t(*v)[3], unsigned int v_cnt, const unsigned int* tri_indices, unsigned int tri_indices_cnt, GeometryMesh_t* mesh) {
	unsigned int i;
	CCTNum_t min_v[3], max_v[3];

	if (v_cnt < 3 || tri_indices_cnt < 3) {
		return NULL;
	}
	if (!mathMeshCookingRawPolygons((const CCTNum_t(*)[3])v, tri_indices, tri_indices_cnt, mesh)) {
		return NULL;
	}
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		GeometryPolygon_t* polygon = mesh->polygons + i;
		if (!mathPolygonCookingDirect((const CCTNum_t(*)[3])polygon->v, polygon->tri_indices, polygon->tri_indices_cnt, polygon)) {
			goto err_1;
		}
	}
	if (!Mesh_Cooking_Edge_InternalProc((const CCTNum_t(*)[3])v, mesh)) {
		goto err_1;
	}
	mathVerticesFindMinMaxXYZ(v, v_cnt, min_v, max_v);
	mathAABBFromTwoVertice(min_v, max_v, mesh->bound_box.o, mesh->bound_box.half);
	mathVec3Set(mesh->o, CCTNums_3(0.0, 0.0, 0.0));
	mesh->v = (CCTNum_t(*)[3])v;
	mesh->v_indices = tri_indices;
	mesh->v_indices_cnt = v_cnt;
	mesh->is_convex = mathMeshIsConvex(mesh, CCT_EPSILON);
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		GeometryPolygon_t* polygon = mesh->polygons + i;
		mathVec3Set(polygon->o, CCTNums_3(0.0, 0.0, 0.0));
		if (mesh->is_convex) {
			polygon->is_convex = 1;
		}
		else {
			polygon->is_convex = mathPolygonIsConvex(polygon, CCT_EPSILON);
		}
	}
	if (mesh->is_convex) {
		mathConvexMeshMakeFacesOut(mesh);
	}
	return mesh;
err_1:
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		mesh->polygons[i].v = NULL;
		mathPolygonFreeData(mesh->polygons + i);
	}
	free(mesh->polygons);
	return NULL;
}

GeometryMesh_t* mathMeshCooking(const CCTNum_t(*v)[3], unsigned int v_cnt, const unsigned int* tri_indices, unsigned int tri_indices_cnt, GeometryMesh_t* mesh) {
	CCTNum_t(*dup_v)[3] = NULL;
	CCTNum_t min_v[3], max_v[3];
	unsigned int i, dup_v_cnt;
	unsigned int* dup_v_indices = NULL;
	unsigned int* dup_tri_indices = NULL;

	if (v_cnt < 3 || tri_indices_cnt < 3) {
		goto err_0;
	}
	dup_v = (CCTNum_t(*)[3])malloc(sizeof(dup_v[0]) * v_cnt);
	if (!dup_v) {
		goto err_0;
	}
	dup_v_indices = (unsigned int*)malloc(sizeof(dup_v_indices[0]) * v_cnt);
	if (!dup_v_indices) {
		goto err_0;
	}
	dup_tri_indices = (unsigned int*)malloc(sizeof(tri_indices[0]) * tri_indices_cnt);
	if (!dup_tri_indices) {
		goto err_0;
	}
	dup_v_cnt = mathVerticesMerge(v, v_cnt, tri_indices, tri_indices_cnt, dup_v, dup_tri_indices);
	if (dup_v_cnt < 3) {
		goto err_0;
	}
	if (!mathMeshCookingRawPolygons((const CCTNum_t(*)[3])dup_v, dup_tri_indices, tri_indices_cnt, mesh)) {
		goto err_0;
	}
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		GeometryPolygon_t* polygon = mesh->polygons + i;
		if (!mathPolygonCookingDirect((const CCTNum_t(*)[3])polygon->v, polygon->tri_indices, polygon->tri_indices_cnt, polygon)) {
			goto err_1;
		}
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
	mesh->v = dup_v;
	mesh->v_indices = dup_v_indices;
	mesh->v_indices_cnt = dup_v_cnt;
	mesh->is_convex = mathMeshIsConvex(mesh, CCT_EPSILON);
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		GeometryPolygon_t* polygon = mesh->polygons + i;
		mathVec3Set(polygon->o, CCTNums_3(0.0, 0.0, 0.0));
		if (mesh->is_convex) {
			polygon->is_convex = 1;
		}
		else {
			polygon->is_convex = mathPolygonIsConvex(polygon, CCT_EPSILON);
		}
	}
	if (mesh->is_convex) {
		mathConvexMeshMakeFacesOut(mesh);
	}
	return mesh;
err_1:
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		mesh->polygons[i].v = NULL;
		mathPolygonFreeData(mesh->polygons + i);
	}
	free(mesh->polygons);
err_0:
	free(dup_v);
	free(dup_v_indices);
	free(dup_tri_indices);
	return NULL;
}

#ifdef __cplusplus
}
#endif