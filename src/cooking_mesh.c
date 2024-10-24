//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/vertex.h"
#include "../inc/aabb.h"
#include "../inc/plane.h"
#include "../inc/polygon.h"
#include "../inc/mesh.h"
#include "../inc/cooking_polygon.h"
#include "../inc/cooking_mesh.h"
#include <stdlib.h>

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

#ifdef __cplusplus
extern "C" {
#endif

GeometryMesh_t* mathCookingMeshDirect(const CCTNum_t(*v)[3], unsigned int v_cnt, const unsigned int* tri_indices, unsigned int tri_indices_cnt, GeometryMesh_t* mesh) {
	unsigned int i;
	CCTNum_t min_v[3], max_v[3];
	GeometryPolygon_t* polygons = NULL;
	unsigned int polygons_cnt = 0;

	if (v_cnt < 3 || tri_indices_cnt < 3) {
		return NULL;
	}
	if (!mathCookingMorePolygons((const CCTNum_t(*)[3])v, tri_indices, tri_indices_cnt, &polygons, &polygons_cnt)) {
		return NULL;
	}
	for (i = 0; i < polygons_cnt; ++i) {
		GeometryPolygon_t* polygon = polygons + i;
		if (!mathCookingPolygonDirect((const CCTNum_t(*)[3])polygon->v, polygon->tri_indices, polygon->tri_indices_cnt, polygon)) {
			goto err_1;
		}
	}
	mesh->polygons = polygons;
	mesh->polygons_cnt = polygons_cnt;
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
	for (i = 0; i < polygons_cnt; ++i) {
		polygons[i].v = NULL;
		mathPolygonFreeData(polygons + i);
	}
	free(polygons);
	return NULL;
}

GeometryMesh_t* mathCookingMesh(const CCTNum_t(*v)[3], unsigned int v_cnt, const unsigned int* tri_indices, unsigned int tri_indices_cnt, GeometryMesh_t* mesh) {
	CCTNum_t(*dup_v)[3] = NULL;
	CCTNum_t min_v[3], max_v[3];
	unsigned int i, dup_v_cnt;
	unsigned int* dup_v_indices = NULL;
	unsigned int* dup_tri_indices = NULL;
	GeometryPolygon_t* polygons = NULL;
	unsigned int polygons_cnt = 0;

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
	dup_v_cnt = mathVerticesMerge(v, tri_indices, tri_indices_cnt, dup_v, dup_tri_indices);
	if (dup_v_cnt < 3) {
		goto err_0;
	}
	if (!mathCookingMorePolygons((const CCTNum_t(*)[3])dup_v, dup_tri_indices, tri_indices_cnt, &polygons, &polygons_cnt)) {
		goto err_0;
	}
	for (i = 0; i < polygons_cnt; ++i) {
		GeometryPolygon_t* polygon = polygons + i;
		if (!mathCookingPolygonDirect((const CCTNum_t(*)[3])polygon->v, polygon->tri_indices, polygon->tri_indices_cnt, polygon)) {
			goto err_1;
		}
	}
	mesh->polygons = polygons;
	mesh->polygons_cnt = polygons_cnt;
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
	for (i = 0; i < polygons_cnt; ++i) {
		polygons[i].v = NULL;
		mathPolygonFreeData(polygons + i);
	}
	free(polygons);
err_0:
	free(dup_v);
	free(dup_v_indices);
	free(dup_tri_indices);
	return NULL;
}

#ifdef __cplusplus
}
#endif
