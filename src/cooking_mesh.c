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
#include <stdlib.h>

static unsigned int Merge_Face_Edge(unsigned int* edge_indices, unsigned int edge_indices_cnt, const GeometryPolygon_t* polygon) {
	unsigned int i;
	for (i = 0; i < polygon->edge_indices_cnt; i += 2) {
		unsigned int j;
		for (j = 0; j < edge_indices_cnt; j += 2) {
			if (edge_indices[j] == polygon->edge_indices[i] && edge_indices[j + 1] == polygon->edge_indices[i + 1]) {
				break;
			}
			if (edge_indices[j] == polygon->edge_indices[i + 1] && edge_indices[j + 1] == polygon->edge_indices[i]) {
				break;
			}
		}
		if (j < edge_indices_cnt) {
			continue;
		}
		edge_indices[edge_indices_cnt++] = polygon->edge_indices[i];
		edge_indices[edge_indices_cnt++] = polygon->edge_indices[i + 1];
	}
	return edge_indices_cnt;
}

static unsigned int* Polygon_Save_MeshEdgeIndex(const GeometryPolygon_t* polygon, const unsigned int* mesh_edge_indices, unsigned int mesh_edge_indices_cnt) {
	unsigned int i;
	unsigned int* mesh_edge_index = (unsigned int*)malloc(sizeof(mesh_edge_index[0]) * polygon->edge_indices_cnt / 2);
	if (!mesh_edge_index) {
		return NULL;
	}
	for (i = 0; i < polygon->edge_indices_cnt; ++i) {
		unsigned int v_idx[2], edge_idx;
		v_idx[0] = polygon->edge_indices[i++];
		v_idx[1] = polygon->edge_indices[i];
		edge_idx = mathFindEdgeIndexByVertexIndices(mesh_edge_indices, mesh_edge_indices_cnt, v_idx[0], v_idx[1]);
		if (-1 == edge_idx) {
			free(mesh_edge_index);
			return NULL;
		}
		mesh_edge_index[i / 2] = edge_idx;
	}
	return mesh_edge_index;
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

#ifdef __cplusplus
extern "C" {
#endif

GeometryMesh_t* mathCookingMesh(const CCTNum_t(*v)[3], const unsigned int* tri_indices, unsigned int tri_indices_cnt, GeometryMesh_t* mesh) {
	CCTNum_t(*dup_v)[3] = NULL;
	CCTNum_t v1[3], v2[3];
	unsigned int* dup_tri_indices = NULL;
	unsigned int dup_v_cnt = 0, i;
	GeometryPolygon_t* tmp_polygons = NULL;
	unsigned int tmp_polygons_cnt = 0;
	unsigned int total_edge_indices_cnt;
	unsigned int* edge_indices = NULL;
	unsigned int* v_indices = NULL;
	unsigned int v_indices_cnt;
	/* check */
	if (tri_indices_cnt < 3) {
		return NULL;
	}
	/* merge distinct vertices, rewrite indices */
	if (!mathCookingStage1(v, tri_indices, tri_indices_cnt, &dup_v, &dup_v_cnt, &dup_tri_indices)) {
		goto err_0;
	}
	/* split all face */
	if (!mathCookingStage2((const CCTNum_t(*)[3])dup_v, dup_tri_indices, tri_indices_cnt, &tmp_polygons, &tmp_polygons_cnt)) {
		goto err_0;
	}
	free(dup_tri_indices);
	dup_tri_indices = NULL;
	/* cooking every face */
	total_edge_indices_cnt = 0;
	for (i = 0; i < tmp_polygons_cnt; ++i) {
		unsigned int* edge_indices = NULL, *v_indices = NULL;
		unsigned int edge_indices_cnt, v_indices_cnt;
		GeometryPolygon_t* pg = tmp_polygons + i;
		/* cooking edge */
		if (!mathCookingStage3((const CCTNum_t(*)[3])pg->v, pg->tri_indices, pg->tri_indices_cnt, pg->normal, &edge_indices, &edge_indices_cnt)) {
			goto err_1;
		}
		/* cooking vertex indice */
		if (!mathCookingStage4(edge_indices, edge_indices_cnt, &v_indices, &v_indices_cnt)) {
			goto err_1;
		}
		total_edge_indices_cnt += edge_indices_cnt;
		/* fill polygon other data */
		mathVertexIndicesAverageXYZ((const CCTNum_t(*)[3])pg->v, v_indices, v_indices_cnt, pg->center);
		pg->v_indices = v_indices;
		pg->v_indices_cnt = v_indices_cnt;
		pg->edge_indices = edge_indices;
		pg->edge_indices_cnt = edge_indices_cnt;
		pg->mesh_edge_index = NULL;
	}
	/* merge face edge */
	edge_indices = (unsigned int*)malloc(sizeof(edge_indices[0]) * total_edge_indices_cnt);
	if (!edge_indices) {
		goto err_1;
	}
	total_edge_indices_cnt = 0;
	for (i = 0; i < tmp_polygons_cnt; ++i) {
		total_edge_indices_cnt = Merge_Face_Edge(edge_indices, total_edge_indices_cnt, tmp_polygons + i);
	}
	for (i = 0; i < tmp_polygons_cnt; ++i) {
		GeometryPolygon_t* pg = tmp_polygons + i;
		pg->mesh_edge_index = Polygon_Save_MeshEdgeIndex(pg, edge_indices, total_edge_indices_cnt);
		if (!pg->mesh_edge_index) {
			goto err_1;
		}
	}
	/* cooking vertex indice */
	if (!mathCookingStage4(edge_indices, total_edge_indices_cnt, &v_indices, &v_indices_cnt)) {
		goto err_1;
	}
	/* save result */
	mathVerticesFindMinMaxXYZ((const CCTNum_t(*)[3])dup_v, dup_v_cnt, v1, v2);
	mathAABBFromTwoVertice(v1, v2, mesh->bound_box.o, mesh->bound_box.half);
	mathVec3Set(mesh->o, CCTNums_3(0.0, 0.0, 0.0));
	mesh->v = dup_v;
	mesh->v_indices = v_indices;
	mesh->v_indices_cnt = v_indices_cnt;
	mesh->edge_indices = edge_indices;
	mesh->edge_indices_cnt = total_edge_indices_cnt;
	mesh->polygons = tmp_polygons;
	mesh->polygons_cnt = tmp_polygons_cnt;
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
			polygon->is_convex = mathPolygonIsConvex(polygon);
		}
	}
	mesh->is_closed = mathMeshIsClosed(mesh);
	return mesh;
err_1:
	for (i = 0; i < tmp_polygons_cnt; ++i) {
		tmp_polygons[i].v = NULL;
		mathPolygonFreeData(tmp_polygons + i);
	}
	free(tmp_polygons);
err_0:
	free(dup_v);
	free(v_indices);
	free(edge_indices);
	free(dup_tri_indices);
	return NULL;
}

#ifdef __cplusplus
}
#endif
