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

extern void free_data_mesh_vertex_adjacent_info(GeometryMeshVertexAdjacentInfo_t* info);

static unsigned int Merge_Face_Edge(unsigned int* edge_v_indices_flat, unsigned int edge_v_indices_cnt, const GeometryPolygon_t* polygon) {
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

static unsigned int* Polygon_Save_MeshEdgeIds(const GeometryPolygon_t* polygon, const unsigned int* mesh_edge_v_indices, unsigned int mesh_edge_v_indices_cnt) {
	unsigned int i, polygon_edge_v_indices_cnt;
	unsigned int* mesh_edge_ids = (unsigned int*)malloc(sizeof(mesh_edge_ids[0]) * polygon->edge_cnt);
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
			free(mesh_edge_ids);
			return NULL;
		}
		mesh_edge_ids[i >> 1] = edge_id;
	}
	return mesh_edge_ids;
}

static unsigned int* Polygon_Save_MeshVertexIds(const GeometryPolygon_t* polygon, const unsigned int* mesh_v_indices, unsigned int mesh_v_indices_cnt) {
	unsigned int i;
	unsigned int* mesh_v_ids = (unsigned int*)malloc(sizeof(mesh_v_ids[0]) * polygon->v_indices_cnt);
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
		free(mesh_v_ids);
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

static int Cooking_MeshVertexAdjacentInfo(const GeometryMesh_t* mesh, unsigned int v_id, GeometryMeshVertexAdjacentInfo_t* info) {
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
		tmp_p = (unsigned int*)realloc(face_ids, info->face_cnt * sizeof(face_ids[0]));
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
		tmp_p = (unsigned int*)realloc(v_ids, info->v_cnt * sizeof(v_ids[0]));
		if (!tmp_p) {
			goto err;
		}
		v_ids = tmp_p;
		v_ids[info->v_cnt - 1] = adj_v_id;

		++info->edge_cnt;
		tmp_p = (unsigned int*)realloc(edge_ids, info->edge_cnt * sizeof(edge_ids[0]));
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
	free(v_ids);
	free(edge_ids);
	free(face_ids);
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

#ifdef __cplusplus
extern "C" {
#endif

GeometryMesh_t* mathCookingMesh(const CCTNum_t(*v)[3], const unsigned int* tri_v_indices, unsigned int tri_v_indices_cnt, GeometryMesh_t* mesh) {
	CCTNum_t(*dup_v)[3] = NULL;
	CCTNum_t v1[3], v2[3];
	unsigned int* dup_tri_v_indices = NULL;
	unsigned int dup_v_cnt = 0, i;
	GeometryPolygon_t* tmp_polygons = NULL;
	unsigned int tmp_polygons_cnt = 0;
	unsigned int total_edge_v_indices_cnt;
	unsigned int* edge_v_indices_flat = NULL, *edge_v_ids_flat = NULL;
	unsigned int* v_indices = NULL;
	unsigned int v_indices_cnt;
	GeometryMeshVertexAdjacentInfo_t* v_adjacent_infos = NULL;
	unsigned int(*edge_adjacent_face_ids)[2] = NULL;
	/* check */
	if (tri_v_indices_cnt < 3) {
		return NULL;
	}
	/* merge distinct vertices, rewrite indices */
	if (!mathCookingStage1(v, tri_v_indices, tri_v_indices_cnt, &dup_v, &dup_v_cnt, &dup_tri_v_indices)) {
		goto err_0;
	}
	/* split all face */
	if (!mathCookingStage2((const CCTNum_t(*)[3])dup_v, dup_tri_v_indices, tri_v_indices_cnt, &tmp_polygons, &tmp_polygons_cnt)) {
		goto err_0;
	}
	free(dup_tri_v_indices);
	dup_tri_v_indices = NULL;
	/* cooking every face */
	total_edge_v_indices_cnt = 0;
	for (i = 0; i < tmp_polygons_cnt; ++i) {
		unsigned int* edge_v_indices_flat = NULL, *v_indices = NULL, *edge_v_ids_flat = NULL;
		GeometryPolygonVertexAdjacentInfo_t* v_adjacent_infos = NULL;
		unsigned int edge_v_indices_cnt, v_indices_cnt, j;
		GeometryPolygon_t* pg = tmp_polygons + i;
		/* cooking edge */
		if (!mathCookingStage3((const CCTNum_t(*)[3])pg->v, pg->tri_v_indices_flat, pg->tri_v_indices_cnt, pg->normal, &edge_v_indices_flat, &edge_v_indices_cnt)) {
			goto err_1;
		}
		pg->edge_cnt = edge_v_indices_cnt / 2;
		pg->edge_v_indices_flat = edge_v_indices_flat;
		/* cooking vertex indice */
		if (!mathCookingStage4(edge_v_indices_flat, edge_v_indices_cnt, &v_indices, &v_indices_cnt, &edge_v_ids_flat)) {
			goto err_1;
		}
		total_edge_v_indices_cnt += edge_v_indices_cnt;
		pg->v_indices = v_indices;
		pg->v_indices_cnt = v_indices_cnt;
		pg->edge_v_ids_flat = edge_v_ids_flat;
		/* cooking vertex adjacent infos */
		v_adjacent_infos = (GeometryPolygonVertexAdjacentInfo_t*)malloc(sizeof(v_adjacent_infos[0]) * v_indices_cnt);
		if (!v_adjacent_infos) {
			goto err_1;
		}
		pg->v_adjacent_infos = v_adjacent_infos;
		for (j = 0; j < v_indices_cnt; ++j) {
			if (!mathPolygonVertexAdjacentInfo(edge_v_ids_flat, edge_v_indices_cnt, j, v_adjacent_infos + j)) {
				goto err_1;
			}
		}
		/* fill polygon other data */
		mathVertexIndicesAverageXYZ((const CCTNum_t(*)[3])pg->v, v_indices, v_indices_cnt, pg->center);
	}
	/* merge face edge */
	edge_v_indices_flat = (unsigned int*)malloc(sizeof(edge_v_indices_flat[0]) * total_edge_v_indices_cnt);
	if (!edge_v_indices_flat) {
		goto err_1;
	}
	total_edge_v_indices_cnt = 0;
	for (i = 0; i < tmp_polygons_cnt; ++i) {
		total_edge_v_indices_cnt = Merge_Face_Edge(edge_v_indices_flat, total_edge_v_indices_cnt, tmp_polygons + i);
	}
	/* cooking vertex indice */
	if (!mathCookingStage4(edge_v_indices_flat, total_edge_v_indices_cnt, &v_indices, &v_indices_cnt, &edge_v_ids_flat)) {
		goto err_1;
	}
	/* alloc adjacent infos buffer */
	v_adjacent_infos = (GeometryMeshVertexAdjacentInfo_t*)malloc(sizeof(v_adjacent_infos[0]) * v_indices_cnt);
	if (!v_adjacent_infos) {
		goto err_1;
	}
	edge_adjacent_face_ids = (unsigned int(*)[2])malloc(sizeof(edge_adjacent_face_ids[0]) * total_edge_v_indices_cnt / 2);
	if (!edge_adjacent_face_ids) {
		goto err_1;
	}
	/* cooking face map relationship data */
	for (i = 0; i < tmp_polygons_cnt; ++i) {
		GeometryPolygon_t* pg = tmp_polygons + i;
		pg->mesh_v_ids = Polygon_Save_MeshVertexIds(pg, v_indices, v_indices_cnt);
		if (!pg->mesh_v_ids) {
			goto err_1;
		}
		pg->mesh_edge_ids = Polygon_Save_MeshEdgeIds(pg, edge_v_indices_flat, total_edge_v_indices_cnt);
		if (!pg->mesh_edge_ids) {
			goto err_1;
		}
	}
	/* save basic data result */
	mathVerticesFindMinMaxXYZ((const CCTNum_t(*)[3])dup_v, dup_v_cnt, v1, v2);
	mathAABBFromTwoVertice(v1, v2, mesh->bound_box.o, mesh->bound_box.half);
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
		if (Cooking_MeshVertexAdjacentInfo(mesh, i, v_adjacent_infos + i)) {
			continue;
		}
		while (i--) {
			free_data_mesh_vertex_adjacent_info(v_adjacent_infos + i);
		}
		goto err_1;
	}
	mesh->v_adjacent_infos = v_adjacent_infos;
	/* cooking edge adjacent infos */
	for (i = 0; i < mesh->edge_cnt; ++i) {
		if (!Cooking_MeshEdgeAdjacentFace(mesh, i, edge_adjacent_face_ids[i])) {
			goto err_1;
		}
	}
	mesh->edge_adjacent_face_ids = (const unsigned int(*)[2])edge_adjacent_face_ids;
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
			polygon->is_convex = mathPolygonIsConvex(polygon);
		}
	}
	mesh->is_closed = mathMeshIsClosed(mesh);
	/* finish */
	return mesh;
err_1:
	for (i = 0; i < tmp_polygons_cnt; ++i) {
		tmp_polygons[i].v = NULL;
		mathPolygonClear(tmp_polygons + i);
	}
	free(tmp_polygons);
err_0:
	free(dup_v);
	free(v_indices);
	free(edge_v_ids_flat);
	free(edge_v_indices_flat);
	free(dup_tri_v_indices);
	free(v_adjacent_infos);
	free(edge_adjacent_face_ids);
	return NULL;
}

#ifdef __cplusplus
}
#endif
