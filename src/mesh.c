//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/polygon.h"
#include "../inc/mesh.h"
#include <stdlib.h>

static void free_all_faces(GeometryMesh_t* mesh) {
	unsigned int i;
	if (!mesh->polygons) {
		return;
	}
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		mesh->polygons[i].v = NULL;
		mathPolygonClear(mesh->polygons + i);
	}
	mesh->polygons_cnt = 0;
	free(mesh->polygons);
	mesh->polygons = NULL;
}

void free_data_mesh_vertex_adjacent_info(GeometryMeshVertexAdjacentInfo_t* info) {
	free((void*)info->v_ids);
	free((void*)info->edge_ids);
	free((void*)info->face_ids);
}

static void free_all_adjacent_infos(GeometryMesh_t* mesh) {
	unsigned int i;
	if (mesh->v_adjacent_infos) {
		for (i = 0; i < mesh->v_indices_cnt; ++i) {
			free_data_mesh_vertex_adjacent_info((GeometryMeshVertexAdjacentInfo_t*)(mesh->v_adjacent_infos + i));
		}
		free((void*)mesh->v_adjacent_infos);
		mesh->v_adjacent_infos = NULL;
	}
	if (mesh->edge_adjacent_face_ids) {
		free((void*)mesh->edge_adjacent_face_ids);
		mesh->edge_adjacent_face_ids = NULL;
	}
}

static int face_deep_copy_without_vertex(GeometryPolygon_t* dst, GeometryPolygon_t* src) {
	unsigned int j;
	unsigned int* dup_v_indices = NULL;
	unsigned int* dup_tri_indices = NULL;
	unsigned int* dup_edge_v_indices = NULL, *dup_edge_v_ids = NULL;
	unsigned int* dup_mesh_edge_ids = NULL, *dup_mesh_v_ids = NULL;
	GeometryPolygonVertexAdjacentInfo_t* dup_v_adjacent_infos = NULL;
	unsigned int src_edge_v_indices_cnt;

	dup_v_indices = (unsigned int*)malloc(sizeof(dup_v_indices[0]) * src->v_indices_cnt);
	if (!dup_v_indices) {
		goto err;
	}
	dup_tri_indices = (unsigned int*)malloc(sizeof(dup_tri_indices[0]) * src->tri_v_indices_cnt);
	if (!dup_tri_indices) {
		goto err;
	}
	src_edge_v_indices_cnt = src->edge_cnt + src->edge_cnt;
	dup_edge_v_ids = (unsigned int*)malloc(sizeof(dup_edge_v_ids[0]) * src_edge_v_indices_cnt);
	if (!dup_edge_v_ids) {
		goto err;
	}
	dup_edge_v_indices = (unsigned int*)malloc(sizeof(dup_edge_v_indices[0]) * src_edge_v_indices_cnt);
	if (!dup_edge_v_indices) {
		goto err;
	}
	dup_v_adjacent_infos = (GeometryPolygonVertexAdjacentInfo_t*)malloc(sizeof(dup_v_adjacent_infos[0]) * src->v_indices_cnt);
	if (!dup_v_adjacent_infos) {
		goto err;
	}
	if (src->mesh_v_ids) {
		dup_mesh_v_ids = (unsigned int*)malloc(sizeof(dup_mesh_v_ids[0]) * src->v_indices_cnt);
		if (!dup_mesh_v_ids) {
			goto err;
		}
		for (j = 0; j < src->v_indices_cnt; ++j) {
			dup_mesh_v_ids[j] = src->mesh_v_ids[j];
		}
	}
	if (src->mesh_edge_ids) {
		dup_mesh_edge_ids = (unsigned int*)malloc(sizeof(dup_mesh_edge_ids[0]) * src->edge_cnt);
		if (!dup_mesh_edge_ids) {
			goto err;
		}
		for (j = 0; j < src->edge_cnt; ++j) {
			dup_mesh_edge_ids[j] = src->mesh_edge_ids[j];
		}
	}
	for (j = 0; j < src->v_indices_cnt; ++j) {
		dup_v_indices[j] = src->v_indices[j];
		dup_v_adjacent_infos[j] = src->v_adjacent_infos[j];
	}
	for (j = 0; j < src->tri_v_indices_cnt; ++j) {
		dup_tri_indices[j] = src->tri_v_indices_flat[j];
	}
	for (j = 0; j < src_edge_v_indices_cnt; ++j) {
		dup_edge_v_ids[j] = src->edge_v_ids_flat[j];
		dup_edge_v_indices[j] = src->edge_v_indices_flat[j];
	}
	mathVec3Copy(dst->center, src->center);
	mathVec3Copy(dst->normal, src->normal);
	dst->is_convex = src->is_convex;
	dst->v = NULL;
	dst->v_indices_cnt = src->v_indices_cnt;
	dst->tri_v_indices_cnt = src->tri_v_indices_cnt;
	dst->edge_cnt = src->edge_cnt;
	dst->v_indices = dup_v_indices;
	dst->tri_v_indices_flat = dup_tri_indices;
	dst->edge_v_ids_flat = dup_edge_v_ids;
	dst->edge_v_indices_flat = dup_edge_v_indices;
	dst->mesh_v_ids = dup_mesh_v_ids;
	dst->mesh_edge_ids = dup_mesh_edge_ids;
	dst->v_adjacent_infos = dup_v_adjacent_infos;
	return 1;
err:
	free(dup_v_indices);
	free(dup_tri_indices);
	free(dup_edge_v_ids);
	free(dup_edge_v_indices);
	free(dup_mesh_v_ids);
	free(dup_mesh_edge_ids);
	free(dup_v_adjacent_infos);
	return 0;
}

static int deep_copy_vertex_adjacent_info(GeometryMeshVertexAdjacentInfo_t* dst, const GeometryMeshVertexAdjacentInfo_t* src) {
	unsigned int i;
	unsigned int *v_ids = NULL, *edge_ids = NULL, *face_ids = NULL;

	v_ids = (unsigned int*)malloc(sizeof(dst->v_ids[0]) * src->v_cnt);
	if (!v_ids) {
		goto err;
	}
	for (i = 0; i < src->v_cnt; ++i) {
		v_ids[i] = src->v_ids[i];
	}

	edge_ids = (unsigned int*)malloc(sizeof(dst->edge_ids[0]) * src->edge_cnt);
	if (!edge_ids) {
		goto err;
	}
	for (i = 0; i < src->edge_cnt; ++i) {
		edge_ids[i] = src->edge_ids[i];
	}

	face_ids = (unsigned int*)malloc(sizeof(dst->face_ids[0]) * src->face_cnt);
	if (!face_ids) {
		goto err;
	}
	for (i = 0; i < src->face_cnt; ++i) {
		face_ids[i] = src->face_ids[i];
	}

	dst->v_cnt = src->v_cnt;
	dst->edge_cnt = src->edge_cnt;
	dst->face_cnt = src->face_cnt;
	dst->v_ids = v_ids;
	dst->edge_ids = edge_ids;
	dst->face_ids = face_ids;
	return 1;
err:
	free(v_ids);
	free(edge_ids);
	free(face_ids);
	return 0;
}

#ifdef __cplusplus
extern "C" {
#endif

GeometryMesh_t* mathMeshDeepCopy(GeometryMesh_t* dst, const GeometryMesh_t* src) {
	unsigned int i, dup_v_cnt = 0;
	CCTNum_t(*dup_v)[3] = NULL;
	unsigned int* dup_v_indices = NULL;
	unsigned int* dup_edge_v_indices = NULL, *dup_edge_v_ids = NULL;
	GeometryPolygon_t* dup_polygons = NULL;
	GeometryMeshVertexAdjacentInfo_t* dup_v_adjacent_infos = NULL;
	unsigned int(*dup_edge_adjacent_face_ids)[2] = NULL;
	unsigned int src_edge_v_indices_cnt;
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
	src_edge_v_indices_cnt = src->edge_cnt + src->edge_cnt;
	dup_edge_v_ids = (unsigned int*)malloc(sizeof(dup_edge_v_ids[0]) * src_edge_v_indices_cnt);
	if (!dup_edge_v_ids) {
		goto err_0;
	}
	dup_edge_v_indices = (unsigned int*)malloc(sizeof(dup_edge_v_indices[0]) * src_edge_v_indices_cnt);
	if (!dup_edge_v_indices) {
		goto err_0;
	}
	dup_polygons = (GeometryPolygon_t*)malloc(sizeof(dup_polygons[0]) * src->polygons_cnt);
	if (!dup_polygons) {
		goto err_0;
	}
	for (i = 0; i < src->polygons_cnt; ++i) {
		if (!face_deep_copy_without_vertex(dup_polygons + i, src->polygons + i)) {
			break;
		}
		dup_polygons[i].v = dup_v;
	}
	if (i < src->polygons_cnt) {
		while (i--) {
			GeometryPolygon_t* dst_polygon = dup_polygons + i;
			free((void*)dst_polygon->v_indices);
			free((void*)dst_polygon->tri_v_indices_flat);
			free((void*)dst_polygon->edge_v_ids_flat);
			free((void*)dst_polygon->edge_v_indices_flat);
			free((void*)dst_polygon->mesh_edge_ids);
			free((void*)dst_polygon->v_adjacent_infos);
		}
		goto err_0;
	}
	dup_v_adjacent_infos = (GeometryMeshVertexAdjacentInfo_t*)malloc(sizeof(dup_v_adjacent_infos[0]) * src->v_indices_cnt);
	if (!dup_v_adjacent_infos) {
		goto err_0;
	}
	for (i = 0; i < src->v_indices_cnt; ++i) {
		unsigned int idx = src->v_indices[i];
		dup_v_indices[i] = idx;
		mathVec3Copy(dup_v[idx], src->v[idx]);

		if (!deep_copy_vertex_adjacent_info(dup_v_adjacent_infos + i, src->v_adjacent_infos + i)) {
			while (i--) {
				free_data_mesh_vertex_adjacent_info(dup_v_adjacent_infos + i);
			}
			goto err_0;
		}
	}
	dup_edge_adjacent_face_ids = (unsigned int(*)[2])malloc(sizeof(dup_edge_adjacent_face_ids[0]) * src->edge_cnt);
	if (!dup_edge_adjacent_face_ids) {
		goto err_0;
	}
	for (i = 0; i < src->edge_cnt; ++i) {
		dup_edge_adjacent_face_ids[i][0] = src->edge_adjacent_face_ids[i][0];
		dup_edge_adjacent_face_ids[i][1] = src->edge_adjacent_face_ids[i][1];
	}
	for (i = 0; i < src_edge_v_indices_cnt; ++i) {
		dup_edge_v_ids[i] = src->edge_v_ids_flat[i];
		dup_edge_v_indices[i] = src->edge_v_indices_flat[i];
	}
	dst->v = dup_v;
	dst->bound_box = src->bound_box;
	dst->polygons_cnt = src->polygons_cnt;
	dst->edge_cnt = src->edge_cnt;
	dst->is_convex = src->is_convex;
	dst->is_closed = src->is_closed;
	dst->v_indices_cnt = src->v_indices_cnt;
	dst->polygons = dup_polygons;
	dst->edge_v_ids_flat = dup_edge_v_ids;
	dst->edge_v_indices_flat = dup_edge_v_indices;
	dst->v_indices = dup_v_indices;
	dst->v_adjacent_infos = dup_v_adjacent_infos;
	dst->edge_adjacent_face_ids = (const unsigned int(*)[2])dup_edge_adjacent_face_ids;
	return dst;
err_0:
	free(dup_v);
	free(dup_v_indices);
	free(dup_edge_v_ids);
	free(dup_edge_v_indices);
	free(dup_polygons);
	free(dup_v_adjacent_infos);
	free(dup_edge_adjacent_face_ids);
	return NULL;
}

void mathMeshClear(GeometryMesh_t* mesh) {
	if (!mesh) {
		return;
	}
	free_all_faces(mesh);
	free_all_adjacent_infos(mesh);
	if (mesh->edge_v_ids_flat) {
		free((void*)mesh->edge_v_ids_flat);
		mesh->edge_v_ids_flat = NULL;
	}
	if (mesh->edge_v_indices_flat) {
		free((void*)mesh->edge_v_indices_flat);
		mesh->edge_v_indices_flat = NULL;
	}
	mesh->edge_cnt = 0;
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
		int flag_sign = 0;
		unsigned int j;
		for (j = 0; j < mesh->v_indices_cnt; ++j) {
			CCTNum_t vj[3], dot;
			mathVec3Sub(vj, mesh->v[mesh->v_indices[j]], polygon->v[polygon->v_indices[0]]);
			dot = mathVec3Dot(polygon->normal, vj);
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

#ifdef __cplusplus
}
#endif
