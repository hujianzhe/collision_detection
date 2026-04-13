//
// Created by hujianzhe
//

#include "../inc/const_data.h"
#include "../inc/math_vec3.h"
#include "../inc/vertex.h"
#include "../inc/box.h"
#include "../inc/polygon.h"
#include "../inc/mesh.h"

extern const CCTConstVal_t CCTConstVal_;

extern void Polygon_ClearWithoutVertices(GeometryPolygon_t* polygon, const CCTAllocator_t* ac);

int MeshVertexAdjacentInfo_GetFaceIds(GeometryMeshVertexAdjacentInfo_t* info, const GeometryPolygon_t* polygons, unsigned int polygons_cnt, unsigned int v_id, const CCTAllocator_t* ac) {
	unsigned int i, ids_cnt = 0;
	unsigned int* ids = NULL;
	for (i = 0; i < polygons_cnt; ++i) {
		unsigned int j;
		unsigned int* tmp_p;
		const GeometryPolygon_t* polygon = polygons + i;
		for (j = 0; j < polygon->v_indices_cnt; ++j) {
			if (polygon->mesh_v_ids[j] == v_id) {
				break;
			}
		}
		if (j >= polygon->v_indices_cnt) {
			continue;
		}
		++ids_cnt;
		tmp_p = (unsigned int*)ac->fn_realloc(ac, ids, sizeof(ids[0]) * ids_cnt);
		if (!tmp_p) {
			goto err;
		}
		ids = tmp_p;
		ids[ids_cnt - 1] = i;
	}
	info->face_ids = ids;
	info->face_cnt = ids_cnt;
	return 1;
err:
	ac->fn_free(ac, ids);
	return 0;
}

int MeshVertexAdjacentInfo_GetEdgeIds(GeometryMeshVertexAdjacentInfo_t* info, const unsigned int* edge_v_ids_flat, unsigned int edge_cnt, unsigned int v_id, const CCTAllocator_t* ac) {
	unsigned int i, ids_cnt = 0;
	unsigned int* ids = NULL;
	for (i = 0; i < edge_cnt * 2; i += 2) {
		unsigned int* tmp_p;
		if (edge_v_ids_flat[i] != v_id && edge_v_ids_flat[i + 1] != v_id) {
			continue;
		}
		++ids_cnt;
		tmp_p = (unsigned int*)ac->fn_realloc(ac, ids, sizeof(ids[0]) * ids_cnt);
		if (!tmp_p) {
			goto err;
		}
		ids = tmp_p;
		ids[ids_cnt - 1] = i / 2;
	}
	info->edge_ids = ids;
	info->edge_cnt = ids_cnt;
	return 1;
err:
	ac->fn_free(ac, ids);
	return 0;
}

int MeshVertexAdjacentInfo_GetVertexIds(GeometryMeshVertexAdjacentInfo_t* info, const unsigned int* edge_v_ids_flat, unsigned int edge_cnt, unsigned int v_id, const CCTAllocator_t* ac) {
	unsigned int i, ids_cnt = 0;
	unsigned int* ids = NULL;
	for (i = 0; i < edge_cnt * 2; i += 2) {
		unsigned int* tmp_p;
		unsigned int adj_v_id;
		if (edge_v_ids_flat[i] == v_id) {
			adj_v_id = edge_v_ids_flat[i + 1];
		}
		else if (edge_v_ids_flat[i + 1] == v_id) {
			adj_v_id = edge_v_ids_flat[i];
		}
		else {
			continue;
		}
		++ids_cnt;
		tmp_p = (unsigned int*)ac->fn_realloc(ac, ids, sizeof(ids[0]) * ids_cnt);
		if (!tmp_p) {
			goto err;
		}
		ids = tmp_p;
		ids[ids_cnt - 1] = adj_v_id;
	}
	info->v_ids = ids;
	info->v_cnt = ids_cnt;
	return 1;
err:
	ac->fn_free(ac, ids);
	return 0;
}

void MeshVertexAdjacentInfo_free(GeometryMeshVertexAdjacentInfo_t* info, const CCTAllocator_t* ac) {
	ac->fn_free(ac, (void*)info->v_ids);
	ac->fn_free(ac, (void*)info->edge_ids);
	ac->fn_free(ac, (void*)info->face_ids);
}

int MeshEdgeAdjacentFace(const GeometryPolygon_t* polygons, unsigned int polygons_cnt, const unsigned int* edge_v_indices_flat, unsigned int edge_id, unsigned int adjacent_faces_ids[2]) {
	unsigned int i, cnt = 0;
	const unsigned int v_idx[2] = {
		edge_v_indices_flat[edge_id + edge_id],
		edge_v_indices_flat[edge_id + edge_id + 1]
	};
	for (i = 0; i < polygons_cnt; ++i) {
		unsigned int offset = mathFindFaceIdByVertexIndices(polygons + i, polygons_cnt - i, v_idx, 2);
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

int MeshVertices_IsConvex(const GeometryMesh_t* mesh) {
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

int Mesh_IsClosed(const GeometryMesh_t* mesh) {
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

static void free_all_faces(GeometryMesh_t* mesh) {
	unsigned int i;
	const CCTAllocator_t* ac;
	if (!mesh->polygons) {
		return;
	}
	ac = mesh->allocator_ptr;
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		Polygon_ClearWithoutVertices(mesh->polygons + i, ac);
	}
	mesh->polygons_cnt = 0;
	ac->fn_free(ac, mesh->polygons);
	mesh->polygons = NULL;
}

static void free_all_adjacent_infos(GeometryMesh_t* mesh) {
	unsigned int i;
	const CCTAllocator_t* ac = mesh->allocator_ptr;
	if (mesh->v_adjacent_infos) {
		for (i = 0; i < mesh->v_indices_cnt; ++i) {
			const GeometryMeshVertexAdjacentInfo_t* info = (mesh->v_adjacent_infos + i);
			MeshVertexAdjacentInfo_free((GeometryMeshVertexAdjacentInfo_t*)info, ac);
		}
		ac->fn_free(ac, (void*)mesh->v_adjacent_infos);
		mesh->v_adjacent_infos = NULL;
	}
	if (mesh->edge_adjacent_face_ids_flat) {
		ac->fn_free(ac, (void*)mesh->edge_adjacent_face_ids_flat);
		mesh->edge_adjacent_face_ids_flat = NULL;
	}
}

static int face_deep_copy_without_vertex(GeometryPolygon_t* dst, GeometryPolygon_t* src, const CCTAllocator_t* ac) {
	unsigned int j;
	unsigned int* dup_v_indices = NULL;
	unsigned int* dup_tri_indices = NULL, *dup_concave_tri_edge_ids = NULL, *dup_concave_tri_v_ids = NULL;
	unsigned int* dup_edge_v_indices = NULL, *dup_edge_v_ids = NULL;
	unsigned int* dup_mesh_edge_ids = NULL, *dup_mesh_v_ids = NULL;
	GeometryPolygonVertexAdjacentInfo_t* dup_v_adjacent_infos = NULL;
	unsigned int src_edge_v_indices_cnt;

	dup_v_indices = (unsigned int*)ac->fn_malloc(ac, sizeof(dup_v_indices[0]) * src->v_indices_cnt);
	if (!dup_v_indices) {
		goto err;
	}
	dup_tri_indices = (unsigned int*)ac->fn_malloc(ac, sizeof(dup_tri_indices[0]) * src->tri_cnt * 3);
	if (!dup_tri_indices) {
		goto err;
	}
	if (src->concave_tri_edge_ids_flat) {
		dup_concave_tri_edge_ids = (unsigned int*)ac->fn_malloc(ac, sizeof(dup_concave_tri_edge_ids[0]) * src->tri_cnt * 3);
		if (!dup_concave_tri_edge_ids) {
			goto err;
		}
	}
	if (src->concave_tri_v_ids_flat) {
		dup_concave_tri_v_ids = (unsigned int*)ac->fn_malloc(ac, sizeof(dup_concave_tri_v_ids[0]) * src->tri_cnt * 3);
		if (!dup_concave_tri_v_ids) {
			goto err;
		}
	}
	src_edge_v_indices_cnt = src->edge_cnt + src->edge_cnt;
	dup_edge_v_ids = (unsigned int*)ac->fn_malloc(ac, sizeof(dup_edge_v_ids[0]) * src_edge_v_indices_cnt);
	if (!dup_edge_v_ids) {
		goto err;
	}
	dup_edge_v_indices = (unsigned int*)ac->fn_malloc(ac, sizeof(dup_edge_v_indices[0]) * src_edge_v_indices_cnt);
	if (!dup_edge_v_indices) {
		goto err;
	}
	dup_v_adjacent_infos = (GeometryPolygonVertexAdjacentInfo_t*)ac->fn_malloc(ac, sizeof(dup_v_adjacent_infos[0]) * src->v_indices_cnt);
	if (!dup_v_adjacent_infos) {
		goto err;
	}
	if (src->mesh_v_ids) {
		dup_mesh_v_ids = (unsigned int*)ac->fn_malloc(ac, sizeof(dup_mesh_v_ids[0]) * src->v_indices_cnt);
		if (!dup_mesh_v_ids) {
			goto err;
		}
		for (j = 0; j < src->v_indices_cnt; ++j) {
			dup_mesh_v_ids[j] = src->mesh_v_ids[j];
		}
	}
	if (src->mesh_edge_ids) {
		dup_mesh_edge_ids = (unsigned int*)ac->fn_malloc(ac, sizeof(dup_mesh_edge_ids[0]) * src->edge_cnt);
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
	for (j = 0; j < src->tri_cnt * 3; ++j) {
		dup_tri_indices[j] = src->tri_v_indices_flat[j];
		if (src->concave_tri_edge_ids_flat) {
			dup_concave_tri_edge_ids[j] = src->concave_tri_edge_ids_flat[j];
		}
		if (src->concave_tri_v_ids_flat) {
			dup_concave_tri_v_ids[j] = src->concave_tri_v_ids_flat[j];
		}
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
	dst->tri_cnt = src->tri_cnt;
	dst->edge_cnt = src->edge_cnt;
	dst->v_indices = dup_v_indices;
	dst->tri_v_indices_flat = dup_tri_indices;
	dst->concave_tri_edge_ids_flat = dup_concave_tri_edge_ids;
	dst->concave_tri_v_ids_flat = dup_concave_tri_v_ids;
	dst->edge_v_ids_flat = dup_edge_v_ids;
	dst->edge_v_indices_flat = dup_edge_v_indices;
	dst->mesh_v_ids = dup_mesh_v_ids;
	dst->mesh_edge_ids = dup_mesh_edge_ids;
	dst->v_adjacent_infos = dup_v_adjacent_infos;
	return 1;
err:
	ac->fn_free(ac, dup_v_indices);
	ac->fn_free(ac, dup_tri_indices);
	ac->fn_free(ac, dup_concave_tri_edge_ids);
	ac->fn_free(ac, dup_concave_tri_v_ids);
	ac->fn_free(ac, dup_edge_v_ids);
	ac->fn_free(ac, dup_edge_v_indices);
	ac->fn_free(ac, dup_mesh_v_ids);
	ac->fn_free(ac, dup_mesh_edge_ids);
	ac->fn_free(ac, dup_v_adjacent_infos);
	return 0;
}

static int deep_copy_vertex_adjacent_info(GeometryMeshVertexAdjacentInfo_t* dst, const GeometryMeshVertexAdjacentInfo_t* src, const CCTAllocator_t* ac) {
	unsigned int i;
	unsigned int *v_ids = NULL, *edge_ids = NULL, *face_ids = NULL;

	v_ids = (unsigned int*)ac->fn_malloc(ac, sizeof(dst->v_ids[0]) * src->v_cnt);
	if (!v_ids) {
		goto err;
	}
	for (i = 0; i < src->v_cnt; ++i) {
		v_ids[i] = src->v_ids[i];
	}

	edge_ids = (unsigned int*)ac->fn_malloc(ac, sizeof(dst->edge_ids[0]) * src->edge_cnt);
	if (!edge_ids) {
		goto err;
	}
	for (i = 0; i < src->edge_cnt; ++i) {
		edge_ids[i] = src->edge_ids[i];
	}

	face_ids = (unsigned int*)ac->fn_malloc(ac, sizeof(dst->face_ids[0]) * src->face_cnt);
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
	ac->fn_free(ac, v_ids);
	ac->fn_free(ac, edge_ids);
	ac->fn_free(ac, face_ids);
	return 0;
}

#ifdef __cplusplus
extern "C" {
#endif

GeometryMesh_t* mathMeshClone(GeometryMesh_t* dst, const GeometryMesh_t* src, const CCTAllocator_t* ac) {
	unsigned int i, dup_v_cnt = 0;
	CCTNum_t(*dup_v)[3] = NULL;
	unsigned int* dup_v_indices = NULL;
	unsigned int* dup_edge_v_indices = NULL, *dup_edge_v_ids = NULL;
	GeometryPolygon_t* dup_polygons = NULL;
	GeometryMeshVertexAdjacentInfo_t* dup_v_adjacent_infos = NULL;
	unsigned int* dup_edge_adjacent_face_ids = NULL;
	unsigned int src_edge_v_indices_cnt;
	if (!ac) {
		ac = CCTAllocator_stdc(NULL);
	}
	/* find max vertex index, dup_v_cnt */
	for (i = 0; i < src->v_indices_cnt; ++i) {
		if (src->v_indices[i] >= dup_v_cnt) {
			dup_v_cnt = src->v_indices[i] + 1;
		}
	}
	/* deep copy */
	dup_v = (CCTNum_t(*)[3])ac->fn_malloc(ac, sizeof(dup_v[0]) * dup_v_cnt);
	if (!dup_v) {
		goto err_0;
	}
	dup_v_indices = (unsigned int*)ac->fn_malloc(ac, sizeof(dup_v_indices[0]) * dup_v_cnt);
	if (!dup_v_indices) {
		goto err_0;
	}
	src_edge_v_indices_cnt = src->edge_cnt + src->edge_cnt;
	dup_edge_v_ids = (unsigned int*)ac->fn_malloc(ac, sizeof(dup_edge_v_ids[0]) * src_edge_v_indices_cnt);
	if (!dup_edge_v_ids) {
		goto err_0;
	}
	dup_edge_v_indices = (unsigned int*)ac->fn_malloc(ac, sizeof(dup_edge_v_indices[0]) * src_edge_v_indices_cnt);
	if (!dup_edge_v_indices) {
		goto err_0;
	}
	dup_polygons = (GeometryPolygon_t*)ac->fn_malloc(ac, sizeof(dup_polygons[0]) * src->polygons_cnt);
	if (!dup_polygons) {
		goto err_0;
	}
	for (i = 0; i < src->polygons_cnt; ++i) {
		if (!face_deep_copy_without_vertex(dup_polygons + i, src->polygons + i, ac)) {
			break;
		}
		dup_polygons[i].v = dup_v;
	}
	if (i < src->polygons_cnt) {
		while (i--) {
			GeometryPolygon_t* dst_polygon = dup_polygons + i;
			ac->fn_free(ac, (void*)dst_polygon->v_indices);
			ac->fn_free(ac, (void*)dst_polygon->tri_v_indices_flat);
			ac->fn_free(ac, (void*)dst_polygon->edge_v_ids_flat);
			ac->fn_free(ac, (void*)dst_polygon->edge_v_indices_flat);
			ac->fn_free(ac, (void*)dst_polygon->mesh_edge_ids);
			ac->fn_free(ac, (void*)dst_polygon->v_adjacent_infos);
		}
		goto err_0;
	}
	dup_v_adjacent_infos = (GeometryMeshVertexAdjacentInfo_t*)ac->fn_malloc(ac, sizeof(dup_v_adjacent_infos[0]) * src->v_indices_cnt);
	if (!dup_v_adjacent_infos) {
		goto err_0;
	}
	for (i = 0; i < src->v_indices_cnt; ++i) {
		unsigned int idx = src->v_indices[i];
		dup_v_indices[i] = idx;
		mathVec3Copy(dup_v[idx], src->v[idx]);

		if (!deep_copy_vertex_adjacent_info(dup_v_adjacent_infos + i, src->v_adjacent_infos + i, ac)) {
			while (i--) {
				MeshVertexAdjacentInfo_free(dup_v_adjacent_infos + i, ac);
			}
			goto err_0;
		}
	}
	dup_edge_adjacent_face_ids = (unsigned int*)ac->fn_malloc(ac, sizeof(dup_edge_adjacent_face_ids[0]) * src->edge_cnt * 2);
	if (!dup_edge_adjacent_face_ids) {
		goto err_0;
	}
	for (i = 0; i < src->edge_cnt + src->edge_cnt; ++i) {
		dup_edge_adjacent_face_ids[i] = src->edge_adjacent_face_ids_flat[i];
	}
	for (i = 0; i < src_edge_v_indices_cnt; ++i) {
		dup_edge_v_ids[i] = src->edge_v_ids_flat[i];
		dup_edge_v_indices[i] = src->edge_v_indices_flat[i];
	}
	dst->v = dup_v;
	dst->allocator_ptr = ac;
	dst->bound_box = src->bound_box;
	dst->polygons_cnt = src->polygons_cnt;
	dst->edge_cnt = src->edge_cnt;
	dst->is_convex = src->is_convex;
	dst->is_closed = src->is_closed;
	dst->_is_buffer_view = 0;
	dst->v_indices_cnt = src->v_indices_cnt;
	dst->polygons = dup_polygons;
	dst->edge_v_ids_flat = dup_edge_v_ids;
	dst->edge_v_indices_flat = dup_edge_v_indices;
	dst->v_indices = dup_v_indices;
	dst->v_adjacent_infos = dup_v_adjacent_infos;
	dst->edge_adjacent_face_ids_flat = dup_edge_adjacent_face_ids;
	return dst;
err_0:
	ac->fn_free(ac, dup_v);
	ac->fn_free(ac, dup_v_indices);
	ac->fn_free(ac, dup_edge_v_ids);
	ac->fn_free(ac, dup_edge_v_indices);
	ac->fn_free(ac, dup_polygons);
	ac->fn_free(ac, dup_v_adjacent_infos);
	ac->fn_free(ac, dup_edge_adjacent_face_ids);
	return NULL;
}

GeometryMesh_t* mathMeshDupFaces(GeometryMesh_t* dup_mesh, const GeometryPolygon_t* faces, unsigned int face_cnt, const CCTAllocator_t* ac) {
	unsigned int i, j;
	unsigned all_face_convex;
	CCTNum_t(*dup_v)[3] = NULL;
	unsigned int v_adjacent_infos_init_cnt = 0;
	GeometryMeshVertexAdjacentInfo_t* dup_v_adjacent_infos = NULL;
	unsigned int dup_v_cnt, dup_edge_v_indices_cnt;
	unsigned int *dup_v_indices_flat = NULL;
	unsigned int *dup_edge_v_indices_flat = NULL, *dup_edge_v_ids_flat = NULL;
	unsigned int* dup_edge_adjacent_face_ids_flat = NULL;
	GeometryPolygon_t* dup_faces = NULL;
	if (!ac) {
		ac = CCTAllocator_stdc(NULL);
	}
	/* collect count */
	dup_faces = (GeometryPolygon_t*)ac->fn_calloc(ac, face_cnt, sizeof(dup_faces[0]));
	if (!dup_faces) {
		goto err;
	}
	dup_v_cnt = 0;
	dup_edge_v_indices_cnt = 0;
	all_face_convex = 1;
	for (i = 0; i < face_cnt; ++i) {
		const GeometryPolygon_t* src_face = faces + i;
		GeometryPolygon_t* dup_face = dup_faces + i;
		if (!src_face->is_convex) {
			all_face_convex = 0;
		}
		mathVec3Copy(dup_face->center, src_face->center);
		mathVec3Copy(dup_face->normal, src_face->normal);
		dup_face->is_convex = src_face->is_convex;
		dup_face->v_indices_cnt = src_face->v_indices_cnt;
		dup_face->edge_cnt = src_face->edge_cnt;
		dup_face->tri_cnt = src_face->tri_cnt;
		dup_face->v_indices = (unsigned int*)ac->fn_malloc(ac, sizeof(dup_face->v_indices[0]) * src_face->v_indices_cnt);
		if (!dup_face->v_indices) {
			goto err;
		}
		dup_face->mesh_v_ids = (unsigned int*)ac->fn_malloc(ac, sizeof(dup_face->mesh_v_ids[0]) * src_face->v_indices_cnt);
		if (!dup_face->mesh_v_ids) {
			goto err;
		}
		dup_face->v_adjacent_infos = (GeometryPolygonVertexAdjacentInfo_t*)ac->fn_malloc(ac, sizeof(dup_face->v_adjacent_infos[0]) * src_face->v_indices_cnt);
		if (!dup_face->v_adjacent_infos) {
			goto err;
		}
		dup_face->edge_v_indices_flat = (unsigned int*)ac->fn_malloc(ac, sizeof(dup_face->edge_v_indices_flat[0]) * src_face->edge_cnt * 2);
		if (!dup_face->edge_v_indices_flat) {
			goto err;
		}
		dup_face->edge_v_ids_flat = (unsigned int*)ac->fn_malloc(ac, sizeof(dup_face->edge_v_ids_flat[0]) * src_face->edge_cnt * 2);
		if (!dup_face->edge_v_ids_flat) {
			goto err;
		}
		dup_face->mesh_edge_ids = (unsigned int*)ac->fn_malloc(ac, sizeof(dup_face->mesh_edge_ids[0]) * src_face->edge_cnt * 2);
		if (!dup_face->mesh_edge_ids) {
			goto err;
		}
		dup_face->tri_v_indices_flat = (unsigned int*)ac->fn_malloc(ac, sizeof(dup_face->tri_v_indices_flat[0]) * src_face->tri_cnt * 3);
		if (!dup_face->tri_v_indices_flat) {
			goto err;
		}
		if (src_face->concave_tri_v_ids_flat) {
			dup_face->concave_tri_v_ids_flat = (unsigned int*)ac->fn_malloc(ac, sizeof(dup_face->concave_tri_v_ids_flat[0]) * src_face->tri_cnt * 3);
			if (!dup_face->concave_tri_v_ids_flat) {
				goto err;
			}
		}
		if (src_face->concave_tri_edge_ids_flat) {
			dup_face->concave_tri_edge_ids_flat = (unsigned int*)ac->fn_malloc(ac, sizeof(dup_face->concave_tri_edge_ids_flat[0]) * src_face->tri_cnt * 3);
			if (!dup_face->concave_tri_edge_ids_flat) {
				goto err;
			}
		}
		dup_v_cnt += src_face->v_indices_cnt;
		dup_edge_v_indices_cnt += src_face->edge_cnt * 2;
	}
	/* rebuild vertices */
	dup_v = (CCTNum_t(*)[3])ac->fn_malloc(ac, sizeof(dup_v[0]) * dup_v_cnt);
	if (!dup_v) {
		goto err;
	}
	dup_v_indices_flat = (unsigned int*)ac->fn_malloc(ac, sizeof(dup_v_indices_flat[0]) * dup_v_cnt);
	if (!dup_v_indices_flat) {
		goto err;
	}
	dup_v_cnt = 0;
	for (i = 0; i < face_cnt; ++i) {
		const GeometryPolygon_t* src_face = faces + i;
		unsigned int* dup_face_v_indices = (unsigned int*)dup_faces[i].v_indices;
		unsigned int* dup_face_mesh_v_ids = (unsigned int*)dup_faces[i].mesh_v_ids;
		GeometryPolygonVertexAdjacentInfo_t* dup_face_v_adjacent_infos = (GeometryPolygonVertexAdjacentInfo_t*)dup_faces[i].v_adjacent_infos;
		for (j = 0; j < src_face->v_indices_cnt; ++j) {
			unsigned int k;
			const CCTNum_t* src_v = src_face->v[src_face->v_indices[j]];
			for (k = 0; k < dup_v_cnt && !mathVec3Equal(dup_v[k], src_v); ++k);
			if (k >= dup_v_cnt) {
				dup_v_indices_flat[dup_v_cnt] = dup_v_cnt;
				mathVec3Copy(dup_v[dup_v_cnt], src_v);
				++dup_v_cnt;
			}
			dup_face_v_indices[j] = k;
			dup_face_mesh_v_ids[j] = k;
			dup_face_v_adjacent_infos[j] = src_face->v_adjacent_infos[j];
		}
		dup_faces[i].v = dup_v;
	}
	/* rebuild face tri vertices indices and concave datas */
	for (i = 0; i < face_cnt; ++i) {
		const GeometryPolygon_t* src_face = faces + i;
		unsigned int* dup_face_tri_v_indices_flat = (unsigned int*)dup_faces[i].tri_v_indices_flat;
		unsigned int* dup_face_concave_tri_v_ids_flat = (unsigned int*)dup_faces[i].concave_tri_v_ids_flat;
		unsigned int* dup_face_concave_tri_edge_ids_flat = (unsigned int*)dup_faces[i].concave_tri_edge_ids_flat;
		for (j = 0; j < src_face->tri_cnt * 3; ++j) {
			unsigned int k;
			const CCTNum_t* src_v = src_face->v[src_face->tri_v_indices_flat[j]];
			for (k = 0; k < dup_v_cnt && !mathVec3Equal(dup_v[k], src_v); ++k);
			dup_face_tri_v_indices_flat[j] = k;
			if (src_face->concave_tri_v_ids_flat) {
				dup_face_concave_tri_v_ids_flat[j] = src_face->concave_tri_v_ids_flat[j];
			}
			if (src_face->concave_tri_edge_ids_flat) {
				dup_face_concave_tri_edge_ids_flat[j] = src_face->concave_tri_edge_ids_flat[j];
			}
		}
	}
	/* rebuild edge */
	dup_edge_v_indices_flat = (unsigned int*)ac->fn_malloc(ac, sizeof(dup_edge_v_indices_flat[0]) * dup_edge_v_indices_cnt);
	if (!dup_edge_v_indices_flat) {
		goto err;
	}
	dup_edge_v_indices_cnt = 0;
	for (i = 0; i < face_cnt; ++i) {
		const GeometryPolygon_t* src_face = faces + i;
		unsigned int* dup_face_edge_v_indices_flat = (unsigned int*)dup_faces[i].edge_v_indices_flat;
		unsigned int* dup_face_edge_v_ids_flat = (unsigned int*)dup_faces[i].edge_v_ids_flat;
		unsigned int* dup_face_mesh_edge_ids = (unsigned int*)dup_faces[i].mesh_edge_ids;
		for (j = 0; j < src_face->edge_cnt * 2; j += 2) {
			unsigned int k;
			const CCTNum_t* src_edge_v0 = src_face->v[src_face->edge_v_indices_flat[j]];
			const CCTNum_t* src_edge_v1 = src_face->v[src_face->edge_v_indices_flat[j + 1]];
			for (k = 0; k < dup_edge_v_indices_cnt; k += 2) {
				const CCTNum_t* dup_edge_v0 = dup_v[dup_edge_v_indices_flat[k]];
				const CCTNum_t* dup_edge_v1 = dup_v[dup_edge_v_indices_flat[k + 1]];
				if (mathVec3Equal(dup_edge_v0, src_edge_v0) && mathVec3Equal(dup_edge_v1, src_edge_v1)) {
					dup_face_edge_v_indices_flat[j] = dup_edge_v_indices_flat[k];
					dup_face_edge_v_indices_flat[j + 1] = dup_edge_v_indices_flat[k + 1];
					break;
				}
				if (mathVec3Equal(dup_edge_v0, src_edge_v1) && mathVec3Equal(dup_edge_v1, src_edge_v0)) {
					dup_face_edge_v_indices_flat[j] = dup_edge_v_indices_flat[k + 1];
					dup_face_edge_v_indices_flat[j + 1] = dup_edge_v_indices_flat[k];
					break;
				}
			}
			dup_face_mesh_edge_ids[j / 2] = k / 2;
			if (k >= dup_edge_v_indices_cnt) {
				for (k = 0; k < dup_v_cnt && !mathVec3Equal(dup_v[k], src_edge_v0); ++k);
				dup_edge_v_indices_flat[dup_edge_v_indices_cnt++] = k;
				dup_face_edge_v_indices_flat[j] = k;

				for (k = 0; k < dup_v_cnt && !mathVec3Equal(dup_v[k], src_edge_v1); ++k);
				dup_edge_v_indices_flat[dup_edge_v_indices_cnt++] = k;
				dup_face_edge_v_indices_flat[j + 1] = k;
			}
			dup_face_edge_v_ids_flat[j] = src_face->edge_v_ids_flat[j];
			dup_face_edge_v_ids_flat[j + 1] = src_face->edge_v_ids_flat[j + 1];
		}
	}
	dup_edge_v_ids_flat = (unsigned int*)ac->fn_malloc(ac, sizeof(dup_edge_v_ids_flat[0]) * dup_edge_v_indices_cnt);
	if (!dup_edge_v_ids_flat) {
		goto err;
	}
	for (i = 0; i < dup_edge_v_indices_cnt; ++i) {
		dup_edge_v_ids_flat[i] = dup_edge_v_indices_flat[i];
	}
	/* rebuild mesh vertices adjacent infos */
	dup_v_adjacent_infos = (GeometryMeshVertexAdjacentInfo_t*)ac->fn_calloc(ac, dup_v_cnt, sizeof(dup_v_adjacent_infos[0]));
	if (!dup_v_adjacent_infos) {
		goto err;
	}
	for (i = 0; i < dup_v_cnt; ++i) {
		GeometryMeshVertexAdjacentInfo_t* info = dup_v_adjacent_infos + i;
		++v_adjacent_infos_init_cnt;
		if (!MeshVertexAdjacentInfo_GetFaceIds(info, dup_faces, face_cnt, i, ac)) {
			goto err;
		}
		if (!MeshVertexAdjacentInfo_GetEdgeIds(info, dup_edge_v_ids_flat, dup_edge_v_indices_cnt / 2, i, ac)) {
			goto err;
		}
		if (!MeshVertexAdjacentInfo_GetVertexIds(info, dup_edge_v_ids_flat, dup_edge_v_indices_cnt / 2, i, ac)) {
			goto err;
		}
	}
	/* rebuild mesh edge adjacent faces */
	dup_edge_adjacent_face_ids_flat = (unsigned int*)ac->fn_malloc(ac, sizeof(dup_edge_adjacent_face_ids_flat[0]) * dup_edge_v_indices_cnt);
	if (!dup_edge_adjacent_face_ids_flat) {
		goto err;
	}
	for (i = 0; i < dup_edge_v_indices_cnt; i += 2) {
		if (!MeshEdgeAdjacentFace(dup_faces, face_cnt, dup_edge_v_indices_flat, i / 2, dup_edge_adjacent_face_ids_flat + i)) {
			goto err;
		}
	}
	/* set mesh fields */
	mathVerticesFindMinMaxXYZ((const CCTNum_t(*)[3])dup_v, dup_v_cnt, dup_mesh->bound_box.min_v, dup_mesh->bound_box.max_v);
	mathAABBFixSize(dup_mesh->bound_box.min_v, dup_mesh->bound_box.max_v);
	dup_mesh->v = dup_v;
	dup_mesh->v_indices_cnt = dup_v_cnt;
	dup_mesh->v_indices = dup_v_indices_flat;
	dup_mesh->v_adjacent_infos = dup_v_adjacent_infos;
	dup_mesh->edge_cnt = dup_edge_v_indices_cnt / 2;
	dup_mesh->edge_v_indices_flat = dup_edge_v_indices_flat;
	dup_mesh->edge_v_ids_flat = dup_edge_v_ids_flat;
	dup_mesh->edge_adjacent_face_ids_flat = dup_edge_adjacent_face_ids_flat;
	dup_mesh->polygons_cnt = face_cnt;
	dup_mesh->polygons = dup_faces;
	dup_mesh->allocator_ptr = ac;
	dup_mesh->_is_buffer_view = 0;
	dup_mesh->is_closed = Mesh_IsClosed(dup_mesh);
	dup_mesh->is_convex = (dup_mesh->is_closed && all_face_convex);
	/* finish */
	return dup_mesh;
err:
	for (i = 0; i < v_adjacent_infos_init_cnt; ++i) {
		MeshVertexAdjacentInfo_free(dup_v_adjacent_infos + i, ac);
	}
	ac->fn_free(ac, dup_v_adjacent_infos);
	for (i = 0; i < face_cnt; ++i) {
		Polygon_ClearWithoutVertices(dup_faces + i, ac);
	}
	ac->fn_free(ac, dup_faces);
	ac->fn_free(ac, dup_v);
	ac->fn_free(ac, dup_v_indices_flat);
	ac->fn_free(ac, dup_edge_v_indices_flat);
	ac->fn_free(ac, dup_edge_v_ids_flat);
	ac->fn_free(ac, dup_edge_adjacent_face_ids_flat);
	return NULL;
}

void mathMeshClear(GeometryMesh_t* mesh) {
	const CCTAllocator_t* ac;
	if (!mesh) {
		return;
	}
	if ((const void*)mesh->v_indices >= (const void*)&CCTConstVal_ &&
		(const void*)mesh->v_indices < (const void*)(&CCTConstVal_ + 1)) {
		return;
	}
	ac = mesh->allocator_ptr;
	if (mesh->_is_buffer_view) {
		ac->fn_free(ac, mesh->polygons);
		mesh->polygons = NULL;
		mesh->polygons_cnt = 0;
		mesh->edge_adjacent_face_ids_flat = NULL;
		mesh->edge_v_ids_flat = NULL;
		mesh->edge_v_indices_flat = NULL;
		mesh->edge_cnt = 0;
		ac->fn_free(ac, (void*)mesh->v_adjacent_infos);
		mesh->v_adjacent_infos = NULL;
		mesh->v_indices = NULL;
		mesh->v_indices_cnt = 0;
		mesh->v = NULL;
		return;
	}
	free_all_faces(mesh);
	free_all_adjacent_infos(mesh);
	if (mesh->edge_v_ids_flat) {
		ac->fn_free(ac, (void*)mesh->edge_v_ids_flat);
		mesh->edge_v_ids_flat = NULL;
	}
	if (mesh->edge_v_indices_flat) {
		ac->fn_free(ac, (void*)mesh->edge_v_indices_flat);
		mesh->edge_v_indices_flat = NULL;
	}
	mesh->edge_cnt = 0;
	if (mesh->v_indices) {
		ac->fn_free(ac, (void*)mesh->v_indices);
		mesh->v_indices = NULL;
		mesh->v_indices_cnt = 0;
	}
	if (mesh->v) {
		ac->fn_free(ac, mesh->v);
		mesh->v = NULL;
	}
}

#ifdef __cplusplus
}
#endif