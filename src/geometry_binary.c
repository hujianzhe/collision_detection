//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/polygon.h"
#include "../inc/mesh.h"
#include <stdint.h>
#include <stdlib.h>

static int safe_add(size_t* total, size_t v) {
	if (SIZE_MAX - v < *total) {
		return 0;
	}
	*total += v;
	return 1;
}
static int safe_add_mul(size_t* total, unsigned int mul_left, size_t mul_right) {
	if (mul_left > SIZE_MAX / mul_right) {
		return 0;
	}
	return safe_add(total, mul_left * mul_right);
}

static size_t BinarySize_GeometryMeshVertexAdjacentInfo(const GeometryMeshVertexAdjacentInfo_t* info) {
	size_t total_size = 0;
	if (!safe_add(&total_size, sizeof(info->v_cnt) + sizeof(info->edge_cnt) + sizeof(info->face_cnt))) { return 0; }
	if (!safe_add_mul(&total_size, sizeof(info->v_ids[0]), info->v_cnt)) { return 0; }
	if (!safe_add_mul(&total_size, sizeof(info->edge_ids[0]), info->edge_cnt)) { return 0; }
	if (!safe_add_mul(&total_size, sizeof(info->face_ids[0]), info->face_cnt)) { return 0; }
	return total_size;
}

static size_t BinarySave_GeometryMeshVertexAdjacentInfo(const GeometryMeshVertexAdjacentInfo_t* info, void* buffer) {
	size_t off = 0;
	char* p = (char*)buffer;
	unsigned int j;

	*(unsigned int*)&p[off] = info->v_cnt;
	off += sizeof(unsigned int);
	for (j = 0; j < info->v_cnt; ++j) {
		*(unsigned int*)&p[off] = info->v_ids[j];
		off += sizeof(unsigned int);
	}

	*(unsigned int*)&p[off] = info->edge_cnt;
	off += sizeof(unsigned int);
	for (j = 0; j < info->edge_cnt; ++j) {
		*(unsigned int*)&p[off] = info->edge_ids[j];
		off += sizeof(unsigned int);
	}

	*(unsigned int*)&p[off] = info->face_cnt;
	off += sizeof(unsigned int);
	for (j = 0; j < info->face_cnt; ++j) {
		*(unsigned int*)&p[off] = info->face_ids[j];
		off += sizeof(unsigned int);
	}
	return off;
}

static size_t BinaryLoad_GeometryMeshVertexAdjacentInfo(const void* buffer, size_t len, GeometryMeshVertexAdjacentInfo_t* info) {
	size_t off = 0;
	const char* p = (const char*)buffer;
	unsigned int* v_ids = NULL, *edge_ids = NULL, *face_ids = NULL;
	unsigned int j;

	if (sizeof(unsigned int) > len - off) {
		goto err;
	}
	info->v_cnt = *(const unsigned int*)&p[off];
	off += sizeof(unsigned int);
	v_ids = (unsigned int*)malloc(sizeof(unsigned int) * info->v_cnt);
	if (!v_ids) {
		goto err;
	}
	for (j = 0; j < info->v_cnt; ++j) {
		if (sizeof(unsigned int) > len - off) {
			goto err;
		}
		v_ids[j] = *(unsigned int*)&p[off];
		off += sizeof(unsigned int);
	}
	info->v_ids = v_ids;

	if (sizeof(unsigned int) > len - off) {
		goto err;
	}
	info->edge_cnt = *(const unsigned int*)&p[off];
	off += sizeof(unsigned int);
	edge_ids = (unsigned int*)malloc(sizeof(unsigned int) * info->edge_cnt);
	if (!edge_ids) {
		goto err;
	}
	for (j = 0; j < info->edge_cnt; ++j) {
		if (sizeof(unsigned int) > len - off) {
			goto err;
		}
		edge_ids[j] = *(unsigned int*)&p[off];
		off += sizeof(unsigned int);
	}
	info->edge_ids = edge_ids;

	if (sizeof(unsigned int) > len - off) {
		goto err;
	}
	info->face_cnt = *(const unsigned int*)&p[off];
	off += sizeof(unsigned int);
	face_ids = (unsigned int*)malloc(sizeof(unsigned int) * info->face_cnt);
	if (!face_ids) {
		goto err;
	}
	for (j = 0; j < info->face_cnt; ++j) {
		if (sizeof(unsigned int) > len - off) {
			goto err;
		}
		face_ids[j] = *(unsigned int*)&p[off];
		off += sizeof(unsigned int);
	}
	info->face_ids = face_ids;

	return off;
err:
	free(v_ids);
	free(edge_ids);
	free(face_ids);
	return 0;
}

static size_t BinarySize_GeometryMeshFace(const GeometryPolygon_t* face) {
	size_t total_size = 0;
	if (!safe_add(&total_size,
		sizeof(face->center) + sizeof(face->normal) + sizeof(face->is_convex) +
		sizeof(face->v_indices_cnt) + sizeof(face->edge_cnt) + sizeof(face->tri_cnt)
	)) { return 0; }

	if (!safe_add_mul(&total_size, sizeof(face->v_indices[0]), face->v_indices_cnt)) { return 0; }
	if (!safe_add_mul(&total_size, sizeof(face->mesh_v_ids[0]), face->v_indices_cnt)) { return 0; }
	if (!safe_add_mul(&total_size, sizeof(face->v_adjacent_infos[0]), face->v_indices_cnt)) { return 0; }

	if (!safe_add_mul(&total_size, sizeof(face->edge_v_indices[0]), face->edge_cnt)) { return 0; }
	if (!safe_add_mul(&total_size, sizeof(face->edge_v_ids[0]), face->edge_cnt)) { return 0; }
	if (!safe_add_mul(&total_size, sizeof(face->mesh_edge_ids[0]), face->edge_cnt)) { return 0; }

	if (!safe_add_mul(&total_size, sizeof(face->tri_v_indices[0]), face->tri_cnt)) { return 0; }
	if (!face->is_convex) {
		if (!safe_add_mul(&total_size, sizeof(face->concave_tri_v_ids[0]), face->tri_cnt)) { return 0; }
		if (!safe_add_mul(&total_size, sizeof(face->concave_tri_edge_ids[0]), face->tri_cnt)) { return 0; }
	}
	return total_size;
}

static size_t BinarySave_GeometryMeshFace(const GeometryPolygon_t* face, void* buffer) {
	size_t off = 0;
	char* p = (char*)buffer;
	unsigned int j;

	mathVec3Copy((CCTNum_t*)&p[off], face->center);
	off += sizeof(face->center);
	mathVec3Copy((CCTNum_t*)&p[off], face->normal);
	off += sizeof(face->normal);
	*(short*)&p[off] = face->is_convex;
	off += sizeof(short);

	*(unsigned int*)&p[off] = face->v_indices_cnt;
	off += sizeof(unsigned int);
	for (j = 0; j < face->v_indices_cnt; ++j) {
		*(unsigned int*)&p[off] = face->v_indices[j];
		off += sizeof(unsigned int);
	}
	for (j = 0; j < face->v_indices_cnt; ++j) {
		*(unsigned int*)&p[off] = face->mesh_v_ids[j];
		off += sizeof(unsigned int);
	}
	for (j = 0; j < face->v_indices_cnt; ++j) {
		*(GeometryPolygonVertexAdjacentInfo_t*)&p[off] = face->v_adjacent_infos[j];
		off += sizeof(GeometryPolygonVertexAdjacentInfo_t);
	}

	*(unsigned int*)&p[off] = face->edge_cnt;
	off += sizeof(unsigned int);
	for (j = 0; j < face->edge_cnt + face->edge_cnt; ++j) {
		*(unsigned int*)&p[off] = face->edge_v_indices_flat[j];
		off += sizeof(unsigned int);
	}
	for (j = 0; j < face->edge_cnt + face->edge_cnt; ++j) {
		*(unsigned int*)&p[off] = face->edge_v_ids_flat[j];
		off += sizeof(unsigned int);
	}
	for (j = 0; j < face->edge_cnt; ++j) {
		*(unsigned int*)&p[off] = face->mesh_edge_ids[j];
		off += sizeof(unsigned int);
	}

	*(unsigned int*)&p[off] = face->tri_cnt;
	off += sizeof(unsigned int);
	for (j = 0; j < face->tri_cnt * 3; ) {
		*(unsigned int*)&p[off] = face->tri_v_indices_flat[j++];
		off += sizeof(unsigned int);
		*(unsigned int*)&p[off] = face->tri_v_indices_flat[j++];
		off += sizeof(unsigned int);
		*(unsigned int*)&p[off] = face->tri_v_indices_flat[j++];
		off += sizeof(unsigned int);
	}
	if (!face->is_convex) {
		for (j = 0; j < face->tri_cnt * 3; ) {
			*(unsigned int*)&p[off] = face->concave_tri_v_ids_flat[j++];
			off += sizeof(unsigned int);
			*(unsigned int*)&p[off] = face->concave_tri_v_ids_flat[j++];
			off += sizeof(unsigned int);
			*(unsigned int*)&p[off] = face->concave_tri_v_ids_flat[j++];
			off += sizeof(unsigned int);
		}
		for (j = 0; j < face->tri_cnt * 3; ) {
			*(unsigned int*)&p[off] = face->concave_tri_edge_ids_flat[j++];
			off += sizeof(unsigned int);
			*(unsigned int*)&p[off] = face->concave_tri_edge_ids_flat[j++];
			off += sizeof(unsigned int);
			*(unsigned int*)&p[off] = face->concave_tri_edge_ids_flat[j++];
			off += sizeof(unsigned int);
		}
	}
	return off;
}

static size_t BinaryLoad_GeometryMeshFace(const void* buffer, size_t len, GeometryPolygon_t* face) {
	unsigned int j;
	unsigned int* v_indices = NULL, *mesh_v_ids = NULL;
	unsigned int* edge_v_indices_flat = NULL, *edge_v_ids_flat = NULL, *mesh_edge_ids = NULL;
	unsigned int* tri_v_indices_flat = NULL, *concave_tri_v_ids_flat = NULL, *concave_tri_edge_ids_flat = NULL;
	GeometryPolygonVertexAdjacentInfo_t* v_adjacent_infos = NULL;
	const char* p = (const char*)buffer;
	size_t off = 0;

	if (sizeof(face->center) > len - off) { goto err; }
	mathVec3Copy(face->center, (const CCTNum_t*)&p[off]);
	off += sizeof(CCTNum_t[3]);

	if (sizeof(face->normal) > len - off) { goto err; }
	mathVec3Copy(face->normal, (const CCTNum_t*)&p[off]);
	off += sizeof(CCTNum_t[3]);

	if (sizeof(short) > len - off) { goto err; }
	face->is_convex = *(const short*)&p[off];
	off += sizeof(short);

	if (sizeof(unsigned int) > len - off) { goto err; }
	face->v_indices_cnt = *(const unsigned int*)&p[off];
	off += sizeof(unsigned int);
	v_indices = (unsigned int*)malloc(sizeof(unsigned int) * face->v_indices_cnt);
	if (!v_indices) { goto err; }
	mesh_v_ids = (unsigned int*)malloc(sizeof(unsigned int) * face->v_indices_cnt);
	if (!mesh_v_ids) { goto err; }
	v_adjacent_infos = (GeometryPolygonVertexAdjacentInfo_t*)malloc(sizeof(GeometryPolygonVertexAdjacentInfo_t) * face->v_indices_cnt);
	if (!v_adjacent_infos) { goto err; }
	for (j = 0; j < face->v_indices_cnt; ++j) {
		if (sizeof(unsigned int) > len - off) { goto err; }
		v_indices[j] = *(const unsigned int*)&p[off];
		off += sizeof(unsigned int);
	}
	face->v_indices = v_indices;
	for (j = 0; j < face->v_indices_cnt; ++j) {
		if (sizeof(unsigned int) > len - off) { goto err; }
		mesh_v_ids[j] = *(const unsigned int*)&p[off];
		off += sizeof(unsigned int);
	}
	face->mesh_v_ids = mesh_v_ids;
	for (j = 0; j < face->v_indices_cnt; ++j) {
		if (sizeof(GeometryPolygonVertexAdjacentInfo_t) > len - off) { goto err; }
		v_adjacent_infos[j] = *(const GeometryPolygonVertexAdjacentInfo_t*)&p[off];
		off += sizeof(GeometryPolygonVertexAdjacentInfo_t);
	}
	face->v_adjacent_infos = v_adjacent_infos;

	if (sizeof(unsigned int) > len - off) { goto err; }
	face->edge_cnt = *(const unsigned int*)&p[off];
	off += sizeof(unsigned int);
	edge_v_indices_flat = (unsigned int*)malloc(sizeof(unsigned int) * (face->edge_cnt + face->edge_cnt));
	if (!edge_v_indices_flat) { goto err; }
	edge_v_ids_flat = (unsigned int*)malloc(sizeof(unsigned int) * (face->edge_cnt + face->edge_cnt));
	if (!edge_v_ids_flat) { goto err; }
	mesh_edge_ids = (unsigned int*)malloc(sizeof(unsigned int) * face->edge_cnt);
	if (!mesh_edge_ids) { goto err; }
	for (j = 0; j < face->edge_cnt + face->edge_cnt; ++j) {
		if (sizeof(unsigned int) > len - off) { goto err; }
		edge_v_indices_flat[j] = *(const unsigned int*)&p[off];
		off += sizeof(unsigned int);
	}
	face->edge_v_indices_flat = edge_v_indices_flat;
	for (j = 0; j < face->edge_cnt + face->edge_cnt; ++j) {
		if (sizeof(unsigned int) > len - off) { goto err; }
		edge_v_ids_flat[j] = *(const unsigned int*)&p[off];
		off += sizeof(unsigned int);
	}
	face->edge_v_ids_flat = edge_v_ids_flat;
	for (j = 0; j < face->edge_cnt; ++j) {
		if (sizeof(unsigned int) > len - off) { goto err; }
		mesh_edge_ids[j] = *(const unsigned int*)&p[off];
		off += sizeof(unsigned int);
	}
	face->mesh_edge_ids = mesh_edge_ids;

	if (sizeof(unsigned int) > len - off) { goto err; }
	face->tri_cnt = *(const unsigned int*)&p[off];
	off += sizeof(unsigned int);
	tri_v_indices_flat = (unsigned int*)malloc(sizeof(unsigned int) * face->tri_cnt * 3);
	if (!tri_v_indices_flat) { goto err; }
	for (j = 0; j < face->tri_cnt * 3; ++j) {
		if (sizeof(unsigned int) > len - off) { goto err; }
		tri_v_indices_flat[j] = *(const unsigned int*)&p[off];
		off += sizeof(unsigned int);
	}
	face->tri_v_indices_flat = tri_v_indices_flat;
	if (face->is_convex) {
		face->concave_tri_v_ids_flat = NULL;
		face->concave_tri_edge_ids_flat = NULL;
	}
	else {
		concave_tri_v_ids_flat = (unsigned int*)malloc(sizeof(unsigned int) * face->tri_cnt * 3);
		if (!concave_tri_v_ids_flat) { goto err; }
		for (j = 0; j < face->tri_cnt * 3; ++j) {
			if (sizeof(unsigned int) > len - off) { goto err; }
			concave_tri_v_ids_flat[j] = *(const unsigned int*)&p[off];
			off += sizeof(unsigned int);
		}
		face->concave_tri_v_ids_flat = concave_tri_v_ids_flat;

		concave_tri_edge_ids_flat = (unsigned int*)malloc(sizeof(unsigned int) * face->tri_cnt * 3);
		if (!concave_tri_edge_ids_flat) { goto err; }
		for (j = 0; j < face->tri_cnt * 3; ++j) {
			if (sizeof(unsigned int) > len - off) { goto err; }
			concave_tri_edge_ids_flat[j] = *(const unsigned int*)&p[off];
			off += sizeof(unsigned int);
		}
		face->concave_tri_edge_ids_flat = concave_tri_edge_ids_flat;
	}
	return off;
err:
	free(v_indices);
	free(v_adjacent_infos);
	free(mesh_v_ids);
	free(edge_v_indices_flat);
	free(edge_v_ids_flat);
	free(mesh_edge_ids);
	free(tri_v_indices_flat);
	free(concave_tri_v_ids_flat);
	free(concave_tri_edge_ids_flat);
	return 0;
}

#ifdef __cplusplus
extern "C" {
#endif

size_t mathMeshBinarySize(const GeometryMesh_t* mesh) {
	size_t total_size = 0;
	unsigned int i, v_cnt = 0;
	for (i = 0; i < mesh->v_indices_cnt; ++i) {
		if (mesh->v_indices[i] >= v_cnt) {
			v_cnt = mesh->v_indices[i] + 1;
		}
	}
	if (!safe_add_mul(&total_size, sizeof(mesh->v[0]), v_cnt)) { return 0; }
	if (!safe_add(&total_size, sizeof(v_cnt) + sizeof(mesh->bound_box) + sizeof(mesh->is_convex) + sizeof(mesh->is_closed))) { return 0; }

	if (!safe_add(&total_size, sizeof(mesh->v_indices_cnt))) { return 0; }
	if (!safe_add_mul(&total_size, sizeof(mesh->v_indices[0]), mesh->v_indices_cnt)) { return 0; }
	for (i = 0; i < mesh->v_indices_cnt; ++i) {
		size_t sz = BinarySize_GeometryMeshVertexAdjacentInfo(mesh->v_adjacent_infos + i);
		if (!sz) { return 0; }
		if (!safe_add(&total_size, sz)) { return 0; }
	}

	if (!safe_add(&total_size, sizeof(mesh->edge_cnt))) { return 0; }
	if (!safe_add_mul(&total_size, sizeof(mesh->edge_v_indices[0]), mesh->edge_cnt)) { return 0; }
	if (!safe_add_mul(&total_size, sizeof(mesh->edge_v_ids[0]), mesh->edge_cnt)) { return 0; }
	if (!safe_add_mul(&total_size, sizeof(mesh->edge_adjacent_face_ids[0]), mesh->edge_cnt)) { return 0; }

	if (!safe_add(&total_size, sizeof(mesh->polygons_cnt))) { return 0; }
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		size_t sz = BinarySize_GeometryMeshFace(mesh->polygons + i);
		if (!sz) { return 0; }
		if (!safe_add(&total_size, sz)) { return 0; }
	}

	return total_size;
}

size_t mathMeshSaveBinary(const GeometryMesh_t* mesh, void* buffer) {
	size_t off = 0;
	char* p = (char*)buffer;
	unsigned int i, v_cnt = 0;
	CCTNum_t nan_value = CCTNum(0.0) / CCTNum(0.0);

	*(GeometryAABB_t*)&p[off] = mesh->bound_box;
	off += sizeof(GeometryAABB_t);
	*(short*)&p[off] = mesh->is_convex;
	off += sizeof(short);
	*(short*)&p[off] = mesh->is_closed;
	off += sizeof(short);

	for (i = 0; i < mesh->v_indices_cnt; ++i) {
		if (mesh->v_indices[i] >= v_cnt) {
			v_cnt = mesh->v_indices[i] + 1;
		}
	}
	*(unsigned int*)&p[off] = v_cnt;
	off += sizeof(unsigned int);
	if (v_cnt > mesh->v_indices_cnt) {
		for (i = 0; i < v_cnt; ++i) {
			CCTNum_t(*pv)[3] = (CCTNum_t(*)[3])&p[off];
			pv[i][0] = nan_value;
			pv[i][1] = nan_value;
			pv[i][2] = nan_value;
		}
	}
	for (i = 0; i < mesh->v_indices_cnt; ++i) {
		CCTNum_t(*pv)[3] = (CCTNum_t(*)[3])&p[off];
		unsigned int idx = mesh->v_indices[i];
		mathVec3Copy(pv[idx], mesh->v[idx]);
	}
	off += sizeof(CCTNum_t[3]) * v_cnt;

	*(unsigned int*)&p[off] = mesh->v_indices_cnt;
	off += sizeof(unsigned int);
	for (i = 0; i < mesh->v_indices_cnt; ++i) {
		*(unsigned int*)&p[off] = mesh->v_indices[i];
		off += sizeof(unsigned int);
	}
	for (i = 0; i < mesh->v_indices_cnt; ++i) {
		size_t sz = BinarySave_GeometryMeshVertexAdjacentInfo(mesh->v_adjacent_infos + i, p + off);
		off += sz;
	}

	*(unsigned int*)&p[off] = mesh->edge_cnt;
	off += sizeof(unsigned int);
	for (i = 0; i < mesh->edge_cnt + mesh->edge_cnt; ++i) {
		*(unsigned int*)&p[off] = mesh->edge_v_indices_flat[i];
		off += sizeof(unsigned int);
	}
	for (i = 0; i < mesh->edge_cnt + mesh->edge_cnt; ++i) {
		*(unsigned int*)&p[off] = mesh->edge_v_ids_flat[i];
		off += sizeof(unsigned int);
	}
	for (i = 0; i < mesh->edge_cnt + mesh->edge_cnt; ++i) {
		*(unsigned int*)&p[off] = mesh->edge_adjacent_face_ids_flat[i];
		off += sizeof(unsigned int);
	}

	*(unsigned int*)&p[off] = mesh->polygons_cnt;
	off += sizeof(unsigned int);
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		size_t sz = BinarySave_GeometryMeshFace(mesh->polygons + i, p + off);
		off += sz;
	}
	return off;
}

size_t mathMeshLoadBinary(const void* buffer, size_t len, GeometryMesh_t* mesh) {
	size_t off = 0;
	const char* p = (const char*)buffer;
	unsigned int i, v_cnt;
	GeometryMesh_t tmp = { 0 };
	unsigned int* mesh_v_indices = NULL;
	GeometryMeshVertexAdjacentInfo_t* mesh_v_adjacent_infos = NULL;
	unsigned int* mesh_edge_v_indices_flat = NULL, *mesh_edge_v_ids_flat = NULL, *mesh_edge_adjacent_face_ids_flat = NULL;
	GeometryPolygon_t* mesh_polygons = NULL;

	if (sizeof(GeometryAABB_t) > len - off) {
		goto err;
	}
	tmp.bound_box = *(const GeometryAABB_t*)&p[off];
	off += sizeof(GeometryAABB_t);

	if (sizeof(unsigned short) > len - off) {
		goto err;
	}
	tmp.is_convex = *(const unsigned short*)&p[off];
	off += sizeof(unsigned short);

	if (sizeof(unsigned short) > len - off) {
		goto err;
	}
	tmp.is_closed = *(const unsigned short*)&p[off];
	off += sizeof(unsigned short);

	if (sizeof(unsigned int) > len - off) {
		goto err;
	}
	v_cnt = *(const unsigned int*)&p[off];
	off += sizeof(unsigned int);
	tmp.v = (CCTNum_t(*)[3])malloc(sizeof(CCTNum_t[3]) * v_cnt);
	if (!tmp.v) {
		goto err;
	}
	for (i = 0; i < v_cnt; ++i) {
		const CCTNum_t* pv = (const CCTNum_t*)&p[off];
		if (sizeof(CCTNum_t[3]) > len - off) {
			goto err;
		}
		mathVec3Copy(tmp.v[i], pv);
		off += sizeof(CCTNum_t[3]);
	}

	if (sizeof(unsigned int) > len - off) {
		goto err;
	}
	tmp.v_indices_cnt = *(const unsigned int*)&p[off];
	off += sizeof(unsigned int);
	mesh_v_indices = (unsigned int*)malloc(sizeof(unsigned int) * tmp.v_indices_cnt);
	if (!mesh_v_indices) {
		goto err;
	}
	for (i = 0; i < tmp.v_indices_cnt; ++i) {
		if (sizeof(unsigned int) > len - off) {
			goto err;
		}
		mesh_v_indices[i] = *(const unsigned int*)&p[off];
		off += sizeof(unsigned int);
	}
	tmp.v_indices = mesh_v_indices;
	mesh_v_adjacent_infos = (GeometryMeshVertexAdjacentInfo_t*)malloc(sizeof(GeometryMeshVertexAdjacentInfo_t) * tmp.v_indices_cnt);
	if (!mesh_v_adjacent_infos) {
		goto err;
	}
	tmp.v_adjacent_infos = mesh_v_adjacent_infos;
	for (i = 0; i < tmp.v_indices_cnt; ++i) {
		size_t sz = BinaryLoad_GeometryMeshVertexAdjacentInfo(p + off, len - off, mesh_v_adjacent_infos + i);
		if (!sz) {
			tmp.v_indices_cnt = i;
			goto err;
		}
		off += sz;
	}

	if (sizeof(unsigned int) > len - off) {
		goto err;
	}
	tmp.edge_cnt = *(const unsigned int*)&p[off];
	off += sizeof(unsigned int);
	mesh_edge_v_indices_flat = (unsigned int*)malloc(sizeof(unsigned int) * (tmp.edge_cnt + tmp.edge_cnt));
	if (!mesh_edge_v_indices_flat) {
		goto err;
	}
	mesh_edge_v_ids_flat = (unsigned int*)malloc(sizeof(unsigned int) * (tmp.edge_cnt + tmp.edge_cnt));
	if (!mesh_edge_v_ids_flat) {
		goto err;
	}
	mesh_edge_adjacent_face_ids_flat = (unsigned int*)malloc(sizeof(unsigned int) * (tmp.edge_cnt + tmp.edge_cnt));
	if (!mesh_edge_adjacent_face_ids_flat) {
		goto err;
	}
	for (i = 0; i < tmp.edge_cnt + tmp.edge_cnt; ++i) {
		if (sizeof(unsigned int) > len - off) {
			goto err;
		}
		mesh_edge_v_indices_flat[i] = *(const unsigned int*)&p[off];
		off += sizeof(unsigned int);
	}
	tmp.edge_v_indices_flat = mesh_edge_v_indices_flat;
	for (i = 0; i < tmp.edge_cnt + tmp.edge_cnt; ++i) {
		if (sizeof(unsigned int) > len - off) {
			goto err;
		}
		mesh_edge_v_ids_flat[i] = *(const unsigned int*)&p[off];
		off += sizeof(unsigned int);
	}
	tmp.edge_v_ids_flat = mesh_edge_v_ids_flat;
	for (i = 0; i < tmp.edge_cnt + tmp.edge_cnt; ++i) {
		if (sizeof(unsigned int) > len - off) {
			goto err;
		}
		mesh_edge_adjacent_face_ids_flat[i] = *(const unsigned int*)&p[off];
		off += sizeof(unsigned int);
	}
	tmp.edge_adjacent_face_ids_flat = mesh_edge_adjacent_face_ids_flat;

	if (sizeof(unsigned int) > len - off) {
		goto err;
	}
	tmp.polygons_cnt = *(const unsigned int*)&p[off];
	off += sizeof(unsigned int);
	mesh_polygons = (GeometryPolygon_t*)malloc(sizeof(GeometryPolygon_t) * tmp.polygons_cnt);
	if (!mesh_polygons) {
		goto err;
	}
	tmp.polygons = mesh_polygons;
	for (i = 0; i < tmp.polygons_cnt; ++i) {
		GeometryPolygon_t* face = mesh_polygons + i;
		size_t sz = BinaryLoad_GeometryMeshFace(p + off, len - off, face);
		if (!sz) {
			tmp.polygons_cnt = i;
			goto err;
		}
		off += sz;
		face->v = tmp.v;
	}

	*mesh = tmp;
	return off;
err:
	mathMeshClear(&tmp);
	return 0;
}

#ifdef __cplusplus
}
#endif
