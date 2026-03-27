//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/mesh.h"
#include <stddef.h>
#include <stdint.h>

extern void Polygon_ClearWithoutVertices(GeometryPolygon_t* polygon, const CCTAllocator_t* ac);
extern void free_data_mesh_vertex_adjacent_info(GeometryMeshVertexAdjacentInfo_t* info, const CCTAllocator_t* ac);

#define BIN_ALIGN_DEF_(sym, T) typedef struct gbin_p_##sym { char c; T v; } gbin_p_##sym##_t
#define BIN_ALIGN_(sym)       offsetof(gbin_p_##sym##_t, v)

BIN_ALIGN_DEF_(GeometryAABB_t, GeometryAABB_t);
BIN_ALIGN_DEF_(short, short);
BIN_ALIGN_DEF_(unsigned_char, unsigned char);
BIN_ALIGN_DEF_(unsigned_int, unsigned int);
BIN_ALIGN_DEF_(CCTNum_t, CCTNum_t);

static size_t align_up_size(size_t v, size_t a) {
	size_t r = v % a;
	if (r != 0) {
		return v + (a - r);
	}
	return v;
}

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

static int safe_align(size_t* total, size_t a) {
	size_t aligned = align_up_size(*total, a);
	size_t pad = aligned - *total;
	return safe_add(total, pad);
}

static int pad_skip(const char* p, size_t len, size_t* off, size_t a) {
	size_t addr, aligned, pad;
	if (!a) { return 1; }
	addr = (size_t)(p + *off);
	aligned = align_up_size(addr, a);
	pad = aligned - addr;
	if (pad > len - *off) { return 0; }
	*off += pad;
	return 1;
}

static int read_u32(const char* p, size_t len, size_t* off, unsigned int* out) {
	if (sizeof(unsigned int) > len - *off) { return 0; }
	*out = *(const unsigned int*)(p + *off);
	*off += sizeof(unsigned int);
	return 1;
}
static int read_i16(const char* p, size_t len, size_t* off, short* out) {
	if (sizeof(short) > len - *off) { return 0; }
	*out = *(const short*)(p + *off);
	*off += sizeof(short);
	return 1;
}
static int read_u8(const char* p, size_t len, size_t* off, unsigned char* out) {
	if (sizeof(unsigned char) > len - *off) { return 0; }
	*out = *(const unsigned char*)(p + *off);
	*off += sizeof(unsigned char);
	return 1;
}

static int write_u32(char* p, size_t* off, unsigned int v) {
	*(unsigned int*)(p + *off) = v;
	*off += sizeof(unsigned int);
	return 1;
}
static int write_i16(char* p, size_t* off, short v) {
	*(short*)(p + *off) = v;
	*off += sizeof(short);
	return 1;
}
static int write_u8(char* p, size_t* off, unsigned char v) {
	*(unsigned char*)(p + *off) = v;
	*off += sizeof(unsigned char);
	return 1;
}

static size_t BinarySize_GeometryMeshVertexAdjacentInfo(const GeometryMeshVertexAdjacentInfo_t* info, size_t total_size) {
	if (!safe_align(&total_size, BIN_ALIGN_(unsigned_int))) { return 0; }
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

	pad_skip(p, SIZE_MAX, &off, BIN_ALIGN_(unsigned_int));
	write_u32(p, &off, info->v_cnt);
	for (j = 0; j < info->v_cnt; ++j) {
		write_u32(p, &off, info->v_ids[j]);
	}

	write_u32(p, &off, info->edge_cnt);
	for (j = 0; j < info->edge_cnt; ++j) {
		write_u32(p, &off, info->edge_ids[j]);
	}

	write_u32(p, &off, info->face_cnt);
	for (j = 0; j < info->face_cnt; ++j) {
		write_u32(p, &off, info->face_ids[j]);
	}
	return off;
}

static size_t BinaryLoad_GeometryMeshVertexAdjacentInfo(const void* buffer, size_t len, GeometryMeshVertexAdjacentInfo_t* info, const CCTAllocator_t* ac) {
	size_t off = 0;
	const char* p = (const char*)buffer;
	unsigned int j;
	unsigned int* v_ids = NULL, *edge_ids = NULL, *face_ids = NULL;

	if (!pad_skip(p, len, &off, BIN_ALIGN_(unsigned_int))) { goto err; }
	if (!read_u32(p, len, &off, &info->v_cnt)) { goto err; }
	v_ids = (unsigned int*)ac->fn_malloc(ac, sizeof(unsigned int) * info->v_cnt);
	if (!v_ids) {
		goto err;
	}
	for (j = 0; j < info->v_cnt; ++j) {
		if (!read_u32(p, len, &off, &v_ids[j])) { goto err; }
	}
	info->v_ids = v_ids;

	if (!read_u32(p, len, &off, &info->edge_cnt)) { goto err; }
	edge_ids = (unsigned int*)ac->fn_malloc(ac, sizeof(unsigned int) * info->edge_cnt);
	if (!edge_ids) {
		goto err;
	}
	for (j = 0; j < info->edge_cnt; ++j) {
		if (!read_u32(p, len, &off, &edge_ids[j])) { goto err; }
	}
	info->edge_ids = edge_ids;

	if (!read_u32(p, len, &off, &info->face_cnt)) { goto err; }
	face_ids = (unsigned int*)ac->fn_malloc(ac, sizeof(unsigned int) * info->face_cnt);
	if (!face_ids) {
		goto err;
	}
	for (j = 0; j < info->face_cnt; ++j) {
		if (!read_u32(p, len, &off, &face_ids[j])) { goto err; }
	}
	info->face_ids = face_ids;

	return off;
err:
	ac->fn_free(ac, v_ids);
	ac->fn_free(ac, edge_ids);
	ac->fn_free(ac, face_ids);
	return 0;
}

static size_t BinarySize_GeometryMeshFace(const GeometryPolygon_t* face, size_t total_size) {
	if (!safe_align(&total_size, BIN_ALIGN_(CCTNum_t))) { return 0; }
	/* center / normal (CCTNum_t aligned) */
	if (!safe_add(&total_size, sizeof(face->center))) { return 0; }
	if (!safe_align(&total_size, BIN_ALIGN_(CCTNum_t))) { return 0; }
	if (!safe_add(&total_size, sizeof(face->normal))) { return 0; }

	/* is_convex (short aligned) */
	if (!safe_align(&total_size, BIN_ALIGN_(short))) { return 0; }
	if (!safe_add(&total_size, sizeof(face->is_convex))) { return 0; }

	/* v_indices_cnt (u32 aligned) */
	if (!safe_align(&total_size, BIN_ALIGN_(unsigned_int))) { return 0; }
	if (!safe_add(&total_size, sizeof(face->v_indices_cnt))) { return 0; }

	if (!safe_add_mul(&total_size, sizeof(face->v_indices[0]), face->v_indices_cnt)) { return 0; }
	if (!safe_add_mul(&total_size, sizeof(face->mesh_v_ids[0]), face->v_indices_cnt)) { return 0; }
	if (!safe_add_mul(&total_size, sizeof(face->v_adjacent_infos[0]), face->v_indices_cnt)) { return 0; }

	/* edge_cnt (u32 aligned) */
	if (!safe_align(&total_size, BIN_ALIGN_(unsigned_int))) { return 0; }
	if (!safe_add(&total_size, sizeof(face->edge_cnt))) { return 0; }
	if (!safe_add_mul(&total_size, sizeof(face->edge_v_indices[0]), face->edge_cnt)) { return 0; }
	if (!safe_add_mul(&total_size, sizeof(face->edge_v_ids[0]), face->edge_cnt)) { return 0; }
	if (!safe_add_mul(&total_size, sizeof(face->mesh_edge_ids[0]), face->edge_cnt)) { return 0; }

	/* tri_cnt (u32 aligned) */
	if (!safe_align(&total_size, BIN_ALIGN_(unsigned_int))) { return 0; }
	if (!safe_add(&total_size, sizeof(face->tri_cnt))) { return 0; }
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

	pad_skip(p, SIZE_MAX, &off, BIN_ALIGN_(CCTNum_t));
	mathVec3Copy((CCTNum_t*)&p[off], face->center);
	off += sizeof(face->center);
	pad_skip(p, SIZE_MAX, &off, BIN_ALIGN_(CCTNum_t));
	mathVec3Copy((CCTNum_t*)&p[off], face->normal);
	off += sizeof(face->normal);
	pad_skip(p, SIZE_MAX, &off, BIN_ALIGN_(short));
	write_i16(p, &off, face->is_convex);

	pad_skip(p, SIZE_MAX, &off, BIN_ALIGN_(unsigned_int));
	write_u32(p, &off, face->v_indices_cnt);
	for (j = 0; j < face->v_indices_cnt; ++j) {
		write_u32(p, &off, face->v_indices[j]);
	}
	for (j = 0; j < face->v_indices_cnt; ++j) {
		write_u32(p, &off, face->mesh_v_ids[j]);
	}
	for (j = 0; j < face->v_indices_cnt; ++j) {
		*(GeometryPolygonVertexAdjacentInfo_t*)&p[off] = face->v_adjacent_infos[j];
		off += sizeof(GeometryPolygonVertexAdjacentInfo_t);
	}

	pad_skip(p, SIZE_MAX, &off, BIN_ALIGN_(unsigned_int));
	write_u32(p, &off, face->edge_cnt);
	for (j = 0; j < face->edge_cnt + face->edge_cnt; ++j) {
		write_u32(p, &off, face->edge_v_indices_flat[j]);
	}
	for (j = 0; j < face->edge_cnt + face->edge_cnt; ++j) {
		write_u32(p, &off, face->edge_v_ids_flat[j]);
	}
	for (j = 0; j < face->edge_cnt; ++j) {
		write_u32(p, &off, face->mesh_edge_ids[j]);
	}

	pad_skip(p, SIZE_MAX, &off, BIN_ALIGN_(unsigned_int));
	write_u32(p, &off, face->tri_cnt);
	for (j = 0; j < face->tri_cnt * 3; ++j) {
		write_u32(p, &off, face->tri_v_indices_flat[j]);
	}
	if (!face->is_convex) {
		for (j = 0; j < face->tri_cnt * 3; ++j) {
			write_u32(p, &off, face->concave_tri_v_ids_flat[j]);
		}
		for (j = 0; j < face->tri_cnt * 3; ++j) {
			write_u32(p, &off, face->concave_tri_edge_ids_flat[j]);
		}
	}
	return off;
}

static size_t BinaryLoad_GeometryMeshFace(const void* buffer, size_t len, GeometryPolygon_t* face, const CCTAllocator_t* ac) {
	unsigned int j;
	unsigned int* v_indices = NULL, *mesh_v_ids = NULL;
	unsigned int* edge_v_indices_flat = NULL, *edge_v_ids_flat = NULL, *mesh_edge_ids = NULL;
	unsigned int* tri_v_indices_flat = NULL, *concave_tri_v_ids_flat = NULL, *concave_tri_edge_ids_flat = NULL;
	GeometryPolygonVertexAdjacentInfo_t* v_adjacent_infos = NULL;
	const char* p = (const char*)buffer;
	size_t off = 0;

	if (!pad_skip(p, len, &off, BIN_ALIGN_(CCTNum_t))) { goto err; }
	if (sizeof(face->center) > len - off) { goto err; }
	mathVec3Copy(face->center, (const CCTNum_t*)&p[off]);
	off += sizeof(face->center);

	if (!pad_skip(p, len, &off, BIN_ALIGN_(CCTNum_t))) { goto err; }
	if (sizeof(face->normal) > len - off) { goto err; }
	mathVec3Copy(face->normal, (const CCTNum_t*)&p[off]);
	off += sizeof(face->normal);

	if (!pad_skip(p, len, &off, BIN_ALIGN_(short))) { goto err; }
	if (!read_i16(p, len, &off, &face->is_convex)) { goto err; }

	if (!pad_skip(p, len, &off, BIN_ALIGN_(unsigned_int))) { goto err; }
	if (!read_u32(p, len, &off, &face->v_indices_cnt)) { goto err; }
	v_indices = (unsigned int*)ac->fn_malloc(ac, sizeof(unsigned int) * face->v_indices_cnt);
	if (!v_indices) { goto err; }
	mesh_v_ids = (unsigned int*)ac->fn_malloc(ac, sizeof(unsigned int) * face->v_indices_cnt);
	if (!mesh_v_ids) { goto err; }
	v_adjacent_infos = (GeometryPolygonVertexAdjacentInfo_t*)ac->fn_malloc(ac, sizeof(GeometryPolygonVertexAdjacentInfo_t) * face->v_indices_cnt);
	if (!v_adjacent_infos) { goto err; }
	for (j = 0; j < face->v_indices_cnt; ++j) {
		if (!read_u32(p, len, &off, &v_indices[j])) { goto err; }
	}
	face->v_indices = v_indices;
	for (j = 0; j < face->v_indices_cnt; ++j) {
		if (!read_u32(p, len, &off, &mesh_v_ids[j])) { goto err; }
	}
	face->mesh_v_ids = mesh_v_ids;
	for (j = 0; j < face->v_indices_cnt; ++j) {
		if (sizeof(GeometryPolygonVertexAdjacentInfo_t) > len - off) { goto err; }
		v_adjacent_infos[j] = *(const GeometryPolygonVertexAdjacentInfo_t*)&p[off];
		off += sizeof(GeometryPolygonVertexAdjacentInfo_t);
	}
	face->v_adjacent_infos = v_adjacent_infos;

	if (!pad_skip(p, len, &off, BIN_ALIGN_(unsigned_int))) { goto err; }
	if (!read_u32(p, len, &off, &face->edge_cnt)) { goto err; }
	edge_v_indices_flat = (unsigned int*)ac->fn_malloc(ac, sizeof(unsigned int) * face->edge_cnt * 2);
	if (!edge_v_indices_flat) { goto err; }
	edge_v_ids_flat = (unsigned int*)ac->fn_malloc(ac, sizeof(unsigned int) * face->edge_cnt * 2);
	if (!edge_v_ids_flat) { goto err; }
	mesh_edge_ids = (unsigned int*)ac->fn_malloc(ac, sizeof(unsigned int) * face->edge_cnt);
	if (!mesh_edge_ids) { goto err; }
	for (j = 0; j < face->edge_cnt * 2; ++j) {
		if (!read_u32(p, len, &off, &edge_v_indices_flat[j])) { goto err; }
	}
	face->edge_v_indices_flat = edge_v_indices_flat;
	for (j = 0; j < face->edge_cnt * 2; ++j) {
		if (!read_u32(p, len, &off, &edge_v_ids_flat[j])) { goto err; }
	}
	face->edge_v_ids_flat = edge_v_ids_flat;
	for (j = 0; j < face->edge_cnt; ++j) {
		if (!read_u32(p, len, &off, &mesh_edge_ids[j])) { goto err; }
	}
	face->mesh_edge_ids = mesh_edge_ids;

	if (!pad_skip(p, len, &off, BIN_ALIGN_(unsigned_int))) { goto err; }
	if (!read_u32(p, len, &off, &face->tri_cnt)) { goto err; }
	tri_v_indices_flat = (unsigned int*)ac->fn_malloc(ac, sizeof(unsigned int) * face->tri_cnt * 3);
	if (!tri_v_indices_flat) { goto err; }
	for (j = 0; j < face->tri_cnt * 3; ++j) {
		if (!read_u32(p, len, &off, &tri_v_indices_flat[j])) { goto err; }
	}
	face->tri_v_indices_flat = tri_v_indices_flat;
	if (face->is_convex) {
		face->concave_tri_v_ids_flat = NULL;
		face->concave_tri_edge_ids_flat = NULL;
	}
	else {
		concave_tri_v_ids_flat = (unsigned int*)ac->fn_malloc(ac, sizeof(unsigned int) * face->tri_cnt * 3);
		if (!concave_tri_v_ids_flat) { goto err; }
		for (j = 0; j < face->tri_cnt * 3; ++j) {
			if (!read_u32(p, len, &off, &concave_tri_v_ids_flat[j])) { goto err; }
		}
		face->concave_tri_v_ids_flat = concave_tri_v_ids_flat;

		concave_tri_edge_ids_flat = (unsigned int*)ac->fn_malloc(ac, sizeof(unsigned int) * face->tri_cnt * 3);
		if (!concave_tri_edge_ids_flat) { goto err; }
		for (j = 0; j < face->tri_cnt * 3; ++j) {
			if (!read_u32(p, len, &off, &concave_tri_edge_ids_flat[j])) { goto err; }
		}
		face->concave_tri_edge_ids_flat = concave_tri_edge_ids_flat;
	}
	return off;
err:
	ac->fn_free(ac, v_indices);
	ac->fn_free(ac, v_adjacent_infos);
	ac->fn_free(ac, mesh_v_ids);
	ac->fn_free(ac, edge_v_indices_flat);
	ac->fn_free(ac, edge_v_ids_flat);
	ac->fn_free(ac, mesh_edge_ids);
	ac->fn_free(ac, tri_v_indices_flat);
	ac->fn_free(ac, concave_tri_v_ids_flat);
	ac->fn_free(ac, concave_tri_edge_ids_flat);
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
	/* AABB + is flags + v_cnt */
	if (!safe_align(&total_size, BIN_ALIGN_(GeometryAABB_t))) { return 0; }
	if (!safe_add(&total_size, sizeof(mesh->bound_box))) { return 0; }
	if (!safe_align(&total_size, BIN_ALIGN_(unsigned_char))) { return 0; }
	if (!safe_add(&total_size, sizeof(mesh->is_convex) + sizeof(mesh->is_closed))) { return 0; }
	if (!safe_align(&total_size, BIN_ALIGN_(unsigned_int))) { return 0; }
	if (!safe_add(&total_size, sizeof(v_cnt))) { return 0; }

	/* vertices */
	if (!safe_align(&total_size, BIN_ALIGN_(CCTNum_t))) { return 0; }
	if (!safe_add_mul(&total_size, sizeof(mesh->v[0]), v_cnt)) { return 0; }

	/* v_indices_cnt + v_indices + adjacent infos */
	if (!safe_align(&total_size, BIN_ALIGN_(unsigned_int))) { return 0; }
	if (!safe_add(&total_size, sizeof(mesh->v_indices_cnt))) { return 0; }
	if (!safe_add_mul(&total_size, sizeof(mesh->v_indices[0]), mesh->v_indices_cnt)) { return 0; }
	for (i = 0; i < mesh->v_indices_cnt; ++i) {
		total_size = BinarySize_GeometryMeshVertexAdjacentInfo(mesh->v_adjacent_infos + i, total_size);
		if (!total_size) { return 0; }
	}

	/* edges */
	if (!safe_align(&total_size, BIN_ALIGN_(unsigned_int))) { return 0; }
	if (!safe_add(&total_size, sizeof(mesh->edge_cnt))) { return 0; }
	if (!safe_add_mul(&total_size, sizeof(mesh->edge_v_indices[0]), mesh->edge_cnt)) { return 0; }
	if (!safe_add_mul(&total_size, sizeof(mesh->edge_v_ids[0]), mesh->edge_cnt)) { return 0; }
	if (!safe_add_mul(&total_size, sizeof(mesh->edge_adjacent_face_ids[0]), mesh->edge_cnt)) { return 0; }

	/* polygons */
	if (!safe_align(&total_size, BIN_ALIGN_(unsigned_int))) { return 0; }
	if (!safe_add(&total_size, sizeof(mesh->polygons_cnt))) { return 0; }
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		total_size = BinarySize_GeometryMeshFace(mesh->polygons + i, total_size);
		if (!total_size) { return 0; }
	}

	return total_size;
}

size_t mathMeshSaveBinary(const GeometryMesh_t* mesh, void* buffer) {
	size_t off = 0;
	char* p = (char*)buffer;
	unsigned int i, v_cnt = 0;
	if (((size_t)buffer % GEOMETRY_MEMORY_ADDRESS_ALIGN) != 0) {
		return 0;
	}

	pad_skip(p, SIZE_MAX, &off, BIN_ALIGN_(GeometryAABB_t));
	*(GeometryAABB_t*)&p[off] = mesh->bound_box;
	off += sizeof(GeometryAABB_t);
	pad_skip(p, SIZE_MAX, &off, BIN_ALIGN_(unsigned_char));
	write_u8(p, &off, mesh->is_convex);
	write_u8(p, &off, mesh->is_closed);

	for (i = 0; i < mesh->v_indices_cnt; ++i) {
		if (mesh->v_indices[i] >= v_cnt) {
			v_cnt = mesh->v_indices[i] + 1;
		}
	}
	pad_skip(p, SIZE_MAX, &off, BIN_ALIGN_(unsigned_int));
	write_u32(p, &off, v_cnt);
	pad_skip(p, SIZE_MAX, &off, BIN_ALIGN_(CCTNum_t));
	{
		CCTNum_t(*pv)[3] = (CCTNum_t(*)[3])&p[off];
		if (v_cnt > mesh->v_indices_cnt) {
			for (i = 0; i < v_cnt; ++i) {
				pv[i][0] = (CCTNum_t)NAN;
				pv[i][1] = (CCTNum_t)NAN;
				pv[i][2] = (CCTNum_t)NAN;
			}
		}
		for (i = 0; i < mesh->v_indices_cnt; ++i) {
			unsigned int idx = mesh->v_indices[i];
			mathVec3Copy(pv[idx], mesh->v[idx]);
		}
	}
	off += sizeof(CCTNum_t[3]) * (size_t)v_cnt;

	pad_skip(p, SIZE_MAX, &off, BIN_ALIGN_(unsigned_int));
	write_u32(p, &off, mesh->v_indices_cnt);
	for (i = 0; i < mesh->v_indices_cnt; ++i) {
		write_u32(p, &off, mesh->v_indices[i]);
	}
	for (i = 0; i < mesh->v_indices_cnt; ++i) {
		size_t sz = BinarySave_GeometryMeshVertexAdjacentInfo(mesh->v_adjacent_infos + i, p + off);
		off += sz;
	}

	pad_skip(p, SIZE_MAX, &off, BIN_ALIGN_(unsigned_int));
	write_u32(p, &off, mesh->edge_cnt);
	for (i = 0; i < mesh->edge_cnt + mesh->edge_cnt; ++i) {
		write_u32(p, &off, mesh->edge_v_indices_flat[i]);
	}
	for (i = 0; i < mesh->edge_cnt + mesh->edge_cnt; ++i) {
		write_u32(p, &off, mesh->edge_v_ids_flat[i]);
	}
	for (i = 0; i < mesh->edge_cnt + mesh->edge_cnt; ++i) {
		write_u32(p, &off, mesh->edge_adjacent_face_ids_flat[i]);
	}

	pad_skip(p, SIZE_MAX, &off, BIN_ALIGN_(unsigned_int));
	write_u32(p, &off, mesh->polygons_cnt);
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		size_t sz = BinarySave_GeometryMeshFace(mesh->polygons + i, p + off);
		off += sz;
	}
	return off;
}

size_t mathMeshLoadBinary(const void* buffer, size_t len, GeometryMesh_t* mesh, const CCTAllocator_t* ac) {
	size_t off = 0;
	const char* p = (const char*)buffer;
	unsigned int i, v_cnt;
	GeometryMesh_t tmp;
	CCTNum_t (*mesh_v)[3] = NULL;
	unsigned int* mesh_v_indices = NULL;
	GeometryMeshVertexAdjacentInfo_t* mesh_v_adjacent_infos = NULL;
	unsigned int* mesh_edge_v_indices_flat = NULL, *mesh_edge_v_ids_flat = NULL, *mesh_edge_adjacent_face_ids_flat = NULL;
	GeometryPolygon_t* mesh_polygons = NULL;
	if (((size_t)buffer % GEOMETRY_MEMORY_ADDRESS_ALIGN) != 0) {
		return 0;
	}
	if (!ac) {
		ac = CCTAllocator_stdc(NULL);
	}
	
	tmp.allocator_ptr = ac;
	tmp.v_indices_cnt = 0;
	tmp.polygons_cnt = 0;
	if (!pad_skip(p, len, &off, BIN_ALIGN_(GeometryAABB_t))) { goto err; }
	if (sizeof(GeometryAABB_t) > len - off) { goto err; }
	tmp.bound_box = *(const GeometryAABB_t*)&p[off];
	off += sizeof(GeometryAABB_t);

	if (!pad_skip(p, len, &off, BIN_ALIGN_(unsigned_char))) { goto err; }
	if (!read_u8(p, len, &off, &tmp.is_convex)) { goto err; }
	if (!read_u8(p, len, &off, &tmp.is_closed)) { goto err; }

	if (!pad_skip(p, len, &off, BIN_ALIGN_(unsigned_int))) { goto err; }
	if (!read_u32(p, len, &off, &v_cnt)) { goto err; }
	mesh_v = (CCTNum_t(*)[3])ac->fn_malloc(ac, sizeof(CCTNum_t[3]) * v_cnt);
	if (!mesh_v) {
		goto err;
	}
	if (!pad_skip(p, len, &off, BIN_ALIGN_(CCTNum_t))) { goto err; }
	for (i = 0; i < v_cnt; ++i) {
		if (sizeof(CCTNum_t[3]) > len - off) { goto err; }
		mathVec3Copy(mesh_v[i], (const CCTNum_t*)&p[off]);
		off += sizeof(CCTNum_t[3]);
	}

	if (!pad_skip(p, len, &off, BIN_ALIGN_(unsigned_int))) { goto err; }
	if (!read_u32(p, len, &off, &tmp.v_indices_cnt)) { goto err; }
	mesh_v_indices = (unsigned int*)ac->fn_malloc(ac, sizeof(unsigned int) * tmp.v_indices_cnt);
	if (!mesh_v_indices) {
		goto err;
	}
	for (i = 0; i < tmp.v_indices_cnt; ++i) {
		if (!read_u32(p, len, &off, &mesh_v_indices[i])) { goto err; }
	}
	mesh_v_adjacent_infos = (GeometryMeshVertexAdjacentInfo_t*)ac->fn_malloc(ac, sizeof(GeometryMeshVertexAdjacentInfo_t) * tmp.v_indices_cnt);
	if (!mesh_v_adjacent_infos) {
		goto err;
	}
	for (i = 0; i < tmp.v_indices_cnt; ++i) {
		size_t sz = BinaryLoad_GeometryMeshVertexAdjacentInfo(p + off, len - off, mesh_v_adjacent_infos + i, ac);
		if (!sz) {
			tmp.v_indices_cnt = i;
			goto err;
		}
		off += sz;
	}

	if (!pad_skip(p, len, &off, BIN_ALIGN_(unsigned_int))) { goto err; }
	if (!read_u32(p, len, &off, &tmp.edge_cnt)) { goto err; }
	mesh_edge_v_indices_flat = (unsigned int*)ac->fn_malloc(ac, sizeof(unsigned int) * tmp.edge_cnt * 2);
	if (!mesh_edge_v_indices_flat) {
		goto err;
	}
	mesh_edge_v_ids_flat = (unsigned int*)ac->fn_malloc(ac, sizeof(unsigned int) * tmp.edge_cnt * 2);
	if (!mesh_edge_v_ids_flat) {
		goto err;
	}
	mesh_edge_adjacent_face_ids_flat = (unsigned int*)ac->fn_malloc(ac, sizeof(unsigned int) * tmp.edge_cnt * 2);
	if (!mesh_edge_adjacent_face_ids_flat) {
		goto err;
	}
	for (i = 0; i < tmp.edge_cnt * 2; ++i) {
		if (!read_u32(p, len, &off, &mesh_edge_v_indices_flat[i])) { goto err; }
	}
	for (i = 0; i < tmp.edge_cnt * 2; ++i) {
		if (!read_u32(p, len, &off, &mesh_edge_v_ids_flat[i])) { goto err; }
	}
	for (i = 0; i < tmp.edge_cnt * 2; ++i) {
		if (!read_u32(p, len, &off, &mesh_edge_adjacent_face_ids_flat[i])) { goto err; }
	}

	if (!pad_skip(p, len, &off, BIN_ALIGN_(unsigned_int))) { goto err; }
	if (!read_u32(p, len, &off, &tmp.polygons_cnt)) { goto err; }
	mesh_polygons = (GeometryPolygon_t*)ac->fn_malloc(ac, sizeof(GeometryPolygon_t) * tmp.polygons_cnt);
	if (!mesh_polygons) {
		goto err;
	}
	for (i = 0; i < tmp.polygons_cnt; ++i) {
		GeometryPolygon_t* face = mesh_polygons + i;
		size_t sz = BinaryLoad_GeometryMeshFace(p + off, len - off, face, ac);
		if (!sz) {
			tmp.polygons_cnt = i;
			goto err;
		}
		off += sz;
		face->v = mesh_v;
	}

	tmp.v = mesh_v;
	tmp.v_indices = mesh_v_indices;
	tmp.v_adjacent_infos = mesh_v_adjacent_infos;
	tmp.edge_v_indices_flat = mesh_edge_v_indices_flat;
	tmp.edge_v_ids_flat = mesh_edge_v_ids_flat;
	tmp.edge_adjacent_face_ids_flat = mesh_edge_adjacent_face_ids_flat;
	tmp.polygons = mesh_polygons;
	tmp._is_buffer_view = 0;
	*mesh = tmp;
	return off;
err:
	if (mesh_polygons) {
		for (i = 0; i < tmp.polygons_cnt; ++i) {
			Polygon_ClearWithoutVertices(mesh_polygons + i, ac);
		}
		ac->fn_free(ac, mesh_polygons);
	}
	if (mesh_v_adjacent_infos) {
		for (i = 0; i < tmp.v_indices_cnt; ++i) {
			free_data_mesh_vertex_adjacent_info(mesh_v_adjacent_infos + i, ac);
		}
		ac->fn_free(ac, mesh_v_adjacent_infos);
	}
	ac->fn_free(ac, mesh_edge_adjacent_face_ids_flat);
	ac->fn_free(ac, mesh_edge_v_ids_flat);
	ac->fn_free(ac, mesh_edge_v_indices_flat);
	ac->fn_free(ac, mesh_v_indices);
	ac->fn_free(ac, mesh_v);
	return 0;
}

size_t mathMeshLoadBinaryView(const void* buffer, size_t len, GeometryMesh_t* mesh, const CCTAllocator_t* ac) {
	size_t off = 0;
	const char* p = (const char*)buffer;
	unsigned int i, v_cnt;
	GeometryMesh_t tmp = { 0 };
	GeometryMeshVertexAdjacentInfo_t* mesh_v_adjacent_infos = NULL;
	GeometryPolygon_t* mesh_polygons = NULL;
	if (((size_t)buffer % GEOMETRY_MEMORY_ADDRESS_ALIGN) != 0) {
		return 0;
	}
	if (!ac) {
		ac = CCTAllocator_stdc(NULL);
	}
	tmp.allocator_ptr = ac;

	/* AABB */
	if (!pad_skip(p, len, &off, BIN_ALIGN_(GeometryAABB_t))) { goto err; }
	if (sizeof(GeometryAABB_t) > len - off) { goto err; }
	tmp.bound_box = *(const GeometryAABB_t*)&p[off];
	off += sizeof(GeometryAABB_t);

	if (!pad_skip(p, len, &off, BIN_ALIGN_(unsigned_char))) { goto err; }
	if (!read_u8(p, len, &off, &tmp.is_convex)) { goto err; }
	if (!read_u8(p, len, &off, &tmp.is_closed)) { goto err; }

	/* vertices */
	if (!pad_skip(p, len, &off, BIN_ALIGN_(unsigned_int))) { goto err; }
	if (!read_u32(p, len, &off, &v_cnt)) { goto err; }
	if (!pad_skip(p, len, &off, BIN_ALIGN_(CCTNum_t))) { goto err; }
	if (sizeof(CCTNum_t[3]) * (size_t)v_cnt > len - off) { goto err; }
	tmp.v = (CCTNum_t(*)[3])&p[off];
	off += sizeof(CCTNum_t[3]) * (size_t)v_cnt;

	/* v_indices */
	if (!pad_skip(p, len, &off, BIN_ALIGN_(unsigned_int))) { goto err; }
	if (!read_u32(p, len, &off, &tmp.v_indices_cnt)) { goto err; }
	if (sizeof(unsigned int) * (size_t)tmp.v_indices_cnt > len - off) { goto err; }
	tmp.v_indices = (const unsigned int*)&p[off];
	off += sizeof(unsigned int) * (size_t)tmp.v_indices_cnt;

	/* v_adjacent_infos (struct headers allocated; arrays point into buffer) */
	mesh_v_adjacent_infos = (GeometryMeshVertexAdjacentInfo_t*)ac->fn_malloc(ac, sizeof(GeometryMeshVertexAdjacentInfo_t) * (size_t)tmp.v_indices_cnt);
	if (!mesh_v_adjacent_infos) { goto err; }
	tmp.v_adjacent_infos = mesh_v_adjacent_infos;
	for (i = 0; i < tmp.v_indices_cnt; ++i) {
		GeometryMeshVertexAdjacentInfo_t* info = mesh_v_adjacent_infos + i;
		unsigned int v_arr_cnt, e_arr_cnt, f_arr_cnt;

		/* v_cnt */
		if (!pad_skip(p, len, &off, BIN_ALIGN_(unsigned_int))) { goto err; }
		if (!read_u32(p, len, &off, &v_arr_cnt)) { goto err; }
		if (sizeof(unsigned int) * (size_t)v_arr_cnt > len - off) { goto err; }
		info->v_cnt = v_arr_cnt;
		info->v_ids = (const unsigned int*)&p[off];
		off += sizeof(unsigned int) * (size_t)v_arr_cnt;

		/* edge_cnt */
		if (!pad_skip(p, len, &off, BIN_ALIGN_(unsigned_int))) { goto err; }
		if (!read_u32(p, len, &off, &e_arr_cnt)) { goto err; }
		if (sizeof(unsigned int) * (size_t)e_arr_cnt > len - off) { goto err; }
		info->edge_cnt = e_arr_cnt;
		info->edge_ids = (const unsigned int*)&p[off];
		off += sizeof(unsigned int) * (size_t)e_arr_cnt;

		/* face_cnt */
		if (!pad_skip(p, len, &off, BIN_ALIGN_(unsigned_int))) { goto err; }
		if (!read_u32(p, len, &off, &f_arr_cnt)) { goto err; }
		if (sizeof(unsigned int) * (size_t)f_arr_cnt > len - off) { goto err; }
		info->face_cnt = f_arr_cnt;
		info->face_ids = (const unsigned int*)&p[off];
		off += sizeof(unsigned int) * (size_t)f_arr_cnt;
	}

	/* edges */
	if (!pad_skip(p, len, &off, BIN_ALIGN_(unsigned_int))) { goto err; }
	if (!read_u32(p, len, &off, &tmp.edge_cnt)) { goto err; }
	if (sizeof(unsigned int) * (size_t)tmp.edge_cnt * 2 > len - off) { goto err; }
	tmp.edge_v_indices_flat = (const unsigned int*)&p[off];
	off += sizeof(unsigned int) * (size_t)tmp.edge_cnt * 2;
	if (sizeof(unsigned int) * (size_t)tmp.edge_cnt * 2 > len - off) { goto err; }
	tmp.edge_v_ids_flat = (const unsigned int*)&p[off];
	off += sizeof(unsigned int) * (size_t)tmp.edge_cnt * 2;
	if (sizeof(unsigned int) * (size_t)tmp.edge_cnt * 2 > len - off) { goto err; }
	tmp.edge_adjacent_face_ids_flat = (const unsigned int*)&p[off];
	off += sizeof(unsigned int) * (size_t)tmp.edge_cnt * 2;

	/* polygons (struct headers allocated; arrays point into buffer) */
	if (!pad_skip(p, len, &off, BIN_ALIGN_(unsigned_int))) { goto err; }
	if (!read_u32(p, len, &off, &tmp.polygons_cnt)) { goto err; }
	mesh_polygons = (GeometryPolygon_t*)ac->fn_malloc(ac, sizeof(GeometryPolygon_t) * (size_t)tmp.polygons_cnt);
	if (!mesh_polygons) { goto err; }
	tmp.polygons = mesh_polygons;
	for (i = 0; i < tmp.polygons_cnt; ++i) {
		GeometryPolygon_t* face = mesh_polygons + i;
		face->v = tmp.v;
		/* center */
		if (!pad_skip(p, len, &off, BIN_ALIGN_(CCTNum_t))) { goto err; }
		if (sizeof(CCTNum_t[3]) > len - off) { goto err; }
		mathVec3Copy(face->center, (const CCTNum_t*)&p[off]);
		off += sizeof(CCTNum_t[3]);
		/* normal */
		if (!pad_skip(p, len, &off, BIN_ALIGN_(CCTNum_t))) { goto err; }
		if (sizeof(CCTNum_t[3]) > len - off) { goto err; }
		mathVec3Copy(face->normal, (const CCTNum_t*)&p[off]);
		off += sizeof(CCTNum_t[3]);
		/* is_convex */
		if (!pad_skip(p, len, &off, BIN_ALIGN_(short))) { goto err; }
		if (!read_i16(p, len, &off, &face->is_convex)) { goto err; }

		/* v_indices_cnt */
		if (!pad_skip(p, len, &off, BIN_ALIGN_(unsigned_int))) { goto err; }
		if (!read_u32(p, len, &off, &face->v_indices_cnt)) { goto err; }
		if (sizeof(unsigned int) * (size_t)face->v_indices_cnt > len - off) { goto err; }
		face->v_indices = (const unsigned int*)&p[off];
		off += sizeof(unsigned int) * (size_t)face->v_indices_cnt;
		if (sizeof(unsigned int) * (size_t)face->v_indices_cnt > len - off) { goto err; }
		face->mesh_v_ids = (const unsigned int*)&p[off];
		off += sizeof(unsigned int) * (size_t)face->v_indices_cnt;
		if (sizeof(GeometryPolygonVertexAdjacentInfo_t) * (size_t)face->v_indices_cnt > len - off) { goto err; }
		face->v_adjacent_infos = (const GeometryPolygonVertexAdjacentInfo_t*)&p[off];
		off += sizeof(GeometryPolygonVertexAdjacentInfo_t) * (size_t)face->v_indices_cnt;

		/* edge_cnt */
		if (!pad_skip(p, len, &off, BIN_ALIGN_(unsigned_int))) { goto err; }
		if (!read_u32(p, len, &off, &face->edge_cnt)) { goto err; }
		if (sizeof(unsigned int) * (size_t)face->edge_cnt * 2 > len - off) { goto err; }
		face->edge_v_indices_flat = (const unsigned int*)&p[off];
		off += sizeof(unsigned int) * (size_t)face->edge_cnt * 2;
		if (sizeof(unsigned int) * (size_t)face->edge_cnt * 2 > len - off) { goto err; }
		face->edge_v_ids_flat = (const unsigned int*)&p[off];
		off += sizeof(unsigned int) * (size_t)face->edge_cnt * 2;
		if (sizeof(unsigned int) * (size_t)face->edge_cnt > len - off) { goto err; }
		face->mesh_edge_ids = (const unsigned int*)&p[off];
		off += sizeof(unsigned int) * (size_t)face->edge_cnt;

		/* tri_cnt */
		if (!pad_skip(p, len, &off, BIN_ALIGN_(unsigned_int))) { goto err; }
		if (!read_u32(p, len, &off, &face->tri_cnt)) { goto err; }
		if (sizeof(unsigned int) * (size_t)face->tri_cnt * 3 > len - off) { goto err; }
		face->tri_v_indices_flat = (const unsigned int*)&p[off];
		off += sizeof(unsigned int) * (size_t)face->tri_cnt * 3;

		if (face->is_convex) {
			face->concave_tri_v_ids_flat = NULL;
			face->concave_tri_edge_ids_flat = NULL;
		}
		else {
			if (sizeof(unsigned int) * (size_t)face->tri_cnt * 3 > len - off) { goto err; }
			face->concave_tri_v_ids_flat = (const unsigned int*)&p[off];
			off += sizeof(unsigned int) * (size_t)face->tri_cnt * 3;
			if (sizeof(unsigned int) * (size_t)face->tri_cnt * 3 > len - off) { goto err; }
			face->concave_tri_edge_ids_flat = (const unsigned int*)&p[off];
			off += sizeof(unsigned int) * (size_t)face->tri_cnt * 3;
		}
	}

	tmp._is_buffer_view = 1;
	*mesh = tmp;
	return off;
err:
	ac->fn_free(ac, mesh_polygons);
	ac->fn_free(ac, mesh_v_adjacent_infos);
	return 0;
}

#ifdef __cplusplus
}
#endif



