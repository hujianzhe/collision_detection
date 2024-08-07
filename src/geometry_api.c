//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/math_quat.h"
#include "../inc/vertex.h"
#include "../inc/aabb.h"
#include "../inc/obb.h"
#include "../inc/polygon.h"
#include "../inc/mesh.h"
#include "../inc/geometry_api.h"
#include <stddef.h>

static void point_rotate(CCTNum_t p[3], const CCTNum_t mark_pos[3], const CCTNum_t q[4]) {
	CCTNum_t v[3];
	mathVec3Sub(v, p, mark_pos);
	mathQuatMulVec3(v, q, v);
	mathVec3Add(p, mark_pos, v);
}

static void indices_rotate(CCTNum_t(*p)[3], const unsigned int* indices, unsigned int indices_cnt, const CCTNum_t mark_pos[3], const CCTNum_t q[4]) {
	unsigned int i;
	for (i = 0; i < indices_cnt; ++i) {
		point_rotate(p[indices[i]], mark_pos, q);
	}
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
extern "C" {
#endif

size_t mathGeometrySize(int geo_type) {
	static const size_t s_geometry_size[] = {
		0,
		sizeof(CCTNum_t[3]),
		sizeof(GeometrySegment_t),
		sizeof(GeometryPlane_t),
		sizeof(GeometrySphere_t),
		sizeof(GeometryAABB_t),
		sizeof(GeometryOBB_t),
		sizeof(GeometryPolygon_t),
		sizeof(GeometryMesh_t),
	};

	if (((size_t)geo_type) >= sizeof(s_geometry_size) / sizeof(s_geometry_size[0])) {
		return 0;
	}
	return s_geometry_size[(size_t)geo_type];
}

unsigned char* mathGeometryClone(unsigned char* dst, const unsigned char* src_geo_data, int src_geo_type) {
	switch (src_geo_type) {
		case GEOMETRY_BODY_POINT:
		{
			mathVec3Copy((CCTNum_t*)dst, (const CCTNum_t*)src_geo_data);
			break;
		}
		case GEOMETRY_BODY_SEGMENT:
		{
			*(GeometrySegment_t*)dst = *(const GeometrySegment_t*)src_geo_data;
			break;
		}
		case GEOMETRY_BODY_PLANE:
		{
			*(GeometryPlane_t*)dst = *(const GeometryPlane_t*)src_geo_data;
			break;
		}
		case GEOMETRY_BODY_SPHERE:
		{
			*(GeometrySphere_t*)dst = *(const GeometrySphere_t*)src_geo_data;
			break;
		}
		case GEOMETRY_BODY_AABB:
		{
			*(GeometryAABB_t*)dst = *(const GeometryAABB_t*)src_geo_data;
			break;
		}
		case GEOMETRY_BODY_OBB:
		{
			*(GeometryOBB_t*)dst = *(const GeometryOBB_t*)src_geo_data;
			break;
		}
		case GEOMETRY_BODY_POLYGON:
		{
			if (!mathPolygonDeepCopy((GeometryPolygon_t*)dst, (const GeometryPolygon_t*)src_geo_data)) {
				return NULL;
			}
			break;
		}
		case GEOMETRY_BODY_CONVEX_MESH:
		{
			if (!mathMeshDeepCopy((GeometryMesh_t*)dst, (const GeometryMesh_t*)src_geo_data)) {
				return NULL;
			}
			break;
		}
		default:
		{
			return NULL;
		}
	}
	return dst;
}

void mathGeometryFree(void* geo_data, int geo_type) {
	if (GEOMETRY_BODY_CONVEX_MESH == geo_type) {
		mathMeshFreeCookingData((GeometryMesh_t*)geo_data);
	}
	else if (GEOMETRY_BODY_POLYGON == geo_type) {
		mathPolygonFreeCookingData((GeometryPolygon_t*)geo_data);
	}
}

void mathGeometryFreeBody(GeometryBody_t* b) {
	if (!b) {
		return;
	}
	mathGeometryFree(&b->data, b->type);
	b->type = 0;
}

void mathGeometryFreeRef(GeometryBodyRef_t* b) {
	if (!b) {
		return;
	}
	mathGeometryFree(b->data, b->type);
	b->data = NULL;
	b->type = 0;
}

const CCTNum_t* mathGeometryGetPosition(const GeometryBodyRef_t* b, CCTNum_t v[3]) {
	const CCTNum_t* ptr_v;
	switch (b->type) {
		case GEOMETRY_BODY_POINT:
		{
			ptr_v = b->point;
			break;
		}
		case GEOMETRY_BODY_SEGMENT:
		{
			ptr_v = b->segment->v[0];
			break;
		}
		case GEOMETRY_BODY_PLANE:
		{
			ptr_v = b->plane->v;
			break;
		}
		case GEOMETRY_BODY_SPHERE:
		{
			ptr_v = b->sphere->o;
			break;
		}
		case GEOMETRY_BODY_AABB:
		{
			ptr_v = b->aabb->o;
			break;
		}
		case GEOMETRY_BODY_OBB:
		{
			ptr_v = b->obb->o;
			break;
		}
		case GEOMETRY_BODY_POLYGON:
		{
			return NULL;
		}
		case GEOMETRY_BODY_CONVEX_MESH:
		{
			ptr_v = b->mesh->bound_box.o;
			break;
		}
		default:
			return NULL;
	}
	if (v) {
		return mathVec3Copy(v, ptr_v);
	}
	return ptr_v;
}

void mathGeometrySetPosition(GeometryBodyRef_t* b, const CCTNum_t v[3]) {
	switch (b->type) {
		case GEOMETRY_BODY_POINT:
		{
			mathVec3Copy(b->point, v);
			return;
		}
		case GEOMETRY_BODY_SEGMENT:
		{
			GeometrySegment_t* segment = b->segment;
			CCTNum_t delta[3];
			mathVec3Sub(delta, v, segment->v[0]);
			mathVec3Copy(segment->v[0], v);
			mathVec3Add(segment->v[1], segment->v[1], delta);
			return;
		}
		case GEOMETRY_BODY_PLANE:
		{
			mathVec3Copy(b->plane->v, v);
			return;
		}
		case GEOMETRY_BODY_SPHERE:
		{
			mathVec3Copy(b->sphere->o, v);
			return;
		}
		case GEOMETRY_BODY_AABB:
		{
			mathVec3Copy(b->aabb->o, v);
			return;
		}
		case GEOMETRY_BODY_OBB:
		{
			mathVec3Copy(b->obb->o, v);
			return;
		}
		case GEOMETRY_BODY_POLYGON:
		{
			/* TODO */
			return;
		}
		case GEOMETRY_BODY_CONVEX_MESH:
		{
			unsigned int i;
			GeometryMesh_t* mesh = b->mesh;
			CCTNum_t delta[3];
			mathVec3Sub(delta, v, mesh->bound_box.o);
			for (i = 0; i < mesh->v_indices_cnt; ++i) {
				CCTNum_t* p = mesh->v[mesh->v_indices[i]];
				mathVec3Add(p, p, delta);
			}
			mathVec3Add(mesh->o, mesh->o, delta);
			mathVec3Copy(mesh->bound_box.o, v);
			return;
		}
	}
}

GeometryAABB_t* mathGeometryBoundingBox(const GeometryBodyRef_t* b, GeometryAABB_t* aabb) {
	switch (b->type) {
		case GEOMETRY_BODY_POINT:
		{
			mathVec3Copy(aabb->o, b->point);
			aabb->half[0] = GEOMETRY_BODY_BOX_MIN_HALF;
			aabb->half[1] = GEOMETRY_BODY_BOX_MIN_HALF;
			aabb->half[2] = GEOMETRY_BODY_BOX_MIN_HALF;
			break;
		}
		case GEOMETRY_BODY_SEGMENT:
		{
			mathAABBFromTwoVertice(b->segment->v[0], b->segment->v[1], aabb->o, aabb->half);
			break;
		}
		case GEOMETRY_BODY_AABB:
		{
			*aabb = *(b->aabb);
			break;
		}
		case GEOMETRY_BODY_SPHERE:
		{
			CCTNum_t r = b->sphere->radius;
			mathVec3Copy(aabb->o, b->sphere->o);
			mathVec3Set(aabb->half, r, r, r);
			break;
		}
		case GEOMETRY_BODY_OBB:
		{
			mathOBBToAABB(b->obb, aabb->o, aabb->half);
			break;
		}
		case GEOMETRY_BODY_POLYGON:
		{
			CCTNum_t min_v[3], max_v[3];
			const GeometryPolygon_t* polygon = b->polygon;
			if (!mathVertexIndicesFindMinMaxXYZ((const CCTNum_t(*)[3])polygon->v, polygon->v_indices, polygon->v_indices_cnt, min_v, max_v)) {
				return NULL;
			}
			mathAABBFromTwoVertice(min_v, max_v, aabb->o, aabb->half);
			break;
		}
		case GEOMETRY_BODY_CONVEX_MESH:
		{
			*aabb = b->mesh->bound_box;
			break;
		}
		default:
		{
			return NULL;
		}
	}
	return aabb;
}

int mathGeometryRotate(GeometryBodyRef_t* b, const CCTNum_t q[4]) {
	switch (b->type) {
		case GEOMETRY_BODY_SEGMENT:
		{
			CCTNum_t o[3];
			mathVec3Add(o, b->segment->v[0], b->segment->v[1]);
			mathVec3MultiplyScalar(o, o, CCTNum(0.5));
			point_rotate(b->segment->v[0], o, q);
			point_rotate(b->segment->v[1], o, q);
			break;
		}
		case GEOMETRY_BODY_PLANE:
		{
			GeometryPlane_t* plane = b->plane;
			mathQuatMulVec3(plane->normal, q, plane->normal);
			break;
		}
		case GEOMETRY_BODY_AABB:
		{
			GeometryAABB_t* aabb = b->aabb;
			CCTNum_t axis[3], new_axis[3];
			/* check rotate by X axis ??? */
			mathVec3Set(axis, CCTNums_3(1.0, 0.0, 0.0));
			mathQuatMulVec3(new_axis, q, axis);
			if (mathVec3Equal(new_axis, axis)) {
				CCTNum_t dot;
				mathVec3Set(axis, CCTNums_3(0.0, 1.0, 0.0));
				mathQuatMulVec3(new_axis, q, axis);
				dot = mathVec3Dot(new_axis, axis);
				if (dot <= CCT_EPSILON && dot >= CCT_EPSILON_NEGATE) { /* rotate PI/2 */
					dot = aabb->half[1];
					aabb->half[1] = aabb->half[2];
					aabb->half[2] = dot;
					break;
				}
				else if (dot <= CCTNum(1.0) + CCT_EPSILON && dot >= CCTNum(1.0) - CCT_EPSILON) { /* rotate PI */
					break;
				}
				return 0;
			}
			/* check rotate by Y axis ??? */
			mathVec3Set(axis, CCTNums_3(0.0, 1.0, 0.0));
			mathQuatMulVec3(new_axis, q, axis);
			if (mathVec3Equal(new_axis, axis)) {
				CCTNum_t dot;
				mathVec3Set(axis, CCTNums_3(1.0, 0.0, 0.0));
				mathQuatMulVec3(new_axis, q, axis);
				dot = mathVec3Dot(new_axis, axis);
				if (dot <= CCT_EPSILON && dot >= CCT_EPSILON_NEGATE) { /* rotate PI/2 */
					dot = aabb->half[0];
					aabb->half[0] = aabb->half[2];
					aabb->half[2] = dot;
					break;
				}
				else if (dot <= CCTNum(1.0) + CCT_EPSILON && dot >= CCTNum(1.0) - CCT_EPSILON) { /* rotate PI */
					break;
				}
				return 0;
			}
			/* check rotate by Z axis ??? */
			mathVec3Set(axis, CCTNums_3(0.0, 0.0, 1.0));
			mathQuatMulVec3(new_axis, q, axis);
			if (mathVec3Equal(new_axis, axis)) {
				CCTNum_t dot;
				mathVec3Set(axis, CCTNums_3(1.0, 0.0, 0.0));
				mathQuatMulVec3(new_axis, q, axis);
				dot = mathVec3Dot(new_axis, axis);
				if (dot <= CCT_EPSILON && dot >= CCT_EPSILON_NEGATE) { /* rotate PI/2 */
					dot = aabb->half[1];
					aabb->half[1] = aabb->half[0];
					aabb->half[0] = dot;
					break;
				}
				else if (dot <= CCTNum(1.0) + CCT_EPSILON && dot >= CCTNum(1.0) - CCT_EPSILON) { /* rotate PI */
					break;
				}
				return 0;
			}
			/* not allow rotate by other axies */
			return 0;
		}
		case GEOMETRY_BODY_OBB:
		{
			GeometryOBB_t* obb = b->obb;
			mathQuatMulVec3(obb->axis[0], q, obb->axis[0]);
			mathQuatMulVec3(obb->axis[1], q, obb->axis[1]);
			mathQuatMulVec3(obb->axis[2], q, obb->axis[2]);
			break;
		}
		case GEOMETRY_BODY_POLYGON:
		{
			GeometryPolygon_t* polygon = b->polygon;
			indices_rotate(polygon->v, polygon->v_indices, polygon->v_indices_cnt, polygon->o, q);
			mathQuatMulVec3(polygon->normal, q, polygon->normal);
			break;
		}
		case GEOMETRY_BODY_CONVEX_MESH:
		{
			GeometryMesh_t* mesh = b->mesh;
			unsigned int i;
			CCTNum_t min_xyz[3], max_xyz[3];

			indices_rotate(mesh->v, mesh->v_indices, mesh->v_indices_cnt, mesh->o, q);
			for (i = 0; i < mesh->polygons_cnt; ++i) {
				GeometryPolygon_t* polygon = mesh->polygons + i;
				if (polygon->v != mesh->v) {
					indices_rotate(polygon->v, polygon->v_indices, polygon->v_indices_cnt, mesh->o, q);
				}
				mathQuatMulVec3(polygon->normal, q, polygon->normal);
			}
			mathVertexIndicesFindMinMaxXYZ((const CCTNum_t(*)[3])mesh->v, mesh->v_indices, mesh->v_indices_cnt, min_xyz, max_xyz);
			mathAABBFromTwoVertice(min_xyz, max_xyz, mesh->bound_box.o, mesh->bound_box.half);
			break;
		}
		default:
			return 0;
	}
	return 1;
}

int mathGeometryRotateAxisRadian(GeometryBodyRef_t* b, const CCTNum_t axis[3], CCTNum_t radian) {
	CCTNum_t q[4];
	if (GEOMETRY_BODY_SPHERE == b->type) {
		return 1;
	}
	mathQuatFromAxisRadian(q, axis, radian);
	return mathGeometryRotate(b, q);
}

#ifdef __cplusplus
}
#endif
