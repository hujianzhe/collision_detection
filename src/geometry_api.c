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
#include "../inc/capsule.h"
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
		sizeof(GeometryCapsule_t)
	};

	if (((size_t)geo_type) >= sizeof(s_geometry_size) / sizeof(s_geometry_size[0])) {
		return 0;
	}
	return s_geometry_size[(size_t)geo_type];
}

unsigned char* mathGeometryClone(unsigned char* dst, int* dst_type, const unsigned char* src_geo_data, int src_geo_type) {
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
		case GEOMETRY_BODY_CAPSULE:
		{
			*(GeometryCapsule_t*)dst = *(const GeometryCapsule_t*)src_geo_data;
			break;
		}
		default:
		{
			return NULL;
		}
	}
	if (dst_type) {
		*dst_type = src_geo_type;
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

const CCTNum_t* mathGeometryGetPosition(const unsigned char* geo_data, int geo_type, CCTNum_t v[3]) {
	const CCTNum_t* ptr_v;
	switch (geo_type) {
		case GEOMETRY_BODY_POINT:
		{
			ptr_v = (const CCTNum_t*)geo_data;
			break;
		}
		case GEOMETRY_BODY_SEGMENT:
		{
			ptr_v = ((const GeometrySegment_t*)geo_data)->v[0];
			break;
		}
		case GEOMETRY_BODY_PLANE:
		{
			ptr_v = ((const GeometryPlane_t*)geo_data)->v;
			break;
		}
		case GEOMETRY_BODY_SPHERE:
		{
			ptr_v = ((const GeometrySphere_t*)geo_data)->o;
			break;
		}
		case GEOMETRY_BODY_AABB:
		{
			ptr_v = ((const GeometryAABB_t*)geo_data)->o;
			break;
		}
		case GEOMETRY_BODY_OBB:
		{
			ptr_v = ((const GeometryOBB_t*)geo_data)->o;
			break;
		}
		case GEOMETRY_BODY_POLYGON:
		{
			ptr_v = ((const GeometryPolygon_t*)geo_data)->center;
			break;
		}
		case GEOMETRY_BODY_CONVEX_MESH:
		{
			ptr_v = ((const GeometryMesh_t*)geo_data)->bound_box.o;
			break;
		}
		case GEOMETRY_BODY_CAPSULE:
		{
			ptr_v = ((const GeometryCapsule_t*)geo_data)->o;
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

void mathGeometrySetPosition(unsigned char* geo_data, int geo_type, const CCTNum_t v[3]) {
	switch (geo_type) {
		case GEOMETRY_BODY_POINT:
		{
			mathVec3Copy((CCTNum_t*)geo_data, v);
			return;
		}
		case GEOMETRY_BODY_SEGMENT:
		{
			GeometrySegment_t* segment = (GeometrySegment_t*)geo_data;
			CCTNum_t delta[3];
			mathVec3Sub(delta, v, segment->v[0]);
			mathVec3Copy(segment->v[0], v);
			mathVec3Add(segment->v[1], segment->v[1], delta);
			return;
		}
		case GEOMETRY_BODY_PLANE:
		{
			GeometryPlane_t* plane = (GeometryPlane_t*)geo_data;
			mathVec3Copy(plane->v, v);
			return;
		}
		case GEOMETRY_BODY_SPHERE:
		{
			GeometrySphere_t* sphere = (GeometrySphere_t*)geo_data;
			mathVec3Copy(sphere->o, v);
			return;
		}
		case GEOMETRY_BODY_AABB:
		{
			GeometryAABB_t* aabb = (GeometryAABB_t*)geo_data;
			mathVec3Copy(aabb->o, v);
			return;
		}
		case GEOMETRY_BODY_OBB:
		{
			GeometryOBB_t* obb = (GeometryOBB_t*)geo_data;
			mathVec3Copy(obb->o, v);
			return;
		}
		case GEOMETRY_BODY_POLYGON:
		{
			unsigned int i;
			GeometryPolygon_t* polygon = (GeometryPolygon_t*)geo_data;
			CCTNum_t delta[3];
			mathVec3Sub(delta, v, polygon->center);
			for (i = 0; i < polygon->v_indices_cnt; ++i) {
				CCTNum_t* p = polygon->v[polygon->v_indices[i]];
				mathVec3Add(p, p, delta);
			}
			mathVec3Add(polygon->o, polygon->o, delta);
			mathVec3Copy(polygon->center, v);
			return;
		}
		case GEOMETRY_BODY_CONVEX_MESH:
		{
			unsigned int i;
			GeometryMesh_t* mesh = (GeometryMesh_t*)geo_data;
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
		case GEOMETRY_BODY_CAPSULE:
		{
			GeometryCapsule_t* capsule = (GeometryCapsule_t*)geo_data;
			mathVec3Copy(capsule->o, v);
			return;
		}
	}
}

GeometryAABB_t* mathGeometryBoundingBox(const unsigned char* geo_data, int geo_type, GeometryAABB_t* aabb) {
	switch (geo_type) {
		case GEOMETRY_BODY_POINT:
		{
			const CCTNum_t* point = (const CCTNum_t*)geo_data;
			mathVec3Copy(aabb->o, point);
			aabb->half[0] = GEOMETRY_BODY_BOX_MIN_HALF;
			aabb->half[1] = GEOMETRY_BODY_BOX_MIN_HALF;
			aabb->half[2] = GEOMETRY_BODY_BOX_MIN_HALF;
			break;
		}
		case GEOMETRY_BODY_SEGMENT:
		{
			const GeometrySegment_t* segment = (const GeometrySegment_t*)geo_data;
			mathAABBFromTwoVertice(segment->v[0], segment->v[1], aabb->o, aabb->half);
			break;
		}
		case GEOMETRY_BODY_AABB:
		{
			*aabb = *(const GeometryAABB_t*)geo_data;
			break;
		}
		case GEOMETRY_BODY_SPHERE:
		{
			const GeometrySphere_t* sphere = (const GeometrySphere_t*)geo_data;
			mathVec3Copy(aabb->o, sphere->o);
			mathVec3Set(aabb->half, sphere->radius, sphere->radius, sphere->radius);
			break;
		}
		case GEOMETRY_BODY_OBB:
		{
			mathOBBToAABB((const GeometryOBB_t*)geo_data, aabb->o, aabb->half);
			break;
		}
		case GEOMETRY_BODY_POLYGON:
		{
			CCTNum_t min_v[3], max_v[3];
			const GeometryPolygon_t* polygon = (const GeometryPolygon_t*)geo_data;
			if (!mathVertexIndicesFindMinMaxXYZ((const CCTNum_t(*)[3])polygon->v, polygon->v_indices, polygon->v_indices_cnt, min_v, max_v)) {
				return NULL;
			}
			mathAABBFromTwoVertice(min_v, max_v, aabb->o, aabb->half);
			break;
		}
		case GEOMETRY_BODY_CONVEX_MESH:
		{
			*aabb = ((const GeometryMesh_t*)geo_data)->bound_box;
			break;
		}
		case GEOMETRY_BODY_CAPSULE:
		{
			CCTNum_t min_v[3], max_v[3];
			const GeometryCapsule_t* capsule = (const GeometryCapsule_t*)geo_data;
			mathCapsuleFindMaxMinXYZ(capsule, min_v, max_v);
			mathAABBFromTwoVertice(min_v, max_v, aabb->o, aabb->half);
			break;
		}
		default:
		{
			return NULL;
		}
	}
	return aabb;
}

int mathGeometryRotate(unsigned char* geo_data, int geo_type, const CCTNum_t q[4]) {
	switch (geo_type) {
		case GEOMETRY_BODY_SEGMENT:
		{
			GeometrySegment_t* segment = (GeometrySegment_t*)geo_data;
			point_rotate(segment->v[1], segment->v[0], q);
			break;
		}
		case GEOMETRY_BODY_PLANE:
		{
			GeometryPlane_t* plane = (GeometryPlane_t*)geo_data;
			mathQuatMulVec3(plane->normal, q, plane->normal);
			break;
		}
		case GEOMETRY_BODY_AABB:
		{
			GeometryAABB_t* aabb = (GeometryAABB_t*)geo_data;
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
			GeometryOBB_t* obb = (GeometryOBB_t*)geo_data;
			mathQuatMulVec3(obb->axis[0], q, obb->axis[0]);
			mathQuatMulVec3(obb->axis[1], q, obb->axis[1]);
			mathQuatMulVec3(obb->axis[2], q, obb->axis[2]);
			break;
		}
		case GEOMETRY_BODY_POLYGON:
		{
			GeometryPolygon_t* polygon = (GeometryPolygon_t*)geo_data;
			indices_rotate(polygon->v, polygon->v_indices, polygon->v_indices_cnt, polygon->o, q);
			mathQuatMulVec3(polygon->normal, q, polygon->normal);
			break;
		}
		case GEOMETRY_BODY_CONVEX_MESH:
		{
			GeometryMesh_t* mesh = (GeometryMesh_t*)geo_data;
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
		case GEOMETRY_BODY_CAPSULE:
		{
			GeometryCapsule_t* capsule = (GeometryCapsule_t*)geo_data;
			mathQuatMulVec3(capsule->axis, q, capsule->axis);
			break;
		}
		default:
			return 0;
	}
	return 1;
}

int mathGeometryRotateAxisRadian(unsigned char* geo_data, int geo_type, const CCTNum_t axis[3], CCTNum_t radian) {
	CCTNum_t q[4];
	if (GEOMETRY_BODY_SPHERE == geo_type) {
		return 1;
	}
	if (GEOMETRY_BODY_CAPSULE == geo_type) {
		CCTNum_t v[3];
		GeometryCapsule_t* capsule = (GeometryCapsule_t*)geo_data;
		mathVec3Cross(v, capsule->axis, axis);
		if (mathVec3IsZero(v)) {
			return 1;
		}
	}
	mathQuatFromAxisRadian(q, axis, radian);
	return mathGeometryRotate(geo_data, geo_type, q);
}

#ifdef __cplusplus
}
#endif
