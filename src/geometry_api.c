//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/math_quat.h"
#include "../inc/vertex.h"
#include "../inc/plane.h"
#include "../inc/aabb.h"
#include "../inc/obb.h"
#include "../inc/polygon.h"
#include "../inc/mesh.h"
#include "../inc/capsule.h"
#include "../inc/geometry_api.h"

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

static CCTNum_t indices_separate_distance(const CCTNum_t(*p)[3], const unsigned int* indices, unsigned int indices_cnt, const CCTNum_t plane_v[3], const CCTNum_t separate_dir[3]) {
	unsigned int i;
	CCTNum_t d = mathPointProjectionPlane(p[indices[0]], plane_v, separate_dir);
	for (i = 1; i < indices_cnt; ++i) {
		CCTNum_t test_d = mathPointProjectionPlane(p[indices[i]], plane_v, separate_dir);
		if (d < test_d) {
			d = test_d;
		}
	}
	return d;
}

static CCTNum_t vertex_separate_distance(const CCTNum_t(*v)[3], unsigned int v_cnt, const CCTNum_t plane_v[3], const CCTNum_t separate_dir[3]) {
	unsigned int i;
	CCTNum_t d = mathPointProjectionPlane(v[0], plane_v, separate_dir);
	for (i = 1; i < v_cnt; ++i) {
		CCTNum_t test_d = mathPointProjectionPlane(v[i], plane_v, separate_dir);
		if (d < test_d) {
			d = test_d;
		}
	}
	return d;
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

int mathGeometryCheckParametersValid(const void* geo_data, int geo_type) {
	switch (geo_type) {
		case GEOMETRY_BODY_POINT:
		{
			return CCTNum_chkvals((const CCTNum_t*)geo_data, 3);
		}
		case GEOMETRY_BODY_SEGMENT:
		{
			const GeometrySegment_t* segment = (const GeometrySegment_t*)geo_data;
			if (!CCTNum_chkvals(segment->v[0], 3)) {
				return 0;
			}
			if (!CCTNum_chkvals(segment->v[1], 3)) {
				return 0;
			}
			if (mathVec3Equal(segment->v[0], segment->v[1])) {
				return 0;
			}
			if (!CCTNum_chkvals(segment->o, 3)) {
				return 0;
			}
			return CCTNum_chkval(mathVec3DistanceSq(segment->v[0], segment->v[1]));
		}
		case GEOMETRY_BODY_PLANE:
		{
			const GeometryPlane_t* plane = (const GeometryPlane_t*)geo_data;
			CCTNum_t lensq;
			if (!CCTNum_chkvals(plane->normal, 3)) {
				return 0;
			}
			lensq = mathVec3LenSq(plane->normal);
			if (lensq > CCTNum(1.0) + CCT_EPSILON || lensq < CCTNum(1.0) - CCT_EPSILON) {
				return 0;
			}
			return CCTNum_chkvals(plane->v, 3);
		}
		case GEOMETRY_BODY_AABB:
		{
			const GeometryAABB_t* aabb = (const GeometryAABB_t*)geo_data;
			unsigned int i;
			if (!CCTNum_chkvals(aabb->o, 3)) {
				return 0;
			}
			for (i = 0; i < 3; ++i) {
				CCTNum_t lensq;
				if (!CCTNum_chkval(aabb->half[i])) {
					return 0;
				}
				if (aabb->half[i] < GEOMETRY_BODY_BOX_MIN_HALF) {
					return 0;
				}
				lensq = CCTNum_sq(aabb->half[i]);
				if (!CCTNum_chkval(lensq)) {
					return 0;
				}
			}
			for (i = 0; i < 8; ++i) {
				CCTNum_t v[3];
				mathAABBVertex(aabb->o, aabb->half, i, v);
				if (!CCTNum_chkvals(v, 3)) {
					return 0;
				}
			}
			return 1;
		}
		case GEOMETRY_BODY_OBB:
		{
			const GeometryOBB_t* obb = (const GeometryOBB_t*)geo_data;
			unsigned int i;
			if (!CCTNum_chkvals(obb->o, 3)) {
				return 0;
			}
			for (i = 0; i < 3; ++i) {
				CCTNum_t lensq;
				if (!CCTNum_chkval(obb->half[i])) {
					return 0;
				}
				if (obb->half[i] < GEOMETRY_BODY_BOX_MIN_HALF) {
					return 0;
				}
				lensq = CCTNum_sq(obb->half[i]);
				if (!CCTNum_chkval(lensq)) {
					return 0;
				}
				if (!CCTNum_chkvals(obb->axis[i], 3)) {
					return 0;
				}
				lensq = mathVec3LenSq(obb->axis[i]);
				if (lensq > CCTNum(1.0) + CCT_EPSILON || lensq < CCTNum(1.0) - CCT_EPSILON) {
					return 0;
				}
			}
			for (i = 0; i < 8; ++i) {
				CCTNum_t v[3];
				mathOBBVertex(obb, i, v);
				if (!CCTNum_chkvals(v, 3)) {
					return 0;
				}
			}
			return 1;
		}
		case GEOMETRY_BODY_SPHERE:
		{
			const GeometrySphere_t* sphere = (const GeometrySphere_t*)geo_data;
			if (!CCTNum_chkval(sphere->radius)) {
				return 0;
			}
			if (sphere->radius <= CCT_EPSILON) {
				return 0;
			}
			if (!CCTNum_chkval(CCTNum_sq(sphere->radius))) {
				return 0;
			}
			if (!CCTNum_chkvals(sphere->o, 3)) {
				return 0;
			}
			if (!CCTNum_chkval(sphere->o[0] + sphere->radius)) {
				return 0;
			}
			if (!CCTNum_chkval(sphere->o[1] + sphere->radius)) {
				return 0;
			}
			if (!CCTNum_chkval(sphere->o[2] + sphere->radius)) {
				return 0;
			}
			return 1;
		}
		case GEOMETRY_BODY_CAPSULE:
		{
			const GeometryCapsule_t* capsule = (const GeometryCapsule_t*)geo_data;
			CCTNum_t lensq, v[2][3];
			unsigned int i;
			if (!CCTNum_chkval(capsule->radius)) {
				return 0;
			}
			if (capsule->radius <= CCT_EPSILON) {
				return 0;
			}
			if (!CCTNum_chkval(capsule->half)) {
				return 0;
			}
			if (capsule->half <= CCT_EPSILON) {
				return 0;
			}
			if (!CCTNum_chkval(capsule->radius + capsule->half)) {
				return 0;
			}
			if (!CCTNum_chkvals(capsule->axis, 3)) {
				return 0;
			}
			lensq = mathVec3LenSq(capsule->axis);
			if (lensq > CCTNum(1.0) + CCT_EPSILON || lensq < CCTNum(1.0) - CCT_EPSILON) {
				return 0;
			}
			if (!CCTNum_chkvals(capsule->o, 3)) {
				return 0;
			}
			mathTwoVertexFromCenterHalf(capsule->o, capsule->axis, capsule->half, v[0], v[1]);
			for (i = 0; i < 2; ++i) {
				unsigned int j;
				for (j = 0; j < 3; ++j) {
					if (!CCTNum_chkval(v[i][j])) {
						return 0;
					}
					if (!CCTNum_chkval(v[i][j] + capsule->radius)) {
						return 0;
					}
					if (!CCTNum_chkval(v[i][j] - capsule->radius)) {
						return 0;
					}
				}
			}
			return 1;
		}
		case GEOMETRY_BODY_POLYGON:
		{
			const GeometryPolygon_t* polygon = (const GeometryPolygon_t*)geo_data;
			unsigned int i;
			CCTNum_t lensq;
			if (polygon->v_indices_cnt < 3) {
				return 0;
			}
			if (polygon->tri_indices_cnt % 3) {
				return 0;
			}
			if (polygon->edge_indices_cnt % 2) {
				return 0;
			}
			if (!CCTNum_chkvals(polygon->o, 3)) {
				return 0;
			}
			if (!CCTNum_chkvals(polygon->center, 3)) {
				return 0;
			}
			if (!CCTNum_chkvals(polygon->normal, 3)) {
				return 0;
			}
			lensq = mathVec3LenSq(polygon->normal);
			if (lensq > CCTNum(1.0) + CCT_EPSILON || lensq < CCTNum(1.0) - CCT_EPSILON) {
				return 0;
			}
			if (polygon->is_convex) {
				if (!mathPolygonIsConvex(polygon, CCT_EPSILON)) {
					return 0;
				}
			}
			else if (mathPolygonIsConvex(polygon, CCT_EPSILON)) {
				return 0;
			}
			for (i = 0; i < polygon->v_indices_cnt; ) {
				GeometrySegment_t segment;
				mathVec3Copy(segment.v[0], polygon->v[polygon->v_indices[i++]]);
				mathVec3Copy(segment.v[1], polygon->v[polygon->v_indices[i >= polygon->v_indices_cnt ? 0 : i]]);
				if (!mathGeometryCheckParametersValid(&segment, GEOMETRY_BODY_SEGMENT)) {
					return 0;
				}
			}
			return 1;
		}
		case GEOMETRY_BODY_CONVEX_MESH:
		{
			const GeometryMesh_t* mesh = (const GeometryMesh_t*)geo_data;
			unsigned int i;
			if (mesh->polygons_cnt <= 0) {
				return 0;
			}
			if (mesh->v_indices_cnt < 4) {
				return 0;
			}
			if (mesh->edge_indices_cnt < 6) {
				return 0;
			}
			if (!mesh->is_convex) {
				return 0;
			}
			if (!mathMeshIsConvex(mesh, CCT_EPSILON)) {
				return 0;
			}
			if (!mathMeshIsClosed(mesh)) {
				return 0;
			}
			for (i = 0; i < mesh->polygons_cnt; ++i) {
				const GeometryPolygon_t* polygon = mesh->polygons + i;
				/* avoid polygon edge is different from mesh edge... */
				if (!mathGeometryCheckParametersValid(polygon, GEOMETRY_BODY_POLYGON)) {
					return 0;
				}
			}
			for (i = 0; i < mesh->edge_indices_cnt; ) {
				GeometrySegment_t segment;
				mathVec3Copy(segment.v[0], mesh->v[mesh->edge_indices[i++]]);
				mathVec3Copy(segment.v[1], mesh->v[mesh->edge_indices[i++]]);
				/* avoid polygon edge is different from mesh edge... */
				if (!mathGeometryCheckParametersValid(&segment, GEOMETRY_BODY_SEGMENT)) {
					return 0;
				}
			}
			if (!mathGeometryCheckParametersValid(&mesh->bound_box, GEOMETRY_BODY_AABB)) {
				return 0;
			}
			return CCTNum_chkvals(mesh->o, 3);
		}
	}
	return 0;
}

void* mathGeometryClone(void* dst, int* dst_type, const void* src_geo_data, int src_geo_type) {
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
		mathMeshFreeData((GeometryMesh_t*)geo_data);
	}
	else if (GEOMETRY_BODY_POLYGON == geo_type) {
		mathPolygonFreeData((GeometryPolygon_t*)geo_data);
	}
}

void mathGeometryFreeBody(GeometryBody_t* b) {
	if (!b) {
		return;
	}
	mathGeometryFree(&b->data, b->type);
	b->type = 0;
}

const CCTNum_t* mathGeometryGetPosition(const void* geo_data, int geo_type, CCTNum_t v[3]) {
	const CCTNum_t* ptr_v;
	switch (geo_type) {
		case GEOMETRY_BODY_POINT:
		{
			ptr_v = (const CCTNum_t*)geo_data;
			break;
		}
		case GEOMETRY_BODY_SEGMENT:
		{
			ptr_v = ((const GeometrySegment_t*)geo_data)->o;
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

void mathGeometrySetPosition(void* geo_data, int geo_type, const CCTNum_t v[3]) {
	switch (geo_type) {
		case GEOMETRY_BODY_POINT:
		{
			mathVec3Copy((CCTNum_t*)geo_data, v);
			return;
		}
		case GEOMETRY_BODY_SEGMENT:
		{
			CCTNum_t delta[3];
			GeometrySegment_t* segment = (GeometrySegment_t*)geo_data;
			mathVec3Sub(delta, v, segment->o);
			mathVec3Copy(segment->o, v);
			mathVec3Add(segment->v[0], segment->v[0], delta);
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

GeometryAABB_t* mathGeometryBoundingBox(const void* geo_data, int geo_type, GeometryAABB_t* aabb) {
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

GeometryBody_t* mathGeometryInflate(const void* geo_data, int geo_type, CCTNum_t inflate, GeometryBody_t* geo_inflate) {
	switch (geo_type) {
		case GEOMETRY_BODY_POINT:
		{
			if (inflate <= CCTNum(0.0)) {
				geo_inflate->type = GEOMETRY_BODY_POINT;
				mathVec3Copy(geo_inflate->point, (const CCTNum_t*)geo_data);
			}
			else {
				geo_inflate->type = GEOMETRY_BODY_SPHERE;
				geo_inflate->sphere.radius = inflate;
				mathVec3Copy(geo_inflate->sphere.o, (const CCTNum_t*)geo_data);
			}
			return geo_inflate;
		}
		case GEOMETRY_BODY_SEGMENT:
		{
			const GeometrySegment_t* segment = (const GeometrySegment_t*)geo_data;
			if (inflate <= CCTNum(0.0)) {
				geo_inflate->type = GEOMETRY_BODY_SEGMENT;
				geo_inflate->segment = *segment;
			}
			else {
				geo_inflate->type = GEOMETRY_BODY_CAPSULE;
				mathTwoVertexToCenterHalf(segment->v[0], segment->v[1], geo_inflate->capsule.o, geo_inflate->capsule.axis, &geo_inflate->capsule.half);
				geo_inflate->capsule.radius = inflate;
			}
			return geo_inflate;
		}
		case GEOMETRY_BODY_SPHERE:
		{
			const GeometrySphere_t* sphere = (const GeometrySphere_t*)geo_data;
			CCTNum_t new_radius = sphere->radius + inflate;
			if (new_radius <= CCTNum(0.0)) {
				geo_inflate->type = GEOMETRY_BODY_POINT;
				mathVec3Copy(geo_inflate->point, sphere->o);
			}
			else {
				geo_inflate->type = GEOMETRY_BODY_SPHERE;
				geo_inflate->sphere = *sphere;
				geo_inflate->sphere.radius = new_radius;
			}
			return geo_inflate;
		}
		case GEOMETRY_BODY_CAPSULE:
		{
			const GeometryCapsule_t* capsule = (const GeometryCapsule_t*)geo_data;
			CCTNum_t new_radius = capsule->radius + inflate;
			if (new_radius <= CCTNum(0.0)) {
				geo_inflate->type = GEOMETRY_BODY_SEGMENT;
				mathVec3Copy(geo_inflate->segment.o, capsule->o);
				mathTwoVertexFromCenterHalf(capsule->o, capsule->axis, capsule->half, geo_inflate->segment.v[0], geo_inflate->segment.v[1]);
			}
			else {
				geo_inflate->type = GEOMETRY_BODY_CAPSULE;
				geo_inflate->capsule = *(const GeometryCapsule_t*)geo_data;
				geo_inflate->capsule.radius = new_radius;
			}
			return geo_inflate;
		}
	}
	return NULL;
}

int mathGeometryRotate(void* geo_data, int geo_type, const CCTNum_t base_p[3], const CCTNum_t q[4]) {
	switch (geo_type) {
		case GEOMETRY_BODY_POINT:
		{
			CCTNum_t* point = (CCTNum_t*)geo_data;
			if (point != base_p && !mathVec3Equal(point, base_p)) {
				point_rotate(point, base_p, q);
			}
			break;
		}
		case GEOMETRY_BODY_SEGMENT:
		{
			GeometrySegment_t* segment = (GeometrySegment_t*)geo_data;
			point_rotate(segment->v[0], base_p, q);
			point_rotate(segment->v[1], base_p, q);
			mathVec3Add(segment->o, segment->v[0], segment->v[1]);
			mathVec3MultiplyScalar(segment->o, segment->o, CCTNum(0.5));
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
			/* not self rotate */
			if (aabb->o != base_p && !mathVec3Equal(aabb->o, base_p)) {
				point_rotate(aabb->o, base_p, q);
				return 0;
			}
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
			if (obb->o != base_p && !mathVec3Equal(obb->o, base_p)) {
				point_rotate(obb->o, base_p, q);
			}
			break;
		}
		case GEOMETRY_BODY_POLYGON:
		{
			GeometryPolygon_t* polygon = (GeometryPolygon_t*)geo_data;
			indices_rotate(polygon->v, polygon->v_indices, polygon->v_indices_cnt, base_p, q);
			mathQuatMulVec3(polygon->normal, q, polygon->normal);
			point_rotate(polygon->center, base_p, q);
			break;
		}
		case GEOMETRY_BODY_CONVEX_MESH:
		{
			GeometryMesh_t* mesh = (GeometryMesh_t*)geo_data;
			CCTNum_t min_xyz[3], max_xyz[3];
			unsigned int i;

			indices_rotate(mesh->v, mesh->v_indices, mesh->v_indices_cnt, base_p, q);
			for (i = 0; i < mesh->polygons_cnt; ++i) {
				GeometryPolygon_t* polygon = mesh->polygons + i;
				if (polygon->v != mesh->v) {
					indices_rotate(polygon->v, polygon->v_indices, polygon->v_indices_cnt, base_p, q);
				}
				mathQuatMulVec3(polygon->normal, q, polygon->normal);
				point_rotate(polygon->center, base_p, q);
			}
			mathVertexIndicesFindMinMaxXYZ((const CCTNum_t(*)[3])mesh->v, mesh->v_indices, mesh->v_indices_cnt, min_xyz, max_xyz);
			mathAABBFromTwoVertice(min_xyz, max_xyz, mesh->bound_box.o, mesh->bound_box.half);
			break;
		}
		case GEOMETRY_BODY_CAPSULE:
		{
			GeometryCapsule_t* capsule = (GeometryCapsule_t*)geo_data;
			mathQuatMulVec3(capsule->axis, q, capsule->axis);
			if (capsule->o != base_p && !mathVec3Equal(capsule->o, base_p)) {
				point_rotate(capsule->o, base_p, q);
			}
			break;
		}
		default:
			return 0;
	}
	return 1;
}

int mathGeometryRotateAxisRadian(void* geo_data, int geo_type, const CCTNum_t base_p[3], const CCTNum_t axis[3], CCTNum_t radian) {
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
	if (CCT_EPSILON_NEGATE <= radian && radian <= CCT_EPSILON) {
		return 1;
	}
	mathQuatFromAxisRadian(q, axis, radian);
	return mathGeometryRotate(geo_data, geo_type, base_p, q);
}

CCTNum_t mathGeometrySeparateDistance(const void* geo_data, int geo_type, const CCTNum_t plane_v[3], const CCTNum_t separate_dir[3]) {
	switch (geo_type) {
		case GEOMETRY_BODY_POINT:
		{
			return mathPointProjectionPlane((const CCTNum_t*)geo_data, plane_v, separate_dir);
		}
		case GEOMETRY_BODY_SPHERE:
		{
			const GeometrySphere_t* sphere = (const GeometrySphere_t*)geo_data;
			CCTNum_t d = mathPointProjectionPlane(sphere->o, plane_v, separate_dir);
			return d + sphere->radius;
		}
		case GEOMETRY_BODY_CAPSULE:
		{
			const GeometryCapsule_t* capsule = (const GeometryCapsule_t*)geo_data;
			CCTNum_t p[3], half_v[3], d0, d1;
			mathVec3MultiplyScalar(half_v, separate_dir, capsule->half);
			mathVec3Sub(p, capsule->o, half_v);
			d0 = mathPointProjectionPlane(p, plane_v, separate_dir);
			mathVec3Add(p, capsule->o, half_v);
			d1 = mathPointProjectionPlane(p, plane_v, separate_dir);
			if (d0 > d1) {
				return d0 + capsule->radius;
			}
			return d1 + capsule->radius;
		}
		case GEOMETRY_BODY_SEGMENT:
		{
			const GeometrySegment_t* segment = (const GeometrySegment_t*)geo_data;
			CCTNum_t d0 = mathPointProjectionPlane(segment->v[0], plane_v, separate_dir);
			CCTNum_t d1 = mathPointProjectionPlane(segment->v[1], plane_v, separate_dir);
			return d0 > d1 ? d0 : d1;
		}
		case GEOMETRY_BODY_POLYGON:
		{
			const GeometryPolygon_t* polygon = (const GeometryPolygon_t*)geo_data;
			return indices_separate_distance((const CCTNum_t(*)[3])polygon->v, polygon->v_indices, polygon->v_indices_cnt, plane_v, separate_dir);
		}
		case GEOMETRY_BODY_CONVEX_MESH:
		{
			const GeometryMesh_t* mesh = (const GeometryMesh_t*)geo_data;
			return indices_separate_distance((const CCTNum_t(*)[3])mesh->v, mesh->v_indices, mesh->v_indices_cnt, plane_v, separate_dir);
		}
		case GEOMETRY_BODY_AABB:
		{
			const GeometryAABB_t* aabb = (const GeometryAABB_t*)geo_data;
			CCTNum_t v[8][3];
			mathAABBVertices(aabb->o, aabb->half, v);
			return vertex_separate_distance((const CCTNum_t(*)[3])v, 8, plane_v, separate_dir);
		}
		case GEOMETRY_BODY_OBB:
		{
			const GeometryOBB_t* obb = (const GeometryOBB_t*)geo_data;
			CCTNum_t v[8][3];
			mathOBBVertices(obb, v);
			return vertex_separate_distance((const CCTNum_t(*)[3])v, 8, plane_v, separate_dir);
		}
	}
	return CCTNum(0.0);
}

#ifdef __cplusplus
}
#endif
