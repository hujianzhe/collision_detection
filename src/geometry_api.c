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

static CCTNum_t indices_separate_distance(const CCTNum_t(*v)[3], const unsigned int* indices, unsigned int indices_cnt, const CCTNum_t plane_v[3], const CCTNum_t separate_dir[3]) {
	unsigned int i;
	CCTNum_t d = mathPointProjectionPlane(v[indices[0]], plane_v, separate_dir);
	for (i = 1; i < indices_cnt; ++i) {
		CCTNum_t test_d = mathPointProjectionPlane(v[indices[i]], plane_v, separate_dir);
		if (d < test_d) {
			d = test_d;
		}
	}
	return d;
}

static CCTNum_t vertices_separate_distance(const CCTNum_t(*v)[3], unsigned int v_cnt, const CCTNum_t plane_v[3], const CCTNum_t separate_dir[3]) {
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

static CCTNum_t indices_dir_projection_length(const CCTNum_t(*v)[3], const unsigned int* indices, unsigned int indices_cnt, const CCTNum_t dir[3]) {
	CCTNum_t max_d, min_d;
	unsigned int i;
	min_d = max_d = mathVec3Dot(v[indices[0]], dir);
	for (i = 1; i < indices_cnt; ++i) {
		CCTNum_t d = mathVec3Dot(v[indices[i]], dir);
		if (d < min_d) {
			min_d = d;
		}
		else if (d > max_d) {
			max_d = d;
		}
	}
	return max_d - min_d;
}

static CCTNum_t vertices_dir_projection_length(const CCTNum_t(*v)[3], unsigned int v_cnt, const CCTNum_t dir[3]) {
	CCTNum_t max_d, min_d;
	unsigned int i;
	min_d = max_d = mathVec3Dot(v[0], dir);
	for (i = 1; i < v_cnt; ++i) {
		CCTNum_t d = mathVec3Dot(v[i], dir);
		if (d < min_d) {
			min_d = d;
		}
		else if (d > max_d) {
			max_d = d;
		}
	}
	return max_d - min_d;
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
			for (i = 0; i < 3; ++i) {
				if (!CCTNum_chkval(aabb->min_v[i])) {
					return 0;
				}
				if (!CCTNum_chkval(aabb->max_v[i])) {
					return 0;
				}
				if (aabb->max_v[i] - aabb->min_v[i] < GEOMETRY_BODY_BOX_MIN_HALF) {
					return 0;
				}
			}
			return 1;
		}
		case GEOMETRY_BODY_OBB:
		{
			const GeometryOBB_t* obb = (const GeometryOBB_t*)geo_data;
			CCTNum_t dot;
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
			dot = mathVec3Dot(obb->axis[0], obb->axis[1]);
			if (dot < CCT_EPSILON_NEGATE || dot > CCT_EPSILON) {
				return 0;
			}
			dot = mathVec3Dot(obb->axis[0], obb->axis[2]);
			if (dot < CCT_EPSILON_NEGATE || dot > CCT_EPSILON) {
				return 0;
			}
			dot = mathVec3Dot(obb->axis[1], obb->axis[2]);
			if (dot < CCT_EPSILON_NEGATE || dot > CCT_EPSILON) {
				return 0;
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
			if (polygon->tri_cnt < 1) {
				return 0;
			}
			if (polygon->edge_cnt < 3) {
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
		case GEOMETRY_BODY_MESH:
		{
			const GeometryMesh_t* mesh = (const GeometryMesh_t*)geo_data;
			unsigned int i;
			if (mesh->polygons_cnt < 1) {
				return 0;
			}
			if (mesh->v_indices_cnt < 3) {
				return 0;
			}
			if (mesh->edge_cnt < 3) {
				return 0;
			}
			for (i = 0; i < mesh->polygons_cnt; ++i) {
				const GeometryPolygon_t* polygon = mesh->polygons + i;
				/* avoid polygon edge is different from mesh edge... */
				if (!mathGeometryCheckParametersValid(polygon, GEOMETRY_BODY_POLYGON)) {
					return 0;
				}
			}
			for (i = 0; i < mesh->edge_cnt + mesh->edge_cnt; ) {
				GeometrySegment_t segment;
				mathVec3Copy(segment.v[0], mesh->v[mesh->edge_v_indices_flat[i++]]);
				mathVec3Copy(segment.v[1], mesh->v[mesh->edge_v_indices_flat[i++]]);
				/* avoid polygon edge is different from mesh edge... */
				if (!mathGeometryCheckParametersValid(&segment, GEOMETRY_BODY_SEGMENT)) {
					return 0;
				}
			}
			if (!mathGeometryCheckParametersValid(&mesh->bound_box, GEOMETRY_BODY_AABB)) {
				return 0;
			}
			return 1;
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
				if (dst_type) {
					*dst_type = 0;
				}
				return NULL;
			}
			break;
		}
		case GEOMETRY_BODY_MESH:
		{
			if (!mathMeshDeepCopy((GeometryMesh_t*)dst, (const GeometryMesh_t*)src_geo_data)) {
				if (dst_type) {
					*dst_type = 0;
				}
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
			if (dst_type) {
				*dst_type = 0;
			}
			return NULL;
		}
	}
	if (dst_type) {
		*dst_type = src_geo_type;
	}
	return dst;
}

void mathGeometryClear(void* geo_data, int geo_type) {
	if (GEOMETRY_BODY_MESH == geo_type) {
		mathMeshClear((GeometryMesh_t*)geo_data);
	}
	else if (GEOMETRY_BODY_POLYGON == geo_type) {
		mathPolygonClear((GeometryPolygon_t*)geo_data);
	}
}

void mathGeometryClearBody(GeometryBody_t* b) {
	if (!b) {
		return;
	}
	mathGeometryClear(&b->data, b->type);
	b->type = 0;
}

CCTNum_t* mathGeometryGetPosition(const void* geo_data, int geo_type, CCTNum_t v[3]) {
	switch (geo_type) {
		case GEOMETRY_BODY_POINT:
		{
			mathVec3Copy(v, (const CCTNum_t*)geo_data);
			break;
		}
		case GEOMETRY_BODY_SEGMENT:
		{
			const GeometrySegment_t* segment = (const GeometrySegment_t*)geo_data;
			mathVec3Add(v, segment->v[0], segment->v[1]);
			mathVec3MultiplyScalar(v, v, CCTNum(0.5));
			break;
		}
		case GEOMETRY_BODY_PLANE:
		{
			mathVec3Copy(v, ((const GeometryPlane_t*)geo_data)->v);
			break;
		}
		case GEOMETRY_BODY_SPHERE:
		{
			mathVec3Copy(v, ((const GeometrySphere_t*)geo_data)->o);
			break;
		}
		case GEOMETRY_BODY_AABB:
		{
			const GeometryAABB_t* aabb = (const GeometryAABB_t*)geo_data;
			mathVec3Add(v, aabb->min_v, aabb->max_v);
			mathVec3MultiplyScalar(v, v, CCTNum(0.5));
			break;
		}
		case GEOMETRY_BODY_OBB:
		{
			mathVec3Copy(v, ((const GeometryOBB_t*)geo_data)->o);
			break;
		}
		case GEOMETRY_BODY_POLYGON:
		{
			mathVec3Copy(v, ((const GeometryPolygon_t*)geo_data)->center);
			break;
		}
		case GEOMETRY_BODY_MESH:
		{
			const GeometryMesh_t* mesh = (const GeometryMesh_t*)geo_data;
			mathVec3Add(v, mesh->bound_box.min_v, mesh->bound_box.max_v);
			mathVec3MultiplyScalar(v, v, CCTNum(0.5));
			break;
		}
		case GEOMETRY_BODY_CAPSULE:
		{
			mathVec3Copy(v, ((const GeometryCapsule_t*)geo_data)->o);
			break;
		}
		default:
			return NULL;
	}
	return v;
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
			CCTNum_t delta[3], o[3];
			GeometrySegment_t* segment = (GeometrySegment_t*)geo_data;
			mathVec3Add(o, segment->v[0], segment->v[1]);
			mathVec3MultiplyScalar(o, o, CCTNum(0.5));
			mathVec3Sub(delta, v, o);
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
			CCTNum_t l[3];
			mathVec3Sub(l, aabb->max_v, aabb->min_v);
			mathVec3MultiplyScalar(l, l, CCTNum(0.5));
			mathVec3Sub(aabb->min_v, v, l);
			mathVec3Add(aabb->max_v, v, l);
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
			if (mathVec3EqualEps(v, polygon->center, CCTNum(0.0))) {
				return;
			}
			mathVec3Sub(delta, v, polygon->center);
			for (i = 0; i < polygon->v_indices_cnt; ++i) {
				CCTNum_t* p = polygon->v[polygon->v_indices[i]];
				mathVec3Add(p, p, delta);
			}
			mathVec3Copy(polygon->center, v);
			return;
		}
		case GEOMETRY_BODY_MESH:
		{
			unsigned int i;
			GeometryMesh_t* mesh = (GeometryMesh_t*)geo_data;
			CCTNum_t delta[3], o[3], l[3];
			mathVec3Add(o, mesh->bound_box.min_v, mesh->bound_box.max_v);
			mathVec3MultiplyScalar(o, o, CCTNum(0.5));
			if (mathVec3EqualEps(v, o, CCTNum(0.0))) {
				return;
			}
			mathVec3Sub(delta, v, o);
			for (i = 0; i < mesh->v_indices_cnt; ++i) {
				CCTNum_t* p = mesh->v[mesh->v_indices[i]];
				mathVec3Add(p, p, delta);
			}
			mathVec3Sub(l, mesh->bound_box.max_v, mesh->bound_box.min_v);
			mathVec3MultiplyScalar(l, l, CCTNum(0.5));
			mathVec3Sub(mesh->bound_box.min_v, v, l);
			mathVec3Add(mesh->bound_box.max_v, v, l);
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
			mathVec3Copy(aabb->min_v, point);
			aabb->min_v[0] -= GEOMETRY_BODY_BOX_MIN_HALF;
			aabb->min_v[1] -= GEOMETRY_BODY_BOX_MIN_HALF;
			aabb->min_v[2] -= GEOMETRY_BODY_BOX_MIN_HALF;
			mathVec3Copy(aabb->max_v, point);
			aabb->max_v[0] += GEOMETRY_BODY_BOX_MIN_HALF;
			aabb->max_v[1] += GEOMETRY_BODY_BOX_MIN_HALF;
			aabb->max_v[2] += GEOMETRY_BODY_BOX_MIN_HALF;
			break;
		}
		case GEOMETRY_BODY_SEGMENT:
		{
			const GeometrySegment_t* segment = (const GeometrySegment_t*)geo_data;
			const CCTNum_t min_sz = GEOMETRY_BODY_BOX_MIN_HALF + GEOMETRY_BODY_BOX_MIN_HALF;
			mathVerticesFindMinMaxXYZ((const CCTNum_t(*)[3])segment->v, 2, aabb->min_v, aabb->max_v);
			mathAABBFixSize(aabb->min_v, aabb->max_v);
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
			mathVec3Copy(aabb->min_v, sphere->o);
			aabb->min_v[0] -= sphere->radius;
			aabb->min_v[1] -= sphere->radius;
			aabb->min_v[2] -= sphere->radius;
			mathVec3Copy(aabb->max_v, sphere->o);
			aabb->max_v[0] += sphere->radius;
			aabb->max_v[1] += sphere->radius;
			aabb->max_v[2] += sphere->radius;
			break;
		}
		case GEOMETRY_BODY_OBB:
		{
			mathOBBToAABB((const GeometryOBB_t*)geo_data, aabb->min_v, aabb->max_v);
			break;
		}
		case GEOMETRY_BODY_CAPSULE:
		{
			const GeometryCapsule_t* capsule = (const GeometryCapsule_t*)geo_data;
			mathCapsuleFindMaxMinXYZ(capsule, aabb->min_v, aabb->max_v);
			break;
		}
		case GEOMETRY_BODY_POLYGON:
		{
			const GeometryPolygon_t* polygon = (const GeometryPolygon_t*)geo_data;
			if (!mathVertexIndicesFindMinMaxXYZ((const CCTNum_t(*)[3])polygon->v, polygon->v_indices, polygon->v_indices_cnt, aabb->min_v, aabb->max_v)) {
				return NULL;
			}
			mathAABBFixSize(aabb->min_v, aabb->max_v);
			break;
		}
		case GEOMETRY_BODY_MESH:
		{
			*aabb = ((const GeometryMesh_t*)geo_data)->bound_box;
			break;
		}
		default:
		{
			return NULL;
		}
	}
	return aabb;
}

CCTNum_t mathGeometryBoundingSphereRadius(const void* geo_data, int geo_type) {
	CCTNum_t l[3];
	switch (geo_type) {
		case GEOMETRY_BODY_POINT:
		{
			return CCT_GAP_DISTANCE;
		}
		case GEOMETRY_BODY_SEGMENT:
		{
			const GeometrySegment_t* segment = (const GeometrySegment_t*)geo_data;
			mathVec3Sub(l, segment->v[1], segment->v[0]);
			return mathVec3Len(l) * CCTNum(0.5);
		}
		case GEOMETRY_BODY_AABB:
		{
			const GeometryAABB_t* aabb = (const GeometryAABB_t*)geo_data;
			mathVec3Sub(l, aabb->max_v, aabb->min_v);
			return mathVec3Len(l) * CCTNum(0.5);
		}
		case GEOMETRY_BODY_SPHERE:
		{
			return ((const GeometrySphere_t*)geo_data)->radius;
		}
		case GEOMETRY_BODY_OBB:
		{
			const GeometryOBB_t* obb = (const GeometryOBB_t*)geo_data;
			CCTNum_t radius_sq = CCTNum_sq(obb->half[0]) + CCTNum_sq(obb->half[1]) + CCTNum_sq(obb->half[2]);
			return CCTNum_sqrt(radius_sq);
		}
		case GEOMETRY_BODY_CAPSULE:
		{
			const GeometryCapsule_t* capsule = (const GeometryCapsule_t*)geo_data;
			return capsule->half + capsule->radius;
		}
		case GEOMETRY_BODY_POLYGON:
		{
			const GeometryPolygon_t* polygon = (const GeometryPolygon_t*)geo_data;
			unsigned int i;
			CCTNum_t max_distance_sq = CCTNum(0.0);
			for (i = 0; i < polygon->v_indices_cnt; ++i) {
				const CCTNum_t* p = polygon->v[polygon->v_indices[i]];
				CCTNum_t d = mathVec3DistanceSq(polygon->center, p);
				if (d > max_distance_sq) {
					max_distance_sq = d;
				}
			}
			return CCTNum_sqrt(max_distance_sq);
		}
		case GEOMETRY_BODY_MESH:
		{
			const GeometryMesh_t* mesh = (const GeometryMesh_t*)geo_data;
			mathVec3Sub(l, mesh->bound_box.max_v, mesh->bound_box.min_v);
			return mathVec3Len(l) * CCTNum(0.5);
		}
	}
	return CCTNum(0.0);
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

int mathGeometryRotate(void* geo_data, int geo_type, const CCTNum_t q[4]) {
	if (mathQuatIsZeroOrIdentity(q)) {
		return 1;
	}
	switch (geo_type) {
		case GEOMETRY_BODY_POINT:
		{
			return 1;
		}
		case GEOMETRY_BODY_SEGMENT:
		{
			GeometrySegment_t* segment = (GeometrySegment_t*)geo_data;
			CCTNum_t o[3];
			mathVec3Add(o, segment->v[0], segment->v[1]);
			mathVec3MultiplyScalar(o, o, CCTNum(0.5));
			point_rotate(segment->v[0], o, q);
			point_rotate(segment->v[1], o, q);
			return 1;
		}
		case GEOMETRY_BODY_PLANE:
		{
			GeometryPlane_t* plane = (GeometryPlane_t*)geo_data;
			mathQuatMulVec3(plane->normal, q, plane->normal);
			return 1;
		}
		case GEOMETRY_BODY_AABB:
		{
			return 0;
		}
		case GEOMETRY_BODY_OBB:
		{
			GeometryOBB_t* obb = (GeometryOBB_t*)geo_data;
			mathQuatMulVec3(obb->axis[0], q, obb->axis[0]);
			mathQuatMulVec3(obb->axis[1], q, obb->axis[1]);
			mathQuatMulVec3(obb->axis[2], q, obb->axis[2]);
			return 1;
		}
		case GEOMETRY_BODY_POLYGON:
		{
			GeometryPolygon_t* polygon = (GeometryPolygon_t*)geo_data;
			indices_rotate(polygon->v, polygon->v_indices, polygon->v_indices_cnt, polygon->center, q);
			mathQuatMulVec3(polygon->normal, q, polygon->normal);
			return 1;
		}
		case GEOMETRY_BODY_MESH:
		{
			GeometryMesh_t* mesh = (GeometryMesh_t*)geo_data;
			unsigned int i;
			CCTNum_t o[3];
			mathVec3Add(o, mesh->bound_box.min_v, mesh->bound_box.max_v);
			mathVec3MultiplyScalar(o, o, CCTNum(0.5));
			indices_rotate(mesh->v, mesh->v_indices, mesh->v_indices_cnt, o, q);
			for (i = 0; i < mesh->polygons_cnt; ++i) {
				GeometryPolygon_t* polygon = mesh->polygons + i;
				if (polygon->v != mesh->v) {
					indices_rotate(polygon->v, polygon->v_indices, polygon->v_indices_cnt, o, q);
				}
				mathQuatMulVec3(polygon->normal, q, polygon->normal);
				point_rotate(polygon->center, o, q);
			}
			mathVertexIndicesFindMinMaxXYZ((const CCTNum_t(*)[3])mesh->v, mesh->v_indices, mesh->v_indices_cnt, mesh->bound_box.min_v, mesh->bound_box.max_v);
			mathAABBFixSize(mesh->bound_box.min_v, mesh->bound_box.max_v);
			return 1;
		}
		case GEOMETRY_BODY_SPHERE:
		{
			return 1;
		}
		case GEOMETRY_BODY_CAPSULE:
		{
			GeometryCapsule_t* capsule = (GeometryCapsule_t*)geo_data;
			CCTNum_t axis[3];
			mathQuatMulVec3(axis, q, capsule->axis);
			if (!mathVec3Equal(axis, capsule->axis)) {
				mathVec3Copy(capsule->axis, axis);
			}
			return 1;
		}
	}
	return 0;
}

int mathGeometryRotateAxisRadian(void* geo_data, int geo_type, const CCTNum_t axis[3], CCTNum_t radian) {
	CCTNum_t q[4];
	if (CCT_EPSILON_NEGATE <= radian && radian <= CCT_EPSILON) {
		return 1;
	}
	if (GEOMETRY_BODY_SPHERE == geo_type || GEOMETRY_BODY_POINT == geo_type) {
		return 1;
	}
	if (GEOMETRY_BODY_CAPSULE == geo_type) {
		const GeometryCapsule_t* capsule = (const GeometryCapsule_t*)geo_data;
		CCTNum_t v[3];
		mathVec3Cross(v, capsule->axis, axis);
		if (mathVec3IsZero(v)) {
			return 1;
		}
	}
	mathQuatFromAxisRadian(q, axis, radian);
	return mathGeometryRotate(geo_data, geo_type, q);
}

CCTNum_t* mathRotateFromUpFront(CCTNum_t q[4], const CCTNum_t from_up[3], const CCTNum_t from_front[3], const CCTNum_t to_up[3], const CCTNum_t to_front[3]) {
	if (!mathVec3Equal(from_up, to_up)) {
		CCTNum_t front_q[4];
		CCTNum_t to_right[3], up_in_plane[3], temp_front[3];
		mathVec3Cross(to_right, to_up, to_front);
		mathVec3Glide(up_in_plane, from_up, to_right);
		if (!mathVec3Equal(up_in_plane, from_up)) {
			CCTNum_t q1[4], q2[4];
			mathVec3Normalized(up_in_plane, up_in_plane);
			mathQuatFromUnitVec3(q1, from_up, up_in_plane, NULL);
			mathQuatFromUnitVec3(q2, up_in_plane, to_up, to_right);
			mathQuatMulQuat(q, q2, q1);
		}
		else {
			mathQuatFromUnitVec3(q, from_up, to_up, to_right);
		}
		mathQuatMulVec3(temp_front, q, from_front);
		mathQuatFromUnitVec3(front_q, temp_front, to_front, to_up);
		mathQuatMulQuat(q, front_q, q);
	}
	else if (!mathVec3Equal(from_front, to_front)) {
		mathQuatFromUnitVec3(q, from_front, to_front, from_up);
	}
	else {
		mathQuatIdentity(q);
	}
	return q;
}

int mathGeometryRevolve(void* geo_data, int geo_type, const CCTNum_t base_p[3], const CCTNum_t q[4]) {
	if (mathQuatIsZeroOrIdentity(q)) {
		return 1;
	}
	switch (geo_type) {
		case GEOMETRY_BODY_POINT:
		{
			CCTNum_t* point = (CCTNum_t*)geo_data;
			point_rotate(point, base_p, q);
			return 1;
		}
		case GEOMETRY_BODY_SEGMENT:
		{
			GeometrySegment_t* segment = (GeometrySegment_t*)geo_data;
			point_rotate(segment->v[0], base_p, q);
			point_rotate(segment->v[1], base_p, q);
			return 1;
		}
		case GEOMETRY_BODY_PLANE:
		{
			GeometryPlane_t* plane = (GeometryPlane_t*)geo_data;
			point_rotate(plane->v, base_p, q);
			return 1;
		}
		case GEOMETRY_BODY_AABB:
		{
			GeometryAABB_t* aabb = (GeometryAABB_t*)geo_data;
			point_rotate(aabb->min_v, base_p, q);
			point_rotate(aabb->max_v, base_p, q);
			return 1;
		}
		case GEOMETRY_BODY_OBB:
		{
			GeometryOBB_t* obb = (GeometryOBB_t*)geo_data;
			point_rotate(obb->o, base_p, q);
			return 1;
		}
		case GEOMETRY_BODY_POLYGON:
		{
			GeometryPolygon_t* polygon = (GeometryPolygon_t*)geo_data;
			unsigned int i;
			CCTNum_t delta[3], new_center[3];
			mathVec3Copy(new_center, polygon->center);
			point_rotate(new_center, base_p, q);
			mathVec3Sub(delta, new_center, polygon->center);
			if (mathVec3IsZero(delta)) {
				return 1;
			}
			mathVec3Copy(polygon->center, new_center);
			for (i = 0; i < polygon->v_indices_cnt; ++i) {
				CCTNum_t* v = polygon->v[polygon->v_indices[i]];
				mathVec3Add(v, v, delta);
			}
			return 1;
		}
		case GEOMETRY_BODY_MESH:
		{
			GeometryMesh_t* mesh = (GeometryMesh_t*)geo_data;
			unsigned int i;
			CCTNum_t delta[3], new_center[3], center[3];
			mathVec3Add(center, mesh->bound_box.min_v, mesh->bound_box.max_v);
			mathVec3MultiplyScalar(center, center, CCTNum(0.5));
			point_rotate(new_center, base_p, q);
			mathVec3Sub(delta, new_center, center);
			if (mathVec3IsZero(delta)) {
				return 1;
			}
			for (i = 0; i < mesh->polygons_cnt; ++i) {
				CCTNum_t* v = mesh->polygons[i].center;
				mathVec3Add(v, v, delta);
			}
			for (i = 0; i < mesh->v_indices_cnt; ++i) {
				CCTNum_t* v = mesh->v[mesh->v_indices[i]];
				mathVec3Add(v, v, delta);
			}
			mathVec3Add(mesh->bound_box.min_v, mesh->bound_box.min_v, delta);
			mathVec3Add(mesh->bound_box.max_v, mesh->bound_box.max_v, delta);
			return 1;
		}
		case GEOMETRY_BODY_SPHERE:
		{
			GeometrySphere_t* sphere = (GeometrySphere_t*)geo_data;
			point_rotate(sphere->o, base_p, q);
			return 1;
		}
		case GEOMETRY_BODY_CAPSULE:
		{
			GeometryCapsule_t* capsule = (GeometryCapsule_t*)geo_data;
			point_rotate(capsule->o, base_p, q);
			return 1;
		}
	}
	return 0;
}

int mathGeometryRevolveAxisRadian(void* geo_data, int geo_type, const CCTNum_t base_p[3], const CCTNum_t axis[3], CCTNum_t radian) {
	CCTNum_t q[4];
	if (CCT_EPSILON_NEGATE <= radian && radian <= CCT_EPSILON) {
		return 1;
	}
	mathQuatFromAxisRadian(q, axis, radian);
	return mathGeometryRevolve(geo_data, geo_type, base_p, q);
}

CCTNum_t mathGeometrySeparateDistance(const void* geo_data, int geo_type, const CCTNum_t plane_v[3], const CCTNum_t separate_dir[3]) {
	CCTNum_t v[8][3];
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
			mathVec3MultiplyScalar(half_v, capsule->axis, capsule->half);
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
		case GEOMETRY_BODY_MESH:
		{
			const GeometryMesh_t* mesh = (const GeometryMesh_t*)geo_data;
			return indices_separate_distance((const CCTNum_t(*)[3])mesh->v, mesh->v_indices, mesh->v_indices_cnt, plane_v, separate_dir);
		}
		case GEOMETRY_BODY_AABB:
		{
			const GeometryAABB_t* aabb = (const GeometryAABB_t*)geo_data;
			mathAABBVertices(aabb->min_v, aabb->max_v, v);
			return vertices_separate_distance((const CCTNum_t(*)[3])v, 8, plane_v, separate_dir);
		}
		case GEOMETRY_BODY_OBB:
		{
			const GeometryOBB_t* obb = (const GeometryOBB_t*)geo_data;
			mathOBBVertices(obb, v);
			return vertices_separate_distance((const CCTNum_t(*)[3])v, 8, plane_v, separate_dir);
		}
	}
	return CCTNum(0.0);
}

CCTNum_t mathGeometryDirProjectionLength(const void* geo_data, int geo_type, const CCTNum_t dir[3]) {
	CCTNum_t v[8][3];
	switch (geo_type) {
		case GEOMETRY_BODY_SPHERE:
		{
			const GeometrySphere_t* sphere = (const GeometrySphere_t*)geo_data;
			return sphere->radius + sphere->radius;
		}
		case GEOMETRY_BODY_CAPSULE:
		{
			const GeometryCapsule_t* capsule = (const GeometryCapsule_t*)geo_data;
			CCTNum_t p0[3], p1[3], d0, d1;
			mathVec3Copy(p0, capsule->o);
			mathVec3AddScalar(p0, capsule->axis, capsule->half);
			mathVec3Copy(p1, capsule->o);
			mathVec3SubScalar(p1, capsule->axis, capsule->half);
			d0 = mathVec3Dot(p0, dir);
			d1 = mathVec3Dot(p1, dir);
			return CCTNum_abs(d1 - d0) + capsule->radius + capsule->radius;
		}
		case GEOMETRY_BODY_OBB:
		{
			const GeometryOBB_t* obb = (const GeometryOBB_t*)geo_data;
			mathOBBVertices(obb, v);
			return vertices_dir_projection_length((const CCTNum_t(*)[3])v, 8, dir);
		}
		case GEOMETRY_BODY_AABB:
		{
			const GeometryAABB_t* aabb = (const GeometryAABB_t*)geo_data;
			mathAABBVertices(aabb->min_v, aabb->max_v, v);
			return vertices_dir_projection_length((const CCTNum_t(*)[3])v, 8, dir);
		}
		case GEOMETRY_BODY_SEGMENT:
		{
			const GeometrySegment_t* segment = (const GeometrySegment_t*)geo_data;
			CCTNum_t d0 = mathVec3Dot(segment->v[0], dir);
			CCTNum_t d1 = mathVec3Dot(segment->v[1], dir);
			return CCTNum_abs(d0 - d1);
		}
		case GEOMETRY_BODY_POLYGON:
		{
			const GeometryPolygon_t* polygon = (const GeometryPolygon_t*)geo_data;
			return indices_dir_projection_length((const CCTNum_t(*)[3])polygon->v, polygon->v_indices, polygon->v_indices_cnt, dir);
		}
		case GEOMETRY_BODY_MESH:
		{
			const GeometryMesh_t* mesh = (const GeometryMesh_t*)geo_data;
			return indices_dir_projection_length((const CCTNum_t(*)[3])mesh->v, mesh->v_indices, mesh->v_indices_cnt, dir);
		}
	}
	return CCTNum(0.0);
}

#ifdef __cplusplus
}
#endif
