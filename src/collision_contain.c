//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/vertex.h"
#include "../inc/line_segment.h"
#include "../inc/plane.h"
#include "../inc/aabb.h"
#include "../inc/obb.h"
#include "../inc/polygon.h"
#include "../inc/mesh.h"
#include "../inc/collision.h"
#include <stddef.h>

extern int Plane_Intersect_Plane(const CCTNum_t v1[3], const CCTNum_t n1[3], const CCTNum_t v2[3], const CCTNum_t n2[3]);

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

int Segment_Contain_Point(const CCTNum_t ls[2][3], const CCTNum_t p[3]) {
	CCTNum_t pv1[3], pv2[3], N[3], dot;
	mathVec3Sub(pv1, ls[0], p);
	mathVec3Sub(pv2, ls[1], p);
	mathVec3Cross(N, pv1, pv2);
	if (!mathVec3IsZero(N)) {
		return 0;
	}
	dot = mathVec3Dot(pv1, pv2);
	return dot <= CCT_EPSILON;
}

static int Segment_Contain_Segment(const CCTNum_t ls1[2][3], const CCTNum_t ls2[2][3]) {
	int i;
	CCTNum_t v1[3], v2[3], N[3];
	mathVec3Sub(v1, ls1[1], ls1[0]);
	mathVec3Sub(v2, ls2[1], ls2[0]);
	mathVec3Cross(N, v1, v2);
	if (!mathVec3IsZero(N)) {
		return 0;
	}
	for (i = 0; i < 2; ++i) {
		CCTNum_t dot;
		mathVec3Sub(v1, ls1[0], ls2[i]);
		mathVec3Sub(v2, ls1[1], ls2[i]);
		dot = mathVec3Dot(v1, v2);
		if (dot > CCT_EPSILON) {
			return 0;
		}
	}
	return 1;
}

int Plane_Contain_Point(const CCTNum_t plane_v[3], const CCTNum_t plane_normal[3], const CCTNum_t p[3]) {
	CCTNum_t v[3], dot;
	mathVec3Sub(v, plane_v, p);
	dot = mathVec3Dot(plane_normal, v);
	return CCT_EPSILON_NEGATE <= dot && dot <= CCT_EPSILON;
}

int Sphere_Contain_Point(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t p[3]) {
	CCTNum_t op[3], op_lensq, radius_sq = radius * radius;
	mathVec3Sub(op, p, o);
	op_lensq = mathVec3LenSq(op);
	return radius_sq >= op_lensq;
}

static int Sphere_Contain_Sphere(const CCTNum_t o1[3], CCTNum_t r1, const CCTNum_t o2[3], CCTNum_t r2) {
	CCTNum_t o1o2[3], len_sq;
	if (r1 < r2) {
		return 0;
	}
	mathVec3Sub(o1o2, o2, o1);
	len_sq = mathVec3LenSq(o1o2);
	return len_sq <= (r1 - r2) * (r1 - r2);
}

static int OBB_Contain_Sphere(const GeometryOBB_t* obb, const CCTNum_t o[3], CCTNum_t radius) {
	int i;
	CCTNum_t v[3];
	mathVec3Sub(v, o, obb->o);
	for (i = 0; i < 3; ++i) {
		CCTNum_t dot = mathVec3Dot(v, obb->axis[i]);
		dot = CCTNum_abs(dot);
		if (dot > obb->half[i] - radius) {
			return 0;
		}
	}
	return 1;
}

static int AABB_Contain_Mesh(const CCTNum_t o[3], const CCTNum_t half[3], const GeometryMesh_t* mesh) {
	unsigned int i;
	if (AABB_Contain_AABB(o, half, mesh->bound_box.o, mesh->bound_box.half)) {
		return 1;
	}
	for (i = 0; i < mesh->v_indices_cnt; ++i) {
		const CCTNum_t* p = mesh->v[mesh->v_indices[i]];
		if (!AABB_Contain_Point(o, half, p)) {
			return 0;
		}
	}
	return 1;
}

static int Sphere_Contain_Mesh(const CCTNum_t o[3], CCTNum_t radius, const GeometryMesh_t* mesh) {
	CCTNum_t v[3];
	unsigned int i;
	mathAABBMinVertice(mesh->bound_box.o, mesh->bound_box.half, v);
	if (Sphere_Contain_Point(o, radius, v)) {
		mathAABBMaxVertice(mesh->bound_box.o, mesh->bound_box.half, v);
		if (Sphere_Contain_Point(o, radius, v)) {
			return 1;
		}
	}
	for (i = 0; i < mesh->v_indices_cnt; ++i) {
		const CCTNum_t* p = mesh->v[mesh->v_indices[i]];
		if (!Sphere_Contain_Point(o, radius, p)) {
			return 0;
		}
	}
	return 1;
}

static int OBB_Contain_Mesh(const GeometryOBB_t* obb, const GeometryMesh_t* mesh) {
	CCTNum_t p[8][3];
	unsigned int i;
	mathAABBVertices(mesh->bound_box.o, mesh->bound_box.half, p);
	for (i = 0; i < 8; ++i) {
		if (!OBB_Contain_Point(obb, p[i])) {
			break;
		}
	}
	if (i >= 8) {
		return 1;
	}
	for (i = 0; i < mesh->v_indices_cnt; ++i) {
		const CCTNum_t* p = mesh->v[mesh->v_indices[i]];
		if (!OBB_Contain_Point(obb, p)) {
			return 0;
		}
	}
	return 1;
}

static int ConvexMesh_Contain_AABB(const GeometryMesh_t* mesh, const CCTNum_t o[3], const CCTNum_t half[3]) {
	CCTNum_t p[8][3];
	unsigned int i;
	if (!AABB_Contain_AABB(mesh->bound_box.o, mesh->bound_box.half, o, half)) {
		return 0;
	}
	mathAABBVertices(o, half, p);
	for (i = 0; i < 8; ++i) {
		if (!ConvexMesh_Contain_Point(mesh, p[i])) {
			return 0;
		}
	}
	return 1;
}

static int ConvexMesh_Contain_OBB(const GeometryMesh_t* mesh, const GeometryOBB_t* obb) {
	CCTNum_t p[8][3];
	unsigned int i;
	mathOBBVertices(obb, p);
	for (i = 0; i < 8; ++i) {
		if (!ConvexMesh_Contain_Point(mesh, p[i])) {
			return 0;
		}
	}
	return 1;
}

static int ConvexMesh_Contain_Sphere(const GeometryMesh_t* mesh, const CCTNum_t o[3], CCTNum_t radius) {
	unsigned int i;
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		CCTNum_t d;
		const GeometryPolygon_t* polygon = mesh->polygons + i;
		mathPointProjectionPlane(o, polygon->v[polygon->v_indices[0]], polygon->normal, NULL, &d);
		if (d < radius) {
			return 0;
		}
	}
	return 1;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
extern "C" {
#endif

int mathCollisionContain(const GeometryBodyRef_t* one, const GeometryBodyRef_t* two) {
	if (one->data == two->data) {
		return 1;
	}
	if (GEOMETRY_BODY_AABB == one->type) {
		switch (two->type) {
		case GEOMETRY_BODY_POINT:
		{
			return AABB_Contain_Point(one->aabb->o, one->aabb->half, two->point);
		}
		case GEOMETRY_BODY_SEGMENT:
		{
			return	AABB_Contain_Point(one->aabb->o, one->aabb->half, two->segment->v[0]) &&
				AABB_Contain_Point(one->aabb->o, one->aabb->half, two->segment->v[1]);
		}
		case GEOMETRY_BODY_AABB:
		{
			return AABB_Contain_AABB(one->aabb->o, one->aabb->half, two->aabb->o, two->aabb->half);
		}
		case GEOMETRY_BODY_OBB:
		{
			GeometryOBB_t one_obb;
			mathOBBFromAABB(&one_obb, one->aabb->o, one->aabb->half);
			return OBB_Contain_OBB(&one_obb, two->obb);
		}
		case GEOMETRY_BODY_SPHERE:
		{
			GeometryOBB_t one_obb;
			mathOBBFromAABB(&one_obb, one->aabb->o, one->aabb->half);
			return OBB_Contain_Sphere(&one_obb, two->sphere->o, two->sphere->radius);
		}
		case GEOMETRY_BODY_CONVEX_MESH:
		{
			return AABB_Contain_Mesh(one->aabb->o, one->aabb->half, two->mesh);
		}
		}
	}
	else if (GEOMETRY_BODY_OBB == one->type) {
		switch (two->type) {
		case GEOMETRY_BODY_POINT:
		{
			return OBB_Contain_Point(one->obb, two->point);
		}
		case GEOMETRY_BODY_SEGMENT:
		{
			return	OBB_Contain_Point(one->obb, two->segment->v[0]) &&
				OBB_Contain_Point(one->obb, two->segment->v[1]);
		}
		case GEOMETRY_BODY_AABB:
		{
			GeometryOBB_t two_obb;
			mathOBBFromAABB(&two_obb, two->aabb->o, two->aabb->half);
			return OBB_Contain_OBB(one->obb, &two_obb);
		}
		case GEOMETRY_BODY_OBB:
		{
			return OBB_Contain_OBB(one->obb, two->obb);
		}
		case GEOMETRY_BODY_SPHERE:
		{
			return OBB_Contain_Sphere(one->obb, two->sphere->o, two->sphere->radius);
		}
		case GEOMETRY_BODY_CONVEX_MESH:
		{
			return OBB_Contain_Mesh(one->obb, two->mesh);
		}
		}
	}
	else if (GEOMETRY_BODY_SPHERE == one->type) {
		switch (two->type) {
		case GEOMETRY_BODY_POINT:
		{
			return Sphere_Contain_Point(one->sphere->o, one->sphere->radius, two->point);
		}
		case GEOMETRY_BODY_SEGMENT:
		{
			return	Sphere_Contain_Point(one->sphere->o, one->sphere->radius, two->segment->v[0]) &&
				Sphere_Contain_Point(one->sphere->o, one->sphere->radius, two->segment->v[1]);
		}
		case GEOMETRY_BODY_AABB:
		{
			CCTNum_t v[3];
			mathAABBMinVertice(two->aabb->o, two->aabb->half, v);
			if (!Sphere_Contain_Point(one->sphere->o, one->sphere->radius, v)) {
				return 0;
			}
			mathAABBMaxVertice(two->aabb->o, two->aabb->half, v);
			return Sphere_Contain_Point(one->sphere->o, one->sphere->radius, v);
		}
		case GEOMETRY_BODY_OBB:
		{
			CCTNum_t v[3];
			mathOBBMinVertice(two->obb, v);
			if (!Sphere_Contain_Point(one->sphere->o, one->sphere->radius, v)) {
				return 0;
			}
			mathOBBMaxVertice(two->obb, v);
			return Sphere_Contain_Point(one->sphere->o, one->sphere->radius, v);
		}
		case GEOMETRY_BODY_SPHERE:
		{
			return Sphere_Contain_Sphere(one->sphere->o, one->sphere->radius, two->sphere->o, two->sphere->radius);
		}
		case GEOMETRY_BODY_CONVEX_MESH:
		{
			return Sphere_Contain_Mesh(one->sphere->o, one->sphere->radius, two->mesh);
		}
		}
	}
	else if (GEOMETRY_BODY_CONVEX_MESH == one->type) {
		switch (two->type) {
		case GEOMETRY_BODY_POINT:
		{
			return ConvexMesh_Contain_Point(one->mesh, two->point);
		}
		case GEOMETRY_BODY_SEGMENT:
		{
			return	ConvexMesh_Contain_Point(one->mesh, two->segment->v[0]) &&
				ConvexMesh_Contain_Point(one->mesh, two->segment->v[1]);
		}
		case GEOMETRY_BODY_AABB:
		{
			return ConvexMesh_Contain_AABB(one->mesh, two->aabb->o, two->aabb->half);
		}
		case GEOMETRY_BODY_OBB:
		{
			return ConvexMesh_Contain_OBB(one->mesh, two->obb);
		}
		case GEOMETRY_BODY_SPHERE:
		{
			return ConvexMesh_Contain_Sphere(one->mesh, two->sphere->o, two->sphere->radius);
		}
		case GEOMETRY_BODY_CONVEX_MESH:
		{
			return ConvexMesh_Contain_ConvexMesh(one->mesh, two->mesh);
		}
		}
	}
	else if (GEOMETRY_BODY_POLYGON == one->type) {
		switch (two->type) {
		case GEOMETRY_BODY_POINT:
		{
			return Polygon_Contain_Point(one->polygon, two->point);
		}
		case GEOMETRY_BODY_SEGMENT:
		{
			return	Polygon_Contain_Point(one->polygon, two->segment->v[0]) &&
				Polygon_Contain_Point(one->polygon, two->segment->v[1]);
		}
		case GEOMETRY_BODY_POLYGON:
		{
			return Polygon_Contain_Polygon(one->polygon, two->polygon);
		}
		}
	}
	else if (GEOMETRY_BODY_PLANE == one->type) {
		switch (two->type) {
		case GEOMETRY_BODY_POINT:
		{
			return Plane_Contain_Point(one->plane->v, one->plane->normal, two->point);
		}
		case GEOMETRY_BODY_SEGMENT:
		{
			return	Plane_Contain_Point(one->plane->v, one->plane->normal, two->segment->v[0]) &&
				Plane_Contain_Point(one->plane->v, one->plane->normal, two->segment->v[1]);
		}
		case GEOMETRY_BODY_PLANE:
		{
			return Plane_Intersect_Plane(one->plane->v, one->plane->normal, two->plane->v, two->plane->normal) == 2;
		}
		case GEOMETRY_BODY_POLYGON:
		{
			const GeometryPolygon_t* polygon = two->polygon;
			return Plane_Intersect_Plane(one->plane->v, one->plane->normal, polygon->v[polygon->v_indices[0]], polygon->normal) == 2;
		}
		}
	}
	else if (GEOMETRY_BODY_SEGMENT == one->type) {
		switch (two->type) {
		case GEOMETRY_BODY_POINT:
		{
			return Segment_Contain_Point((const CCTNum_t(*)[3])one->segment->v, two->point);
		}
		case GEOMETRY_BODY_SEGMENT:
		{
			return Segment_Contain_Segment((const CCTNum_t(*)[3])one->segment->v, (const CCTNum_t(*)[3])two->segment->v);
		}
		}
	}
	else if (GEOMETRY_BODY_POINT == one->type) {
		if (GEOMETRY_BODY_POINT == two->type) {
			return mathVec3Equal(one->point, two->point);
		}
	}
	return 0;
}

#ifdef __cplusplus
}
#endif
