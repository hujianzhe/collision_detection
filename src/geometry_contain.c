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
#include "../inc/capsule.h"
#include "../inc/geometry_api.h"

extern const unsigned int Box_Vertice_Indices_Default[8];
extern const unsigned int Box_Face_Vertice_Indices[6][4];

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

int Segment_Contain_Point(const CCTNum_t ls0[3], const CCTNum_t ls1[3], const CCTNum_t p[3]) {
	CCTNum_t pv1[3], pv2[3], N[3], dot;
	mathVec3Sub(pv1, ls0, p);
	mathVec3Sub(pv2, ls1, p);
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

static int Plane_Contain_Plane(const CCTNum_t v1[3], const CCTNum_t n1[3], const CCTNum_t v2[3], const CCTNum_t n2[3]) {
	CCTNum_t v[3], dot;
	mathVec3Sub(v, v2, v1);
	dot = mathVec3Dot(n1, v);
	if (dot < CCT_EPSILON_NEGATE || dot > CCT_EPSILON) {
		return 0;
	}
	mathVec3Cross(v, n1, n2);
	return mathVec3IsZero(v);
}

int Sphere_Contain_Point(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t p[3]) {
	CCTNum_t op[3], op_lensq;
	mathVec3Sub(op, p, o);
	op_lensq = mathVec3LenSq(op);
	return CCTNum_sq(radius) >= op_lensq;
}

static int Sphere_Contain_Sphere(const CCTNum_t o1[3], CCTNum_t r1, const CCTNum_t o2[3], CCTNum_t r2) {
	CCTNum_t o1o2[3], len_sq;
	if (r1 < r2) {
		return 0;
	}
	mathVec3Sub(o1o2, o2, o1);
	len_sq = mathVec3LenSq(o1o2);
	return len_sq <= CCTNum_sq(r1 - r2);
}

static int Sphere_Contain_Capsule(const CCTNum_t o[3], CCTNum_t r, const GeometryCapsule_t* capsule) {
	CCTNum_t sp_o[3];
	mathVec3Copy(sp_o, capsule->o);
	mathVec3SubScalar(sp_o, capsule->axis, capsule->half);
	if (!Sphere_Contain_Sphere(o, r, sp_o, capsule->radius)) {
		return 0;
	}
	mathVec3AddScalar(sp_o, capsule->axis, capsule->half + capsule->half);
	return Sphere_Contain_Sphere(o, r, sp_o, capsule->radius);
}

static int Box_Contain_Point(const CCTNum_t v[8][3], const CCTNum_t p[3]) {
	CCTNum_t vp[3], edge_v[3], dot;
	mathVec3Sub(vp, p, v[0]);

	mathVec3Sub(edge_v, v[1], v[0]);
	dot = mathVec3Dot(edge_v, vp);
	if (dot < CCTNum(0.0) || dot > mathVec3LenSq(edge_v)) {
		return 0;
	}
	mathVec3Sub(edge_v, v[4], v[0]);
	dot = mathVec3Dot(edge_v, vp);
	if (dot < CCTNum(0.0) || dot > mathVec3LenSq(edge_v)) {
		return 0;
	}
	mathVec3Sub(edge_v, v[3], v[0]);
	dot = mathVec3Dot(edge_v, vp);
	if (dot < CCTNum(0.0) || dot > mathVec3LenSq(edge_v)) {
		return 0;
	}
	return 1;
}

int OBB_Contain_Point(const GeometryOBB_t* obb, const CCTNum_t p[3]) {
	CCTNum_t op[3], dot;
	mathVec3Sub(op, p, obb->o);
	dot = mathVec3Dot(op, obb->axis[0]);
	if (dot > obb->half[0] || dot < -obb->half[0]) {
		return 0;
	}
	dot = mathVec3Dot(op, obb->axis[1]);
	if (dot > obb->half[1] || dot < -obb->half[1]) {
		return 0;
	}
	dot = mathVec3Dot(op, obb->axis[2]);
	if (dot > obb->half[2] || dot < -obb->half[2]) {
		return 0;
	}
	return 1;
}

static int OBB_Contain_OBB(const GeometryOBB_t* obb0, const GeometryOBB_t* obb1) {
	CCTNum_t AX[3][3], p[3];
	if (obb0 == obb1) {
		return 1;
	}
	mathVec3MultiplyScalar(AX[0], obb1->axis[0], obb1->half[0]);
	mathVec3MultiplyScalar(AX[1], obb1->axis[1], obb1->half[1]);
	mathVec3MultiplyScalar(AX[2], obb1->axis[2], obb1->half[2]);

	mathVec3Copy(p, obb1->o);
	mathVec3Sub(p, p, AX[0]);
	mathVec3Sub(p, p, AX[1]);
	mathVec3Sub(p, p, AX[2]);
	if (!OBB_Contain_Point(obb0, p)) {
		return 0;
	}
	mathVec3Copy(p, obb1->o);
	mathVec3Add(p, p, AX[0]);
	mathVec3Sub(p, p, AX[1]);
	mathVec3Sub(p, p, AX[2]);
	if (!OBB_Contain_Point(obb0, p)) {
		return 0;
	}
	mathVec3Copy(p, obb1->o);
	mathVec3Add(p, p, AX[0]);
	mathVec3Add(p, p, AX[1]);
	mathVec3Sub(p, p, AX[2]);
	if (!OBB_Contain_Point(obb0, p)) {
		return 0;
	}
	mathVec3Copy(p, obb1->o);
	mathVec3Sub(p, p, AX[0]);
	mathVec3Add(p, p, AX[1]);
	mathVec3Sub(p, p, AX[2]);
	if (!OBB_Contain_Point(obb0, p)) {
		return 0;
	}
	mathVec3Copy(p, obb1->o);
	mathVec3Sub(p, p, AX[0]);
	mathVec3Sub(p, p, AX[1]);
	mathVec3Add(p, p, AX[2]);
	if (!OBB_Contain_Point(obb0, p)) {
		return 0;
	}
	mathVec3Copy(p, obb1->o);
	mathVec3Add(p, p, AX[0]);
	mathVec3Sub(p, p, AX[1]);
	mathVec3Add(p, p, AX[2]);
	if (!OBB_Contain_Point(obb0, p)) {
		return 0;
	}
	mathVec3Copy(p, obb1->o);
	mathVec3Add(p, p, AX[0]);
	mathVec3Add(p, p, AX[1]);
	mathVec3Add(p, p, AX[2]);
	if (!OBB_Contain_Point(obb0, p)) {
		return 0;
	}
	mathVec3Copy(p, obb1->o);
	mathVec3Sub(p, p, AX[0]);
	mathVec3Add(p, p, AX[1]);
	mathVec3Add(p, p, AX[2]);
	if (!OBB_Contain_Point(obb0, p)) {
		return 0;
	}
	return 1;
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

int Capsule_Contain_Point(const GeometryCapsule_t* capsule, const CCTNum_t p[3]) {
	CCTNum_t v[3], dot, lensq, radius_sq = CCTNum_sq(capsule->radius);
	mathVec3Sub(v, p, capsule->o);
	dot = mathVec3Dot(v, capsule->axis);
	lensq = mathVec3LenSq(v) - CCTNum_sq(dot);
	if (lensq > radius_sq) {
		return 0;
	}
	if (CCTNum_abs(dot) <= capsule->half) {
		return 1;
	}
	mathVec3Copy(v, capsule->o);
	if (dot > CCTNum(0.0)) {
		mathVec3AddScalar(v, capsule->axis, capsule->half);
	}
	else {
		mathVec3SubScalar(v, capsule->axis, capsule->half);
	}
	lensq = mathVec3DistanceSq(v, p);
	return lensq <= radius_sq;
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

static int OBB_Contain_Capsule(const GeometryOBB_t* obb, const GeometryCapsule_t* capsule) {
	CCTNum_t sp_o[3];
	mathVec3Copy(sp_o, capsule->o);
	mathVec3SubScalar(sp_o, capsule->axis, capsule->half);
	if (!OBB_Contain_Sphere(obb, sp_o, capsule->radius)) {
		return 0;
	}
	mathVec3AddScalar(sp_o, capsule->axis, capsule->half + capsule->half);
	return OBB_Contain_Sphere(obb, sp_o, capsule->radius);
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

static int ConvexPolygon_Contain_Point(const GeometryPolygon_t* polygon, const CCTNum_t p[3]) {
	unsigned int i;
	CCTNum_t v[3], dot;
	mathVec3Sub(v, p, polygon->v[polygon->v_indices[0]]);
	dot = mathVec3Dot(polygon->normal, v);
	if (dot > CCT_EPSILON || dot < CCT_EPSILON_NEGATE) {
		return 0;
	}
	for (i = 0; i < polygon->edge_indices_cnt; ) {
		CCTNum_t ls_dir[3], ls_n[3], test_dot;
		unsigned int edge_idx[2], other_i;
		/* test edge */
		edge_idx[0] = polygon->edge_indices[i++];
		edge_idx[1] = polygon->edge_indices[i++];
		mathVec3Sub(ls_dir, polygon->v[edge_idx[1]], polygon->v[edge_idx[0]]);
		mathVec3Cross(ls_n, ls_dir, polygon->normal);
		/* check edge contain p */
		mathVec3Sub(v, p, polygon->v[edge_idx[0]]);
		dot = mathVec3Dot(v, ls_n);
		if (dot >= CCT_EPSILON_NEGATE && dot <= CCT_EPSILON) {
			CCTNum_t l[3], r[3];
			mathVec3Sub(l, polygon->v[edge_idx[0]], p);
			mathVec3Sub(r, polygon->v[edge_idx[1]], p);
			dot = mathVec3Dot(l, r);
			return dot <= CCT_EPSILON;
		}
		/* get test_point */
		other_i = (i < polygon->edge_indices_cnt ? i : 0);
		if (polygon->edge_indices[other_i] != edge_idx[0] && polygon->edge_indices[other_i] != edge_idx[1]) {
			mathVec3Sub(v, polygon->v[polygon->edge_indices[other_i]], polygon->v[edge_idx[0]]);
		}
		else {
			mathVec3Sub(v, polygon->v[polygon->edge_indices[other_i + 1]], polygon->v[edge_idx[0]]);
		}
		test_dot = mathVec3Dot(v, ls_n);
		/* check in same side */
		if (test_dot > CCTNum(0.0) && dot < CCTNum(0.0)) {
			return 0;
		}
		else if (test_dot < CCTNum(0.0) && dot > CCTNum(0.0)) {
			return 0;
		}
	}
	return 1;
}

int Polygon_Contain_Point(const GeometryPolygon_t* polygon, const CCTNum_t p[3]) {
	if (polygon->v_indices_cnt < 3) {
		return 0;
	}
	if (3 == polygon->v_indices_cnt) {
		return mathTrianglePointUV(
			polygon->v[polygon->v_indices[0]],
			polygon->v[polygon->v_indices[1]],
			polygon->v[polygon->v_indices[2]],
			p, NULL, NULL
		);
	}
	if ((const void*)polygon->v_indices >= (const void*)Box_Face_Vertice_Indices &&
		(const void*)polygon->v_indices < (const void*)(Box_Face_Vertice_Indices + 6))
	{
		CCTNum_t ls_vec[3], v[3], dot;
		mathVec3Sub(v, p, polygon->v[polygon->v_indices[0]]);
		dot = mathVec3Dot(polygon->normal, v);
		if (dot < CCT_EPSILON_NEGATE || dot > CCT_EPSILON) {
			return 0;
		}
		mathVec3Sub(ls_vec, polygon->v[polygon->v_indices[1]], polygon->v[polygon->v_indices[0]]);
		dot = mathVec3Dot(ls_vec, v);
		if (dot < CCTNum(0.0) || dot > mathVec3LenSq(ls_vec)) {
			return 0;
		}
		mathVec3Sub(ls_vec, polygon->v[polygon->v_indices[3]], polygon->v[polygon->v_indices[0]]);
		dot = mathVec3Dot(ls_vec, v);
		if (dot < CCTNum(0.0) || dot > mathVec3LenSq(ls_vec)) {
			return 0;
		}
		return 1;
	}
	if (polygon->is_convex) {
		return ConvexPolygon_Contain_Point(polygon, p);
	}
	if (polygon->tri_indices && polygon->tri_indices_cnt >= 3) {
		unsigned int i;
		if (!Plane_Contain_Point(polygon->v[polygon->tri_indices[0]], polygon->normal, p)) {
			return 0;
		}
		for (i = 0; i < polygon->tri_indices_cnt; ) {
			unsigned int v_idx[3];
			v_idx[0] = polygon->tri_indices[i++];
			v_idx[1] = polygon->tri_indices[i++];
			v_idx[2] = polygon->tri_indices[i++];
			if (mathTrianglePointUV(polygon->v[v_idx[0]], polygon->v[v_idx[1]], polygon->v[v_idx[2]], p, NULL, NULL)) {
				return 1;
			}
		}
		return 0;
	}
	return 0;
}

static int Polygon_Contain_Segment(const GeometryPolygon_t* polygon, const CCTNum_t p1[3], const CCTNum_t p2[3]) {
	// TODO
	return 0;
}

static int Polygon_Contain_Polygon(const GeometryPolygon_t* polygon1, const GeometryPolygon_t* polygon2) {
	unsigned int i;
	if (!Plane_Contain_Plane(polygon1->v[polygon1->v_indices[0]], polygon1->normal, polygon2->v[polygon2->v_indices[0]], polygon2->normal)) {
		return 0;
	}
	for (i = 0; i < polygon2->v_indices_cnt; ++i) {
		const CCTNum_t* p = polygon2->v[polygon2->v_indices[i]];
		if (!Polygon_Contain_Point(polygon1, p)) {
			return 0;
		}
	}
	if (polygon1->is_convex || polygon2->is_convex) {
		return 1;
	}
	for (i = 0; i < polygon2->edge_indices_cnt; ) {
		const CCTNum_t* p1 = polygon2->v[polygon2->edge_indices[i++]];
		const CCTNum_t* p2 = polygon2->v[polygon2->edge_indices[i++]];
		if (!Polygon_Contain_Segment(polygon1, p1, p2)) {
			return 0;
		}
	}
	return 0;
}

static int ConvexMesh_Contain_Point_InternalProc(const GeometryMesh_t* mesh, const CCTNum_t p[3]) {
	unsigned int i;
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		CCTNum_t v[3], dot;
		const GeometryPolygon_t* polygon = mesh->polygons + i;
		mathVec3Sub(v, p, polygon->v[polygon->v_indices[0]]);
		dot = mathVec3Dot(v, polygon->normal);
		if (dot < CCTNum(0.0)) {
			continue;
		}
		if (dot > CCTNum(0.0)) {
			return 0;
		}
		return ConvexPolygon_Contain_Point(polygon, p);
	}
	return 1;
}

int ConvexMesh_Contain_Point(const GeometryMesh_t* mesh, const CCTNum_t p[3]) {
	if (Box_Vertice_Indices_Default == mesh->v_indices) {
		return Box_Contain_Point((const CCTNum_t(*)[3])mesh->v, p);
	}
	if (!AABB_Contain_Point(mesh->bound_box.o, mesh->bound_box.half, p)) {
		return 0;
	}
	return ConvexMesh_Contain_Point_InternalProc(mesh, p);
}

static int ConvexMesh_Contain_ConvexMesh(const GeometryMesh_t* mesh1, const GeometryMesh_t* mesh2) {
	unsigned int i;
	if (!AABB_Intersect_AABB(mesh1->bound_box.o, mesh1->bound_box.half, mesh2->bound_box.o, mesh2->bound_box.half)) {
		return 0;
	}
	for (i = 0; i < mesh2->v_indices_cnt; ++i) {
		const CCTNum_t* p = mesh2->v[mesh2->v_indices[i]];
		if (!ConvexMesh_Contain_Point_InternalProc(mesh1, p)) {
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
		const GeometryPolygon_t* polygon = mesh->polygons + i;
		CCTNum_t d = mathPointProjectionPlane(o, polygon->v[polygon->v_indices[0]], polygon->normal);
		if (CCTNum_abs(d) < radius) {
			return 0;
		}
	}
	return ConvexMesh_Contain_Point(mesh, o);
}

static int ConvexMesh_Contain_Capsule(const GeometryMesh_t* mesh, const GeometryCapsule_t* capsule) {
	CCTNum_t sp_o[3];
	mathVec3Copy(sp_o, capsule->o);
	mathVec3SubScalar(sp_o, capsule->axis, capsule->half);
	if (!ConvexMesh_Contain_Sphere(mesh, sp_o, capsule->radius)) {
		return 0;
	}
	mathVec3AddScalar(sp_o, capsule->axis, capsule->half + capsule->half);
	return ConvexMesh_Contain_Sphere(mesh, sp_o, capsule->radius);
}

static int ConvexMesh_Contain_VerticeIndices(const GeometryMesh_t* mesh, const CCTNum_t(*v)[3], const unsigned int* v_indices, size_t v_indices_cnt) {
	unsigned int i;
	for (i = 0; i < v_indices_cnt; ++i) {
		const CCTNum_t* p = v[v_indices[i]];
		if (!ConvexMesh_Contain_Point(mesh, p)) {
			return 0;
		}
	}
	return 1;
}

static int OBB_Contain_VerticeIndices(const GeometryOBB_t* obb, const CCTNum_t(*v)[3], const unsigned int* v_indices, size_t v_indices_cnt) {
	unsigned int i;
	for (i = 0; i < v_indices_cnt; ++i) {
		const CCTNum_t* p = v[v_indices[i]];
		if (!OBB_Contain_Point(obb, p)) {
			return 0;
		}
	}
	return 1;
}

static int Sphere_Contain_VerticeIndices(const CCTNum_t o[3], CCTNum_t r, const CCTNum_t(*v)[3], const unsigned int* v_indices, size_t v_indices_cnt) {
	unsigned int i;
	for (i = 0; i < v_indices_cnt; ++i) {
		const CCTNum_t* p = v[v_indices[i]];
		if (!Sphere_Contain_Point(o, r, p)) {
			return 0;
		}
	}
	return 1;
}

static int Capsule_Contain_VerticeIndices(const GeometryCapsule_t* capsule, const CCTNum_t(*v)[3], const unsigned int* v_indices, size_t v_indices_cnt) {
	unsigned int i;
	for (i = 0; i < v_indices_cnt; ++i) {
		const CCTNum_t* p = v[v_indices[i]];
		if (!Capsule_Contain_Point(capsule, p)) {
			return 0;
		}
	}
	return 1;
}

static int Capsule_Contain_Sphere(const GeometryCapsule_t* capsule, const CCTNum_t o[3], CCTNum_t r) {
	CCTNum_t closest_p[3];
	mathSegmentClosestPointTo_v2(capsule->o, capsule->axis, capsule->half, o, closest_p);
	return Sphere_Contain_Sphere(closest_p, capsule->radius, o, r);
}

static int Capsule_Contain_Capsule(const GeometryCapsule_t* capsule1, const GeometryCapsule_t* capsule2) {
	CCTNum_t sp_o[3];
	mathVec3Copy(sp_o, capsule2->o);
	mathVec3SubScalar(sp_o, capsule2->axis, capsule2->half);
	if (!Capsule_Contain_Sphere(capsule1, sp_o, capsule2->radius)) {
		return 0;
	}
	mathVec3AddScalar(sp_o, capsule2->axis, capsule2->half + capsule2->half);
	return Capsule_Contain_Sphere(capsule1, sp_o, capsule2->radius);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
extern "C" {
#endif

int mathGeometryContain(const void* geo_data1, int geo_type1, const void* geo_data2, int geo_type2) {
	if (geo_data1 == geo_data2) {
		return 1;
	}
	if (GEOMETRY_BODY_AABB == geo_type1) {
		const GeometryAABB_t* aabb1 = (const GeometryAABB_t*)geo_data1;
		switch (geo_type2) {
			case GEOMETRY_BODY_POINT:
			{
				const CCTNum_t* point2 = (const CCTNum_t*)geo_data2;
				return AABB_Contain_Point(aabb1->o, aabb1->half, point2);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				return	AABB_Contain_Point(aabb1->o, aabb1->half, segment2->v[0]) &&
					AABB_Contain_Point(aabb1->o, aabb1->half, segment2->v[1]);
			}
			case GEOMETRY_BODY_AABB:
			{
				const GeometryAABB_t* aabb2 = (const GeometryAABB_t*)geo_data2;
				return AABB_Contain_AABB(aabb1->o, aabb1->half, aabb2->o, aabb2->half);
			}
			case GEOMETRY_BODY_OBB:
			{
				GeometryOBB_t obb1;
				mathOBBFromAABB(&obb1, aabb1->o, aabb1->half);
				return OBB_Contain_OBB(&obb1, (const GeometryOBB_t*)geo_data2);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				const GeometryPolygon_t* polygon2 = (const GeometryPolygon_t*)geo_data2;
				GeometryOBB_t obb1;
				mathOBBFromAABB(&obb1, aabb1->o, aabb1->half);
				return OBB_Contain_VerticeIndices(&obb1, (const CCTNum_t(*)[3])polygon2->v, polygon2->v_indices, polygon2->v_indices_cnt);
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				return AABB_Contain_Mesh(aabb1->o, aabb1->half, (const GeometryMesh_t*)geo_data2);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				const GeometrySphere_t* sphere2 = (const GeometrySphere_t*)geo_data2;
				GeometryOBB_t obb1;
				mathOBBFromAABB(&obb1, aabb1->o, aabb1->half);
				return OBB_Contain_Sphere(&obb1, sphere2->o, sphere2->radius);
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				const GeometryCapsule_t* capsule2 = (const GeometryCapsule_t*)geo_data2;
				GeometryOBB_t obb1;
				mathOBBFromAABB(&obb1, aabb1->o, aabb1->half);
				return OBB_Contain_Capsule(&obb1, capsule2);
			}
		}
	}
	else if (GEOMETRY_BODY_OBB == geo_type1) {
		const GeometryOBB_t* obb1 = (const GeometryOBB_t*)geo_data1;
		switch (geo_type2) {
			case GEOMETRY_BODY_POINT:
			{
				const CCTNum_t* point2 = (const CCTNum_t*)geo_data2;
				return OBB_Contain_Point(obb1, point2);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				return OBB_Contain_Point(obb1, segment2->v[0]) && OBB_Contain_Point(obb1, segment2->v[1]);
			}
			case GEOMETRY_BODY_AABB:
			{
				const GeometryAABB_t* aabb2 = (const GeometryAABB_t*)geo_data2;
				GeometryOBB_t obb2;
				mathOBBFromAABB(&obb2, aabb2->o, aabb2->half);
				return OBB_Contain_OBB(obb1, &obb2);
			}
			case GEOMETRY_BODY_OBB:
			{
				return OBB_Contain_OBB(obb1, (const GeometryOBB_t*)geo_data2);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				const GeometryPolygon_t* polygon2 = (const GeometryPolygon_t*)geo_data2;
				return OBB_Contain_VerticeIndices(obb1, (const CCTNum_t(*)[3])polygon2->v, polygon2->v_indices, polygon2->v_indices_cnt);
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				return OBB_Contain_Mesh(obb1, (const GeometryMesh_t*)geo_data2);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				const GeometrySphere_t* sphere2 = (const GeometrySphere_t*)geo_data2;
				return OBB_Contain_Sphere(obb1, sphere2->o, sphere2->radius);
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				return OBB_Contain_Capsule(obb1, (const GeometryCapsule_t*)geo_data2);
			}
		}
	}
	else if (GEOMETRY_BODY_SPHERE == geo_type1) {
		const GeometrySphere_t* sphere1 = (const GeometrySphere_t*)geo_data1;
		switch (geo_type2) {
			case GEOMETRY_BODY_POINT:
			{
				const CCTNum_t* point2 = (const CCTNum_t*)geo_data2;
				return Sphere_Contain_Point(sphere1->o, sphere1->radius, point2);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				return Sphere_Contain_Point(sphere1->o, sphere1->radius, segment2->v[0]) &&
					Sphere_Contain_Point(sphere1->o, sphere1->radius, segment2->v[1]);
			}
			case GEOMETRY_BODY_AABB:
			{
				const GeometryAABB_t* aabb2 = (const GeometryAABB_t*)geo_data2;
				CCTNum_t v[8][3];
				mathAABBVertices(aabb2->o, aabb2->half, v);
				return Sphere_Contain_VerticeIndices(sphere1->o, sphere1->radius, (const CCTNum_t(*)[3])v,
					Box_Vertice_Indices_Default, sizeof(Box_Vertice_Indices_Default) / sizeof(Box_Vertice_Indices_Default[0]));
			}
			case GEOMETRY_BODY_OBB:
			{
				const GeometryOBB_t* obb2 = (const GeometryOBB_t*)geo_data2;
				CCTNum_t v[8][3];
				mathOBBVertices(obb2, v);
				return Sphere_Contain_VerticeIndices(sphere1->o, sphere1->radius, (const CCTNum_t(*)[3])v,
					Box_Vertice_Indices_Default, sizeof(Box_Vertice_Indices_Default) / sizeof(Box_Vertice_Indices_Default[0]));
			}
			case GEOMETRY_BODY_POLYGON:
			{
				const GeometryPolygon_t* polygon2 = (const GeometryPolygon_t*)geo_data2;
				return Sphere_Contain_VerticeIndices(sphere1->o, sphere1->radius,
						(const CCTNum_t(*)[3])polygon2->v, polygon2->v_indices, polygon2->v_indices_cnt);
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				const GeometryMesh_t* mesh2 = (const GeometryMesh_t*)geo_data2;
				return Sphere_Contain_VerticeIndices(sphere1->o, sphere1->radius,
						(const CCTNum_t(*)[3])mesh2->v, mesh2->v_indices, mesh2->v_indices_cnt);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				const GeometrySphere_t* sphere2 = (GeometrySphere_t*)geo_data2;
				return Sphere_Contain_Sphere(sphere1->o, sphere1->radius, sphere2->o, sphere2->radius);
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				const GeometryCapsule_t* capsule2 = (GeometryCapsule_t*)geo_data2;
				return Sphere_Contain_Capsule(sphere1->o, sphere1->radius, capsule2);
			}
		}
	}
	else if (GEOMETRY_BODY_CONVEX_MESH == geo_type1) {
		const GeometryMesh_t* mesh1 = (const GeometryMesh_t*)geo_data1;
		switch (geo_type2) {
			case GEOMETRY_BODY_POINT:
			{
				const CCTNum_t* point2 = (const CCTNum_t*)geo_data2;
				return ConvexMesh_Contain_Point(mesh1, point2);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				return ConvexMesh_Contain_Point(mesh1, segment2->v[0]) && ConvexMesh_Contain_Point(mesh1, segment2->v[1]);
			}
			case GEOMETRY_BODY_AABB:
			{
				const GeometryAABB_t* aabb2 = (const GeometryAABB_t*)geo_data2;
				return ConvexMesh_Contain_AABB(mesh1, aabb2->o, aabb2->half);
			}
			case GEOMETRY_BODY_OBB:
			{
				const GeometryOBB_t* obb2 = (const GeometryOBB_t*)geo_data2;
				return ConvexMesh_Contain_OBB(mesh1, obb2);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				const GeometryPolygon_t* polygon2 = (const GeometryPolygon_t*)geo_data2;
				return ConvexMesh_Contain_VerticeIndices(mesh1, (const CCTNum_t(*)[3])polygon2->v, polygon2->v_indices, polygon2->v_indices_cnt);
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				return ConvexMesh_Contain_ConvexMesh(mesh1, (const GeometryMesh_t*)geo_data2);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				const GeometrySphere_t* sphere2 = (const GeometrySphere_t*)geo_data2;
				return ConvexMesh_Contain_Sphere(mesh1, sphere2->o, sphere2->radius);
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				return ConvexMesh_Contain_Capsule(mesh1, (const GeometryCapsule_t*)geo_data2);
			}
		}
	}
	else if (GEOMETRY_BODY_CAPSULE == geo_type1) {
		const GeometryCapsule_t* capsule1 = (const GeometryCapsule_t*)geo_data1;
		switch (geo_type2) {
			case GEOMETRY_BODY_POINT:
			{
				const CCTNum_t* point2 = (const CCTNum_t*)geo_data2;
				return Capsule_Contain_Point(capsule1, point2);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				return Capsule_Contain_Point(capsule1, segment2->v[0]) && Capsule_Contain_Point(capsule1, segment2->v[1]);
			}
			case GEOMETRY_BODY_AABB:
			{
				const GeometryAABB_t* aabb2 = (const GeometryAABB_t*)geo_data2;
				CCTNum_t v[8][3];
				mathAABBVertices(aabb2->o, aabb2->half, v);
				return Capsule_Contain_VerticeIndices(capsule1, (const CCTNum_t(*)[3])v,
					Box_Vertice_Indices_Default, sizeof(Box_Vertice_Indices_Default) / sizeof(Box_Vertice_Indices_Default[0]));
			}
			case GEOMETRY_BODY_OBB:
			{
				const GeometryOBB_t* obb2 = (const GeometryOBB_t*)geo_data2;
				CCTNum_t v[8][3];
				mathOBBVertices(obb2, v);
				return Capsule_Contain_VerticeIndices(capsule1, (const CCTNum_t(*)[3])v,
					Box_Vertice_Indices_Default, sizeof(Box_Vertice_Indices_Default) / sizeof(Box_Vertice_Indices_Default[0]));
			}
			case GEOMETRY_BODY_POLYGON:
			{
				const GeometryPolygon_t* polygon2 = (const GeometryPolygon_t*)geo_data2;
				return Capsule_Contain_VerticeIndices(capsule1, (const CCTNum_t(*)[3])polygon2->v, polygon2->v_indices, polygon2->v_indices_cnt);
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				const GeometryMesh_t* mesh2 = (const GeometryMesh_t*)geo_data2;
				return Capsule_Contain_VerticeIndices(capsule1, (const CCTNum_t(*)[3])mesh2->v, mesh2->v_indices, mesh2->v_indices_cnt);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				const GeometrySphere_t* sphere2 = (const GeometrySphere_t*)geo_data2;
				return Capsule_Contain_Sphere(capsule1, sphere2->o, sphere2->radius);
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				return Capsule_Contain_Capsule(capsule1, (const GeometryCapsule_t*)geo_data2);
			}
		}
	}
	else if (GEOMETRY_BODY_POLYGON == geo_type1) {
		const GeometryPolygon_t* polygon1 = (const GeometryPolygon_t*)geo_data1;
		switch (geo_type2) {
			case GEOMETRY_BODY_POINT:
			{
				const CCTNum_t* point2 = (const CCTNum_t*)geo_data2;
				return Polygon_Contain_Point(polygon1, point2);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				return Polygon_Contain_Segment(polygon1, segment2->v[0], segment2->v[1]);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				return Polygon_Contain_Polygon(polygon1, (const GeometryPolygon_t*)geo_data2);
			}
		}
	}
	else if (GEOMETRY_BODY_PLANE == geo_type1) {
		const GeometryPlane_t* plane1 = (const GeometryPlane_t*)geo_data1;
		switch (geo_type2) {
			case GEOMETRY_BODY_POINT:
			{
				const CCTNum_t* point2 = (const CCTNum_t*)geo_data2;
				return Plane_Contain_Point(plane1->v, plane1->normal, point2);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				return Plane_Contain_Point(plane1->v, plane1->normal, segment2->v[0]) &&
					Plane_Contain_Point(plane1->v, plane1->normal, segment2->v[1]);
			}
			case GEOMETRY_BODY_PLANE:
			{
				const GeometryPlane_t* plane2 = (const GeometryPlane_t*)geo_data2;
				return Plane_Contain_Plane(plane1->v, plane1->normal, plane2->v, plane2->normal);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				const GeometryPolygon_t* polygon2 = (const GeometryPolygon_t*)geo_data2;
				return Plane_Contain_Plane(plane1->v, plane1->normal, polygon2->v[polygon2->v_indices[0]], polygon2->normal);
			}
		}
	}
	else if (GEOMETRY_BODY_SEGMENT == geo_type1) {
		const GeometrySegment_t* segment1 = (const GeometrySegment_t*)geo_data1;
		switch (geo_type2) {
			case GEOMETRY_BODY_POINT:
			{
				const CCTNum_t* point2 = (const CCTNum_t*)geo_data2;
				return Segment_Contain_Point(segment1->v[0], segment1->v[1], point2);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				return Segment_Contain_Segment((const CCTNum_t(*)[3])segment1->v, (const CCTNum_t(*)[3])segment2->v);
			}
		}
	}
	else if (GEOMETRY_BODY_POINT == geo_type1) {
		if (GEOMETRY_BODY_POINT == geo_type2) {
			return mathVec3Equal((const CCTNum_t*)geo_data1, (const CCTNum_t*)geo_data2);
		}
	}
	return 0;
}

#ifdef __cplusplus
}
#endif
