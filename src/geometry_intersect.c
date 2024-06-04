//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/line_segment.h"
#include "../inc/plane.h"
#include "../inc/aabb.h"
#include "../inc/obb.h"
#include "../inc/polygon.h"
#include "../inc/mesh.h"
#include "../inc/geometry_api.h"
#include <math.h>
#include <stddef.h>

extern const unsigned int Box_Edge_Indices[24];

extern int Segment_Contain_Point(const CCTNum_t ls[2][3], const CCTNum_t p[3]);
extern int Sphere_Contain_Point(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t p[3]);
extern int Circle_Contain_Point(const GeometryCircle_t* circle, const CCTNum_t p[3]);
extern int Plane_Contain_Point(const CCTNum_t plane_v[3], const CCTNum_t plane_normal[3], const CCTNum_t p[3]);
extern int OBB_Contain_Point(const GeometryOBB_t* obb, const CCTNum_t p[3]);
extern int ConvexMesh_Contain_Point(const GeometryMesh_t* mesh, const CCTNum_t p[3]);
extern int Segment_Intersect_Segment(const CCTNum_t ls1[2][3], const CCTNum_t ls2[2][3], CCTNum_t p[3], int* line_mask);

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

int Plane_Intersect_Plane(const CCTNum_t v1[3], const CCTNum_t n1[3], const CCTNum_t v2[3], const CCTNum_t n2[3]) {
	CCTNum_t n[3];
	mathVec3Cross(n, n1, n2);
	if (!mathVec3IsZero(n)) {
		return 1;
	}
	return Plane_Contain_Point(v1, n1, v2) ? 2 : 0;
}

static int Circle_Intersect_Plane(const GeometryCircle_t* circle, const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTNum_t p[3], CCTNum_t line[3]) {
	CCTNum_t dir[3];
	if (!line) {
		line = dir;
	}
	mathVec3Cross(line, circle->normal, plane_n);
	if (!mathVec3IsZero(line)) {
		CCTNum_t v[3], d, dot, abs_d;
		mathVec3Normalized(line, line);
		mathVec3Cross(v, line, circle->normal);
		dot = mathVec3Dot(v, plane_n);
		mathPointProjectionPlane(circle->o, plane_v, plane_n, NULL, &d);
		d /= dot;
		if (p) {
			mathVec3Copy(p, circle->o);
			mathVec3AddScalar(p, v, d);
		}
		abs_d = CCTNum_abs(d);
		if (abs_d > circle->radius) {
			return 0;
		}
		if (abs_d >= circle->radius - CCT_EPSILON) {
			return 1;
		}
		return 3;
	}
	if (Plane_Contain_Point(plane_v, plane_n, circle->o)) {
		return 2;
	}
	return 0;
}

static int Circle_Intersect_Circle(const GeometryCircle_t* c1, const GeometryCircle_t* c2) {
	CCTNum_t p[3], line[3];
	int res = Circle_Intersect_Plane(c1, c2->o, c2->normal, p, line);
	if (0 == res) {
		return 0;
	}
	if (1 == res) {
		CCTNum_t lensq = mathVec3DistanceSq(p, c2->o);
		return lensq <= c2->radius * c2->radius;
	}
	if (2 == res) {
		CCTNum_t lensq = mathVec3DistanceSq(c1->o, c2->o);
		CCTNum_t rsum = c1->radius + c2->radius;
		return lensq <= rsum * rsum;
	}
	if (3 == res) {
		CCTNum_t closest_p[3], lensq, half;
		lensq = mathVec3DistanceSq(c1->o, p);
		half = CCTNum_sqrt(c1->radius * c1->radius - lensq);
		mathSegmentClosestPointTo_v2(p, line, half, c2->o, closest_p);
		lensq = mathVec3DistanceSq(closest_p, c2->o);
		return lensq <= c2->radius * c2->radius;
	}
	return 0;
}

int Segment_Intersect_Plane(const CCTNum_t ls[2][3], const CCTNum_t plane_v[3], const CCTNum_t plane_normal[3], CCTNum_t p[3]) {
	CCTNum_t d[2], lsdir[3], dot;
	mathVec3Sub(lsdir, ls[1], ls[0]);
	dot = mathVec3Dot(lsdir, plane_normal);
	if (dot <= CCT_EPSILON && dot >= CCT_EPSILON_NEGATE) {
		return Plane_Contain_Point(plane_v, plane_normal, ls[0]) ? 2 : 0;
	}
	mathPointProjectionPlane(ls[0], plane_v, plane_normal, NULL, &d[0]);
	mathPointProjectionPlane(ls[1], plane_v, plane_normal, NULL, &d[1]);
	if (d[0] > CCT_EPSILON && d[1] > CCT_EPSILON) {
		return 0;
	}
	if (d[0] < CCT_EPSILON_NEGATE && d[1] < CCT_EPSILON_NEGATE) {
		return 0;
	}
	if (d[0] <= CCT_EPSILON && d[0] >= CCT_EPSILON_NEGATE) {
		if (p) {
			mathVec3Copy(p, ls[0]);
		}
		return 1;
	}
	if (d[1] <= CCT_EPSILON && d[1] >= CCT_EPSILON_NEGATE) {
		if (p) {
			mathVec3Copy(p, ls[1]);
		}
		return 1;
	}
	if (p) {
		mathVec3Normalized(lsdir, lsdir);
		dot = mathVec3Dot(lsdir, plane_normal);
		mathVec3AddScalar(mathVec3Copy(p, ls[0]), lsdir, d[0] / dot);
	}
	return 1;
}

static int Segment_Intersect_Circle(const CCTNum_t ls[2][3], const GeometryCircle_t* circle, CCTNum_t p[3]) {
	int res;
	CCTNum_t v[3];
	if (!p) {
		p = v;
	}
	res = Segment_Intersect_Plane(ls, circle->o, circle->normal, p);
	if (1 == res) {
		CCTNum_t op[3], op_lensq;
		mathVec3Sub(op, p, circle->o);
		op_lensq = mathVec3LenSq(op);
		return op_lensq <= circle->radius * circle->radius;
	}
	if (2 == res) {
		CCTNum_t op[3], op_lensq;
		mathSegmentClosestPointTo(ls, circle->o, p);
		mathVec3Sub(op, p, circle->o);
		op_lensq = mathVec3LenSq(op);
		if (op_lensq <= circle->radius * circle->radius) {
			return 2;
		}
	}
	return 0;
}

static int Segment_Intersect_Polygon(const CCTNum_t ls[2][3], const GeometryPolygon_t* polygon, CCTNum_t p[3]) {
	int res, i;
	CCTNum_t point[3];
	if (!p) {
		p = point;
	}
	res = Segment_Intersect_Plane(ls, polygon->v[polygon->v_indices[0]], polygon->normal, p);
	if (0 == res) {
		return 0;
	}
	if (1 == res) {
		return Polygon_Contain_Point(polygon, p);
	}
	if (Polygon_Contain_Point(polygon, ls[0]) || Polygon_Contain_Point(polygon, ls[1])) {
		return 2;
	}
	for (i = 0; i < polygon->v_indices_cnt; ) {
		CCTNum_t edge[2][3];
		mathVec3Copy(edge[0], polygon->v[polygon->v_indices[i++]]);
		mathVec3Copy(edge[1], polygon->v[polygon->v_indices[i >= polygon->v_indices_cnt ? 0 : i]]);
		if (Segment_Intersect_Segment(ls, (const CCTNum_t(*)[3])edge, NULL, NULL)) {
			return 2;
		}
	}
	return 0;
}

static int Segment_Intersect_ConvexMesh(const CCTNum_t ls[2][3], const GeometryMesh_t* mesh) {
	unsigned int i;
	if (ConvexMesh_Contain_Point(mesh, ls[0])) {
		return 1;
	}
	if (ConvexMesh_Contain_Point(mesh, ls[1])) {
		return 1;
	}
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		if (Segment_Intersect_Polygon(ls, mesh->polygons + i, NULL)) {
			return 1;
		}
	}
	return 0;
}

static int Polygon_Intersect_Polygon(const GeometryPolygon_t* polygon1, const GeometryPolygon_t* polygon2) {
	int i;
	if (!Plane_Intersect_Plane(polygon1->v[polygon1->v_indices[0]], polygon1->normal, polygon2->v[polygon2->v_indices[0]], polygon2->normal)) {
		return 0;
	}
	for (i = 0; i < polygon1->v_indices_cnt; ) {
		CCTNum_t edge[2][3];
		mathVec3Copy(edge[0], polygon1->v[polygon1->v_indices[i++]]);
		mathVec3Copy(edge[1], polygon1->v[polygon1->v_indices[i >= polygon1->v_indices_cnt ? 0 : i]]);
		if (Segment_Intersect_Polygon((const CCTNum_t(*)[3])edge, polygon2, NULL)) {
			return 1;
		}
	}
	return 0;
}

int Polygon_Intersect_ConvexMesh(const GeometryPolygon_t* polygon, const GeometryMesh_t* mesh) {
	unsigned int i;
	for (i = 0; i < polygon->v_indices_cnt; ++i) {
		const CCTNum_t* p = polygon->v[polygon->v_indices[i]];
		if (ConvexMesh_Contain_Point(mesh, p)) {
			return 1;
		}
	}
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		if (Polygon_Intersect_Polygon(polygon, mesh->polygons + i)) {
			return 1;
		}
	}
	return 0;
}

static int Polygon_Intersect_Plane(const GeometryPolygon_t* polygon, const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTNum_t p[3]) {
	unsigned int i, has_gt0, has_le0, idx_0;
	if (!Plane_Intersect_Plane(polygon->v[polygon->v_indices[0]], polygon->normal, plane_v, plane_n)) {
		return 0;
	}
	idx_0 = has_gt0 = has_le0 = 0;
	for (i = 0; i < polygon->v_indices_cnt; ++i) {
		CCTNum_t d;
		mathPointProjectionPlane(polygon->v[polygon->v_indices[i]], plane_v, plane_n, NULL, &d);
		if (d > CCT_EPSILON) {
			if (has_le0) {
				return 2;
			}
			has_gt0 = 1;
		}
		else if (d < CCT_EPSILON_NEGATE) {
			if (has_gt0) {
				return 2;
			}
			has_le0 = 1;
		}
		else if (idx_0) {
			return 2;
		}
		else {
			if (p) {
				mathVec3Copy(p, polygon->v[polygon->v_indices[i]]);
			}
			idx_0 = 1;
		}
	}
	return idx_0;
}

static int Mesh_Intersect_Plane(const GeometryMesh_t* mesh, const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTNum_t p[3]) {
	unsigned int i, has_gt0 = 0, has_le0 = 0, idx_0 = 0;
	for (i = 0; i < mesh->v_indices_cnt; ++i) {
		CCTNum_t d;
		mathPointProjectionPlane(mesh->v[mesh->v_indices[i]], plane_v, plane_n, NULL, &d);
		if (d > CCT_EPSILON) {
			if (has_le0) {
				return 2;
			}
			has_gt0 = 1;
		}
		else if (d < CCT_EPSILON_NEGATE) {
			if (has_gt0) {
				return 2;
			}
			has_le0 = 1;
		}
		else if (idx_0) {
			return 2;
		}
		else {
			if (p) {
				mathVec3Copy(p, mesh->v[mesh->v_indices[i]]);
			}
			idx_0 = 1;
		}
	}
	return idx_0;
}

int Sphere_Intersect_Segment(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t ls[2][3], CCTNum_t p[3]) {
	CCTNum_t closest_v_lensq, radius_sq;
	CCTNum_t closest_p[3], closest_v[3];
	mathSegmentClosestPointTo(ls, o, closest_p);
	mathVec3Sub(closest_v, closest_p, o);
	closest_v_lensq = mathVec3LenSq(closest_v);
	radius_sq = radius * radius;
	if (closest_v_lensq < radius_sq - CCT_EPSILON) {
		return 2;
	}
	if (closest_v_lensq <= radius_sq + CCT_EPSILON) {
		if (p) {
			mathVec3Copy(p, closest_p);
		}
		return 1;
	}
	return 0;
}

int Sphere_Intersect_Plane(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t plane_v[3], const CCTNum_t plane_normal[3], CCTNum_t new_o[3], CCTNum_t* new_r) {
	CCTNum_t d;
	mathPointProjectionPlane(o, plane_v, plane_normal, new_o, &d);
	d = CCTNum_abs(d);
	if (d > radius + CCT_EPSILON) {
		if (new_r) {
			*new_r = CCTNum(0.0);
		}
		return 0;
	}
	if (d >= radius - CCT_EPSILON) {
		if (new_r) {
			*new_r = CCTNum(0.0);
		}
		return 1;
	}
	if (new_r) {
		*new_r = CCTNum_sqrt(radius * radius - d * d);
	}
	return 2;
}

static int Sphere_Intersect_Polygon(const CCTNum_t o[3], CCTNum_t radius, const GeometryPolygon_t* polygon, CCTNum_t p[3]) {
	int res, i;
	CCTNum_t point[3];
	if (!p) {
		p = point;
	}
	res = Sphere_Intersect_Plane(o, radius, polygon->v[polygon->v_indices[0]], polygon->normal, p, NULL);
	if (0 == res) {
		return 0;
	}
	if (Polygon_Contain_Point(polygon, p)) {
		return res;
	}
	for (i = 0; i < polygon->v_indices_cnt; ) {
		CCTNum_t edge[2][3];
		mathVec3Copy(edge[0], polygon->v[polygon->v_indices[i++]]);
		mathVec3Copy(edge[1], polygon->v[polygon->v_indices[i >= polygon->v_indices_cnt ? 0 : i]]);
		res = Sphere_Intersect_Segment(o, radius, (const CCTNum_t(*)[3])edge, p);
		if (res != 0) {
			return res;
		}
	}
	return 0;
}

static int Sphere_Intersect_ConvexMesh(const CCTNum_t o[3], CCTNum_t radius, const GeometryMesh_t* mesh, CCTNum_t p[3]) {
	unsigned int i;
	if (ConvexMesh_Contain_Point(mesh, o)) {
		return 1;
	}
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		int ret = Sphere_Intersect_Polygon(o, radius, mesh->polygons + i, p);
		if (ret) {
			return ret;
		}
	}
	return 0;
}

int Sphere_Intersect_OBB(const CCTNum_t o[3], CCTNum_t radius, const GeometryOBB_t* obb) {
	CCTNum_t v[3];
	mathOBBClosestPointTo(obb, o, v);
	mathVec3Sub(v, o, v);
	return mathVec3LenSq(v) <= radius * radius;
}

static int Sphere_Intersect_Sphere(const CCTNum_t o1[3], CCTNum_t r1, const CCTNum_t o2[3], CCTNum_t r2, CCTNum_t p[3]) {
	CCTNum_t o1o2[3];
	CCTNum_t o1o2_lensq, radius_sum_sq = (r1 + r2) * (r1 + r2);
	mathVec3Sub(o1o2, o2, o1);
	o1o2_lensq = mathVec3LenSq(o1o2);
	if (o1o2_lensq > radius_sum_sq + CCT_EPSILON) {
		return 0;
	}
	else if (o1o2_lensq < radius_sum_sq - CCT_EPSILON) {
		return 2;
	}
	if (p) {
		mathVec3Normalized(o1o2, o1o2);
		mathVec3AddScalar(mathVec3Copy(p, o1), o1o2, r1);
	}
	return 1;
}

static int Box_Intersect_Plane(const CCTNum_t vertices[8][3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTNum_t p[3]) {
	int i, has_gt0 = 0, has_le0 = 0, idx_0 = 0;
	for (i = 0; i < 8; ++i) {
		CCTNum_t d;
		mathPointProjectionPlane(vertices[i], plane_v, plane_n, NULL, &d);
		if (d > CCT_EPSILON) {
			if (has_le0) {
				return 2;
			}
			has_gt0 = 1;
		}
		else if (d < CCT_EPSILON_NEGATE) {
			if (has_gt0) {
				return 2;
			}
			has_le0 = 1;
		}
		else if (idx_0) {
			return 2;
		}
		else {
			if (p) {
				mathVec3Copy(p, vertices[i]);
			}
			idx_0 = 1;
		}
	}
	return idx_0;
}

static int OBB_Intersect_Plane(const GeometryOBB_t* obb, const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTNum_t p[3]) {
	CCTNum_t vertices[8][3];
	mathOBBVertices(obb, vertices);
	return Box_Intersect_Plane((const CCTNum_t(*)[3])vertices, plane_v, plane_n, p);
}

static int AABB_Intersect_Plane(const CCTNum_t o[3], const CCTNum_t half[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTNum_t p[3]) {
	CCTNum_t vertices[8][3];
	mathAABBVertices(o, half, vertices);
	return Box_Intersect_Plane((const CCTNum_t(*)[3])vertices, plane_v, plane_n, p);
}

static int AABB_Intersect_Sphere(const CCTNum_t aabb_o[3], const CCTNum_t aabb_half[3], const CCTNum_t sp_o[3], CCTNum_t sp_radius) {
	CCTNum_t closest_v[3];
	mathAABBClosestPointTo(aabb_o, aabb_half, sp_o, closest_v);
	mathVec3Sub(closest_v, closest_v, sp_o);
	return mathVec3LenSq(closest_v) <= sp_radius * sp_radius;
}

static int AABB_Intersect_Segment(const CCTNum_t o[3], const CCTNum_t half[3], const CCTNum_t ls[2][3]) {
	int i;
	GeometryPolygon_t polygon;
	if (AABB_Contain_Point(o, half, ls[0]) || AABB_Contain_Point(o, half, ls[1])) {
		return 1;
	}
	for (i = 0; i < 6; ++i) {
		GeometryRect_t rect;
		CCTNum_t p[4][3];
		mathAABBPlaneRect(o, half, i, &rect);
		mathRectToPolygon(&rect, &polygon, p);
		if (Segment_Intersect_Polygon(ls, &polygon, NULL)) {
			return 1;
		}
	}
	return 0;
}

int OBB_Intersect_Segment(const GeometryOBB_t* obb, const CCTNum_t ls[2][3]) {
	int i;
	if (OBB_Contain_Point(obb, ls[0]) || OBB_Contain_Point(obb, ls[1])) {
		return 1;
	}
	for (i = 0; i < 6; ++i) {
		GeometryPolygon_t polygon;
		GeometryRect_t rect;
		CCTNum_t p[4][3];

		mathOBBPlaneRect(obb, i, &rect);
		mathRectToPolygon(&rect, &polygon, p);
		if (Segment_Intersect_Polygon(ls, &polygon, NULL)) {
			return 1;
		}
	}
	return 0;
}

int OBB_Intersect_Polygon(const GeometryOBB_t* obb, const GeometryPolygon_t* polygon, CCTNum_t p[3]) {
	int res, i;
	CCTNum_t point[3], obb_vertices[8][3];
	if (!p) {
		p = point;
	}
	mathOBBVertices(obb, obb_vertices);
	res = Box_Intersect_Plane((const CCTNum_t(*)[3])obb_vertices, polygon->v[polygon->v_indices[0]], polygon->normal, p);
	if (0 == res) {
		return 0;
	}
	if (1 == res) {
		return Polygon_Contain_Point(polygon, p);
	}
	mathOBBVertices(obb, obb_vertices);
	for (i = 0; i < sizeof(Box_Edge_Indices) / sizeof(Box_Edge_Indices[0]); i += 2) {
		CCTNum_t edge[2][3];
		mathVec3Copy(edge[0], obb_vertices[Box_Edge_Indices[i]]);
		mathVec3Copy(edge[1], obb_vertices[Box_Edge_Indices[i+1]]);
		if (Segment_Intersect_Polygon((const CCTNum_t(*)[3])edge, polygon, NULL)) {
			return 2;
		}
	}
	for (i = 0; i < polygon->v_indices_cnt; ) {
		CCTNum_t edge[2][3];
		mathVec3Copy(edge[0], polygon->v[polygon->v_indices[i++]]);
		mathVec3Copy(edge[1], polygon->v[polygon->v_indices[i >= polygon->v_indices_cnt ? 0 : i]]);
		if (OBB_Intersect_Segment(obb, (const CCTNum_t(*)[3])edge)) {
			return 2;
		}
	}
	return 0;
}

int OBB_Intersect_OBB(const GeometryOBB_t* obb0, const GeometryOBB_t* obb1) {
	/* these code is copy from PhysX-3.4 */
	CCTNum_t v[3], T[3];
	CCTNum_t R[3][3], FR[3][3], ra, rb, t;
	const CCTNum_t* e0 = obb0->half, * e1 = obb1->half;
	int i;
	mathVec3Sub(v, obb1->o, obb0->o);
	mathVec3Set(T, mathVec3Dot(v, obb0->axis[0]), mathVec3Dot(v, obb0->axis[1]), mathVec3Dot(v, obb0->axis[2]));

	for (i = 0; i < 3; ++i) {
		int k;
		for (k = 0; k < 3; ++k) {
			R[i][k] = mathVec3Dot(obb0->axis[i], obb1->axis[k]);
			FR[i][k] = CCTNum(1e-6) + CCTNum_abs(R[i][k]);
		}
	}

	for (i = 0; i < 3; ++i) {
		ra = e0[i];
		rb = e1[0] * FR[i][0] + e1[1] * FR[i][1] + e1[2] * FR[i][2];
		t = CCTNum_abs(T[i]);
		if (t > ra + rb) {
			return 0;
		}
	}

	for (i = 0; i < 3; ++i) {
		ra = e0[0] * FR[0][i] + e0[1] * FR[1][i] + e0[2] * FR[2][i];
		rb = e1[i];
		t = T[0] * R[0][i] + T[1] * R[1][i] + T[2] * R[2][i];
		t = CCTNum_abs(t);
		if (t > ra + rb) {
			return 0;
		}
	}

	if (1) {
		/* 9 cross products */

		//L = A0 x B0
		ra = e0[1] * FR[2][0] + e0[2] * FR[1][0];
		rb = e1[1] * FR[0][2] + e1[2] * FR[0][1];
		t = T[2] * R[1][0] - T[1] * R[2][0];
		t = CCTNum_abs(t);
		if (t > ra + rb) {
			return 0;
		}
		//L = A0 x B1
		ra = e0[1] * FR[2][1] + e0[2] * FR[1][1];
		rb = e1[0] * FR[0][2] + e1[2] * FR[0][0];
		t = T[2] * R[1][1] - T[1] * R[2][1];
		t = CCTNum_abs(t);
		if (t > ra + rb) {
			return 0;
		}
		//L = A0 x B2
		ra = e0[1] * FR[2][2] + e0[2] * FR[1][2];
		rb = e1[0] * FR[0][1] + e1[1] * FR[0][0];
		t = T[2] * R[1][2] - T[1] * R[2][2];
		t = CCTNum_abs(t);
		if (t > ra + rb) {
			return 0;
		}
		//L = A1 x B0
		ra = e0[0] * FR[2][0] + e0[2] * FR[0][0];
		rb = e1[1] * FR[1][2] + e1[2] * FR[1][1];
		t = T[0] * R[2][0] - T[2] * R[0][0];
		t = CCTNum_abs(t);
		if (t > ra + rb) {
			return 0;
		}
		//L = A1 x B1
		ra = e0[0] * FR[2][1] + e0[2] * FR[0][1];
		rb = e1[0] * FR[1][2] + e1[2] * FR[1][0];
		t = T[0] * R[2][1] - T[2] * R[0][1];
		t = CCTNum_abs(t);
		if (t > ra + rb) {
			return 0;
		}
		//L = A1 x B2
		ra = e0[0] * FR[2][2] + e0[2] * FR[0][2];
		rb = e1[0] * FR[1][1] + e1[1] * FR[1][0];
		t = T[0] * R[2][2] - T[2] * R[0][2];
		t = CCTNum_abs(t);
		if (t > ra + rb) {
			return 0;
		}
		//L = A2 x B0
		ra = e0[0] * FR[1][0] + e0[1] * FR[0][0];
		rb = e1[1] * FR[2][2] + e1[2] * FR[2][1];
		t = T[1] * R[0][0] - T[0] * R[1][0];
		t = CCTNum_abs(t);
		if (t > ra + rb) {
			return 0;
		}
		//L = A2 x B1
		ra = e0[0] * FR[1][1] + e0[1] * FR[0][1];
		rb = e1[0] * FR[2][2] + e1[2] * FR[2][0];
		t = T[1] * R[0][1] - T[0] * R[1][1];
		t = CCTNum_abs(t);
		if (t > ra + rb) {
			return 0;
		}
		//L = A2 x B2
		ra = e0[0] * FR[1][2] + e0[1] * FR[0][2];
		rb = e1[0] * FR[2][1] + e1[1] * FR[2][0];
		t = T[1] * R[0][2] - T[0] * R[1][2];
		t = CCTNum_abs(t);
		if (t > ra + rb) {
			return 0;
		}
	}
	return 1;
}

static int ConvexMesh_HasAny_OBBVertices(const GeometryMesh_t* mesh, const GeometryOBB_t* obb) {
	CCTNum_t v[8][3];
	unsigned int i;
	mathOBBVertices(obb, v);
	for (i = 0; i < 8; ++i) {
		if (ConvexMesh_Contain_Point(mesh, v[i])) {
			return 1;
		}
	}
	return 0;
}

static int OBB_Intersect_ConvexMesh(const GeometryOBB_t* obb, const GeometryMesh_t* mesh) {
	unsigned int i;
	if (ConvexMesh_HasAny_OBBVertices(mesh, obb)) {
		return 1;
	}
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		if (OBB_Intersect_Polygon(obb, mesh->polygons + i, NULL)) {
			return 1;
		}
	}
	return 0;
}

static int ConvexMesh_Intersect_ConvexMesh(const GeometryMesh_t* mesh1, const GeometryMesh_t* mesh2) {
	unsigned int i;
	for (i = 0; i < mesh1->v_indices_cnt; ++i) {
		const CCTNum_t* p = mesh1->v[mesh1->v_indices[i]];
		if (ConvexMesh_Contain_Point(mesh2, p)) {
			return 1;
		}
	}
	for (i = 0; i < mesh2->v_indices_cnt; ++i) {
		const CCTNum_t* p = mesh2->v[mesh2->v_indices[i]];
		if (ConvexMesh_Contain_Point(mesh1, p)) {
			return 1;
		}
	}
	for (i = 0; i < mesh1->polygons_cnt; ++i) {
		unsigned int j;
		for (j = 0; j < mesh2->polygons_cnt; ++j) {
			if (Polygon_Intersect_Polygon(mesh1->polygons + i, mesh2->polygons + j)) {
				return 1;
			}
		}
	}
	return 0;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
extern "C" {
#endif

int mathGeometryIntersect(const GeometryBodyRef_t* one, const GeometryBodyRef_t* two) {
	if (one->data == two->data) {
		return 1;
	}
	if (GEOMETRY_BODY_POINT == one->type) {
		switch (two->type) {
			case GEOMETRY_BODY_POINT:
			{
				return mathVec3Equal(one->point, two->point);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				return Segment_Contain_Point((const CCTNum_t(*)[3])two->segment->v, one->point);
			}
			case GEOMETRY_BODY_PLANE:
			{
				return Plane_Contain_Point(two->plane->v, two->plane->normal, one->point);
			}
			case GEOMETRY_BODY_AABB:
			{
				return AABB_Contain_Point(two->aabb->o, two->aabb->half, one->point);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				return Sphere_Contain_Point(two->sphere->o, two->sphere->radius, one->point);
			}
			case GEOMETRY_BODY_OBB:
			{
				return OBB_Contain_Point(two->obb, one->point);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				return Polygon_Contain_Point(two->polygon, one->point);
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				return ConvexMesh_Contain_Point(two->mesh, one->point);
			}
		}
	}
	else if (GEOMETRY_BODY_SEGMENT == one->type) {
		const CCTNum_t(*one_segment_v)[3] = (const CCTNum_t(*)[3])one->segment->v;
		switch (two->type) {
			case GEOMETRY_BODY_POINT:
			{
				return Segment_Contain_Point(one_segment_v, two->point);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				return Segment_Intersect_Segment(one_segment_v, (const CCTNum_t(*)[3])two->segment->v, NULL, NULL);
			}
			case GEOMETRY_BODY_PLANE:
			{
				return Segment_Intersect_Plane(one_segment_v, two->plane->v, two->plane->normal, NULL);
			}
			case GEOMETRY_BODY_AABB:
			{
				return AABB_Intersect_Segment(two->aabb->o, two->aabb->half, one_segment_v);
			}
			case GEOMETRY_BODY_OBB:
			{
				return OBB_Intersect_Segment(two->obb, one_segment_v);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				return Sphere_Intersect_Segment(two->sphere->o, two->sphere->radius, one_segment_v, NULL);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				return Segment_Intersect_Polygon(one_segment_v, two->polygon, NULL);
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				return Segment_Intersect_ConvexMesh(one_segment_v, two->mesh);
			}
		}
	}
	else if (GEOMETRY_BODY_AABB == one->type) {
		switch (two->type) {
			case GEOMETRY_BODY_POINT:
			{
				return AABB_Contain_Point(one->aabb->o, one->aabb->half, two->point);
			}
			case GEOMETRY_BODY_AABB:
			{
				return AABB_Intersect_AABB(one->aabb->o, one->aabb->half, two->aabb->o, two->aabb->half);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				return AABB_Intersect_Sphere(one->aabb->o, one->aabb->half, two->sphere->o, two->sphere->radius);
			}
			case GEOMETRY_BODY_PLANE:
			{
				return AABB_Intersect_Plane(one->aabb->o, one->aabb->half, two->plane->v, two->plane->normal, NULL);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				return AABB_Intersect_Segment(one->aabb->o, one->aabb->half, (const CCTNum_t(*)[3])two->segment->v);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				GeometryOBB_t one_obb;
				mathOBBFromAABB(&one_obb, one->aabb->o, one->aabb->half);
				return OBB_Intersect_Polygon(&one_obb, two->polygon, NULL);
			}
			case GEOMETRY_BODY_OBB:
			{
				GeometryOBB_t one_obb;
				mathOBBFromAABB(&one_obb, one->aabb->o, one->aabb->half);
				return OBB_Intersect_OBB(&one_obb, two->obb);
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				GeometryOBB_t one_obb;
				mathOBBFromAABB(&one_obb, one->aabb->o, one->aabb->half);
				return OBB_Intersect_ConvexMesh(&one_obb, two->mesh);
			}
		}
	}
	else if (GEOMETRY_BODY_SPHERE == one->type) {
		switch (two->type) {
			case GEOMETRY_BODY_POINT:
			{
				return Sphere_Contain_Point(one->sphere->o, one->sphere->radius, two->point);
			}
			case GEOMETRY_BODY_AABB:
			{
				return AABB_Intersect_Sphere(two->aabb->o, two->aabb->half, one->sphere->o, one->sphere->radius);
			}
			case GEOMETRY_BODY_OBB:
			{
				return Sphere_Intersect_OBB(one->sphere->o, one->sphere->radius, two->obb);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				return Sphere_Intersect_Sphere(one->sphere->o, one->sphere->radius, two->sphere->o, two->sphere->radius, NULL);
			}
			case GEOMETRY_BODY_PLANE:
			{
				return Sphere_Intersect_Plane(one->sphere->o, one->sphere->radius, two->plane->v, two->plane->normal, NULL, NULL);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				return Sphere_Intersect_Segment(one->sphere->o, one->sphere->radius, (const CCTNum_t(*)[3])two->segment->v, NULL);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				return Sphere_Intersect_Polygon(one->sphere->o, one->sphere->radius, two->polygon, NULL);
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				return Sphere_Intersect_ConvexMesh(one->sphere->o, one->sphere->radius, two->mesh, NULL);
			}
		}
	}
	else if (GEOMETRY_BODY_PLANE == one->type) {
		switch (two->type) {
			case GEOMETRY_BODY_POINT:
			{
				return Plane_Contain_Point(one->plane->v, one->plane->normal, two->point);
			}
			case GEOMETRY_BODY_AABB:
			{
				return AABB_Intersect_Plane(two->aabb->o, two->aabb->half, one->plane->v, one->plane->normal, NULL);
			}
			case GEOMETRY_BODY_OBB:
			{
				return OBB_Intersect_Plane(two->obb, one->plane->v, one->plane->normal, NULL);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				return Sphere_Intersect_Plane(two->sphere->o, two->sphere->radius, one->plane->v, one->plane->normal, NULL, NULL);
			}
			case GEOMETRY_BODY_PLANE:
			{
				return Plane_Intersect_Plane(one->plane->v, one->plane->normal, two->plane->v, two->plane->normal);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				return Segment_Intersect_Plane((const CCTNum_t(*)[3])two->segment->v, one->plane->v, one->plane->normal, NULL);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				return Polygon_Intersect_Plane(two->polygon, one->plane->v, one->plane->normal, NULL);
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				return Mesh_Intersect_Plane(two->mesh, one->plane->v, one->plane->normal, NULL);
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
				return Segment_Intersect_Polygon((const CCTNum_t(*)[3])two->segment->v, one->polygon, NULL);
			}
			case GEOMETRY_BODY_PLANE:
			{
				return Polygon_Intersect_Plane(one->polygon, two->plane->v, two->plane->normal, NULL);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				return Sphere_Intersect_Polygon(two->sphere->o, two->sphere->radius, one->polygon, NULL);
			}
			case GEOMETRY_BODY_AABB:
			{
				GeometryOBB_t two_obb;
				mathOBBFromAABB(&two_obb, two->aabb->o, two->aabb->half);
				return OBB_Intersect_Polygon(&two_obb, one->polygon, NULL);
			}
			case GEOMETRY_BODY_OBB:
			{
				return OBB_Intersect_Polygon(two->obb, one->polygon, NULL);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				return Polygon_Intersect_Polygon(one->polygon, two->polygon);
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				return Polygon_Intersect_ConvexMesh(one->polygon, two->mesh);
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
				return OBB_Intersect_Segment(one->obb, (const CCTNum_t(*)[3])two->segment->v);
			}
			case GEOMETRY_BODY_PLANE:
			{
				return OBB_Intersect_Plane(one->obb, two->plane->v, two->plane->normal, NULL);
			}
			case GEOMETRY_BODY_OBB:
			{
				return OBB_Intersect_OBB(one->obb, two->obb);
			}
			case GEOMETRY_BODY_AABB:
			{
				GeometryOBB_t two_obb;
				mathOBBFromAABB(&two_obb, two->aabb->o, two->aabb->half);
				return OBB_Intersect_OBB(one->obb, &two_obb);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				return Sphere_Intersect_OBB(two->sphere->o, two->sphere->radius, one->obb);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				return OBB_Intersect_Polygon(one->obb, two->polygon, NULL);
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				return OBB_Intersect_ConvexMesh(one->obb, two->mesh);
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
				return Segment_Intersect_ConvexMesh((const CCTNum_t(*)[3])two->segment->v, one->mesh);
			}
			case GEOMETRY_BODY_PLANE:
			{
				return Mesh_Intersect_Plane(one->mesh, two->plane->v, two->plane->normal, NULL);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				return Sphere_Intersect_ConvexMesh(two->sphere->o, two->sphere->radius, one->mesh, NULL);
			}
			case GEOMETRY_BODY_AABB:
			{
				GeometryOBB_t two_obb;
				mathOBBFromAABB(&two_obb, two->aabb->o, two->aabb->half);
				return OBB_Intersect_ConvexMesh(&two_obb, one->mesh);
			}
			case GEOMETRY_BODY_OBB:
			{
				return OBB_Intersect_ConvexMesh(two->obb, one->mesh);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				return Polygon_Intersect_ConvexMesh(two->polygon, one->mesh);
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				return ConvexMesh_Intersect_ConvexMesh(one->mesh, two->mesh);
			}
		}
	}
	return 0;
}

#ifdef __cplusplus
}
#endif
