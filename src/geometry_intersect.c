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

extern const CCTNum_t AABB_Axis[3][3];
extern const unsigned int Box_Edge_Indices[24];
extern const unsigned int Box_Vertice_Indices_Default[8];

extern int Segment_Contain_Point(const CCTNum_t ls[2][3], const CCTNum_t p[3]);
extern int Sphere_Contain_Point(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t p[3]);
extern int Plane_Contain_Point(const CCTNum_t plane_v[3], const CCTNum_t plane_normal[3], const CCTNum_t p[3]);
extern int Polygon_Contain_Point(const GeometryPolygon_t* polygon, const CCTNum_t p[3]);
extern int OBB_Contain_Point(const GeometryOBB_t* obb, const CCTNum_t p[3]);
extern int ConvexMesh_Contain_Point(const GeometryMesh_t* mesh, const CCTNum_t p[3]);
extern int Segment_Intersect_Segment(const CCTNum_t ls1[2][3], const CCTNum_t ls2[2][3], CCTNum_t p[3], int* line_mask);

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

static int Plane_Intersect_Plane(const CCTNum_t v1[3], const CCTNum_t n1[3], const CCTNum_t v2[3], const CCTNum_t n2[3]) {
	CCTNum_t n[3];
	mathVec3Cross(n, n1, n2);
	if (!mathVec3IsZero(n)) {
		return 1;
	}
	return Plane_Contain_Point(v1, n1, v2) ? 2 : 0;
}

static int Circle_Intersect_Plane(const GeometryCircle_t* circle, const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTNum_t ls_center[3], CCTNum_t ls_dir[3]) {
	CCTNum_t dir[3];
	if (!ls_dir) {
		ls_dir = dir;
	}
	mathVec3Cross(ls_dir, circle->normal, plane_n);
	if (!mathVec3IsZero(ls_dir)) {
		CCTNum_t v[3], d, dot, abs_d;
		mathVec3Normalized(ls_dir, ls_dir);
		mathVec3Cross(v, ls_dir, circle->normal);
		dot = mathVec3Dot(v, plane_n);
		d = mathPointProjectionPlane(circle->o, plane_v, plane_n, NULL);
		d /= dot;
		if (ls_center) {
			mathVec3Copy(ls_center, circle->o);
			mathVec3AddScalar(ls_center, v, d);
		}
		abs_d = CCTNum_abs(d);
		if (abs_d > circle->radius) {
			return 0;
		}
		if (abs_d < circle->radius) {
			return 3;
		}
		return 1;
	}
	if (ls_center) {
		mathVec3Copy(ls_center, circle->o);
	}
	mathVec3Set(ls_dir, CCTNums_3(0.0, 0.0, 0.0));
	if (Plane_Contain_Point(plane_v, plane_n, circle->o)) {
		return 2;
	}
	return 0;
}

static int Circle_Intersect_Circle(const GeometryCircle_t* c1, const GeometryCircle_t* c2) {
	CCTNum_t ls_center[3], ls_dir[3];
	int res = Circle_Intersect_Plane(c1, c2->o, c2->normal, ls_center, ls_dir);
	if (0 == res) {
		return 0;
	}
	if (1 == res) {
		CCTNum_t lensq = mathVec3DistanceSq(ls_center, c2->o);
		return lensq <= CCTNum_sq(c2->radius);
	}
	if (2 == res) {
		CCTNum_t lensq = mathVec3DistanceSq(c1->o, c2->o);
		CCTNum_t rsum = c1->radius + c2->radius;
		return lensq <= CCTNum_sq(rsum);
	}
	if (3 == res) {
		CCTNum_t closest_p[3], lensq, half;
		lensq = mathVec3DistanceSq(c1->o, ls_center);
		half = CCTNum_sqrt(CCTNum_sq(c1->radius) - lensq);
		mathSegmentClosestPointTo_v2(ls_center, ls_dir, half, c2->o, closest_p);
		lensq = mathVec3DistanceSq(closest_p, c2->o);
		return lensq <= CCTNum_sq(c2->radius);
	}
	return 0;
}

int Vertices_Intersect_Plane(const CCTNum_t(*v)[3], const unsigned int* v_indices, unsigned int v_indices_cnt, const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTNum_t* p_min_d, unsigned int* p_v_indices_idx) {
	int i, has_gt0 = 0, has_le0 = 0, has_eq0 = 0, idx_min = -1;
	CCTNum_t min_d;
	for (i = 0; i < v_indices_cnt; ++i) {
		CCTNum_t d, abs_d, abs_min_d;
		d = mathPointProjectionPlane(v[v_indices[i]], plane_v, plane_n, NULL);
		if (d > CCTNum(0.0)) {
			if (has_le0) {
				return 2;
			}
			has_gt0 = 1;
			abs_d = CCTNum_abs(d);
		}
		else if (d < CCTNum(0.0)) {
			if (has_gt0) {
				return 2;
			}
			has_le0 = 1;
			abs_d = CCTNum_abs(d);
		}
		else {
			d = abs_d = CCTNum(0.0);
			has_eq0 = 1;
		}

		if (0 == i) {
			min_d = d;
			abs_min_d = abs_d;
			idx_min = 0;
		}
		else if (abs_min_d > abs_d) {
			min_d = d;
			abs_min_d = abs_d;
			idx_min = i;
		}
		else if (abs_min_d == abs_d) {
			idx_min = -1;
		}
	}
	if (p_min_d) {
		*p_min_d = min_d;
	}
	if (p_v_indices_idx) {
		*p_v_indices_idx = idx_min;
	}
	return has_eq0;
}

int Segment_Intersect_Plane(const CCTNum_t ls[2][3], const CCTNum_t plane_v[3], const CCTNum_t plane_normal[3], CCTNum_t intersect_p[3], CCTNum_t d[3]) {
	CCTNum_t temp_d[3];
	if (!d) {
		d = temp_d;
	}
	d[0] = mathPointProjectionPlane(ls[0], plane_v, plane_normal, NULL);
	d[1] = mathPointProjectionPlane(ls[1], plane_v, plane_normal, NULL);
	if (d[0] == d[1]) {
		d[2] = d[0];
		if (d[0] != CCTNum(0.0)) {
			return 0;
		}
		return 2;
	}
	if (d[0] > CCTNum(0.0) && d[1] > CCTNum(0.0)) {
		d[2] = (d[0] < d[1] ? d[0] : d[1]);
		return 0;
	}
	if (d[0] < CCTNum(0.0) && d[1] < CCTNum(0.0)) {
		d[2] = (d[0] < d[1] ? d[1] : d[0]);
		return 0;
	}
	d[2] = CCTNum(0.0);
	if (CCTNum(0.0) == d[0]) {
		if (intersect_p) {
			mathVec3Copy(intersect_p, ls[0]);
		}
		return 1;
	}
	if (CCTNum(0.0) == d[1]) {
		if (intersect_p) {
			mathVec3Copy(intersect_p, ls[1]);
		}
		return 1;
	}
	if (intersect_p) {
		CCTNum_t lsdir[3], dot;
		mathVec3Sub(lsdir, ls[1], ls[0]);
		mathVec3Normalized(lsdir, lsdir);
		dot = mathVec3Dot(lsdir, plane_normal);
		mathVec3Copy(intersect_p, ls[0]);
		mathVec3AddScalar(intersect_p, lsdir, d[0] / dot);
	}
	return 1;
}

static int Segment_Intersect_Circle(const CCTNum_t ls[2][3], const GeometryCircle_t* circle, CCTNum_t p[3]) {
	int res;
	CCTNum_t v[3];
	if (!p) {
		p = v;
	}
	res = Segment_Intersect_Plane(ls, circle->o, circle->normal, p, NULL);
	if (1 == res) {
		CCTNum_t op[3], op_lensq;
		mathVec3Sub(op, p, circle->o);
		op_lensq = mathVec3LenSq(op);
		return op_lensq <= CCTNum_sq(circle->radius);
	}
	if (2 == res) {
		CCTNum_t op[3], op_lensq;
		mathSegmentClosestPointTo(ls, circle->o, p);
		mathVec3Sub(op, p, circle->o);
		op_lensq = mathVec3LenSq(op);
		if (op_lensq <= CCTNum_sq(circle->radius)) {
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
	res = Segment_Intersect_Plane(ls, polygon->v[polygon->v_indices[0]], polygon->normal, p, NULL);
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

int Segment_Intersect_ConvexMesh(const CCTNum_t ls[2][3], const GeometryMesh_t* mesh) {
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

int Polygon_Intersect_Polygon(const GeometryPolygon_t* polygon1, const GeometryPolygon_t* polygon2) {
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
	for (i = 0; i < polygon2->v_indices_cnt; ) {
		CCTNum_t edge[2][3];
		mathVec3Copy(edge[0], polygon2->v[polygon2->v_indices[i++]]);
		mathVec3Copy(edge[1], polygon2->v[polygon2->v_indices[i >= polygon2->v_indices_cnt ? 0 : i]]);
		if (Segment_Intersect_Polygon((const CCTNum_t(*)[3])edge, polygon1, NULL)) {
			return 1;
		}
	}
	return 0;
}

static int Polygon_Intersect_Circle(const GeometryPolygon_t* polygon, const GeometryCircle_t* circle) {
	CCTNum_t p[3], line[3];
	int res = Circle_Intersect_Plane(circle, polygon->v[polygon->v_indices[0]], polygon->normal, p, line);
	if (0 == res) {
		return 0;
	}
	if (1 == res) {
		return Polygon_Contain_Point(polygon, p);
	}
	if (2 == res) {
		unsigned int i;
		CCTNum_t radius_sq;
		if (Polygon_Contain_Point(polygon, circle->o)) {
			return 1;
		}
		radius_sq = CCTNum_sq(circle->radius);
		for (i = 0; i < polygon->v_indices_cnt; ) {
			CCTNum_t edge[2][3], closest_p[3], lensq;
			mathVec3Copy(edge[0], polygon->v[polygon->v_indices[i++]]);
			mathVec3Copy(edge[1], polygon->v[polygon->v_indices[i >= polygon->v_indices_cnt ? 0 : i]]);
			mathSegmentClosestPointTo((const CCTNum_t(*)[3])edge, circle->o, closest_p);
			lensq = mathVec3DistanceSq(closest_p, circle->o);
			if (lensq <= radius_sq) {
				return 1;
			}
		}
	}
	if (3 == res) {
		unsigned int i;
		CCTNum_t lensq, half, ls[2][3];
		if (Polygon_Contain_Point(polygon, p)) {
			return 1;
		}
		lensq = mathVec3DistanceSq(circle->o, p);
		half = CCTNum_sqrt(CCTNum_sq(circle->radius) - lensq);
		mathVec3Copy(ls[0], p);
		mathVec3AddScalar(ls[0], line, half);
		if (Polygon_Contain_Point(polygon, ls[0])) {
			return 1;
		}
		mathVec3Copy(ls[1], p);
		mathVec3SubScalar(ls[1], line, half);
		if (Polygon_Contain_Point(polygon, ls[1])) {
			return 1;
		}
		for (i = 0; i < polygon->v_indices_cnt; ) {
			CCTNum_t edge[2][3];
			mathVec3Copy(edge[0], polygon->v[polygon->v_indices[i++]]);
			mathVec3Copy(edge[1], polygon->v[polygon->v_indices[i >= polygon->v_indices_cnt ? 0 : i]]);
			if (Segment_Intersect_Segment((const CCTNum_t(*)[3])ls, (const CCTNum_t(*)[3])edge, NULL, NULL)) {
				return 1;
			}
		}
	}
	return 0;
}

int ConvexMesh_Intersect_Circle(const GeometryMesh_t* mesh, const GeometryCircle_t* circle) {
	int res;
	unsigned int i, v_indices_idx = -1;
	res = Vertices_Intersect_Plane((const CCTNum_t(*)[3])mesh->v, mesh->v_indices, mesh->v_indices_cnt, circle->o, circle->normal, NULL, &v_indices_idx);
	if (0 == res) {
		return 0;
	}
	if (1 == res && v_indices_idx != -1) {
		CCTNum_t lensq = mathVec3DistanceSq(circle->o, mesh->v[mesh->v_indices[v_indices_idx]]);
		return lensq <= CCTNum_sq(circle->radius);
	}
	if (ConvexMesh_Contain_Point(mesh, circle->o)) {
		return 1;
	}
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		if (Polygon_Intersect_Circle(mesh->polygons + i, circle)) {
			return 2;
		}
	}
	return 0;
}

int ConvexMesh_Intersect_Polygon(const GeometryMesh_t* mesh, const GeometryPolygon_t* polygon) {
	int res;
	unsigned int i, v_indices_idx = -1;
	res = Vertices_Intersect_Plane((const CCTNum_t(*)[3])mesh->v, mesh->v_indices, mesh->v_indices_cnt, polygon->v[polygon->v_indices[0]], polygon->normal, NULL, &v_indices_idx);
	if (0 == res) {
		return 0;
	}
	if (1 == res && v_indices_idx != -1) {
		return Polygon_Contain_Point(polygon, mesh->v[mesh->v_indices[v_indices_idx]]);
	}
	for (i = 0; i < mesh->edge_indices_cnt; ) {
		CCTNum_t edge[2][3];
		mathVec3Copy(edge[0], mesh->v[mesh->edge_indices[i++]]);
		mathVec3Copy(edge[1], mesh->v[mesh->edge_indices[i++]]);
		if (Segment_Intersect_Polygon((const CCTNum_t(*)[3])edge, polygon, NULL)) {
			return 2;
		}
	}
	for (i = 0; i < polygon->v_indices_cnt; ) {
		CCTNum_t edge[2][3];
		mathVec3Copy(edge[0], polygon->v[polygon->v_indices[i++]]);
		mathVec3Copy(edge[1], polygon->v[polygon->v_indices[i >= polygon->v_indices_cnt ? 0 : i]]);
		if (Segment_Intersect_ConvexMesh((const CCTNum_t(*)[3])edge, mesh)) {
			return 2;
		}
	}
	return 0;
}

int Sphere_Intersect_Segment(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t ls[2][3], CCTNum_t closest_p[3]) {
	CCTNum_t closest_v_lensq, radius_sq;
	CCTNum_t temp_closest_p[3];
	if (!closest_p) {
		closest_p = temp_closest_p;
	}
	mathSegmentClosestPointTo(ls, o, closest_p);
	closest_v_lensq = mathVec3DistanceSq(closest_p, o);
	radius_sq = CCTNum_sq(radius);
	if (closest_v_lensq > radius_sq) {
		return 0;
	}
	if (closest_v_lensq < radius_sq) {
		return 2;
	}
	return 1;
}

int Sphere_Intersect_Plane(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t plane_v[3], const CCTNum_t plane_normal[3], CCTNum_t new_o[3], CCTNum_t* new_r) {
	CCTNum_t abs_d, d;
	d = mathPointProjectionPlane(o, plane_v, plane_normal, new_o);
	abs_d = CCTNum_abs(d);
	if (abs_d > radius) {
		if (new_r) {
			*new_r = CCTNum(0.0);
		}
		return 0;
	}
	if (abs_d == radius) {
		if (new_r) {
			*new_r = CCTNum(0.0);
		}
		return 1;
	}
	if (new_r) {
		*new_r = CCTNum_sqrt(CCTNum_sq(radius) - CCTNum_sq(abs_d));
	}
	return 2;
}

static int Sphere_Intersect_Circle(const CCTNum_t o[3], CCTNum_t radius, const GeometryCircle_t* circle) {
	CCTNum_t new_o[3], new_r, dsq, rsum_sq;
	int res = Sphere_Intersect_Plane(o, radius, circle->o, circle->normal, new_o, &new_r);
	if (0 == res) {
		return 0;
	}
	dsq = mathVec3DistanceSq(new_o, circle->o);
	if (1 == res) {
		if (dsq > CCTNum_sq(circle->radius)) {
			return 0;
		}
		return 1;
	}
	rsum_sq = CCTNum_sq(circle->radius + new_r);
	if (dsq > rsum_sq) {
		return 0;
	}
	if (dsq == rsum_sq) {
		return 1;
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

int Sphere_Intersect_ConvexMesh(const CCTNum_t o[3], CCTNum_t radius, const GeometryMesh_t* mesh) {
	unsigned int i;
	if (ConvexMesh_Contain_Point(mesh, o)) {
		return 2;
	}
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		CCTNum_t p[3];
		const GeometryPolygon_t* polygon = mesh->polygons + i;
		int res = Sphere_Intersect_Plane(o, radius, polygon->v[polygon->v_indices[0]], polygon->normal, p, NULL);
		if (0 == res) {
			continue;
		}
		if (Polygon_Contain_Point(polygon, p)) {
			return res;
		}
	}
	for (i = 0; i < mesh->edge_indices_cnt; ) {
		int res;
		CCTNum_t edge[2][3];
		mathVec3Copy(edge[0], mesh->v[mesh->edge_indices[i++]]);
		mathVec3Copy(edge[1], mesh->v[mesh->edge_indices[i++]]);
		res = Sphere_Intersect_Segment(o, radius, (const CCTNum_t(*)[3])edge, NULL);
		if (res) {
			return res;
		}
	}
	return 0;
}

int Sphere_Intersect_OBB(const CCTNum_t o[3], CCTNum_t radius, const GeometryOBB_t* obb) {
	CCTNum_t v[3];
	mathOBBClosestPointTo(obb, o, v);
	mathVec3Sub(v, o, v);
	return mathVec3LenSq(v) <= CCTNum_sq(radius);
}

static int Sphere_Intersect_Sphere(const CCTNum_t o1[3], CCTNum_t r1, const CCTNum_t o2[3], CCTNum_t r2, CCTNum_t p[3]) {
	CCTNum_t o1o2[3];
	CCTNum_t o1o2_lensq, radius_sum_sq = CCTNum_sq(r1 + r2);
	mathVec3Sub(o1o2, o2, o1);
	o1o2_lensq = mathVec3LenSq(o1o2);
	if (o1o2_lensq > radius_sum_sq) {
		return 0;
	}
	else if (o1o2_lensq < radius_sum_sq) {
		return 2;
	}
	if (p) {
		mathVec3Normalized(o1o2, o1o2);
		mathVec3AddScalar(mathVec3Copy(p, o1), o1o2, r1);
	}
	return 1;
}

static int Sphere_Intersect_AABB(const CCTNum_t sp_o[3], CCTNum_t sp_radius, const CCTNum_t aabb_o[3], const CCTNum_t aabb_half[3]) {
	CCTNum_t closest_v[3];
	mathAABBClosestPointTo(aabb_o, aabb_half, sp_o, closest_v);
	mathVec3Sub(closest_v, closest_v, sp_o);
	return mathVec3LenSq(closest_v) <= CCTNum_sq(sp_radius);
}

static int AABB_Intersect_Segment(const CCTNum_t o[3], const CCTNum_t half[3], const CCTNum_t ls[2][3]) {
	int i;
	CCTNum_t v[8][3];
	if (AABB_Contain_Point(o, half, ls[0]) || AABB_Contain_Point(o, half, ls[1])) {
		return 1;
	}
	mathAABBVertices(o, half, v);
	for (i = 0; i < 6; ++i) {
		GeometryPolygon_t polygon;
		mathBoxFace((const CCTNum_t(*)[3])v, AABB_Axis, i, &polygon);
		if (Segment_Intersect_Polygon(ls, &polygon, NULL)) {
			return 1;
		}
	}
	return 0;
}

static int Segment_Intersect_OBB(const CCTNum_t ls[2][3], const GeometryOBB_t* obb) {
	int i;
	CCTNum_t v[8][3];
	if (OBB_Contain_Point(obb, ls[0]) || OBB_Contain_Point(obb, ls[1])) {
		return 1;
	}
	mathOBBVertices(obb, v);
	for (i = 0; i < 6; ++i) {
		GeometryPolygon_t polygon;
		mathBoxFace((const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])obb->axis, i, &polygon);
		if (Segment_Intersect_Polygon(ls, &polygon, NULL)) {
			return 1;
		}
	}
	return 0;
}

int OBB_Intersect_OBB(const GeometryOBB_t* obb0, const GeometryOBB_t* obb1) {
	/* these code is copy from PhysX-3.4 */
	CCTNum_t v[3], T[3];
	CCTNum_t R[3][3], FR[3][3], ra, rb, t;
	const CCTNum_t* e0 = obb0->half, *e1 = obb1->half;
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

int ConvexMesh_Intersect_ConvexMesh(const GeometryMesh_t* mesh1, const GeometryMesh_t* mesh2) {
	unsigned int i;
	for (i = 0; i < mesh1->v_indices_cnt; ++i) {
		const CCTNum_t* p = mesh1->v[mesh1->v_indices[i]];
		if (ConvexMesh_Contain_Point(mesh2, p)) {
			return 1;
		}
	}
	for (i = 0; i < mesh2->polygons_cnt; ++i) {
		if (ConvexMesh_Intersect_Polygon(mesh1, mesh2->polygons + i)) {
			return 1;
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
				return Segment_Intersect_Plane(one_segment_v, two->plane->v, two->plane->normal, NULL, NULL);
			}
			case GEOMETRY_BODY_AABB:
			{
				return AABB_Intersect_Segment(two->aabb->o, two->aabb->half, one_segment_v);
			}
			case GEOMETRY_BODY_OBB:
			{
				return Segment_Intersect_OBB(one_segment_v, two->obb);
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
				return Sphere_Intersect_AABB(two->sphere->o, two->sphere->radius, one->aabb->o, one->aabb->half);
			}
			case GEOMETRY_BODY_PLANE:
			{
				const GeometryPlane_t* plane = two->plane;
				CCTNum_t v[8][3];
				mathAABBVertices(one->aabb->o, one->aabb->half, v);
				return Vertices_Intersect_Plane((const CCTNum_t(*)[3])v, Box_Vertice_Indices_Default, 8, plane->v, plane->normal, NULL, NULL);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				return AABB_Intersect_Segment(one->aabb->o, one->aabb->half, (const CCTNum_t(*)[3])two->segment->v);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				GeometryBoxMesh_t one_mesh;
				CCTNum_t v[8][3];
				mathAABBVertices(one->aabb->o, one->aabb->half, v);
				mathBoxMesh((const CCTNum_t(*)[3])v, AABB_Axis, &one_mesh);
				return ConvexMesh_Intersect_Polygon(&one_mesh.mesh, two->polygon);
			}
			case GEOMETRY_BODY_OBB:
			{
				GeometryOBB_t one_obb;
				mathOBBFromAABB(&one_obb, one->aabb->o, one->aabb->half);
				return OBB_Intersect_OBB(&one_obb, two->obb);
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				GeometryBoxMesh_t one_mesh;
				CCTNum_t v[8][3];
				mathAABBVertices(one->aabb->o, one->aabb->half, v);
				mathBoxMesh((const CCTNum_t(*)[3])v, AABB_Axis, &one_mesh);
				return ConvexMesh_Intersect_ConvexMesh(&one_mesh.mesh, two->mesh);
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
				return Sphere_Intersect_AABB(one->sphere->o, one->sphere->radius, two->aabb->o, two->aabb->half);
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
				return Sphere_Intersect_ConvexMesh(one->sphere->o, one->sphere->radius, two->mesh);
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
				const GeometryPlane_t* plane = one->plane;
				CCTNum_t v[8][3];
				mathAABBVertices(two->aabb->o, two->aabb->half, v);
				return Vertices_Intersect_Plane((const CCTNum_t(*)[3])v, Box_Vertice_Indices_Default, 8, plane->v, plane->normal, NULL, NULL);
			}
			case GEOMETRY_BODY_OBB:
			{
				const GeometryPlane_t* plane = one->plane;
				CCTNum_t v[8][3];
				mathOBBVertices(two->obb, v);
				return Vertices_Intersect_Plane((const CCTNum_t(*)[3])v, Box_Vertice_Indices_Default, 8, plane->v, plane->normal, NULL, NULL);
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
				return Segment_Intersect_Plane((const CCTNum_t(*)[3])two->segment->v, one->plane->v, one->plane->normal, NULL, NULL);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				const GeometryPlane_t* plane = one->plane;
				const GeometryPolygon_t* polygon = two->polygon;
				return Vertices_Intersect_Plane((const CCTNum_t(*)[3])polygon->v, polygon->v_indices, polygon->v_indices_cnt, plane->v, plane->normal, NULL, NULL);
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				const GeometryPlane_t* plane = one->plane;
				const GeometryMesh_t* mesh = two->mesh;
				return Vertices_Intersect_Plane((const CCTNum_t(*)[3])mesh->v, mesh->v_indices, mesh->v_indices_cnt, plane->v, plane->normal, NULL, NULL);
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
				const GeometryPlane_t* plane = two->plane;
				const GeometryPolygon_t* polygon = one->polygon;
				return Vertices_Intersect_Plane((const CCTNum_t(*)[3])polygon->v, polygon->v_indices, polygon->v_indices_cnt, plane->v, plane->normal, NULL, NULL);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				return Sphere_Intersect_Polygon(two->sphere->o, two->sphere->radius, one->polygon, NULL);
			}
			case GEOMETRY_BODY_AABB:
			{
				GeometryBoxMesh_t two_mesh;
				CCTNum_t v[8][3];
				mathAABBVertices(two->aabb->o, two->aabb->half, v);
				mathBoxMesh((const CCTNum_t(*)[3])v, AABB_Axis, &two_mesh);
				return ConvexMesh_Intersect_Polygon(&two_mesh.mesh, one->polygon);
			}
			case GEOMETRY_BODY_OBB:
			{
				GeometryBoxMesh_t two_mesh;
				CCTNum_t v[8][3];
				mathOBBVertices(two->obb, v);
				mathBoxMesh((const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])two->obb->axis, &two_mesh);
				return ConvexMesh_Intersect_Polygon(&two_mesh.mesh, one->polygon);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				return Polygon_Intersect_Polygon(one->polygon, two->polygon);
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				return ConvexMesh_Intersect_Polygon(two->mesh, one->polygon);
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
				return Segment_Intersect_OBB((const CCTNum_t(*)[3])two->segment->v, one->obb);
			}
			case GEOMETRY_BODY_PLANE:
			{
				const GeometryPlane_t* plane = two->plane;
				CCTNum_t v[8][3];
				mathOBBVertices(one->obb, v);
				return Vertices_Intersect_Plane((const CCTNum_t(*)[3])v, Box_Vertice_Indices_Default, 8, plane->v, plane->normal, NULL, NULL);
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
				GeometryBoxMesh_t one_mesh;
				CCTNum_t v[8][3];
				mathOBBVertices(one->obb, v);
				mathBoxMesh((const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])one->obb->axis, &one_mesh);
				return ConvexMesh_Intersect_Polygon(&one_mesh.mesh, two->polygon);
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				GeometryBoxMesh_t one_mesh;
				CCTNum_t v[8][3];
				mathOBBVertices(one->obb, v);
				mathBoxMesh((const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])one->obb->axis, &one_mesh);
				return ConvexMesh_Intersect_ConvexMesh(&one_mesh.mesh, two->mesh);
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
				const GeometryPlane_t* plane = two->plane;
				const GeometryMesh_t* mesh = one->mesh;
				return Vertices_Intersect_Plane((const CCTNum_t(*)[3])mesh->v, mesh->v_indices, mesh->v_indices_cnt, plane->v, plane->normal, NULL, NULL);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				return Sphere_Intersect_ConvexMesh(two->sphere->o, two->sphere->radius, one->mesh);
			}
			case GEOMETRY_BODY_AABB:
			{
				GeometryBoxMesh_t two_mesh;
				CCTNum_t v[8][3];
				mathAABBVertices(two->aabb->o, two->aabb->half, v);
				mathBoxMesh((const CCTNum_t(*)[3])v, AABB_Axis, &two_mesh);
				return ConvexMesh_Intersect_ConvexMesh(&two_mesh.mesh, one->mesh);
			}
			case GEOMETRY_BODY_OBB:
			{
				GeometryBoxMesh_t two_mesh;
				CCTNum_t v[8][3];
				mathOBBVertices(two->obb, v);
				mathBoxMesh((const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])two->obb->axis, &two_mesh);
				return ConvexMesh_Intersect_ConvexMesh(&two_mesh.mesh, one->mesh);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				return ConvexMesh_Intersect_Polygon(one->mesh, two->polygon);
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
