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
#include "../inc/vertex.h"
#include "../inc/capsule.h"
#include "../inc/geometry_api.h"

extern const CCTNum_t AABB_Axis[3][3];
extern const unsigned int Box_Edge_Indices[24];
extern const unsigned int Box_Vertice_Indices_Default[8];

extern int Segment_Contain_Point(const CCTNum_t ls[2][3], const CCTNum_t p[3]);
extern int Sphere_Contain_Point(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t p[3]);
extern int Plane_Contain_Point(const CCTNum_t plane_v[3], const CCTNum_t plane_normal[3], const CCTNum_t p[3]);
extern int Polygon_Contain_Point(const GeometryPolygon_t* polygon, const CCTNum_t p[3]);
extern int OBB_Contain_Point(const GeometryOBB_t* obb, const CCTNum_t p[3]);
extern int ConvexMesh_Contain_Point(const GeometryMesh_t* mesh, const CCTNum_t p[3]);
extern int Capsule_Contain_Point(const GeometryCapsule_t* capsule, const CCTNum_t p[3]);

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

int Ray_Intersect_Plane(const CCTNum_t o[3], const CCTNum_t dir[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3]) {
	CCTNum_t d, cos_theta;
	d = mathPointProjectionPlane(o, plane_v, plane_n);
	if (CCTNum(0.0) == d) {
		return 1;
	}
	cos_theta = mathVec3Dot(dir, plane_n);
	if (CCTNum(0.0) == cos_theta) {
		return 0;
	}
	d /= cos_theta;
	if (d < CCTNum(0.0)) {
		return 0;
	}
	return 1;
}

static int Vertices_Intersect_Plane(const CCTNum_t(*v)[3], const unsigned int* v_indices, unsigned int v_indices_cnt, const CCTNum_t plane_v[3], const CCTNum_t plane_n[3]) {
	int flag_sign = 0;
	unsigned int i;
	for (i = 0; i < v_indices_cnt; ++i) {
		CCTNum_t d = mathPointProjectionPlane(v[v_indices[i]], plane_v, plane_n);
		if (d > CCTNum(0.0)) {
			if (flag_sign < 0) {
				return 1;
			}
			flag_sign = 1;
		}
		else if (d < CCTNum(0.0)) {
			if (flag_sign > 0) {
				return 1;
			}
			flag_sign = -1;
		}
		else {
			return 1;
		}
	}
	return 0;
}

static int Segment_Intersect_Segment(const CCTNum_t ls1[2][3], const CCTNum_t ls2[2][3]) {
	CCTNum_t d, ls1_len, cos_theta;
	CCTNum_t v[3], N[3], p[3];
	CCTNum_t ls1_dir[3], ls2_dir[3];

	mathVec3Sub(ls1_dir, ls1[1], ls1[0]);
	mathVec3Sub(ls2_dir, ls2[1], ls2[0]);
	mathVec3Cross(N, ls1_dir, ls2_dir);
	mathVec3Sub(v, ls2[0], ls1[0]);
	/* check Line vs Line parallel or collinear */
	if (mathVec3IsZero(N)) {
		CCTNum_t l[3], r[3];
		mathVec3Cross(N, v, ls1_dir);
		if (!mathVec3IsZero(N)) {
			/* parallel */
			return 0;
		}
		/* collinear */
		mathVec3Sub(l, ls1[0], ls2[0]);
		mathVec3Sub(r, ls1[1], ls2[0]);
		d = mathVec3Dot(l, r);
		if (d <= CCTNum(0.0)) {
			return 1;
		}
		mathVec3Sub(l, ls1[0], ls2[1]);
		mathVec3Sub(r, ls1[1], ls2[1]);
		d = mathVec3Dot(l, r);
		if (d <= CCTNum(0.0)) {
			return 1;
		}
		mathVec3Sub(l, ls2[0], ls1[0]);
		mathVec3Sub(r, ls2[1], ls1[0]);
		d = mathVec3Dot(l, r);
		if (d <= CCTNum(0.0)) {
			return 1;
		}
		mathVec3Sub(l, ls2[0], ls1[1]);
		mathVec3Sub(r, ls2[1], ls1[1]);
		d = mathVec3Dot(l, r);
		if (d <= CCTNum(0.0)) {
			return 1;
		}
		return 0;
	}
	d = mathVec3Dot(v, N);
	if (d < CCT_EPSILON_NEGATE || d > CCT_EPSILON) {
		/* opposite */
		return 0;
	}
	/* cross */
	if (mathVec3IsZero(v)) {
		return 1;
	}
	mathVec3Normalized(ls2_dir, ls2_dir);
	mathPointProjectionLine(ls1[0], ls2[0], ls2_dir, p);
	if (mathVec3Equal(p, ls1[0])) {
		mathVec3Sub(N, ls2[0], p);
		mathVec3Sub(v, ls2[1], p);
		d = mathVec3Dot(N, v);
		return d <= CCTNum(0.0);
	}
	ls1_len = mathVec3Normalized(ls1_dir, ls1_dir);
	mathVec3Sub(N, p, ls1[0]);
	d = mathVec3Normalized(N, N);
	cos_theta = mathVec3Dot(N, ls1_dir);
	if (CCTNum(0.0) == cos_theta) {
		/* no possible */
		return 1;
	}
	d /= cos_theta;
	if (d < CCTNum(0.0) || d > ls1_len) {
		return 0;
	}
	mathVec3Copy(p, ls1[0]);
	mathVec3AddScalar(p, ls1_dir, d);
	mathVec3Sub(N, ls2[0], p);
	mathVec3Sub(v, ls2[1], p);
	d = mathVec3Dot(v, N);
	return d <= CCTNum(0.0);
}

int Segment_Intersect_Plane(const CCTNum_t ls[2][3], const CCTNum_t plane_v[3], const CCTNum_t plane_normal[3], CCTNum_t intersect_p[3], CCTNum_t d[3]) {
	CCTNum_t temp_d[3];
	if (!d) {
		d = temp_d;
	}
	d[0] = mathPointProjectionPlane(ls[0], plane_v, plane_normal);
	d[1] = mathPointProjectionPlane(ls[1], plane_v, plane_normal);
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

int Segment_Intersect_Polygon(const CCTNum_t ls[2][3], const GeometryPolygon_t* polygon, int* ret_plane_side) {
	unsigned int i;
	CCTNum_t p[3], d[3];
	int res = Segment_Intersect_Plane(ls, polygon->v[polygon->v_indices[0]], polygon->normal, p, d);
	if (0 == res) {
		if (ret_plane_side) {
			*ret_plane_side = (d[2] > CCTNum(0.0) ? 1 : -1);
		}
		return 0;
	}
	if (ret_plane_side) {
		*ret_plane_side = 0;
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
		if (Segment_Intersect_Segment(ls, (const CCTNum_t(*)[3])edge)) {
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

static int Segment_Intersect_Capsule(const CCTNum_t ls[2][3], const GeometryCapsule_t* capsule) {
	CCTNum_t edge[2][3], min_lensq;
	mathTwoVertexFromCenterHalf(capsule->o, capsule->axis, capsule->half, edge[0], edge[1]);
	min_lensq = mathSegmentClosestSegmentDistanceSq(ls, NULL, 0, (const CCTNum_t(*)[3])edge, capsule->axis, capsule->half + capsule->half);
	return min_lensq <= CCTNum_sq(capsule->radius);
}

int Polygon_Intersect_Polygon(const GeometryPolygon_t* polygon1, const GeometryPolygon_t* polygon2, int* ret_plane_side) {
	int plane_side = 0;
	unsigned int i;
	for (i = 0; i < polygon1->v_indices_cnt; ) {
		int ret_side;
		CCTNum_t edge[2][3];
		mathVec3Copy(edge[0], polygon1->v[polygon1->v_indices[i++]]);
		mathVec3Copy(edge[1], polygon1->v[polygon1->v_indices[i >= polygon1->v_indices_cnt ? 0 : i]]);
		if (Segment_Intersect_Polygon((const CCTNum_t(*)[3])edge, polygon2, &ret_side)) {
			if (ret_plane_side) {
				*ret_plane_side = 0;
			}
			return 1;
		}
		if (0 == i) {
			plane_side = ret_side;
			continue;
		}
		if (plane_side * ret_side <= 0) {
			plane_side = 0;
		}
	}
	if (ret_plane_side) {
		*ret_plane_side = plane_side;
	}
	if (plane_side) {
		return 0;
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

int ConvexMesh_Intersect_Polygon(const GeometryMesh_t* mesh, const GeometryPolygon_t* polygon, int* ret_plane_side) {
	int plane_side = 0;
	unsigned int i;
	for (i = 0; i < mesh->edge_indices_cnt; ) {
		int ret_side;
		CCTNum_t edge[2][3];
		unsigned int v_idx[2];
		v_idx[0] = mesh->edge_indices[i++];
		if (2 == mesh->edge_stride) {
			v_idx[1] = mesh->edge_indices[i++];
		}
		else {
			v_idx[1] = mesh->edge_indices[i >= mesh->edge_indices_cnt ? 0 : i];
		}
		mathVec3Copy(edge[0], mesh->v[v_idx[0]]);
		mathVec3Copy(edge[1], mesh->v[v_idx[1]]);
		if (Segment_Intersect_Polygon((const CCTNum_t(*)[3])edge, polygon, &ret_side)) {
			if (ret_plane_side) {
				*ret_plane_side = 0;
			}
			return 1;
		}
		if (0 == i) {
			plane_side = ret_side;
			continue;
		}
		if (plane_side * ret_side <= 0) {
			plane_side = 0;
		}
	}
	if (ret_plane_side) {
		*ret_plane_side = plane_side;
	}
	if (plane_side) {
		return 0;
	}
	for (i = 0; i < polygon->v_indices_cnt; ) {
		CCTNum_t edge[2][3];
		mathVec3Copy(edge[0], polygon->v[polygon->v_indices[i++]]);
		mathVec3Copy(edge[1], polygon->v[polygon->v_indices[i >= polygon->v_indices_cnt ? 0 : i]]);
		if (Segment_Intersect_ConvexMesh((const CCTNum_t(*)[3])edge, mesh)) {
			return 1;
		}
	}
	return 0;
}

static int Capsule_Intersect_Plane(const GeometryCapsule_t* capsule, const CCTNum_t plane_v[3], const CCTNum_t plane_n[3]) {
	CCTNum_t edge[2][3], d[3], abs_d;
	mathTwoVertexFromCenterHalf(capsule->o, capsule->axis, capsule->half, edge[0], edge[1]);
	if (Segment_Intersect_Plane((const CCTNum_t(*)[3])edge, plane_v, plane_n, NULL, d)) {
		return 2;
	}
	abs_d = CCTNum_abs(d[2]);
	if (abs_d > capsule->radius) {
		return 0;
	}
	if (abs_d < capsule->radius) {
		return 2;
	}
	return 1;
}

static void capsule_fill_extra(const GeometryCapsule_t* capsule, GeometryCapsuleExtra_t* capsule_extra) {
	mathTwoVertexFromCenterHalf(capsule->o, capsule->axis, capsule->half, capsule_extra->axis_edge[0], capsule_extra->axis_edge[1]);
	capsule_extra->axis_len = capsule->half + capsule->half;
	capsule_extra->radius_sq = CCTNum_sq(capsule->radius);
}

int Capsule_Intersect_Polygon(const GeometryCapsule_t* capsule, const GeometryCapsuleExtra_t* capsule_extra, const GeometryPolygon_t* polygon, int* ret_plane_side) {
	int res, i;
	CCTNum_t p[3], d[3];
	if (ret_plane_side) {
		*ret_plane_side = 0;
	}
	res = Segment_Intersect_Plane((const CCTNum_t(*)[3])capsule_extra->axis_edge, polygon->v[polygon->v_indices[0]], polygon->normal, NULL, d);
	if (2 == res) {
		if (Polygon_Contain_Point(polygon, capsule->o)) {
			return 1;
		}
		if (Polygon_Contain_Point(polygon, capsule_extra->axis_edge[0])) {
			return 1;
		}
		if (Polygon_Contain_Point(polygon, capsule_extra->axis_edge[1])) {
			return 1;
		}
	}
	else if (1 == res) {
		CCTNum_t cos_theta = mathVec3Dot(capsule->axis, polygon->normal);
		mathVec3Copy(p, capsule_extra->axis_edge[0]);
		mathVec3AddScalar(p, capsule->axis, d[0] / cos_theta);
		if (Polygon_Contain_Point(polygon, p)) {
			return 1;
		}
	}
	else if (CCTNum_abs(d[2]) > capsule->radius) {
		if (ret_plane_side) {
			*ret_plane_side = (d[2] > CCTNum(0.0) ? 1 : -1);
		}
		return 0;
	}
	else {
		if (CCTNum_abs(d[0]) <= capsule->radius) {
			mathVec3Copy(p, capsule_extra->axis_edge[0]);
			mathVec3AddScalar(p, polygon->normal, d[0]);
			if (Polygon_Contain_Point(polygon, p)) {
				return 1;
			}
		}
		if (CCTNum_abs(d[1]) <= capsule->radius) {
			mathVec3Copy(p, capsule_extra->axis_edge[1]);
			mathVec3AddScalar(p, polygon->normal, d[1]);
			if (Polygon_Contain_Point(polygon, p)) {
				return 1;
			}
		}
		if (d[0] == d[1]) {
			mathVec3Copy(p, capsule->o);
			mathVec3AddScalar(p, polygon->normal, d[2]);
			if (Polygon_Contain_Point(polygon, p)) {
				return 1;
			}
		}
	}
	for (i = 0; i < polygon->v_indices_cnt; ) {
		CCTNum_t edge[2][3], lensq;
		mathVec3Copy(edge[0], polygon->v[polygon->v_indices[i++]]);
		mathVec3Copy(edge[1], polygon->v[polygon->v_indices[i >= polygon->v_indices_cnt ? 0 : i]]);
		lensq = mathSegmentClosestSegmentDistanceSq(
			(const CCTNum_t(*)[3])edge, NULL, 0,
			(const CCTNum_t(*)[3])capsule_extra->axis_edge, capsule->axis, capsule_extra->axis_len
		);
		if (lensq <= capsule_extra->radius_sq) {
			return 1;
		}
	}
	return 0;
}

int Capsule_Intersect_ConvexMesh(const GeometryCapsule_t* capsule, const GeometryMesh_t* mesh) {
	unsigned int i;
	struct GeometryCapsuleExtra_t capsule_extra;
	if (ConvexMesh_Contain_Point(mesh, capsule->o)) {
		return 1;
	}
	mathTwoVertexFromCenterHalf(capsule->o, capsule->axis, capsule->half, capsule_extra.axis_edge[0], capsule_extra.axis_edge[1]);
	if (ConvexMesh_Contain_Point(mesh, capsule_extra.axis_edge[0])) {
		return 1;
	}
	if (ConvexMesh_Contain_Point(mesh, capsule_extra.axis_edge[1])) {
		return 1;
	}
	capsule_extra.axis_len = capsule->half + capsule->half;
	capsule_extra.radius_sq = CCTNum_sq(capsule->radius);
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		const GeometryPolygon_t* polygon = mesh->polygons + i;
		if (Capsule_Intersect_Polygon(capsule, &capsule_extra, polygon, NULL)) {
			return 1;
		}
	}
	return 0;
}

static int Capsule_Intersect_Capsule(const GeometryCapsule_t* c1, const GeometryCapsule_t* c2) {
	CCTNum_t c1_edge[2][3], c2_edge[2][3], min_lensq, radius_sum;
	mathTwoVertexFromCenterHalf(c1->o, c1->axis, c1->half, c1_edge[0], c1_edge[1]);
	mathTwoVertexFromCenterHalf(c2->o, c2->axis, c2->half, c2_edge[0], c2_edge[1]);
	min_lensq = mathSegmentClosestSegmentDistanceSq(
		(const CCTNum_t(*)[3])c1_edge, c1->axis, c1->half + c1->half,
		(const CCTNum_t(*)[3])c2_edge, c2->axis, c2->half + c2->half
	);
	radius_sum = c1->radius + c2->radius;
	return min_lensq <= CCTNum_sq(radius_sum);
}

int Sphere_Intersect_Segment(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t ls[2][3]) {
	CCTNum_t lensq, radius_sq;
	CCTNum_t closest_p[3];
	mathSegmentClosestPointTo(ls, o, closest_p);
	lensq = mathVec3DistanceSq(closest_p, o);
	radius_sq = CCTNum_sq(radius);
	if (lensq > radius_sq) {
		return 0;
	}
	if (lensq < radius_sq) {
		return 2;
	}
	return 1;
}

static int Sphere_Intersect_Capsule(const CCTNum_t o[3], CCTNum_t radius, const GeometryCapsule_t* capsule) {
	CCTNum_t lensq, radius_sq;
	CCTNum_t closest_p[3];
	mathSegmentClosestPointTo_v2(capsule->o, capsule->axis, capsule->half, o, closest_p);
	lensq = mathVec3DistanceSq(o, closest_p);
	radius_sq = CCTNum_sq(radius + capsule->radius);
	if (lensq > radius_sq) {
		return 0;
	}
	if (lensq < radius_sq) {
		return 2;
	}
	return 1;
}

int Sphere_Intersect_Plane(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t plane_v[3], const CCTNum_t plane_normal[3], CCTNum_t new_o[3], CCTNum_t* new_r) {
	CCTNum_t abs_d, d;
	d = mathPointProjectionPlane(o, plane_v, plane_normal);
	if (new_o) {
		mathVec3Copy(new_o, o);
		mathVec3AddScalar(new_o, plane_normal, d);
	}
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

int Sphere_Intersect_Polygon(const CCTNum_t o[3], CCTNum_t radius, const GeometryPolygon_t* polygon, int* ret_plane_side) {
	int res;
	unsigned int i;
	CCTNum_t p[3];
	res = Sphere_Intersect_Plane(o, radius, polygon->v[polygon->v_indices[0]], polygon->normal, p, NULL);
	if (0 == res) {
		if (ret_plane_side) {
			CCTNum_t d = mathPointProjectionPlane(o, polygon->v[polygon->v_indices[0]], polygon->normal);
			*ret_plane_side = (d > CCTNum(0.0) ? 1 : -1);
		}
		return 0;
	}
	if (ret_plane_side) {
		*ret_plane_side = 0;
	}
	if (Polygon_Contain_Point(polygon, p)) {
		return res;
	}
	if (1 == res) {
		return 0;
	}
	for (i = 0; i < polygon->v_indices_cnt; ) {
		CCTNum_t edge[2][3];
		mathVec3Copy(edge[0], polygon->v[polygon->v_indices[i++]]);
		mathVec3Copy(edge[1], polygon->v[polygon->v_indices[i >= polygon->v_indices_cnt ? 0 : i]]);
		res = Sphere_Intersect_Segment(o, radius, (const CCTNum_t(*)[3])edge);
		if (res) {
			return res;
		}
	}
	return 0;
}

int Sphere_Intersect_ConvexMesh(const CCTNum_t o[3], CCTNum_t radius, const GeometryMesh_t* mesh) {
	unsigned int i;
	if (ConvexMesh_Contain_Point(mesh, o)) {
		return 1;
	}
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		const GeometryPolygon_t* polygon = mesh->polygons + i;
		if (Sphere_Intersect_Polygon(o, radius, polygon, NULL)) {
			return 1;
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
		if (ConvexMesh_Intersect_Polygon(mesh1, mesh2->polygons + i, NULL)) {
			return 1;
		}
	}
	return 0;
}

static int Geometry_Intersect_InflateBox(const GeometryBodyRef_t* one, const GeometryOBB_t* obb, CCTNum_t inflate) {
	unsigned int i;
	GeometryCapsule_t capsule;
	GeometryOBB_t face_obb;
	GeometryBodyRef_t geo_ref;
	CCTNum_t v[8][3];
	/* face inflate to box */
	geo_ref.type = GEOMETRY_BODY_OBB;
	geo_ref.obb = &face_obb;
	mathVec3Copy(face_obb.o, obb->o);
	mathVec3Copy(face_obb.axis[0], obb->axis[0]);
	mathVec3Copy(face_obb.axis[1], obb->axis[1]);
	mathVec3Copy(face_obb.axis[2], obb->axis[2]);
	for (i = 0; i < 3; ++i) {
		mathVec3Copy(face_obb.half, obb->half);
		face_obb.half[i] += inflate;
		if (mathGeometryIntersect(one, &geo_ref)) {
			return 1;
		}
	}
	/* edge inflate to Capsule */
	geo_ref.type = GEOMETRY_BODY_CAPSULE;
	geo_ref.capsule = &capsule;
	capsule.radius = inflate;
	mathOBBVertices(obb, v);
	for (i = 0; i < 24; ) {
		const CCTNum_t* p0 = v[Box_Edge_Indices[i++]];
		const CCTNum_t* p1 = v[Box_Edge_Indices[i++]];
		mathTwoVertexToCenterHalf(p0, p1, capsule.o, capsule.axis, &capsule.half);
		if (mathGeometryIntersect(one, &geo_ref)) {
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
			case GEOMETRY_BODY_CAPSULE:
			{
				return Capsule_Contain_Point(two->capsule, one->point);
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
				return Segment_Intersect_Segment(one_segment_v, (const CCTNum_t(*)[3])two->segment->v);
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
				return Sphere_Intersect_Segment(two->sphere->o, two->sphere->radius, one_segment_v);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				return Segment_Intersect_Polygon(one_segment_v, two->polygon, NULL);
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				return Segment_Intersect_ConvexMesh(one_segment_v, two->mesh);
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				return Segment_Intersect_Capsule(one_segment_v, two->capsule);
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
				return Vertices_Intersect_Plane((const CCTNum_t(*)[3])v, Box_Vertice_Indices_Default, 8, plane->v, plane->normal);
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
				mathBoxMesh(&one_mesh, (const CCTNum_t(*)[3])v, AABB_Axis);
				return ConvexMesh_Intersect_Polygon(&one_mesh.mesh, two->polygon, NULL);
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
				mathBoxMesh(&one_mesh, (const CCTNum_t(*)[3])v, AABB_Axis);
				return ConvexMesh_Intersect_ConvexMesh(&one_mesh.mesh, two->mesh);
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				GeometryBoxMesh_t one_mesh;
				CCTNum_t v[8][3];
				mathAABBVertices(one->aabb->o, one->aabb->half, v);
				mathBoxMesh(&one_mesh, (const CCTNum_t(*)[3])v, AABB_Axis);
				return Capsule_Intersect_ConvexMesh(two->capsule, &one_mesh.mesh);
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
				return Sphere_Intersect_Segment(one->sphere->o, one->sphere->radius, (const CCTNum_t(*)[3])two->segment->v);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				return Sphere_Intersect_Polygon(one->sphere->o, one->sphere->radius, two->polygon, NULL);
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				return Sphere_Intersect_ConvexMesh(one->sphere->o, one->sphere->radius, two->mesh);
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				return Sphere_Intersect_Capsule(one->sphere->o, one->sphere->radius, two->capsule);
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
				return Vertices_Intersect_Plane((const CCTNum_t(*)[3])v, Box_Vertice_Indices_Default, 8, plane->v, plane->normal);
			}
			case GEOMETRY_BODY_OBB:
			{
				const GeometryPlane_t* plane = one->plane;
				CCTNum_t v[8][3];
				mathOBBVertices(two->obb, v);
				return Vertices_Intersect_Plane((const CCTNum_t(*)[3])v, Box_Vertice_Indices_Default, 8, plane->v, plane->normal);
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
				return Vertices_Intersect_Plane((const CCTNum_t(*)[3])polygon->v, polygon->v_indices, polygon->v_indices_cnt, plane->v, plane->normal);
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				const GeometryPlane_t* plane = one->plane;
				const GeometryMesh_t* mesh = two->mesh;
				return Vertices_Intersect_Plane((const CCTNum_t(*)[3])mesh->v, mesh->v_indices, mesh->v_indices_cnt, plane->v, plane->normal);
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				return Capsule_Intersect_Plane(two->capsule, one->plane->v, one->plane->normal);
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
				return Vertices_Intersect_Plane((const CCTNum_t(*)[3])polygon->v, polygon->v_indices, polygon->v_indices_cnt, plane->v, plane->normal);
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
				mathBoxMesh(&two_mesh, (const CCTNum_t(*)[3])v, AABB_Axis);
				return ConvexMesh_Intersect_Polygon(&two_mesh.mesh, one->polygon, NULL);
			}
			case GEOMETRY_BODY_OBB:
			{
				GeometryBoxMesh_t two_mesh;
				CCTNum_t v[8][3];
				mathOBBVertices(two->obb, v);
				mathBoxMesh(&two_mesh, (const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])two->obb->axis);
				return ConvexMesh_Intersect_Polygon(&two_mesh.mesh, one->polygon, NULL);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				return Polygon_Intersect_Polygon(one->polygon, two->polygon, NULL);
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				return ConvexMesh_Intersect_Polygon(two->mesh, one->polygon, NULL);
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				GeometryCapsuleExtra_t capsule_extra;
				capsule_fill_extra(two->capsule, &capsule_extra);
				return Capsule_Intersect_Polygon(two->capsule, &capsule_extra, one->polygon, NULL);
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
				return Vertices_Intersect_Plane((const CCTNum_t(*)[3])v, Box_Vertice_Indices_Default, 8, plane->v, plane->normal);
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
				mathBoxMesh(&one_mesh, (const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])one->obb->axis);
				return ConvexMesh_Intersect_Polygon(&one_mesh.mesh, two->polygon, NULL);
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				GeometryBoxMesh_t one_mesh;
				CCTNum_t v[8][3];
				mathOBBVertices(one->obb, v);
				mathBoxMesh(&one_mesh, (const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])one->obb->axis);
				return ConvexMesh_Intersect_ConvexMesh(&one_mesh.mesh, two->mesh);
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				GeometryBoxMesh_t one_mesh;
				CCTNum_t v[8][3];
				mathOBBVertices(one->obb, v);
				mathBoxMesh(&one_mesh, (const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])one->obb->axis);
				return Capsule_Intersect_ConvexMesh(two->capsule, &one_mesh.mesh);
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
				return Vertices_Intersect_Plane((const CCTNum_t(*)[3])mesh->v, mesh->v_indices, mesh->v_indices_cnt, plane->v, plane->normal);
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
				mathBoxMesh(&two_mesh, (const CCTNum_t(*)[3])v, AABB_Axis);
				return ConvexMesh_Intersect_ConvexMesh(&two_mesh.mesh, one->mesh);
			}
			case GEOMETRY_BODY_OBB:
			{
				GeometryBoxMesh_t two_mesh;
				CCTNum_t v[8][3];
				mathOBBVertices(two->obb, v);
				mathBoxMesh(&two_mesh, (const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])two->obb->axis);
				return ConvexMesh_Intersect_ConvexMesh(&two_mesh.mesh, one->mesh);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				return ConvexMesh_Intersect_Polygon(one->mesh, two->polygon, NULL);
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				return ConvexMesh_Intersect_ConvexMesh(one->mesh, two->mesh);
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				return Capsule_Intersect_ConvexMesh(two->capsule, one->mesh);
			}
		}
	}
	else if (GEOMETRY_BODY_CAPSULE == one->type) {
		switch (two->type) {
			case GEOMETRY_BODY_POINT:
			{
				return Capsule_Contain_Point(one->capsule, two->point);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				return Segment_Intersect_Capsule((const CCTNum_t(*)[3])two->segment, one->capsule);
			}
			case GEOMETRY_BODY_PLANE:
			{
				return Capsule_Intersect_Plane(one->capsule, two->plane->v, two->plane->normal);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				return Sphere_Intersect_Capsule(two->sphere->o, two->sphere->radius, one->capsule);
			}
			case GEOMETRY_BODY_AABB:
			{
				GeometryBoxMesh_t two_mesh;
				CCTNum_t v[8][3];
				mathAABBVertices(two->aabb->o, two->aabb->half, v);
				mathBoxMesh(&two_mesh, (const CCTNum_t(*)[3])v, AABB_Axis);
				return Capsule_Intersect_ConvexMesh(one->capsule, &two_mesh.mesh);
			}
			case GEOMETRY_BODY_OBB:
			{
				GeometryBoxMesh_t two_mesh;
				CCTNum_t v[8][3];
				mathOBBVertices(two->obb, v);
				mathBoxMesh(&two_mesh, (const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])two->obb->axis);
				return Capsule_Intersect_ConvexMesh(one->capsule, &two_mesh.mesh);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				GeometryCapsuleExtra_t capsule_extra;
				capsule_fill_extra(one->capsule, &capsule_extra);
				return Capsule_Intersect_Polygon(one->capsule, &capsule_extra, two->polygon, NULL);
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				return Capsule_Intersect_ConvexMesh(one->capsule, two->mesh);
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				return Capsule_Intersect_Capsule(one->capsule, two->capsule);
			}
		}
	}
	return 0;
}

int mathGeometryIntersectInflate(const GeometryBodyRef_t* one, const GeometryBodyRef_t* two, CCTNum_t inflate) {
	int i;
	const GeometryBodyRef_t* geo_refs[2];
	if (CCTNum_abs(inflate) < CCT_GAP_DISTANCE) {
		return mathGeometryIntersect(one, two);
	}
	if (one->data == two->data) {
		return 1;
	}
	geo_refs[0] = one;
	geo_refs[1] = two;
	for (i = 0; i < 2; ++i) {
		GeometryBodyRef_t inflate_ref;
		switch (geo_refs[i]->type) {
			case GEOMETRY_BODY_POINT:
			{
				GeometrySphere_t sphere;
				mathVec3Copy(sphere.o, geo_refs[i]->point);
				sphere.radius = inflate;
				inflate_ref.type = GEOMETRY_BODY_SPHERE;
				inflate_ref.sphere = &sphere;
				return mathGeometryIntersect(geo_refs[i ? 0 : 1], &inflate_ref);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				GeometryCapsule_t capsule;
				mathTwoVertexToCenterHalf(geo_refs[i]->segment->v[0], geo_refs[i]->segment->v[1], capsule.o, capsule.axis, &capsule.half);
				capsule.radius = inflate;
				inflate_ref.type = GEOMETRY_BODY_CAPSULE;
				inflate_ref.capsule = &capsule;
				return mathGeometryIntersect(geo_refs[i ? 0 : 1], &inflate_ref);
			}
			case GEOMETRY_BODY_PLANE:
			{
				GeometryPlane_t plane = *(geo_refs[i]->plane);
				mathVec3AddScalar(plane.v, plane.normal, inflate);
				inflate_ref.type = GEOMETRY_BODY_PLANE;
				inflate_ref.plane = &plane;
				return mathGeometryIntersect(geo_refs[i ? 0 : 1], &inflate_ref);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				GeometrySphere_t sphere = *(geo_refs[i]->sphere);
				sphere.radius += inflate;
				inflate_ref.type = GEOMETRY_BODY_SPHERE;
				inflate_ref.sphere = &sphere;
				return mathGeometryIntersect(geo_refs[i ? 0 : 1], &inflate_ref);
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				GeometryCapsule_t capsule = *(geo_refs[i]->capsule);
				capsule.radius += inflate;
				inflate_ref.type = GEOMETRY_BODY_CAPSULE;
				inflate_ref.capsule = &capsule;
				return mathGeometryIntersect(geo_refs[i ? 0 : 1], &inflate_ref);
			}
			case GEOMETRY_BODY_AABB:
			{
				GeometryOBB_t obb;
				mathOBBFromAABB(&obb, geo_refs[i]->aabb->o, geo_refs[i]->aabb->half);
				if (CCTNum_abs(inflate) < CCT_GAP_DISTANCE + CCT_GAP_DISTANCE) {
					obb.half[0] += inflate;
					obb.half[1] += inflate;
					obb.half[2] += inflate;
					inflate_ref.type = GEOMETRY_BODY_OBB;
					inflate_ref.obb = &obb;
					return mathGeometryIntersect(geo_refs[i ? 0 : 1], &inflate_ref);
				}
				return Geometry_Intersect_InflateBox(geo_refs[i ? 0 : 1], &obb, inflate);
			}
			case GEOMETRY_BODY_OBB:
			{
				if (CCTNum_abs(inflate) < CCT_GAP_DISTANCE + CCT_GAP_DISTANCE) {
					GeometryOBB_t obb = *(geo_refs[i]->obb);
					obb.half[0] += inflate;
					obb.half[1] += inflate;
					obb.half[2] += inflate;
					inflate_ref.type = GEOMETRY_BODY_OBB;
					inflate_ref.obb = &obb;
					return mathGeometryIntersect(geo_refs[i ? 0 : 1], &inflate_ref);
				}
				return Geometry_Intersect_InflateBox(geo_refs[i ? 0 : 1], geo_refs[i]->obb, inflate);
			}
		}
	}
	return mathGeometryIntersect(one, two);
}

#ifdef __cplusplus
}
#endif
