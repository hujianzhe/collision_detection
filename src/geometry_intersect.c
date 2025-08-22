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
#include "../inc/geometry_closest.h"
#include "../inc/geometry_api.h"

extern int Segment_Contain_Point(const CCTNum_t ls0[3], const CCTNum_t ls1[3], const CCTNum_t p[3]);
extern int Sphere_Contain_Point(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t p[3]);
extern int Plane_Contain_Point(const CCTNum_t plane_v[3], const CCTNum_t plane_normal[3], const CCTNum_t p[3]);
extern int Polygon_Contain_Point_SamePlane(const GeometryPolygon_t* polygon, const CCTNum_t p[3], GeometryBorderId_t* bi);
extern int Polygon_Contain_Point(const GeometryPolygon_t* polygon, const CCTNum_t p[3]);
extern int OBB_Contain_Point(const GeometryOBB_t* obb, const CCTNum_t p[3]);
extern int ConvexMesh_Contain_Point(const GeometryMesh_t* mesh, const CCTNum_t p[3]);
extern int Capsule_Contain_Point(const GeometryCapsule_t* capsule, const CCTNum_t p[3]);
extern CCTNum_t Segment_ClosestLenSq_Segment(const CCTNum_t ls1[2][3], const CCTNum_t ls1_dir[3], CCTNum_t ls1_len, const CCTNum_t ls2[2][3], const CCTNum_t ls2_dir[3], CCTNum_t ls2_len);

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
	unsigned int i, polygon_edge_v_indices_cnt;
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
		return Polygon_Contain_Point_SamePlane(polygon, p, NULL);
	}
	if (Polygon_Contain_Point_SamePlane(polygon, ls[0], NULL) || Polygon_Contain_Point_SamePlane(polygon, ls[1], NULL)) {
		return 2;
	}
	polygon_edge_v_indices_cnt = polygon->edge_cnt + polygon->edge_cnt;
	for (i = 0; i < polygon_edge_v_indices_cnt; ) {
		CCTNum_t edge[2][3];
		mathVec3Copy(edge[0], polygon->v[polygon->edge_v_indices_flat[i++]]);
		mathVec3Copy(edge[1], polygon->v[polygon->edge_v_indices_flat[i++]]);
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
	min_lensq = Segment_ClosestLenSq_Segment(ls, NULL, 0, (const CCTNum_t(*)[3])edge, capsule->axis, capsule->half + capsule->half);
	return min_lensq <= CCTNum_sq(capsule->radius);
}

int Polygon_Intersect_Polygon(const GeometryPolygon_t* polygon1, const GeometryPolygon_t* polygon2, int* ret_plane_side) {
	int plane_side = 0;
	unsigned int i, polygon2_edge_v_indices_cnt;
	unsigned int polygon1_edge_v_indices_cnt = polygon1->edge_cnt + polygon1->edge_cnt;
	for (i = 0; i < polygon1_edge_v_indices_cnt; i += 2) {
		int ret_side;
		CCTNum_t edge[2][3];
		mathVec3Copy(edge[0], polygon1->v[polygon1->edge_v_indices_flat[i]]);
		mathVec3Copy(edge[1], polygon1->v[polygon1->edge_v_indices_flat[i+1]]);
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
	polygon2_edge_v_indices_cnt = polygon2->edge_cnt + polygon2->edge_cnt;
	for (i = 0; i < polygon2_edge_v_indices_cnt; ) {
		CCTNum_t edge[2][3];
		mathVec3Copy(edge[0], polygon2->v[polygon2->edge_v_indices_flat[i++]]);
		mathVec3Copy(edge[1], polygon2->v[polygon2->edge_v_indices_flat[i++]]);
		if (Segment_Intersect_Polygon((const CCTNum_t(*)[3])edge, polygon1, NULL)) {
			return 1;
		}
	}
	return 0;
}

int ConvexMesh_Intersect_Polygon(const GeometryMesh_t* mesh, const GeometryPolygon_t* polygon, int* ret_plane_side) {
	int plane_side = 0;
	unsigned int i, polygon_edge_v_indices_cnt;
	unsigned int mesh_edge_v_indices_cnt = mesh->edge_cnt + mesh->edge_cnt;
	for (i = 0; i < mesh_edge_v_indices_cnt; i += 2) {
		int ret_side;
		CCTNum_t edge[2][3];
		mathVec3Copy(edge[0], mesh->v[mesh->edge_v_indices_flat[i]]);
		mathVec3Copy(edge[1], mesh->v[mesh->edge_v_indices_flat[i+1]]);
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
	polygon_edge_v_indices_cnt = polygon->edge_cnt + polygon->edge_cnt;
	for (i = 0; i < polygon_edge_v_indices_cnt; ) {
		CCTNum_t edge[2][3];
		mathVec3Copy(edge[0], polygon->v[polygon->edge_v_indices_flat[i++]]);
		mathVec3Copy(edge[1], polygon->v[polygon->edge_v_indices_flat[i++]]);
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
	if (abs_d > capsule->radius + CCT_EPSILON) {
		return 0;
	}
	if (abs_d < capsule->radius - CCT_EPSILON) {
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
	int res, i, polygon_edge_v_indices_cnt;
	CCTNum_t p[3], d[3];
	if (ret_plane_side) {
		*ret_plane_side = 0;
	}
	res = Segment_Intersect_Plane((const CCTNum_t(*)[3])capsule_extra->axis_edge, polygon->v[polygon->v_indices[0]], polygon->normal, NULL, d);
	if (2 == res) {
		if (Polygon_Contain_Point_SamePlane(polygon, capsule->o, NULL)) {
			return 1;
		}
		if (Polygon_Contain_Point_SamePlane(polygon, capsule_extra->axis_edge[0], NULL)) {
			return 1;
		}
		if (Polygon_Contain_Point_SamePlane(polygon, capsule_extra->axis_edge[1], NULL)) {
			return 1;
		}
	}
	else if (1 == res) {
		CCTNum_t cos_theta = mathVec3Dot(capsule->axis, polygon->normal);
		mathVec3Copy(p, capsule_extra->axis_edge[0]);
		mathVec3AddScalar(p, capsule->axis, d[0] / cos_theta);
		if (Polygon_Contain_Point_SamePlane(polygon, p, NULL)) {
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
			if (Polygon_Contain_Point_SamePlane(polygon, p, NULL)) {
				return 1;
			}
		}
		if (CCTNum_abs(d[1]) <= capsule->radius) {
			mathVec3Copy(p, capsule_extra->axis_edge[1]);
			mathVec3AddScalar(p, polygon->normal, d[1]);
			if (Polygon_Contain_Point_SamePlane(polygon, p, NULL)) {
				return 1;
			}
		}
		if (d[0] == d[1]) {
			mathVec3Copy(p, capsule->o);
			mathVec3AddScalar(p, polygon->normal, d[2]);
			if (Polygon_Contain_Point_SamePlane(polygon, p, NULL)) {
				return 1;
			}
		}
	}
	polygon_edge_v_indices_cnt = polygon->edge_cnt + polygon->edge_cnt;
	for (i = 0; i < polygon_edge_v_indices_cnt; ) {
		CCTNum_t edge[2][3], lensq;
		mathVec3Copy(edge[0], polygon->v[polygon->edge_v_indices_flat[i++]]);
		mathVec3Copy(edge[1], polygon->v[polygon->edge_v_indices_flat[i++]]);
		lensq = Segment_ClosestLenSq_Segment(
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
	min_lensq = Segment_ClosestLenSq_Segment(
		(const CCTNum_t(*)[3])c1_edge, c1->axis, c1->half + c1->half,
		(const CCTNum_t(*)[3])c2_edge, c2->axis, c2->half + c2->half
	);
	radius_sum = c1->radius + c2->radius;
	return min_lensq <= CCTNum_sq(radius_sum) + CCT_EPSILON;
}

int Sphere_Intersect_Segment(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t ls0[3], const CCTNum_t ls1[3]) {
	CCTNum_t lensq, radius_sq;
	CCTNum_t closest_p[3];
	mathSegmentClosestPoint(ls0, ls1, o, closest_p);
	lensq = mathVec3DistanceSq(closest_p, o);
	radius_sq = CCTNum_sq(radius);
	if (lensq > radius_sq + CCT_EPSILON) {
		return 0;
	}
	if (lensq < radius_sq - CCT_EPSILON) {
		return 2;
	}
	return 1;
}

static int Sphere_Intersect_Capsule(const CCTNum_t o[3], CCTNum_t radius, const GeometryCapsule_t* capsule) {
	CCTNum_t lensq, radius_sq;
	CCTNum_t closest_p[3];
	mathSegmentClosestPoint_v2(capsule->o, capsule->axis, capsule->half, o, closest_p);
	lensq = mathVec3DistanceSq(o, closest_p);
	radius_sq = CCTNum_sq(radius + capsule->radius);
	if (lensq > radius_sq + CCT_EPSILON) {
		return 0;
	}
	if (lensq < radius_sq - CCT_EPSILON) {
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
	if (abs_d > radius + CCT_EPSILON) {
		if (new_r) {
			*new_r = CCTNum(0.0);
		}
		return 0;
	}
	if (abs_d >= radius - CCT_EPSILON) {
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
	unsigned int i, polygon_edge_v_indices_cnt;
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
	if (Polygon_Contain_Point_SamePlane(polygon, p, NULL)) {
		return res;
	}
	if (1 == res) {
		return 0;
	}
	polygon_edge_v_indices_cnt = polygon->edge_cnt + polygon->edge_cnt;
	for (i = 0; i < polygon_edge_v_indices_cnt; ) {
		unsigned int v_idx[2];
		v_idx[0] = polygon->edge_v_indices_flat[i++];
		v_idx[1] = polygon->edge_v_indices_flat[i++];
		res = Sphere_Intersect_Segment(o, radius, polygon->v[v_idx[0]], polygon->v[v_idx[1]]);
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
	mathOBBClosestPoint(obb, o, v);
	mathVec3Sub(v, o, v);
	return mathVec3LenSq(v) <= CCTNum_sq(radius);
}

static int Sphere_Intersect_Sphere(const CCTNum_t o1[3], CCTNum_t r1, const CCTNum_t o2[3], CCTNum_t r2, CCTNum_t p[3]) {
	CCTNum_t o1o2[3];
	CCTNum_t o1o2_lensq, radius_sum_sq = CCTNum_sq(r1 + r2);
	mathVec3Sub(o1o2, o2, o1);
	o1o2_lensq = mathVec3LenSq(o1o2);
	if (o1o2_lensq > radius_sum_sq + CCT_EPSILON) {
		return 0;
	}
	if (o1o2_lensq < radius_sum_sq - CCT_EPSILON) {
		return 2;
	}
	if (p) {
		mathVec3Normalized(o1o2, o1o2);
		mathVec3AddScalar(mathVec3Copy(p, o1), o1o2, r1);
	}
	return 1;
}

static int Sphere_Intersect_AABB(const CCTNum_t sp_o[3], CCTNum_t sp_radius, const CCTNum_t aabb_min_v[3], const CCTNum_t aabb_max_v[3]) {
	CCTNum_t closest_v[3];
	mathAABBClosestPoint(aabb_min_v, aabb_max_v, sp_o, closest_v);
	mathVec3Sub(closest_v, closest_v, sp_o);
	return mathVec3LenSq(closest_v) <= CCTNum_sq(sp_radius);
}

static int AABB_Intersect_Segment(const CCTNum_t min_v[3], const CCTNum_t max_v[3], const CCTNum_t ls[2][3]) {
	int i;
	CCTNum_t v[8][3];
	if (AABB_Contain_Point(min_v, max_v, ls[0]) || AABB_Contain_Point(min_v, max_v, ls[1])) {
		return 1;
	}
	mathAABBVertices(min_v, max_v, v);
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

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
extern "C" {
#endif

int mathGeometryIntersect(const void* geo_data1, int geo_type1, const void* geo_data2, int geo_type2) {
	GeometryBoxMesh_t box_data;
	const GeometryMesh_t* mesh2;
	if (geo_data1 == geo_data2) {
		return 1;
	}
	if (GEOMETRY_BODY_POINT == geo_type1) {
		const CCTNum_t* point1 = (const CCTNum_t*)geo_data1;
		switch (geo_type2) {
			case GEOMETRY_BODY_POINT:
			{
				return mathVec3Equal((const CCTNum_t*)geo_data2, point1);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				return Segment_Contain_Point(segment2->v[0], segment2->v[1], point1);
			}
			case GEOMETRY_BODY_PLANE:
			{
				const GeometryPlane_t* plane2 = (const GeometryPlane_t*)geo_data2;
				return Plane_Contain_Point(plane2->v, plane2->normal, point1);
			}
			case GEOMETRY_BODY_AABB:
			{
				const GeometryAABB_t* aabb2 = (const GeometryAABB_t*)geo_data2;
				return AABB_Contain_Point(aabb2->min_v, aabb2->max_v, point1);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				const GeometrySphere_t* sphere2 = (const GeometrySphere_t*)geo_data2;
				return Sphere_Contain_Point(sphere2->o, sphere2->radius, point1);
			}
			case GEOMETRY_BODY_OBB:
			{
				return OBB_Contain_Point((const GeometryOBB_t*)geo_data2, point1);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				return Polygon_Contain_Point((const GeometryPolygon_t*)geo_data2, point1);
			}
			case GEOMETRY_BODY_MESH:
			{
				mesh2 = (const GeometryMesh_t*)geo_data2;
				if (!mesh2->is_convex || !mesh2->is_closed) {
					return 0;
				}
				return ConvexMesh_Contain_Point(mesh2, point1);
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				return Capsule_Contain_Point((const GeometryCapsule_t*)geo_data2, point1);
			}
		}
	}
	else if (GEOMETRY_BODY_SEGMENT == geo_type1) {
		const GeometrySegment_t* segment1 = (const GeometrySegment_t*)geo_data1;
		const CCTNum_t(*segment1_v)[3] = (const CCTNum_t(*)[3])segment1->v;
		switch (geo_type2) {
			case GEOMETRY_BODY_POINT:
			{
				return Segment_Contain_Point(segment1_v[0], segment1_v[1], (const CCTNum_t*)geo_data2);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				return Segment_Intersect_Segment(segment1_v, (const CCTNum_t(*)[3])segment2->v);
			}
			case GEOMETRY_BODY_PLANE:
			{
				const GeometryPlane_t* plane2 = (const GeometryPlane_t*)geo_data2;
				return Segment_Intersect_Plane(segment1_v, plane2->v, plane2->normal, NULL, NULL);
			}
			case GEOMETRY_BODY_AABB:
			{
				const GeometryAABB_t* aabb2 = (const GeometryAABB_t*)geo_data2;
				return AABB_Intersect_Segment(aabb2->min_v, aabb2->max_v, segment1_v);
			}
			case GEOMETRY_BODY_OBB:
			{
				return Segment_Intersect_OBB(segment1_v, (const GeometryOBB_t*)geo_data2);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				const GeometrySphere_t* sphere2 = (const GeometrySphere_t*)geo_data2;
				return Sphere_Intersect_Segment(sphere2->o, sphere2->radius, segment1_v[0], segment1_v[1]);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				return Segment_Intersect_Polygon(segment1_v, (const GeometryPolygon_t*)geo_data2, NULL);
			}
			case GEOMETRY_BODY_MESH:
			{
				mesh2 = (const GeometryMesh_t*)geo_data2;
				if (!mesh2->is_convex || !mesh2->is_closed) {
					return 0;
				}
				return Segment_Intersect_ConvexMesh(segment1_v, mesh2);
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				return Segment_Intersect_Capsule(segment1_v, (const GeometryCapsule_t*)geo_data2);
			}
		}
	}
	else if (GEOMETRY_BODY_AABB == geo_type1) {
		const GeometryAABB_t* aabb1 = (const GeometryAABB_t*)geo_data1;
		switch (geo_type2) {
			case GEOMETRY_BODY_POINT:
			{
				return AABB_Contain_Point(aabb1->min_v, aabb1->max_v, (const CCTNum_t*)geo_data2);
			}
			case GEOMETRY_BODY_AABB:
			{
				const GeometryAABB_t* aabb2 = (const GeometryAABB_t*)geo_data2;
				return mathAABBIntersectAABB(aabb1->min_v, aabb1->max_v, aabb2->min_v, aabb2->max_v);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				const GeometrySphere_t* sphere2 = (const GeometrySphere_t*)geo_data2;
				return Sphere_Intersect_AABB(sphere2->o, sphere2->radius, aabb1->min_v, aabb1->max_v);
			}
			case GEOMETRY_BODY_PLANE:
			{
				const GeometryPlane_t* plane2 = (const GeometryPlane_t*)geo_data2;
				CCTNum_t v[8][3];
				mathAABBVertices(aabb1->min_v, aabb1->max_v, v);
				return Vertices_Intersect_Plane((const CCTNum_t(*)[3])v, Box_Vertice_Indices_Default, 8, plane2->v, plane2->normal);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				return AABB_Intersect_Segment(aabb1->min_v, aabb1->max_v, (const CCTNum_t(*)[3])segment2->v);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				mathAABBMesh(&box_data, aabb1->min_v, aabb1->max_v);
				return ConvexMesh_Intersect_Polygon(&box_data.mesh, (const GeometryPolygon_t*)geo_data2, NULL);
			}
			case GEOMETRY_BODY_OBB:
			{
				GeometryOBB_t obb1;
				mathOBBFromAABB(&obb1, aabb1->min_v, aabb1->max_v);
				return OBB_Intersect_OBB(&obb1, (const GeometryOBB_t*)geo_data2);
			}
			case GEOMETRY_BODY_MESH:
			{
				mesh2 = (const GeometryMesh_t*)geo_data2;
				if (!mesh2->is_convex || !mesh2->is_closed) {
					return 0;
				}
				mathAABBMesh(&box_data, aabb1->min_v, aabb1->max_v);
				return ConvexMesh_Intersect_ConvexMesh(&box_data.mesh, mesh2);
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				mathAABBMesh(&box_data, aabb1->min_v, aabb1->max_v);
				return Capsule_Intersect_ConvexMesh((const GeometryCapsule_t*)geo_data2, &box_data.mesh);
			}
		}
	}
	else if (GEOMETRY_BODY_SPHERE == geo_type1) {
		const GeometrySphere_t* sphere1 = (const GeometrySphere_t*)geo_data1;
		switch (geo_type2) {
			case GEOMETRY_BODY_POINT:
			{
				return Sphere_Contain_Point(sphere1->o, sphere1->radius, (const CCTNum_t*)geo_data2);
			}
			case GEOMETRY_BODY_AABB:
			{
				const GeometryAABB_t* aabb2 = (const GeometryAABB_t*)geo_data2;
				return Sphere_Intersect_AABB(sphere1->o, sphere1->radius, aabb2->min_v, aabb2->max_v);
			}
			case GEOMETRY_BODY_OBB:
			{
				return Sphere_Intersect_OBB(sphere1->o, sphere1->radius, (const GeometryOBB_t*)geo_data2);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				const GeometrySphere_t* sphere2 = (const GeometrySphere_t*)geo_data2;
				return Sphere_Intersect_Sphere(sphere1->o, sphere1->radius, sphere2->o, sphere2->radius, NULL);
			}
			case GEOMETRY_BODY_PLANE:
			{
				const GeometryPlane_t* plane2 = (const GeometryPlane_t*)geo_data2;
				return Sphere_Intersect_Plane(sphere1->o, sphere1->radius, plane2->v, plane2->normal, NULL, NULL);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				return Sphere_Intersect_Segment(sphere1->o, sphere1->radius, segment2->v[0], segment2->v[1]);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				return Sphere_Intersect_Polygon(sphere1->o, sphere1->radius, (const GeometryPolygon_t*)geo_data2, NULL);
			}
			case GEOMETRY_BODY_MESH:
			{
				mesh2 = (const GeometryMesh_t*)geo_data2;
				if (!mesh2->is_convex || !mesh2->is_closed) {
					return 0;
				}
				return Sphere_Intersect_ConvexMesh(sphere1->o, sphere1->radius, mesh2);
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				return Sphere_Intersect_Capsule(sphere1->o, sphere1->radius, (const GeometryCapsule_t*)geo_data2);
			}
		}
	}
	else if (GEOMETRY_BODY_PLANE == geo_type1) {
		const GeometryPlane_t* plane1 = (const GeometryPlane_t*)geo_data1;
		switch (geo_type2) {
			case GEOMETRY_BODY_POINT:
			{
				return Plane_Contain_Point(plane1->v, plane1->normal, (const CCTNum_t*)geo_data2);
			}
			case GEOMETRY_BODY_AABB:
			{
				const GeometryAABB_t* aabb2 = (const GeometryAABB_t*)geo_data2;
				CCTNum_t v[8][3];
				mathAABBVertices(aabb2->min_v, aabb2->max_v, v);
				return Vertices_Intersect_Plane((const CCTNum_t(*)[3])v, Box_Vertice_Indices_Default, 8, plane1->v, plane1->normal);
			}
			case GEOMETRY_BODY_OBB:
			{
				const GeometryOBB_t* obb2 = (const GeometryOBB_t*)geo_data2;
				CCTNum_t v[8][3];
				mathOBBVertices(obb2, v);
				return Vertices_Intersect_Plane((const CCTNum_t(*)[3])v, Box_Vertice_Indices_Default, 8, plane1->v, plane1->normal);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				const GeometrySphere_t* sphere2 = (const GeometrySphere_t*)geo_data2;
				return Sphere_Intersect_Plane(sphere2->o, sphere2->radius, plane1->v, plane1->normal, NULL, NULL);
			}
			case GEOMETRY_BODY_PLANE:
			{
				const GeometryPlane_t* plane2 = (const GeometryPlane_t*)geo_data2;
				return Plane_Intersect_Plane(plane1->v, plane1->normal, plane2->v, plane2->normal);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				return Segment_Intersect_Plane((const CCTNum_t(*)[3])segment2->v, plane1->v, plane1->normal, NULL, NULL);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				const GeometryPolygon_t* polygon2 = (const GeometryPolygon_t*)geo_data2;
				return Vertices_Intersect_Plane((const CCTNum_t(*)[3])polygon2->v, polygon2->v_indices, polygon2->v_indices_cnt, plane1->v, plane1->normal);
			}
			case GEOMETRY_BODY_MESH:
			{
				mesh2 = (const GeometryMesh_t*)geo_data2;
				return Vertices_Intersect_Plane((const CCTNum_t(*)[3])mesh2->v, mesh2->v_indices, mesh2->v_indices_cnt, plane1->v, plane1->normal);
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				return Capsule_Intersect_Plane((const GeometryCapsule_t*)geo_data2, plane1->v, plane1->normal);
			}
		}
	}
	else if (GEOMETRY_BODY_POLYGON == geo_type1) {
		const GeometryPolygon_t* polygon1 = (const GeometryPolygon_t*)geo_data1;
		switch (geo_type2) {
			case GEOMETRY_BODY_POINT:
			{
				return Polygon_Contain_Point(polygon1, (const CCTNum_t*)geo_data2);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				return Segment_Intersect_Polygon((const CCTNum_t(*)[3])segment2->v, polygon1, NULL);
			}
			case GEOMETRY_BODY_PLANE:
			{
				const GeometryPlane_t* plane2 = (const GeometryPlane_t*)geo_data2;
				return Vertices_Intersect_Plane((const CCTNum_t(*)[3])polygon1->v, polygon1->v_indices, polygon1->v_indices_cnt, plane2->v, plane2->normal);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				const GeometrySphere_t* sphere2 = (const GeometrySphere_t*)geo_data2;
				return Sphere_Intersect_Polygon(sphere2->o, sphere2->radius, polygon1, NULL);
			}
			case GEOMETRY_BODY_AABB:
			{
				const GeometryAABB_t* aabb2 = (const GeometryAABB_t*)geo_data2;
				mathAABBMesh(&box_data, aabb2->min_v, aabb2->max_v);
				return ConvexMesh_Intersect_Polygon(&box_data.mesh, polygon1, NULL);
			}
			case GEOMETRY_BODY_OBB:
			{
				const GeometryOBB_t* obb2 = (const GeometryOBB_t*)geo_data2;
				mathBoxMesh(&box_data, obb2->o, obb2->half, (const CCTNum_t(*)[3])obb2->axis);
				return ConvexMesh_Intersect_Polygon(&box_data.mesh, polygon1, NULL);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				return Polygon_Intersect_Polygon(polygon1, (const GeometryPolygon_t*)geo_data2, NULL);
			}
			case GEOMETRY_BODY_MESH:
			{
				mesh2 = (const GeometryMesh_t*)geo_data2;
				if (!mesh2->is_convex || !mesh2->is_closed) {
					return 0;
				}
				return ConvexMesh_Intersect_Polygon(mesh2, polygon1, NULL);
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				const GeometryCapsule_t* capsule2 = (const GeometryCapsule_t*)geo_data2;
				GeometryCapsuleExtra_t capsule2_extra;
				capsule_fill_extra(capsule2, &capsule2_extra);
				return Capsule_Intersect_Polygon(capsule2, &capsule2_extra, polygon1, NULL);
			}
		}
	}
	else if (GEOMETRY_BODY_OBB == geo_type1) {
		const GeometryOBB_t* obb1 = (const GeometryOBB_t*)geo_data1;
		switch (geo_type2) {
			case GEOMETRY_BODY_POINT:
			{
				return OBB_Contain_Point(obb1, (const CCTNum_t*)geo_data2);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				return Segment_Intersect_OBB((const CCTNum_t(*)[3])segment2->v, obb1);
			}
			case GEOMETRY_BODY_PLANE:
			{
				const GeometryPlane_t* plane2 = (const GeometryPlane_t*)geo_data2;
				CCTNum_t v[8][3];
				mathOBBVertices(obb1, v);
				return Vertices_Intersect_Plane((const CCTNum_t(*)[3])v, Box_Vertice_Indices_Default, 8, plane2->v, plane2->normal);
			}
			case GEOMETRY_BODY_OBB:
			{
				return OBB_Intersect_OBB(obb1, (const GeometryOBB_t*)geo_data2);
			}
			case GEOMETRY_BODY_AABB:
			{
				const GeometryAABB_t* aabb2 = (const GeometryAABB_t*)geo_data2;
				GeometryOBB_t obb2;
				mathOBBFromAABB(&obb2, aabb2->min_v, aabb2->max_v);
				return OBB_Intersect_OBB(obb1, &obb2);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				const GeometrySphere_t* sphere2 = (const GeometrySphere_t*)geo_data2;
				return Sphere_Intersect_OBB(sphere2->o, sphere2->radius, obb1);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				mathBoxMesh(&box_data, obb1->o, obb1->half, (const CCTNum_t(*)[3])obb1->axis);
				return ConvexMesh_Intersect_Polygon(&box_data.mesh, (const GeometryPolygon_t*)geo_data2, NULL);
			}
			case GEOMETRY_BODY_MESH:
			{
				mesh2 = (const GeometryMesh_t*)geo_data2;
				if (!mesh2->is_convex || !mesh2->is_closed) {
					return 0;
				}
				mathBoxMesh(&box_data, obb1->o, obb1->half, (const CCTNum_t(*)[3])obb1->axis);
				return ConvexMesh_Intersect_ConvexMesh(&box_data.mesh, mesh2);
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				mathBoxMesh(&box_data, obb1->o, obb1->half, (const CCTNum_t(*)[3])obb1->axis);
				return Capsule_Intersect_ConvexMesh((const GeometryCapsule_t*)geo_data2, &box_data.mesh);
			}
		}
	}
	else if (GEOMETRY_BODY_MESH == geo_type1) {
		const GeometryMesh_t* mesh1 = (const GeometryMesh_t*)geo_data1;
		if (mesh1->is_convex && mesh1->is_closed) {
			switch (geo_type2) {
				case GEOMETRY_BODY_POINT:
				{
					return ConvexMesh_Contain_Point(mesh1, (const CCTNum_t*)geo_data2);
				}
				case GEOMETRY_BODY_SEGMENT:
				{
					const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
					return Segment_Intersect_ConvexMesh((const CCTNum_t(*)[3])segment2->v, mesh1);
				}
				case GEOMETRY_BODY_PLANE:
				{
					const GeometryPlane_t* plane2 = (const GeometryPlane_t*)geo_data2;
					return Vertices_Intersect_Plane((const CCTNum_t(*)[3])mesh1->v, mesh1->v_indices, mesh1->v_indices_cnt, plane2->v, plane2->normal);
				}
				case GEOMETRY_BODY_SPHERE:
				{
					const GeometrySphere_t* sphere2 = (const GeometrySphere_t*)geo_data2;
					return Sphere_Intersect_ConvexMesh(sphere2->o, sphere2->radius, mesh1);
				}
				case GEOMETRY_BODY_AABB:
				{
					const GeometryAABB_t* aabb2 = (const GeometryAABB_t*)geo_data2;
					mathAABBMesh(&box_data, aabb2->min_v, aabb2->max_v);
					return ConvexMesh_Intersect_ConvexMesh(&box_data.mesh, mesh1);
				}
				case GEOMETRY_BODY_OBB:
				{
					const GeometryOBB_t* obb2 = (const GeometryOBB_t*)geo_data2;
					mathBoxMesh(&box_data, obb2->o, obb2->half, (const CCTNum_t(*)[3])obb2->axis);
					return ConvexMesh_Intersect_ConvexMesh(&box_data.mesh, mesh1);
				}
				case GEOMETRY_BODY_POLYGON:
				{
					return ConvexMesh_Intersect_Polygon(mesh1, (const GeometryPolygon_t*)geo_data2, NULL);
				}
				case GEOMETRY_BODY_MESH:
				{
					mesh2 = (const GeometryMesh_t*)geo_data2;
					if (!mesh2->is_convex || !mesh2->is_closed) {
						return 0;
					}
					return ConvexMesh_Intersect_ConvexMesh(mesh1, mesh2);
				}
				case GEOMETRY_BODY_CAPSULE:
				{
					return Capsule_Intersect_ConvexMesh((const GeometryCapsule_t*)geo_data2, mesh1);
				}
			}
		}
		else if (GEOMETRY_BODY_PLANE == geo_type2) {
			const GeometryPlane_t* plane2 = (const GeometryPlane_t*)geo_data2;
			return Vertices_Intersect_Plane((const CCTNum_t(*)[3])mesh1->v, mesh1->v_indices, mesh1->v_indices_cnt, plane2->v, plane2->normal);
		}
	}
	else if (GEOMETRY_BODY_CAPSULE == geo_type1) {
		const GeometryCapsule_t* capsule1 = (const GeometryCapsule_t*)geo_data1;
		switch (geo_type2) {
			case GEOMETRY_BODY_POINT:
			{
				return Capsule_Contain_Point(capsule1, (const CCTNum_t*)geo_data2);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				return Segment_Intersect_Capsule((const CCTNum_t(*)[3])segment2->v, capsule1);
			}
			case GEOMETRY_BODY_PLANE:
			{
				const GeometryPlane_t* plane2 = (const GeometryPlane_t*)geo_data2;
				return Capsule_Intersect_Plane(capsule1, plane2->v, plane2->normal);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				const GeometrySphere_t* sphere2 = (const GeometrySphere_t*)geo_data2;
				return Sphere_Intersect_Capsule(sphere2->o, sphere2->radius, capsule1);
			}
			case GEOMETRY_BODY_AABB:
			{
				const GeometryAABB_t* aabb2 = (const GeometryAABB_t*)geo_data2;
				mathAABBMesh(&box_data, aabb2->min_v, aabb2->max_v);
				return Capsule_Intersect_ConvexMesh(capsule1, &box_data.mesh);
			}
			case GEOMETRY_BODY_OBB:
			{
				const GeometryOBB_t* obb2 = (const GeometryOBB_t*)geo_data2;
				mathBoxMesh(&box_data, obb2->o, obb2->half, (const CCTNum_t(*)[3])obb2->axis);
				return Capsule_Intersect_ConvexMesh(capsule1, &box_data.mesh);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				GeometryCapsuleExtra_t capsule1_extra;
				capsule_fill_extra(capsule1, &capsule1_extra);
				return Capsule_Intersect_Polygon(capsule1, &capsule1_extra, (const GeometryPolygon_t*)geo_data2, NULL);
			}
			case GEOMETRY_BODY_MESH:
			{
				mesh2 = (const GeometryMesh_t*)geo_data2;
				if (!mesh2->is_convex || !mesh2->is_closed) {
					return 0;
				}
				return Capsule_Intersect_ConvexMesh(capsule1, mesh2);
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				return Capsule_Intersect_Capsule(capsule1, (const GeometryCapsule_t*)geo_data2);
			}
		}
	}
	return 0;
}

#ifdef __cplusplus
}
#endif
