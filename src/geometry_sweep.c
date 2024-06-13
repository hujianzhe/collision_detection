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
extern const CCTNum_t AABB_Plane_Normal[6][3];
extern const unsigned int Box_Edge_Indices[24];
extern const unsigned int Box_Vertice_Indices_Default[8];

extern int Segment_Contain_Point(const CCTNum_t ls[2][3], const CCTNum_t p[3]);
extern int Segment_Intersect_Segment(const CCTNum_t ls1[2][3], const CCTNum_t ls2[2][3], CCTNum_t p[3], int* line_mask);
extern int Segment_Intersect_Plane(const CCTNum_t ls[2][3], const CCTNum_t plane_v[3], const CCTNum_t plane_normal[3], CCTNum_t p[3]);
extern int Segment_Intersect_OBB(const CCTNum_t ls[2][3], const GeometryOBB_t* obb);
extern int Segment_Intersect_ConvexMesh(const CCTNum_t ls[2][3], const GeometryMesh_t* mesh);
extern int Sphere_Intersect_Segment(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t ls[2][3], CCTNum_t p[3]);
extern int Sphere_Intersect_Plane(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t plane_v[3], const CCTNum_t plane_normal[3], CCTNum_t new_o[3], CCTNum_t* new_r);
extern int OBB_Contain_Point(const GeometryOBB_t* obb, const CCTNum_t p[3]);
extern int OBB_Intersect_OBB(const GeometryOBB_t* obb0, const GeometryOBB_t* obb1);
extern int Sphere_Intersect_OBB(const CCTNum_t o[3], CCTNum_t radius, const GeometryOBB_t* obb);
extern int ConvexMesh_Contain_Point(const GeometryMesh_t* mesh, const CCTNum_t p[3]);
extern int ConvexMesh_Intersect_ConvexMesh(const GeometryMesh_t* mesh1, const GeometryMesh_t* mesh2);
extern int Polygon_Contain_Point(const GeometryPolygon_t* polygon, const CCTNum_t p[3]);
extern int Polygon_Intersect_Polygon(const GeometryPolygon_t* polygon1, const GeometryPolygon_t* polygon2);
extern int ConvexMesh_Intersect_Polygon(const GeometryMesh_t* mesh, const GeometryPolygon_t* polygon);
extern int Circle_Intersect_Plane(const GeometryCircle_t* circle, const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTNum_t p[3], CCTNum_t line[3]);
extern int Vertices_Intersect_Plane(const CCTNum_t(*v)[3], const unsigned int* v_indices, unsigned int v_indices_cnt, const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTNum_t* p_min_d, unsigned int* p_v_indices_idx);

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

static CCTSweepResult_t* copy_result(CCTSweepResult_t* dst, CCTSweepResult_t* src) {
	dst->distance = src->distance;
	mathVec3Copy(dst->hit_normal, src->hit_normal);
	return dst;
}

static CCTSweepResult_t* merge_result(CCTSweepResult_t* dst, CCTSweepResult_t* src) {
	if (dst->distance < src->distance) {
		return dst;
	}
	dst->distance = src->distance;
	mathVec3Copy(dst->hit_normal, src->hit_normal);
	return dst;
}

static CCTSweepResult_t* set_result(CCTSweepResult_t* result, CCTNum_t distance, const CCTNum_t hit_normal[3]) {
	result->distance = distance;
	result->hit_point_cnt = -1;
	mathVec3Copy(result->hit_normal, hit_normal);
	return result;
}

static CCTSweepResult_t* set_unique_hit_point(CCTSweepResult_t* result, const CCTNum_t p[3]) {
	mathVec3Copy(result->unique_hit_point, p);
	result->hit_point_cnt = 1;
	return result;
}

static void neg_dir_hit_points(CCTSweepResult_t* result, const CCTNum_t dir[3]) {
	if (1 == result->hit_point_cnt) {
		mathVec3AddScalar(result->unique_hit_point, dir, result->distance);
	}
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

static CCTSweepResult_t* Ray_Sweep_Segment(const CCTNum_t o[3], const CCTNum_t dir[3], const CCTNum_t ls[2][3], CCTSweepResult_t* result) {
	CCTNum_t v0[3], v1[3], N[3], dot;
	mathVec3Sub(v0, ls[0], o);
	mathVec3Sub(v1, ls[1], o);
	mathVec3Cross(N, v0, v1);
	if (mathVec3IsZero(N)) {
		dot = mathVec3Dot(v0, v1);
		if (dot <= CCTNum(0.0)) {
			set_result(result, CCTNum(0.0), dir);
			return result;
		}
		mathVec3Cross(N, dir, v0);
		if (!mathVec3IsZero(N)) {
			return NULL;
		}
		dot = mathVec3Dot(dir, v0);
		if (dot < CCTNum(0.0)) {
			return NULL;
		}
		if (mathVec3LenSq(v0) > mathVec3LenSq(v1)) {
			dot = mathVec3Dot(v1, dir);
		}
		set_result(result, dot, dir);
		return result;
	}
	else {
		CCTNum_t lsdir[3], p[3], op[3], d;
		dot = mathVec3Dot(N, dir);
		if (dot < CCT_EPSILON_NEGATE || dot > CCT_EPSILON) {
			return NULL;
		}
		mathVec3Sub(lsdir, ls[1], ls[0]);
		mathVec3Normalized(lsdir, lsdir);
		mathPointProjectionLine(o, ls[0], lsdir, p);

		mathVec3Sub(op, p, o);
		if (mathVec3IsZero(op)) {
			set_result(result, CCTNum(0.0), dir);
			return result;
		}
		dot = mathVec3Dot(op, dir);
		if (dot <= CCTNum(0.0)) {
			return NULL;
		}
		d = mathVec3Normalized(op, op);
		dot = mathVec3Dot(op, dir);
		d /= dot;
		mathVec3Copy(p, o);
		mathVec3AddScalar(p, dir, d);

		mathVec3Sub(v0, ls[0], p);
		mathVec3Sub(v1, ls[1], p);
		dot = mathVec3Dot(v0, v1);
		if (dot > CCTNum(0.0)) {
			return NULL;
		}
		set_result(result, d, op);
		return result;
	}
}

static CCTSweepResult_t* Ray_Sweep_Plane(const CCTNum_t o[3], const CCTNum_t dir[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTSweepResult_t* result) {
	CCTNum_t d, cos_theta;
	mathPointProjectionPlane(o, plane_v, plane_n, NULL, &d);
	if (d <= CCT_EPSILON && d >= CCT_EPSILON_NEGATE) {
		set_result(result, CCTNum(0.0), dir);
		return result;
	}
	cos_theta = mathVec3Dot(dir, plane_n);
	if (cos_theta <= CCT_EPSILON && cos_theta >= CCT_EPSILON_NEGATE) {
		return NULL;
	}
	d /= cos_theta;
	if (d < CCTNum(0.0)) {
		return NULL;
	}
	set_result(result, d, plane_n);
	return result;
}

static CCTSweepResult_t* Ray_Sweep_Polygon(const CCTNum_t o[3], const CCTNum_t dir[3], const GeometryPolygon_t* polygon, CCTSweepResult_t* result) {
	CCTSweepResult_t* p_result;
	int i;
	CCTNum_t dot, hit_point[3];
	if (!Ray_Sweep_Plane(o, dir, polygon->v[polygon->v_indices[0]], polygon->normal, result)) {
		return NULL;
	}
	mathVec3Copy(hit_point, o);
	mathVec3AddScalar(hit_point, dir, result->distance);
	if (Polygon_Contain_Point(polygon, hit_point)) {
		return result;
	}
	if (result->distance > CCTNum(0.0)) {
		return NULL;
	}
	dot = mathVec3Dot(dir, polygon->normal);
	if (dot < CCT_EPSILON_NEGATE || dot > CCT_EPSILON) {
		return NULL;
	}
	p_result = NULL;
	for (i = 0; i < polygon->v_indices_cnt; ) {
		CCTSweepResult_t result_temp;
		CCTNum_t edge[2][3];
		mathVec3Copy(edge[0], polygon->v[polygon->v_indices[i++]]);
		mathVec3Copy(edge[1], polygon->v[polygon->v_indices[i >= polygon->v_indices_cnt ? 0 : i]]);
		if (!Ray_Sweep_Segment(o, dir, (const CCTNum_t(*)[3])edge, &result_temp)) {
			continue;
		}
		if (!p_result) {
			p_result = result;
			copy_result(p_result, &result_temp);
		}
		else {
			merge_result(p_result, &result_temp);
		}
	}
	return p_result;
}

static CCTSweepResult_t* Ray_Sweep_OBB(const CCTNum_t o[3], const CCTNum_t dir[3], const GeometryOBB_t* obb, int check_intersect, CCTSweepResult_t* result) {
	if (check_intersect && OBB_Contain_Point(obb, o)) {
		set_result(result, CCTNum(0.0), dir);
		return result;
	}
	else {
		int i;
		CCTNum_t v[8][3];
		CCTSweepResult_t *p_result = NULL;
		mathOBBVertices(obb, v);
		for (i = 0; i < 6; ++i) {
			CCTNum_t hit_point[3];
			GeometryPolygon_t polygon;
			CCTSweepResult_t result_temp;
			mathBoxFace((const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])obb->axis, i, &polygon);
			if (!Ray_Sweep_Plane(o, dir, polygon.v[polygon.v_indices[0]], polygon.normal, &result_temp)) {
				continue;
			}
			mathVec3Copy(hit_point, o);
			mathVec3AddScalar(hit_point, dir, result_temp.distance);
			if (!Polygon_Contain_Point(&polygon, hit_point)) {
				continue;
			}
			if (!p_result) {
				p_result = result;
				copy_result(p_result, &result_temp);
			}
			else {
				merge_result(p_result, &result_temp);
			}
		}
		return p_result;
	}
}

static CCTSweepResult_t* Ray_Sweep_Sphere(const CCTNum_t o[3], const CCTNum_t dir[3], const CCTNum_t sp_o[3], CCTNum_t sp_radius, CCTSweepResult_t* result) {
	CCTNum_t dr2, oc2, dir_d;
	CCTNum_t oc[3], hit_point[3], hit_normal[3];
	CCTNum_t radius2 = sp_radius * sp_radius;
	mathVec3Sub(oc, sp_o, o);
	oc2 = mathVec3LenSq(oc);
	if (oc2 <= radius2) {
		set_result(result, CCTNum(0.0), dir);
		return result;
	}
	dir_d = mathVec3Dot(dir, oc);
	if (dir_d <= CCT_EPSILON) {
		return NULL;
	}
	dr2 = oc2 - dir_d * dir_d;
	if (dr2 > radius2) {
		return NULL;
	}
	dir_d -= CCTNum_sqrt(radius2 - dr2);
	mathVec3Copy(hit_point, o);
	mathVec3AddScalar(hit_point, dir, dir_d);
	mathVec3Sub(hit_normal, hit_point, sp_o);
	mathVec3MultiplyScalar(hit_normal, hit_normal, CCTNum(1.0) / sp_radius);
	set_result(result, dir_d, hit_normal);
	return result;
}

static CCTSweepResult_t* Ray_Sweep_Circle(const CCTNum_t o[3], const CCTNum_t dir[3], const GeometryCircle_t* circle, CCTSweepResult_t* result) {
	if (!Ray_Sweep_Plane(o, dir, circle->o, circle->normal, result)) {
		return NULL;
	}
	if (result->distance > CCTNum(0.0)) {
		CCTNum_t hit_point[3], lensq;
		mathVec3Copy(hit_point, o);
		mathVec3AddScalar(hit_point, dir, result->distance);
		lensq = mathVec3DistanceSq(circle->o, hit_point);
		if (lensq <= circle->radius * circle->radius) {
			return result;
		}
		return NULL;
	}
	return Ray_Sweep_Sphere(o, dir, circle->o, circle->radius, result);
}

static CCTSweepResult_t* Ray_Sweep_ConvexMesh(const CCTNum_t o[3], const CCTNum_t dir[3], const GeometryMesh_t* mesh, int check_intersect, CCTSweepResult_t* result) {
	unsigned int i;
	CCTSweepResult_t* p_result;
	if (check_intersect && ConvexMesh_Contain_Point(mesh, o)) {
		set_result(result, CCTNum(0.0), dir);
		return result;
	}
	p_result = NULL;
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		CCTSweepResult_t result_temp;
		if (!Ray_Sweep_Polygon(o, dir, mesh->polygons + i, &result_temp)) {
			continue;
		}
		if (!p_result) {
			p_result = result;
			copy_result(p_result, &result_temp);
		}
		else {
			merge_result(p_result, &result_temp);
		}
	}
	return p_result;
}

static CCTSweepResult_t* Segment_Sweep_Plane(const CCTNum_t ls[2][3], const CCTNum_t dir[3], const CCTNum_t vertice[3], const CCTNum_t normal[3], CCTSweepResult_t* result) {
	CCTNum_t p[3];
	int res = Segment_Intersect_Plane(ls, vertice, normal, p);
	if (2 == res) {
		set_result(result, CCTNum(0.0), dir);
		return result;
	}
	else if (1 == res) {
		set_result(result, CCTNum(0.0), normal);
		set_unique_hit_point(result, p);
		return result;
	}
	else {
		CCTNum_t d[2], min_d;
		CCTNum_t cos_theta = mathVec3Dot(normal, dir);
		if (cos_theta <= CCT_EPSILON && cos_theta >= CCT_EPSILON_NEGATE) {
			return NULL;
		}
		mathPointProjectionPlane(ls[0], vertice, normal, NULL, &d[0]);
		mathPointProjectionPlane(ls[1], vertice, normal, NULL, &d[1]);
		if (d[0] <= d[1] + CCT_EPSILON && d[0] >= d[1] - CCT_EPSILON) {
			min_d = d[0];
			min_d /= cos_theta;
			if (min_d < CCTNum(0.0)) {
				return NULL;
			}
			set_result(result, min_d, normal);
		}
		else {
			const CCTNum_t *p = NULL;
			if (d[0] > CCTNum(0.0)) {
				if (d[0] < d[1]) {
					min_d = d[0];
					p = ls[0];
				}
				else {
					min_d = d[1];
					p = ls[1];
				}
			}
			else {
				if (d[0] < d[1]) {
					min_d = d[1];
					p = ls[1];
				}
				else {
					min_d = d[0];
					p = ls[0];
				}
			}
			min_d /= cos_theta;
			if (min_d < CCTNum(0.0)) {
				return NULL;
			}
			set_result(result, min_d, normal);
			set_unique_hit_point(result, p);
			mathVec3AddScalar(result->unique_hit_point, dir, min_d);
		}
		return result;
	}
}

static CCTSweepResult_t* Segment_Sweep_Segment(const CCTNum_t ls1[2][3], const CCTNum_t dir[3], const CCTNum_t ls2[2][3], CCTSweepResult_t* result) {
	int line_mask;
	CCTNum_t p[3];
	int res = Segment_Intersect_Segment(ls1, ls2, p, &line_mask);
	if (GEOMETRY_SEGMENT_CONTACT == res) {
		set_result(result, CCTNum(0.0), dir);
		return result;
	}
	else if (GEOMETRY_SEGMENT_OVERLAP == res) {
		set_result(result, CCTNum(0.0), dir);
		return result;
	}
	else if (GEOMETRY_LINE_PARALLEL == line_mask) {
		CCTSweepResult_t* p_result = NULL;
		CCTNum_t neg_dir[3];
		int i;
		for (i = 0; i < 2; ++i) {
			CCTSweepResult_t result_temp;
			if (!Ray_Sweep_Segment(ls1[i], dir, ls2, &result_temp)) {
				continue;
			}
			if (!p_result) {
				p_result = result;
				copy_result(p_result, &result_temp);
			}
			else {
				merge_result(p_result, &result_temp);
			}
		}
		if (p_result) {
			p_result->hit_point_cnt = 2;
			return p_result;
		}
		mathVec3Negate(neg_dir, dir);
		for (i = 0; i < 2; ++i) {
			CCTSweepResult_t result_temp;
			if (!Ray_Sweep_Segment(ls2[i], neg_dir, ls1, &result_temp)) {
				continue;
			}
			if (!p_result) {
				p_result = result;
				copy_result(p_result, &result_temp);
			}
			else {
				merge_result(p_result, &result_temp);
			}
		}
		if (p_result) {
			p_result->hit_point_cnt = 2;
		}
		return p_result;
	}
	else if (GEOMETRY_LINE_CROSS == line_mask) {
		CCTSweepResult_t* p_result = NULL;
		CCTNum_t neg_dir[3];
		int i;
		for (i = 0; i < 2; ++i) {
			CCTSweepResult_t result_temp;
			if (!Ray_Sweep_Segment(ls1[i], dir, ls2, &result_temp)) {
				continue;
			}
			if (!p_result) {
				p_result = result;
				copy_result(p_result, &result_temp);
			}
			else {
				merge_result(p_result, &result_temp);
			}
		}
		mathVec3Negate(neg_dir, dir);
		for (i = 0; i < 2; ++i) {
			CCTSweepResult_t result_temp;
			if (!Ray_Sweep_Segment(ls2[i], neg_dir, ls1, &result_temp)) {
				continue;
			}
			if (!p_result) {
				p_result = result;
				copy_result(p_result, &result_temp);
			}
			else {
				merge_result(p_result, &result_temp);
			}
		}
		return p_result;
	}
	else if (GEOMETRY_LINE_OVERLAP == line_mask) {
		CCTNum_t v[3], N[3], dot;
		unsigned int closest_ls1_indice, closest_ls2_indice;
		mathVec3Sub(v, ls1[1], ls1[0]);
		mathVec3Cross(N, v, dir);
		if (!mathVec3IsZero(N)) {
			return NULL;
		}
		mathSegmentSegmentClosestIndices(ls1, ls2, &closest_ls1_indice, &closest_ls2_indice);
		mathVec3Sub(v, ls2[closest_ls2_indice], ls1[closest_ls1_indice]);
		dot = mathVec3Dot(v, dir);
		if (dot < CCTNum(0.0)) {
			return NULL;
		}
		set_result(result, dot, dir);
		return result;
	}
	else {
		CCTNum_t N[3], v[3], neg_dir[3];
		mathVec3Sub(v, ls1[1], ls1[0]);
		mathVec3Cross(N, v, dir);
		if (mathVec3IsZero(N)) {
			return NULL;
		}
		mathVec3Normalized(N, N);
		if (!Segment_Intersect_Plane(ls2, ls1[0], N, v)) {
			return NULL;
		}
		mathVec3Negate(neg_dir, dir);
		if (!Ray_Sweep_Segment(v, neg_dir, ls1, result)) {
			return NULL;
		}
		return result;
	}
}

typedef struct GeometrySegmentIndices_t {
	CCTNum_t(*v)[3];
	const unsigned int* indices;
	unsigned int indices_cnt;
	unsigned int stride;
} GeometrySegmentIndices_t;

static CCTSweepResult_t* Segment_Sweep_SegmentIndices(const CCTNum_t ls[2][3], const CCTNum_t dir[3], const GeometrySegmentIndices_t* si, CCTSweepResult_t* result) {
	int i;
	CCTSweepResult_t* p_result = NULL;
	for (i = 0; i < si->indices_cnt; ) {
		CCTSweepResult_t result_temp;
		CCTNum_t edge[2][3];
		mathVec3Copy(edge[0], si->v[si->indices[i++]]);
		if (2 == si->stride) {
			mathVec3Copy(edge[1], si->v[si->indices[i++]]);
		}
		else {
			mathVec3Copy(edge[1], si->v[si->indices[i >= si->indices_cnt ? 0 : i]]);
		}
		if (!Segment_Sweep_Segment(ls, dir, (const CCTNum_t(*)[3])edge, &result_temp)) {
			continue;
		}
		if (!p_result) {
			p_result = result;
			copy_result(p_result, &result_temp);
		}
		else {
			merge_result(p_result, &result_temp);
		}
	}
	return p_result;
}

static CCTSweepResult_t* SegmentIndices_Sweep_SegmentIndices(const GeometrySegmentIndices_t* s1, const CCTNum_t dir[3], const GeometrySegmentIndices_t* s2, CCTSweepResult_t* result) {
	int i;
	CCTSweepResult_t* p_result = NULL;
	for (i = 0; i < s1->indices_cnt; ) {
		int j;
		CCTNum_t edge1[2][3];
		mathVec3Copy(edge1[0], s1->v[s1->indices[i++]]);
		if (2 == s1->stride) {
			mathVec3Copy(edge1[1], s1->v[s1->indices[i++]]);
		}
		else {
			mathVec3Copy(edge1[1], s1->v[s1->indices[i >= s1->indices_cnt ? 0 : i]]);
		}
		for (j = 0; j < s2->indices_cnt; ) {
			CCTSweepResult_t result_temp;
			CCTNum_t edge2[2][3];
			mathVec3Copy(edge2[0], s2->v[s2->indices[j++]]);
			if (2 == s2->stride) {
				mathVec3Copy(edge2[1], s2->v[s2->indices[j++]]);
			}
			else {
				mathVec3Copy(edge2[1], s2->v[s2->indices[j >= s2->indices_cnt ? 0 : j]]);
			}
			if (!Segment_Sweep_Segment((const CCTNum_t(*)[3])edge1, dir, (const CCTNum_t(*)[3])edge2, &result_temp)) {
				continue;
			}
			if (!p_result) {
				p_result = result;
				copy_result(p_result, &result_temp);
			}
			else {
				merge_result(p_result, &result_temp);
			}
		}
	}
	return p_result;
}

static CCTSweepResult_t* Segment_Sweep_OBB(const CCTNum_t ls[2][3], const CCTNum_t dir[3], const GeometryOBB_t* obb, int check_intersect, CCTSweepResult_t* result) {
	if (check_intersect && Segment_Intersect_OBB(ls, obb)) {
		set_result(result, CCTNum(0.0), dir);
		return result;
	}
	else {
		int i;
		CCTSweepResult_t* p_result;
		GeometrySegmentIndices_t si;
		CCTNum_t v[8][3];
		mathOBBVertices(obb, v);
		si.v = v;
		si.indices = Box_Edge_Indices;
		si.indices_cnt = sizeof(Box_Edge_Indices) / sizeof(Box_Edge_Indices[0]);
		si.stride = 2;
		p_result = Segment_Sweep_SegmentIndices(ls, dir, &si, result);
		for (i = 0; i < 2; ++i) {
			CCTSweepResult_t result_temp;
			if (!Ray_Sweep_OBB(ls[i], dir, obb, 0, &result_temp)) {
				continue;
			}
			if (!p_result) {
				p_result = result;
				copy_result(p_result, &result_temp);
			}
			else {
				merge_result(p_result, &result_temp);
			}
		}
		return p_result;
	}
}

static CCTSweepResult_t* Segment_Sweep_Polygon(const CCTNum_t ls[2][3], const CCTNum_t dir[3], const GeometryPolygon_t* polygon, CCTSweepResult_t* result) {
	GeometrySegmentIndices_t si;
	if (!Segment_Sweep_Plane(ls, dir, polygon->v[polygon->v_indices[0]], polygon->normal, result)) {
		return NULL;
	}
	if (1 == result->hit_point_cnt) {
		CCTNum_t lsdir[3], N[3];
		if (Polygon_Contain_Point(polygon, result->unique_hit_point)) {
			return result;
		}
		mathVec3Sub(lsdir, ls[1], ls[0]);
		mathVec3Cross(N, lsdir, dir);
		if (mathVec3IsZero(N)) {
			return NULL;
		}
	}
	else if (result->distance > CCTNum(0.0)) {
		CCTNum_t test_p[3];
		mathVec3Copy(test_p, ls[0]);
		mathVec3AddScalar(test_p, dir, result->distance);
		if (Polygon_Contain_Point(polygon, test_p)) {
			return result;
		}
		mathVec3Copy(test_p, ls[1]);
		mathVec3AddScalar(test_p, dir, result->distance);
		if (Polygon_Contain_Point(polygon, test_p)) {
			return result;
		}
	}
	else if (Polygon_Contain_Point(polygon, ls[0]) || Polygon_Contain_Point(polygon, ls[1])) {
		return result;
	}
	si.v = polygon->v;
	si.indices = polygon->v_indices;
	si.indices_cnt = polygon->v_indices_cnt;
	si.stride = 1;
	return Segment_Sweep_SegmentIndices(ls, dir, &si, result);
}

static CCTSweepResult_t* Segment_Sweep_ConvexMesh(const CCTNum_t ls[2][3], const CCTNum_t dir[3], const GeometryMesh_t* mesh, CCTSweepResult_t* result) {
	unsigned int i;
	CCTSweepResult_t* p_result;
	GeometrySegmentIndices_t si;
	if (Segment_Intersect_ConvexMesh(ls, mesh)) {
		set_result(result, CCTNum(0.0), dir);
		return result;
	}
	si.v = mesh->v;
	si.indices = mesh->edge_indices;
	si.indices_cnt = mesh->edge_indices_cnt;
	si.stride = 2;
	p_result = Segment_Sweep_SegmentIndices(ls, dir, &si, result);
	for (i = 0; i < 2; ++i) {
		CCTSweepResult_t result_temp;
		unsigned int j;
		for (j = 0; j < mesh->polygons_cnt; ++j) {
			if (!Ray_Sweep_Polygon(ls[i], dir, mesh->polygons + j, &result_temp)) {
				continue;
			}
			if (!p_result) {
				p_result = result;
				copy_result(p_result, &result_temp);
			}
			else {
				merge_result(p_result, &result_temp);
			}
		}
	}
	return p_result;
}

static CCTSweepResult_t* Segment_Sweep_Circle_InSamePlane(const CCTNum_t ls[2][3], const CCTNum_t dir[3], const GeometryCircle_t* circle, CCTSweepResult_t* result) {
	CCTNum_t lsdir[3], p[3], pco[3];
	mathVec3Sub(lsdir, ls[1], ls[0]);
	mathVec3Normalized(lsdir, lsdir);
	mathPointProjectionLine(circle->o, ls[0], lsdir, p);
	mathVec3Sub(pco, circle->o, p);
	if (mathVec3LenSq(pco) > circle->radius * circle->radius) {
		CCTNum_t new_ls[2][3], d;
		CCTNum_t dot = mathVec3Dot(pco, dir);
		if (dot <= CCTNum(0.0)) {
			return NULL;
		}
		d = mathVec3Normalized(pco, pco);
		dot = mathVec3Dot(pco, dir);
		d -= circle->radius;
		mathVec3AddScalar(p, pco, d);
		d /= dot;
		if (d <= CCTNum(0.0)) {
			return NULL;
		}
		mathVec3Copy(new_ls[0], ls[0]);
		mathVec3Copy(new_ls[1], ls[1]);
		mathVec3AddScalar(new_ls[0], dir, d);
		mathVec3AddScalar(new_ls[1], dir, d);
		if (Segment_Contain_Point((const CCTNum_t(*)[3])new_ls, p)) {
			set_result(result, d, pco);
			return result;
		}
	}
	if (Ray_Sweep_Sphere(ls[0], dir, circle->o, circle->radius, result)) {
		CCTSweepResult_t result_temp;
		if (!Ray_Sweep_Sphere(ls[1], dir, circle->o, circle->radius, &result_temp)) {
			return result;
		}
		merge_result(result, &result_temp);
		return result;
	}
	return Ray_Sweep_Sphere(ls[1], dir, circle->o, circle->radius, result);
}

static CCTSweepResult_t* Segment_Sweep_Circle(const CCTNum_t ls[2][3], const CCTNum_t dir[3], const GeometryCircle_t* circle, CCTSweepResult_t* result) {
	if (!Segment_Sweep_Plane(ls, dir, circle->o, circle->normal, result)) {
		return NULL;
	}
	if (1 == result->hit_point_cnt) {
		int res;
		CCTNum_t lensq, N[3], lsdir[3], p[3];
		lensq = mathVec3DistanceSq(result->unique_hit_point, circle->o);
		if (lensq <= circle->radius * circle->radius) {
			return result;
		}
		mathVec3Sub(lsdir, ls[1], ls[0]);
		mathVec3Cross(N, dir, lsdir);
		if (mathVec3IsZero(N)) {
			return NULL;
		}
		res = Circle_Intersect_Plane(circle, ls[0], N, p, lsdir);
		if (0 == res) {
			return NULL;
		}
		if (1 == res) {
			CCTNum_t neg_dir[3];
			mathVec3Negate(neg_dir, dir);
			if (!Ray_Sweep_Segment(p, neg_dir, ls, result)) {
				return NULL;
			}
			set_unique_hit_point(result, p);
			return result;
		}
		if (3 == res) {
			CCTNum_t new_ls[2][3], half;
			lensq = mathVec3DistanceSq(circle->o, p);
			half = CCTNum_sqrt(circle->radius * circle->radius - lensq);
			mathVec3Copy(new_ls[0], p);
			mathVec3AddScalar(new_ls[0], lsdir, half);
			mathVec3Copy(new_ls[1], p);
			mathVec3SubScalar(new_ls[1], lsdir, half);
			return Segment_Sweep_Segment(ls, dir, (const CCTNum_t(*)[3])new_ls, result);
		}
		return NULL;
	}
	else if (result->distance > CCTNum(0.0)) {
		CCTNum_t new_ls[2][3], closest_p[3], lensq;
		mathVec3Copy(new_ls[0], ls[0]);
		mathVec3AddScalar(new_ls[0], dir, result->distance);
		mathVec3Copy(new_ls[1], ls[1]);
		mathVec3AddScalar(new_ls[1], dir, result->distance);
		mathSegmentClosestPointTo((const CCTNum_t(*)[3])new_ls, circle->o, closest_p);
		lensq = mathVec3DistanceSq(circle->o, closest_p);
		if (lensq > circle->radius * circle->radius) {
			return NULL;
		}
		return result;
	}
	else {
		CCTNum_t closest_p[3], lensq, dot;
		mathSegmentClosestPointTo(ls, circle->o, closest_p);
		lensq = mathVec3DistanceSq(circle->o, closest_p);
		if (lensq <= circle->radius * circle->radius) {
			return result;
		}
		dot = mathVec3Dot(circle->normal, dir);
		if (dot < CCT_EPSILON_NEGATE || dot > CCT_EPSILON) {
			return NULL;
		}
		return Segment_Sweep_Circle_InSamePlane(ls, dir, circle, result);
	}
}

static CCTSweepResult_t* Segment_Sweep_Sphere(const CCTNum_t ls[2][3], const CCTNum_t dir[3], const CCTNum_t center[3], CCTNum_t radius, int check_intersect, CCTSweepResult_t* result) {
	CCTNum_t lsdir[3], N[3];
	if (check_intersect) {
		int res = Sphere_Intersect_Segment(center, radius, ls, NULL);
		if (res) {
			set_result(result, CCTNum(0.0), dir);
			return result;
		}
	}
	mathVec3Sub(lsdir, ls[1], ls[0]);
	mathVec3Cross(N, lsdir, dir);
	if (!mathVec3IsZero(N)) {
		GeometryCircle_t circle;
		mathVec3Normalized(circle.normal, N);
		if (0 == Sphere_Intersect_Plane(center, radius, ls[0], circle.normal, circle.o, &circle.radius)) {
			return NULL;
		}
		return Segment_Sweep_Circle_InSamePlane(ls, dir, &circle, result);
	}
	if (Ray_Sweep_Sphere(ls[0], dir, center, radius, result)) {
		CCTSweepResult_t result_temp;
		if (!Ray_Sweep_Sphere(ls[1], dir, center, radius, &result_temp)) {
			return NULL;
		}
		merge_result(result, &result_temp);
		return result;
	}
	return NULL;
}

static CCTSweepResult_t* Circle_Sweep_Plane(const GeometryCircle_t* circle, const CCTNum_t dir[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTSweepResult_t* result) {
	CCTNum_t p[3];
	int res = Circle_Intersect_Plane(circle, plane_v, plane_n, p, NULL);
	if (res) {
		set_result(result, CCTNum(0.0), dir);
		return result;
	}
	if (!mathVec3Equal(p, circle->o)) {
		CCTNum_t v[3];
		mathVec3Sub(v, p, circle->o);
		mathVec3Normalized(v, v);
		mathVec3Copy(p, circle->o);
		mathVec3AddScalar(p, v, circle->radius);
		if (!Ray_Sweep_Plane(p, dir, plane_v, plane_n, result)) {
			return NULL;
		}
		set_unique_hit_point(result, p);
		mathVec3AddScalar(result->unique_hit_point, dir, result->distance);
		return result;
	}
	return Ray_Sweep_Plane(p, dir, plane_v, plane_n, result);
}

static CCTSweepResult_t* Vertices_Sweep_Plane(const CCTNum_t(*v)[3], const unsigned int* v_indices, unsigned int v_indices_cnt, const CCTNum_t dir[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTSweepResult_t* result) {
	CCTNum_t min_d, dot;
	unsigned int v_indices_idx;
	int res = Vertices_Intersect_Plane(v, v_indices, v_indices_cnt, plane_v, plane_n, &min_d, &v_indices_idx);
	if (res) {
		set_result(result, CCTNum(0.0), dir);
		if (1 == res && v_indices_idx != -1) {
			set_unique_hit_point(result, v[v_indices[v_indices_idx]]);
		}
		return result;
	}
	dot = mathVec3Dot(dir, plane_n);
	if (dot <= CCT_EPSILON && dot >= CCT_EPSILON_NEGATE) {
		return NULL;
	}
	min_d /= dot;
	if (min_d < CCTNum(0.0)) {
		return NULL;
	}
	set_result(result, min_d, plane_n);
	if (v_indices_idx != -1) {
		set_unique_hit_point(result, v[v_indices[v_indices_idx]]);
		mathVec3AddScalar(result->unique_hit_point, dir, min_d);
	}
	return result;
}

static CCTSweepResult_t* Polygon_Sweep_Polygon(const GeometryPolygon_t* polygon1, const CCTNum_t dir[3], const GeometryPolygon_t* polygon2, CCTSweepResult_t* result) {
	int i;
	GeometrySegmentIndices_t s1, s2;
	CCTNum_t neg_dir[3], *p2p;
	CCTSweepResult_t* p_result;

	p2p = polygon2->v[polygon2->v_indices[0]];
	if (!Vertices_Sweep_Plane((const CCTNum_t(*)[3])polygon1->v, polygon1->v_indices, polygon1->v_indices_cnt, dir, p2p, polygon2->normal, result)) {
		return NULL;
	}
	if (1 == result->hit_point_cnt) {
		if (Polygon_Contain_Point(polygon2, result->unique_hit_point)) {
			return result;
		}
	}
	else if (result->distance > CCTNum(0.0)) {
		for (i = 0; i < polygon1->v_indices_cnt; ++i) {
			CCTNum_t test_p[3];
			mathVec3Copy(test_p, polygon1->v[polygon1->v_indices[i]]);
			mathVec3AddScalar(test_p, dir, result->distance);
			if (Polygon_Contain_Point(polygon2, test_p)) {
				return result;
			}
		}
	}
	else if (Polygon_Intersect_Polygon(polygon1, polygon2)) {
		return result;
	}
	s1.v = polygon1->v;
	s1.indices = polygon1->v_indices;
	s1.indices_cnt = polygon1->v_indices_cnt;
	s1.stride = 1;
	s2.v = polygon2->v;
	s2.indices = polygon2->v_indices;
	s2.indices_cnt = polygon2->v_indices_cnt;
	s2.stride = 1;
	p_result = SegmentIndices_Sweep_SegmentIndices(&s1, dir, &s2, result);
	mathVec3Negate(neg_dir, dir);
	for (i = 0; i < polygon2->v_indices_cnt; ++i) {
		CCTSweepResult_t result_temp;
		p2p = polygon2->v[polygon2->v_indices[i]];
		if (!Ray_Sweep_Polygon(p2p, neg_dir, polygon1, &result_temp)) {
			continue;
		}
		if (!p_result) {
			p_result = result;
			copy_result(p_result, &result_temp);
		}
		else {
			merge_result(p_result, &result_temp);
		}
	}
	return p_result;
}

static CCTSweepResult_t* AABB_Sweep_AABB(const CCTNum_t o1[3], const CCTNum_t half1[3], const CCTNum_t dir[3], const CCTNum_t o2[3], const CCTNum_t half2[3], CCTSweepResult_t* result) {
	if (AABB_Intersect_AABB(o1, half1, o2, half2)) {
		set_result(result, CCTNum(0.0), dir);
		return result;
	}
	else {
		CCTSweepResult_t *p_result = NULL;
		int i;
		CCTNum_t v1[6][3], v2[6][3];
		mathAABBPlaneVertices(o1, half1, v1);
		mathAABBPlaneVertices(o2, half2, v2);
		for (i = 0; i < 6; ++i) {
			CCTNum_t new_o1[3];
			CCTSweepResult_t result_temp;
			if (i & 1) {
				if (!Ray_Sweep_Plane(v1[i], dir, v2[i-1], AABB_Plane_Normal[i], &result_temp)) {
					continue;
				}
			}
			else {
				if (!Ray_Sweep_Plane(v1[i], dir, v2[i+1], AABB_Plane_Normal[i], &result_temp)) {
					continue;
				}
			}
			mathVec3Copy(new_o1, o1);
			mathVec3AddScalar(new_o1, dir, result_temp.distance);
			if (!AABB_Intersect_AABB(new_o1, half1, o2, half2)) {
				continue;
			}
			if (!p_result) {
				p_result = result;
				copy_result(p_result, &result_temp);
			}
			else {
				merge_result(p_result, &result_temp);
			}
		}
		return p_result;
	}
}

static CCTSweepResult_t* ConvexMesh_Sweep_Polygon(const GeometryMesh_t* mesh, const CCTNum_t dir[3], const GeometryPolygon_t* polygon, CCTSweepResult_t* result) {
	int i;
	CCTNum_t* pp, neg_dir[3];
	CCTSweepResult_t* p_result;
	GeometrySegmentIndices_t s1, s2;

	pp = polygon->v[polygon->v_indices[0]];
	if (!Vertices_Sweep_Plane((const CCTNum_t(*)[3])mesh->v, mesh->v_indices, mesh->v_indices_cnt, dir, pp, polygon->normal, result)) {
		return NULL;
	}
	if (1 == result->hit_point_cnt) {
		if (Polygon_Contain_Point(polygon, result->unique_hit_point)) {
			return result;
		}
	}
	else if (result->distance > CCTNum(0.0)) {
		for (i = 0; i < mesh->v_indices_cnt; ++i) {
			CCTNum_t test_p[3];
			mathVec3Copy(test_p, mesh->v[mesh->v_indices[i]]);
			mathVec3AddScalar(test_p, dir, result->distance);
			if (Polygon_Contain_Point(polygon, test_p)) {
				return result;
			}
		}
	}
	else if (ConvexMesh_Intersect_Polygon(mesh, polygon)) {
		return result;
	}
	s1.v = mesh->v;
	s1.indices = mesh->edge_indices;
	s1.indices_cnt = mesh->edge_indices_cnt;
	s1.stride = 2;
	s2.v = polygon->v;
	s2.indices = polygon->v_indices;
	s2.indices_cnt = polygon->v_indices_cnt;
	s2.stride = 1;
	p_result = SegmentIndices_Sweep_SegmentIndices(&s1, dir, &s2, result);
	mathVec3Negate(neg_dir, dir);
	for (i = 0; i < polygon->v_indices_cnt; ++i) {
		CCTSweepResult_t result_temp;
		pp = polygon->v[polygon->v_indices[i]];
		if (!Ray_Sweep_ConvexMesh(pp, neg_dir, mesh, 0, &result_temp)) {
			continue;
		}
		if (!p_result) {
			p_result = result;
			copy_result(p_result, &result_temp);
		}
		else {
			merge_result(p_result, &result_temp);
		}
	}
	return p_result;
}

static CCTSweepResult_t* ConvexMesh_Sweep_ConvexMesh(const GeometryMesh_t* mesh1, const CCTNum_t dir[3], const GeometryMesh_t* mesh2, CCTSweepResult_t* result) {
	unsigned int i;
	CCTNum_t neg_dir[3];
	CCTSweepResult_t* p_result;
	GeometrySegmentIndices_t s1, s2;

	if (ConvexMesh_Intersect_ConvexMesh(mesh1, mesh2)) {
		return set_result(result, CCTNum(0.0), dir);
	}
	s1.v = mesh1->v;
	s1.indices = mesh1->edge_indices;
	s1.indices_cnt = mesh1->edge_indices_cnt;
	s1.stride = 2;
	s2.v = mesh2->v;
	s2.indices = mesh2->edge_indices;
	s2.indices_cnt = mesh2->edge_indices_cnt;
	s2.stride = 2;
	p_result = SegmentIndices_Sweep_SegmentIndices(&s1, dir, &s2, result);
	for (i = 0; i < mesh1->v_indices_cnt; ++i) {
		CCTSweepResult_t result_temp;
		const CCTNum_t* pp = mesh1->v[mesh1->v_indices[i]];
		if (!Ray_Sweep_ConvexMesh(pp, dir, mesh2, 0, &result_temp)) {
			continue;
		}
		if (!p_result) {
			p_result = result;
			copy_result(p_result, &result_temp);
		}
		else {
			merge_result(p_result, &result_temp);
		}
	}
	mathVec3Negate(neg_dir, dir);
	for (i = 0; i < mesh2->v_indices_cnt; ++i) {
		CCTSweepResult_t result_temp;
		const CCTNum_t* pp = mesh2->v[mesh2->v_indices[i]];
		if (!Ray_Sweep_ConvexMesh(pp, neg_dir, mesh1, 0, &result_temp)) {
			continue;
		}
		if (!p_result) {
			p_result = result;
			copy_result(p_result, &result_temp);
		}
		else {
			merge_result(p_result, &result_temp);
		}
	}
	return p_result;
}

static CCTSweepResult_t* OBB_Sweep_OBB(const GeometryOBB_t* obb1, const CCTNum_t dir[3], const GeometryOBB_t* obb2, CCTSweepResult_t* result) {
	int i;
	CCTNum_t v1[8][3], v2[8][3], neg_dir[3];
	GeometrySegmentIndices_t s1, s2;
	CCTSweepResult_t* p_result;
	if (OBB_Intersect_OBB(obb1, obb2)) {
		set_result(result, CCTNum(0.0), dir);
		return result;
	}
	mathOBBVertices(obb1, v1);
	mathOBBVertices(obb2, v2);
	s1.v = v1;
	s1.indices = Box_Edge_Indices;
	s1.indices_cnt = sizeof(Box_Edge_Indices) / sizeof(Box_Edge_Indices[0]);
	s1.stride = 2;
	s2.v = v2;
	s2.indices = Box_Edge_Indices;
	s2.indices_cnt = sizeof(Box_Edge_Indices) / sizeof(Box_Edge_Indices[0]);
	s2.stride = 2;
	p_result = SegmentIndices_Sweep_SegmentIndices(&s1, dir, &s2, result);
	for (i = 0; i < 8; ++i) {
		CCTSweepResult_t result_temp;
		if (!Ray_Sweep_OBB(v1[i], dir, obb2, 0, &result_temp)) {
			continue;
		}
		if (!p_result) {
			p_result = result;
			copy_result(p_result, &result_temp);
		}
		else {
			merge_result(p_result, &result_temp);
		}
	}
	mathVec3Negate(neg_dir, dir);
	for (i = 0; i < 8; ++i) {
		CCTSweepResult_t result_temp;
		if (!Ray_Sweep_OBB(v2[i], neg_dir, obb1, 0, &result_temp)) {
			continue;
		}
		if (!p_result) {
			p_result = result;
			copy_result(p_result, &result_temp);
		}
		else {
			merge_result(p_result, &result_temp);
		}
	}
	return p_result;
}

static CCTSweepResult_t* Sphere_Sweep_Plane(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t dir[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTSweepResult_t* result) {
	CCTNum_t dn, dn_abs, cos_theta, hit_point[3];
	mathPointProjectionPlane(o, plane_v, plane_n, NULL, &dn);
	dn_abs = CCTNum_abs(dn);
	if (dn_abs < radius - CCT_EPSILON) {
		set_result(result, CCTNum(0.0), dir);
		return result;
	}
	if (dn_abs <= radius) {
		set_result(result, CCTNum(0.0), dir);
		set_unique_hit_point(result, o);
		mathVec3AddScalar(result->unique_hit_point, plane_n, dn);
		return result;
	}
	cos_theta = mathVec3Dot(plane_n, dir);
	if (cos_theta <= CCT_EPSILON && cos_theta >= CCT_EPSILON_NEGATE) {
		return NULL;
	}
	mathVec3Copy(hit_point, o);
	if (dn >= CCTNum(0.0)) {
		dn -= radius;
		mathVec3AddScalar(hit_point, plane_n, radius);
	}
	else {
		dn += radius;
		mathVec3SubScalar(hit_point, plane_n, radius);
	}
	dn /= cos_theta;
	if (dn < CCTNum(0.0)) {
		return NULL;
	}
	mathVec3AddScalar(hit_point, dir, dn);
	set_result(result, dn, plane_n);
	set_unique_hit_point(result, hit_point);
	return result;
}

static CCTSweepResult_t* Sphere_Sweep_Polygon(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t dir[3], const GeometryPolygon_t* polygon, CCTSweepResult_t* result) {
	int i;
	CCTSweepResult_t *p_result = NULL;
	CCTNum_t neg_dir[3];

	if (!Sphere_Sweep_Plane(o, radius, dir, polygon->v[polygon->v_indices[0]], polygon->normal, result)) {
		return NULL;
	}
	if (result->distance > CCTNum(0.0)) {
		if (Polygon_Contain_Point(polygon, result->unique_hit_point)) {
			return result;
		}
	}
	else {
		CCTNum_t p[3];
		mathPointProjectionPlane(o, polygon->v[polygon->v_indices[0]], polygon->normal, p, NULL);
		if (Polygon_Contain_Point(polygon, p)) {
			return set_result(result, CCTNum(0.0), dir);
		}
	}
	p_result = NULL;
	mathVec3Negate(neg_dir, dir);
	for (i = 0; i < polygon->v_indices_cnt; ) {
		CCTSweepResult_t result_temp;
		CCTNum_t edge[2][3];
		mathVec3Copy(edge[0], polygon->v[polygon->v_indices[i++]]);
		mathVec3Copy(edge[1], polygon->v[polygon->v_indices[i >= polygon->v_indices_cnt ? 0 : i]]);
		if (!Segment_Sweep_Sphere((const CCTNum_t(*)[3])edge, neg_dir, o, radius, 1, &result_temp)) {
			continue;
		}
		if (result_temp.distance <= CCTNum(0.0)) {
			return set_result(result, CCTNum(0.0), dir);
		}
		if (!p_result) {
			p_result = result;
			copy_result(p_result, &result_temp);
		}
		else {
			merge_result(p_result, &result_temp);
		}
	}
	if (p_result) {
		neg_dir_hit_points(p_result, dir);
	}
	return p_result;
}

static CCTSweepResult_t* Sphere_Sweep_Sphere(const CCTNum_t o1[3], CCTNum_t r1, const CCTNum_t dir[3], const CCTNum_t o2[3], CCTNum_t r2, CCTSweepResult_t* result) {
	CCTNum_t hit_point[3];
	if (!Ray_Sweep_Sphere(o1, dir, o2, r1 + r2, result)) {
		return NULL;
	}
	mathVec3Copy(hit_point, o1);
	mathVec3AddScalar(hit_point, dir, result->distance);
	mathVec3Sub(result->hit_normal, hit_point, o2);
	mathVec3Normalized(result->hit_normal, result->hit_normal);
	set_unique_hit_point(result, hit_point);
	mathVec3SubScalar(result->unique_hit_point, result->hit_normal, r1);
	return result;
}

static CCTSweepResult_t* Sphere_Sweep_OBB(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t dir[3], const GeometryOBB_t* obb, CCTSweepResult_t* result) {
	if (Sphere_Intersect_OBB(o, radius, obb)) {
		set_result(result, CCTNum(0.0), dir);
		return result;
	}
	else {
		int i;
		CCTSweepResult_t* p_result = NULL;
		CCTNum_t v[8][3], neg_dir[3];

		mathOBBVertices(obb, v);
		for (i = 0; i < 6; ++i) {
			CCTSweepResult_t result_temp;
			GeometryPolygon_t polygon;
			mathBoxFace((const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])obb->axis, i, &polygon);
			if (!Sphere_Sweep_Plane(o, radius, dir, polygon.v[polygon.v_indices[0]], polygon.normal, &result_temp)) {
				continue;
			}
			if (result_temp.distance <= CCTNum(0.0)) {
				continue;
			}
			if (!Polygon_Contain_Point(&polygon, result_temp.unique_hit_point)) {
				continue;
			}
			if (!p_result) {
				p_result = result;
				copy_result(p_result, &result_temp);
			}
			else {
				merge_result(p_result, &result_temp);
			}
		}
		mathVec3Negate(neg_dir, dir);
		for (i = 0; i < sizeof(Box_Edge_Indices) / sizeof(Box_Edge_Indices[0]); i += 2) {
			CCTNum_t edge[2][3];
			CCTSweepResult_t result_temp;
			mathVec3Copy(edge[0], v[Box_Edge_Indices[i]]);
			mathVec3Copy(edge[1], v[Box_Edge_Indices[i+1]]);
			if (!Segment_Sweep_Sphere((const CCTNum_t(*)[3])edge, neg_dir, o, radius, 0, &result_temp)) {
				continue;
			}
			if (!p_result) {
				p_result = result;
				copy_result(p_result, &result_temp);
			}
			else {
				merge_result(p_result, &result_temp);
			}
		}
		return p_result;
	}
}

static CCTSweepResult_t* Sphere_Sweep_ConvexMesh(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t dir[3], const GeometryMesh_t* mesh, CCTSweepResult_t* result) {
	unsigned int i;
	CCTSweepResult_t* p_result;
	if (ConvexMesh_Contain_Point(mesh, o)) {
		set_result(result, CCTNum(0.0), dir);
		return result;
	}
	p_result = NULL;
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		CCTSweepResult_t result_temp;
		if (!Sphere_Sweep_Polygon(o, radius, dir, mesh->polygons + i, &result_temp)) {
			continue;
		}
		if (result_temp.distance <= CCTNum(0.0)) {
			return set_result(result, CCTNum(0.0), dir);
		}
		if (!p_result) {
			p_result = result;
			copy_result(p_result, &result_temp);
		}
		else {
			merge_result(p_result, &result_temp);
		}
	}
	return p_result;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
extern "C" {
#endif

CCTSweepResult_t* mathGeometrySweep(const GeometryBodyRef_t* one, const CCTNum_t dir[3], const GeometryBodyRef_t* two, CCTSweepResult_t* result) {
	int flag_neg_dir;
	if (one->data == two->data || mathVec3IsZero(dir)) {
		return NULL;
	}
	flag_neg_dir = 0;
	if (GEOMETRY_BODY_POINT == one->type) {
		switch (two->type) {
			case GEOMETRY_BODY_AABB:
			{
				GeometryOBB_t obb2;
				mathOBBFromAABB(&obb2, two->aabb->o, two->aabb->half);
				result = Ray_Sweep_OBB(one->point, dir, &obb2, 1, result);
				break;
			}
			case GEOMETRY_BODY_OBB:
			{
				result = Ray_Sweep_OBB(one->point, dir, two->obb, 1, result);
				break;
			}
			case GEOMETRY_BODY_SPHERE:
			{
				result = Ray_Sweep_Sphere(one->point, dir, two->sphere->o, two->sphere->radius, result);
				break;
			}
			case GEOMETRY_BODY_PLANE:
			{
				result = Ray_Sweep_Plane(one->point, dir, two->plane->v, two->plane->normal, result);
				break;
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				result = Ray_Sweep_Segment(one->point, dir, (const CCTNum_t(*)[3])two->segment->v, result);
				break;
			}
			case GEOMETRY_BODY_POLYGON:
			{
				result = Ray_Sweep_Polygon(one->point, dir, two->polygon, result);
				break;
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				result = Ray_Sweep_ConvexMesh(one->point, dir, two->mesh, 1, result);
				break;
			}
		}
	}
	else if (GEOMETRY_BODY_SEGMENT == one->type) {
		const CCTNum_t(*one_segment_v)[3] = (const CCTNum_t(*)[3])one->segment->v;
		switch (two->type) {
			case GEOMETRY_BODY_SEGMENT:
			{
				result = Segment_Sweep_Segment(one_segment_v, dir, (const CCTNum_t(*)[3])two->segment->v, result);
				break;
			}
			case GEOMETRY_BODY_PLANE:
			{
				result = Segment_Sweep_Plane(one_segment_v, dir, two->plane->v, two->plane->normal, result);
				break;
			}
			case GEOMETRY_BODY_OBB:
			{
				result = Segment_Sweep_OBB(one_segment_v, dir, two->obb, 1, result);
				break;
			}
			case GEOMETRY_BODY_AABB:
			{
				GeometryOBB_t obb2;
				mathOBBFromAABB(&obb2, two->aabb->o, two->aabb->half);
				result = Segment_Sweep_OBB(one_segment_v, dir, &obb2, 1, result);
				break;
			}
			case GEOMETRY_BODY_SPHERE:
			{
				result = Segment_Sweep_Sphere(one_segment_v, dir, two->sphere->o, two->sphere->radius, 1, result);
				break;
			}
			case GEOMETRY_BODY_POLYGON:
			{
				result = Segment_Sweep_Polygon(one_segment_v, dir, two->polygon, result);
				break;
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				result = Segment_Sweep_ConvexMesh(one_segment_v, dir, two->mesh, result);
				break;
			}
		}
	}
	else if (GEOMETRY_BODY_AABB == one->type) {
		switch (two->type) {
			case GEOMETRY_BODY_AABB:
			{
				result = AABB_Sweep_AABB(one->aabb->o, one->aabb->half, dir, two->aabb->o, two->aabb->half, result);
				break;
			}
			case GEOMETRY_BODY_OBB:
			{
				GeometryOBB_t obb1;
				mathOBBFromAABB(&obb1, one->aabb->o, one->aabb->half);
				result = OBB_Sweep_OBB(&obb1, dir, two->obb, result);
				break;
			}
			case GEOMETRY_BODY_SPHERE:
			{
				GeometryOBB_t obb1;
				CCTNum_t neg_dir[3];
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				mathOBBFromAABB(&obb1, one->aabb->o, one->aabb->half);
				result = Sphere_Sweep_OBB(two->sphere->o, two->sphere->radius, neg_dir, &obb1, result);
				break;
			}
			case GEOMETRY_BODY_PLANE:
			{
				CCTNum_t v[8][3];
				mathAABBVertices(one->aabb->o, one->aabb->half, v);
				result = Vertices_Sweep_Plane((const CCTNum_t(*)[3])v, Box_Vertice_Indices_Default, 8, dir, two->plane->v, two->plane->normal, result);
				break;
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				GeometryOBB_t obb1;
				CCTNum_t neg_dir[3];
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				mathOBBFromAABB(&obb1, one->aabb->o, one->aabb->half);
				result = Segment_Sweep_OBB((const CCTNum_t(*)[3])two->segment->v, neg_dir, &obb1, 1, result);
				break;
			}
			case GEOMETRY_BODY_POLYGON:
			{
				GeometryBoxMesh_t one_mesh;
				CCTNum_t v[8][3];
				mathAABBVertices(one->aabb->o, one->aabb->half, v);
				mathBoxMesh((const CCTNum_t(*)[3])v, AABB_Axis, &one_mesh);
				result = ConvexMesh_Sweep_Polygon(&one_mesh.mesh, dir, two->polygon, result);
				break;
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				GeometryBoxMesh_t one_mesh;
				CCTNum_t v[8][3];
				mathAABBVertices(one->aabb->o, one->aabb->half, v);
				mathBoxMesh((const CCTNum_t(*)[3])v, AABB_Axis, &one_mesh);
				result = ConvexMesh_Sweep_ConvexMesh(&one_mesh.mesh, dir, two->mesh, result);
				break;
			}
		}
	}
	else if (GEOMETRY_BODY_OBB == one->type) {
		switch (two->type) {
			case GEOMETRY_BODY_SEGMENT:
			{
				CCTNum_t neg_dir[3];
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = Segment_Sweep_OBB((const CCTNum_t(*)[3])two->segment->v, neg_dir, one->obb, 1, result);
				break;
			}
			case GEOMETRY_BODY_PLANE:
			{
				CCTNum_t v[8][3];
				mathOBBVertices(one->obb, v);
				result = Vertices_Sweep_Plane((const CCTNum_t(*)[3])v, Box_Vertice_Indices_Default, 8, dir, two->plane->v, two->plane->normal, result);
				break;
			}
			case GEOMETRY_BODY_SPHERE:
			{
				CCTNum_t neg_dir[3];
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = Sphere_Sweep_OBB(two->sphere->o, two->sphere->radius, neg_dir, one->obb, result);
				break;
			}
			case GEOMETRY_BODY_OBB:
			{
				result = OBB_Sweep_OBB(one->obb, dir, two->obb, result);
				break;
			}
			case GEOMETRY_BODY_AABB:
			{
				GeometryOBB_t obb2;
				mathOBBFromAABB(&obb2, two->aabb->o, two->aabb->half);
				result = OBB_Sweep_OBB(one->obb, dir, &obb2, result);
				break;
			}
			case GEOMETRY_BODY_POLYGON:
			{
				GeometryBoxMesh_t one_mesh;
				CCTNum_t v[8][3];
				mathOBBVertices(one->obb, v);
				mathBoxMesh((const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])one->obb->axis, &one_mesh);
				result = ConvexMesh_Sweep_Polygon(&one_mesh.mesh, dir, two->polygon, result);
				break;
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				GeometryBoxMesh_t one_mesh;
				CCTNum_t v[8][3];
				mathOBBVertices(one->obb, v);
				mathBoxMesh((const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])one->obb->axis, &one_mesh);
				result = ConvexMesh_Sweep_ConvexMesh(&one_mesh.mesh, dir, two->mesh, result);
				break;
			}
		}
	}
	else if (GEOMETRY_BODY_SPHERE == one->type) {
		switch (two->type) {
			case GEOMETRY_BODY_OBB:
			{
				result = Sphere_Sweep_OBB(one->sphere->o, one->sphere->radius, dir, two->obb, result);
				break;
			}
			case GEOMETRY_BODY_AABB:
			{
				GeometryOBB_t obb2;
				mathOBBFromAABB(&obb2, two->aabb->o, two->aabb->half);
				result = Sphere_Sweep_OBB(one->sphere->o, one->sphere->radius, dir, &obb2, result);
				break;
			}
			case GEOMETRY_BODY_SPHERE:
			{
				result = Sphere_Sweep_Sphere(one->sphere->o, one->sphere->radius, dir, two->sphere->o, two->sphere->radius, result);
				break;
			}
			case GEOMETRY_BODY_PLANE:
			{
				result = Sphere_Sweep_Plane(one->sphere->o, one->sphere->radius, dir, two->plane->v, two->plane->normal, result);
				break;
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				CCTNum_t neg_dir[3];
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = Segment_Sweep_Sphere((const CCTNum_t(*)[3])two->segment->v, neg_dir, one->sphere->o, one->sphere->radius, 1, result);
				break;
			}
			case GEOMETRY_BODY_POLYGON:
			{
				result = Sphere_Sweep_Polygon(one->sphere->o, one->sphere->radius, dir, two->polygon, result);
				break;
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				result = Sphere_Sweep_ConvexMesh(one->sphere->o, one->sphere->radius, dir, two->mesh, result);
				break;
			}
		}
	}
	else if (GEOMETRY_BODY_POLYGON == one->type) {
		switch (two->type) {
			case GEOMETRY_BODY_SEGMENT:
			{
				CCTNum_t neg_dir[3];
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = Segment_Sweep_Polygon((const CCTNum_t(*)[3])two->segment->v, neg_dir, one->polygon, result);
				break;
			}
			case GEOMETRY_BODY_PLANE:
			{
				const GeometryPolygon_t* polygon = one->polygon;
				const GeometryPlane_t* plane = two->plane;
				result = Vertices_Sweep_Plane((const CCTNum_t(*)[3])polygon->v, polygon->v_indices, polygon->v_indices_cnt, dir, plane->v, plane->normal, result);
				break;
			}
			case GEOMETRY_BODY_SPHERE:
			{
				CCTNum_t neg_dir[3];
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = Sphere_Sweep_Polygon(two->sphere->o, two->sphere->radius, neg_dir, one->polygon, result);
				break;
			}
			case GEOMETRY_BODY_OBB:
			{
				GeometryBoxMesh_t two_mesh;
				CCTNum_t v[8][3], neg_dir[3];
				mathOBBVertices(two->obb, v);
				mathBoxMesh((const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])two->obb->axis, &two_mesh);
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = ConvexMesh_Sweep_Polygon(&two_mesh.mesh, neg_dir, one->polygon, result);
				break;
			}
			case GEOMETRY_BODY_AABB:
			{
				GeometryBoxMesh_t two_mesh;
				CCTNum_t v[8][3], neg_dir[3];
				mathAABBVertices(two->aabb->o, two->aabb->half, v);
				mathBoxMesh((const CCTNum_t(*)[3])v, AABB_Axis, &two_mesh);
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = ConvexMesh_Sweep_Polygon(&two_mesh.mesh, neg_dir, one->polygon, result);
				break;
			}
			case GEOMETRY_BODY_POLYGON:
			{
				result = Polygon_Sweep_Polygon(one->polygon, dir, two->polygon, result);
				break;
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				CCTNum_t neg_dir[3];
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = ConvexMesh_Sweep_Polygon(two->mesh, neg_dir, one->polygon, result);
				break;
			}
		}
	}
	else if (GEOMETRY_BODY_CONVEX_MESH == one->type) {
		switch (two->type) {
			case GEOMETRY_BODY_SEGMENT:
			{
				CCTNum_t neg_dir[3];
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = Segment_Sweep_ConvexMesh((const CCTNum_t(*)[3])two->segment->v, neg_dir, one->mesh, result);
				break;
			}
			case GEOMETRY_BODY_PLANE:
			{
				const GeometryMesh_t* mesh = one->mesh;
				const GeometryPlane_t* plane = two->plane;
				result = Vertices_Sweep_Plane((const CCTNum_t(*)[3])mesh->v, mesh->v_indices, mesh->v_indices_cnt, dir, plane->v, plane->normal, result);
				break;
			}
			case GEOMETRY_BODY_SPHERE:
			{
				CCTNum_t neg_dir[3];
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = Sphere_Sweep_ConvexMesh(two->sphere->o, two->sphere->radius, neg_dir, one->mesh, result);
				break;
			}
			case GEOMETRY_BODY_OBB:
			{
				GeometryBoxMesh_t two_mesh;
				CCTNum_t v[8][3];
				mathOBBVertices(two->obb, v);
				mathBoxMesh((const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])two->obb->axis, &two_mesh);
				result = ConvexMesh_Sweep_ConvexMesh(one->mesh, dir, &two_mesh.mesh, result);
				break;
			}
			case GEOMETRY_BODY_AABB:
			{
				GeometryBoxMesh_t two_mesh;
				CCTNum_t v[8][3];
				mathAABBVertices(two->aabb->o, two->aabb->half, v);
				mathBoxMesh((const CCTNum_t(*)[3])v, AABB_Axis, &two_mesh);
				result = ConvexMesh_Sweep_ConvexMesh(one->mesh, dir, &two_mesh.mesh, result);
				break;
			}
			case GEOMETRY_BODY_POLYGON:
			{
				result = ConvexMesh_Sweep_Polygon(one->mesh, dir, two->polygon, result);
				break;
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				result = ConvexMesh_Sweep_ConvexMesh(one->mesh, dir, two->mesh, result);
				break;
			}
		}
	}
	if (!result) {
		return NULL;
	}
	if (flag_neg_dir) {
		neg_dir_hit_points(result, dir);
	}
	return result;
}

#ifdef __cplusplus
}
#endif
