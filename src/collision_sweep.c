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
#include "../inc/collision.h"
#include <math.h>
#include <stddef.h>

extern const CCTNum_t AABB_Plane_Normal[6][3];
extern const unsigned int Box_Edge_Indices[24];

extern int Segment_Contain_Point(const CCTNum_t ls[2][3], const CCTNum_t p[3]);
extern int Segment_Intersect_Segment(const CCTNum_t ls1[2][3], const CCTNum_t ls2[2][3], CCTNum_t p[3], int* line_mask);
extern int Segment_Intersect_Plane(const CCTNum_t ls[2][3], const CCTNum_t plane_v[3], const CCTNum_t plane_normal[3], CCTNum_t p[3]);
extern int Sphere_Intersect_Segment(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t ls[2][3], CCTNum_t p[3]);
extern int Sphere_Intersect_Plane(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t plane_v[3], const CCTNum_t plane_normal[3], CCTNum_t new_o[3], CCTNum_t* new_r);
extern int OBB_Contain_Point(const GeometryOBB_t* obb, const CCTNum_t p[3]);
extern int OBB_Intersect_Segment(const GeometryOBB_t* obb, const CCTNum_t ls[2][3]);
extern int OBB_Intersect_OBB(const GeometryOBB_t* obb0, const GeometryOBB_t* obb1);
extern int Sphere_Intersect_OBB(const CCTNum_t o[3], CCTNum_t radius, const GeometryOBB_t* obb);
extern int Plane_Intersect_Plane(const CCTNum_t v1[3], const CCTNum_t n1[3], const CCTNum_t v2[3], const CCTNum_t n2[3]);
extern int ConvexMesh_Contain_Point(const GeometryMesh_t* mesh, const CCTNum_t p[3]);

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

static CCTResult_t* copy_result(CCTResult_t* dst, CCTResult_t* src) {
	if (dst == src) {
		return dst;
	}
	dst->distance = src->distance;
	dst->has_unique_hit_point = src->has_unique_hit_point;
	if (src->has_unique_hit_point) {
		mathVec3Copy(dst->unique_hit_point, src->unique_hit_point);
	}
	mathVec3Copy(dst->hit_normal, src->hit_normal);
	return dst;
}

static CCTResult_t* set_result(CCTResult_t* result, CCTNum_t distance, const CCTNum_t hit_normal[3]) {
	result->distance = distance;
	result->has_unique_hit_point = 0;
	if (hit_normal) {
		mathVec3Copy(result->hit_normal, hit_normal);
	}
	else {
		mathVec3Set(result->hit_normal, CCTNums_3(0.0, 0.0, 0.0));
	}
	return result;
}

static CCTResult_t* add_result_hit_point(CCTResult_t* result, const CCTNum_t p[3]) {
	mathVec3Copy(result->unique_hit_point, p);
	result->has_unique_hit_point = 1;
	return result;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

static CCTResult_t* Ray_Sweep_Segment(const CCTNum_t o[3], const CCTNum_t dir[3], const CCTNum_t ls[2][3], CCTResult_t* result) {
	CCTNum_t v0[3], v1[3], N[3], dot, d;
	mathVec3Sub(v0, ls[0], o);
	mathVec3Sub(v1, ls[1], o);
	mathVec3Cross(N, v0, v1);
	if (mathVec3IsZero(N)) {
		dot = mathVec3Dot(v0, v1);
		if (dot <= CCTNum(0.0)) {
			set_result(result, CCTNum(0.0), dir);
			add_result_hit_point(result, o);
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
		if (mathVec3LenSq(v0) < mathVec3LenSq(v1)) {
			d = mathVec3Dot(v0, dir);
			set_result(result, d, dir);
			add_result_hit_point(result, v0);
		}
		else {
			d = mathVec3Dot(v1, dir);
			set_result(result, d, dir);
			add_result_hit_point(result, v1);
		}
		return result;
	}
	else {
		CCTNum_t lsdir[3], p[3], op[3];
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
			add_result_hit_point(result, o);
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
		add_result_hit_point(result, p);
		return result;
	}
}

static CCTResult_t* Ray_Sweep_Plane(const CCTNum_t o[3], const CCTNum_t dir[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTResult_t* result) {
	CCTNum_t d, cos_theta;
	mathPointProjectionPlane(o, plane_v, plane_n, NULL, &d);
	if (d <= CCT_EPSILON && d >= CCT_EPSILON_NEGATE) {
		set_result(result, CCTNum(0.0), dir);
		add_result_hit_point(result, o);
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
	add_result_hit_point(result, o);
	mathVec3AddScalar(result->unique_hit_point, dir, d);
	return result;
}

static CCTResult_t* Ray_Sweep_Polygon(const CCTNum_t o[3], const CCTNum_t dir[3], const GeometryPolygon_t* polygon, CCTResult_t* result) {
	CCTResult_t* p_result;
	int i;
	CCTNum_t dot;
	if (!Ray_Sweep_Plane(o, dir, polygon->v[polygon->v_indices[0]], polygon->normal, result)) {
		return NULL;
	}
	if (result->distance > CCTNum(0.0)) {
		if (Polygon_Contain_Point(polygon, result->unique_hit_point)) {
			return result;
		}
		return NULL;
	}
	dot = mathVec3Dot(dir, polygon->normal);
	if (dot < CCT_EPSILON_NEGATE || dot > CCT_EPSILON) {
		return NULL;
	}
	p_result = NULL;
	for (i = 0; i < polygon->v_indices_cnt; ) {
		CCTResult_t result_temp;
		CCTNum_t edge[2][3];
		mathVec3Copy(edge[0], polygon->v[polygon->v_indices[i++]]);
		mathVec3Copy(edge[1], polygon->v[polygon->v_indices[i >= polygon->v_indices_cnt ? 0 : i]]);
		if (!Ray_Sweep_Segment(o, dir, (const CCTNum_t(*)[3])edge, &result_temp)) {
			continue;
		}
		if (!p_result || p_result->distance > result_temp.distance) {
			p_result = result;
			copy_result(result, &result_temp);
		}
	}
	return p_result;
}

static CCTResult_t* Ray_Sweep_OBB(const CCTNum_t o[3], const CCTNum_t dir[3], const GeometryOBB_t* obb, CCTResult_t* result) {
	if (OBB_Contain_Point(obb, o)) {
		set_result(result, CCTNum(0.0), dir);
		add_result_hit_point(result, o);
		return result;
	}
	else {
		CCTResult_t *p_result = NULL;
		int i;
		for (i = 0; i < 6; ++i) {
			CCTResult_t result_temp;
			GeometryRect_t rect;
			mathOBBPlaneRect(obb, i, &rect);
			if (!Ray_Sweep_Plane(o, dir, rect.o, rect.normal, &result_temp)) {
				continue;
			}
			if (!mathRectHasPoint(&rect, result_temp.unique_hit_point)) {
				continue;
			}
			if (!p_result || p_result->distance > result_temp.distance) {
				copy_result(result, &result_temp);
				p_result = result;
			}
		}
		return p_result;
	}
}

static CCTResult_t* Ray_Sweep_Sphere(const CCTNum_t o[3], const CCTNum_t dir[3], const CCTNum_t sp_o[3], CCTNum_t sp_radius, CCTResult_t* result) {
	CCTNum_t dr2, oc2, dir_d;
	CCTNum_t oc[3];
	CCTNum_t radius2 = sp_radius * sp_radius;
	mathVec3Sub(oc, sp_o, o);
	oc2 = mathVec3LenSq(oc);
	if (oc2 <= radius2) {
		set_result(result, CCTNum(0.0), dir);
		add_result_hit_point(result, o);
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
	set_result(result, dir_d, NULL);
	add_result_hit_point(result, o);
	mathVec3AddScalar(result->unique_hit_point, dir, dir_d);
	mathVec3Sub(result->hit_normal, result->unique_hit_point, sp_o);
	mathVec3MultiplyScalar(result->hit_normal, result->hit_normal, CCTNum(1.0) / sp_radius);
	return result;
}

static CCTResult_t* Ray_Sweep_ConvexMesh(const CCTNum_t o[3], const CCTNum_t dir[3], const GeometryMesh_t* mesh, CCTResult_t* result) {
	unsigned int i;
	CCTResult_t* p_result = NULL;
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		CCTResult_t result_temp;
		if (!Ray_Sweep_Polygon(o, dir, mesh->polygons + i, &result_temp)) {
			continue;
		}
		if (!p_result || p_result->distance > result_temp.distance) {
			if (CCT_EPSILON_NEGATE <= result_temp.distance && result_temp.distance <= CCT_EPSILON) {
				set_result(result, CCTNum(0.0), dir);
				add_result_hit_point(result, o);
				return result;
			}
			copy_result(result, &result_temp);
			p_result = result;
		}
	}
	if (p_result && AABB_Contain_Point(mesh->bound_box.o, mesh->bound_box.half, o)) {
		CCTNum_t neg_dir[3];
		mathVec3Negate(neg_dir, dir);
		for (i = 0; i < mesh->polygons_cnt; ++i) {
			CCTResult_t result_temp;
			if (Ray_Sweep_Polygon(o, neg_dir, mesh->polygons + i, &result_temp)) {
				set_result(result, CCTNum(0.0), dir);
				add_result_hit_point(result, o);
				return result;
			}
		}
	}
	return p_result;
}

static CCTResult_t* Segment_Sweep_Plane(const CCTNum_t ls[2][3], const CCTNum_t dir[3], const CCTNum_t vertice[3], const CCTNum_t normal[3], CCTResult_t* result) {
	CCTNum_t p[3];
	int res = Segment_Intersect_Plane(ls, vertice, normal, p);
	if (2 == res) {
		set_result(result, CCTNum(0.0), dir);
		return result;
	}
	else if (1 == res) {
		set_result(result, CCTNum(0.0), normal);
		add_result_hit_point(result, p);
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
			add_result_hit_point(result, p);
			mathVec3AddScalar(result->unique_hit_point, dir, min_d);
		}
		return result;
	}
}

static CCTResult_t* Segment_Sweep_Segment(const CCTNum_t ls1[2][3], const CCTNum_t dir[3], const CCTNum_t ls2[2][3], CCTResult_t* result) {
	int line_mask;
	CCTNum_t p[3];
	int res = Segment_Intersect_Segment(ls1, ls2, p, &line_mask);
	if (GEOMETRY_SEGMENT_CONTACT == res) {
		set_result(result, CCTNum(0.0), dir);
		add_result_hit_point(result, p);
		return result;
	}
	else if (GEOMETRY_SEGMENT_OVERLAP == res) {
		set_result(result, CCTNum(0.0), dir);
		return result;
	}
	else if (GEOMETRY_LINE_PARALLEL == line_mask) {
		CCTResult_t* p_result = NULL;
		CCTNum_t neg_dir[3];
		int i;
		for (i = 0; i < 2; ++i) {
			CCTResult_t result_temp;
			if (!Ray_Sweep_Segment(ls1[i], dir, ls2, &result_temp)) {
				continue;
			}
			if (!p_result || p_result->distance > result_temp.distance) {
				p_result = result;
				copy_result(p_result, &result_temp);
			}
		}
		if (p_result) {
			p_result->has_unique_hit_point = 0;
			return p_result;
		}
		mathVec3Negate(neg_dir, dir);
		for (i = 0; i < 2; ++i) {
			CCTResult_t result_temp;
			if (!Ray_Sweep_Segment(ls2[i], neg_dir, ls1, &result_temp)) {
				continue;
			}
			if (!p_result || p_result->distance > result_temp.distance) {
				p_result = result;
				copy_result(p_result, &result_temp);
			}
		}
		if (p_result) {
			p_result->has_unique_hit_point = 0;
		}
		return p_result;
	}
	else if (GEOMETRY_LINE_CROSS == line_mask) {
		CCTResult_t* p_result = NULL;
		CCTNum_t neg_dir[3];
		int i;
		for (i = 0; i < 2; ++i) {
			CCTResult_t result_temp;
			if (!Ray_Sweep_Segment(ls1[i], dir, ls2, &result_temp)) {
				continue;
			}
			if (!p_result || p_result->distance > result_temp.distance) {
				p_result = result;
				copy_result(p_result, &result_temp);
			}
		}
		mathVec3Negate(neg_dir, dir);
		for (i = 0; i < 2; ++i) {
			CCTResult_t result_temp;
			if (!Ray_Sweep_Segment(ls2[i], neg_dir, ls1, &result_temp)) {
				continue;
			}
			if (!p_result || p_result->distance > result_temp.distance) {
				p_result = result;
				copy_result(p_result, &result_temp);
				mathVec3Copy(p_result->unique_hit_point, ls2[i]);
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
		add_result_hit_point(result, ls2[closest_ls2_indice]);
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
		mathVec3Copy(result->unique_hit_point, v);
		return result;
	}
}

static CCTResult_t* Segment_Sweep_OBB(const CCTNum_t ls[2][3], const CCTNum_t dir[3], const GeometryOBB_t* obb, CCTResult_t* result) {
	if (OBB_Intersect_Segment(obb, ls)) {
		set_result(result, CCTNum(0.0), dir);
		return result;
	}
	else {
		CCTResult_t* p_result = NULL;
		int i;
		CCTNum_t v[8][3];
		mathOBBVertices(obb, v);
		for (i = 0; i < sizeof(Box_Edge_Indices) / sizeof(Box_Edge_Indices[0]); i += 2) {
			CCTNum_t edge[2][3];
			CCTResult_t result_temp;
			mathVec3Copy(edge[0], v[Box_Edge_Indices[i]]);
			mathVec3Copy(edge[1], v[Box_Edge_Indices[i+1]]);
			if (!Segment_Sweep_Segment(ls, dir, (const CCTNum_t(*)[3])edge, &result_temp)) {
				continue;
			}
			if (!p_result || p_result->distance > result_temp.distance) {
				p_result = result;
				copy_result(p_result, &result_temp);
			}
		}
		for (i = 0; i < 2; ++i) {
			CCTResult_t result_temp;
			if (!Ray_Sweep_OBB(ls[i], dir, obb, &result_temp)) {
				continue;
			}
			if (!p_result || p_result->distance > result_temp.distance) {
				p_result = result;
				copy_result(p_result, &result_temp);
			}
		}
		return p_result;
	}
}

static CCTResult_t* Segment_Sweep_Polygon(const CCTNum_t ls[2][3], const CCTNum_t dir[3], const GeometryPolygon_t* polygon, CCTResult_t* result) {
	CCTResult_t *p_result;
	int i;
	if (!Segment_Sweep_Plane(ls, dir, polygon->v[polygon->v_indices[0]], polygon->normal, result)) {
		return NULL;
	}
	if (result->has_unique_hit_point) {
		if (Polygon_Contain_Point(polygon, result->unique_hit_point)) {
			return result;
		}
	}
	else {
		for (i = 0; i < 2; ++i) {
			CCTNum_t test_p[3];
			mathVec3Copy(test_p, ls[i]);
			mathVec3AddScalar(test_p, dir, result->distance);
			if (Polygon_Contain_Point(polygon, test_p)) {
				return result;
			}
		}
	}
	p_result = NULL;
	for (i = 0; i < polygon->v_indices_cnt; ) {
		CCTResult_t result_temp;
		CCTNum_t edge[2][3];
		mathVec3Copy(edge[0], polygon->v[polygon->v_indices[i++]]);
		mathVec3Copy(edge[1], polygon->v[polygon->v_indices[i >= polygon->v_indices_cnt ? 0 : i]]);
		if (!Segment_Sweep_Segment(ls, dir, (const CCTNum_t(*)[3])edge, &result_temp)) {
			continue;
		}
		if (!p_result || p_result->distance > result_temp.distance) {
			p_result = result;
			copy_result(result, &result_temp);
		}
	}
	return p_result;
}

static CCTResult_t* Segment_Sweep_ConvexMesh(const CCTNum_t ls[2][3], const CCTNum_t dir[3], const GeometryMesh_t* mesh, CCTResult_t* result) {
	unsigned int i;
	CCTResult_t* p_result;
	if (ConvexMesh_Contain_Point(mesh, ls[0])) {
		set_result(result, CCTNum(0.0), dir);
		return result;
	}
	if (ConvexMesh_Contain_Point(mesh, ls[1])) {
		set_result(result, CCTNum(0.0), dir);
		return result;
	}
	p_result = NULL;
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		CCTResult_t result_temp;
		const GeometryPolygon_t* polygon = mesh->polygons + i;
		if (!Segment_Sweep_Polygon(ls, dir, polygon, &result_temp)) {
			continue;
		}
		if (!p_result || p_result->distance > result_temp.distance) {
			copy_result(result, &result_temp);
			p_result = result;
		}
	}
	return p_result;
}

static CCTResult_t* Segment_Sweep_Sphere(const CCTNum_t ls[2][3], const CCTNum_t dir[3], const CCTNum_t center[3], CCTNum_t radius, CCTResult_t* result) {
	CCTNum_t p[3];
	int res = Sphere_Intersect_Segment(center, radius, ls, p);
	if (1 == res) {
		set_result(result, CCTNum(0.0), dir);
		add_result_hit_point(result, p);
		return result;
	}
	else if (2 == res) {
		set_result(result, CCTNum(0.0), dir);
		return result;
	}
	else {
		CCTResult_t results[2];
		CCTNum_t lsdir[3], N[3];
		mathVec3Sub(lsdir, ls[1], ls[0]);
		mathVec3Cross(N, lsdir, dir);
		if (mathVec3IsZero(N)) {
			if (!Ray_Sweep_Sphere(ls[0], dir, center, radius, &results[0])) {
				return NULL;
			}
			if (!Ray_Sweep_Sphere(ls[1], dir, center, radius, &results[1])) {
				return NULL;
			}
			copy_result(result, results[0].distance < results[1].distance ? &results[0] : &results[1]);
			return result;
		}
		else {
			int i;
			CCTResult_t* p_result;
			CCTNum_t circle_o[3], circle_radius;
			mathVec3Normalized(N, N);
			res = Sphere_Intersect_Plane(center, radius, ls[0], N, circle_o, &circle_radius);
			if (res) {
				CCTNum_t plo[3], p[3];
				CCTNum_t plo_lensq, circle_radius_sq;
				mathVec3Normalized(lsdir, lsdir);
				mathPointProjectionLine(circle_o, ls[0], lsdir, p);
				mathVec3Sub(plo, circle_o, p);
				plo_lensq = mathVec3LenSq(plo);
				circle_radius_sq = circle_radius * circle_radius;
				if (plo_lensq > circle_radius_sq) {
					CCTNum_t new_ls[2][3], d;
					CCTNum_t dot = mathVec3Dot(plo, dir);
					if (dot <= CCTNum(0.0)) {
						return NULL;
					}
					d = mathVec3Normalized(plo, plo);
					dot = mathVec3Dot(plo, dir);
					d -= circle_radius;
					mathVec3AddScalar(p, plo, d);
					d /= dot;
					if (d <= CCTNum(0.0)) {
						return NULL;
					}
					mathVec3Copy(new_ls[0], ls[0]);
					mathVec3Copy(new_ls[1], ls[1]);
					mathVec3AddScalar(new_ls[0], dir, d);
					mathVec3AddScalar(new_ls[1], dir, d);
					if (Segment_Contain_Point((const CCTNum_t(*)[3])new_ls, p)) {
						set_result(result, d, plo);
						add_result_hit_point(result, p);
						return result;
					}
				}
				else if (Segment_Contain_Point(ls, p)) {
					mathVec3Normalized(plo, plo);
					set_result(result, CCTNum(0.0), plo);
					add_result_hit_point(result, p);
					return result;
				}
				p_result = NULL;
				for (i = 0; i < 2; ++i) {
					if (!Ray_Sweep_Sphere(ls[i], dir, center, radius, &results[i])) {
						continue;
					}
					if (!p_result || p_result->distance > results[i].distance) {
						p_result = &results[i];
					}
				}
				if (p_result) {
					copy_result(result, p_result);
					return result;
				}
			}
			return NULL;
		}
	}
}

static CCTResult_t* Polygon_Sweep_Plane(const GeometryPolygon_t* polygon, const CCTNum_t dir[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTResult_t* result) {
	int i, has_gt0 = 0, has_le0 = 0, idx_min = -1;
	CCTNum_t min_d, dot;
	for (i = 0; i < polygon->v_indices_cnt; ++i) {
		CCTNum_t d;
		mathPointProjectionPlane(polygon->v[polygon->v_indices[i]], plane_v, plane_n, NULL, &d);
		if (d > CCT_EPSILON) {
			if (has_le0) {
				set_result(result, CCTNum(0.0), dir);
				return result;
			}
			has_gt0 = 1;
		}
		else if (d < CCT_EPSILON_NEGATE) {
			if (has_gt0) {
				set_result(result, CCTNum(0.0), dir);
				return result;
			}
			has_le0 = 1;
		}

		if (!i || CCTNum_abs(min_d) > CCTNum_abs(d)) {
			min_d = d;
			idx_min = i;
		}
		else if (CCTNum_abs(min_d) >= CCTNum_abs(d) - CCT_EPSILON) {
			idx_min = -1;
		}
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
	if (idx_min >= 0) {
		add_result_hit_point(result, polygon->v[polygon->v_indices[idx_min]]);
		mathVec3AddScalar(result->unique_hit_point, dir, min_d);
	}
	return result;
}

static CCTResult_t* Polygon_Sweep_Polygon(const GeometryPolygon_t* polygon1, const CCTNum_t dir[3], const GeometryPolygon_t* polygon2, CCTResult_t* result) {
	int i;
	CCTNum_t neg_dir[3];
	CCTResult_t* p_result;
	if (!Plane_Intersect_Plane(polygon1->v[polygon1->v_indices[0]], polygon1->normal, polygon2->v[polygon2->v_indices[0]], polygon2->normal)) {
		CCTNum_t d, dot;
		mathPointProjectionPlane(polygon1->v[polygon1->v_indices[0]], polygon2->v[polygon2->v_indices[0]], polygon2->normal, NULL, &d);
		dot = mathVec3Dot(dir, polygon2->normal);
		if (dot <= CCT_EPSILON && dot >= CCT_EPSILON_NEGATE) {
			return NULL;
		}
		d /= dot;
		if (d < CCTNum(0.0)) {
			return NULL;
		}
	}
	p_result = NULL;
	for (i = 0; i < polygon1->v_indices_cnt; ) {
		CCTResult_t result_temp;
		CCTNum_t edge[2][3];
		mathVec3Copy(edge[0], polygon1->v[polygon1->v_indices[i++]]);
		mathVec3Copy(edge[1], polygon1->v[polygon1->v_indices[i >= polygon1->v_indices_cnt ? 0 : i]]);
		if (!Segment_Sweep_Polygon((const CCTNum_t(*)[3])edge, dir, polygon2, &result_temp)) {
			continue;
		}
		if (result_temp.distance <= CCTNum(0.0)) {
			return set_result(result, CCTNum(0.0), result_temp.hit_normal);
		}
		if (!p_result || p_result->distance > result_temp.distance) {
			p_result = result;
			copy_result(result, &result_temp);
		}
	}
	mathVec3Negate(neg_dir, dir);
	for (i = 0; i < polygon2->v_indices_cnt; ) {
		CCTResult_t result_temp;
		CCTNum_t edge[2][3];
		mathVec3Copy(edge[0], polygon2->v[polygon2->v_indices[i++]]);
		mathVec3Copy(edge[1], polygon2->v[polygon2->v_indices[i >= polygon2->v_indices_cnt ? 0 : i]]);
		if (!Segment_Sweep_Polygon((const CCTNum_t(*)[3])edge, neg_dir, polygon1, &result_temp)) {
			continue;
		}
		if (result_temp.distance <= CCTNum(0.0)) {
			return set_result(result, CCTNum(0.0), result_temp.hit_normal);
		}
		if (!p_result || p_result->distance > result_temp.distance) {
			p_result = result;
			copy_result(result, &result_temp);
		}
	}
	if (p_result) {
		p_result->has_unique_hit_point = 0;
	}
	return p_result;
}

static CCTResult_t* Polygon_Sweep_ConvexMesh(const GeometryPolygon_t* polygon, const CCTNum_t dir[3], const GeometryMesh_t* mesh, CCTResult_t* result) {
	unsigned int i;
	CCTResult_t* p_result = NULL;
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		CCTResult_t result_temp;
		if (!Polygon_Sweep_Polygon(polygon, dir, mesh->polygons + i, &result_temp)) {
			continue;
		}
		if (!p_result || p_result->distance > result_temp.distance) {
			p_result = result;
			copy_result(p_result, &result_temp);
		}
	}
	return p_result;
}

static CCTResult_t* Box_Sweep_Plane(const CCTNum_t v[8][3], const CCTNum_t dir[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTResult_t* result) {
	int i;
	CCTResult_t* p_result = NULL;
	for (i = 0; i < 8; ++i) {
		CCTResult_t result_temp;
		if (!Ray_Sweep_Plane(v[i], dir, plane_v, plane_n, &result_temp)) {
			if (p_result) {
				set_result(result, CCTNum(0.0), plane_n);
				return result;
			}
			continue;
		}
		if (!p_result) {
			if (i > 0) {
				set_result(result, CCTNum(0.0), plane_n);
				return result;
			}
			copy_result(result, &result_temp);
			p_result = result;
			continue;
		}
		if (p_result->distance < result_temp.distance - CCT_EPSILON) {
			continue;
		}
		if (p_result->distance <= result_temp.distance + CCT_EPSILON) {
			p_result->has_unique_hit_point = 0;
			continue;
		}
		copy_result(result, &result_temp);
	}
	return p_result;
}

static CCTResult_t* OBB_Sweep_Plane(const GeometryOBB_t* obb, const CCTNum_t dir[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTResult_t* result) {
	CCTNum_t v[8][3];
	mathOBBVertices(obb, v);
	return Box_Sweep_Plane((const CCTNum_t(*)[3])v, dir, plane_v, plane_n, result);
}

static CCTResult_t* AABB_Sweep_Plane(const CCTNum_t o[3], const CCTNum_t half[3], const CCTNum_t dir[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTResult_t* result) {
	CCTNum_t v[8][3];
	mathAABBVertices(o, half, v);
	return Box_Sweep_Plane((const CCTNum_t(*)[3])v, dir, plane_v, plane_n, result);
}

static CCTResult_t* Mesh_Sweep_Plane(const GeometryMesh_t* mesh, const CCTNum_t dir[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTResult_t* result) {
	int i;
	CCTResult_t* p_result = NULL;
	for (i = 0; i < mesh->v_indices_cnt; ++i) {
		CCTResult_t result_temp;
		const CCTNum_t* v = mesh->v[mesh->v_indices[i]];
		if (!Ray_Sweep_Plane(v, dir, plane_v, plane_n, &result_temp)) {
			if (p_result) {
				set_result(result, CCTNum(0.0), plane_n);
				return result;
			}
			continue;
		}
		if (!p_result) {
			if (i > 0) {
				set_result(result, CCTNum(0.0), plane_n);
				return result;
			}
			copy_result(result, &result_temp);
			p_result = result;
			continue;
		}
		if (p_result->distance < result_temp.distance - CCT_EPSILON) {
			continue;
		}
		if (p_result->distance <= result_temp.distance + CCT_EPSILON) {
			p_result->has_unique_hit_point = 0;
			continue;
		}
		copy_result(result, &result_temp);
	}
	return p_result;
}

static CCTResult_t* AABB_Sweep_AABB(const CCTNum_t o1[3], const CCTNum_t half1[3], const CCTNum_t dir[3], const CCTNum_t o2[3], const CCTNum_t half2[3], CCTResult_t* result) {
	if (AABB_Intersect_AABB(o1, half1, o2, half2)) {
		set_result(result, CCTNum(0.0), dir);
		return result;
	}
	else {
		CCTResult_t *p_result = NULL;
		int i;
		CCTNum_t v1[6][3], v2[6][3];
		mathAABBPlaneVertices(o1, half1, v1);
		mathAABBPlaneVertices(o2, half2, v2);
		for (i = 0; i < 6; ++i) {
			CCTNum_t new_o1[3];
			CCTResult_t result_temp;
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
			if (!p_result || p_result->distance > result_temp.distance) {
				p_result = result;
				copy_result(p_result, &result_temp);
			}
		}
		if (p_result) {
			p_result->has_unique_hit_point = 0;
		}
		return p_result;
	}
}

static CCTResult_t* OBB_Sweep_Polygon(const GeometryOBB_t* obb, const CCTNum_t dir[3], const GeometryPolygon_t* polygon, CCTResult_t* result) {
	int i;
	CCTNum_t v[8][3], neg_dir[3];
	CCTResult_t *p_result;

	mathOBBVertices(obb, v);
	if (!Box_Sweep_Plane((const CCTNum_t(*)[3])v, dir, polygon->v[polygon->v_indices[0]], polygon->normal, result)) {
		return NULL;
	}
	if (result->has_unique_hit_point) {
		if (Polygon_Contain_Point(polygon, result->unique_hit_point)) {
			return result;
		}
	}
	else if (result->distance != CCTNum(0.0)) {
		for (i = 0; i < 8; ++i) {
			CCTNum_t test_p[3];
			mathVec3Copy(test_p, v[i]);
			mathVec3AddScalar(test_p, dir, result->distance);
			if (Polygon_Contain_Point(polygon, test_p)) {
				return result;
			}
		}
	}
	p_result = NULL;
	mathVec3Negate(neg_dir, dir);
	for (i = 0; i < polygon->v_indices_cnt; ) {
		CCTResult_t result_temp;
		CCTNum_t edge[2][3];
		mathVec3Copy(edge[0], polygon->v[polygon->v_indices[i++]]);
		mathVec3Copy(edge[1], polygon->v[polygon->v_indices[i >= polygon->v_indices_cnt ? 0 : i]]);
		if (!Segment_Sweep_OBB((const CCTNum_t(*)[3])edge, neg_dir, obb, &result_temp)) {
			continue;
		}
		if (!p_result || p_result->distance > result_temp.distance) {
			p_result = result;
			copy_result(p_result, &result_temp);
		}
	}
	if (p_result && p_result->has_unique_hit_point) {
		mathVec3AddScalar(p_result->unique_hit_point, dir, p_result->distance);
	}
	return p_result;
}

static CCTResult_t* OBB_Sweep_ConvexMesh(const GeometryOBB_t* obb, const CCTNum_t dir[3], const GeometryMesh_t* mesh, CCTResult_t* result) {
	unsigned int i;
	CCTResult_t* p_result = NULL;
	for (i = 0; i < 6; ++i) {
		GeometryRect_t rect;
		GeometryPolygon_t polygon;
		CCTResult_t result_temp;
		CCTNum_t p[4][3];

		mathOBBPlaneRect(obb, i, &rect);
		mathRectToPolygon(&rect, &polygon, p);
		if (!Polygon_Sweep_ConvexMesh(&polygon, dir, mesh, &result_temp)) {
			continue;
		}
		if (!p_result || p_result->distance > result_temp.distance) {
			p_result = result;
			copy_result(p_result, &result_temp);
		}
	}
	return p_result;
}

static CCTResult_t* OBB_Sweep_OBB(const GeometryOBB_t* obb1, const CCTNum_t dir[3], const GeometryOBB_t* obb2, CCTResult_t* result) {
	int i;
	CCTResult_t* p_result;
	if (OBB_Intersect_OBB(obb1, obb2)) {
		set_result(result, CCTNum(0.0), dir);
		return result;
	}
	p_result = NULL;
	for (i = 0; i < 6; ++i)	{
		CCTNum_t p[4][3];
		CCTResult_t result_temp;
		GeometryRect_t rect2;
		GeometryPolygon_t polygon2;
		mathOBBPlaneRect(obb2, i, &rect2);
		mathRectToPolygon(&rect2, &polygon2, p);
		if (!OBB_Sweep_Polygon(obb1, dir, &polygon2, &result_temp)) {
			continue;
		}
		if (!p_result || p_result->distance > result_temp.distance) {
			p_result = result;
			copy_result(result, &result_temp);
		}
	}
	return p_result;
}

static CCTResult_t* Sphere_Sweep_Plane(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t dir[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTResult_t* result) {
	CCTNum_t dn, dn_sq, radius_sq;
	mathPointProjectionPlane(o, plane_v, plane_n, NULL, &dn);
	dn_sq = dn * dn;
	radius_sq = radius * radius;
	if (dn_sq < radius_sq - CCT_EPSILON) {
		set_result(result, CCTNum(0.0), dir);
		return result;
	}
	else if (dn_sq <= radius_sq + CCT_EPSILON) {
		set_result(result, CCTNum(0.0), dir);
		add_result_hit_point(result, o);
		mathVec3AddScalar(result->unique_hit_point, plane_n, dn);
		return result;
	}
	else {
		CCTNum_t d, dn_abs, cos_theta = mathVec3Dot(plane_n, dir);
		if (cos_theta <= CCT_EPSILON && cos_theta >= CCT_EPSILON_NEGATE) {
			return NULL;
		}
		d = dn / cos_theta;
		if (d < CCTNum(0.0)) {
			return NULL;
		}
		dn_abs = CCTNum_abs(dn);
		d -= radius / dn_abs * d;
		set_result(result, d, plane_n);
		add_result_hit_point(result, o);
		mathVec3AddScalar(result->unique_hit_point, dir, d);
		if (dn >= CCTNum(0.0)) {
			mathVec3AddScalar(result->unique_hit_point, plane_n, radius);
		}
		else {
			mathVec3SubScalar(result->unique_hit_point, plane_n, radius);
		}
		return result;
	}
}

static CCTResult_t* Sphere_Sweep_Polygon(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t dir[3], const GeometryPolygon_t* polygon, CCTResult_t* result) {
	int i;
	CCTResult_t *p_result = NULL;
	CCTNum_t neg_dir[3], p[3], dn;

	mathPointProjectionPlane(o, polygon->v[polygon->v_indices[0]], polygon->normal, p, &dn);
	if (radius * radius > dn * dn + CCT_EPSILON) {
		if (Polygon_Contain_Point(polygon, p)) {
			set_result(result, CCTNum(0.0), dir);
			return result;
		}
	}
	else if (radius * radius >= dn * dn - CCT_EPSILON) {
		if (Polygon_Contain_Point(polygon, p)) {
			set_result(result, CCTNum(0.0), dir);
			add_result_hit_point(result, p);
			return result;
		}
	}
	else {
		CCTNum_t d, dn_abs;
		CCTNum_t cos_theta = mathVec3Dot(polygon->normal, dir);
		if (cos_theta <= CCT_EPSILON && cos_theta >= CCT_EPSILON_NEGATE) {
			return NULL;
		}
		d = dn / cos_theta;
		if (d < CCTNum(0.0)) {
			return NULL;
		}
		dn_abs = CCTNum_abs(dn);
		d -= radius / dn_abs * d;
		mathVec3Copy(p, o);
		mathVec3AddScalar(p, dir, d);
		mathVec3AddScalar(p, polygon->normal, dn >= CCTNum(0.0) ? radius : -radius);
		if (Polygon_Contain_Point(polygon, p)) {
			set_result(result, d, polygon->normal);
			add_result_hit_point(result, p);
			return result;
		}
	}
	p_result = NULL;
	mathVec3Negate(neg_dir, dir);
	for (i = 0; i < polygon->v_indices_cnt; ) {
		CCTResult_t result_temp;
		CCTNum_t edge[2][3];
		mathVec3Copy(edge[0], polygon->v[polygon->v_indices[i++]]);
		mathVec3Copy(edge[1], polygon->v[polygon->v_indices[i >= polygon->v_indices_cnt ? 0 : i]]);
		if (!Segment_Sweep_Sphere((const CCTNum_t(*)[3])edge, neg_dir, o, radius, &result_temp)) {
			continue;
		}
		if (!p_result || p_result->distance > result_temp.distance) {
			p_result = result;
			copy_result(result, &result_temp);
		}
	}
	if (p_result) {
		mathVec3AddScalar(p_result->unique_hit_point, dir, p_result->distance);
	}
	return p_result;
}

static CCTResult_t* Sphere_Sweep_Sphere(const CCTNum_t o1[3], CCTNum_t r1, const CCTNum_t dir[3], const CCTNum_t o2[3], CCTNum_t r2, CCTResult_t* result) {
	if (!Ray_Sweep_Sphere(o1, dir, o2, r1 + r2, result)) {
		return NULL;
	}
	mathVec3Sub(result->hit_normal, result->unique_hit_point, o2);
	mathVec3Normalized(result->hit_normal, result->hit_normal);
	mathVec3SubScalar(result->unique_hit_point, result->hit_normal, r1);
	return result;
}

static CCTResult_t* Sphere_Sweep_OBB(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t dir[3], const GeometryOBB_t* obb, CCTResult_t* result) {
	if (Sphere_Intersect_OBB(o, radius, obb)) {
		set_result(result, CCTNum(0.0), dir);
		return result;
	}
	else {
		CCTResult_t* p_result = NULL;
		CCTNum_t v[8][3], neg_dir[3];
		int i;
		for (i = 0; i < 6; ++i) {
			CCTResult_t result_temp;
			GeometryRect_t rect;
			mathOBBPlaneRect(obb, i, &rect);
			if (!Sphere_Sweep_Plane(o, radius, dir, rect.o, rect.normal, &result_temp)) {
				continue;
			}
			if (result_temp.distance <= CCT_EPSILON && result_temp.distance >= CCT_EPSILON_NEGATE) {
				continue;
			}
			if (!mathRectHasPoint(&rect, result_temp.unique_hit_point)) {
				continue;
			}
			if (!p_result || p_result->distance > result_temp.distance) {
				p_result = result;
				copy_result(p_result, &result_temp);
			}
		}
		mathOBBVertices(obb, v);
		mathVec3Negate(neg_dir, dir);
		for (i = 0; i < sizeof(Box_Edge_Indices) / sizeof(Box_Edge_Indices[0]); i += 2) {
			CCTNum_t edge[2][3];
			CCTResult_t result_temp;
			mathVec3Copy(edge[0], v[Box_Edge_Indices[i]]);
			mathVec3Copy(edge[1], v[Box_Edge_Indices[i+1]]);
			if (!Segment_Sweep_Sphere((const CCTNum_t(*)[3])edge, neg_dir, o, radius, &result_temp)) {
				continue;
			}
			if (!p_result || p_result->distance > result_temp.distance) {
				p_result = result;
				copy_result(p_result, &result_temp);
			}
		}
		return p_result;
	}
}

static CCTResult_t* Sphere_Sweep_ConvexMesh(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t dir[3], const GeometryMesh_t* mesh, CCTResult_t* result) {
	unsigned int i;
	CCTResult_t* p_result;
	if (ConvexMesh_Contain_Point(mesh, o)) {
		set_result(result, CCTNum(0.0), dir);
		return result;
	}
	p_result = NULL;
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		CCTResult_t result_temp;
		if (!Sphere_Sweep_Polygon(o, radius, dir, mesh->polygons + i, &result_temp)) {
			continue;
		}
		if (!p_result || p_result->distance > result_temp.distance) {
			p_result = result;
			copy_result(p_result, &result_temp);
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

CCTResult_t* mathCollisionSweep(const GeometryBodyRef_t* one, const CCTNum_t dir[3], const GeometryBodyRef_t* two, CCTResult_t* result) {
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
				result = Ray_Sweep_OBB(one->point, dir, &obb2, result);
				break;
			}
			case GEOMETRY_BODY_OBB:
			{
				result = Ray_Sweep_OBB(one->point, dir, two->obb, result);
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
				result = Ray_Sweep_ConvexMesh(one->point, dir, two->mesh, result);
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
				result = Segment_Sweep_OBB(one_segment_v, dir, two->obb, result);
				break;
			}
			case GEOMETRY_BODY_AABB:
			{
				GeometryOBB_t obb2;
				mathOBBFromAABB(&obb2, two->aabb->o, two->aabb->half);
				result = Segment_Sweep_OBB(one_segment_v, dir, &obb2, result);
				break;
			}
			case GEOMETRY_BODY_SPHERE:
			{
				result = Segment_Sweep_Sphere(one_segment_v, dir, two->sphere->o, two->sphere->radius, result);
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
				result = AABB_Sweep_Plane(one->aabb->o, one->aabb->half, dir, two->plane->v, two->plane->normal, result);
				break;
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				GeometryOBB_t obb1;
				CCTNum_t neg_dir[3];
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				mathOBBFromAABB(&obb1, one->aabb->o, one->aabb->half);
				result = Segment_Sweep_OBB((const CCTNum_t(*)[3])two->segment->v, neg_dir, &obb1, result);
				break;
			}
			case GEOMETRY_BODY_POLYGON:
			{
				GeometryOBB_t obb1;
				mathOBBFromAABB(&obb1, one->aabb->o, one->aabb->half);
				result = OBB_Sweep_Polygon(&obb1, dir, two->polygon, result);
				break;
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				GeometryOBB_t obb1;
				mathOBBFromAABB(&obb1, one->aabb->o, one->aabb->half);
				result = OBB_Sweep_ConvexMesh(&obb1, dir, two->mesh, result);
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
				result = Segment_Sweep_OBB((const CCTNum_t(*)[3])two->segment->v, neg_dir, one->obb, result);
				break;
			}
			case GEOMETRY_BODY_PLANE:
			{
				result = OBB_Sweep_Plane(one->obb, dir, two->plane->v, two->plane->normal, result);
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
				result = OBB_Sweep_Polygon(one->obb, dir, two->polygon, result);
				break;
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				result = OBB_Sweep_ConvexMesh(one->obb, dir, two->mesh, result);
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
				result = Segment_Sweep_Sphere((const CCTNum_t(*)[3])two->segment->v, neg_dir, one->sphere->o, one->sphere->radius, result);
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
				result = Polygon_Sweep_Plane(one->polygon, dir, two->plane->v, two->plane->normal, result);
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
				CCTNum_t neg_dir[3];
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = OBB_Sweep_Polygon(two->obb, neg_dir, one->polygon, result);
				break;
			}
			case GEOMETRY_BODY_AABB:
			{
				GeometryOBB_t obb2;
				CCTNum_t neg_dir[3];
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				mathOBBFromAABB(&obb2, two->aabb->o, two->aabb->half);
				result = OBB_Sweep_Polygon(&obb2, neg_dir, one->polygon, result);
				break;
			}
			case GEOMETRY_BODY_POLYGON:
			{
				result = Polygon_Sweep_Polygon(one->polygon, dir, two->polygon, result);
				break;
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				result = Polygon_Sweep_ConvexMesh(one->polygon, dir, two->mesh, result);
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
				result = Mesh_Sweep_Plane(one->mesh, dir, two->plane->v, two->plane->normal, result);
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
				CCTNum_t neg_dir[3];
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = OBB_Sweep_ConvexMesh(two->obb, neg_dir, one->mesh, result);
				break;
			}
			case GEOMETRY_BODY_AABB:
			{
				GeometryOBB_t obb2;
				CCTNum_t neg_dir[3];
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				mathOBBFromAABB(&obb2, two->aabb->o, two->aabb->half);
				result = OBB_Sweep_ConvexMesh(&obb2, neg_dir, one->mesh, result);
				break;
			}
			case GEOMETRY_BODY_POLYGON:
			{
				CCTNum_t neg_dir[3];
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = Polygon_Sweep_ConvexMesh(two->polygon, neg_dir, one->mesh, result);
				break;
			}
		}
	}
	if (!result) {
		return NULL;
	}
	if (flag_neg_dir) {
		if (result->has_unique_hit_point) {
			mathVec3AddScalar(result->unique_hit_point, dir, result->distance);
		}
	}
	return result;
}

#ifdef __cplusplus
}
#endif
