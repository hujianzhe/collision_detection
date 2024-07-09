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
extern int Segment_Intersect_Plane(const CCTNum_t ls[2][3], const CCTNum_t plane_v[3], const CCTNum_t plane_normal[3], CCTNum_t p[3], CCTNum_t d[3]);
extern int Segment_Intersect_ConvexMesh(const CCTNum_t ls[2][3], const GeometryMesh_t* mesh);
extern int Sphere_Intersect_Segment(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t ls[2][3], CCTNum_t p[3]);
extern int Sphere_Intersect_Plane(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t plane_v[3], const CCTNum_t plane_normal[3], CCTNum_t new_o[3], CCTNum_t* new_r);
extern int OBB_Intersect_OBB(const GeometryOBB_t* obb0, const GeometryOBB_t* obb1);
extern int Sphere_Intersect_OBB(const CCTNum_t o[3], CCTNum_t radius, const GeometryOBB_t* obb);
extern int Sphere_Intersect_ConvexMesh(const CCTNum_t o[3], CCTNum_t radius, const GeometryMesh_t* mesh);
extern int ConvexMesh_Contain_Point(const GeometryMesh_t* mesh, const CCTNum_t p[3]);
extern int ConvexMesh_Intersect_ConvexMesh(const GeometryMesh_t* mesh1, const GeometryMesh_t* mesh2);
extern int Polygon_Contain_Point(const GeometryPolygon_t* polygon, const CCTNum_t p[3]);
extern int Polygon_Intersect_Polygon(const GeometryPolygon_t* polygon1, const GeometryPolygon_t* polygon2);
extern int Polygon_Intersect_Circle(const GeometryPolygon_t* polygon, const GeometryCircle_t* circle);
extern int ConvexMesh_Intersect_Polygon(const GeometryMesh_t* mesh, const GeometryPolygon_t* polygon);
extern int Vertices_Intersect_Plane(const CCTNum_t(*v)[3], const unsigned int* v_indices, unsigned int v_indices_cnt, const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTNum_t* p_min_d, unsigned int* p_v_indices_idx);

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

static void merge_result(CCTSweepResult_t* dst, CCTSweepResult_t* src) {
	if (dst->distance < src->distance) {
		return;
	}
	*dst = *src;
}

static CCTSweepResult_t* set_intersect(CCTSweepResult_t* result) {
	result->distance = CCTNum(0.0);
	mathVec3Set(result->hit_plane_n, CCTNums_3(0.0, 0.0, 0.0));
	result->hit_bits = 0;
	result->peer[0].hit_bits = 0;
	result->peer[0].idx = 0;
	result->peer[1].hit_bits = 0;
	result->peer[1].idx = 0;
	return result;
}

static void reverse_result(CCTSweepResult_t* result, const CCTNum_t dir[3]) {
	int hit_bits = result->peer[0].hit_bits;
	unsigned int idx = result->peer[0].idx;
	result->peer[0] = result->peer[1];
	result->peer[1].hit_bits = hit_bits;
	result->peer[1].idx = idx;
	if (result->hit_bits & CCT_SWEEP_BIT_POINT) {
		mathVec3AddScalar(result->hit_plane_v, dir, result->distance);
	}
}

static void set_unique_hit_point(CCTSweepResult_t* result, const CCTNum_t p[3]) {
	result->hit_bits |= CCT_SWEEP_BIT_POINT;
	mathVec3Copy(result->hit_plane_v, p);
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
			set_intersect(result);
			set_unique_hit_point(result, o);
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
		result->peer[1].hit_bits = CCT_SWEEP_BIT_POINT;
		if (mathVec3LenSq(v0) > mathVec3LenSq(v1)) {
			dot = mathVec3Dot(dir, v1);
			mathVec3Copy(result->hit_plane_v, ls[1]);
			result->peer[1].idx = 1;
		}
		else {
			mathVec3Copy(result->hit_plane_v, ls[0]);
			result->peer[1].idx = 0;
		}
		mathVec3Copy(result->hit_plane_n, dir);
		result->distance = dot;
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
			set_intersect(result);
			set_unique_hit_point(result, o);
			return result;
		}
		dot = mathVec3Dot(op, dir);
		if (dot <= CCTNum(0.0)) {
			return NULL;
		}
		d = mathVec3Normalized(op, op);
		dot = mathVec3Dot(op, dir);
		if (dot <= CCTNum(0.0)) {
			return NULL;
		}
		d /= dot;
		mathVec3Copy(p, o);
		mathVec3AddScalar(p, dir, d);

		if (mathVec3Equal(ls[0], p)) {
			result->peer[1].hit_bits = CCT_SWEEP_BIT_POINT;
			result->peer[1].idx = 0;
		}
		else if (mathVec3Equal(ls[1], p)) {
			result->peer[1].hit_bits = CCT_SWEEP_BIT_POINT;
			result->peer[1].idx = 1;
		}
		else {
			mathVec3Sub(v0, ls[0], p);
			mathVec3Sub(v1, ls[1], p);
			dot = mathVec3Dot(v0, v1);
			if (dot > CCTNum(0.0)) {
				return NULL;
			}
			result->peer[1].hit_bits = CCT_SWEEP_BIT_SEGMENT;
			result->peer[1].idx = 0;
		}
		mathVec3Copy(result->hit_plane_v, p);
		mathVec3Copy(result->hit_plane_n, op);
		result->distance = d;
	}
	result->hit_bits = CCT_SWEEP_BIT_POINT;
	result->peer[0].hit_bits = CCT_SWEEP_BIT_POINT;
	result->peer[0].idx = 0;
	return result;
}

static CCTSweepResult_t* Ray_Sweep_SegmentIndices(const CCTNum_t o[3], const CCTNum_t dir[3], const GeometrySegmentIndices_t* si, CCTSweepResult_t* result) {
	unsigned int i;
	CCTSweepResult_t* p_result = NULL;
	for (i = 0; i < si->indices_cnt; ) {
		CCTSweepResult_t result_temp;
		CCTNum_t edge[2][3];
		unsigned int v_indices_idx[2];
		v_indices_idx[0] = i++;
		if (2 == si->stride) {
			v_indices_idx[1] = i++;
		}
		else {
			v_indices_idx[1] = (i >= si->indices_cnt ? 0 : i);
		}
		mathVec3Copy(edge[0], si->v[si->indices[v_indices_idx[0]]]);
		mathVec3Copy(edge[1], si->v[si->indices[v_indices_idx[1]]]);
		if (!Ray_Sweep_Segment(o, dir, (const CCTNum_t(*)[3])edge, &result_temp)) {
			continue;
		}
		if (result_temp.distance <= CCTNum(0.0)) {
			*result = result_temp;
			return result;
		}
		if (result_temp.peer[1].hit_bits & CCT_SWEEP_BIT_POINT) {
			result_temp.peer[1].idx = v_indices_idx[result_temp.peer[1].idx ? 1 : 0];
		}
		else {
			result_temp.peer[1].hit_bits = CCT_SWEEP_BIT_SEGMENT;
			result_temp.peer[1].idx = (i - 1) / si->stride;
		}
		if (!p_result) {
			p_result = result;
			*p_result = result_temp;
		}
		else {
			merge_result(p_result, &result_temp);
		}
	}
	return p_result;
}

static CCTSweepResult_t* Ray_Sweep_Plane(const CCTNum_t o[3], const CCTNum_t dir[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTSweepResult_t* result) {
	CCTNum_t d, cos_theta;
	d = mathPointProjectionPlane(o, plane_v, plane_n, NULL);
	if (CCTNum(0.0) == d) {
		set_intersect(result);
		set_unique_hit_point(result, o);
		return result;
	}
	cos_theta = mathVec3Dot(dir, plane_n);
	if (CCTNum(0.0) == cos_theta) {
		return NULL;
	}
	d /= cos_theta;
	if (d < CCTNum(0.0)) {
		return NULL;
	}
	mathVec3Copy(result->hit_plane_v, o);
	mathVec3AddScalar(result->hit_plane_v, dir, d);
	mathVec3Copy(result->hit_plane_n, plane_n);
	result->hit_bits = CCT_SWEEP_BIT_POINT;
	result->distance = d;
	result->peer[0].hit_bits = CCT_SWEEP_BIT_POINT;
	result->peer[0].idx = 0;
	result->peer[1].hit_bits = CCT_SWEEP_BIT_FACE;
	result->peer[1].idx = 0;
	return result;
}

static CCTSweepResult_t* Ray_Sweep_Polygon(const CCTNum_t o[3], const CCTNum_t dir[3], const GeometryPolygon_t* polygon, CCTSweepResult_t* result) {
	CCTNum_t dot;
	GeometrySegmentIndices_t si;
	if (!Ray_Sweep_Plane(o, dir, polygon->v[polygon->v_indices[0]], polygon->normal, result)) {
		return NULL;
	}
	if (Polygon_Contain_Point(polygon, result->hit_plane_v)) {
		return result;
	}
	if (result->distance > CCTNum(0.0)) {
		return NULL;
	}
	dot = mathVec3Dot(dir, polygon->normal);
	if (dot < CCT_EPSILON_NEGATE || dot > CCT_EPSILON) {
		return NULL;
	}
	si.v = polygon->v;
	si.indices = polygon->v_indices;
	si.indices_cnt = polygon->v_indices_cnt;
	si.stride = 1;
	return Ray_Sweep_SegmentIndices(o, dir, &si, result);
}

static CCTSweepResult_t* Ray_Sweep_Sphere(const CCTNum_t o[3], const CCTNum_t dir[3], const CCTNum_t sp_o[3], CCTNum_t sp_radius, CCTSweepResult_t* result) {
	CCTNum_t d_sq, oc_lensq, dir_d;
	CCTNum_t oc[3];
	CCTNum_t radius_sq = CCTNum_sq(sp_radius);
	mathVec3Sub(oc, sp_o, o);
	oc_lensq = mathVec3LenSq(oc);
	if (oc_lensq <= radius_sq) {
		return set_intersect(result);
	}
	dir_d = mathVec3Dot(dir, oc);
	if (dir_d <= CCTNum(0.0)) {
		return NULL;
	}
	d_sq = oc_lensq - CCTNum_sq(dir_d);
	if (d_sq > radius_sq) {
		return NULL;
	}
	dir_d -= CCTNum_sqrt(radius_sq - d_sq);
	mathVec3Copy(result->hit_plane_v, o);
	mathVec3AddScalar(result->hit_plane_v, dir, dir_d);
	result->hit_bits = CCT_SWEEP_BIT_POINT;
	mathVec3Sub(result->hit_plane_n, result->hit_plane_v, sp_o);
	mathVec3MultiplyScalar(result->hit_plane_n, result->hit_plane_n, CCTNum(1.0) / sp_radius);
	result->distance = dir_d;
	result->peer[0].hit_bits = CCT_SWEEP_BIT_POINT;
	result->peer[0].idx = 0;
	result->peer[1].hit_bits = 0;
	result->peer[1].idx = 0;
	return result;
}

static CCTSweepResult_t* Ray_Sweep_Circle(const CCTNum_t o[3], const CCTNum_t dir[3], const GeometryCircle_t* circle, CCTSweepResult_t* result) {
	if (!Ray_Sweep_Plane(o, dir, circle->o, circle->normal, result)) {
		return NULL;
	}
	if (result->distance > CCTNum(0.0)) {
		CCTNum_t lensq = mathVec3DistanceSq(circle->o, result->hit_plane_v);
		if (lensq <= CCTNum_sq(circle->radius)) {
			return result;
		}
		return NULL;
	}
	if (!Ray_Sweep_Sphere(o, dir, circle->o, circle->radius, result)) {
		return NULL;
	}
	result->peer[1].hit_bits = CCT_SWEEP_BIT_FACE;
	return result;
}

static CCTSweepResult_t* Ray_Sweep_ConvexMesh(const CCTNum_t o[3], const CCTNum_t dir[3], const GeometryMesh_t* mesh, int check_intersect, CCTSweepResult_t* result) {
	unsigned int i;
	CCTSweepResult_t* p_result;
	GeometrySegmentIndices_t si;
	if (check_intersect && ConvexMesh_Contain_Point(mesh, o)) {
		return set_intersect(result);
	}
	p_result = NULL;
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		CCTSweepResult_t result_temp;
		const GeometryPolygon_t* polygon = mesh->polygons + i;
		if (!Ray_Sweep_Plane(o, dir, polygon->v[polygon->v_indices[0]], polygon->normal, &result_temp)) {
			continue;
		}
		if (!Polygon_Contain_Point(polygon, result_temp.hit_plane_v)) {
			continue;
		}
		result_temp.peer[1].hit_bits = CCT_SWEEP_BIT_FACE;
		result_temp.peer[1].idx = i;
		if (!p_result) {
			p_result = result;
			*p_result = result_temp;
		}
		else {
			merge_result(p_result, &result_temp);
		}
	}
	if (p_result) {
		return p_result;
	}
	si.v = mesh->v;
	si.indices = mesh->edge_indices;
	si.indices_cnt = mesh->edge_indices_cnt;
	si.stride = 2;
	return Ray_Sweep_SegmentIndices(o, dir, &si, result);
}

static CCTSweepResult_t* Segment_Sweep_Plane(const CCTNum_t ls[2][3], const CCTNum_t dir[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTSweepResult_t* result) {
	CCTNum_t p[3], d[3], dlen, cos_theta;
	int res = Segment_Intersect_Plane(ls, plane_v, plane_n, p, d);
	if (res) {
		set_intersect(result);
		if (1 == res) {
			set_unique_hit_point(result, p);
		}
		return result;
	}
	cos_theta = mathVec3Dot(plane_n, dir);
	if (CCTNum(0.0) == cos_theta) {
		return NULL;
	}
	dlen = d[2] / cos_theta;
	if (dlen < CCTNum(0.0)) {
		return NULL;
	}
	result->distance = dlen;
	mathVec3Copy(result->hit_plane_n, plane_n);
	if (d[0] == d[1]) {
		mathVec3Copy(result->hit_plane_v, plane_v);
		result->hit_bits = CCT_SWEEP_BIT_SEGMENT;
		result->peer[0].hit_bits = CCT_SWEEP_BIT_SEGMENT;
		result->peer[0].idx = 0;
	}
	else {
		result->peer[0].hit_bits = CCT_SWEEP_BIT_POINT;
		if (d[0] == d[2]) {
			mathVec3Copy(result->hit_plane_v, ls[0]);
			result->peer[0].idx = 0;
		}
		else {
			mathVec3Copy(result->hit_plane_v, ls[1]);
			result->peer[0].idx = 1;
		}
		mathVec3AddScalar(result->hit_plane_v, dir, dlen);
		result->hit_bits = CCT_SWEEP_BIT_POINT;
	}
	result->peer[1].hit_bits = CCT_SWEEP_BIT_FACE;
	result->peer[1].idx = 0;
	return result;
}

static CCTSweepResult_t* Segment_Sweep_Segment(const CCTNum_t ls1[2][3], const CCTNum_t dir[3], const CCTNum_t ls2[2][3], CCTSweepResult_t* result) {
	int i;
	CCTSweepResult_t* p_result;
	CCTNum_t N[3], ls1_dir[3], ls2_dir[3], v[3];
	CCTNum_t d, cos_theta;

	mathVec3Sub(ls1_dir, ls1[1], ls1[0]);
	mathVec3Sub(ls2_dir, ls2[1], ls2[0]);
	mathVec3Sub(v, ls2[0], ls1[0]);
	/* check Line vs Line parallel or collinear */
	mathVec3Cross(N, ls1_dir, ls2_dir);
	if (mathVec3IsZero(N)) {
		mathVec3Cross(N, v, ls1_dir);
		if (mathVec3IsZero(N)) {
			/* collinear */
			unsigned int closest_ls1_indice, closest_ls2_indice;
			CCTNum_t d;
			const CCTNum_t* intersect_p = NULL;
			if (Segment_Contain_Point(ls1, ls2[0])) {
				intersect_p = ls2[0];
			}
			if (Segment_Contain_Point(ls1, ls2[1])) {
				if (intersect_p && !mathVec3Equal(intersect_p, ls2[1])) {
					return set_intersect(result);
				}
				intersect_p = ls2[1];
			}
			if (Segment_Contain_Point(ls2, ls1[0])) {
				if (intersect_p && !mathVec3Equal(intersect_p, ls1[0])) {
					return set_intersect(result);
				}
				intersect_p = ls1[0];
			}
			if (Segment_Contain_Point(ls2, ls1[1])) {
				if (intersect_p && !mathVec3Equal(intersect_p, ls1[1])) {
					return set_intersect(result);
				}
				intersect_p = ls1[1];
			}
			if (intersect_p) {
				set_intersect(result);
				set_unique_hit_point(result, intersect_p);
				return result;
			}
			/* dir and ls1_dir must parallel */
			mathVec3Cross(N, ls1_dir, dir);
			if (!mathVec3IsZero(N)) {
				return NULL;
			}
			/* calculate result */
			mathSegmentSegmentClosestIndices(ls1, ls2, &closest_ls1_indice, &closest_ls2_indice);
			mathVec3Sub(N, ls2[closest_ls2_indice], ls1[closest_ls1_indice]);
			d = mathVec3Dot(N, dir);
			if (d <= CCTNum(0.0)) {
				return NULL;
			}
			mathVec3Copy(result->hit_plane_v, ls2[closest_ls2_indice]);
			mathVec3Copy(result->hit_plane_n, dir);
			result->hit_bits = CCT_SWEEP_BIT_POINT;
			result->distance = d;
			return result;
		}
		else {
			CCTNum_t intersect_points[2][3];
			unsigned int intersect_cnt;
			/* parallel, check ls2 on the <ls1,dir> plane */
			mathVec3Cross(N, ls1_dir, dir);
			if (mathVec3IsZero(N)) {
				return NULL;
			}
			cos_theta = mathVec3Dot(v, N);
			if (cos_theta < CCT_EPSILON_NEGATE || cos_theta > CCT_EPSILON) {
				return NULL;
			}
			/* calculate ray sweep distance */
			mathVec3Normalized(ls1_dir, ls1_dir);
			mathVec3Normalized(ls2_dir, ls2_dir);
			mathPointProjectionLine(ls1[0], ls2[0], ls2_dir, v);
			mathVec3Sub(N, v, ls1[0]);
			d = mathVec3Normalized(N, N);
			cos_theta = mathVec3Dot(N, dir);
			if (cos_theta <= CCTNum(0.0)) {
				return NULL;
			}
			d /= cos_theta;
			/* check ray point locate */
			p_result = NULL;
			intersect_cnt = 0;
			for (i = 0; i < 2; ++i) {
				CCTNum_t p[3], vp[3];
				mathVec3Copy(p, ls1[i]);
				mathVec3AddScalar(p, dir, d);
				mathVec3Sub(v, ls2[0], p);
				mathVec3Sub(vp, ls2[1], p);
				cos_theta = mathVec3Dot(v, vp);
				if (cos_theta > CCT_EPSILON) {
					continue;
				}
				if (intersect_cnt < 2) {
					if (0 == intersect_cnt || !mathVec3Equal(intersect_points[intersect_cnt], p)) {
						mathVec3Copy(intersect_points[intersect_cnt++], p);
					}
				}
				p_result = result;
			}
			if (!p_result) {
				for (i = 0; i < 2; ++i) {
					CCTNum_t p[3], vp[3];
					mathVec3Copy(p, ls2[i]);
					mathVec3SubScalar(p, dir, d);
					mathVec3Sub(v, ls1[0], p);
					mathVec3Sub(vp, ls1[1], p);
					cos_theta = mathVec3Dot(v, vp);
					if (cos_theta > CCT_EPSILON) {
						continue;
					}
					if (intersect_cnt < 2) {
						if (0 == intersect_cnt || !mathVec3Equal(intersect_points[intersect_cnt], ls2[i])) {
							mathVec3Copy(intersect_points[intersect_cnt++], ls2[i]);
						}
					}
					p_result = result;
				}
			}
			if (!p_result) {
				return NULL;
			}
			mathVec3Copy(result->hit_plane_v, intersect_points[0]);
			mathVec3Copy(result->hit_plane_n, N);
			if (1 == intersect_cnt) {
				result->hit_bits = CCT_SWEEP_BIT_POINT;
			}
			else {
				result->hit_bits = CCT_SWEEP_BIT_SEGMENT;
			}
			result->distance = d;
			return result;
		}
	}
	else {
		CCTNum_t p[3], vp[3];
		d = mathVec3Dot(v, N);
		if (d < CCT_EPSILON_NEGATE || d > CCT_EPSILON) {
			/* opposite */
			mathVec3Cross(N, ls1_dir, dir);
			if (mathVec3IsZero(N)) {
				return NULL;
			}
			mathVec3Normalized(N, N);
			if (Segment_Intersect_Plane(ls2, ls1[0], N, v, NULL) != 1) {
				return NULL;
			}
			mathVec3Normalized(ls1_dir, ls1_dir);
			mathPointProjectionLine(v, ls1[0], ls1_dir, p);
			mathVec3Sub(N, v, p);
			d = mathVec3Normalized(N, N);
			cos_theta = mathVec3Dot(N, dir);
			if (cos_theta <= CCTNum(0.0)) {
				return NULL;
			}
			d /= cos_theta;
			mathVec3Copy(p, v);
			mathVec3SubScalar(p, dir, d);
			mathVec3Sub(v, ls1[0], p);
			mathVec3Sub(vp, ls1[1], p);
			cos_theta = mathVec3Dot(v, vp);
			if (cos_theta > CCT_EPSILON) {
				return NULL;
			}
			mathVec3Copy(result->hit_plane_v, v);
			mathVec3Copy(result->hit_plane_n, N);
			result->hit_bits = CCT_SWEEP_BIT_POINT;
			result->distance = d;
			return result;
		}
		else {
			CCTNum_t hn[3], ls1_len, hn_len;
			/* cross */
			if (mathVec3IsZero(v)) {
				set_intersect(result);
				set_unique_hit_point(result, ls2[0]);
				return result;
			}
			/* calculate Line cross point */
			ls1_len = mathVec3Normalized(ls1_dir, ls1_dir);
			mathVec3Normalized(ls2_dir, ls2_dir);
			mathPointProjectionLine(ls1[0], ls2[0], ls2_dir, p);
			if (mathVec3Equal(p, ls1[0])) {
				/* ls1[0] is cross point */
				hn_len = d = CCTNum(0.0);
				mathVec3Set(hn, CCTNums_3(0.0, 0.0, 0.0));
				mathVec3Copy(p, ls1[0]);
			}
			else {
				mathVec3Sub(hn, p, ls1[0]);
				hn_len = mathVec3Normalized(hn, hn);
				cos_theta = mathVec3Dot(hn, ls1_dir);
				if (CCTNum(0.0) == cos_theta) {
					/* no possible */
					return NULL;
				}
				d = hn_len / cos_theta;
				mathVec3Copy(p, ls1[0]);
				mathVec3AddScalar(p, ls1_dir, d);
			}
			/* check cross point locate ls2 */
			mathVec3Sub(v, ls2[0], p);
			mathVec3Sub(vp, ls2[1], p);
			cos_theta = mathVec3Dot(v, vp);
			if (cos_theta <= CCT_EPSILON) {
				/* check cross point locate ls1 */
				if (d >= CCTNum(0.0) && d <= ls1_len + CCT_EPSILON) {
					set_intersect(result);
					set_unique_hit_point(result, p);
					return result;
				}
				mathVec3Cross(v, ls1_dir, dir);
				if (mathVec3IsZero(v)) {
					/* dir and ls1_dir parallel */
					mathVec3Sub(v, p, ls1[0]);
					cos_theta = mathVec3Dot(v, dir);
					if (cos_theta < CCTNum(0.0)) {
						return NULL;
					}
					mathVec3Sub(v, p, ls1[1]);
					d = mathVec3Dot(v, dir);
					if (d > cos_theta) {
						d = cos_theta;
					}
					mathVec3Copy(result->hit_plane_v, p);
					mathVec3Copy(result->hit_plane_n, hn);
					result->hit_bits = CCT_SWEEP_BIT_POINT;
					result->distance = d;
					return result;
				}
			}
			else {
				mathVec3Cross(v, ls1_dir, dir);
				if (mathVec3IsZero(v)) {
					return NULL;
				}
			}
			/* check dir on the <ls1,ls2> plane */
			cos_theta = mathVec3Dot(dir, N);
			if (cos_theta < CCT_EPSILON_NEGATE || cos_theta > CCT_EPSILON) {
				return NULL;
			}
			p_result = NULL;
			/* ls1 ray sweep ls2 */
			do {
				cos_theta = mathVec3Dot(hn, dir);
				if (cos_theta <= CCTNum(0.0)) {
					break;
				}
				hn_len /= cos_theta;
				mathVec3Copy(p, ls1[0]);
				mathVec3AddScalar(p, dir, hn_len);
				mathVec3Sub(v, ls2[0], p);
				mathVec3Sub(vp, ls2[1], p);
				cos_theta = mathVec3Dot(v, vp);
				if (cos_theta > CCT_EPSILON) {
					break;
				}
				mathVec3Copy(result->hit_plane_v, p);
				mathVec3Copy(result->hit_plane_n, hn);
				result->hit_bits = CCT_SWEEP_BIT_POINT;
				result->distance = hn_len;
				p_result = result;
			} while (0);
			do {
				mathPointProjectionLine(ls1[1], ls2[0], ls2_dir, p);
				mathVec3Sub(hn, p, ls1[1]);
				hn_len = mathVec3Normalized(hn, hn);
				cos_theta = mathVec3Dot(hn, dir);
				if (cos_theta <= CCTNum(0.0)) {
					break;
				}
				hn_len /= cos_theta;
				if (p_result && hn_len >= p_result->distance) {
					break;
				}
				mathVec3Copy(p, ls1[1]);
				mathVec3AddScalar(p, dir, hn_len);
				mathVec3Sub(v, ls2[0], p);
				mathVec3Sub(vp, ls2[1], p);
				cos_theta = mathVec3Dot(v, vp);
				if (cos_theta > CCT_EPSILON) {
					break;
				}
				mathVec3Copy(result->hit_plane_v, p);
				mathVec3Copy(result->hit_plane_n, hn);
				result->hit_bits = CCT_SWEEP_BIT_POINT;
				result->distance = hn_len;
				p_result = result;
			} while (0);
			/* ls2 ray sweep ls1 */
			for (i = 0; i < 2; ++i) {
				CCTNum_t p[3], vp[3];
				mathPointProjectionLine(ls2[i], ls1[0], ls1_dir, p);
				mathVec3Sub(hn, ls2[i], p);
				hn_len = mathVec3Normalized(hn, hn);
				cos_theta = mathVec3Dot(hn, dir);
				if (cos_theta <= CCTNum(0.0)) {
					continue;
				}
				hn_len /= cos_theta;
				if (p_result && hn_len >= p_result->distance) {
					continue;
				}
				mathVec3Copy(p, ls2[i]);
				mathVec3SubScalar(p, dir, hn_len);
				mathVec3Sub(v, ls1[0], p);
				mathVec3Sub(vp, ls1[1], p);
				cos_theta = mathVec3Dot(v, vp);
				if (cos_theta > CCT_EPSILON) {
					continue;
				}
				mathVec3Copy(result->hit_plane_v, ls2[i]);
				mathVec3Copy(result->hit_plane_n, hn);
				result->hit_bits = CCT_SWEEP_BIT_POINT;
				result->distance = hn_len;
				p_result = result;
			}
			return p_result;
		}
	}
}

static CCTSweepResult_t* Segment_Sweep_SegmentIndices(const CCTNum_t ls[2][3], const CCTNum_t dir[3], const GeometrySegmentIndices_t* si, CCTSweepResult_t* result) {
	unsigned int i;
	CCTSweepResult_t* p_result = NULL;
	for (i = 0; i < si->indices_cnt; ) {
		CCTSweepResult_t result_temp;
		CCTNum_t edge[2][3];
		unsigned int v_indices_idx[2];
		v_indices_idx[0] = i++;
		if (2 == si->stride) {
			v_indices_idx[1] = i++;
		}
		else {
			v_indices_idx[1] = (i >= si->indices_cnt ? 0 : i);
		}
		mathVec3Copy(edge[0], si->v[si->indices[v_indices_idx[0]]]);
		mathVec3Copy(edge[1], si->v[si->indices[v_indices_idx[1]]]);
		if (!Segment_Sweep_Segment(ls, dir, (const CCTNum_t(*)[3])edge, &result_temp)) {
			continue;
		}
		if (!p_result) {
			p_result = result;
			*p_result = result_temp;
		}
		else {
			merge_result(p_result, &result_temp);
		}
	}
	return p_result;
}

static CCTSweepResult_t* SegmentIndices_Sweep_SegmentIndices(const GeometrySegmentIndices_t* s1, const CCTNum_t dir[3], const GeometrySegmentIndices_t* s2, CCTSweepResult_t* result) {
	unsigned int i;
	CCTSweepResult_t* p_result = NULL;
	for (i = 0; i < s1->indices_cnt; ) {
		unsigned int j;
		CCTNum_t edge1[2][3];
		unsigned int v_indices_idx1[2];
		v_indices_idx1[0] = i++;
		if (2 == s1->stride) {
			v_indices_idx1[1] = i++;
		}
		else {
			v_indices_idx1[1] = (i >= s1->indices_cnt ? 0 : i);
		}
		mathVec3Copy(edge1[0], s1->v[s1->indices[v_indices_idx1[0]]]);
		mathVec3Copy(edge1[1], s1->v[s1->indices[v_indices_idx1[1]]]);
		for (j = 0; j < s2->indices_cnt; ) {
			CCTSweepResult_t result_temp;
			CCTNum_t edge2[2][3];
			unsigned int v_indices_idx2[2];
			v_indices_idx2[0] = j++;
			if (2 == s2->stride) {
				v_indices_idx2[1] = j++;
			}
			else {
				v_indices_idx2[1] = (j >= s2->indices_cnt ? 0 : j);
			}
			mathVec3Copy(edge2[0], s2->v[s2->indices[v_indices_idx2[0]]]);
			mathVec3Copy(edge2[1], s2->v[s2->indices[v_indices_idx2[1]]]);
			if (!Segment_Sweep_Segment((const CCTNum_t(*)[3])edge1, dir, (const CCTNum_t(*)[3])edge2, &result_temp)) {
				continue;
			}
			if (result_temp.distance <= CCTNum(0.0)) {
				*result = result_temp;
				return result;
			}
			if (!p_result) {
				p_result = result;
				*p_result = result_temp;
			}
			else {
				merge_result(p_result, &result_temp);
			}
		}
	}
	return p_result;
}

static CCTSweepResult_t* Segment_Sweep_Polygon(const CCTNum_t ls[2][3], const CCTNum_t dir[3], const GeometryPolygon_t* polygon, CCTSweepResult_t* result) {
	GeometrySegmentIndices_t si;
	CCTNum_t p[3], d[3];
	int res = Segment_Intersect_Plane(ls, polygon->v[polygon->v_indices[0]], polygon->normal, p, d);
	if (1 == res) {
		CCTNum_t v[3];
		if (Polygon_Contain_Point(polygon, p)) {
			set_intersect(result);
			set_unique_hit_point(result, p);
			return result;
		}
		mathVec3Sub(v, ls[1], ls[0]);
		mathVec3Cross(v, v, dir);
		if (mathVec3IsZero(v)) {
			return NULL;
		}
	}
	else if (2 == res) {
		if (Polygon_Contain_Point(polygon, ls[0]) || Polygon_Contain_Point(polygon, ls[1])) {
			return set_intersect(result);
		}
	}
	else {
		CCTNum_t dlen, cos_theta;
		cos_theta = mathVec3Dot(polygon->normal, dir);
		if (CCTNum(0.0) == cos_theta) {
			return NULL;
		}
		dlen = d[2] / cos_theta;
		if (dlen < CCTNum(0.0)) {
			return NULL;
		}
		if (d[0] == d[1]) {
			const CCTNum_t* pp = polygon->v[polygon->v_indices[0]];
			mathVec3Copy(p, ls[0]);
			mathVec3AddScalar(p, dir, dlen);
			if (Polygon_Contain_Point(polygon, p)) {
				mathVec3Copy(result->hit_plane_v, pp);
				mathVec3Copy(result->hit_plane_n, polygon->normal);
				result->hit_bits = CCT_SWEEP_BIT_SEGMENT;
				result->distance = dlen;
				return result;
			}
			mathVec3Copy(p, ls[1]);
			mathVec3AddScalar(p, dir, dlen);
			if (Polygon_Contain_Point(polygon, p)) {
				mathVec3Copy(result->hit_plane_v, pp);
				mathVec3Copy(result->hit_plane_n, polygon->normal);
				result->hit_bits = CCT_SWEEP_BIT_SEGMENT;
				result->distance = dlen;
				return result;
			}
		}
		else {
			CCTNum_t v[3];
			if (d[0] == d[2]) {
				mathVec3Copy(p, ls[0]);
			}
			else {
				mathVec3Copy(p, ls[1]);
			}
			mathVec3AddScalar(p, dir, dlen);
			if (Polygon_Contain_Point(polygon, p)) {
				mathVec3Copy(result->hit_plane_v, p);
				mathVec3Copy(result->hit_plane_n, polygon->normal);
				result->hit_bits = CCT_SWEEP_BIT_POINT;
				result->distance = dlen;
				return result;
			}
			mathVec3Sub(v, ls[1], ls[0]);
			mathVec3Cross(v, v, dir);
			if (mathVec3IsZero(v)) {
				return NULL;
			}
		}
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
		return set_intersect(result);
	}
	si.v = mesh->v;
	si.indices = mesh->edge_indices;
	si.indices_cnt = mesh->edge_indices_cnt;
	si.stride = 2;
	p_result = Segment_Sweep_SegmentIndices(ls, dir, &si, result);
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		CCTSweepResult_t result_temp;
		const GeometryPolygon_t* polygon = mesh->polygons + i;
		if (!Segment_Sweep_Plane(ls, dir, polygon->v[polygon->v_indices[0]], polygon->normal, &result_temp)) {
			continue;
		}
		if (result_temp.hit_bits & CCT_SWEEP_BIT_POINT) {
			if (!Polygon_Contain_Point(polygon, result_temp.hit_plane_v)) {
				continue;
			}
		}
		else if (!Polygon_Contain_Point(polygon, ls[0]) && !Polygon_Contain_Point(polygon, ls[1])) {
			continue;
		}
		if (!p_result) {
			p_result = result;
			*p_result = result_temp;
		}
		else {
			merge_result(p_result, &result_temp);
		}
	}
	return p_result;
}

static CCTSweepResult_t* Segment_Sweep_Circle_InSamePlane(const CCTNum_t ls[2][3], const CCTNum_t dir[3], const CCTNum_t circle_o[3], CCTNum_t circle_r, CCTSweepResult_t* result) {
	CCTNum_t lsdir[3], p[3], pco[3];
	mathVec3Sub(lsdir, ls[1], ls[0]);
	mathVec3Normalized(lsdir, lsdir);
	mathPointProjectionLine(circle_o, ls[0], lsdir, p);
	mathVec3Sub(pco, circle_o, p);
	if (mathVec3LenSq(pco) > CCTNum_sq(circle_r)) {
		int hit_ok;
		CCTNum_t d, dot;
		dot = mathVec3Dot(pco, dir);
		if (dot <= CCTNum(0.0)) {
			return NULL;
		}
		d = mathVec3Normalized(pco, pco);
		dot = mathVec3Dot(pco, dir);
		d -= circle_r;
		mathVec3AddScalar(p, pco, d);
		d /= dot;
		if (d <= CCTNum(0.0)) {
			return NULL;
		}
		do {
			CCTNum_t v0[3], v1[3];
			hit_ok = 1;
			mathVec3Copy(v0, ls[0]);
			mathVec3AddScalar(v0, dir, d);
			mathVec3Sub(v0, v0, p);
			if (mathVec3IsZero(v0)) {
				result->peer[0].hit_bits = CCT_SWEEP_BIT_POINT;
				result->peer[0].idx = 0;
				break;
			}
			mathVec3Copy(v1, ls[1]);
			mathVec3AddScalar(v1, dir, d);
			mathVec3Sub(v1, v1, p);
			if (mathVec3IsZero(v1)) {
				result->peer[0].hit_bits = CCT_SWEEP_BIT_POINT;
				result->peer[0].idx = 1;
				break;
			}
			dot = mathVec3Dot(v0, v1);
			if (dot <= CCTNum(0.0)) {
				result->peer[0].hit_bits = CCT_SWEEP_BIT_SEGMENT;
				result->peer[0].idx = 0;
				break;
			}
			hit_ok = 0;
		} while (0);
		if (hit_ok) {
			mathVec3Copy(result->hit_plane_v, p);
			mathVec3Copy(result->hit_plane_n, pco);
			result->hit_bits = CCT_SWEEP_BIT_POINT;
			result->distance = d;
			result->peer[1].hit_bits = 0;
			result->peer[1].idx = 0;
			return result;
		}
	}
	if (Ray_Sweep_Sphere(ls[0], dir, circle_o, circle_r, result)) {
		CCTSweepResult_t result_temp;
		result->peer[0].hit_bits = CCT_SWEEP_BIT_POINT;
		result->peer[0].idx = 0;
		result->peer[1].hit_bits = 0;
		result->peer[1].idx = 0;
		if (!Ray_Sweep_Sphere(ls[1], dir, circle_o, circle_r, &result_temp)) {
			return result;
		}
		result_temp.peer[0].hit_bits = CCT_SWEEP_BIT_POINT;
		result_temp.peer[0].idx = 1;
		result_temp.peer[1].hit_bits = 0;
		result_temp.peer[1].idx = 0;
		merge_result(result, &result_temp);
		return result;
	}
	if (!Ray_Sweep_Sphere(ls[1], dir, circle_o, circle_r, result)) {
		return NULL;
	}
	result->peer[0].hit_bits = CCT_SWEEP_BIT_POINT;
	result->peer[0].idx = 1;
	result->peer[1].hit_bits = 0;
	result->peer[1].idx = 0;
	return result;
}

static CCTSweepResult_t* Segment_Sweep_Circle(const CCTNum_t ls[2][3], const CCTNum_t dir[3], const GeometryCircle_t* circle, CCTSweepResult_t* result) {
	CCTNum_t p[3], d[3], v[3], plane_n[3], cos_theta;
	int res = Segment_Intersect_Plane(ls, circle->o, circle->normal, p, d);
	if (1 == res) {
		CCTNum_t lensq = mathVec3DistanceSq(circle->o, p);
		if (lensq <= CCTNum_sq(circle->radius)) {
			set_intersect(result);
			set_unique_hit_point(result, p);
			return result;
		}
	}
	else if (2 == res) {
		CCTNum_t lensq;
		mathSegmentClosestPointTo(ls, circle->o, p);
		lensq = mathVec3DistanceSq(circle->o, p);
		if (lensq <= CCTNum_sq(circle->radius)) {
			return set_intersect(result);
		}
		cos_theta = mathVec3Dot(circle->normal, dir);
		if (cos_theta < CCT_EPSILON_NEGATE || cos_theta > CCT_EPSILON) {
			return NULL;
		}
		return Segment_Sweep_Circle_InSamePlane(ls, dir, circle->o, circle->radius, result);
	}
	else {
		CCTNum_t dlen, lensq;
		cos_theta = mathVec3Dot(circle->normal, dir);
		if (CCTNum(0.0) == cos_theta) {
			return NULL;
		}
		dlen = d[2] / cos_theta;
		if (dlen < CCTNum(0.0)) {
			return NULL;
		}
		if (d[0] == d[1]) {
			CCTNum_t new_ls[2][3], lensq;
			mathVec3Copy(new_ls[0], ls[0]);
			mathVec3AddScalar(new_ls[0], dir, dlen);
			mathVec3Copy(new_ls[1], ls[1]);
			mathVec3AddScalar(new_ls[1], dir, dlen);
			mathSegmentClosestPointTo((const CCTNum_t(*)[3])new_ls, circle->o, p);
			lensq = mathVec3DistanceSq(circle->o, p);
			if (lensq > CCTNum_sq(circle->radius)) {
				return NULL;
			}
			mathVec3Copy(result->hit_plane_v, p);
			mathVec3Copy(result->hit_plane_n, circle->normal);
			result->hit_bits = CCT_SWEEP_BIT_SEGMENT;
			result->distance = dlen;
			return result;
		}
		if (d[0] == d[2]) {
			mathVec3Copy(p, ls[0]);
		}
		else {
			mathVec3Copy(p, ls[1]);
		}
		mathVec3AddScalar(p, dir, dlen);
		lensq = mathVec3DistanceSq(circle->o, p);
		if (lensq <= CCTNum_sq(circle->radius)) {
			mathVec3Copy(result->hit_plane_v, p);
			mathVec3Copy(result->hit_plane_n, circle->normal);
			result->hit_bits = CCT_SWEEP_BIT_POINT;
			result->distance = dlen;
			return result;
		}
	}
	mathVec3Sub(v, ls[1], ls[0]);
	mathVec3Cross(plane_n, v, dir);
	if (mathVec3IsZero(plane_n)) {
		return NULL;
	}
	/* circle vs sweep_plane */
	mathVec3Normalized(plane_n, plane_n);
	mathVec3Cross(v, circle->normal, plane_n);
	mathVec3Cross(v, circle->normal, v);
	mathVec3Normalized(v, v);
	cos_theta = mathVec3Dot(v, plane_n);
	if (CCTNum(0.0) == cos_theta) {
		return NULL;
	}
	d[0] = mathPointProjectionPlane(circle->o, ls[0], plane_n, NULL);
	d[0] /= cos_theta;
	d[1] = CCTNum_abs(d[0]);
	if (d[1] > circle->radius) {
		return NULL;
	}
	mathVec3Copy(p, circle->o);
	mathVec3AddScalar(p, v, d[0]);
	if (d[1] < circle->radius) {
		CCTNum_t new_ls[2][3];
		CCTNum_t lensq = mathVec3DistanceSq(circle->o, p);
		CCTNum_t half = CCTNum_sqrt(CCTNum_sq(circle->radius) - lensq);
		mathVec3Cross(v, circle->normal, v);
		mathVec3Copy(new_ls[0], p);
		mathVec3AddScalar(new_ls[0], v, half);
		mathVec3Copy(new_ls[1], p);
		mathVec3SubScalar(new_ls[1], v, half);
		return Segment_Sweep_Segment(ls, dir, (const CCTNum_t(*)[3])new_ls, result);
	}
	mathVec3Negate(v, dir);
	if (!Ray_Sweep_Segment(p, v, ls, result)) {
		return NULL;
	}
	set_unique_hit_point(result, p);
	return result;
}

static CCTSweepResult_t* Segment_Sweep_Sphere(const CCTNum_t ls[2][3], const CCTNum_t dir[3], const CCTNum_t center[3], CCTNum_t radius, int check_intersect, CCTSweepResult_t* result) {
	CCTNum_t lsdir[3], N[3];
	if (check_intersect) {
		int res = Sphere_Intersect_Segment(center, radius, ls, result->hit_plane_v);
		if (res) {
			set_intersect(result);
			if (1 == res) {
				result->hit_bits = CCT_SWEEP_BIT_POINT;
			}
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
		return Segment_Sweep_Circle_InSamePlane(ls, dir, circle.o, circle.radius, result);
	}
	if (Ray_Sweep_Sphere(ls[0], dir, center, radius, result)) {
		CCTSweepResult_t result_temp;
		if (!Ray_Sweep_Sphere(ls[1], dir, center, radius, &result_temp)) {
			/* no possible */
			return NULL;
		}
		result->peer[0].idx = 0;
		result_temp.peer[0].idx = 1;
		merge_result(result, &result_temp);
		return result;
	}
	return NULL;
}

static CCTSweepResult_t* SegmentIndices_Sweep_Sphere(const GeometrySegmentIndices_t* si, const CCTNum_t dir[3], const CCTNum_t center[3], CCTNum_t radius, int check_intersect, CCTSweepResult_t* result) {
	unsigned int i;
	CCTSweepResult_t* p_result = NULL;
	for (i = 0; i < si->indices_cnt; ) {
		CCTSweepResult_t result_temp;
		CCTNum_t edge[2][3];
		unsigned int v_indices_idx[2];
		v_indices_idx[0] = i++;
		if (2 == si->stride) {
			v_indices_idx[1] = i++;
		}
		else {
			v_indices_idx[1] = (i >= si->indices_cnt ? 0 : i);
		}
		mathVec3Copy(edge[0], si->v[si->indices[v_indices_idx[0]]]);
		mathVec3Copy(edge[1], si->v[si->indices[v_indices_idx[1]]]);
		if (!Segment_Sweep_Sphere((const CCTNum_t(*)[3])edge, dir, center, radius, check_intersect, &result_temp)) {
			continue;
		}
		if (result_temp.distance <= CCTNum(0.0)) {
			*result = result_temp;
			return result;
		}
		if (result_temp.peer[0].hit_bits & CCT_SWEEP_BIT_POINT) {
			result_temp.peer[0].idx = v_indices_idx[result_temp.peer[0].idx ? 1 : 0];
		}
		else {
			result_temp.peer[0].hit_bits = CCT_SWEEP_BIT_SEGMENT;
			result_temp.peer[0].idx = (i - 1) / si->stride;
		}
		if (!p_result) {
			p_result = result;
			*p_result = result_temp;
		}
		else {
			merge_result(p_result, &result_temp);
		}
	}
	return p_result;
}

static CCTSweepResult_t* Circle_Sweep_Plane(const GeometryCircle_t* circle, const CCTNum_t dir[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTSweepResult_t* result) {
	CCTNum_t v[3], p[3], d, abs_d, cos_theta;
	mathVec3Cross(v, circle->normal, plane_n);
	if (mathVec3IsZero(v)) {
		if (!Ray_Sweep_Plane(circle->o, dir, plane_v, plane_n, result)) {
			return NULL;
		}
		mathVec3Copy(result->hit_plane_v, plane_v);
		mathVec3Copy(result->hit_plane_n, plane_n);
		result->hit_bits = CCT_SWEEP_BIT_FACE;
		return result;
	}
	d = mathPointProjectionPlane(circle->o, plane_v, plane_n, NULL);
	mathVec3Cross(v, circle->normal, v);
	mathVec3Normalized(v, v);
	cos_theta = mathVec3Dot(v, plane_n);
	if (CCTNum(0.0) == cos_theta) {
		return NULL;
	}
	d /= cos_theta;
	abs_d = CCTNum_abs(d);
	if (abs_d < circle->radius) {
		return set_intersect(result);
	}
	mathVec3Copy(p, circle->o);
	if (d > CCTNum(0.0)) {
		mathVec3AddScalar(p, v, circle->radius);
	}
	else {
		mathVec3SubScalar(p, v, circle->radius);
	}
	return Ray_Sweep_Plane(p, dir, plane_v, plane_n, result);
}

static CCTSweepResult_t* Circle_Sweep_Circle(const GeometryCircle_t* c1, const CCTNum_t dir[3], const GeometryCircle_t* c2, CCTSweepResult_t* result) {
	return NULL;
}

static CCTSweepResult_t* Circle_Sweep_Polygon(const GeometryCircle_t* circle, const CCTNum_t dir[3], const GeometryPolygon_t* polygon, CCTSweepResult_t* result) {
	unsigned int i;
	CCTSweepResult_t* p_result;
	CCTNum_t p[3], neg_dir[3];
	if (!Circle_Sweep_Plane(circle, dir, polygon->v[polygon->v_indices[0]], polygon->normal, result)) {
		return NULL;
	}
	if (result->hit_bits & CCT_SWEEP_BIT_POINT) {
		if (Polygon_Contain_Point(polygon, result->hit_plane_v)) {
			return result;
		}
	}
	mathVec3Copy(p, circle->o);
	mathVec3AddScalar(p, dir, result->distance);
	if (Polygon_Contain_Point(polygon, p)) {
		return result;
	}
	p_result = NULL;
	mathVec3Negate(neg_dir, dir);
	for (i = 0; i < polygon->v_indices_cnt; ) {
		CCTSweepResult_t result_temp;
		CCTNum_t edge[2][3];
		mathVec3Copy(edge[0], polygon->v[polygon->v_indices[i++]]);
		mathVec3Copy(edge[0], polygon->v[polygon->v_indices[i >= polygon->v_indices_cnt ? 0 : i]]);
		if (!Segment_Sweep_Circle((const CCTNum_t(*)[3])edge, neg_dir, circle, &result_temp)) {
			continue;
		}
		if (!p_result) {
			p_result = result;
			*p_result = result_temp;
		}
		else {
			merge_result(p_result, &result_temp);
		}
	}
	return p_result;
}

static CCTSweepResult_t* Vertices_Sweep_Plane(const CCTNum_t(*v)[3], const unsigned int* v_indices, unsigned int v_indices_cnt, const CCTNum_t dir[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTSweepResult_t* result) {
	CCTNum_t min_d, dot;
	unsigned int v_indices_idx;
	int res = Vertices_Intersect_Plane(v, v_indices, v_indices_cnt, plane_v, plane_n, &min_d, &v_indices_idx);
	if (res) {
		set_intersect(result);
		if (1 == res && v_indices_idx != -1) {
			set_unique_hit_point(result, v[v_indices[v_indices_idx]]);
		}
		return result;
	}
	dot = mathVec3Dot(dir, plane_n);
	if (CCTNum(0.0) == dot) {
		return NULL;
	}
	min_d /= dot;
	if (min_d < CCTNum(0.0)) {
		return NULL;
	}
	result->distance = min_d;
	mathVec3Copy(result->hit_plane_n, plane_n);
	if (v_indices_idx != -1) {
		mathVec3Copy(result->hit_plane_v, v[v_indices[v_indices_idx]]);
		mathVec3AddScalar(result->hit_plane_v, dir, min_d);
		result->hit_bits = CCT_SWEEP_BIT_POINT;
	}
	else {
		mathVec3Copy(result->hit_plane_v, plane_v);
		result->hit_bits = 0;
	}
	return result;
}

static CCTSweepResult_t* Polygon_Sweep_Polygon(const GeometryPolygon_t* polygon1, const CCTNum_t dir[3], const GeometryPolygon_t* polygon2, CCTSweepResult_t* result) {
	unsigned int i;
	GeometrySegmentIndices_t s1, s2;
	CCTNum_t neg_dir[3];
	CCTSweepResult_t* p_result;
	const CCTNum_t* p2p = polygon2->v[polygon2->v_indices[0]];
	if (!Vertices_Sweep_Plane((const CCTNum_t(*)[3])polygon1->v, polygon1->v_indices, polygon1->v_indices_cnt, dir, p2p, polygon2->normal, result)) {
		return NULL;
	}
	if (result->hit_bits & CCT_SWEEP_BIT_POINT) {
		if (Polygon_Contain_Point(polygon2, result->hit_plane_v)) {
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
		return set_intersect(result);
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
		if (!Ray_Sweep_Plane(p2p, neg_dir, polygon1->v[polygon1->v_indices[0]], polygon1->normal, &result_temp)) {
			continue;
		}
		if (!Polygon_Contain_Point(polygon1, result_temp.hit_plane_v)) {
			continue;
		}
		if (!p_result) {
			p_result = result;
			*p_result = result_temp;
		}
		else {
			merge_result(p_result, &result_temp);
		}
	}
	return p_result;
}

static CCTSweepResult_t* AABB_Sweep_AABB(const CCTNum_t o1[3], const CCTNum_t half1[3], const CCTNum_t dir[3], const CCTNum_t o2[3], const CCTNum_t half2[3], CCTSweepResult_t* result) {
	if (AABB_Intersect_AABB(o1, half1, o2, half2)) {
		return set_intersect(result);
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
				*p_result = result_temp;
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
	if (result->hit_bits & CCT_SWEEP_BIT_POINT) {
		if (Polygon_Contain_Point(polygon, result->hit_plane_v)) {
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
		unsigned int j;
		CCTSweepResult_t result_temp;
		pp = polygon->v[polygon->v_indices[i]];
		for (j = 0; j < mesh->polygons_cnt; ++j) {
			const GeometryPolygon_t* mesh_polygon = mesh->polygons + j;
			if (!Ray_Sweep_Plane(pp, neg_dir, mesh_polygon->v[mesh_polygon->v_indices[0]], mesh_polygon->normal, &result_temp)) {
				continue;
			}
			if (!Polygon_Contain_Point(mesh_polygon, result_temp.hit_plane_v)) {
				continue;
			}
			reverse_result(&result_temp, dir);
			if (!p_result) {
				p_result = result;
				*p_result = result_temp;
			}
			else {
				merge_result(p_result, &result_temp);
			}
		}
	}
	return p_result;
}

static CCTSweepResult_t* ConvexMesh_Sweep_ConvexMesh(const GeometryMesh_t* mesh1, const CCTNum_t dir[3], const GeometryMesh_t* mesh2, int check_intersect, CCTSweepResult_t* result) {
	unsigned int i;
	CCTNum_t neg_dir[3];
	CCTSweepResult_t* p_result;
	GeometrySegmentIndices_t s1, s2;

	if (check_intersect && ConvexMesh_Intersect_ConvexMesh(mesh1, mesh2)) {
		return set_intersect(result);
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
		unsigned int j;
		for (j = 0; j < mesh2->polygons_cnt; ++j) {
			const GeometryPolygon_t* polygon = mesh2->polygons + j;
			if (!Ray_Sweep_Plane(pp, dir, polygon->v[polygon->v_indices[0]], polygon->normal, &result_temp)) {
				continue;
			}
			if (!Polygon_Contain_Point(polygon, result_temp.hit_plane_v)) {
				continue;
			}
			if (!p_result) {
				p_result = result;
				*p_result = result_temp;
			}
			else {
				merge_result(p_result, &result_temp);
			}
		}
	}
	mathVec3Negate(neg_dir, dir);
	for (i = 0; i < mesh2->v_indices_cnt; ++i) {
		CCTSweepResult_t result_temp;
		const CCTNum_t* pp = mesh2->v[mesh2->v_indices[i]];
		unsigned int j;
		for (j = 0; j < mesh1->polygons_cnt; ++j) {
			const GeometryPolygon_t* polygon = mesh1->polygons + j;
			if (!Ray_Sweep_Plane(pp, neg_dir, polygon->v[polygon->v_indices[0]], polygon->normal, &result_temp)) {
				continue;
			}
			if (!Polygon_Contain_Point(polygon, result_temp.hit_plane_v)) {
				continue;
			}
			if (!p_result) {
				p_result = result;
				*p_result = result_temp;
			}
			else {
				merge_result(p_result, &result_temp);
			}
		}
	}
	return p_result;
}

static CCTSweepResult_t* OBB_Sweep_OBB(const GeometryOBB_t* obb1, const CCTNum_t dir[3], const GeometryOBB_t* obb2, CCTSweepResult_t* result) {
	GeometryBoxMesh_t mesh1, mesh2;
	CCTNum_t v1[8][3], v2[8][3];
	if (OBB_Intersect_OBB(obb1, obb2)) {
		return set_intersect(result);
	}
	mathOBBVertices(obb1, v1);
	mathBoxMesh((const CCTNum_t(*)[3])v1, (const CCTNum_t(*)[3])obb1->axis, &mesh1);
	mathOBBVertices(obb2, v2);
	mathBoxMesh((const CCTNum_t(*)[3])v2, (const CCTNum_t(*)[3])obb2->axis, &mesh2);
	return ConvexMesh_Sweep_ConvexMesh(&mesh1.mesh, dir, &mesh2.mesh, 0, result);
}

static CCTSweepResult_t* Sphere_Sweep_Plane(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t dir[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTSweepResult_t* result) {
	CCTNum_t dn, dn_abs, cos_theta;
	dn = mathPointProjectionPlane(o, plane_v, plane_n, NULL);
	dn_abs = CCTNum_abs(dn);
	if (dn_abs < radius) {
		return set_intersect(result);
	}
	if (dn_abs == radius) {
		set_intersect(result);
		set_unique_hit_point(result, o);
		mathVec3AddScalar(result->hit_plane_v, plane_n, dn);
		return result;
	}
	cos_theta = mathVec3Dot(plane_n, dir);
	if (CCTNum(0.0) == cos_theta) {
		return NULL;
	}
	mathVec3Copy(result->hit_plane_v, o);
	if (dn >= CCTNum(0.0)) {
		dn -= radius;
		mathVec3AddScalar(result->hit_plane_v, plane_n, radius);
	}
	else {
		dn += radius;
		mathVec3SubScalar(result->hit_plane_v, plane_n, radius);
	}
	dn /= cos_theta;
	if (dn < CCTNum(0.0)) {
		return NULL;
	}
	mathVec3AddScalar(result->hit_plane_v, dir, dn);
	result->hit_bits = CCT_SWEEP_BIT_POINT;
	mathVec3Copy(result->hit_plane_n, plane_n);
	result->distance = dn;
	result->peer[0].hit_bits = 0;
	result->peer[0].idx = 0;
	result->peer[1].hit_bits = CCT_SWEEP_BIT_FACE;
	result->peer[1].idx = 0;
	return result;
}

static CCTSweepResult_t* Sphere_Sweep_Polygon(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t dir[3], const GeometryPolygon_t* polygon, CCTSweepResult_t* result) {
	CCTNum_t neg_dir[3];
	GeometrySegmentIndices_t si;

	if (!Sphere_Sweep_Plane(o, radius, dir, polygon->v[polygon->v_indices[0]], polygon->normal, result)) {
		return NULL;
	}
	if (result->hit_bits & CCT_SWEEP_BIT_POINT) {
		if (Polygon_Contain_Point(polygon, result->hit_plane_v)) {
			return result;
		}
	}
	else {
		CCTNum_t p[3];
		mathPointProjectionPlane(o, polygon->v[polygon->v_indices[0]], polygon->normal, p);
		if (Polygon_Contain_Point(polygon, p)) {
			return result;
		}
	}
	si.v = polygon->v;
	si.indices = polygon->v_indices;
	si.indices_cnt = polygon->v_indices_cnt;
	si.stride = 1;
	mathVec3Negate(neg_dir, dir);
	if (!SegmentIndices_Sweep_Sphere(&si, neg_dir, o, radius, 1, result)) {
		return NULL;
	}
	reverse_result(result, dir);
	return result;
}

static CCTSweepResult_t* Sphere_Sweep_Sphere(const CCTNum_t o1[3], CCTNum_t r1, const CCTNum_t dir[3], const CCTNum_t o2[3], CCTNum_t r2, CCTSweepResult_t* result) {
	if (!Ray_Sweep_Sphere(o1, dir, o2, r1 + r2, result)) {
		return NULL;
	}
	mathVec3Copy(result->hit_plane_v, o1);
	mathVec3AddScalar(result->hit_plane_v, dir, result->distance);
	mathVec3SubScalar(result->hit_plane_v, result->hit_plane_n, r1);
	result->peer[0].hit_bits = 0;
	return result;
}

static CCTSweepResult_t* Sphere_Sweep_ConvexMesh(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t dir[3], const GeometryMesh_t* mesh, int check_intersect, CCTSweepResult_t* result) {
	unsigned int i;
	CCTSweepResult_t* p_result;
	CCTNum_t neg_dir[3];
	GeometrySegmentIndices_t si;

	if (check_intersect && Sphere_Intersect_ConvexMesh(o, radius, mesh)) {
		set_intersect(result);
		return result;
	}
	si.v = mesh->v;
	si.indices = mesh->edge_indices;
	si.indices_cnt = mesh->edge_indices_cnt;
	si.stride = 2;
	mathVec3Negate(neg_dir, dir);
	p_result = SegmentIndices_Sweep_Sphere(&si, neg_dir, o, radius, 0, result);
	if (p_result) {
		reverse_result(p_result, dir);
	}
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		CCTSweepResult_t result_temp;
		const GeometryPolygon_t* polygon = mesh->polygons + i;
		if (!Sphere_Sweep_Plane(o, radius, dir, polygon->v[polygon->v_indices[0]], polygon->normal, &result_temp)) {
			continue;
		}
		if (!(result_temp.hit_bits & CCT_SWEEP_BIT_POINT)) {
			continue;
		}
		if (!Polygon_Contain_Point(polygon, result_temp.hit_plane_v)) {
			continue;
		}
		result_temp.peer[1].idx = i;
		if (!p_result) {
			p_result = result;
			*p_result = result_temp;
		}
		else {
			merge_result(p_result, &result_temp);
		}
	}
	return p_result;
}

static CCTSweepResult_t* Sphere_Sweep_OBB(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t dir[3], const GeometryOBB_t* obb, CCTSweepResult_t* result) {
	GeometryBoxMesh_t mesh;
	CCTNum_t v[8][3];
	if (Sphere_Intersect_OBB(o, radius, obb)) {
		return set_intersect(result);
	}
	mathOBBVertices(obb, v);
	mathBoxMesh((const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])obb->axis, &mesh);
	return Sphere_Sweep_ConvexMesh(o, radius, dir, &mesh.mesh, 0, result);
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
				GeometryBoxMesh_t two_mesh;
				CCTNum_t v[8][3];
				mathAABBVertices(two->aabb->o, two->aabb->half, v);
				mathBoxMesh((const CCTNum_t(*)[3])v, AABB_Axis, &two_mesh);
				result = Ray_Sweep_ConvexMesh(one->point, dir, &two_mesh.mesh, 1, result);
				break;
			}
			case GEOMETRY_BODY_OBB:
			{
				GeometryBoxMesh_t two_mesh;
				CCTNum_t v[8][3];
				mathOBBVertices(two->obb, v);
				mathBoxMesh((const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])two->obb->axis, &two_mesh);
				result = Ray_Sweep_ConvexMesh(one->point, dir, &two_mesh.mesh, 1, result);
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
				GeometryBoxMesh_t two_mesh;
				CCTNum_t v[8][3];
				mathOBBVertices(two->obb, v);
				mathBoxMesh((const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])two->obb->axis, &two_mesh);
				result = Segment_Sweep_ConvexMesh(one_segment_v, dir, &two_mesh.mesh, result);
				break;
			}
			case GEOMETRY_BODY_AABB:
			{
				GeometryBoxMesh_t two_mesh;
				CCTNum_t v[8][3];
				mathAABBVertices(two->aabb->o, two->aabb->half, v);
				mathBoxMesh((const CCTNum_t(*)[3])v, AABB_Axis, &two_mesh);
				result = Segment_Sweep_ConvexMesh(one_segment_v, dir, &two_mesh.mesh, result);
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
				GeometryBoxMesh_t one_mesh;
				CCTNum_t neg_dir[3], v[8][3];
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				mathAABBVertices(one->aabb->o, one->aabb->half, v);
				mathBoxMesh((const CCTNum_t(*)[3])v, AABB_Axis, &one_mesh);
				result = Segment_Sweep_ConvexMesh((const CCTNum_t(*)[3])two->segment->v, neg_dir, &one_mesh.mesh, result);
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
				result = ConvexMesh_Sweep_ConvexMesh(&one_mesh.mesh, dir, two->mesh, 1, result);
				break;
			}
		}
	}
	else if (GEOMETRY_BODY_OBB == one->type) {
		switch (two->type) {
			case GEOMETRY_BODY_SEGMENT:
			{
				GeometryBoxMesh_t one_mesh;
				CCTNum_t neg_dir[3], v[8][3];
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				mathOBBVertices(one->obb, v);
				mathBoxMesh((const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])one->obb->axis, &one_mesh);
				result = Segment_Sweep_ConvexMesh((const CCTNum_t(*)[3])two->segment->v, neg_dir, &one_mesh.mesh, result);
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
				result = ConvexMesh_Sweep_ConvexMesh(&one_mesh.mesh, dir, two->mesh, 1, result);
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
				result = Sphere_Sweep_ConvexMesh(one->sphere->o, one->sphere->radius, dir, two->mesh, 1, result);
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
				result = Sphere_Sweep_ConvexMesh(two->sphere->o, two->sphere->radius, neg_dir, one->mesh, 1, result);
				break;
			}
			case GEOMETRY_BODY_OBB:
			{
				GeometryBoxMesh_t two_mesh;
				CCTNum_t v[8][3];
				mathOBBVertices(two->obb, v);
				mathBoxMesh((const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])two->obb->axis, &two_mesh);
				result = ConvexMesh_Sweep_ConvexMesh(one->mesh, dir, &two_mesh.mesh, 1, result);
				break;
			}
			case GEOMETRY_BODY_AABB:
			{
				GeometryBoxMesh_t two_mesh;
				CCTNum_t v[8][3];
				mathAABBVertices(two->aabb->o, two->aabb->half, v);
				mathBoxMesh((const CCTNum_t(*)[3])v, AABB_Axis, &two_mesh);
				result = ConvexMesh_Sweep_ConvexMesh(one->mesh, dir, &two_mesh.mesh, 1, result);
				break;
			}
			case GEOMETRY_BODY_POLYGON:
			{
				result = ConvexMesh_Sweep_Polygon(one->mesh, dir, two->polygon, result);
				break;
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				result = ConvexMesh_Sweep_ConvexMesh(one->mesh, dir, two->mesh, 1, result);
				break;
			}
		}
	}
	if (!result) {
		return NULL;
	}
	if (flag_neg_dir) {
		reverse_result(result, dir);
	}
	return result;
}

#ifdef __cplusplus
}
#endif
