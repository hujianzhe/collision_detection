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
#include "../inc/geometry_api.h"
#include <math.h>
#include <stddef.h>

extern const CCTNum_t AABB_Axis[3][3];
extern const CCTNum_t AABB_Plane_Normal[6][3];
extern const unsigned int Segment_Indices_Default[2];

extern int Segment_Contain_Point(const CCTNum_t ls[2][3], const CCTNum_t p[3]);
extern int Segment_Intersect_Plane(const CCTNum_t ls[2][3], const CCTNum_t plane_v[3], const CCTNum_t plane_normal[3], CCTNum_t p[3], CCTNum_t d[3]);
extern int Segment_Intersect_Polygon(const CCTNum_t ls[2][3], const GeometryPolygon_t* polygon, int* ret_plane_side);
extern int Segment_Intersect_ConvexMesh(const CCTNum_t ls[2][3], const GeometryMesh_t* mesh);
extern int Sphere_Intersect_Segment(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t ls[2][3]);
extern int Sphere_Intersect_Plane(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t plane_v[3], const CCTNum_t plane_normal[3], CCTNum_t new_o[3], CCTNum_t* new_r);
extern int Sphere_Intersect_Polygon(const CCTNum_t o[3], CCTNum_t radius, const GeometryPolygon_t* polygon, int* ret_plane_side);
extern int OBB_Intersect_OBB(const GeometryOBB_t* obb0, const GeometryOBB_t* obb1);
extern int Sphere_Intersect_OBB(const CCTNum_t o[3], CCTNum_t radius, const GeometryOBB_t* obb);
extern int Sphere_Intersect_ConvexMesh(const CCTNum_t o[3], CCTNum_t radius, const GeometryMesh_t* mesh);
extern int ConvexMesh_Contain_Point(const GeometryMesh_t* mesh, const CCTNum_t p[3]);
extern int ConvexMesh_Intersect_ConvexMesh(const GeometryMesh_t* mesh1, const GeometryMesh_t* mesh2);
extern int Polygon_Contain_Point(const GeometryPolygon_t* polygon, const CCTNum_t p[3]);
extern int Polygon_Intersect_Polygon(const GeometryPolygon_t* polygon1, const GeometryPolygon_t* polygon2, int* ret_plane_side);
extern int ConvexMesh_Intersect_Polygon(const GeometryMesh_t* mesh, const GeometryPolygon_t* polygon, int* ret_plane_side);

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

static void set_intersect(CCTSweepResult_t* result) {
	mathVec3Set(result->hit_plane_n, CCTNums_3(0.0, 0.0, 0.0));
	result->distance = CCTNum(0.0);
	result->hit_bits = 0;
	result->peer[0].hit_bits = 0;
	result->peer[0].idx = 0;
	result->peer[1].hit_bits = 0;
	result->peer[1].idx = 0;
}

static void reverse_result(CCTSweepResult_t* result, const CCTNum_t dir[3]) {
	CCTSweepHitInfo_t hit_info_0 = result->peer[0];
	result->peer[0] = result->peer[1];
	result->peer[1] = hit_info_0;
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

static void sweep_mesh_convert_from_segment(GeometryMesh_t* mesh, const CCTNum_t ls[2][3]) {
	mesh->v = (CCTNum_t(*)[3])ls;
	mesh->v_indices = Segment_Indices_Default;
	mesh->v_indices_cnt = 2;
	mesh->edge_indices = Segment_Indices_Default;
	mesh->edge_indices_cnt = 2;
	mesh->edge_stride = 2;
	mesh->is_convex = 1;
	mesh->polygons = NULL;
	mesh->polygons_cnt = 0;
}

static void sweep_mesh_convert_from_polygon(GeometryMesh_t* mesh, const GeometryPolygon_t* polygon) {
	mesh->v = polygon->v;
	mesh->v_indices = polygon->v_indices;
	mesh->v_indices_cnt = polygon->v_indices_cnt;
	mesh->edge_indices = polygon->v_indices;
	mesh->edge_indices_cnt = polygon->v_indices_cnt;
	mesh->edge_stride = 1;
	mesh->is_convex = polygon->is_convex;
	mesh->polygons = (GeometryPolygon_t*)polygon;
	mesh->polygons_cnt = 1;
}

static unsigned int polygon_find_edge_idx(const GeometryPolygon_t* polygon, const CCTNum_t p[3]) {
	CCTNum_t edge[2][3];
	unsigned int i, v_idx[2];
	for (i = 1; i < polygon->v_indices_cnt; ++i) {
		v_idx[0] = polygon->v_indices[i - 1];
		v_idx[1] = polygon->v_indices[i];
		mathVec3Copy(edge[0], polygon->v[v_idx[0]]);
		mathVec3Copy(edge[1], polygon->v[v_idx[1]]);
		if (Segment_Contain_Point((const CCTNum_t(*)[3])edge, p)) {
			return i - 1;
		}
	}
	v_idx[0] = polygon->v_indices[--i];
	v_idx[1] = polygon->v_indices[0];
	mathVec3Copy(edge[0], polygon->v[v_idx[0]]);
	mathVec3Copy(edge[1], polygon->v[v_idx[1]]);
	return Segment_Contain_Point((const CCTNum_t(*)[3])edge, p) ? i : -1;
}

static unsigned int polygon_find_v_idx(const GeometryPolygon_t* polygon, const CCTNum_t p[3]) {
	unsigned int i;
	for (i = 0; i < polygon->v_indices_cnt; ++i) {
		unsigned int v_idx = polygon->v_indices[i];
		if (mathVec3Equal(polygon->v[v_idx], p)) {
			return v_idx;
		}
	}
	return -1;
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

static CCTSweepResult_t* Ray_Sweep_MeshSegment(const CCTNum_t o[3], const CCTNum_t dir[3], const GeometryMesh_t* mesh, CCTSweepResult_t* result) {
	unsigned int i;
	CCTSweepResult_t* p_result = NULL;
	for (i = 0; i < mesh->edge_indices_cnt; ) {
		CCTSweepResult_t result_temp;
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
		if (!Ray_Sweep_Segment(o, dir, (const CCTNum_t(*)[3])edge, &result_temp)) {
			continue;
		}
		if (result_temp.distance <= CCTNum(0.0)) {
			*result = result_temp;
			return result;
		}
		if (!p_result) {
			p_result = result;
			*result = result_temp;
		}
		else if (result_temp.distance < result->distance) {
			*result = result_temp;
		}
		else {
			continue;
		}
		if (result_temp.peer[1].hit_bits & CCT_SWEEP_BIT_POINT) {
			result->peer[1].idx = v_idx[result_temp.peer[1].idx ? 1 : 0];
		}
		else {
			result->peer[1].idx = (i - 1) / mesh->edge_stride;
		}
	}
	return p_result;
}

static CCTSweepResult_t* Ray_Sweep_Plane(const CCTNum_t o[3], const CCTNum_t dir[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTSweepResult_t* result) {
	CCTNum_t d, cos_theta;
	d = mathPointProjectionPlane(o, plane_v, plane_n);
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
	GeometryMesh_t mesh;
	if (!Ray_Sweep_Plane(o, dir, polygon->v[polygon->v_indices[0]], polygon->normal, result)) {
		return NULL;
	}
	if (Polygon_Contain_Point(polygon, result->hit_plane_v)) {
		unsigned int idx = polygon_find_v_idx(polygon, result->hit_plane_v);
		if (idx != -1) {
			result->peer[1].hit_bits = CCT_SWEEP_BIT_POINT;
			result->peer[1].idx = idx;
			return result;
		}
		idx = polygon_find_edge_idx(polygon, result->hit_plane_v);
		if (idx != -1) {
			result->peer[1].hit_bits = CCT_SWEEP_BIT_SEGMENT;
			result->peer[1].idx = idx;
			return result;
		}
		return result;
	}
	if (result->distance > CCTNum(0.0)) {
		return NULL;
	}
	dot = mathVec3Dot(dir, polygon->normal);
	if (dot < CCT_EPSILON_NEGATE || dot > CCT_EPSILON) {
		return NULL;
	}
	sweep_mesh_convert_from_polygon(&mesh, polygon);
	return Ray_Sweep_MeshSegment(o, dir, &mesh, result);
}

static CCTSweepResult_t* Ray_Sweep_Sphere(const CCTNum_t o[3], const CCTNum_t dir[3], const CCTNum_t sp_o[3], CCTNum_t sp_radius, CCTSweepResult_t* result) {
	CCTNum_t d_sq, oc_lensq, dir_d;
	CCTNum_t oc[3];
	CCTNum_t radius_sq = CCTNum_sq(sp_radius);
	mathVec3Sub(oc, sp_o, o);
	oc_lensq = mathVec3LenSq(oc);
	if (oc_lensq <= radius_sq) {
		set_intersect(result);
		return result;
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
	result->peer[1].hit_bits = CCT_SWEEP_BIT_SPHERE;
	result->peer[1].idx = 0;
	return result;
}

static CCTSweepResult_t* Ray_Sweep_ConvexMesh(const CCTNum_t o[3], const CCTNum_t dir[3], const GeometryMesh_t* mesh, CCTSweepResult_t* result) {
	unsigned int i;
	CCTSweepResult_t* p_result;
	if (ConvexMesh_Contain_Point(mesh, o)) {
		set_intersect(result);
		return result;
	}
	p_result = NULL;
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		CCTSweepResult_t result_temp;
		const GeometryPolygon_t* polygon = mesh->polygons + i;
		if (!Ray_Sweep_Plane(o, dir, polygon->v[polygon->v_indices[0]], polygon->normal, &result_temp)) {
			continue;
		}
		if (p_result && result_temp.distance >= result->distance) {
			continue;
		}
		if (!Polygon_Contain_Point(polygon, result_temp.hit_plane_v)) {
			continue;
		}
		p_result = result;
		*result = result_temp;
		result->peer[1].idx = i;
	}
	if (p_result) {
		const GeometryPolygon_t* polygon = mesh->polygons + result->peer[1].idx;
		unsigned int idx = polygon_find_v_idx(polygon, result->hit_plane_v);
		if (idx != -1) {
			result->peer[1].hit_bits = CCT_SWEEP_BIT_POINT;
			result->peer[1].idx = idx;
			return result;
		}
		idx = polygon_find_edge_idx(polygon, result->hit_plane_v);
		if (idx != -1) {
			result->peer[1].hit_bits = CCT_SWEEP_BIT_SEGMENT;
			result->peer[1].idx = idx;
			return result;
		}
		return result;
	}
	return Ray_Sweep_MeshSegment(o, dir, mesh, result);
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
			CCTNum_t l[3], r[3];
			unsigned int closest_ls1_indice, closest_ls2_indice;
			CCTNum_t d;
			const CCTNum_t* intersect_p = NULL;

			mathVec3Sub(l, ls1[0], ls2[0]);
			mathVec3Sub(r, ls1[1], ls2[0]);
			d = mathVec3Dot(l, r);
			if (d <= CCT_EPSILON) {
				intersect_p = ls2[0];
			}
			mathVec3Sub(l, ls1[0], ls2[1]);
			mathVec3Sub(r, ls1[1], ls2[1]);
			d = mathVec3Dot(l, r);
			if (d <= CCT_EPSILON) {
				if (intersect_p && !mathVec3Equal(intersect_p, ls2[1])) {
					set_intersect(result);
					return result;
				}
				intersect_p = ls2[1];
			}
			mathVec3Sub(l, ls2[0], ls1[0]);
			mathVec3Sub(r, ls2[1], ls1[0]);
			d = mathVec3Dot(l, r);
			if (d <= CCT_EPSILON) {
				if (intersect_p && !mathVec3Equal(intersect_p, ls1[0])) {
					set_intersect(result);
					return result;
				}
				intersect_p = ls1[0];
			}
			mathVec3Sub(l, ls2[0], ls1[1]);
			mathVec3Sub(r, ls2[1], ls1[1]);
			d = mathVec3Dot(l, r);
			if (d <= CCT_EPSILON) {
				if (intersect_p && !mathVec3Equal(intersect_p, ls1[1])) {
					set_intersect(result);
					return result;
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
			result->peer[0].hit_bits = CCT_SWEEP_BIT_POINT;
			result->peer[0].idx = closest_ls1_indice;
			result->peer[1].hit_bits = CCT_SWEEP_BIT_POINT;
			result->peer[1].idx = closest_ls2_indice;
			return result;
		}
		else {
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
			for (i = 0; i < 2; ++i) {
				int j;
				CCTNum_t p[3], vp[3];
				mathVec3Copy(p, ls1[i]);
				mathVec3AddScalar(p, dir, d);
				for (j = 0; j < 2; ++j) {
					int unique_point = 0;
					if (!mathVec3Equal(ls2[j], p)) {
						continue;
					}
					cos_theta = mathVec3Dot(ls1_dir, ls2_dir);
					if (1 == i + j) {
						if (cos_theta > CCTNum(0.0)) {
							unique_point = 1;
						}
					}
					else if (cos_theta < CCTNum(0.0)) {
						unique_point = 1;
					}
					if (unique_point) {
						result->hit_bits = CCT_SWEEP_BIT_POINT;
						result->peer[0].hit_bits = CCT_SWEEP_BIT_POINT;
						result->peer[0].idx = i;
						result->peer[1].hit_bits = CCT_SWEEP_BIT_POINT;
						result->peer[1].idx = j;
					}
					else {
						result->hit_bits = CCT_SWEEP_BIT_SEGMENT;
						result->peer[0].hit_bits = CCT_SWEEP_BIT_SEGMENT;
						result->peer[0].idx = 0;
						result->peer[1].hit_bits = CCT_SWEEP_BIT_SEGMENT;
						result->peer[1].idx = 0;
					}
					p_result = result;
					mathVec3Copy(result->hit_plane_v, p);
					break;
				}
				if (p_result) {
					break;
				}
				mathVec3Sub(v, ls2[0], p);
				mathVec3Sub(vp, ls2[1], p);
				cos_theta = mathVec3Dot(v, vp);
				if (cos_theta > CCT_EPSILON) {
					continue;
				}
				p_result = result;
				mathVec3Copy(result->hit_plane_v, ls2[i]);
				result->hit_bits = CCT_SWEEP_BIT_SEGMENT;
				result->peer[0].hit_bits = CCT_SWEEP_BIT_SEGMENT;
				result->peer[0].idx = 0;
				result->peer[1].hit_bits = CCT_SWEEP_BIT_SEGMENT;
				result->peer[1].idx = 0;
				break;
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
					p_result = result;
					mathVec3Copy(result->hit_plane_v, ls2[i]);
					result->hit_bits = CCT_SWEEP_BIT_SEGMENT;
					result->peer[0].hit_bits = CCT_SWEEP_BIT_SEGMENT;
					result->peer[0].idx = 0;
					result->peer[1].hit_bits = CCT_SWEEP_BIT_SEGMENT;
					result->peer[1].idx = 0;
					break;
				}
			}
			if (!p_result) {
				return NULL;
			}
			mathVec3Copy(result->hit_plane_n, N);
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
			if (mathVec3Equal(ls1[0], p)) {
				result->peer[0].hit_bits = CCT_SWEEP_BIT_POINT;
				result->peer[0].idx = 0;
			}
			else if (mathVec3Equal(ls1[1], p)) {
				result->peer[0].hit_bits = CCT_SWEEP_BIT_POINT;
				result->peer[0].idx = 1;
			}
			else {
				CCTNum_t vr[3];
				mathVec3Sub(vr, ls1[0], p);
				mathVec3Sub(vp, ls1[1], p);
				cos_theta = mathVec3Dot(vr, vp);
				if (cos_theta > CCT_EPSILON) {
					return NULL;
				}
				result->peer[0].hit_bits = CCT_SWEEP_BIT_SEGMENT;
				result->peer[0].idx = 0;
			}
			mathVec3Copy(result->hit_plane_v, v);
			mathVec3Copy(result->hit_plane_n, N);
			result->hit_bits = CCT_SWEEP_BIT_POINT;
			result->distance = d;
			if (mathVec3Equal(v, ls2[0])) {
				result->peer[1].hit_bits = CCT_SWEEP_BIT_POINT;
				result->peer[1].idx = 0;
			}
			else if (mathVec3Equal(v, ls2[1])) {
				result->peer[1].hit_bits = CCT_SWEEP_BIT_POINT;
				result->peer[1].idx = 1;
			}
			else {
				result->peer[1].hit_bits = CCT_SWEEP_BIT_SEGMENT;
				result->peer[1].idx = 0;
			}
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
						result->peer[0].hit_bits = CCT_SWEEP_BIT_POINT;
						result->peer[0].idx = 0;
					}
					else {
						result->peer[0].hit_bits = CCT_SWEEP_BIT_POINT;
						result->peer[0].idx = 1;
					}
					mathVec3Copy(result->hit_plane_v, p);
					mathVec3Copy(result->hit_plane_n, hn);
					result->hit_bits = CCT_SWEEP_BIT_POINT;
					result->distance = d;
					if (mathVec3Equal(p, ls2[0])) {
						result->peer[1].hit_bits = CCT_SWEEP_BIT_POINT;
						result->peer[1].idx = 0;
					}
					else if (mathVec3Equal(p, ls2[1])) {
						result->peer[1].hit_bits = CCT_SWEEP_BIT_POINT;
						result->peer[1].idx = 1;
					}
					else {
						result->peer[1].hit_bits = CCT_SWEEP_BIT_SEGMENT;
						result->peer[1].idx = 0;
					}
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
				p_result = result;
				mathVec3Copy(result->hit_plane_v, p);
				mathVec3Copy(result->hit_plane_n, hn);
				result->hit_bits = CCT_SWEEP_BIT_POINT;
				result->distance = hn_len;
				result->peer[0].hit_bits = CCT_SWEEP_BIT_POINT;
				result->peer[0].idx = 0;
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
				p_result = result;
				mathVec3Copy(result->hit_plane_v, p);
				mathVec3Copy(result->hit_plane_n, hn);
				result->hit_bits = CCT_SWEEP_BIT_POINT;
				result->distance = hn_len;
				result->peer[0].hit_bits = CCT_SWEEP_BIT_POINT;
				result->peer[0].idx = 1;
			} while (0);
			if (p_result) {
				if (mathVec3Equal(result->hit_plane_v, ls2[0])) {
					result->peer[1].hit_bits = CCT_SWEEP_BIT_POINT;
					result->peer[1].idx = 0;
				}
				else if (mathVec3Equal(result->hit_plane_v, ls2[1])) {
					result->peer[1].hit_bits = CCT_SWEEP_BIT_POINT;
					result->peer[1].idx = 1;
				}
				else {
					result->peer[1].hit_bits = CCT_SWEEP_BIT_SEGMENT;
					result->peer[1].idx = 0;
				}
			}
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
				if (mathVec3Equal(ls1[0], p)) {
					result->peer[0].hit_bits = CCT_SWEEP_BIT_POINT;
					result->peer[0].idx = 0;
				}
				else if (mathVec3Equal(ls1[1], p)) {
					result->peer[0].hit_bits = CCT_SWEEP_BIT_POINT;
					result->peer[0].idx = 1;
				}
				else {
					mathVec3Sub(v, ls1[0], p);
					mathVec3Sub(vp, ls1[1], p);
					cos_theta = mathVec3Dot(v, vp);
					if (cos_theta > CCT_EPSILON) {
						continue;
					}
					result->peer[0].hit_bits = CCT_SWEEP_BIT_SEGMENT;
					result->peer[0].idx = 0;
				}
				p_result = result;
				mathVec3Copy(result->hit_plane_v, ls2[i]);
				mathVec3Copy(result->hit_plane_n, hn);
				result->hit_bits = CCT_SWEEP_BIT_POINT;
				result->distance = hn_len;
				result->peer[1].hit_bits = CCT_SWEEP_BIT_POINT;
				result->peer[1].idx = i;
			}
			return p_result;
		}
	}
}

static unsigned int geometry_indices_find_face_index(const GeometryPolygon_t* faces, unsigned int faces_cnt, const unsigned int* v_idx, unsigned int v_idx_cnt) {
	unsigned int i;
	for (i = 0; i < faces_cnt; ++i) {
		const GeometryPolygon_t* face = faces + i;
		unsigned int j;
		for (j = 0; j < v_idx_cnt; ++j) {
			unsigned int k;
			for (k = 0; k < face->v_indices_cnt; ++k) {
				if (face->v_indices[k] == v_idx[j]) {
					break;
				}
			}
			if (k >= face->v_indices_cnt) {
				break;
			}
		}
		if (j >= v_idx_cnt) {
			return i;
		}
	}
	return -1;
}

static int merge_mesh_hit_info(CCTSweepHitInfo_t* dst_info, const CCTSweepHitInfo_t* src_info, const GeometryMesh_t* mesh, int* ret_new_hit_bits) {
	unsigned int idx, v_idx[3];
	int new_hit_bits = 0;
	if (!ret_new_hit_bits) {
		ret_new_hit_bits = &new_hit_bits;
	}
	if ((dst_info->hit_bits & CCT_SWEEP_BIT_POINT) && (src_info->hit_bits & CCT_SWEEP_BIT_POINT)) {
		if (dst_info->idx == src_info->idx) {
			return 0;
		}
		idx = mathFindEdgeIndexByTwoVertexIndex(mesh->edge_indices, mesh->edge_indices_cnt, mesh->edge_stride, dst_info->idx, src_info->idx);
		if (idx != -1) {
			dst_info->hit_bits = CCT_SWEEP_BIT_SEGMENT;
			dst_info->idx = idx;
			*ret_new_hit_bits = CCT_SWEEP_BIT_SEGMENT;
			return 1;
		}
		v_idx[0] = dst_info->idx;
		v_idx[1] = src_info->idx;
		idx = geometry_indices_find_face_index(mesh->polygons, mesh->polygons_cnt, v_idx, 2);
		if (idx != -1) {
			dst_info->hit_bits = CCT_SWEEP_BIT_FACE;
			dst_info->idx = idx;
			*ret_new_hit_bits = CCT_SWEEP_BIT_FACE;
			return 1;
		}
		dst_info->hit_bits = 0;
		dst_info->idx = 0;
		*ret_new_hit_bits = 0;
		return 1;
	}
	if ((dst_info->hit_bits & CCT_SWEEP_BIT_SEGMENT) && (src_info->hit_bits & CCT_SWEEP_BIT_SEGMENT)) {
		if (dst_info->idx == src_info->idx) {
			return 0;
		}
		idx = mesh->edge_stride * dst_info->idx;
		v_idx[0] = mesh->edge_indices[idx++];
		v_idx[1] = mesh->edge_indices[idx >= mesh->edge_indices_cnt ? 0 : idx];
		idx = mesh->edge_stride * src_info->idx;
		v_idx[2] = mesh->edge_indices[idx++];
		if (v_idx[2] == v_idx[0] || v_idx[2] == v_idx[1]) {
			v_idx[2] = mesh->edge_indices[idx >= mesh->edge_indices_cnt ? 0 : idx];
		}
		idx = geometry_indices_find_face_index(mesh->polygons, mesh->polygons_cnt, v_idx, 3);
		if (idx != -1) {
			dst_info->hit_bits = CCT_SWEEP_BIT_FACE;
			dst_info->idx = idx;
			*ret_new_hit_bits = CCT_SWEEP_BIT_FACE;
			return 1;
		}
		dst_info->hit_bits = 0;
		dst_info->idx = 0;
		*ret_new_hit_bits = 0;
		return 1;
	}
	if ((dst_info->hit_bits & CCT_SWEEP_BIT_POINT) && (src_info->hit_bits & CCT_SWEEP_BIT_SEGMENT)) {
		CCTNum_t edge[2][3];
		idx = src_info->idx * mesh->edge_stride;
		mathVec3Copy(edge[0], mesh->v[mesh->edge_indices[idx++]]);
		mathVec3Copy(edge[1], mesh->v[mesh->edge_indices[idx >= mesh->edge_indices_cnt ? 0 : idx]]);
		if (Segment_Contain_Point((const CCTNum_t(*)[3])edge, mesh->v[dst_info->idx])) {
			*dst_info = *src_info;
			return 1;
		}
		idx = mesh->edge_stride * src_info->idx;
		v_idx[0] = mesh->edge_indices[idx++];
		v_idx[1] = mesh->edge_indices[idx >= mesh->edge_indices_cnt ? 0 : idx];
		v_idx[2] = dst_info->idx;
		idx = geometry_indices_find_face_index(mesh->polygons, mesh->polygons_cnt, v_idx, 3);
		if (idx != -1) {
			dst_info->hit_bits = CCT_SWEEP_BIT_FACE;
			dst_info->idx = idx;
			*ret_new_hit_bits = CCT_SWEEP_BIT_FACE;
			return 1;
		}
		dst_info->hit_bits = 0;
		dst_info->idx = 0;
		*ret_new_hit_bits = 0;
		return 1;
	}
	else if ((dst_info->hit_bits & CCT_SWEEP_BIT_SEGMENT) && (src_info->hit_bits & CCT_SWEEP_BIT_POINT)) {
		CCTNum_t edge[2][3];
		idx = dst_info->idx * mesh->edge_stride;
		mathVec3Copy(edge[0], mesh->v[mesh->edge_indices[idx++]]);
		mathVec3Copy(edge[1], mesh->v[mesh->edge_indices[idx >= mesh->edge_indices_cnt ? 0 : idx]]);
		if (Segment_Contain_Point((const CCTNum_t(*)[3])edge, mesh->v[src_info->idx])) {
			return 0;
		}
		idx = mesh->edge_stride * dst_info->idx;
		v_idx[0] = mesh->edge_indices[idx++];
		v_idx[1] = mesh->edge_indices[idx >= mesh->edge_indices_cnt ? 0 : idx];
		v_idx[2] = src_info->idx;
		idx = geometry_indices_find_face_index(mesh->polygons, mesh->polygons_cnt, v_idx, 3);
		if (idx != -1) {
			dst_info->hit_bits = CCT_SWEEP_BIT_FACE;
			dst_info->idx = idx;
			*ret_new_hit_bits = CCT_SWEEP_BIT_FACE;
			return 1;
		}
		dst_info->hit_bits = 0;
		dst_info->idx = 0;
		*ret_new_hit_bits = 0;
		return 1;
	}
	if (dst_info->hit_bits != 0 && (src_info->hit_bits & CCT_SWEEP_BIT_FACE)) {
		if (dst_info->hit_bits < src_info->hit_bits) {
			*dst_info = *src_info;
			*ret_new_hit_bits = CCT_SWEEP_BIT_FACE;
			return 1;
		}
		else if (dst_info->hit_bits > src_info->hit_bits) {
			dst_info->hit_bits = 0;
			dst_info->idx = 0;
			*ret_new_hit_bits = 0;
			return 1;
		}
		else if (dst_info->idx != src_info->idx) {
			dst_info->hit_bits = 0;
			dst_info->idx = 0;
			*ret_new_hit_bits = 0;
			return 1;
		}
		return 0;
	}
	return 0;
}

static CCTSweepResult_t* MeshSegment_Sweep_MeshSegment(const GeometryMesh_t* s1, const CCTNum_t dir[3], const GeometryMesh_t* s2, CCTSweepResult_t* result) {
	unsigned int i, j;
	CCTSweepResult_t result_temp;
	CCTSweepResult_t* p_result = NULL;
	for (i = 0; i < s1->edge_indices_cnt; ) {
		CCTNum_t edge1[2][3];
		unsigned int v_idx1[2];
		v_idx1[0] = s1->edge_indices[i++];
		if (2 == s1->edge_stride) {
			v_idx1[1] = s1->edge_indices[i++];
		}
		else {
			v_idx1[1] = s1->edge_indices[i >= s1->edge_indices_cnt ? 0 : i];
		}
		mathVec3Copy(edge1[0], s1->v[v_idx1[0]]);
		mathVec3Copy(edge1[1], s1->v[v_idx1[1]]);
		for (j = 0; j < s2->edge_indices_cnt; ) {
			CCTNum_t edge2[2][3];
			unsigned int v_idx2[2];
			v_idx2[0] = s2->edge_indices[j++];
			if (2 == s2->edge_stride) {
				v_idx2[1] = s2->edge_indices[j++];
			}
			else {
				v_idx2[1] = s2->edge_indices[j >= s2->edge_indices_cnt ? 0 : j];
			}
			mathVec3Copy(edge2[0], s2->v[v_idx2[0]]);
			mathVec3Copy(edge2[1], s2->v[v_idx2[1]]);
			if (!Segment_Sweep_Segment((const CCTNum_t(*)[3])edge1, dir, (const CCTNum_t(*)[3])edge2, &result_temp)) {
				continue;
			}
			if (result_temp.distance <= CCTNum(0.0)) {
				*result = result_temp;
				return result;
			}
			if (!p_result) {
				p_result = result;
				*result = result_temp;
			}
			else if (result_temp.distance > result->distance + CCT_EPSILON) {
				continue;
			}
			else if (result_temp.distance < result->distance - CCT_EPSILON) {
				*result = result_temp;
			}
			else {
				int new_hit_bits = result_temp.hit_bits;
				if (result_temp.distance < result->distance) {
					result->distance = result_temp.distance;
				}
				if (result_temp.peer[0].hit_bits & CCT_SWEEP_BIT_POINT) {
					result_temp.peer[0].idx = v_idx1[result_temp.peer[0].idx ? 1 : 0];
				}
				else {
					result_temp.peer[0].idx = (i - 1) / s1->edge_stride;
				}
				if (result_temp.peer[1].hit_bits & CCT_SWEEP_BIT_POINT) {
					result_temp.peer[1].idx = v_idx2[result_temp.peer[1].idx ? 1 : 0];
				}
				else {
					result_temp.peer[1].idx = (j - 1) / s2->edge_stride;
				}
				merge_mesh_hit_info(&result->peer[0], &result_temp.peer[0], s1, NULL);
				if (!merge_mesh_hit_info(&result->peer[1], &result_temp.peer[1], s2, &new_hit_bits)) {
					continue;
				}
				result->hit_bits = new_hit_bits;
				continue;
			}
			if (result_temp.peer[0].hit_bits & CCT_SWEEP_BIT_POINT) {
				result->peer[0].idx = v_idx1[result_temp.peer[0].idx ? 1 : 0];
			}
			else {
				result->peer[0].idx = (i - 1) / s1->edge_stride;
			}
			if (result_temp.peer[1].hit_bits & CCT_SWEEP_BIT_POINT) {
				result->peer[1].idx = v_idx2[result_temp.peer[1].idx ? 1 : 0];
			}
			else {
				result->peer[1].idx = (j - 1) / s2->edge_stride;
			}
		}
	}
	return p_result;
}

static CCTSweepResult_t* Mesh_Sweep_Mesh_InternalProc(const GeometryMesh_t* mesh1, const CCTNum_t dir[3], const GeometryMesh_t* mesh2, CCTSweepResult_t* result) {
	unsigned int i;
	CCTNum_t neg_dir[3];
	CCTSweepResult_t* p_result;
	p_result = MeshSegment_Sweep_MeshSegment(mesh1, dir, mesh2, result);
	for (i = 0; i < mesh2->polygons_cnt; ++i) {
		const GeometryPolygon_t* polygon2 = mesh2->polygons + i;
		const CCTNum_t* polygon2_p = polygon2->v[polygon2->v_indices[0]];
		CCTSweepResult_t result_temp;
		unsigned int j;
		for (j = 0; j < mesh1->v_indices_cnt; ++j) {
			const CCTNum_t* mesh1_p = mesh1->v[mesh1->v_indices[j]];
			if (!Ray_Sweep_Plane(mesh1_p, dir, polygon2_p, polygon2->normal, &result_temp)) {
				continue;
			}
			if (!p_result) {
				if (!Polygon_Contain_Point(polygon2, result_temp.hit_plane_v)) {
					continue;
				}
				p_result = result;
			}
			else if (result_temp.distance > result->distance + CCT_EPSILON) {
				continue;
			}
			else if (result_temp.distance < result->distance - CCT_EPSILON) {
				if (!Polygon_Contain_Point(polygon2, result_temp.hit_plane_v)) {
					continue;
				}
			}
			else {
				int new_hit_bits = result_temp.hit_bits;
				if (!Polygon_Contain_Point(polygon2, result_temp.hit_plane_v)) {
					continue;
				}
				if (result_temp.distance < result->distance) {
					result->distance = result_temp.distance;
				}
				if (polygon_find_edge_idx(polygon2, result_temp.hit_plane_v) != -1) {
					continue;
				}
				result_temp.peer[0].idx = mesh1->v_indices[j];
				result_temp.peer[1].idx = i;
				merge_mesh_hit_info(&result->peer[0], &result_temp.peer[0], mesh1, NULL);
				if (!merge_mesh_hit_info(&result->peer[1], &result_temp.peer[1], mesh2, &new_hit_bits)) {
					continue;
				}
				result->hit_bits = new_hit_bits;
				continue;
			}
			*result = result_temp;
			result->peer[0].idx = mesh1->v_indices[j];
			result->peer[1].idx = i;
		}
	}
	mathVec3Negate(neg_dir, dir);
	for (i = 0; i < mesh1->polygons_cnt; ++i) {
		const GeometryPolygon_t* polygon1 = mesh1->polygons + i;
		const CCTNum_t* polygon1_p = polygon1->v[polygon1->v_indices[0]];
		CCTSweepResult_t result_temp;
		unsigned int j;
		for (j = 0; j < mesh2->v_indices_cnt; ++j) {
			const CCTNum_t* mesh2_p = mesh2->v[mesh2->v_indices[j]];
			if (!Ray_Sweep_Plane(mesh2_p, neg_dir, polygon1_p, polygon1->normal, &result_temp)) {
				continue;
			}
			if (!p_result) {
				if (!Polygon_Contain_Point(polygon1, result_temp.hit_plane_v)) {
					continue;
				}
				p_result = result;
			}
			else if (result_temp.distance > result->distance + CCT_EPSILON) {
				continue;
			}
			else if (result_temp.distance < result->distance - CCT_EPSILON) {
				if (!Polygon_Contain_Point(polygon1, result_temp.hit_plane_v)) {
					continue;
				}
			}
			else {
				int new_hit_bits = result_temp.hit_bits;
				if (!Polygon_Contain_Point(polygon1, result_temp.hit_plane_v)) {
					continue;
				}
				if (result_temp.distance < result->distance) {
					result->distance = result_temp.distance;
				}
				if (polygon_find_edge_idx(polygon1, result_temp.hit_plane_v) != -1) {
					continue;
				}
				mathVec3Copy(result_temp.hit_plane_v, mesh2_p);
				result_temp.peer[0].hit_bits = CCT_SWEEP_BIT_FACE;
				result_temp.peer[0].idx = i;
				result_temp.peer[1].hit_bits = CCT_SWEEP_BIT_POINT;
				result_temp.peer[1].idx = mesh2->v_indices[j];
				merge_mesh_hit_info(&result->peer[0], &result_temp.peer[0], mesh1, NULL);
				if (!merge_mesh_hit_info(&result->peer[1], &result_temp.peer[1], mesh2, &new_hit_bits)) {
					continue;
				}
				result->hit_bits = new_hit_bits;
				continue;
			}
			*result = result_temp;
			mathVec3Copy(result->hit_plane_v, mesh2_p);
			result->peer[0].hit_bits = CCT_SWEEP_BIT_FACE;
			result->peer[0].idx = i;
			result->peer[1].hit_bits = CCT_SWEEP_BIT_POINT;
			result->peer[1].idx = mesh2->v_indices[j];
		}
	}
	if (!p_result) {
		return NULL;
	}
	return result;
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
		if (Ray_Sweep_Sphere(ls[1], dir, circle_o, circle_r, &result_temp) &&
			result_temp.distance < result->distance)
		{
			*result = result_temp;
			result->peer[0].idx = 1;
		}
		return result;
	}
	if (!Ray_Sweep_Sphere(ls[1], dir, circle_o, circle_r, result)) {
		return NULL;
	}
	result->peer[0].idx = 1;
	return result;
}

static CCTSweepResult_t* Segment_Sweep_Sphere(const CCTNum_t ls[2][3], const CCTNum_t dir[3], const CCTNum_t center[3], CCTNum_t radius, int check_intersect, CCTSweepResult_t* result) {
	CCTNum_t lsdir[3], N[3];
	if (check_intersect && Sphere_Intersect_Segment(center, radius, ls)) {
		set_intersect(result);
		return result;
	}
	mathVec3Sub(lsdir, ls[1], ls[0]);
	mathVec3Cross(N, lsdir, dir);
	if (!mathVec3IsZero(N)) {
		CCTNum_t circle_o[3], circle_r;
		mathVec3Normalized(N, N);
		if (0 == Sphere_Intersect_Plane(center, radius, ls[0], N, circle_o, &circle_r)) {
			return NULL;
		}
		if (!Segment_Sweep_Circle_InSamePlane(ls, dir, circle_o, circle_r, result)) {
			return NULL;
		}
		result->peer[1].hit_bits = CCT_SWEEP_BIT_SPHERE;
		result->peer[1].idx = 0;
		return result;
	}
	if (Ray_Sweep_Sphere(ls[0], dir, center, radius, result)) {
		CCTSweepResult_t result_temp;
		if (!Ray_Sweep_Sphere(ls[1], dir, center, radius, &result_temp)) {
			/* no possible */
			return NULL;
		}
		if (result_temp.distance < result->distance) {
			*result = result_temp;
			result->peer[0].idx = 1;
		}
		return result;
	}
	return NULL;
}

static CCTSweepResult_t* MeshSegment_Sweep_Sphere(const GeometryMesh_t* mesh, const CCTNum_t dir[3], const CCTNum_t center[3], CCTNum_t radius, int check_intersect, CCTSweepResult_t* result) {
	unsigned int i;
	CCTSweepResult_t* p_result = NULL;
	for (i = 0; i < mesh->edge_indices_cnt; ) {
		CCTSweepResult_t result_temp;
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
		if (!Segment_Sweep_Sphere((const CCTNum_t(*)[3])edge, dir, center, radius, check_intersect, &result_temp)) {
			continue;
		}
		if (result_temp.distance <= CCTNum(0.0)) {
			*result = result_temp;
			return result;
		}
		if (!p_result) {
			p_result = result;
			*result = result_temp;
		}
		else if (result_temp.distance > result->distance + CCT_EPSILON) {
			continue;
		}
		else if (result_temp.distance < result->distance - CCT_EPSILON) {
			*result = result_temp;
		}
		else {
			if (result_temp.distance < result->distance) {
				result->distance = result_temp.distance;
			}
			if (result_temp.peer[0].hit_bits & CCT_SWEEP_BIT_POINT) {
				result_temp.peer[0].idx = v_idx[result_temp.peer[0].idx ? 1 : 0];
			}
			else {
				result_temp.peer[0].hit_bits = CCT_SWEEP_BIT_SEGMENT;
				result_temp.peer[0].idx = (i - 1) / mesh->edge_stride;
			}
			merge_mesh_hit_info(&result->peer[0], &result_temp.peer[0], mesh, NULL);
			continue;
		}
		if (result_temp.peer[0].hit_bits & CCT_SWEEP_BIT_POINT) {
			result->peer[0].idx = v_idx[result_temp.peer[0].idx ? 1 : 0];
		}
		else {
			result_temp.peer[0].hit_bits = CCT_SWEEP_BIT_SEGMENT;
			result->peer[0].idx = (i - 1) / mesh->edge_stride;
		}
	}
	return p_result;
}

static CCTSweepResult_t* Mesh_Sweep_Plane(const GeometryMesh_t* mesh, const CCTNum_t dir[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTSweepResult_t* result) {
	int flag_sign = 0, flag_nohit;
	unsigned int i, same_v_idx[3], same_v_cnt = 0;
	CCTNum_t cos_theta = mathVec3Dot(dir, plane_n);
	flag_nohit = (CCTNum(0.0) == cos_theta);
	for (i = 0; i < mesh->v_indices_cnt; ++i) {
		CCTNum_t d = mathPointProjectionPlane(mesh->v[mesh->v_indices[i]], plane_v, plane_n);
		if (d > CCTNum(0.0)) {
			if (flag_sign < 0) {
				set_intersect(result);
				return result;
			}
			flag_sign = 1;
		}
		else if (d < CCTNum(0.0)) {
			if (flag_sign > 0) {
				set_intersect(result);
				return result;
			}
			flag_sign = -1;
		}
		else {
			set_intersect(result);
			return result;
		}
		if (flag_nohit) {
			continue;
		}
		d /= cos_theta;
		if (d < CCTNum(0.0)) {
			flag_nohit = 1;
			continue;
		}
		if (0 == i) {
			result->distance = d;
		}
		else if (d > result->distance + CCT_EPSILON) {
			continue;
		}
		else if (d < result->distance - CCT_EPSILON) {
			result->distance = d;
		}
		else {
			unsigned int idx;
			if (d < result->distance) {
				result->distance = d;
			}
			if (same_v_cnt >= 3) {
				continue;
			}
			same_v_idx[same_v_cnt++] = mesh->v_indices[i];
			if (2 == same_v_cnt) {
				idx = mathFindEdgeIndexByTwoVertexIndex(mesh->edge_indices, mesh->edge_indices_cnt, mesh->edge_stride, same_v_idx[0], same_v_idx[1]);
				if (idx != -1) {
					result->hit_bits = CCT_SWEEP_BIT_SEGMENT;
					result->peer[0].hit_bits = CCT_SWEEP_BIT_SEGMENT;
					result->peer[0].idx = idx;
					continue;
				}
			}
			idx = geometry_indices_find_face_index(mesh->polygons, mesh->polygons_cnt, same_v_idx, same_v_cnt);
			if (idx != -1) {
				result->hit_bits = CCT_SWEEP_BIT_FACE;
				result->peer[0].hit_bits = CCT_SWEEP_BIT_FACE;
				result->peer[0].idx = idx;
				continue;
			}
			result->hit_bits = 0;
			result->peer[0].hit_bits = 0;
			result->peer[0].idx = 0;
			continue;
		}
		result->hit_bits = CCT_SWEEP_BIT_POINT;
		result->peer[0].hit_bits = CCT_SWEEP_BIT_POINT;
		result->peer[0].idx = mesh->v_indices[i];
		same_v_idx[0] = mesh->v_indices[i];
		same_v_cnt = 1;
	}
	if (flag_nohit) {
		return NULL;
	}
	if (result->peer[0].hit_bits & CCT_SWEEP_BIT_POINT) {
		mathVec3Copy(result->hit_plane_v, mesh->v[result->peer[0].idx]);
		mathVec3AddScalar(result->hit_plane_v, dir, result->distance);
	}
	else {
		mathVec3Copy(result->hit_plane_v, plane_v);
	}
	mathVec3Copy(result->hit_plane_n, plane_n);
	result->peer[1].hit_bits = CCT_SWEEP_BIT_FACE;
	result->peer[1].idx = 0;
	return result;
}

static CCTSweepResult_t* AABB_Sweep_AABB(const CCTNum_t o1[3], const CCTNum_t half1[3], const CCTNum_t dir[3], const CCTNum_t o2[3], const CCTNum_t half2[3], CCTSweepResult_t* result) {
	if (AABB_Intersect_AABB(o1, half1, o2, half2)) {
		set_intersect(result);
		return result;
	}
	else {
		CCTSweepResult_t* p_result = NULL;
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
				*result = result_temp;
			}
			else if (result_temp.distance < result->distance) {
				*result = result_temp;
			}
			else {
				continue;
			}
		}
		return p_result;
	}
}

static CCTSweepResult_t* Segment_Sweep_ConvexMesh(const CCTNum_t ls[2][3], const CCTNum_t dir[3], const GeometryMesh_t* mesh, CCTSweepResult_t* result) {
	GeometryMesh_t m1;
	if (Segment_Intersect_ConvexMesh(ls, mesh)) {
		set_intersect(result);
		return result;
	}
	sweep_mesh_convert_from_segment(&m1, ls);
	return Mesh_Sweep_Mesh_InternalProc(&m1, dir, mesh, result);
}

static CCTSweepResult_t* Segment_Sweep_Polygon(const CCTNum_t ls[2][3], const CCTNum_t dir[3], const GeometryPolygon_t* polygon, CCTSweepResult_t* result) {
	int plane_side;
	GeometryMesh_t m1, m2;
	if (Segment_Intersect_Polygon(ls, polygon, &plane_side)) {
		set_intersect(result);
		return result;
	}
	if (plane_side) {
		CCTNum_t d, cos_theta = mathVec3Dot(dir, polygon->normal);
		if (CCTNum(0.0) == cos_theta) {
			return NULL;
		}
		d = mathPointProjectionPlane(ls[0], polygon->v[polygon->v_indices[0]], polygon->normal);
		d /= cos_theta;
		if (d < CCTNum(0.0)) {
			return NULL;
		}
	}
	sweep_mesh_convert_from_segment(&m1, ls);
	sweep_mesh_convert_from_polygon(&m2, polygon);
	return Mesh_Sweep_Mesh_InternalProc(&m1, dir, &m2, result);
}

static CCTSweepResult_t* Polygon_Sweep_Polygon(const GeometryPolygon_t* polygon1, const CCTNum_t dir[3], const GeometryPolygon_t* polygon2, CCTSweepResult_t* result) {
	int plane_side;
	GeometryMesh_t m1, m2;
	if (Polygon_Intersect_Polygon(polygon1, polygon2, &plane_side)) {
		set_intersect(result);
		return result;
	}
	if (plane_side) {
		CCTNum_t d, cos_theta = mathVec3Dot(dir, polygon2->normal);
		if (CCTNum(0.0) == cos_theta) {
			return NULL;
		}
		d = mathPointProjectionPlane(polygon1->v[polygon1->v_indices[0]], polygon2->v[polygon2->v_indices[0]], polygon2->normal);
		d /= cos_theta;
		if (d < CCTNum(0.0)) {
			return NULL;
		}
	}
	sweep_mesh_convert_from_polygon(&m1, polygon1);
	sweep_mesh_convert_from_polygon(&m2, polygon2);
	return Mesh_Sweep_Mesh_InternalProc(&m1, dir, &m2, result);
}

static CCTSweepResult_t* ConvexMesh_Sweep_Polygon(const GeometryMesh_t* mesh, const CCTNum_t dir[3], const GeometryPolygon_t* polygon, CCTSweepResult_t* result) {
	int plane_side;
	GeometryMesh_t m2;
	if (ConvexMesh_Intersect_Polygon(mesh, polygon, &plane_side)) {
		set_intersect(result);
		return result;
	}
	if (plane_side) {
		CCTNum_t d, cos_theta = mathVec3Dot(dir, polygon->normal);
		if (CCTNum(0.0) == cos_theta) {
			return NULL;
		}
		d = mathPointProjectionPlane(mesh->v[mesh->v_indices[0]], polygon->v[polygon->v_indices[0]], polygon->normal);
		d /= cos_theta;
		if (d < CCTNum(0.0)) {
			return NULL;
		}
	}
	sweep_mesh_convert_from_polygon(&m2, polygon);
	return Mesh_Sweep_Mesh_InternalProc(mesh, dir, &m2, result);
}

static CCTSweepResult_t* ConvexMesh_Sweep_ConvexMesh(const GeometryMesh_t* mesh1, const CCTNum_t dir[3], const GeometryMesh_t* mesh2, CCTSweepResult_t* result) {
	if (ConvexMesh_Intersect_ConvexMesh(mesh1, mesh2)) {
		set_intersect(result);
		return result;
	}
	return Mesh_Sweep_Mesh_InternalProc(mesh1, dir, mesh2, result);
}

static CCTSweepResult_t* OBB_Sweep_OBB(const GeometryOBB_t* obb1, const CCTNum_t dir[3], const GeometryOBB_t* obb2, CCTSweepResult_t* result) {
	GeometryBoxMesh_t mesh1, mesh2;
	CCTNum_t v1[8][3], v2[8][3];
	if (OBB_Intersect_OBB(obb1, obb2)) {
		set_intersect(result);
		return result;
	}
	mathOBBVertices(obb1, v1);
	mathBoxMesh(&mesh1, (const CCTNum_t(*)[3])v1, (const CCTNum_t(*)[3])obb1->axis);
	mathOBBVertices(obb2, v2);
	mathBoxMesh(&mesh2, (const CCTNum_t(*)[3])v2, (const CCTNum_t(*)[3])obb2->axis);
	return Mesh_Sweep_Mesh_InternalProc(&mesh1.mesh, dir, &mesh2.mesh, result);
}

static CCTSweepResult_t* Sphere_Sweep_Plane(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t dir[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTSweepResult_t* result) {
	CCTNum_t dn, dn_abs, cos_theta;
	dn = mathPointProjectionPlane(o, plane_v, plane_n);
	dn_abs = CCTNum_abs(dn);
	if (dn_abs < radius) {
		set_intersect(result);
		return result;
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
	result->peer[0].hit_bits = CCT_SWEEP_BIT_SPHERE;
	result->peer[0].idx = 0;
	result->peer[1].hit_bits = CCT_SWEEP_BIT_FACE;
	result->peer[1].idx = 0;
	return result;
}

static CCTSweepResult_t* Mesh_Sweep_Sphere_InternalProc(const GeometryMesh_t* mesh, const CCTNum_t dir[3], const CCTNum_t o[3], CCTNum_t radius, CCTSweepResult_t* result) {
	unsigned int i;
	CCTSweepResult_t* p_result;
	CCTNum_t neg_dir[3];

	p_result = MeshSegment_Sweep_Sphere(mesh, dir, o, radius, 0, result);
	mathVec3Negate(neg_dir, dir);
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		CCTSweepResult_t result_temp;
		const GeometryPolygon_t* polygon = mesh->polygons + i;
		if (!Sphere_Sweep_Plane(o, radius, neg_dir, polygon->v[polygon->v_indices[0]], polygon->normal, &result_temp)) {
			continue;
		}
		if (!(result_temp.hit_bits & CCT_SWEEP_BIT_POINT)) {
			continue;
		}
		if (!p_result) {
			if (!Polygon_Contain_Point(polygon, result_temp.hit_plane_v)) {
				continue;
			}
			p_result = result;
			*result = result_temp;
		}
		else if (result_temp.distance > result->distance + CCT_EPSILON) {
			continue;
		}
		else if (result_temp.distance < result->distance - CCT_EPSILON) {
			if (!Polygon_Contain_Point(polygon, result_temp.hit_plane_v)) {
				continue;
			}
			*result = result_temp;
		}
		else {
			if (!Polygon_Contain_Point(polygon, result_temp.hit_plane_v)) {
				continue;
			}
			if (result_temp.distance < result->distance) {
				result->distance = result_temp.distance;
			}
			if (polygon_find_edge_idx(polygon, result_temp.hit_plane_v) != -1) {
				continue;
			}
			result_temp.peer[0].hit_bits = CCT_SWEEP_BIT_FACE;
			result_temp.peer[0].idx = i;
			result_temp.peer[1].hit_bits = CCT_SWEEP_BIT_SPHERE;
			result_temp.peer[1].idx = 0;
			merge_mesh_hit_info(&result->peer[0], &result_temp.peer[0], mesh, NULL);
			continue;
		}
		result->peer[0].hit_bits = CCT_SWEEP_BIT_FACE;
		result->peer[0].idx = i;
		result->peer[1].hit_bits = CCT_SWEEP_BIT_SPHERE;
		result->peer[1].idx = 0;
		mathVec3AddScalar(result->hit_plane_v, dir, result->distance);
	}
	return p_result;
}

static CCTSweepResult_t* Sphere_Sweep_Polygon(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t dir[3], const GeometryPolygon_t* polygon, CCTSweepResult_t* result) {
	int plane_side;
	CCTNum_t neg_dir[3];
	GeometryMesh_t m;
	if (Sphere_Intersect_Polygon(o, radius, polygon, &plane_side)) {
		set_intersect(result);
		return result;
	}
	if (plane_side) {
		if (!Sphere_Sweep_Plane(o, radius, dir, polygon->v[polygon->v_indices[0]], polygon->normal, result)) {
			return NULL;
		}
		if (Polygon_Contain_Point(polygon, result->hit_plane_v)) {
			return result;
		}
	}
	sweep_mesh_convert_from_polygon(&m, polygon);
	m.polygons = NULL;
	m.polygons_cnt = 0;
	mathVec3Negate(neg_dir, dir);
	if (!MeshSegment_Sweep_Sphere(&m, neg_dir, o, radius, 0, result)) {
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
	result->peer[0].hit_bits = CCT_SWEEP_BIT_SPHERE;
	return result;
}

static CCTSweepResult_t* ConvexMesh_Sweep_Sphere(const GeometryMesh_t* mesh, const CCTNum_t dir[3], const CCTNum_t o[3], CCTNum_t radius, CCTSweepResult_t* result) {
	if (Sphere_Intersect_ConvexMesh(o, radius, mesh)) {
		set_intersect(result);
		return result;
	}
	return Mesh_Sweep_Sphere_InternalProc(mesh, dir, o, radius, result);
}

static CCTSweepResult_t* OBB_Sweep_Sphere(const GeometryOBB_t* obb, const CCTNum_t dir[3], const CCTNum_t o[3], CCTNum_t radius, CCTSweepResult_t* result) {
	GeometryBoxMesh_t mesh;
	CCTNum_t v[8][3];
	if (Sphere_Intersect_OBB(o, radius, obb)) {
		set_intersect(result);
		return result;
	}
	mathOBBVertices(obb, v);
	mathBoxMesh(&mesh, (const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])obb->axis);
	return Mesh_Sweep_Sphere_InternalProc(&mesh.mesh, dir, o, radius, result);
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
				mathBoxMesh(&two_mesh, (const CCTNum_t(*)[3])v, AABB_Axis);
				result = Ray_Sweep_ConvexMesh(one->point, dir, &two_mesh.mesh, result);
				break;
			}
			case GEOMETRY_BODY_OBB:
			{
				GeometryBoxMesh_t two_mesh;
				CCTNum_t v[8][3];
				mathOBBVertices(two->obb, v);
				mathBoxMesh(&two_mesh, (const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])two->obb->axis);
				result = Ray_Sweep_ConvexMesh(one->point, dir, &two_mesh.mesh, result);
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
				GeometryMesh_t one_mesh;
				sweep_mesh_convert_from_segment(&one_mesh, one_segment_v);
				result = Mesh_Sweep_Plane(&one_mesh, dir, two->plane->v, two->plane->normal, result);
				break;
			}
			case GEOMETRY_BODY_OBB:
			{
				GeometryBoxMesh_t two_mesh;
				CCTNum_t v[8][3];
				mathOBBVertices(two->obb, v);
				mathBoxMesh(&two_mesh, (const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])two->obb->axis);
				result = Segment_Sweep_ConvexMesh(one_segment_v, dir, &two_mesh.mesh, result);
				break;
			}
			case GEOMETRY_BODY_AABB:
			{
				GeometryBoxMesh_t two_mesh;
				CCTNum_t v[8][3];
				mathAABBVertices(two->aabb->o, two->aabb->half, v);
				mathBoxMesh(&two_mesh, (const CCTNum_t(*)[3])v, AABB_Axis);
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
				mathOBBFromAABB(&obb1, one->aabb->o, one->aabb->half);
				result = OBB_Sweep_Sphere(&obb1, dir, two->sphere->o, two->sphere->radius, result);
				break;
			}
			case GEOMETRY_BODY_PLANE:
			{
				GeometryBoxMesh_t one_mesh;
				CCTNum_t v[8][3];
				mathAABBVertices(one->aabb->o, one->aabb->half, v);
				mathBoxMesh(&one_mesh, (const CCTNum_t(*)[3])v, AABB_Axis);
				result = Mesh_Sweep_Plane(&one_mesh.mesh, dir, two->plane->v, two->plane->normal, result);
				break;
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				GeometryBoxMesh_t one_mesh;
				CCTNum_t neg_dir[3], v[8][3];
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				mathAABBVertices(one->aabb->o, one->aabb->half, v);
				mathBoxMesh(&one_mesh, (const CCTNum_t(*)[3])v, AABB_Axis);
				result = Segment_Sweep_ConvexMesh((const CCTNum_t(*)[3])two->segment->v, neg_dir, &one_mesh.mesh, result);
				break;
			}
			case GEOMETRY_BODY_POLYGON:
			{
				GeometryBoxMesh_t one_mesh;
				CCTNum_t v[8][3];
				mathAABBVertices(one->aabb->o, one->aabb->half, v);
				mathBoxMesh(&one_mesh, (const CCTNum_t(*)[3])v, AABB_Axis);
				result = ConvexMesh_Sweep_Polygon(&one_mesh.mesh, dir, two->polygon, result);
				break;
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				GeometryBoxMesh_t one_mesh;
				CCTNum_t v[8][3];
				mathAABBVertices(one->aabb->o, one->aabb->half, v);
				mathBoxMesh(&one_mesh, (const CCTNum_t(*)[3])v, AABB_Axis);
				result = ConvexMesh_Sweep_ConvexMesh(&one_mesh.mesh, dir, two->mesh, result);
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
				mathBoxMesh(&one_mesh, (const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])one->obb->axis);
				result = Segment_Sweep_ConvexMesh((const CCTNum_t(*)[3])two->segment->v, neg_dir, &one_mesh.mesh, result);
				break;
			}
			case GEOMETRY_BODY_PLANE:
			{
				GeometryBoxMesh_t one_mesh;
				CCTNum_t v[8][3];
				mathOBBVertices(one->obb, v);
				mathBoxMesh(&one_mesh, (const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])one->obb->axis);
				result = Mesh_Sweep_Plane(&one_mesh.mesh, dir, two->plane->v, two->plane->normal, result);
				break;
			}
			case GEOMETRY_BODY_SPHERE:
			{
				result = OBB_Sweep_Sphere(one->obb, dir, two->sphere->o, two->sphere->radius, result);
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
				mathBoxMesh(&one_mesh, (const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])one->obb->axis);
				result = ConvexMesh_Sweep_Polygon(&one_mesh.mesh, dir, two->polygon, result);
				break;
			}
			case GEOMETRY_BODY_CONVEX_MESH:
			{
				GeometryBoxMesh_t one_mesh;
				CCTNum_t v[8][3];
				mathOBBVertices(one->obb, v);
				mathBoxMesh(&one_mesh, (const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])one->obb->axis);
				result = ConvexMesh_Sweep_ConvexMesh(&one_mesh.mesh, dir, two->mesh, result);
				break;
			}
		}
	}
	else if (GEOMETRY_BODY_SPHERE == one->type) {
		switch (two->type) {
			case GEOMETRY_BODY_OBB:
			{
				CCTNum_t neg_dir[3];
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = OBB_Sweep_Sphere(two->obb, neg_dir, one->sphere->o, one->sphere->radius, result);
				break;
			}
			case GEOMETRY_BODY_AABB:
			{
				GeometryOBB_t obb2;
				CCTNum_t neg_dir[3];
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				mathOBBFromAABB(&obb2, two->aabb->o, two->aabb->half);
				result = OBB_Sweep_Sphere(&obb2, neg_dir, one->sphere->o, one->sphere->radius, result);
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
				CCTNum_t neg_dir[3];
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = ConvexMesh_Sweep_Sphere(two->mesh, neg_dir, one->sphere->o, one->sphere->radius, result);
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
				GeometryMesh_t one_mesh;
				const GeometryPolygon_t* polygon = one->polygon;
				const GeometryPlane_t* plane = two->plane;
				sweep_mesh_convert_from_polygon(&one_mesh, polygon);
				result = Mesh_Sweep_Plane(&one_mesh, dir, plane->v, plane->normal, result);
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
				mathBoxMesh(&two_mesh, (const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])two->obb->axis);
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
				mathBoxMesh(&two_mesh, (const CCTNum_t(*)[3])v, AABB_Axis);
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
				result = Mesh_Sweep_Plane(mesh, dir, plane->v, plane->normal, result);
				break;
			}
			case GEOMETRY_BODY_SPHERE:
			{
				result = ConvexMesh_Sweep_Sphere(one->mesh, dir, two->sphere->o, two->sphere->radius, result);
				break;
			}
			case GEOMETRY_BODY_OBB:
			{
				GeometryBoxMesh_t two_mesh;
				CCTNum_t v[8][3];
				mathOBBVertices(two->obb, v);
				mathBoxMesh(&two_mesh, (const CCTNum_t(*)[3])v, (const CCTNum_t(*)[3])two->obb->axis);
				result = ConvexMesh_Sweep_ConvexMesh(one->mesh, dir, &two_mesh.mesh, result);
				break;
			}
			case GEOMETRY_BODY_AABB:
			{
				GeometryBoxMesh_t two_mesh;
				CCTNum_t v[8][3];
				mathAABBVertices(two->aabb->o, two->aabb->half, v);
				mathBoxMesh(&two_mesh, (const CCTNum_t(*)[3])v, AABB_Axis);
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
		reverse_result(result, dir);
	}
	return result;
}

#ifdef __cplusplus
}
#endif
