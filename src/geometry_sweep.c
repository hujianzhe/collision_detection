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
#include "../inc/geometry_closest.h"
#include "../inc/geometry_api.h"

extern int Ray_Intersect_Plane(const CCTNum_t o[3], const CCTNum_t dir[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3]);
extern int Segment_Contain_Point(const CCTNum_t ls0[3], const CCTNum_t ls1[3], const CCTNum_t p[3]);
extern int Segment_Intersect_Plane(const CCTNum_t ls[2][3], const CCTNum_t plane_v[3], const CCTNum_t plane_normal[3], CCTNum_t p[3], CCTNum_t d[3]);
extern int Segment_Intersect_Polygon(const CCTNum_t ls[2][3], const GeometryPolygon_t* polygon, int* ret_plane_side);
extern int Segment_Intersect_ConvexMesh(const CCTNum_t ls[2][3], const GeometryMesh_t* mesh);
extern int Sphere_Intersect_Segment(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t ls0[3], const CCTNum_t ls1[3]);
extern int Sphere_Intersect_Plane(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t plane_v[3], const CCTNum_t plane_normal[3], CCTNum_t new_o[3], CCTNum_t* new_r);
extern int Sphere_Intersect_Polygon(const CCTNum_t o[3], CCTNum_t radius, const GeometryPolygon_t* polygon, int* ret_plane_side);
extern int OBB_Intersect_OBB(const GeometryOBB_t* obb0, const GeometryOBB_t* obb1);
extern int Sphere_Intersect_OBB(const CCTNum_t o[3], CCTNum_t radius, const GeometryOBB_t* obb);
extern int Sphere_Intersect_ConvexMesh(const CCTNum_t o[3], CCTNum_t radius, const GeometryMesh_t* mesh);
extern int ConvexMesh_Contain_Point(const GeometryMesh_t* mesh, const CCTNum_t p[3]);
extern int ConvexMesh_Intersect_ConvexMesh(const GeometryMesh_t* mesh1, const GeometryMesh_t* mesh2);
extern int Polygon_Contain_Point_SamePlane(const GeometryPolygon_t* polygon, const CCTNum_t p[3], GeometryBorderId_t* bi);
extern int Polygon_Intersect_Polygon(const GeometryPolygon_t* polygon1, const GeometryPolygon_t* polygon2, int* ret_plane_side);
extern int ConvexMesh_Intersect_Polygon(const GeometryMesh_t* mesh, const GeometryPolygon_t* polygon, int* ret_plane_side);
extern int Capsule_Contain_Point(const GeometryCapsule_t* capsule, const CCTNum_t p[3]);
extern int Capsule_Intersect_Polygon(const GeometryCapsule_t* capsule, const GeometryCapsuleExtra_t* capsule_extra, const GeometryPolygon_t* polygon, int* ret_plane_side);
extern int Capsule_Intersect_ConvexMesh(const GeometryCapsule_t* capsule, const GeometryMesh_t* mesh);
extern CCTNum_t Segment_ClosestVertexIndices_Segment(const CCTNum_t ls1[2][3], const CCTNum_t ls2[2][3], unsigned int* ls1_indices, unsigned int* ls2_indices);
extern CCTNum_t Segment_ClosestLenSq_Segment(const CCTNum_t ls1[2][3], const CCTNum_t ls1_dir[3], CCTNum_t ls1_len, const CCTNum_t ls2[2][3], const CCTNum_t ls2_dir[3], CCTNum_t ls2_len);

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

static void set_intersect(CCTSweepResult_t* result) {
	mathVec3Set(result->hit_plane_n, CCTNums_3(0.0, 0.0, 0.0));
	result->distance = CCTNum(0.0);
	result->overlap = 1;
	result->hit_unique_point = 0;
	result->peer[0].hit_part = 0;
	result->peer[0].id = 0;
	result->peer[1].hit_part = 0;
	result->peer[1].id = 0;
}

static void reverse_result(CCTSweepResult_t* result, const CCTNum_t dir[3]) {
	CCTSweepHitInfo_t hit_info_0 = result->peer[0];
	result->peer[0] = result->peer[1];
	result->peer[1] = hit_info_0;
	mathVec3AddScalar(result->hit_plane_v, dir, result->distance);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

static void sweep_mesh_convert_from_segment(GeometryMesh_t* mesh, const CCTNum_t ls[2][3]) {
	static const unsigned int Segment_Indices_Default[2] = { 0, 1 };
	mesh->v = (CCTNum_t(*)[3])ls;
	mesh->v_indices = Segment_Indices_Default;
	mesh->v_indices_cnt = 2;
	mesh->edge_v_ids_flat = Segment_Indices_Default;
	mesh->edge_v_indices_flat = Segment_Indices_Default;
	mesh->edge_cnt = 1;
	mesh->is_convex = 1;
	mesh->is_closed = 0;
	mesh->polygons = NULL;
	mesh->polygons_cnt = 0;
}

static void sweep_mesh_convert_from_polygon(GeometryMesh_t* mesh, const GeometryPolygon_t* polygon) {
	mesh->v = polygon->v;
	mesh->v_indices = polygon->v_indices;
	mesh->v_indices_cnt = polygon->v_indices_cnt;
	mesh->edge_v_ids_flat = polygon->edge_v_ids_flat;
	mesh->edge_v_indices_flat = polygon->edge_v_indices_flat;
	mesh->edge_cnt = polygon->edge_cnt;
	mesh->is_convex = polygon->is_convex;
	mesh->is_closed = 0;
	mesh->polygons = (GeometryPolygon_t*)polygon;
	mesh->polygons_cnt = 1;
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
		result->peer[1].hit_part = CCT_SWEEP_HIT_POINT;
		if (mathVec3LenSq(v0) > mathVec3LenSq(v1)) {
			dot = mathVec3Dot(dir, v1);
			mathVec3Copy(result->hit_plane_v, ls[1]);
			result->peer[1].id = 1;
		}
		else {
			mathVec3Copy(result->hit_plane_v, ls[0]);
			result->peer[1].id = 0;
		}
		mathVec3Negate(result->hit_plane_n, dir);
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
			result->peer[1].hit_part = CCT_SWEEP_HIT_POINT;
			result->peer[1].id = 0;
		}
		else if (mathVec3Equal(ls[1], p)) {
			result->peer[1].hit_part = CCT_SWEEP_HIT_POINT;
			result->peer[1].id = 1;
		}
		else {
			mathVec3Sub(v0, ls[0], p);
			mathVec3Sub(v1, ls[1], p);
			dot = mathVec3Dot(v0, v1);
			if (dot > CCTNum(0.0)) {
				return NULL;
			}
			result->peer[1].hit_part = CCT_SWEEP_HIT_EDGE;
			result->peer[1].id = 0;
		}
		mathVec3Copy(result->hit_plane_v, p);
		mathVec3Negate(result->hit_plane_n, op);
		result->distance = d;
	}
	result->hit_unique_point = 1;
	result->overlap = 0;
	result->peer[0].hit_part = CCT_SWEEP_HIT_POINT;
	result->peer[0].id = 0;
	return result;
}

static CCTSweepResult_t* Ray_Sweep_MeshSegment(const CCTNum_t o[3], const CCTNum_t dir[3], const GeometryMesh_t* mesh, CCTSweepResult_t* result) {
	unsigned int i, mesh_edge_v_indices_cnt = mesh->edge_cnt + mesh->edge_cnt;
	CCTSweepResult_t* p_result = NULL;
	for (i = 0; i < mesh_edge_v_indices_cnt; ) {
		CCTSweepResult_t result_temp;
		CCTNum_t edge[2][3];
		unsigned int v_idx[2];
		v_idx[0] = mesh->edge_v_indices_flat[i++];
		v_idx[1] = mesh->edge_v_indices_flat[i++];
		mathVec3Copy(edge[0], mesh->v[v_idx[0]]);
		mathVec3Copy(edge[1], mesh->v[v_idx[1]]);
		if (!Ray_Sweep_Segment(o, dir, (const CCTNum_t(*)[3])edge, &result_temp)) {
			continue;
		}
		if (result_temp.overlap) {
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
		if (result_temp.peer[1].hit_part == CCT_SWEEP_HIT_POINT) {
			if (result_temp.peer[1].id) {
				result->peer[1].id = mesh->edge_v_ids_flat[i - 1];
			}
			else {
				result->peer[1].id = mesh->edge_v_ids_flat[i - 2];
			}
		}
		else {
			result->peer[1].id = (i - 1) / 2;
		}
	}
	return p_result;
}

static CCTSweepResult_t* Ray_Sweep_Plane(const CCTNum_t o[3], const CCTNum_t dir[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTSweepResult_t* result) {
	CCTNum_t d, cos_theta;
	d = mathPointProjectionPlane(o, plane_v, plane_n);
	if (CCTNum(0.0) == d) {
		set_intersect(result);
		result->hit_unique_point = 1;
		mathVec3Copy(result->hit_plane_v, o);
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
	if (cos_theta > CCTNum(0.0)) {
		mathVec3Negate(result->hit_plane_n, plane_n);
	}
	else {
		mathVec3Copy(result->hit_plane_n, plane_n);
	}
	result->hit_unique_point = 1;
	result->overlap = 0;
	result->distance = d;
	result->peer[0].hit_part = CCT_SWEEP_HIT_POINT;
	result->peer[0].id = 0;
	result->peer[1].hit_part = CCT_SWEEP_HIT_FACE;
	result->peer[1].id = 0;
	return result;
}

static CCTSweepResult_t* Ray_Sweep_Polygon(const CCTNum_t o[3], const CCTNum_t dir[3], const GeometryPolygon_t* polygon, CCTSweepResult_t* result) {
	CCTNum_t dot;
	GeometryMesh_t mesh;
	GeometryBorderId_t bi;
	if (!Ray_Sweep_Plane(o, dir, polygon->v[polygon->v_indices[0]], polygon->normal, result)) {
		return NULL;
	}
	if (Polygon_Contain_Point_SamePlane(polygon, result->hit_plane_v, &bi)) {
		if (result->overlap) {
			return result;
		}
		if (bi.v_id != -1) {
			result->peer[1].hit_part = CCT_SWEEP_HIT_POINT;
			result->peer[1].id = bi.v_id;
		}
		else if (bi.edge_id != -1) {
			result->peer[1].hit_part = CCT_SWEEP_HIT_EDGE;
			result->peer[1].id = bi.edge_id;
		}
		return result;
	}
	if (!result->overlap) {
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
	if (d_sq > radius_sq + CCT_EPSILON) {
		return NULL;
	}
	if (d_sq < radius_sq) {
		dir_d -= CCTNum_sqrt(radius_sq - d_sq);
	}
	mathVec3Copy(result->hit_plane_v, o);
	mathVec3AddScalar(result->hit_plane_v, dir, dir_d);
	result->hit_unique_point = 1;
	result->overlap = 0;
	mathVec3Sub(result->hit_plane_n, result->hit_plane_v, sp_o);
	mathVec3MultiplyScalar(result->hit_plane_n, result->hit_plane_n, CCTNum(1.0) / sp_radius);
	result->distance = dir_d;
	result->peer[0].hit_part = CCT_SWEEP_HIT_POINT;
	result->peer[0].id = 0;
	result->peer[1].hit_part = CCT_SWEEP_HIT_SPHERE;
	result->peer[1].id = 0;
	return result;
}

static CCTSweepResult_t* Ray_Sweep_ConvexMesh(const CCTNum_t o[3], const CCTNum_t dir[3], const GeometryMesh_t* mesh, CCTSweepResult_t* result) {
	unsigned int i;
	CCTSweepResult_t* p_result;
	const GeometryPolygon_t* rface;
	if (ConvexMesh_Contain_Point(mesh, o)) {
		set_intersect(result);
		return result;
	}
	p_result = NULL;
	rface = NULL;
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		CCTSweepResult_t result_temp;
		const GeometryPolygon_t* polygon = mesh->polygons + i;
		if (!Ray_Sweep_Plane(o, dir, polygon->v[polygon->v_indices[0]], polygon->normal, &result_temp)) {
			continue;
		}
		if (result_temp.overlap) {
			continue;
		}
		if (p_result && result_temp.distance >= result->distance) {
			continue;
		}
		if (!Polygon_Contain_Point_SamePlane(polygon, result_temp.hit_plane_v, NULL)) {
			continue;
		}
		rface = polygon;
		p_result = result;
		*result = result_temp;
	}
	if (p_result) {
		GeometryBorderId_t bi;
		unsigned int rface_edge_v_indices_cnt = rface->edge_cnt + rface->edge_cnt;
		mathFindBorderIdByPoint((const CCTNum_t(*)[3])rface->v, rface->v_indices, rface->edge_v_ids_flat, rface_edge_v_indices_cnt, result->hit_plane_v, &bi);
		if (bi.v_id != -1) {
			result->peer[1].hit_part = CCT_SWEEP_HIT_POINT;
			result->peer[1].id = rface->mesh_v_ids[bi.v_id];
		}
		else if (bi.edge_id != -1) {
			result->peer[1].hit_part = CCT_SWEEP_HIT_EDGE;
			result->peer[1].id = rface->mesh_edge_ids[bi.edge_id];
		}
		else {
			result->peer[1].hit_part = CCT_SWEEP_HIT_FACE;
			result->peer[1].id = rface - mesh->polygons;
		}
		return result;
	}
	return Ray_Sweep_MeshSegment(o, dir, mesh, result);
}

static CCTSweepResult_t* Ray_Sweep_Capsule(const CCTNum_t o[3], const CCTNum_t dir[3], const GeometryCapsule_t* capsule, int check_intersect, CCTSweepResult_t* result) {
	CCTNum_t v[3];
	CCTNum_t d, cos_theta;
	if (check_intersect && Capsule_Contain_Point(capsule, o)) {
		set_intersect(result);
		return result;
	}
	mathPointProjectionLine(o, capsule->o, capsule->axis, v);
	d = mathVec3DistanceSq(o, v);
	if (d <= CCTNum_sq(capsule->radius)) {
		/* point locate in infinite cylinder */
		mathVec3Sub(v, o, capsule->o);
		d = mathVec3Dot(v, capsule->axis);
	}
	else {
		CCTNum_t N[3];
		mathVec3Cross(N, dir, capsule->axis);
		mathVec3Sub(v, o, capsule->o);
		if (mathVec3IsZero(N)) {
			/* Line vs Line parallel */
			d = mathVec3Dot(v, capsule->axis);
		}
		else {
			/* Line vs Line opposite or cross */
			d = mathVec3Dot(v, N);
			if (d < CCT_EPSILON_NEGATE || d > CCT_EPSILON) {
				CCTNum_t closest_p[2][3];
				CCTNum_t od, cd, lensq, radius_sq;
				mathLineClosestLine_opposite(o, dir, capsule->o, capsule->axis, &od, &cd);
				if (od < CCTNum(0.0)) {
					return NULL;
				}
				mathVec3Copy(closest_p[0], o);
				mathVec3AddScalar(closest_p[0], dir, od);
				mathVec3Copy(closest_p[1], capsule->o);
				mathVec3AddScalar(closest_p[1], capsule->axis, cd);
				lensq = mathVec3DistanceSq(closest_p[0], closest_p[1]);
				radius_sq = CCTNum_sq(capsule->radius);
				if (lensq > radius_sq + CCT_EPSILON) {
					return NULL;
				}
				else if (lensq < radius_sq) {
					CCTNum_t sin_theta;
					d = CCTNum_sqrt(radius_sq - lensq);
					mathVec3Cross(v, dir, capsule->axis);
					sin_theta = mathVec3Normalized(v, v);
					d /= sin_theta;
					d = od - d;
				}
				else {
					d = od;
				}
			}
			else {
				CCTNum_t lensq;
				d = mathLineCrossLine(o, dir, capsule->o, capsule->axis);
				if (d < CCTNum(0.0)) {
					return NULL;
				}
				cos_theta = mathVec3Dot(dir, capsule->axis);
				lensq = CCTNum_sq(capsule->radius) / (1 - CCTNum_sq(cos_theta));
				d -= CCTNum_sqrt(lensq);
			}
			result->distance = d;
			mathVec3Copy(result->hit_plane_v, o);
			mathVec3AddScalar(result->hit_plane_v, dir, d);
			mathVec3Sub(v, result->hit_plane_v, capsule->o);
			d = mathVec3Dot(v, capsule->axis);
			if (CCTNum_abs(d) <= capsule->half + CCT_EPSILON) {
				mathPointProjectionLine(result->hit_plane_v, capsule->o, capsule->axis, v);
				mathVec3Sub(result->hit_plane_n, result->hit_plane_v, v);
				mathVec3Normalized(result->hit_plane_n, result->hit_plane_n);
				result->hit_unique_point = 1;
				result->overlap = 0;
				result->peer[0].hit_part = CCT_SWEEP_HIT_POINT;
				result->peer[0].id = 0;
				result->peer[1].hit_part = 0;
				result->peer[1].id = 0;
				return result;
			}
		}
	}
	mathVec3Copy(v, capsule->o);
	if (d > CCTNum(0.0)) {
		mathVec3AddScalar(v, capsule->axis, capsule->half);
	}
	else {
		mathVec3SubScalar(v, capsule->axis, capsule->half);
	}
	if (!Ray_Sweep_Sphere(o, dir, v, capsule->radius, result)) {
		return NULL;
	}
	result->peer[1].id = (d > CCTNum(0.0) ? 1 : 0);
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
			CCTNum_t l[3], r[3], d;
			unsigned int closest_ls1_indice, closest_ls2_indice;

			mathVec3Sub(l, ls1[0], ls2[0]);
			mathVec3Sub(r, ls1[1], ls2[0]);
			d = mathVec3Dot(l, r);
			if (d <= CCTNum(0.0)) {
				set_intersect(result);
				return result;
			}
			mathVec3Sub(l, ls1[0], ls2[1]);
			mathVec3Sub(r, ls1[1], ls2[1]);
			d = mathVec3Dot(l, r);
			if (d <= CCTNum(0.0)) {
				set_intersect(result);
				return result;
			}
			mathVec3Sub(l, ls2[0], ls1[0]);
			mathVec3Sub(r, ls2[1], ls1[0]);
			d = mathVec3Dot(l, r);
			if (d <= CCTNum(0.0)) {
				set_intersect(result);
				return result;
			}
			mathVec3Sub(l, ls2[0], ls1[1]);
			mathVec3Sub(r, ls2[1], ls1[1]);
			d = mathVec3Dot(l, r);
			if (d <= CCTNum(0.0)) {
				set_intersect(result);
				return result;
			}
			/* dir and ls1_dir must parallel */
			mathVec3Cross(N, ls1_dir, dir);
			if (!mathVec3IsZero(N)) {
				return NULL;
			}
			/* calculate result */
			Segment_ClosestVertexIndices_Segment(ls1, ls2, &closest_ls1_indice, &closest_ls2_indice);
			mathVec3Sub(N, ls2[closest_ls2_indice], ls1[closest_ls1_indice]);
			d = mathVec3Dot(N, dir);
			if (d <= CCTNum(0.0)) {
				return NULL;
			}
			mathVec3Copy(result->hit_plane_v, ls2[closest_ls2_indice]);
			mathVec3Negate(result->hit_plane_n, dir);
			result->hit_unique_point = 1;
			result->overlap = 0;
			result->distance = d;
			result->peer[0].hit_part = CCT_SWEEP_HIT_POINT;
			result->peer[0].id = closest_ls1_indice;
			result->peer[1].hit_part = CCT_SWEEP_HIT_POINT;
			result->peer[1].id = closest_ls2_indice;
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
						result->hit_unique_point = 1;
						result->peer[0].hit_part = CCT_SWEEP_HIT_POINT;
						result->peer[0].id = i;
						result->peer[1].hit_part = CCT_SWEEP_HIT_POINT;
						result->peer[1].id = j;
					}
					else {
						result->hit_unique_point = 0;
						result->peer[0].hit_part = CCT_SWEEP_HIT_EDGE;
						result->peer[0].id = 0;
						result->peer[1].hit_part = CCT_SWEEP_HIT_EDGE;
						result->peer[1].id = 0;
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
				result->hit_unique_point = 0;
				result->peer[0].hit_part = CCT_SWEEP_HIT_EDGE;
				result->peer[0].id = 0;
				result->peer[1].hit_part = CCT_SWEEP_HIT_EDGE;
				result->peer[1].id = 0;
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
					result->hit_unique_point = 0;
					result->peer[0].hit_part = CCT_SWEEP_HIT_EDGE;
					result->peer[0].id = 0;
					result->peer[1].hit_part = CCT_SWEEP_HIT_EDGE;
					result->peer[1].id = 0;
					break;
				}
			}
			if (!p_result) {
				return NULL;
			}
			mathVec3Negate(result->hit_plane_n, N);
			result->distance = d;
			result->overlap = 0;
			return result;
		}
	}
	else {
		CCTNum_t p[3], vp[3];
		d = mathVec3Dot(v, N);
		if (d < CCT_EPSILON_NEGATE || d > CCT_EPSILON) {
			/* opposite */
			mathVec3Cross(p, ls1_dir, dir);
			if (mathVec3IsZero(p)) {
				return NULL;
			}
			mathVec3Normalized(p, p);
			if (Segment_Intersect_Plane(ls2, ls1[0], p, v, NULL) != 1) {
				return NULL;
			}
			mathVec3Normalized(ls1_dir, ls1_dir);
			mathPointProjectionLine(v, ls1[0], ls1_dir, p);
			mathVec3Sub(p, v, p);
			d = mathVec3Normalized(p, p);
			cos_theta = mathVec3Dot(p, dir);
			if (cos_theta <= CCTNum(0.0)) {
				return NULL;
			}
			d /= cos_theta;
			mathVec3Copy(p, v);
			mathVec3SubScalar(p, dir, d);
			if (mathVec3Equal(ls1[0], p)) {
				result->peer[0].hit_part = CCT_SWEEP_HIT_POINT;
				result->peer[0].id = 0;
			}
			else if (mathVec3Equal(ls1[1], p)) {
				result->peer[0].hit_part = CCT_SWEEP_HIT_POINT;
				result->peer[0].id = 1;
			}
			else {
				CCTNum_t vr[3];
				mathVec3Sub(vr, ls1[0], p);
				mathVec3Sub(vp, ls1[1], p);
				cos_theta = mathVec3Dot(vr, vp);
				if (cos_theta > CCT_EPSILON) {
					return NULL;
				}
				result->peer[0].hit_part = CCT_SWEEP_HIT_EDGE;
				result->peer[0].id = 0;
			}
			mathVec3Copy(result->hit_plane_v, v);
			mathVec3Normalized(result->hit_plane_n, N);
			if (mathVec3Dot(result->hit_plane_n, dir) > CCTNum(0.0)) {
				mathVec3Negate(result->hit_plane_n, result->hit_plane_n);
			}
			result->hit_unique_point = 1;
			result->overlap = 0;
			result->distance = d;
			if (mathVec3Equal(v, ls2[0])) {
				result->peer[1].hit_part = CCT_SWEEP_HIT_POINT;
				result->peer[1].id = 0;
			}
			else if (mathVec3Equal(v, ls2[1])) {
				result->peer[1].hit_part = CCT_SWEEP_HIT_POINT;
				result->peer[1].id = 1;
			}
			else {
				result->peer[1].hit_part = CCT_SWEEP_HIT_EDGE;
				result->peer[1].id = 0;
			}
			return result;
		}
		else {
			CCTNum_t hn[3], ls1_len, hn_len;
			/* cross */
			if (mathVec3IsZero(v)) {
				set_intersect(result);
				return result;
			}
			/* calculate Line cross point */
			ls1_len = mathVec3Normalized(ls1_dir, ls1_dir);
			mathVec3Normalized(ls2_dir, ls2_dir);
			mathPointProjectionLine(ls1[0], ls2[0], ls2_dir, p);
			if (mathVec3Equal(p, ls1[0])) {
				/* ls1[0] is cross point */
				hn_len = d = CCTNum(0.0);
				mathVec3Copy(hn, dir);
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
			if (cos_theta <= CCTNum(0.0)) {
				/* check cross point locate ls1 */
				if (d >= CCTNum(0.0) && d <= ls1_len) {
					set_intersect(result);
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
						result->peer[0].hit_part = CCT_SWEEP_HIT_POINT;
						result->peer[0].id = 0;
					}
					else {
						result->peer[0].hit_part = CCT_SWEEP_HIT_POINT;
						result->peer[0].id = 1;
					}
					mathVec3Copy(result->hit_plane_v, p);
					mathVec3Negate(result->hit_plane_n, hn);
					result->hit_unique_point = 1;
					result->overlap = 0;
					result->distance = d;
					if (mathVec3Equal(p, ls2[0])) {
						result->peer[1].hit_part = CCT_SWEEP_HIT_POINT;
						result->peer[1].id = 0;
					}
					else if (mathVec3Equal(p, ls2[1])) {
						result->peer[1].hit_part = CCT_SWEEP_HIT_POINT;
						result->peer[1].id = 1;
					}
					else {
						result->peer[1].hit_part = CCT_SWEEP_HIT_EDGE;
						result->peer[1].id = 0;
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
				mathVec3Negate(result->hit_plane_n, hn);
				result->hit_unique_point = 1;
				result->distance = hn_len;
				result->peer[0].hit_part = CCT_SWEEP_HIT_POINT;
				result->peer[0].id = 0;
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
				mathVec3Negate(result->hit_plane_n, hn);
				result->hit_unique_point = 1;
				result->distance = hn_len;
				result->peer[0].hit_part = CCT_SWEEP_HIT_POINT;
				result->peer[0].id = 1;
			} while (0);
			if (p_result) {
				if (mathVec3Equal(result->hit_plane_v, ls2[0])) {
					result->peer[1].hit_part = CCT_SWEEP_HIT_POINT;
					result->peer[1].id = 0;
				}
				else if (mathVec3Equal(result->hit_plane_v, ls2[1])) {
					result->peer[1].hit_part = CCT_SWEEP_HIT_POINT;
					result->peer[1].id = 1;
				}
				else {
					result->peer[1].hit_part = CCT_SWEEP_HIT_EDGE;
					result->peer[1].id = 0;
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
					result->peer[0].hit_part = CCT_SWEEP_HIT_POINT;
					result->peer[0].id = 0;
				}
				else if (mathVec3Equal(ls1[1], p)) {
					result->peer[0].hit_part = CCT_SWEEP_HIT_POINT;
					result->peer[0].id = 1;
				}
				else {
					mathVec3Sub(v, ls1[0], p);
					mathVec3Sub(vp, ls1[1], p);
					cos_theta = mathVec3Dot(v, vp);
					if (cos_theta > CCT_EPSILON) {
						continue;
					}
					result->peer[0].hit_part = CCT_SWEEP_HIT_EDGE;
					result->peer[0].id = 0;
				}
				p_result = result;
				mathVec3Copy(result->hit_plane_v, ls2[i]);
				mathVec3Negate(result->hit_plane_n, hn);
				result->hit_unique_point = 1;
				result->distance = hn_len;
				result->peer[1].hit_part = CCT_SWEEP_HIT_POINT;
				result->peer[1].id = i;
			}
			if (p_result) {
				p_result->overlap = 0;
			}
			return p_result;
		}
	}
}

static int merge_mesh_hit_info(CCTSweepHitInfo_t* dst_info, const CCTSweepHitInfo_t* src_info, const GeometryMesh_t* mesh) {
	unsigned int part_id, idx, v_idx[3];
	if (dst_info->hit_part == CCT_SWEEP_HIT_POINT && src_info->hit_part == CCT_SWEEP_HIT_POINT) {
		unsigned int mesh_edge_v_indices_cnt;
		if (dst_info->id == src_info->id) {
			return 0;
		}
		v_idx[0] = mesh->v_indices[dst_info->id];
		v_idx[1] = mesh->v_indices[src_info->id];
		mesh_edge_v_indices_cnt = mesh->edge_cnt + mesh->edge_cnt;
		part_id = mathFindEdgeIdByVertexIndices(mesh->edge_v_indices_flat, mesh_edge_v_indices_cnt, v_idx[0], v_idx[1]);
		if (part_id != -1) {
			dst_info->hit_part = CCT_SWEEP_HIT_EDGE;
			dst_info->id = part_id;
			return 1;
		}
		part_id = mathFindFaceIdByVertexIndices(mesh->polygons, mesh->polygons_cnt, v_idx, 2);
		if (part_id != -1) {
			dst_info->hit_part = CCT_SWEEP_HIT_FACE;
			dst_info->id = part_id;
			return 1;
		}
		dst_info->hit_part = 0;
		dst_info->id = 0;
		return 1;
	}
	if (dst_info->hit_part == CCT_SWEEP_HIT_EDGE && src_info->hit_part == CCT_SWEEP_HIT_EDGE) {
		if (dst_info->id == src_info->id) {
			return 0;
		}
		idx = 2 * dst_info->id;
		v_idx[0] = mesh->edge_v_indices_flat[idx++];
		v_idx[1] = mesh->edge_v_indices_flat[idx];
		idx = 2 * src_info->id;
		v_idx[2] = mesh->edge_v_indices_flat[idx++];
		if (v_idx[2] == v_idx[0] || v_idx[2] == v_idx[1]) {
			v_idx[2] = mesh->edge_v_indices_flat[idx];
		}
		part_id = mathFindFaceIdByVertexIndices(mesh->polygons, mesh->polygons_cnt, v_idx, 3);
		if (part_id != -1) {
			dst_info->hit_part = CCT_SWEEP_HIT_FACE;
			dst_info->id = part_id;
			return 1;
		}
		dst_info->hit_part = 0;
		dst_info->id = 0;
		return 1;
	}
	if (dst_info->hit_part == CCT_SWEEP_HIT_POINT && src_info->hit_part == CCT_SWEEP_HIT_EDGE) {
		idx = 2 * src_info->id;
		v_idx[0] = mesh->edge_v_indices_flat[idx++];
		v_idx[1] = mesh->edge_v_indices_flat[idx];
		v_idx[2] = mesh->v_indices[dst_info->id];
		if (Segment_Contain_Point(mesh->v[v_idx[0]], mesh->v[v_idx[1]], mesh->v[v_idx[2]])) {
			*dst_info = *src_info;
			return 1;
		}
		part_id = mathFindFaceIdByVertexIndices(mesh->polygons, mesh->polygons_cnt, v_idx, 3);
		if (part_id != -1) {
			dst_info->hit_part = CCT_SWEEP_HIT_FACE;
			dst_info->id = part_id;
			return 1;
		}
		dst_info->hit_part = 0;
		dst_info->id = 0;
		return 1;
	}
	else if (dst_info->hit_part == CCT_SWEEP_HIT_EDGE && src_info->hit_part == CCT_SWEEP_HIT_POINT) {
		idx = 2 * dst_info->id;
		v_idx[0] = mesh->edge_v_indices_flat[idx++];
		v_idx[1] = mesh->edge_v_indices_flat[idx];
		v_idx[2] = mesh->v_indices[src_info->id];
		if (Segment_Contain_Point(mesh->v[v_idx[0]], mesh->v[v_idx[1]], mesh->v[v_idx[2]])) {
			return 0;
		}
		part_id = mathFindFaceIdByVertexIndices(mesh->polygons, mesh->polygons_cnt, v_idx, 3);
		if (idx != -1) {
			dst_info->hit_part = CCT_SWEEP_HIT_FACE;
			dst_info->id = part_id;
			return 1;
		}
		dst_info->hit_part = 0;
		dst_info->id = 0;
		return 1;
	}
	if (dst_info->hit_part != 0 && src_info->hit_part == CCT_SWEEP_HIT_FACE) {
		if (dst_info->hit_part < src_info->hit_part) {
			*dst_info = *src_info;
			return 1;
		}
		else if (dst_info->hit_part > src_info->hit_part) {
			dst_info->hit_part = 0;
			dst_info->id = 0;
			return 1;
		}
		else if (dst_info->id != src_info->id) {
			dst_info->hit_part = 0;
			dst_info->id = 0;
			return 1;
		}
		return 0;
	}
	return 0;
}

static CCTSweepResult_t* MeshSegment_Sweep_MeshSegment(const GeometryMesh_t* s1, const CCTNum_t dir[3], const GeometryMesh_t* s2, CCTSweepResult_t* result) {
	unsigned int i, j;
	unsigned int s1_edge_v_indices_cnt, s2_edge_v_indices_cnt = s2->edge_cnt + s2->edge_cnt;
	CCTSweepResult_t result_temp;
	CCTSweepResult_t* p_result;
	if (s2_edge_v_indices_cnt < 2) {
		return NULL;
	}
	p_result = NULL;
	s1_edge_v_indices_cnt = s1->edge_cnt + s1->edge_cnt;
	for (i = 0; i < s1_edge_v_indices_cnt; ) {
		CCTNum_t edge1[2][3];
		unsigned int v_idx1[2];
		v_idx1[0] = s1->edge_v_indices_flat[i++];
		v_idx1[1] = s1->edge_v_indices_flat[i++];
		mathVec3Copy(edge1[0], s1->v[v_idx1[0]]);
		mathVec3Copy(edge1[1], s1->v[v_idx1[1]]);
		for (j = 0; j < s2_edge_v_indices_cnt; ) {
			CCTNum_t edge2[2][3];
			unsigned int v_idx2[2];
			v_idx2[0] = s2->edge_v_indices_flat[j++];
			v_idx2[1] = s2->edge_v_indices_flat[j++];
			mathVec3Copy(edge2[0], s2->v[v_idx2[0]]);
			mathVec3Copy(edge2[1], s2->v[v_idx2[1]]);
			if (!Segment_Sweep_Segment((const CCTNum_t(*)[3])edge1, dir, (const CCTNum_t(*)[3])edge2, &result_temp)) {
				continue;
			}
			if (result_temp.overlap) {
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
				if (result_temp.peer[0].hit_part == CCT_SWEEP_HIT_POINT) {
					if (result_temp.peer[0].id) {
						result_temp.peer[0].id = s1->edge_v_ids_flat[i - 1];
					}
					else {
						result_temp.peer[0].id = s1->edge_v_ids_flat[i - 2];
					}
				}
				else {
					result_temp.peer[0].id = (i - 1) / 2;
				}
				if (result_temp.peer[1].hit_part == CCT_SWEEP_HIT_POINT) {
					if (result_temp.peer[1].id) {
						result_temp.peer[1].id = s2->edge_v_ids_flat[j - 1];
					}
					else {
						result_temp.peer[1].id = s2->edge_v_ids_flat[j - 2];
					}
				}
				else {
					result_temp.peer[1].id = (j - 1) / 2;
				}
				merge_mesh_hit_info(&result->peer[0], &result_temp.peer[0], s1);
				merge_mesh_hit_info(&result->peer[1], &result_temp.peer[1], s2);
				if (result->peer[0].hit_part != CCT_SWEEP_HIT_POINT && result->peer[1].hit_part != CCT_SWEEP_HIT_POINT) {
					result->hit_unique_point = 0;
				}
				continue;
			}
			if (result_temp.peer[0].hit_part == CCT_SWEEP_HIT_POINT) {
				if (result_temp.peer[0].id) {
					result->peer[0].id = s1->edge_v_ids_flat[i - 1];
				}
				else {
					result->peer[0].id = s1->edge_v_ids_flat[i - 2];
				}
			}
			else {
				result->peer[0].id = (i - 1) / 2;
			}
			if (result_temp.peer[1].hit_part == CCT_SWEEP_HIT_POINT) {
				if (result_temp.peer[1].id) {
					result->peer[1].id = s2->edge_v_ids_flat[j - 1];
				}
				else {
					result->peer[1].id = s2->edge_v_ids_flat[j - 2];
				}
			}
			else {
				result->peer[1].id = (j - 1) / 2;
			}
		}
	}
	return p_result;
}

static CCTSweepResult_t* Mesh_Sweep_Mesh_InternalProc(const GeometryMesh_t* mesh1, const CCTNum_t dir[3], const GeometryMesh_t* mesh2, CCTSweepResult_t* result) {
	unsigned int i;
	CCTNum_t neg_dir[3];
	GeometryBorderId_t bi;
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
				if (!Polygon_Contain_Point_SamePlane(polygon2, result_temp.hit_plane_v, &bi)) {
					continue;
				}
				p_result = result;
			}
			else if (result_temp.distance > result->distance + CCT_EPSILON) {
				continue;
			}
			else if (result_temp.distance < result->distance - CCT_EPSILON) {
				if (!Polygon_Contain_Point_SamePlane(polygon2, result_temp.hit_plane_v, &bi)) {
					continue;
				}
			}
			else {
				if (!Polygon_Contain_Point_SamePlane(polygon2, result_temp.hit_plane_v, &bi)) {
					continue;
				}
				if (result_temp.distance < result->distance) {
					result->distance = result_temp.distance;
				}
				if (bi.v_id != -1 || bi.edge_id != -1) {
					continue;
				}
				result_temp.peer[0].id = j;
				result_temp.peer[1].id = i;
				merge_mesh_hit_info(&result->peer[0], &result_temp.peer[0], mesh1);
				merge_mesh_hit_info(&result->peer[1], &result_temp.peer[1], mesh2);
				result->hit_unique_point = 0;
				continue;
			}
			*result = result_temp;
			result->peer[0].id = j;
			if (bi.v_id != -1) {
				result->peer[1].hit_part = CCT_SWEEP_HIT_POINT;
				if (polygon2->mesh_v_ids && mesh2->polygons_cnt > 1) {
					result->peer[1].id = polygon2->mesh_v_ids[bi.v_id];
				}
				else {
					result->peer[1].id = bi.v_id;
				}
			}
			else if (bi.edge_id != -1) {
				result->peer[1].hit_part = CCT_SWEEP_HIT_EDGE;
				if (polygon2->mesh_edge_ids && mesh2->polygons_cnt > 1) {
					result->peer[1].id = polygon2->mesh_edge_ids[bi.edge_id];
				}
				else {
					result->peer[1].id = bi.edge_id;
				}
			}
			else {
				result->peer[1].id = i;
			}
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
				if (!Polygon_Contain_Point_SamePlane(polygon1, result_temp.hit_plane_v, &bi)) {
					continue;
				}
				p_result = result;
			}
			else if (result_temp.distance > result->distance + CCT_EPSILON) {
				continue;
			}
			else if (result_temp.distance < result->distance - CCT_EPSILON) {
				if (!Polygon_Contain_Point_SamePlane(polygon1, result_temp.hit_plane_v, &bi)) {
					continue;
				}
			}
			else {
				if (!Polygon_Contain_Point_SamePlane(polygon1, result_temp.hit_plane_v, &bi)) {
					continue;
				}
				if (result_temp.distance < result->distance) {
					result->distance = result_temp.distance;
				}
				if (bi.v_id != -1 || bi.edge_id != -1) {
					continue;
				}
				mathVec3Copy(result_temp.hit_plane_v, mesh2_p);
				result_temp.peer[0].hit_part = CCT_SWEEP_HIT_FACE;
				result_temp.peer[0].id = i;
				result_temp.peer[1].hit_part = CCT_SWEEP_HIT_POINT;
				result_temp.peer[1].id = j;
				merge_mesh_hit_info(&result->peer[0], &result_temp.peer[0], mesh1);
				merge_mesh_hit_info(&result->peer[1], &result_temp.peer[1], mesh2);
				result->hit_unique_point = 0;
				continue;
			}
			*result = result_temp;
			mathVec3Copy(result->hit_plane_v, mesh2_p);
			if (bi.v_id != -1) {
				result->peer[0].hit_part = CCT_SWEEP_HIT_POINT;
				if (polygon1->mesh_v_ids && mesh1->polygons_cnt > 1) {
					result->peer[0].id = polygon1->mesh_v_ids[bi.v_id];
				}
				else {
					result->peer[0].id = bi.v_id;
				}
			}
			else if (bi.edge_id != -1) {
				result->peer[0].hit_part = CCT_SWEEP_HIT_EDGE;
				if (polygon1->mesh_edge_ids && mesh1->polygons_cnt > 1) {
					result->peer[0].id = polygon1->mesh_edge_ids[bi.edge_id];
				}
				else {
					result->peer[0].id = bi.edge_id;
				}
			}
			else {
				result->peer[0].hit_part = CCT_SWEEP_HIT_FACE;
				result->peer[0].id = i;
			}
			result->peer[1].hit_part = CCT_SWEEP_HIT_POINT;
			result->peer[1].id = mesh2->v_indices[j];
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
				result->peer[0].hit_part = CCT_SWEEP_HIT_POINT;
				result->peer[0].id = 0;
				break;
			}
			mathVec3Copy(v1, ls[1]);
			mathVec3AddScalar(v1, dir, d);
			mathVec3Sub(v1, v1, p);
			if (mathVec3IsZero(v1)) {
				result->peer[0].hit_part = CCT_SWEEP_HIT_POINT;
				result->peer[0].id = 1;
				break;
			}
			dot = mathVec3Dot(v0, v1);
			if (dot <= CCTNum(0.0)) {
				result->peer[0].hit_part = CCT_SWEEP_HIT_EDGE;
				result->peer[0].id = 0;
				break;
			}
			hit_ok = 0;
		} while (0);
		if (hit_ok) {
			mathVec3Copy(result->hit_plane_v, p);
			mathVec3Negate(result->hit_plane_n, pco);
			result->hit_unique_point = 1;
			result->overlap = 0;
			result->distance = d;
			result->peer[1].hit_part = 0;
			result->peer[1].id = 0;
			return result;
		}
	}
	if (Ray_Sweep_Sphere(ls[0], dir, circle_o, circle_r, result)) {
		CCTSweepResult_t result_temp;
		if (Ray_Sweep_Sphere(ls[1], dir, circle_o, circle_r, &result_temp) &&
			result_temp.distance < result->distance)
		{
			*result = result_temp;
			result->peer[0].id = 1;
		}
		return result;
	}
	if (!Ray_Sweep_Sphere(ls[1], dir, circle_o, circle_r, result)) {
		return NULL;
	}
	result->peer[0].id = 1;
	return result;
}

static CCTSweepResult_t* Segment_Sweep_Sphere(const CCTNum_t ls[2][3], const CCTNum_t dir[3], const CCTNum_t center[3], CCTNum_t radius, int check_intersect, CCTSweepResult_t* result) {
	CCTNum_t lsdir[3], N[3];
	if (check_intersect && Sphere_Intersect_Segment(center, radius, ls[0], ls[1])) {
		set_intersect(result);
		return result;
	}
	mathVec3Sub(lsdir, ls[1], ls[0]);
	mathVec3Cross(N, lsdir, dir);
	if (!mathVec3IsZero(N)) {
		int res;
		CCTNum_t circle_o[3], circle_r;
		mathVec3Normalized(N, N);
		res = Sphere_Intersect_Plane(center, radius, ls[0], N, circle_o, &circle_r);
		if (0 == res) {
			return NULL;
		}
		if (1 == res) {
			CCTNum_t v[3], l[3], p[3], pco[3];
			CCTNum_t d, dot;
			mathVec3Normalized(lsdir, lsdir);
			mathPointProjectionLine(circle_o, ls[0], lsdir, p);
			mathVec3Sub(pco, circle_o, p);
			d = mathVec3Normalized(pco, pco);
			dot = mathVec3Dot(pco, dir);
			if (dot <= CCTNum(0.0)) {
				return NULL;
			}
			d /= dot;
			mathVec3Copy(p, circle_o);
			mathVec3SubScalar(p, dir, d);
			mathVec3Sub(l, ls[0], p);
			mathVec3Sub(v, ls[1], p);
			dot = mathVec3Dot(l, v);
			if (dot > CCT_EPSILON) {
				return NULL;
			}
			result->distance = d;
			result->overlap = 0;
			result->hit_unique_point = 1;
			mathVec3Copy(result->hit_plane_v, circle_o);
			mathVec3Sub(v, center, circle_o);
			if (mathVec3Dot(v, N) > CCTNum(0.0)) {
				mathVec3Negate(result->hit_plane_n, N);
			}
			else {
				mathVec3Copy(result->hit_plane_n, N);
			}
			if (dot < CCTNum(0.0)) {
				result->peer[0].hit_part = CCT_SWEEP_HIT_EDGE;
				result->peer[0].id = 0;
			}
			else {
				result->peer[0].hit_part = CCT_SWEEP_HIT_POINT;
				result->peer[0].id = (mathVec3IsZero(l) ? 0 : 1);
			}
			result->peer[1].hit_part = CCT_SWEEP_HIT_SPHERE;
			result->peer[1].id = 0;
			return result;
		}
		if (!Segment_Sweep_Circle_InSamePlane(ls, dir, circle_o, circle_r, result)) {
			return NULL;
		}
		mathVec3Sub(result->hit_plane_n, result->hit_plane_v, center);
		mathVec3Normalized(result->hit_plane_n, result->hit_plane_n);
		result->peer[1].hit_part = CCT_SWEEP_HIT_SPHERE;
		result->peer[1].id = 0;
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
			result->peer[0].id = 1;
		}
		return result;
	}
	return NULL;
}

static CCTSweepResult_t* Segment_Sweep_Capsule(const CCTNum_t ls[2][3], const CCTNum_t ls_dir[3], CCTNum_t ls_len, const CCTNum_t dir[3], const GeometryCapsule_t* capsule, int check_intersect, CCTSweepResult_t* result) {
	CCTNum_t ls_dir_temp[3], axis_edge[2][3];
	CCTNum_t plane_n[3], v[3], cos_theta;
	CCTNum_t radius_sq = CCTNum_sq(capsule->radius);
	CCTSweepResult_t result_temp;
	CCTSweepResult_t* p_result;

	if (!ls_dir) {
		ls_dir = ls_dir_temp;
		mathVec3Sub(ls_dir_temp, ls[1], ls[0]);
		ls_len = mathVec3Normalized(ls_dir_temp, ls_dir_temp);
	}
	mathTwoVertexFromCenterHalf(capsule->o, capsule->axis, capsule->half, axis_edge[0], axis_edge[1]);
	if (check_intersect) {
		CCTNum_t lensq = Segment_ClosestLenSq_Segment(
			ls, ls_dir, ls_len,
			(const CCTNum_t(*)[3])axis_edge, capsule->axis, capsule->half + capsule->half
		);
		if (lensq <= radius_sq) {
			set_intersect(result);
			return result;
		}
	}
	mathVec3Cross(plane_n, ls_dir, dir);
	if (mathVec3IsZero(plane_n)) {
		/* no sweep plane, Ray vs Capsule */
		cos_theta = mathVec3Dot(ls_dir, dir);
		if (cos_theta > CCTNum(0.0)) {
			if (!Ray_Sweep_Capsule(ls[1], dir, capsule, 0, result)) {
				return NULL;
			}
			result->peer[0].id = 1;
			return result;
		}
		return Ray_Sweep_Capsule(ls[0], dir, capsule, 0, result);
	}
	mathVec3Normalized(plane_n, plane_n);
	cos_theta = mathVec3Dot(plane_n, capsule->axis);
	if (CCT_EPSILON_NEGATE <= cos_theta && cos_theta <= CCT_EPSILON) {
		/* capsule parallel sweep plane */
		CCTNum_t temp_ls[2][3];
		CCTNum_t d, abs_d;
		mathVec3Sub(v, capsule->o, ls[0]);
		mathVec3Cross(v, v, capsule->axis);
		if (mathVec3IsZero(v)) {
			unsigned int v_idx, s_idx;
			Segment_ClosestVertexIndices_Segment(ls, (const CCTNum_t(*)[3])axis_edge, &v_idx, &s_idx);
			if (!Ray_Sweep_Sphere(ls[v_idx], dir, axis_edge[s_idx], capsule->radius, result)) {
				return NULL;
			}
			result->peer[0].id = v_idx;
			result->peer[1].id = s_idx;
			return result;
		}
		d = mathPointProjectionPlane(axis_edge[0], ls[0], plane_n);
		abs_d = CCTNum_abs(d);
		if (abs_d > capsule->radius + CCT_EPSILON) {
			return NULL;
		}
		mathVec3Copy(temp_ls[0], axis_edge[0]);
		mathVec3Copy(temp_ls[1], axis_edge[1]);
		if (CCT_EPSILON_NEGATE <= d && d <= CCT_EPSILON) {
			d = CCTNum(0.0);
		}
		else {
			mathVec3AddScalar(temp_ls[0], plane_n, d);
			mathVec3AddScalar(temp_ls[1], plane_n, d);
		}
		if (abs_d >= capsule->radius) {
			if (!Segment_Sweep_Segment(ls, dir, (const CCTNum_t(*)[3])temp_ls, result)) {
				return NULL;
			}
			mathVec3Sub(v, ls[0], capsule->o);
			if (mathVec3Dot(v, plane_n) > CCTNum(0.0)) {
				mathVec3Copy(result->hit_plane_n, plane_n);
			}
			else {
				mathVec3Negate(result->hit_plane_n, plane_n);
			}
			if (result->peer[1].hit_part == CCT_SWEEP_HIT_POINT) {
				result->peer[1].hit_part = CCT_SWEEP_HIT_SPHERE;
			}
			return result;
		}
		if (d != CCTNum(0.0)) {
			d = CCTNum_sqrt(radius_sq - CCTNum_sq(d));
		}
		else {
			d = capsule->radius;
		}
		mathVec3Cross(v, capsule->axis, plane_n);
		mathVec3AddScalar(temp_ls[0], v, d);
		mathVec3AddScalar(temp_ls[1], v, d);
		p_result = NULL;
		if (Segment_Sweep_Segment(ls, dir, (const CCTNum_t(*)[3])temp_ls, &result_temp)) {
			*result = result_temp;
			p_result = result;
			mathPointProjectionLine_v2(capsule->o, temp_ls[0], temp_ls[1], v);
			mathVec3Sub(result->hit_plane_n, v, capsule->o);
			mathVec3Normalized(result->hit_plane_n, result->hit_plane_n);
			if (result->peer[1].hit_part == CCT_SWEEP_HIT_POINT) {
				result->peer[1].hit_part = CCT_SWEEP_HIT_SPHERE;
			}
			else {
				result->peer[1].hit_part = 0;
			}
		}
		mathVec3SubScalar(temp_ls[0], v, d + d);
		mathVec3SubScalar(temp_ls[1], v, d + d);
		if (Segment_Sweep_Segment(ls, dir, (const CCTNum_t(*)[3])temp_ls, &result_temp)) {
			if (!p_result || result_temp.distance < result->distance) {
				*result = result_temp;
				p_result = result;
				mathPointProjectionLine_v2(capsule->o, temp_ls[0], temp_ls[1], v);
				mathVec3Sub(result->hit_plane_n, v, capsule->o);
				mathVec3Normalized(result->hit_plane_n, result->hit_plane_n);
				if (result->peer[1].hit_part == CCT_SWEEP_HIT_POINT) {
					result->peer[1].hit_part = CCT_SWEEP_HIT_SPHERE;
				}
				else {
					result->peer[1].hit_part = 0;
				}
			}
		}
		if (Segment_Sweep_Sphere(ls, dir, axis_edge[0], capsule->radius, 0, &result_temp)) {
			if (!p_result) {
				*result = result_temp;
				p_result = result;
			}
			else if (result_temp.distance < result->distance - CCT_EPSILON) {
				*result = result_temp;
			}
			else if (result_temp.distance <= result->distance + CCT_EPSILON) {
				if (result_temp.distance < result->distance) {
					result->distance = result_temp.distance;
				}
				if (result->peer[1].hit_part == CCT_SWEEP_HIT_POINT) {
					result->peer[1].hit_part = CCT_SWEEP_HIT_SPHERE;
					result->peer[1].id = 0;
				}
			}
		}
		if (Segment_Sweep_Sphere(ls, dir, axis_edge[1], capsule->radius, 0, &result_temp)) {
			if (!p_result) {
				*result = result_temp;
				p_result = result;
				result->peer[1].id = 1;
			}
			else if (result_temp.distance < result->distance - CCT_EPSILON) {
				*result = result_temp;
				result->peer[1].id = 1;
			}
			else if (result_temp.distance <= result->distance + CCT_EPSILON) {
				if (result_temp.distance < result->distance) {
					result->distance = result_temp.distance;
				}
				if (result->peer[1].hit_part == CCT_SWEEP_HIT_POINT) {
					result->peer[1].hit_part = CCT_SWEEP_HIT_SPHERE;
					result->peer[1].id = 1;
				}
			}
		}
		return p_result;
	}
	else {
		/* capsule not paralle sweep plane */
		CCTNum_t d[3], N[3], dot;
		int res = Segment_Intersect_Plane((const CCTNum_t(*)[3])axis_edge, ls[0], plane_n, NULL, d);
		if (2 == res) {
			/* no possible */
			return NULL;
		}
		if (0 == res && CCTNum_abs(d[2]) > capsule->radius + CCT_EPSILON) {
			return NULL;
		}
		mathVec3Cross(N, ls_dir, capsule->axis);
		mathVec3Sub(v, capsule->o, ls[0]);
		dot = mathVec3Dot(v, N);
		if (dot < CCT_EPSILON_NEGATE || dot > CCT_EPSILON) {
			CCTNum_t temp_ls[2][3];
			mathLineClosestLine_opposite_v2(capsule->o, capsule->axis, ls[0], ls_dir, temp_ls[0], temp_ls[1]);
			mathVec3Sub(v, temp_ls[1], temp_ls[0]);
			dot = mathVec3Normalized(v, v);
			if (dot > capsule->radius) {
				mathVec3Copy(temp_ls[0], axis_edge[0]);
				mathVec3AddScalar(temp_ls[0], v, capsule->radius);
				mathVec3Copy(temp_ls[1], axis_edge[1]);
				mathVec3AddScalar(temp_ls[1], v, capsule->radius);
				if (Segment_Sweep_Segment(ls, dir, (const CCTNum_t(*)[3])temp_ls, result)) {
					if (result->peer[1].hit_part == CCT_SWEEP_HIT_POINT) {
						result->peer[1].hit_part = CCT_SWEEP_HIT_SPHERE;
					}
					else {
						result->peer[1].hit_part = 0;
					}
					return result;
				}
			}
		}
		p_result = NULL;
		if (CCTNum_abs(d[0]) <= capsule->radius + CCT_EPSILON) {
			if (Segment_Sweep_Sphere(ls, dir, axis_edge[0], capsule->radius, 0, &result_temp)) {
				*result = result_temp;
				p_result = result;
			}
		}
		if (CCTNum_abs(d[1]) <= capsule->radius + CCT_EPSILON) {
			if (Segment_Sweep_Sphere(ls, dir, axis_edge[1], capsule->radius, 0, &result_temp)) {
				if (!p_result) {
					*result = result_temp;
					result->peer[1].id = 1;
					p_result = result;
				}
				else if (result_temp.distance < result->distance) {
					*result = result_temp;
					result->peer[1].id = 1;
				}
			}
		}
		if (Ray_Sweep_Capsule(ls[0], dir, capsule, 0, &result_temp)) {
			if (!p_result) {
				*result = result_temp;
				p_result = result;
			}
			else if (result_temp.distance < result->distance - CCT_EPSILON) {
				*result = result_temp;
			}
			else if (result_temp.distance < result->distance) {
				result->distance = result_temp.distance;
			}
		}
		if (Ray_Sweep_Capsule(ls[1], dir, capsule, 0, &result_temp)) {
			if (!p_result) {
				*result = result_temp;
				result->peer[0].id = 1;
				p_result = result;
			}
			else if (result_temp.distance < result->distance - CCT_EPSILON) {
				*result = result_temp;
				result->peer[0].id = 1;
			}
			else if (result_temp.distance < result->distance) {
				result->distance = result_temp.distance;
			}
		}
		return p_result;
	}
	return NULL;
}

static CCTSweepResult_t* Capsule_Sweep_Capsule(const GeometryCapsule_t* capsule1, const CCTNum_t dir[3], const GeometryCapsule_t* capsule2, int check_intersect, CCTSweepResult_t* result) {
	CCTNum_t axis_edge[2][3];
	GeometryCapsule_t new_capsule2 = *capsule2;
	new_capsule2.radius += capsule1->radius;
	mathTwoVertexFromCenterHalf(capsule1->o, capsule1->axis, capsule1->half, axis_edge[0], axis_edge[1]);
	if (!Segment_Sweep_Capsule((const CCTNum_t(*)[3])axis_edge, capsule1->axis, capsule1->half + capsule1->half, dir, &new_capsule2, check_intersect, result)) {
		return NULL;
	}
	if (result->peer[0].hit_part == CCT_SWEEP_HIT_POINT) {
		result->peer[0].hit_part = CCT_SWEEP_HIT_SPHERE;
	}
	return result;
}

static CCTSweepResult_t* MeshSegment_Sweep_Capsule(const GeometryMesh_t* mesh, const CCTNum_t dir[3], const GeometryCapsule_t* capsule, int check_intersect, CCTSweepResult_t* result) {
	unsigned int i, mesh_edge_v_indices_cnt = mesh->edge_cnt + mesh->edge_cnt;
	CCTSweepResult_t* p_result = NULL;
	for (i = 0; i < mesh_edge_v_indices_cnt; ) {
		CCTSweepResult_t result_temp;
		CCTNum_t edge[2][3];
		unsigned int v_idx[2];
		v_idx[0] = mesh->edge_v_indices_flat[i++];
		v_idx[1] = mesh->edge_v_indices_flat[i++];
		mathVec3Copy(edge[0], mesh->v[v_idx[0]]);
		mathVec3Copy(edge[1], mesh->v[v_idx[1]]);
		if (!Segment_Sweep_Capsule((const CCTNum_t(*)[3])edge, NULL, 0, dir, capsule, check_intersect, &result_temp)) {
			continue;
		}
		if (result_temp.overlap) {
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
			if (result_temp.peer[0].hit_part == CCT_SWEEP_HIT_POINT) {
				if (result_temp.peer[0].id) {
					result_temp.peer[0].id = mesh->edge_v_ids_flat[i - 1];
				}
				else {
					result_temp.peer[0].id = mesh->edge_v_ids_flat[i - 2];
				}
			}
			else {
				result_temp.peer[0].hit_part = CCT_SWEEP_HIT_EDGE;
				result_temp.peer[0].id = (i - 1) / 2;
			}
			merge_mesh_hit_info(&result->peer[0], &result_temp.peer[0], mesh);
			if (result->peer[1].hit_part != result_temp.peer[1].hit_part ||
				result->peer[1].id != result_temp.peer[1].id)
			{
				result->peer[1].hit_part = 0;
				result->hit_unique_point = 0;
			}
			continue;
		}
		if (result_temp.peer[0].hit_part == CCT_SWEEP_HIT_POINT) {
			if (result_temp.peer[0].id) {
				result->peer[0].id = mesh->edge_v_ids_flat[i - 1];
			}
			else {
				result->peer[0].id = mesh->edge_v_ids_flat[i - 2];
			}
		}
		else {
			result->peer[0].hit_part = CCT_SWEEP_HIT_EDGE;
			result->peer[0].id = (i - 1) / 2;
		}
	}
	return p_result;
}

static CCTSweepResult_t* MeshSegment_Sweep_Sphere(const GeometryMesh_t* mesh, const CCTNum_t dir[3], const CCTNum_t center[3], CCTNum_t radius, int check_intersect, CCTSweepResult_t* result) {
	unsigned int i, mesh_edge_v_indices_cnt = mesh->edge_cnt + mesh->edge_cnt;
	CCTSweepResult_t* p_result = NULL;
	for (i = 0; i < mesh_edge_v_indices_cnt; ) {
		CCTSweepResult_t result_temp;
		CCTNum_t edge[2][3];
		unsigned int v_idx[2];
		v_idx[0] = mesh->edge_v_indices_flat[i++];
		v_idx[1] = mesh->edge_v_indices_flat[i++];
		mathVec3Copy(edge[0], mesh->v[v_idx[0]]);
		mathVec3Copy(edge[1], mesh->v[v_idx[1]]);
		if (!Segment_Sweep_Sphere((const CCTNum_t(*)[3])edge, dir, center, radius, check_intersect, &result_temp)) {
			continue;
		}
		if (result_temp.overlap) {
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
			if (result_temp.peer[0].hit_part == CCT_SWEEP_HIT_POINT) {
				if (result_temp.peer[0].id) {
					result_temp.peer[0].id = mesh->edge_v_ids_flat[i - 1];
				}
				else {
					result_temp.peer[0].id = mesh->edge_v_ids_flat[i - 2];
				}
			}
			else {
				result_temp.peer[0].hit_part = CCT_SWEEP_HIT_EDGE;
				result_temp.peer[0].id = (i - 1) / 2;
			}
			merge_mesh_hit_info(&result->peer[0], &result_temp.peer[0], mesh);
			continue;
		}
		if (result_temp.peer[0].hit_part == CCT_SWEEP_HIT_POINT) {
			if (result_temp.peer[0].id) {
				result->peer[0].id = mesh->edge_v_ids_flat[i - 1];
			}
			else {
				result->peer[0].id = mesh->edge_v_ids_flat[i - 2];
			}
		}
		else {
			result->peer[0].hit_part = CCT_SWEEP_HIT_EDGE;
			result->peer[0].id = (i - 1) / 2;
		}
	}
	return p_result;
}

static CCTSweepResult_t* Mesh_Sweep_Plane(const GeometryMesh_t* mesh, const CCTNum_t dir[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTSweepResult_t* result) {
	int flag_sign = 0, flag_nohit;
	unsigned int i, same_v_ids[3], same_v_cnt = 0;
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
			if (d < result->distance) {
				result->distance = d;
			}
			if (same_v_cnt < 3) {
				same_v_ids[same_v_cnt++] = i;
			}
			continue;
		}
		same_v_ids[0] = i;
		same_v_cnt = 1;
	}
	if (flag_nohit) {
		return NULL;
	}
	result->overlap = 0;
	if (cos_theta > CCTNum(0.0)) {
		mathVec3Negate(result->hit_plane_n, plane_n);
	}
	else {
		mathVec3Copy(result->hit_plane_n, plane_n);
	}
	if (1 == same_v_cnt) {
		mathVec3Copy(result->hit_plane_v, mesh->v[mesh->v_indices[same_v_ids[0]]]);
		mathVec3AddScalar(result->hit_plane_v, dir, result->distance);
		result->hit_unique_point = 1;
		result->peer[0].hit_part = CCT_SWEEP_HIT_POINT;
		result->peer[0].id = same_v_ids[0];
	}
	else {
		unsigned int part_id, v_idx[3];
		mathVec3Copy(result->hit_plane_v, plane_v);
		result->hit_unique_point = 0;
		v_idx[0] = mesh->v_indices[same_v_ids[0]];
		v_idx[1] = mesh->v_indices[same_v_ids[1]];
		if (2 == same_v_cnt) {
			unsigned int mesh_edge_v_indices_cnt = mesh->edge_cnt + mesh->edge_cnt;
			part_id = mathFindEdgeIdByVertexIndices(mesh->edge_v_indices_flat, mesh_edge_v_indices_cnt, v_idx[0], v_idx[1]);
			if (part_id != -1) {
				result->peer[0].hit_part = CCT_SWEEP_HIT_EDGE;
				result->peer[0].id = part_id;
			}
			else {
				result->peer[0].hit_part = 0;
				result->peer[0].id = 0;
			}
		}
		else {
			v_idx[2] = mesh->v_indices[same_v_ids[2]];
			part_id = mathFindFaceIdByVertexIndices(mesh->polygons, mesh->polygons_cnt, v_idx, 3);
			if (part_id != -1) {
				result->peer[0].hit_part = CCT_SWEEP_HIT_FACE;
				result->peer[0].id = part_id;
			}
			else {
				result->peer[0].hit_part = 0;
				result->peer[0].id = 0;
			}
		}
	}
	result->peer[1].hit_part = CCT_SWEEP_HIT_FACE;
	result->peer[1].id = 0;
	return result;
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
		if (!Ray_Intersect_Plane(ls[0], dir, polygon->v[polygon->v_indices[0]], polygon->normal)) {
			return NULL;
		}
	}
	sweep_mesh_convert_from_segment(&m1, ls);
	sweep_mesh_convert_from_polygon(&m2, polygon);
	return MeshSegment_Sweep_MeshSegment(&m1, dir, &m2, result);
}

static CCTSweepResult_t* Polygon_Sweep_Polygon(const GeometryPolygon_t* polygon1, const CCTNum_t dir[3], const GeometryPolygon_t* polygon2, CCTSweepResult_t* result) {
	int plane_side;
	GeometryMesh_t m1, m2;
	if (Polygon_Intersect_Polygon(polygon1, polygon2, &plane_side)) {
		set_intersect(result);
		return result;
	}
	if (plane_side) {
		if (!Ray_Intersect_Plane(polygon1->v[polygon1->v_indices[0]], dir, polygon2->v[polygon2->v_indices[0]], polygon2->normal)) {
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
		if (!Ray_Intersect_Plane(mesh->v[mesh->v_indices[0]], dir, polygon->v[polygon->v_indices[0]], polygon->normal)) {
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
	if (OBB_Intersect_OBB(obb1, obb2)) {
		set_intersect(result);
		return result;
	}
	mathBoxMesh(&mesh1, obb1->o, obb1->half, (const CCTNum_t(*)[3])obb1->axis);
	mathBoxMesh(&mesh2, obb2->o, obb2->half, (const CCTNum_t(*)[3])obb2->axis);
	return Mesh_Sweep_Mesh_InternalProc(&mesh1.mesh, dir, &mesh2.mesh, result);
}

static CCTSweepResult_t* Capsule_Sweep_Plane(const GeometryCapsule_t* capsule, const CCTNum_t dir[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTSweepResult_t* result) {
	unsigned int idx;
	CCTNum_t cos_theta;
	CCTNum_t axis_edge[2][3], d[2], abs_d[2];
	mathTwoVertexFromCenterHalf(capsule->o, capsule->axis, capsule->half, axis_edge[0], axis_edge[1]);
	d[0] = mathPointProjectionPlane(axis_edge[0], plane_v, plane_n);
	d[1] = mathPointProjectionPlane(axis_edge[1], plane_v, plane_n);
	if (d[0] >= CCTNum(0.0) && d[1] <= CCTNum(0.0)) {
		set_intersect(result);
		return result;
	}
	if (d[0] <= CCTNum(0.0) && d[1] >= CCTNum(0.0)) {
		set_intersect(result);
		return result;
	}
	abs_d[0] = CCTNum_abs(d[0]);
	abs_d[1] = CCTNum_abs(d[1]);
	idx = (abs_d[0] < abs_d[1] ? 0 : 1);
	if (abs_d[idx] <= capsule->radius) {
		set_intersect(result);
		return result;
	}
	cos_theta = mathVec3Dot(dir, plane_n);
	if (CCTNum(0.0) == cos_theta) {
		return NULL;
	}
	mathVec3Copy(result->hit_plane_v, axis_edge[idx]);
	if (d[idx] > CCTNum(0.0)) {
		mathVec3AddScalar(result->hit_plane_v, plane_n, capsule->radius);
		d[idx] -= capsule->radius;
	}
	else {
		mathVec3SubScalar(result->hit_plane_v, plane_n, capsule->radius);
		d[idx] += capsule->radius;
	}
	d[idx] /= cos_theta;
	if (d[idx] < CCTNum(0.0)) {
		return NULL;
	}
	result->overlap = 0;
	if (abs_d[0] == abs_d[1]) {
		mathVec3Copy(result->hit_plane_v, plane_v);
		result->hit_unique_point = 0;
		result->peer[0].hit_part = 0;
		result->peer[0].id = 0;
	}
	else {
		mathVec3AddScalar(result->hit_plane_v, dir, d[idx]);
		result->hit_unique_point = 1;
		result->peer[0].hit_part = CCT_SWEEP_HIT_SPHERE;
		result->peer[0].id = idx;
	}
	if (cos_theta > CCTNum(0.0)) {
		mathVec3Negate(result->hit_plane_n, plane_n);
	}
	else {
		mathVec3Copy(result->hit_plane_n, plane_n);
	}
	result->distance = d[idx];
	result->peer[1].hit_part = CCT_SWEEP_HIT_FACE;
	result->peer[1].id = 0;
	return result;
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
	if (cos_theta > CCTNum(0.0)) {
		mathVec3Negate(result->hit_plane_n, plane_n);
	}
	else {
		mathVec3Copy(result->hit_plane_n, plane_n);
	}
	mathVec3AddScalar(result->hit_plane_v, dir, dn);
	result->hit_unique_point = 1;
	result->overlap = 0;
	result->distance = dn;
	result->peer[0].hit_part = CCT_SWEEP_HIT_SPHERE;
	result->peer[0].id = 0;
	result->peer[1].hit_part = CCT_SWEEP_HIT_FACE;
	result->peer[1].id = 0;
	return result;
}

static CCTSweepResult_t* Mesh_Sweep_Sphere_InternalProc(const GeometryMesh_t* mesh, const CCTNum_t dir[3], const CCTNum_t o[3], CCTNum_t radius, CCTSweepResult_t* result) {
	unsigned int i;
	CCTSweepResult_t* p_result;
	CCTNum_t neg_dir[3];

	p_result = MeshSegment_Sweep_Sphere(mesh, dir, o, radius, 0, result);
	mathVec3Negate(neg_dir, dir);
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		GeometryBorderId_t bi;
		CCTSweepResult_t result_temp;
		const GeometryPolygon_t* polygon = mesh->polygons + i;
		if (!Sphere_Sweep_Plane(o, radius, neg_dir, polygon->v[polygon->v_indices[0]], polygon->normal, &result_temp)) {
			continue;
		}
		if (result_temp.overlap) {
			continue;
		}
		if (!p_result) {
			if (!Polygon_Contain_Point_SamePlane(polygon, result_temp.hit_plane_v, &bi)) {
				continue;
			}
			p_result = result;
			*result = result_temp;
		}
		else if (result_temp.distance > result->distance + CCT_EPSILON) {
			continue;
		}
		else if (result_temp.distance < result->distance - CCT_EPSILON) {
			if (!Polygon_Contain_Point_SamePlane(polygon, result_temp.hit_plane_v, &bi)) {
				continue;
			}
			*result = result_temp;
		}
		else {
			if (!Polygon_Contain_Point_SamePlane(polygon, result_temp.hit_plane_v, &bi)) {
				continue;
			}
			if (result_temp.distance < result->distance) {
				result->distance = result_temp.distance;
			}
			if (bi.v_id != -1 || bi.edge_id != -1) {
				continue;
			}
			result_temp.peer[0].hit_part = CCT_SWEEP_HIT_FACE;
			result_temp.peer[0].id = i;
			result_temp.peer[1].hit_part = CCT_SWEEP_HIT_SPHERE;
			result_temp.peer[1].id = 0;
			merge_mesh_hit_info(&result->peer[0], &result_temp.peer[0], mesh);
			continue;
		}
		if (bi.v_id != -1) {
			result->peer[0].hit_part = CCT_SWEEP_HIT_POINT;
			if (polygon->mesh_v_ids && mesh->polygons_cnt > 1) {
				result->peer[0].id = polygon->mesh_v_ids[bi.v_id];
			}
			else {
				result->peer[0].id = bi.v_id;
			}
		}
		else if (bi.edge_id != -1) {
			result->peer[0].hit_part = CCT_SWEEP_HIT_EDGE;
			if (polygon->mesh_edge_ids && mesh->polygons_cnt > 1) {
				result->peer[0].id = polygon->mesh_edge_ids[bi.edge_id];
			}
			else {
				result->peer[0].id = bi.edge_id;
			}
		}
		else {
			result->peer[0].hit_part = CCT_SWEEP_HIT_FACE;
			result->peer[0].id = i;
		}
		result->peer[1].hit_part = CCT_SWEEP_HIT_SPHERE;
		result->peer[1].id = 0;
		mathVec3AddScalar(result->hit_plane_v, dir, result->distance);
	}
	return p_result;
}

static int Capsule_MoveTo_Polygon(const GeometryCapsuleExtra_t* extra, CCTNum_t radius, const CCTNum_t dir[3], const GeometryPolygon_t* polygon, CCTSweepResult_t* result) {
	GeometryBorderId_t bi;
	if (!result->hit_unique_point) {
		const CCTNum_t* polygon_v0 = polygon->v[polygon->v_indices[0]];
		int i;
		for (i = 0; i < 2; ++i) {
			CCTNum_t p[3];
			mathSphereProjectionPlane(extra->axis_edge[i], radius, polygon_v0, polygon->normal, p);
			mathVec3AddScalar(p, dir, result->distance);
			if (!Polygon_Contain_Point_SamePlane(polygon, p, &bi)) {
				continue;
			}
			if (bi.v_id == -1 && bi.edge_id == -1) {
				return 1;
			}
		}
	}
	else if (Polygon_Contain_Point_SamePlane(polygon, result->hit_plane_v, &bi)) {
		if (bi.v_id != -1) {
			result->peer[1].hit_part = CCT_SWEEP_HIT_POINT;
			result->peer[1].id = bi.v_id;
		}
		else if (bi.edge_id != -1) {
			result->peer[1].hit_part = CCT_SWEEP_HIT_EDGE;
			result->peer[1].id = bi.edge_id;
		}
		return 1;
	}
	/* need test Segment sweep Capsule */
	return 0;
}

static CCTSweepResult_t* Capsule_Sweep_Polygon(const GeometryCapsule_t* capsule, const CCTNum_t dir[3], const GeometryPolygon_t* polygon, CCTSweepResult_t* result) {
	int plane_side;
	CCTNum_t neg_dir[3];
	GeometryMesh_t m;
	GeometryCapsuleExtra_t extra;
	mathTwoVertexFromCenterHalf(capsule->o, capsule->axis, capsule->half, extra.axis_edge[0], extra.axis_edge[1]);
	extra.axis_len = capsule->half + capsule->half;
	extra.radius_sq = CCTNum_sq(capsule->radius);
	if (Capsule_Intersect_Polygon(capsule, &extra, polygon, &plane_side)) {
		set_intersect(result);
		return result;
	}
	if (plane_side) {
		if (!Capsule_Sweep_Plane(capsule, dir, polygon->v[polygon->v_indices[0]], polygon->normal, result)) {
			return NULL;
		}
		if (Capsule_MoveTo_Polygon(&extra, capsule->radius, dir, polygon, result)) {
			return result;
		}
	}
	sweep_mesh_convert_from_polygon(&m, polygon);
	mathVec3Negate(neg_dir, dir);
	if (!MeshSegment_Sweep_Capsule(&m, neg_dir, capsule, 0, result)) {
		return NULL;
	}
	reverse_result(result, dir);
	return result;
}

static CCTSweepResult_t* Mesh_Sweep_Capsule_InternalProc(const GeometryMesh_t* mesh, const CCTNum_t dir[3], const GeometryCapsule_t* capsule, CCTSweepResult_t* result) {
	unsigned int i, plane_flag = 0;
	CCTNum_t neg_dir[3];
	GeometryCapsuleExtra_t extra;
	CCTSweepResult_t* p_result;

	p_result = MeshSegment_Sweep_Capsule(mesh, dir, capsule, 0, result);
	mathVec3Negate(neg_dir, dir);
	mathTwoVertexFromCenterHalf(capsule->o, capsule->axis, capsule->half, extra.axis_edge[0], extra.axis_edge[1]);
	extra.axis_len = capsule->half + capsule->half;
	extra.radius_sq = CCTNum_sq(capsule->radius);
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		CCTSweepResult_t result_temp;
		const GeometryPolygon_t* polygon = mesh->polygons + i;
		if (!Capsule_Sweep_Plane(capsule, neg_dir, polygon->v[polygon->v_indices[0]], polygon->normal, &result_temp)) {
			continue;
		}
		if (result_temp.overlap) {
			continue;
		}
		if (!p_result) {
			if (!Capsule_MoveTo_Polygon(&extra, capsule->radius, neg_dir, polygon, &result_temp)) {
				continue;
			}
			*result = result_temp;
			p_result = result;
		}
		else if (result_temp.distance > result->distance + CCT_EPSILON) {
			continue;
		}
		else if (result_temp.distance < result->distance - CCT_EPSILON) {
			if (!Capsule_MoveTo_Polygon(&extra, capsule->radius, neg_dir, polygon, &result_temp)) {
				continue;
			}
			*result = result_temp;
		}
		else {
			CCTNum_t d;
			if (!Capsule_MoveTo_Polygon(&extra, capsule->radius, neg_dir, polygon, &result_temp)) {
				continue;
			}
			d = result->distance;
			*result = result_temp;
			if (d < result_temp.distance) {
				result->distance = d;
			}
		}
		if (result->peer[1].hit_part == CCT_SWEEP_HIT_FACE) {
			result->peer[1].id = i;
		}
		plane_flag = 1;
	}
	if (!p_result) {
		return NULL;
	}
	if (plane_flag) {
		reverse_result(result, dir);
	}
	if (result->peer[1].hit_part != CCT_SWEEP_HIT_SPHERE) {
		result->hit_unique_point = 0;
	}
	return result;
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
		GeometryBorderId_t bi;
		if (!Sphere_Sweep_Plane(o, radius, dir, polygon->v[polygon->v_indices[0]], polygon->normal, result)) {
			return NULL;
		}
		if (Polygon_Contain_Point_SamePlane(polygon, result->hit_plane_v, &bi)) {
			if (bi.v_id != -1) {
				result->peer[1].hit_part = CCT_SWEEP_HIT_POINT;
				result->peer[1].id = bi.v_id;
			}
			else if (bi.edge_id != -1) {
				result->peer[1].hit_part = CCT_SWEEP_HIT_EDGE;
				result->peer[1].id = bi.edge_id;
			}
			return result;
		}
	}
	sweep_mesh_convert_from_polygon(&m, polygon);
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
	mathVec3SubScalar(result->hit_plane_v, result->hit_plane_n, r1);
	result->peer[0].hit_part = CCT_SWEEP_HIT_SPHERE;
	return result;
}

static CCTSweepResult_t* Sphere_Sweep_Capsule(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t dir[3], const GeometryCapsule_t* capsule, int check_intersect, CCTSweepResult_t* result) {
	GeometryCapsule_t new_capsule;
	if (check_intersect) {
		CCTNum_t closest_p[3], lensq, radius_sum = radius + capsule->radius;
		mathSegmentClosestPoint_v2(capsule->o, capsule->axis, capsule->half, o, closest_p);
		lensq = mathVec3DistanceSq(closest_p, o);
		if (lensq <= CCTNum_sq(radius_sum)) {
			set_intersect(result);
			return result;
		}
	}
	new_capsule = *capsule;
	new_capsule.radius += radius;
	if (!Ray_Sweep_Capsule(o, dir, &new_capsule, 0, result)) {
		return NULL;
	}
	mathVec3SubScalar(result->hit_plane_v, result->hit_plane_n, radius);
	result->peer[0].hit_part = CCT_SWEEP_HIT_SPHERE;
	return result;
}

static CCTSweepResult_t* ConvexMesh_Sweep_Capsule(const GeometryMesh_t* mesh, const CCTNum_t dir[3], const GeometryCapsule_t* capsule, CCTSweepResult_t* result) {
	if (Capsule_Intersect_ConvexMesh(capsule, mesh)) {
		set_intersect(result);
		return result;
	}
	return Mesh_Sweep_Capsule_InternalProc(mesh, dir, capsule, result);
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
	if (Sphere_Intersect_OBB(o, radius, obb)) {
		set_intersect(result);
		return result;
	}
	mathBoxMesh(&mesh, obb->o, obb->half, (const CCTNum_t(*)[3])obb->axis);
	return Mesh_Sweep_Sphere_InternalProc(&mesh.mesh, dir, o, radius, result);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
extern "C" {
#endif

CCTSweepResult_t* mathGeometrySweep(const void* geo_data1, int geo_type1, const CCTNum_t dir[3], const void* geo_data2, int geo_type2, CCTSweepResult_t* result) {
	CCTNum_t neg_dir[3];
	int flag_neg_dir;
	GeometryBoxMesh_t box_data;
	const GeometryMesh_t* mesh2;
	if (geo_data1 == geo_data2 || mathVec3IsZero(dir)) {
		return NULL;
	}
	flag_neg_dir = 0;
	if (GEOMETRY_BODY_POINT == geo_type1) {
		const CCTNum_t* point1 = (const CCTNum_t*)geo_data1;
		switch (geo_type2) {
			case GEOMETRY_BODY_AABB:
			{
				const GeometryAABB_t* aabb2 = (const GeometryAABB_t*)geo_data2;
				mathBoxMesh(&box_data, aabb2->o, aabb2->half, AABB_Axis);
				result = Ray_Sweep_ConvexMesh(point1, dir, &box_data.mesh, result);
				break;
			}
			case GEOMETRY_BODY_OBB:
			{
				const GeometryOBB_t* obb2 = (const GeometryOBB_t*)geo_data2;
				mathBoxMesh(&box_data, obb2->o, obb2->half, (const CCTNum_t(*)[3])obb2->axis);
				result = Ray_Sweep_ConvexMesh(point1, dir, &box_data.mesh, result);
				break;
			}
			case GEOMETRY_BODY_SPHERE:
			{
				const GeometrySphere_t* sphere2 = (const GeometrySphere_t*)geo_data2;
				result = Ray_Sweep_Sphere(point1, dir, sphere2->o, sphere2->radius, result);
				break;
			}
			case GEOMETRY_BODY_PLANE:
			{
				const GeometryPlane_t* plane2 = (const GeometryPlane_t*)geo_data2;
				result = Ray_Sweep_Plane(point1, dir, plane2->v, plane2->normal, result);
				break;
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				result = Ray_Sweep_Segment(point1, dir, (const CCTNum_t(*)[3])segment2->v, result);
				break;
			}
			case GEOMETRY_BODY_POLYGON:
			{
				result = Ray_Sweep_Polygon(point1, dir, (const GeometryPolygon_t*)geo_data2, result);
				break;
			}
			case GEOMETRY_BODY_MESH:
			{
				mesh2 = (const GeometryMesh_t*)geo_data2;
				if (!mesh2->is_convex || !mesh2->is_closed) {
					return NULL;
				}
				result = Ray_Sweep_ConvexMesh(point1, dir, mesh2, result);
				break;
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				result = Ray_Sweep_Capsule(point1, dir, (const GeometryCapsule_t*)geo_data2, 1, result);
				break;
			}
			default:
			{
				return NULL;
			}
		}
	}
	else if (GEOMETRY_BODY_SEGMENT == geo_type1) {
		const GeometrySegment_t* segment1 = (const GeometrySegment_t*)geo_data1;
		const CCTNum_t(*segment1_v)[3] = (const CCTNum_t(*)[3])segment1->v;
		switch (geo_type2) {
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				result = Segment_Sweep_Segment(segment1_v, dir, (const CCTNum_t(*)[3])segment2->v, result);
				break;
			}
			case GEOMETRY_BODY_PLANE:
			{
				const GeometryPlane_t* plane2 = (const GeometryPlane_t*)geo_data2;
				GeometryMesh_t mesh1;
				sweep_mesh_convert_from_segment(&mesh1, segment1_v);
				result = Mesh_Sweep_Plane(&mesh1, dir, plane2->v, plane2->normal, result);
				break;
			}
			case GEOMETRY_BODY_OBB:
			{
				const GeometryOBB_t* obb2 = (const GeometryOBB_t*)geo_data2;
				mathBoxMesh(&box_data, obb2->o, obb2->half, (const CCTNum_t(*)[3])obb2->axis);
				result = Segment_Sweep_ConvexMesh(segment1_v, dir, &box_data.mesh, result);
				break;
			}
			case GEOMETRY_BODY_AABB:
			{
				const GeometryAABB_t* aabb2 = (const GeometryAABB_t*)geo_data2;
				mathBoxMesh(&box_data, aabb2->o, aabb2->half, AABB_Axis);
				result = Segment_Sweep_ConvexMesh(segment1_v, dir, &box_data.mesh, result);
				break;
			}
			case GEOMETRY_BODY_SPHERE:
			{
				const GeometrySphere_t* sphere2 = (const GeometrySphere_t*)geo_data2;
				result = Segment_Sweep_Sphere(segment1_v, dir, sphere2->o, sphere2->radius, 1, result);
				break;
			}
			case GEOMETRY_BODY_POLYGON:
			{
				result = Segment_Sweep_Polygon(segment1_v, dir, (const GeometryPolygon_t*)geo_data2, result);
				break;
			}
			case GEOMETRY_BODY_MESH:
			{
				mesh2 = (const GeometryMesh_t*)geo_data2;
				if (!mesh2->is_convex || !mesh2->is_closed) {
					return NULL;
				}
				result = Segment_Sweep_ConvexMesh(segment1_v, dir, mesh2, result);
				break;
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				result = Segment_Sweep_Capsule(segment1_v, NULL, 0, dir, (const GeometryCapsule_t*)geo_data2, 1, result);
				break;
			}
			default:
			{
				return NULL;
			}
		}
	}
	else if (GEOMETRY_BODY_AABB == geo_type1) {
		const GeometryAABB_t* aabb1 = (const GeometryAABB_t*)geo_data1;
		switch (geo_type2) {
			case GEOMETRY_BODY_AABB:
			{
				const GeometryAABB_t* aabb2 = (const GeometryAABB_t*)geo_data2;
				GeometryOBB_t obb1, obb2;
				mathOBBFromAABB(&obb1, aabb1->o, aabb1->half);
				mathOBBFromAABB(&obb2, aabb2->o, aabb2->half);
				result = OBB_Sweep_OBB(&obb1, dir, &obb2, result);
				break;
			}
			case GEOMETRY_BODY_OBB:
			{
				const GeometryOBB_t* obb2 = (const GeometryOBB_t*)geo_data2;
				GeometryOBB_t obb1;
				mathOBBFromAABB(&obb1, aabb1->o, aabb1->half);
				result = OBB_Sweep_OBB(&obb1, dir, obb2, result);
				break;
			}
			case GEOMETRY_BODY_SPHERE:
			{
				const GeometrySphere_t* sphere2 = (const GeometrySphere_t*)geo_data2;
				GeometryOBB_t obb1;
				mathOBBFromAABB(&obb1, aabb1->o, aabb1->half);
				result = OBB_Sweep_Sphere(&obb1, dir, sphere2->o, sphere2->radius, result);
				break;
			}
			case GEOMETRY_BODY_PLANE:
			{
				const GeometryPlane_t* plane2 = (const GeometryPlane_t*)geo_data2;
				mathBoxMesh(&box_data, aabb1->o, aabb1->half, AABB_Axis);
				result = Mesh_Sweep_Plane(&box_data.mesh, dir, plane2->v, plane2->normal, result);
				break;
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				mathBoxMesh(&box_data, aabb1->o, aabb1->half, AABB_Axis);
				result = Segment_Sweep_ConvexMesh((const CCTNum_t(*)[3])segment2->v, neg_dir, &box_data.mesh, result);
				break;
			}
			case GEOMETRY_BODY_POLYGON:
			{
				mathBoxMesh(&box_data, aabb1->o, aabb1->half, AABB_Axis);
				result = ConvexMesh_Sweep_Polygon(&box_data.mesh, dir, (const GeometryPolygon_t*)geo_data2, result);
				break;
			}
			case GEOMETRY_BODY_MESH:
			{
				mesh2 = (const GeometryMesh_t*)geo_data2;
				if (!mesh2->is_convex || !mesh2->is_closed) {
					return NULL;
				}
				mathBoxMesh(&box_data, aabb1->o, aabb1->half, AABB_Axis);
				result = ConvexMesh_Sweep_ConvexMesh(&box_data.mesh, dir, mesh2, result);
				break;
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				mathBoxMesh(&box_data, aabb1->o, aabb1->half, AABB_Axis);
				result = ConvexMesh_Sweep_Capsule(&box_data.mesh, dir, (const GeometryCapsule_t*)geo_data2, result);
				break;
			}
			default:
			{
				return NULL;
			}
		}
	}
	else if (GEOMETRY_BODY_OBB == geo_type1) {
		const GeometryOBB_t* obb1 = (const GeometryOBB_t*)geo_data1;
		switch (geo_type2) {
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				mathBoxMesh(&box_data, obb1->o, obb1->half, (const CCTNum_t(*)[3])obb1->axis);
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = Segment_Sweep_ConvexMesh((const CCTNum_t(*)[3])segment2->v, neg_dir, &box_data.mesh, result);
				break;
			}
			case GEOMETRY_BODY_PLANE:
			{
				const GeometryPlane_t* plane2 = (const GeometryPlane_t*)geo_data2;
				mathBoxMesh(&box_data, obb1->o, obb1->half, (const CCTNum_t(*)[3])obb1->axis);
				result = Mesh_Sweep_Plane(&box_data.mesh, dir, plane2->v, plane2->normal, result);
				break;
			}
			case GEOMETRY_BODY_SPHERE:
			{
				const GeometrySphere_t* sphere2 = (const GeometrySphere_t*)geo_data2;
				result = OBB_Sweep_Sphere(obb1, dir, sphere2->o, sphere2->radius, result);
				break;
			}
			case GEOMETRY_BODY_OBB:
			{
				result = OBB_Sweep_OBB(obb1, dir, (const GeometryOBB_t*)geo_data2, result);
				break;
			}
			case GEOMETRY_BODY_AABB:
			{
				const GeometryAABB_t* aabb2 = (const GeometryAABB_t*)geo_data2;
				GeometryOBB_t obb2;
				mathOBBFromAABB(&obb2, aabb2->o, aabb2->half);
				result = OBB_Sweep_OBB(obb1, dir, &obb2, result);
				break;
			}
			case GEOMETRY_BODY_POLYGON:
			{
				mathBoxMesh(&box_data, obb1->o, obb1->half, (const CCTNum_t(*)[3])obb1->axis);
				result = ConvexMesh_Sweep_Polygon(&box_data.mesh, dir, (const GeometryPolygon_t*)geo_data2, result);
				break;
			}
			case GEOMETRY_BODY_MESH:
			{
				mesh2 = (const GeometryMesh_t*)geo_data2;
				if (!mesh2->is_convex || !mesh2->is_closed) {
					return NULL;
				}
				mathBoxMesh(&box_data, obb1->o, obb1->half, (const CCTNum_t(*)[3])obb1->axis);
				result = ConvexMesh_Sweep_ConvexMesh(&box_data.mesh, dir, mesh2, result);
				break;
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				mathBoxMesh(&box_data, obb1->o, obb1->half, (const CCTNum_t(*)[3])obb1->axis);
				result = ConvexMesh_Sweep_Capsule(&box_data.mesh, dir, (const GeometryCapsule_t*)geo_data2, result);
				break;
			}
			default:
			{
				return NULL;
			}
		}
	}
	else if (GEOMETRY_BODY_SPHERE == geo_type1) {
		const GeometrySphere_t* sphere1 = (const GeometrySphere_t*)geo_data1;
		switch (geo_type2) {
			case GEOMETRY_BODY_OBB:
			{
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = OBB_Sweep_Sphere((const GeometryOBB_t*)geo_data2, neg_dir, sphere1->o, sphere1->radius, result);
				break;
			}
			case GEOMETRY_BODY_AABB:
			{
				const GeometryAABB_t* aabb2 = (const GeometryAABB_t*)geo_data2;
				GeometryOBB_t obb2;
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				mathOBBFromAABB(&obb2, aabb2->o, aabb2->half);
				result = OBB_Sweep_Sphere(&obb2, neg_dir, sphere1->o, sphere1->radius, result);
				break;
			}
			case GEOMETRY_BODY_SPHERE:
			{
				const GeometrySphere_t* sphere2 = (const GeometrySphere_t*)geo_data2;
				result = Sphere_Sweep_Sphere(sphere1->o, sphere1->radius, dir, sphere2->o, sphere2->radius, result);
				break;
			}
			case GEOMETRY_BODY_PLANE:
			{
				const GeometryPlane_t* plane2 = (const GeometryPlane_t*)geo_data2;
				result = Sphere_Sweep_Plane(sphere1->o, sphere1->radius, dir, plane2->v, plane2->normal, result);
				break;
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = Segment_Sweep_Sphere((const CCTNum_t(*)[3])segment2->v, neg_dir, sphere1->o, sphere1->radius, 1, result);
				break;
			}
			case GEOMETRY_BODY_POLYGON:
			{
				result = Sphere_Sweep_Polygon(sphere1->o, sphere1->radius, dir, (const GeometryPolygon_t*)geo_data2, result);
				break;
			}
			case GEOMETRY_BODY_MESH:
			{
				mesh2 = (const GeometryMesh_t*)geo_data2;
				if (!mesh2->is_convex || !mesh2->is_closed) {
					return NULL;
				}
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = ConvexMesh_Sweep_Sphere(mesh2, neg_dir, sphere1->o, sphere1->radius, result);
				break;
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				result = Sphere_Sweep_Capsule(sphere1->o, sphere1->radius, dir, (const GeometryCapsule_t*)geo_data2, 1, result);
				break;
			}
			default:
			{
				return NULL;
			}
		}
	}
	else if (GEOMETRY_BODY_POLYGON == geo_type1) {
		const GeometryPolygon_t* polygon1 = (const GeometryPolygon_t*)geo_data1;
		switch (geo_type2) {
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = Segment_Sweep_Polygon((const CCTNum_t(*)[3])segment2->v, neg_dir, polygon1, result);
				break;
			}
			case GEOMETRY_BODY_PLANE:
			{
				const GeometryPlane_t* plane2 = (const GeometryPlane_t*)geo_data2;
				GeometryMesh_t mesh1;
				sweep_mesh_convert_from_polygon(&mesh1, polygon1);
				result = Mesh_Sweep_Plane(&mesh1, dir, plane2->v, plane2->normal, result);
				break;
			}
			case GEOMETRY_BODY_SPHERE:
			{
				const GeometrySphere_t* sphere2 = (const GeometrySphere_t*)geo_data2;
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = Sphere_Sweep_Polygon(sphere2->o, sphere2->radius, neg_dir, polygon1, result);
				break;
			}
			case GEOMETRY_BODY_OBB:
			{
				const GeometryOBB_t* obb2 = (const GeometryOBB_t*)geo_data2;
				mathBoxMesh(&box_data, obb2->o, obb2->half, (const CCTNum_t(*)[3])obb2->axis);
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = ConvexMesh_Sweep_Polygon(&box_data.mesh, neg_dir, polygon1, result);
				break;
			}
			case GEOMETRY_BODY_AABB:
			{
				const GeometryAABB_t* aabb2 = (const GeometryAABB_t*)geo_data2;
				mathBoxMesh(&box_data, aabb2->o, aabb2->half, AABB_Axis);
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = ConvexMesh_Sweep_Polygon(&box_data.mesh, neg_dir, polygon1, result);
				break;
			}
			case GEOMETRY_BODY_POLYGON:
			{
				result = Polygon_Sweep_Polygon(polygon1, dir, (const GeometryPolygon_t*)geo_data2, result);
				break;
			}
			case GEOMETRY_BODY_MESH:
			{
				mesh2 = (const GeometryMesh_t*)geo_data2;
				if (!mesh2->is_convex || !mesh2->is_closed) {
					return NULL;
				}
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = ConvexMesh_Sweep_Polygon(mesh2, neg_dir, polygon1, result);
				break;
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = Capsule_Sweep_Polygon((const GeometryCapsule_t*)geo_data2, neg_dir, polygon1, result);
				break;
			}
			default:
			{
				return NULL;
			}
		}
	}
	else if (GEOMETRY_BODY_MESH == geo_type1) {
		const GeometryMesh_t* mesh1 = (const GeometryMesh_t*)geo_data1;
		if (mesh1->is_convex && mesh1->is_closed) {
			switch (geo_type2) {
				case GEOMETRY_BODY_SEGMENT:
				{
					const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
					mathVec3Negate(neg_dir, dir);
					flag_neg_dir = 1;
					result = Segment_Sweep_ConvexMesh((const CCTNum_t(*)[3])segment2->v, neg_dir, mesh1, result);
					break;
				}
				case GEOMETRY_BODY_PLANE:
				{
					const GeometryPlane_t* plane2 = (const GeometryPlane_t*)geo_data2;
					result = Mesh_Sweep_Plane(mesh1, dir, plane2->v, plane2->normal, result);
					break;
				}
				case GEOMETRY_BODY_SPHERE:
				{
					const GeometrySphere_t* sphere2 = (const GeometrySphere_t*)geo_data2;
					result = ConvexMesh_Sweep_Sphere(mesh1, dir, sphere2->o, sphere2->radius, result);
					break;
				}
				case GEOMETRY_BODY_OBB:
				{
					const GeometryOBB_t* obb2 = (const GeometryOBB_t*)geo_data2;
					mathBoxMesh(&box_data, obb2->o, obb2->half, (const CCTNum_t(*)[3])obb2->axis);
					result = ConvexMesh_Sweep_ConvexMesh(mesh1, dir, &box_data.mesh, result);
					break;
				}
				case GEOMETRY_BODY_AABB:
				{
					const GeometryAABB_t* aabb2 = (const GeometryAABB_t*)geo_data2;
					mathBoxMesh(&box_data, aabb2->o, aabb2->half, AABB_Axis);
					result = ConvexMesh_Sweep_ConvexMesh(mesh1, dir, &box_data.mesh, result);
					break;
				}
				case GEOMETRY_BODY_POLYGON:
				{
					result = ConvexMesh_Sweep_Polygon(mesh1, dir, (const GeometryPolygon_t*)geo_data2, result);
					break;
				}
				case GEOMETRY_BODY_MESH:
				{
					mesh2 = (const GeometryMesh_t*)geo_data2;
					if (!mesh2->is_convex || !mesh2->is_closed) {
						return NULL;
					}
					result = ConvexMesh_Sweep_ConvexMesh(mesh1, dir, mesh2, result);
					break;
				}
				case GEOMETRY_BODY_CAPSULE:
				{
					result = ConvexMesh_Sweep_Capsule(mesh1, dir, (const GeometryCapsule_t*)geo_data2, result);
					break;
				}
				default:
				{
					return NULL;
				}
			}
		}
		else if (GEOMETRY_BODY_PLANE == geo_type2) {
			const GeometryPlane_t* plane2 = (const GeometryPlane_t*)geo_data2;
			result = Mesh_Sweep_Plane(mesh1, dir, plane2->v, plane2->normal, result);
		}
		else {
			return NULL;
		}
	}
	else if (GEOMETRY_BODY_CAPSULE == geo_type1) {
		const GeometryCapsule_t* capsule1 = (const GeometryCapsule_t*)geo_data1;
		switch (geo_type2) {
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = Segment_Sweep_Capsule((const CCTNum_t(*)[3])segment2->v, NULL, 0, neg_dir, capsule1, 1, result);
				break;
			}
			case GEOMETRY_BODY_PLANE:
			{
				const GeometryPlane_t* plane2 = (const GeometryPlane_t*)geo_data2;
				result = Capsule_Sweep_Plane(capsule1, dir, plane2->v, plane2->normal, result);
				break;
			}
			case GEOMETRY_BODY_POLYGON:
			{
				result = Capsule_Sweep_Polygon(capsule1, dir, (const GeometryPolygon_t*)geo_data2, result);
				break;
			}
			case GEOMETRY_BODY_MESH:
			{
				mesh2 = (const GeometryMesh_t*)geo_data2;
				if (!mesh2->is_convex || !mesh2->is_closed) {
					return NULL;
				}
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = ConvexMesh_Sweep_Capsule(mesh2, neg_dir, capsule1, result);
				break;
			}
			case GEOMETRY_BODY_OBB:
			{
				const GeometryOBB_t* obb2 = (const GeometryOBB_t*)geo_data2;
				mathBoxMesh(&box_data, obb2->o, obb2->half, (const CCTNum_t(*)[3])obb2->axis);
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = ConvexMesh_Sweep_Capsule(&box_data.mesh, neg_dir, capsule1, result);
				break;
			}
			case GEOMETRY_BODY_AABB:
			{
				const GeometryAABB_t* aabb2 = (const GeometryAABB_t*)geo_data2;
				mathBoxMesh(&box_data, aabb2->o, aabb2->half, AABB_Axis);
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = ConvexMesh_Sweep_Capsule(&box_data.mesh, neg_dir, capsule1, result);
				break;
			}
			case GEOMETRY_BODY_SPHERE:
			{
				const GeometrySphere_t* sphere2 = (const GeometrySphere_t*)geo_data2;
				mathVec3Negate(neg_dir, dir);
				flag_neg_dir = 1;
				result = Sphere_Sweep_Capsule(sphere2->o, sphere2->radius, neg_dir, capsule1, 1, result);
				break;
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				result = Capsule_Sweep_Capsule(capsule1, dir, (const GeometryCapsule_t*)geo_data2, 1, result);
				break;
			}
			default:
			{
				return NULL;
			}
		}
	}
	else {
		return NULL;
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
