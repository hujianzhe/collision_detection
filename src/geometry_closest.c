//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/line_segment.h"
#include "../inc/plane.h"
#include "../inc/aabb.h"
#include "../inc/obb.h"
#include "../inc/geometry_closest.h"

extern int Polygon_Contain_Point_SamePlane(const GeometryPolygon_t* polygon, const CCTNum_t p[3], GeometryBorderId_t* bi);

CCTNum_t Segment_ClosestVertexIndices_Segment(const CCTNum_t ls1[2][3], const CCTNum_t ls2[2][3], unsigned int* ls1_indices, unsigned int* ls2_indices) {
	CCTNum_t lensq, min_lensq, v[3];

	mathVec3Sub(v, ls1[0], ls2[0]);
	min_lensq = mathVec3LenSq(v);
	*ls1_indices = 0;
	*ls2_indices = 0;

	mathVec3Sub(v, ls1[0], ls2[1]);
	lensq = mathVec3LenSq(v);
	if (min_lensq > lensq) {
		min_lensq = lensq;
		*ls1_indices = 0;
		*ls2_indices = 1;
	}

	mathVec3Sub(v, ls1[1], ls2[0]);
	lensq = mathVec3LenSq(v);
	if (min_lensq > lensq) {
		min_lensq = lensq;
		*ls1_indices = 1;
		*ls2_indices = 0;
	}

	mathVec3Sub(v, ls1[1], ls2[1]);
	lensq = mathVec3LenSq(v);
	if (min_lensq > lensq) {
		min_lensq = lensq;
		*ls1_indices = 1;
		*ls2_indices = 1;
	}

	return min_lensq;
}

CCTNum_t Segment_ClosestLenSq_Segment(const CCTNum_t ls1[2][3], const CCTNum_t ls1_dir[3], CCTNum_t ls1_len, const CCTNum_t ls2[2][3], const CCTNum_t ls2_dir[3], CCTNum_t ls2_len) {
	CCTNum_t v[3], N[3], d;
	CCTNum_t ls1_dir_temp[3], ls2_dir_temp[3];
	if (!ls1_dir) {
		ls1_dir = ls1_dir_temp;
		mathVec3Sub(ls1_dir_temp, ls1[1], ls1[0]);
	}
	if (!ls2_dir) {
		ls2_dir = ls2_dir_temp;
		mathVec3Sub(ls2_dir_temp, ls2[1], ls2[0]);
	}
	mathVec3Cross(N, ls1_dir, ls2_dir);
	mathVec3Sub(v, ls2[0], ls1[0]);
	if (mathVec3IsZero(N)) {
		unsigned int v_idx[2];
		mathVec3Cross(N, v, ls1_dir);
		if (mathVec3IsZero(N)) {
			/* collinear */
			CCTNum_t l[3], r[3];
			mathVec3Sub(l, ls1[0], ls2[0]);
			mathVec3Sub(r, ls1[1], ls2[0]);
			d = mathVec3Dot(l, r);
			if (d <= CCTNum(0.0)) {
				return CCTNum(0.0);
			}
			mathVec3Sub(l, ls1[0], ls2[1]);
			mathVec3Sub(r, ls1[1], ls2[1]);
			d = mathVec3Dot(l, r);
			if (d <= CCTNum(0.0)) {
				return CCTNum(0.0);
			}
			mathVec3Sub(l, ls2[0], ls1[0]);
			mathVec3Sub(r, ls2[1], ls1[0]);
			d = mathVec3Dot(l, r);
			if (d <= CCTNum(0.0)) {
				return CCTNum(0.0);
			}
			mathVec3Sub(l, ls2[0], ls1[1]);
			mathVec3Sub(r, ls2[1], ls1[1]);
			d = mathVec3Dot(l, r);
			if (d <= CCTNum(0.0)) {
				return CCTNum(0.0);
			}
		}
		else {
			/* parallel */
			int i;
			if (ls1_dir == ls1_dir_temp) {
				ls1_len = mathVec3Normalized(ls1_dir_temp, ls1_dir_temp);
			}
			if (ls2_dir == ls2_dir_temp) {
				ls2_len = mathVec3Normalized(ls2_dir_temp, ls2_dir_temp);
			}
			for (i = 0; i < 2; ++i) {
				mathVec3Sub(v, ls1[i], ls2[0]);
				d = mathVec3Dot(v, ls2_dir);
				if (d < CCTNum(0.0)) {
					continue;
				}
				if (d > ls2_len) {
					continue;
				}
				return mathVec3LenSq(v) - CCTNum_sq(d);
			}
			for (i = 0; i < 2; ++i) {
				mathVec3Sub(v, ls2[i], ls1[0]);
				d = mathVec3Dot(v, ls1_dir);
				if (d < CCTNum(0.0)) {
					continue;
				}
				if (d > ls1_len) {
					continue;
				}
				return mathVec3LenSq(v) - CCTNum_sq(d);
			}
		}
		return Segment_ClosestVertexIndices_Segment(ls1, ls2, &v_idx[0], &v_idx[1]);
	}
	else {
		int i, lensq_set, set_cnt;
		CCTNum_t lensq;
		if (ls1_dir == ls1_dir_temp) {
			ls1_len = mathVec3Normalized(ls1_dir_temp, ls1_dir_temp);
		}
		if (ls2_dir == ls2_dir_temp) {
			ls2_len = mathVec3Normalized(ls2_dir_temp, ls2_dir_temp);
		}
		d = mathVec3Dot(v, N);
		if (d < CCT_EPSILON_NEGATE || d > CCT_EPSILON) {
			/* opposite */
			CCTNum_t ls1_dir_d, ls2_dir_d;
			mathLineClosestLine_opposite(ls1[0], ls1_dir, ls2[0], ls2_dir, &ls1_dir_d, &ls2_dir_d);
			if ((ls1_dir_d <= ls1_len && ls1_dir_d >= CCTNum(0.0)) &&
				(ls2_dir_d <= ls2_len && ls2_dir_d >= CCTNum(0.0)))
			{
				CCTNum_t closest_p1[3], closest_p2[3];
				mathVec3Copy(closest_p1, ls1[0]);
				mathVec3AddScalar(closest_p1, ls1_dir, ls1_dir_d);
				mathVec3Copy(closest_p2, ls2[0]);
				mathVec3AddScalar(closest_p2, ls2_dir, ls2_dir_d);
				return mathVec3DistanceSq(closest_p1, closest_p2);
			}
		}
		else {
			/* cross */
			CCTNum_t l[3], r[3];
			d = mathLineCrossLine(ls1[0], ls1_dir, ls2[0], ls2_dir);
			mathVec3Copy(v, ls1[0]);
			if (d != CCTNum(0.0)) {
				mathVec3AddScalar(v, ls1_dir, d);
			}
			mathVec3Sub(l, ls1[0], v);
			mathVec3Sub(r, ls1[1], v);
			d = mathVec3Dot(l, r);
			if (d <= CCTNum(0.0)) {
				mathVec3Sub(l, ls2[0], v);
				mathVec3Sub(r, ls2[1], v);
				d = mathVec3Dot(l, r);
				if (d <= CCTNum(0.0)) {
					return CCTNum(0.0);
				}
			}
		}
		lensq_set = 0;
		set_cnt = 0;
		for (i = 0; i < 2; ++i) {
			CCTNum_t l[3], r[3];
			d = mathPointProjectionLine(ls1[i], ls2[0], ls2_dir, v);
			mathVec3Sub(l, ls2[0], v);
			mathVec3Sub(r, ls2[1], v);
			if (mathVec3Dot(l, r) > CCTNum(0.0)) {
				continue;
			}
			set_cnt++;
			d = mathVec3DistanceSq(ls1[i], ls2[0]) - CCTNum_sq(d);
			if (lensq_set && lensq <= d) {
				continue;
			}
			lensq = d;
			lensq_set = 1;
		}
		if (set_cnt >= 2) {
			return lensq;
		}
		set_cnt = 0;
		for (i = 0; i < 2; ++i) {
			CCTNum_t l[3], r[3];
			d = mathPointProjectionLine(ls2[i], ls1[0], ls1_dir, v);
			mathVec3Sub(l, ls1[0], v);
			mathVec3Sub(r, ls1[1], v);
			if (mathVec3Dot(l, r) > CCTNum(0.0)) {
				continue;
			}
			set_cnt++;
			d = mathVec3DistanceSq(ls2[i], ls1[0]) - CCTNum_sq(d);
			if (lensq_set && lensq <= d) {
				continue;
			}
			lensq = d;
			lensq_set = 1;
		}
		if (set_cnt < 2) {
			unsigned int v_idx[2];
			CCTNum_t lensq2 = Segment_ClosestVertexIndices_Segment(ls1, ls2, &v_idx[0], &v_idx[1]);
			if (lensq_set && lensq < lensq2) {
				return lensq;
			}
			return lensq2;
		}
		return lensq;
	}
}

#ifdef	__cplusplus
extern "C" {
#endif

void mathLineClosestLine_opposite(const CCTNum_t lsv1[3], const CCTNum_t lsdir1[3], const CCTNum_t lsv2[3], const CCTNum_t lsdir2[3], CCTNum_t* lsdir_d1, CCTNum_t* lsdir_d2) {
	CCTNum_t temp[3], n[3], v[3], nlensq_inv;
	mathVec3Sub(v, lsv2, lsv1);
	mathVec3Cross(n, lsdir1, lsdir2);
	nlensq_inv = CCTNum(1.0) / mathVec3LenSq(n);
	*lsdir_d1 = mathVec3Dot(mathVec3Cross(temp, v, lsdir2), n) * nlensq_inv;
	*lsdir_d2 = mathVec3Dot(mathVec3Cross(temp, v, lsdir1), n) * nlensq_inv;
}

void mathLineClosestLine_opposite_v2(const CCTNum_t lsv1[3], const CCTNum_t lsdir1[3], const CCTNum_t lsv2[3], const CCTNum_t lsdir2[3], CCTNum_t closest_p1[3], CCTNum_t closest_p2[3]) {
	CCTNum_t d1, d2;
	mathLineClosestLine_opposite(lsv1, lsdir1, lsv2, lsdir2, &d1, &d2);
	mathVec3Copy(closest_p1, lsv1);
	mathVec3AddScalar(closest_p1, lsdir1, d1);
	mathVec3Copy(closest_p2, lsv2);
	mathVec3AddScalar(closest_p2, lsdir2, d2);
}

void mathSegmentClosestPoint(const CCTNum_t ls0[3], const CCTNum_t ls1[3], const CCTNum_t p[3], CCTNum_t closest_p[3]) {
	CCTNum_t ls_v[3], vp[3], lensq, dot;
	mathVec3Sub(vp, p, ls0);
	mathVec3Sub(ls_v, ls1, ls0);
	dot = mathVec3Dot(vp, ls_v);
	if (dot <= CCTNum(0.0)) {
		mathVec3Copy(closest_p, ls0);
		return;
	}
	lensq = mathVec3LenSq(ls_v);
	if (dot >= lensq) {
		mathVec3Copy(closest_p, ls1);
		return;
	}
	mathVec3Copy(closest_p, ls0);
	mathVec3AddScalar(closest_p, ls_v, dot / lensq);
}

void mathSegmentClosestPoint_v2(const CCTNum_t ls_center_p[3], const CCTNum_t lsdir[3], const CCTNum_t ls_half_len, const CCTNum_t p[3], CCTNum_t closest_p[3]) {
	CCTNum_t v[3], d, abs_d;
	mathVec3Sub(v, p, ls_center_p);
	d = mathVec3Dot(v, lsdir);
	abs_d = CCTNum_abs(d);
	mathVec3Copy(closest_p, ls_center_p);
	if (abs_d > ls_half_len) {
		if (d > CCTNum(0.0)) {
			mathVec3AddScalar(closest_p, lsdir, ls_half_len);
		}
		else {
			mathVec3SubScalar(closest_p, lsdir, ls_half_len);
		}
		return;
	}
	mathVec3AddScalar(closest_p, lsdir, d);
}

void mathSegmentClosestPoint_v3(const CCTNum_t ls_v[3], const CCTNum_t lsdir[3], const CCTNum_t ls_len, const CCTNum_t p[3], CCTNum_t closest_p[3]) {
	CCTNum_t v[3], d;
	mathVec3Sub(v, p, ls_v);
	d = mathVec3Dot(v, lsdir);
	mathVec3Copy(closest_p, ls_v);
	if (d <= CCTNum(0.0)) {
		return;
	}
	if (d > ls_len) {
		mathVec3AddScalar(closest_p, lsdir, ls_len);
	}
	else {
		mathVec3AddScalar(closest_p, lsdir, d);
	}
}

void mathSegmentIndicesClosestPoint(const CCTNum_t(*v)[3], const unsigned int* edge_v_indices_flat, unsigned int edge_v_indices_cnt, const CCTNum_t p[3], CCTNum_t closest_p[3]) {
	CCTNum_t min_d;
	unsigned int i, v_idx[2];
	v_idx[0] = edge_v_indices_flat[0];
	v_idx[1] = edge_v_indices_flat[1];
	mathSegmentClosestPoint(v[v_idx[0]], v[v_idx[1]], p, closest_p);
	min_d = mathVec3DistanceSq(p, closest_p);
	if (CCTNum(0.0) == min_d) {
		return;
	}
	for (i = 2; i < edge_v_indices_cnt; ) {
		CCTNum_t cp[3], d;
		v_idx[0] = edge_v_indices_flat[i++];
		v_idx[1] = edge_v_indices_flat[i++];
		mathSegmentClosestPoint(v[v_idx[0]], v[v_idx[1]], p, cp);
		d = mathVec3DistanceSq(p, cp);
		if (min_d <= d) {
			continue;
		}
		min_d = d;
		mathVec3Copy(closest_p, cp);
		if (CCTNum(0.0) == min_d) {
			return;
		}
	}
}

void mathAABBClosestPoint(const CCTNum_t o[3], const CCTNum_t half[3], const CCTNum_t p[3], CCTNum_t closest_p[3]) {
	int i;
	CCTNum_t min_v[3], max_v[3];
	mathAABBMinVertice(o, half, min_v);
	mathAABBMaxVertice(o, half, max_v);
	for (i = 0; i < 3; ++i) {
		if (p[i] < min_v[i]) {
			closest_p[i] = min_v[i];
		}
		else if (p[i] > max_v[i]) {
			closest_p[i] = max_v[i];
		}
		else {
			closest_p[i] = p[i];
		}
	}
}

void mathOBBClosestPoint(const GeometryOBB_t* obb, const CCTNum_t p[3], CCTNum_t closest_p[3]) {
	int i;
	CCTNum_t v[3];

	mathVec3Sub(v, p, obb->o);
	mathVec3Copy(closest_p, obb->o);
	for (i = 0; i < 3; ++i) {
		CCTNum_t d = mathVec3Dot(v, obb->axis[i]);
		if (d > obb->half[i]) {
			d = obb->half[i];
		}
		else if (d < -obb->half[i]) {
			d = -obb->half[i];
		}
		mathVec3AddScalar(closest_p, obb->axis[i], d);
	}
}

void mathSphereClosestPoint(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t p[3], CCTNum_t closest_p[3]) {
	CCTNum_t v[3];
	mathVec3Sub(v, p, o);
	mathVec3Normalized(v, v);
	mathVec3Copy(closest_p, o);
	mathVec3AddScalar(closest_p, v, radius);
}

void mathCapsuleClosestPoint(const GeometryCapsule_t* capsule, const CCTNum_t p[3], CCTNum_t closest_p[3]) {
	CCTNum_t v[3];
	mathSegmentClosestPoint_v2(capsule->o, capsule->axis, capsule->half, p, closest_p);
	mathVec3Sub(v, p, closest_p);
	mathVec3Normalized(v, v);
	mathVec3AddScalar(closest_p, v, capsule->radius);
}

void mathPolygonClosestPoint(const GeometryPolygon_t* polygon, const CCTNum_t p[3], CCTNum_t closest_p[3]) {
	unsigned int polygon_edge_v_indices_cnt;
	CCTNum_t d = mathPointProjectionPlane(p, polygon->v[polygon->v_indices[0]], polygon->normal);
	mathVec3Copy(closest_p, p);
	mathVec3AddScalar(closest_p, polygon->normal, d);
	if (Polygon_Contain_Point_SamePlane(polygon, closest_p, NULL)) {
		return;
	}
	polygon_edge_v_indices_cnt = polygon->edge_cnt + polygon->edge_cnt;
	mathSegmentIndicesClosestPoint((const CCTNum_t(*)[3])polygon->v, polygon->edge_v_indices_flat, polygon_edge_v_indices_cnt, p, closest_p);
}

void mathMeshClosestPoint(const GeometryMesh_t* mesh, const CCTNum_t p[3], CCTNum_t closest_p[3]) {
	CCTNum_t min_d;
	unsigned int i, flag = 0, mesh_edge_v_indices_cnt;
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		CCTNum_t cp[3];
		const GeometryPolygon_t* polygon = mesh->polygons + i;
		CCTNum_t d = mathPointProjectionPlane(p, polygon->v[polygon->v_indices[0]], polygon->normal);
		mathVec3Copy(cp, p);
		mathVec3AddScalar(cp, polygon->normal, d);
		if (!Polygon_Contain_Point_SamePlane(polygon, cp, NULL)) {
			continue;
		}
		if (!flag) {
			flag = 1;
		}
		else if (CCTNum_abs(min_d) <= CCTNum_abs(d)) {
			continue;
		}
		min_d = d;
		mathVec3Copy(closest_p, cp);
	}
	if (mesh->is_convex && flag) {
		return;
	}
	mesh_edge_v_indices_cnt = mesh->edge_cnt + mesh->edge_cnt;
	mathSegmentIndicesClosestPoint((const CCTNum_t(*)[3])mesh->v, mesh->edge_v_indices_flat, mesh_edge_v_indices_cnt, p, closest_p);
}

#ifdef	__cplusplus
}
#endif