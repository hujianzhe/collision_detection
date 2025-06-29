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
#include <stdlib.h>

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

int Segment_Contain_Point(const CCTNum_t ls0[3], const CCTNum_t ls1[3], const CCTNum_t p[3]) {
	CCTNum_t pv1[3], pv2[3], N[3], dot;
	mathVec3Sub(pv1, ls0, p);
	mathVec3Sub(pv2, ls1, p);
	mathVec3Cross(N, pv1, pv2);
	if (!mathVec3IsZero(N)) {
		return 0;
	}
	dot = mathVec3Dot(pv1, pv2);
	return dot <= CCT_EPSILON;
}

static int Segment_Contain_Segment(const CCTNum_t ls1[2][3], const CCTNum_t ls2[2][3]) {
	int i;
	CCTNum_t v1[3], v2[3], N[3];
	mathVec3Sub(v1, ls1[1], ls1[0]);
	mathVec3Sub(v2, ls2[1], ls2[0]);
	mathVec3Cross(N, v1, v2);
	if (!mathVec3IsZero(N)) {
		return 0;
	}
	for (i = 0; i < 2; ++i) {
		CCTNum_t dot;
		mathVec3Sub(v1, ls1[0], ls2[i]);
		mathVec3Sub(v2, ls1[1], ls2[i]);
		dot = mathVec3Dot(v1, v2);
		if (dot > CCT_EPSILON) {
			return 0;
		}
	}
	return 1;
}

int Plane_Contain_Point(const CCTNum_t plane_v[3], const CCTNum_t plane_normal[3], const CCTNum_t p[3]) {
	CCTNum_t v[3], dot;
	mathVec3Sub(v, plane_v, p);
	dot = mathVec3Dot(plane_normal, v);
	return CCT_EPSILON_NEGATE <= dot && dot <= CCT_EPSILON;
}

int Sphere_Contain_Point(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t p[3]) {
	CCTNum_t op[3], op_lensq;
	mathVec3Sub(op, p, o);
	op_lensq = mathVec3LenSq(op);
	return CCTNum_sq(radius) >= op_lensq;
}

static int Sphere_Contain_Sphere(const CCTNum_t o1[3], CCTNum_t r1, const CCTNum_t o2[3], CCTNum_t r2) {
	CCTNum_t o1o2[3], len_sq;
	if (r1 < r2) {
		return 0;
	}
	mathVec3Sub(o1o2, o2, o1);
	len_sq = mathVec3LenSq(o1o2);
	return len_sq <= CCTNum_sq(r1 - r2);
}

static int Sphere_Contain_Capsule(const CCTNum_t o[3], CCTNum_t r, const GeometryCapsule_t* capsule) {
	CCTNum_t sp_o[3];
	mathVec3Copy(sp_o, capsule->o);
	mathVec3SubScalar(sp_o, capsule->axis, capsule->half);
	if (!Sphere_Contain_Sphere(o, r, sp_o, capsule->radius)) {
		return 0;
	}
	mathVec3AddScalar(sp_o, capsule->axis, capsule->half + capsule->half);
	return Sphere_Contain_Sphere(o, r, sp_o, capsule->radius);
}

static int Box_Contain_Point(const CCTNum_t v[8][3], const CCTNum_t p[3]) {
	CCTNum_t vp[3], edge_v[3], dot;
	mathVec3Sub(vp, p, v[0]);

	mathVec3Sub(edge_v, v[1], v[0]);
	dot = mathVec3Dot(edge_v, vp);
	if (dot < CCTNum(0.0) || dot > mathVec3LenSq(edge_v)) {
		return 0;
	}
	mathVec3Sub(edge_v, v[4], v[0]);
	dot = mathVec3Dot(edge_v, vp);
	if (dot < CCTNum(0.0) || dot > mathVec3LenSq(edge_v)) {
		return 0;
	}
	mathVec3Sub(edge_v, v[3], v[0]);
	dot = mathVec3Dot(edge_v, vp);
	if (dot < CCTNum(0.0) || dot > mathVec3LenSq(edge_v)) {
		return 0;
	}
	return 1;
}

int OBB_Contain_Point(const GeometryOBB_t* obb, const CCTNum_t p[3]) {
	CCTNum_t op[3], dot;
	mathVec3Sub(op, p, obb->o);
	dot = mathVec3Dot(op, obb->axis[0]);
	if (dot > obb->half[0] + CCT_EPSILON || dot < -obb->half[0] - CCT_EPSILON) {
		return 0;
	}
	dot = mathVec3Dot(op, obb->axis[1]);
	if (dot > obb->half[1] + CCT_EPSILON || dot < -obb->half[1] - CCT_EPSILON) {
		return 0;
	}
	dot = mathVec3Dot(op, obb->axis[2]);
	if (dot > obb->half[2] + CCT_EPSILON || dot < -obb->half[2] - CCT_EPSILON) {
		return 0;
	}
	return 1;
}

static int OBB_Contain_OBB(const GeometryOBB_t* obb0, const GeometryOBB_t* obb1) {
	CCTNum_t AX[3][3], p[3];
	if (obb0 == obb1) {
		return 1;
	}
	mathVec3MultiplyScalar(AX[0], obb1->axis[0], obb1->half[0]);
	mathVec3MultiplyScalar(AX[1], obb1->axis[1], obb1->half[1]);
	mathVec3MultiplyScalar(AX[2], obb1->axis[2], obb1->half[2]);

	mathVec3Copy(p, obb1->o);
	mathVec3Sub(p, p, AX[0]);
	mathVec3Sub(p, p, AX[1]);
	mathVec3Sub(p, p, AX[2]);
	if (!OBB_Contain_Point(obb0, p)) {
		return 0;
	}
	mathVec3Copy(p, obb1->o);
	mathVec3Add(p, p, AX[0]);
	mathVec3Sub(p, p, AX[1]);
	mathVec3Sub(p, p, AX[2]);
	if (!OBB_Contain_Point(obb0, p)) {
		return 0;
	}
	mathVec3Copy(p, obb1->o);
	mathVec3Add(p, p, AX[0]);
	mathVec3Add(p, p, AX[1]);
	mathVec3Sub(p, p, AX[2]);
	if (!OBB_Contain_Point(obb0, p)) {
		return 0;
	}
	mathVec3Copy(p, obb1->o);
	mathVec3Sub(p, p, AX[0]);
	mathVec3Add(p, p, AX[1]);
	mathVec3Sub(p, p, AX[2]);
	if (!OBB_Contain_Point(obb0, p)) {
		return 0;
	}
	mathVec3Copy(p, obb1->o);
	mathVec3Sub(p, p, AX[0]);
	mathVec3Sub(p, p, AX[1]);
	mathVec3Add(p, p, AX[2]);
	if (!OBB_Contain_Point(obb0, p)) {
		return 0;
	}
	mathVec3Copy(p, obb1->o);
	mathVec3Add(p, p, AX[0]);
	mathVec3Sub(p, p, AX[1]);
	mathVec3Add(p, p, AX[2]);
	if (!OBB_Contain_Point(obb0, p)) {
		return 0;
	}
	mathVec3Copy(p, obb1->o);
	mathVec3Add(p, p, AX[0]);
	mathVec3Add(p, p, AX[1]);
	mathVec3Add(p, p, AX[2]);
	if (!OBB_Contain_Point(obb0, p)) {
		return 0;
	}
	mathVec3Copy(p, obb1->o);
	mathVec3Sub(p, p, AX[0]);
	mathVec3Add(p, p, AX[1]);
	mathVec3Add(p, p, AX[2]);
	if (!OBB_Contain_Point(obb0, p)) {
		return 0;
	}
	return 1;
}

static int OBB_Contain_Sphere(const GeometryOBB_t* obb, const CCTNum_t o[3], CCTNum_t radius) {
	int i;
	CCTNum_t v[3];
	mathVec3Sub(v, o, obb->o);
	for (i = 0; i < 3; ++i) {
		CCTNum_t dot = mathVec3Dot(v, obb->axis[i]);
		dot = CCTNum_abs(dot);
		if (dot > obb->half[i] - radius) {
			return 0;
		}
	}
	return 1;
}

int Capsule_Contain_Point(const GeometryCapsule_t* capsule, const CCTNum_t p[3]) {
	CCTNum_t v[3], dot, lensq, radius_sq = CCTNum_sq(capsule->radius);
	mathVec3Sub(v, p, capsule->o);
	dot = mathVec3Dot(v, capsule->axis);
	lensq = mathVec3LenSq(v) - CCTNum_sq(dot);
	if (lensq > radius_sq) {
		return 0;
	}
	if (CCTNum_abs(dot) <= capsule->half) {
		return 1;
	}
	mathVec3Copy(v, capsule->o);
	if (dot > CCTNum(0.0)) {
		mathVec3AddScalar(v, capsule->axis, capsule->half);
	}
	else {
		mathVec3SubScalar(v, capsule->axis, capsule->half);
	}
	lensq = mathVec3DistanceSq(v, p);
	return lensq <= radius_sq;
}

static int AABB_Contain_Mesh(const CCTNum_t o[3], const CCTNum_t half[3], const GeometryMesh_t* mesh) {
	unsigned int i;
	if (AABB_Contain_AABB(o, half, mesh->bound_box.o, mesh->bound_box.half)) {
		return 1;
	}
	for (i = 0; i < mesh->v_indices_cnt; ++i) {
		const CCTNum_t* p = mesh->v[mesh->v_indices[i]];
		if (!AABB_Contain_Point(o, half, p)) {
			return 0;
		}
	}
	return 1;
}

static int OBB_Contain_Capsule(const GeometryOBB_t* obb, const GeometryCapsule_t* capsule) {
	CCTNum_t sp_o[3];
	mathVec3Copy(sp_o, capsule->o);
	mathVec3SubScalar(sp_o, capsule->axis, capsule->half);
	if (!OBB_Contain_Sphere(obb, sp_o, capsule->radius)) {
		return 0;
	}
	mathVec3AddScalar(sp_o, capsule->axis, capsule->half + capsule->half);
	return OBB_Contain_Sphere(obb, sp_o, capsule->radius);
}

static int OBB_Contain_Mesh(const GeometryOBB_t* obb, const GeometryMesh_t* mesh) {
	CCTNum_t p[8][3];
	unsigned int i;
	mathAABBVertices(mesh->bound_box.o, mesh->bound_box.half, p);
	for (i = 0; i < 8; ++i) {
		if (!OBB_Contain_Point(obb, p[i])) {
			break;
		}
	}
	if (i >= 8) {
		return 1;
	}
	for (i = 0; i < mesh->v_indices_cnt; ++i) {
		const CCTNum_t* p = mesh->v[mesh->v_indices[i]];
		if (!OBB_Contain_Point(obb, p)) {
			return 0;
		}
	}
	return 1;
}

static int ConvexPolygon_Contain_Point_SamePlane(const GeometryPolygon_t* polygon, const CCTNum_t p[3], GeometryBorderId_t* bi) {
	unsigned int i, polygon_edge_v_indices_cnt = polygon->edge_cnt + polygon->edge_cnt;
	for (i = 0; i < polygon_edge_v_indices_cnt; ) {
		CCTNum_t ls_dir[3], ls_n[3], v[3], test_dot, dot;
		unsigned int edge_v_idx[2], other_i;
		/* test edge */
		edge_v_idx[0] = polygon->edge_v_indices_flat[i++];
		edge_v_idx[1] = polygon->edge_v_indices_flat[i++];
		mathVec3Sub(ls_dir, polygon->v[edge_v_idx[1]], polygon->v[edge_v_idx[0]]);
		mathVec3Cross(ls_n, ls_dir, polygon->normal);
		/* check edge contain p */
		mathVec3Sub(v, p, polygon->v[edge_v_idx[0]]);
		dot = mathVec3Dot(v, ls_n);
		if (dot >= CCT_EPSILON_NEGATE && dot <= CCT_EPSILON) {
			CCTNum_t l[3], r[3];
			mathVec3Sub(l, polygon->v[edge_v_idx[0]], p);
			mathVec3Sub(r, polygon->v[edge_v_idx[1]], p);
			dot = mathVec3Dot(l, r);
			if (dot > CCT_EPSILON) {
				return 0;
			}
			if (bi) {
				if (mathVec3IsZero(l)) {
					bi->v_id = polygon->edge_v_ids_flat[i - 2];
					bi->edge_id = -1;
				}
				else if (mathVec3IsZero(r)) {
					bi->v_id = polygon->edge_v_ids_flat[i - 1];
					bi->edge_id = -1;
				}
				else {
					bi->v_id = -1;
					bi->edge_id = (i - 1) / 2;
				}
			}
			return 1;
		}
		/* get test_point */
		other_i = (i < polygon_edge_v_indices_cnt ? i : 0);
		if (polygon->edge_v_indices_flat[other_i] == edge_v_idx[0] || polygon->edge_v_indices_flat[other_i] == edge_v_idx[1]) {
			++other_i;
		}
		mathVec3Sub(v, polygon->v[polygon->edge_v_indices_flat[other_i]], polygon->v[edge_v_idx[0]]);
		test_dot = mathVec3Dot(v, ls_n);
		/* check in same side */
		if (test_dot > CCTNum(0.0) && dot < CCTNum(0.0)) {
			return 0;
		}
		else if (test_dot < CCTNum(0.0) && dot > CCTNum(0.0)) {
			return 0;
		}
	}
	if (bi) {
		bi->v_id = bi->edge_id = -1;
	}
	return 1;
}

int Triangle_Contain_Point_SamePlane(const CCTNum_t a[3], const CCTNum_t b[3], const CCTNum_t c[3], const CCTNum_t N[3], const CCTNum_t p[3]) {
	CCTNum_t test_dot, dot, test_v[3];
	CCTNum_t ab[3], ac[3], bc[3], edge_N[3];
	/* edge ab test */
	mathVec3Sub(ab, b, a);
	mathVec3Cross(edge_N, ab, N);
	mathVec3Sub(test_v, p, a);
	dot = mathVec3Dot(test_v, edge_N);
	if (dot >= CCT_EPSILON_NEGATE && dot <= CCT_EPSILON) {
		CCTNum_t l[3], r[3];
		mathVec3Sub(l, a, p);
		mathVec3Sub(r, b, p);
		dot = mathVec3Dot(l, r);
		return dot <= CCT_EPSILON;
	}
	mathVec3Sub(ac, c, a);
	test_dot = mathVec3Dot(ac, edge_N);
	if (test_dot > CCTNum(0.0) && dot < CCTNum(0.0)) {
		return 0;
	}
	else if (test_dot < CCTNum(0.0) && dot > CCTNum(0.0)) {
		return 0;
	}
	/* edge ac test */
	mathVec3Cross(edge_N, ac, N);
	dot = mathVec3Dot(test_v, edge_N);
	if (dot >= CCT_EPSILON_NEGATE && dot <= CCT_EPSILON) {
		CCTNum_t l[3], r[3];
		mathVec3Sub(l, a, p);
		mathVec3Sub(r, c, p);
		dot = mathVec3Dot(l, r);
		return dot <= CCT_EPSILON;
	}
	test_dot = mathVec3Dot(ab, edge_N);
	if (test_dot > CCTNum(0.0) && dot < CCTNum(0.0)) {
		return 0;
	}
	else if (test_dot < CCTNum(0.0) && dot > CCTNum(0.0)) {
		return 0;
	}
	/* edge bc test */
	mathVec3Sub(bc, c, b);
	mathVec3Cross(edge_N, bc, N);
	mathVec3Sub(test_v, b, p);
	dot = mathVec3Dot(test_v, edge_N);
	if (dot >= CCT_EPSILON_NEGATE && dot <= CCT_EPSILON) {
		CCTNum_t r[3];
		mathVec3Sub(r, c, p);
		dot = mathVec3Dot(test_v, r);
		return dot <= CCT_EPSILON;
	}
	test_dot = mathVec3Dot(ab, edge_N);
	if (test_dot > CCTNum(0.0) && dot < CCTNum(0.0)) {
		return 0;
	}
	else if (test_dot < CCTNum(0.0) && dot > CCTNum(0.0)) {
		return 0;
	}
	/* finish */
	return 1;
}

static void concave_polygon_point_Locate_proc(const GeometryPolygon_t* polygon, unsigned int tri_v_flat_idx, const CCTNum_t p[3], GeometryBorderId_t* bi) {
	CCTNum_t l[3], r[3], N[3];
	unsigned int j;
	for (j = tri_v_flat_idx; j < tri_v_flat_idx + 3; ) {
		const CCTNum_t* v;
		unsigned int v_id = polygon->concave_tri_v_ids_flat[j++];
		if (v_id == -1) {
			continue;
		}
		v = polygon->v[polygon->v_indices[v_id]];
		if (mathVec3Equal(v, p)) {
			bi->v_id = v_id;
			bi->edge_id = -1;
			return;
		}
	}
	for (j = tri_v_flat_idx; j < tri_v_flat_idx + 3; ) {
		const CCTNum_t* v0, *v1;
		unsigned int v_id0, v_id1, edge_v_idx;
		unsigned int edge_id = polygon->concave_tri_edge_ids_flat[j++];
		if (-1 == edge_id) {
			break;
		}
		edge_v_idx = edge_id + edge_id;

		v_id0 = polygon->edge_v_ids_flat[edge_v_idx];
		v0 = polygon->v[polygon->v_indices[v_id0]];
		v_id1 = polygon->edge_v_ids_flat[edge_v_idx + 1];
		v1 = polygon->v[polygon->v_indices[v_id1]];
		mathVec3Sub(l, v0, p);
		mathVec3Sub(r, v1, p);
		mathVec3Cross(N, l, r);
		if (mathVec3IsZero(N)) {
			bi->v_id = -1;
			bi->edge_id = edge_id;
			return;
		}
	}
	bi->v_id = bi->edge_id = -1;
}

int Polygon_Contain_Point_SamePlane(const GeometryPolygon_t* polygon, const CCTNum_t p[3], GeometryBorderId_t* bi) {
	if ((const void*)polygon->v_indices >= (const void*)Box_Face_MeshVerticeIds &&
		(const void*)polygon->v_indices < (const void*)(Box_Face_MeshVerticeIds + 6))
	{
		CCTNum_t ls_vec[3], v[3], dot;
		mathVec3Sub(v, p, polygon->v[polygon->v_indices[0]]);
		mathVec3Sub(ls_vec, polygon->v[polygon->v_indices[1]], polygon->v[polygon->v_indices[0]]);
		dot = mathVec3Dot(ls_vec, v);
		if (dot < CCT_EPSILON_NEGATE || dot > mathVec3LenSq(ls_vec) + CCT_EPSILON) {
			return 0;
		}
		mathVec3Sub(ls_vec, polygon->v[polygon->v_indices[3]], polygon->v[polygon->v_indices[0]]);
		dot = mathVec3Dot(ls_vec, v);
		if (dot < CCT_EPSILON_NEGATE || dot > mathVec3LenSq(ls_vec) + CCT_EPSILON) {
			return 0;
		}
		if (bi) {
			unsigned int polygon_edge_v_indices_cnt = polygon->edge_cnt + polygon->edge_cnt;
			mathFindBorderIdByPoint((const CCTNum_t(*)[3])polygon->v, polygon->v_indices, polygon->edge_v_ids_flat, polygon_edge_v_indices_cnt, p, bi);
		}
		return 1;
	}
	if (polygon->is_convex) {
		return ConvexPolygon_Contain_Point_SamePlane(polygon, p, bi);
	}
	if (polygon->tri_v_indices_flat) {
		unsigned int i, polygon_tri_v_indices_cnt = polygon->tri_cnt * 3;
		for (i = 0; i < polygon_tri_v_indices_cnt; ) {
			unsigned int v_idx[3];
			v_idx[0] = polygon->tri_v_indices_flat[i++];
			v_idx[1] = polygon->tri_v_indices_flat[i++];
			v_idx[2] = polygon->tri_v_indices_flat[i++];
			if (!Triangle_Contain_Point_SamePlane(polygon->v[v_idx[0]], polygon->v[v_idx[1]], polygon->v[v_idx[2]], polygon->normal, p)) {
				continue;
			}
			if (bi) {
				concave_polygon_point_Locate_proc(polygon, i - 3, p, bi);
			}
			return 1;
		}
	}
	return 0;
}

int Polygon_Contain_Point(const GeometryPolygon_t* polygon, const CCTNum_t p[3]) {
	CCTNum_t v[3], dot;
	if (polygon->v_indices_cnt < 3) {
		return 0;
	}
	mathVec3Sub(v, p, polygon->v[polygon->v_indices[0]]);
	dot = mathVec3Dot(polygon->normal, v);
	if (dot < CCT_EPSILON_NEGATE || dot > CCT_EPSILON) {
		return 0;
	}
	return Polygon_Contain_Point_SamePlane(polygon, p, NULL);
}

static int Polygon_Contain_Segment(const GeometryPolygon_t* polygon, const CCTNum_t ls_p0[3], const CCTNum_t ls_p1[3]) {
	CCTNum_t ls_dir[3], v[3], dot;
	mathVec3Sub(ls_dir, ls_p1, ls_p0);
	dot = mathVec3Dot(ls_dir, polygon->normal);
	if (dot < CCT_EPSILON_NEGATE || dot > CCT_EPSILON) {
		return 0;
	}
	mathVec3Sub(v, ls_p0, polygon->v[polygon->v_indices[0]]);
	dot = mathVec3Dot(v, polygon->normal);
	if (dot < CCT_EPSILON_NEGATE || dot > CCT_EPSILON) {
		return 0;
	}
	if (!Polygon_Contain_Point_SamePlane(polygon, ls_p0, NULL)) {
		return 0;
	}
	if (!Polygon_Contain_Point_SamePlane(polygon, ls_p1, NULL)) {
		return 0;
	}
	if (!polygon->is_convex) {
		unsigned int i, polygon_edge_v_indics_cnt = polygon->edge_cnt + polygon->edge_cnt;
		CCTNum_t ls_len = mathVec3Normalized(ls_dir, ls_dir);
		for (i = 0; i < polygon_edge_v_indics_cnt; ) {
			const CCTNum_t* ep0 = polygon->v[polygon->edge_v_indices_flat[i++]];
			const CCTNum_t* ep1 = polygon->v[polygon->edge_v_indices_flat[i++]];
			CCTNum_t e_dir[3], N[3], d;
			mathVec3Sub(e_dir, ep1, ep0);
			mathVec3Cross(N, e_dir, ls_dir);
			if (mathVec3IsZero(N)) {
				continue;
			}
			mathVec3Normalized(e_dir, e_dir);
			d = mathLineCrossLine(ls_p0, ls_dir, ep0, e_dir);
			if (d < ls_len) {
				return 0;
			}
		}
	}
	return 1;
}

static int Polygon_Contain_Polygon(const GeometryPolygon_t* polygon1, const GeometryPolygon_t* polygon2) {
	unsigned int i, polygon1_edge_v_indices_cnt, polygon2_edge_v_indices_cnt;
	CCTNum_t(*polygon1_ls_dir_caches)[3];
	if (!mathPlaneEqual(polygon1->v[polygon1->v_indices[0]], polygon1->normal, polygon2->v[polygon2->v_indices[0]], polygon2->normal)) {
		return 0;
	}
	for (i = 0; i < polygon2->v_indices_cnt; ++i) {
		const CCTNum_t* p = polygon2->v[polygon2->v_indices[i]];
		if (!Polygon_Contain_Point_SamePlane(polygon1, p, NULL)) {
			return 0;
		}
	}
	if (polygon1->is_convex) {
		return 1;
	}
	polygon2_edge_v_indices_cnt = polygon2->edge_cnt + polygon2->edge_cnt;
	polygon1_edge_v_indices_cnt = polygon1->edge_cnt + polygon1->edge_cnt;
	polygon1_ls_dir_caches = (CCTNum_t(*)[3])malloc(sizeof(*polygon1_ls_dir_caches) * polygon1->edge_cnt);
	if (polygon1_ls_dir_caches) {
		for (i = 0; i < polygon1_edge_v_indices_cnt; ++i) {
			const CCTNum_t* ls1_v1 = polygon1->v[polygon1->edge_v_indices_flat[i++]];
			const CCTNum_t* ls1_v2 = polygon1->v[polygon1->edge_v_indices_flat[i]];
			mathVec3Sub(polygon1_ls_dir_caches[i / 2], ls1_v2, ls1_v1);
			mathVec3Normalized(polygon1_ls_dir_caches[i / 2], polygon1_ls_dir_caches[i / 2]);
		}
		for (i = 0; i < polygon2_edge_v_indices_cnt; ) {
			unsigned int j;
			CCTNum_t ls2_dir[3], ls2_len;
			const CCTNum_t* ls2_v1 = polygon2->v[polygon2->edge_v_indices_flat[i++]];
			const CCTNum_t* ls2_v2 = polygon2->v[polygon2->edge_v_indices_flat[i++]];
			mathVec3Sub(ls2_dir, ls2_v2, ls2_v1);
			ls2_len = mathVec3Normalized(ls2_dir, ls2_dir);
			for (j = 0; j < polygon1_edge_v_indices_cnt; j += 2) {
				CCTNum_t d, N[3];
				const CCTNum_t* ls1_dir = polygon1_ls_dir_caches[j / 2];
				mathVec3Cross(N, ls2_dir, ls1_dir);
				if (mathVec3IsZero(N)) {
					continue;
				}
				d = mathLineCrossLine(ls2_v1, ls2_dir, polygon1->v[polygon1->edge_v_indices_flat[j]], ls1_dir);
				if (d < ls2_len) {
					free(polygon1_ls_dir_caches);
					return 0;
				}
			}
		}
		free(polygon1_ls_dir_caches);
	}
	else {
		for (i = 0; i < polygon2_edge_v_indices_cnt; ) {
			unsigned int j;
			CCTNum_t ls2_dir[3], ls2_len;
			const CCTNum_t* ls2_v1 = polygon2->v[polygon2->edge_v_indices_flat[i++]];
			const CCTNum_t* ls2_v2 = polygon2->v[polygon2->edge_v_indices_flat[i++]];
			mathVec3Sub(ls2_dir, ls2_v2, ls2_v1);
			ls2_len = mathVec3Normalized(ls2_dir, ls2_dir);
			for (j = 0; j < polygon1_edge_v_indices_cnt; ) {
				CCTNum_t ls1_dir[3], N[3], d;
				const CCTNum_t* ls1_v1 = polygon1->v[polygon1->edge_v_indices_flat[j++]];
				const CCTNum_t* ls1_v2 = polygon1->v[polygon1->edge_v_indices_flat[j++]];
				mathVec3Sub(ls1_dir, ls1_v2, ls1_v1);
				mathVec3Cross(N, ls2_dir, ls1_dir);
				if (mathVec3IsZero(N)) {
					continue;
				}
				mathVec3Normalized(ls1_dir, ls1_dir);
				d = mathLineCrossLine(ls2_v1, ls2_dir, ls1_v1, ls1_dir);
				if (d < ls2_len) {
					return 0;
				}
			}
		}
	}
	return 1;
}

static int ConvexMesh_Contain_Point_InternalProc(const GeometryMesh_t* mesh, const CCTNum_t p[3]) {
	unsigned int i;
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		CCTNum_t v[3], dot;
		const GeometryPolygon_t* polygon = mesh->polygons + i;
		mathVec3Sub(v, p, polygon->v[polygon->v_indices[0]]);
		dot = mathVec3Dot(v, polygon->normal);
		if (dot < CCTNum(0.0)) {
			continue;
		}
		if (dot > CCT_EPSILON) {
			return 0;
		}
		return ConvexPolygon_Contain_Point_SamePlane(polygon, p, NULL);
	}
	return 1;
}

int ConvexMesh_Contain_Point(const GeometryMesh_t* mesh, const CCTNum_t p[3]) {
	if (Box_Vertice_Indices_Default == mesh->v_indices) {
		return Box_Contain_Point((const CCTNum_t(*)[3])mesh->v, p);
	}
	if (!AABB_Contain_Point(mesh->bound_box.o, mesh->bound_box.half, p)) {
		return 0;
	}
	return ConvexMesh_Contain_Point_InternalProc(mesh, p);
}

static int ConvexMesh_Contain_Mesh(const GeometryMesh_t* mesh1, const GeometryMesh_t* mesh2) {
	unsigned int i;
	if (!mathAABBIntersectAABB(mesh1->bound_box.o, mesh1->bound_box.half, mesh2->bound_box.o, mesh2->bound_box.half)) {
		return 0;
	}
	for (i = 0; i < mesh2->v_indices_cnt; ++i) {
		const CCTNum_t* p = mesh2->v[mesh2->v_indices[i]];
		if (!ConvexMesh_Contain_Point_InternalProc(mesh1, p)) {
			return 0;
		}
	}
	return 1;
}

static int ConvexMesh_Contain_AABB(const GeometryMesh_t* mesh, const CCTNum_t o[3], const CCTNum_t half[3]) {
	CCTNum_t p[8][3];
	unsigned int i;
	if (!AABB_Contain_AABB(mesh->bound_box.o, mesh->bound_box.half, o, half)) {
		return 0;
	}
	mathAABBVertices(o, half, p);
	for (i = 0; i < 8; ++i) {
		if (!ConvexMesh_Contain_Point(mesh, p[i])) {
			return 0;
		}
	}
	return 1;
}

static int ConvexMesh_Contain_OBB(const GeometryMesh_t* mesh, const GeometryOBB_t* obb) {
	CCTNum_t p[8][3];
	unsigned int i;
	mathOBBVertices(obb, p);
	for (i = 0; i < 8; ++i) {
		if (!ConvexMesh_Contain_Point(mesh, p[i])) {
			return 0;
		}
	}
	return 1;
}

static int ConvexMesh_Contain_Sphere(const GeometryMesh_t* mesh, const CCTNum_t o[3], CCTNum_t radius) {
	unsigned int i;
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		const GeometryPolygon_t* polygon = mesh->polygons + i;
		CCTNum_t v[3], d;
		mathVec3Sub(v, polygon->v[polygon->v_indices[0]], o);
		d = mathVec3Dot(v, polygon->normal);
		if (d < radius) {
			return 0;
		}
	}
	return 1;
}

static int ConvexMesh_Contain_Capsule(const GeometryMesh_t* mesh, const GeometryCapsule_t* capsule) {
	CCTNum_t sp_o[3];
	mathVec3Copy(sp_o, capsule->o);
	mathVec3SubScalar(sp_o, capsule->axis, capsule->half);
	if (!ConvexMesh_Contain_Sphere(mesh, sp_o, capsule->radius)) {
		return 0;
	}
	mathVec3AddScalar(sp_o, capsule->axis, capsule->half + capsule->half);
	return ConvexMesh_Contain_Sphere(mesh, sp_o, capsule->radius);
}

static int ConvexMesh_Contain_VerticeIndices(const GeometryMesh_t* mesh, const CCTNum_t(*v)[3], const unsigned int* v_indices, size_t v_indices_cnt) {
	unsigned int i;
	for (i = 0; i < v_indices_cnt; ++i) {
		const CCTNum_t* p = v[v_indices[i]];
		if (!ConvexMesh_Contain_Point(mesh, p)) {
			return 0;
		}
	}
	return 1;
}

static int OBB_Contain_VerticeIndices(const GeometryOBB_t* obb, const CCTNum_t(*v)[3], const unsigned int* v_indices, size_t v_indices_cnt) {
	unsigned int i;
	for (i = 0; i < v_indices_cnt; ++i) {
		const CCTNum_t* p = v[v_indices[i]];
		if (!OBB_Contain_Point(obb, p)) {
			return 0;
		}
	}
	return 1;
}

static int Sphere_Contain_VerticeIndices(const CCTNum_t o[3], CCTNum_t r, const CCTNum_t(*v)[3], const unsigned int* v_indices, size_t v_indices_cnt) {
	unsigned int i;
	for (i = 0; i < v_indices_cnt; ++i) {
		const CCTNum_t* p = v[v_indices[i]];
		if (!Sphere_Contain_Point(o, r, p)) {
			return 0;
		}
	}
	return 1;
}

static int Capsule_Contain_VerticeIndices(const GeometryCapsule_t* capsule, const CCTNum_t(*v)[3], const unsigned int* v_indices, size_t v_indices_cnt) {
	unsigned int i;
	for (i = 0; i < v_indices_cnt; ++i) {
		const CCTNum_t* p = v[v_indices[i]];
		if (!Capsule_Contain_Point(capsule, p)) {
			return 0;
		}
	}
	return 1;
}

static int Capsule_Contain_Sphere(const GeometryCapsule_t* capsule, const CCTNum_t o[3], CCTNum_t r) {
	CCTNum_t closest_p[3];
	mathSegmentClosestPoint_v2(capsule->o, capsule->axis, capsule->half, o, closest_p);
	return Sphere_Contain_Sphere(closest_p, capsule->radius, o, r);
}

static int Capsule_Contain_Capsule(const GeometryCapsule_t* capsule1, const GeometryCapsule_t* capsule2) {
	CCTNum_t sp_o[3];
	mathVec3Copy(sp_o, capsule2->o);
	mathVec3SubScalar(sp_o, capsule2->axis, capsule2->half);
	if (!Capsule_Contain_Sphere(capsule1, sp_o, capsule2->radius)) {
		return 0;
	}
	mathVec3AddScalar(sp_o, capsule2->axis, capsule2->half + capsule2->half);
	return Capsule_Contain_Sphere(capsule1, sp_o, capsule2->radius);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
extern "C" {
#endif

int mathGeometryContain(const void* geo_data1, int geo_type1, const void* geo_data2, int geo_type2) {
	if (geo_data1 == geo_data2) {
		return 1;
	}
	if (GEOMETRY_BODY_AABB == geo_type1) {
		const GeometryAABB_t* aabb1 = (const GeometryAABB_t*)geo_data1;
		switch (geo_type2) {
			case GEOMETRY_BODY_POINT:
			{
				const CCTNum_t* point2 = (const CCTNum_t*)geo_data2;
				return AABB_Contain_Point(aabb1->o, aabb1->half, point2);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				return	AABB_Contain_Point(aabb1->o, aabb1->half, segment2->v[0]) &&
					AABB_Contain_Point(aabb1->o, aabb1->half, segment2->v[1]);
			}
			case GEOMETRY_BODY_AABB:
			{
				const GeometryAABB_t* aabb2 = (const GeometryAABB_t*)geo_data2;
				return AABB_Contain_AABB(aabb1->o, aabb1->half, aabb2->o, aabb2->half);
			}
			case GEOMETRY_BODY_OBB:
			{
				GeometryOBB_t obb1;
				mathOBBFromAABB(&obb1, aabb1->o, aabb1->half);
				return OBB_Contain_OBB(&obb1, (const GeometryOBB_t*)geo_data2);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				const GeometryPolygon_t* polygon2 = (const GeometryPolygon_t*)geo_data2;
				GeometryOBB_t obb1;
				mathOBBFromAABB(&obb1, aabb1->o, aabb1->half);
				return OBB_Contain_VerticeIndices(&obb1, (const CCTNum_t(*)[3])polygon2->v, polygon2->v_indices, polygon2->v_indices_cnt);
			}
			case GEOMETRY_BODY_MESH:
			{
				return AABB_Contain_Mesh(aabb1->o, aabb1->half, (const GeometryMesh_t*)geo_data2);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				const GeometrySphere_t* sphere2 = (const GeometrySphere_t*)geo_data2;
				GeometryOBB_t obb1;
				mathOBBFromAABB(&obb1, aabb1->o, aabb1->half);
				return OBB_Contain_Sphere(&obb1, sphere2->o, sphere2->radius);
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				const GeometryCapsule_t* capsule2 = (const GeometryCapsule_t*)geo_data2;
				GeometryOBB_t obb1;
				mathOBBFromAABB(&obb1, aabb1->o, aabb1->half);
				return OBB_Contain_Capsule(&obb1, capsule2);
			}
		}
	}
	else if (GEOMETRY_BODY_OBB == geo_type1) {
		const GeometryOBB_t* obb1 = (const GeometryOBB_t*)geo_data1;
		switch (geo_type2) {
			case GEOMETRY_BODY_POINT:
			{
				const CCTNum_t* point2 = (const CCTNum_t*)geo_data2;
				return OBB_Contain_Point(obb1, point2);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				return OBB_Contain_Point(obb1, segment2->v[0]) && OBB_Contain_Point(obb1, segment2->v[1]);
			}
			case GEOMETRY_BODY_AABB:
			{
				const GeometryAABB_t* aabb2 = (const GeometryAABB_t*)geo_data2;
				GeometryOBB_t obb2;
				mathOBBFromAABB(&obb2, aabb2->o, aabb2->half);
				return OBB_Contain_OBB(obb1, &obb2);
			}
			case GEOMETRY_BODY_OBB:
			{
				return OBB_Contain_OBB(obb1, (const GeometryOBB_t*)geo_data2);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				const GeometryPolygon_t* polygon2 = (const GeometryPolygon_t*)geo_data2;
				return OBB_Contain_VerticeIndices(obb1, (const CCTNum_t(*)[3])polygon2->v, polygon2->v_indices, polygon2->v_indices_cnt);
			}
			case GEOMETRY_BODY_MESH:
			{
				return OBB_Contain_Mesh(obb1, (const GeometryMesh_t*)geo_data2);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				const GeometrySphere_t* sphere2 = (const GeometrySphere_t*)geo_data2;
				return OBB_Contain_Sphere(obb1, sphere2->o, sphere2->radius);
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				return OBB_Contain_Capsule(obb1, (const GeometryCapsule_t*)geo_data2);
			}
		}
	}
	else if (GEOMETRY_BODY_SPHERE == geo_type1) {
		const GeometrySphere_t* sphere1 = (const GeometrySphere_t*)geo_data1;
		switch (geo_type2) {
			case GEOMETRY_BODY_POINT:
			{
				const CCTNum_t* point2 = (const CCTNum_t*)geo_data2;
				return Sphere_Contain_Point(sphere1->o, sphere1->radius, point2);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				return Sphere_Contain_Point(sphere1->o, sphere1->radius, segment2->v[0]) &&
					Sphere_Contain_Point(sphere1->o, sphere1->radius, segment2->v[1]);
			}
			case GEOMETRY_BODY_AABB:
			{
				const GeometryAABB_t* aabb2 = (const GeometryAABB_t*)geo_data2;
				CCTNum_t v[8][3];
				mathAABBVertices(aabb2->o, aabb2->half, v);
				return Sphere_Contain_VerticeIndices(sphere1->o, sphere1->radius, (const CCTNum_t(*)[3])v,
					Box_Vertice_Indices_Default, sizeof(Box_Vertice_Indices_Default) / sizeof(Box_Vertice_Indices_Default[0]));
			}
			case GEOMETRY_BODY_OBB:
			{
				const GeometryOBB_t* obb2 = (const GeometryOBB_t*)geo_data2;
				CCTNum_t v[8][3];
				mathOBBVertices(obb2, v);
				return Sphere_Contain_VerticeIndices(sphere1->o, sphere1->radius, (const CCTNum_t(*)[3])v,
					Box_Vertice_Indices_Default, sizeof(Box_Vertice_Indices_Default) / sizeof(Box_Vertice_Indices_Default[0]));
			}
			case GEOMETRY_BODY_POLYGON:
			{
				const GeometryPolygon_t* polygon2 = (const GeometryPolygon_t*)geo_data2;
				return Sphere_Contain_VerticeIndices(sphere1->o, sphere1->radius,
						(const CCTNum_t(*)[3])polygon2->v, polygon2->v_indices, polygon2->v_indices_cnt);
			}
			case GEOMETRY_BODY_MESH:
			{
				const GeometryMesh_t* mesh2 = (const GeometryMesh_t*)geo_data2;
				return Sphere_Contain_VerticeIndices(sphere1->o, sphere1->radius,
						(const CCTNum_t(*)[3])mesh2->v, mesh2->v_indices, mesh2->v_indices_cnt);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				const GeometrySphere_t* sphere2 = (GeometrySphere_t*)geo_data2;
				return Sphere_Contain_Sphere(sphere1->o, sphere1->radius, sphere2->o, sphere2->radius);
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				const GeometryCapsule_t* capsule2 = (GeometryCapsule_t*)geo_data2;
				return Sphere_Contain_Capsule(sphere1->o, sphere1->radius, capsule2);
			}
		}
	}
	else if (GEOMETRY_BODY_MESH == geo_type1) {
		const GeometryMesh_t* mesh1 = (const GeometryMesh_t*)geo_data1;
		if (!mesh1->is_convex || !mesh1->is_closed) {
			return 0;
		}
		switch (geo_type2) {
			case GEOMETRY_BODY_POINT:
			{
				const CCTNum_t* point2 = (const CCTNum_t*)geo_data2;
				return ConvexMesh_Contain_Point(mesh1, point2);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				return ConvexMesh_Contain_Point(mesh1, segment2->v[0]) && ConvexMesh_Contain_Point(mesh1, segment2->v[1]);
			}
			case GEOMETRY_BODY_AABB:
			{
				const GeometryAABB_t* aabb2 = (const GeometryAABB_t*)geo_data2;
				return ConvexMesh_Contain_AABB(mesh1, aabb2->o, aabb2->half);
			}
			case GEOMETRY_BODY_OBB:
			{
				const GeometryOBB_t* obb2 = (const GeometryOBB_t*)geo_data2;
				return ConvexMesh_Contain_OBB(mesh1, obb2);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				const GeometryPolygon_t* polygon2 = (const GeometryPolygon_t*)geo_data2;
				return ConvexMesh_Contain_VerticeIndices(mesh1, (const CCTNum_t(*)[3])polygon2->v, polygon2->v_indices, polygon2->v_indices_cnt);
			}
			case GEOMETRY_BODY_MESH:
			{
				return ConvexMesh_Contain_Mesh(mesh1, (const GeometryMesh_t*)geo_data2);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				const GeometrySphere_t* sphere2 = (const GeometrySphere_t*)geo_data2;
				return ConvexMesh_Contain_Sphere(mesh1, sphere2->o, sphere2->radius);
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				return ConvexMesh_Contain_Capsule(mesh1, (const GeometryCapsule_t*)geo_data2);
			}
		}
	}
	else if (GEOMETRY_BODY_CAPSULE == geo_type1) {
		const GeometryCapsule_t* capsule1 = (const GeometryCapsule_t*)geo_data1;
		switch (geo_type2) {
			case GEOMETRY_BODY_POINT:
			{
				const CCTNum_t* point2 = (const CCTNum_t*)geo_data2;
				return Capsule_Contain_Point(capsule1, point2);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				return Capsule_Contain_Point(capsule1, segment2->v[0]) && Capsule_Contain_Point(capsule1, segment2->v[1]);
			}
			case GEOMETRY_BODY_AABB:
			{
				const GeometryAABB_t* aabb2 = (const GeometryAABB_t*)geo_data2;
				CCTNum_t v[8][3];
				mathAABBVertices(aabb2->o, aabb2->half, v);
				return Capsule_Contain_VerticeIndices(capsule1, (const CCTNum_t(*)[3])v,
					Box_Vertice_Indices_Default, sizeof(Box_Vertice_Indices_Default) / sizeof(Box_Vertice_Indices_Default[0]));
			}
			case GEOMETRY_BODY_OBB:
			{
				const GeometryOBB_t* obb2 = (const GeometryOBB_t*)geo_data2;
				CCTNum_t v[8][3];
				mathOBBVertices(obb2, v);
				return Capsule_Contain_VerticeIndices(capsule1, (const CCTNum_t(*)[3])v,
					Box_Vertice_Indices_Default, sizeof(Box_Vertice_Indices_Default) / sizeof(Box_Vertice_Indices_Default[0]));
			}
			case GEOMETRY_BODY_POLYGON:
			{
				const GeometryPolygon_t* polygon2 = (const GeometryPolygon_t*)geo_data2;
				return Capsule_Contain_VerticeIndices(capsule1, (const CCTNum_t(*)[3])polygon2->v, polygon2->v_indices, polygon2->v_indices_cnt);
			}
			case GEOMETRY_BODY_MESH:
			{
				const GeometryMesh_t* mesh2 = (const GeometryMesh_t*)geo_data2;
				return Capsule_Contain_VerticeIndices(capsule1, (const CCTNum_t(*)[3])mesh2->v, mesh2->v_indices, mesh2->v_indices_cnt);
			}
			case GEOMETRY_BODY_SPHERE:
			{
				const GeometrySphere_t* sphere2 = (const GeometrySphere_t*)geo_data2;
				return Capsule_Contain_Sphere(capsule1, sphere2->o, sphere2->radius);
			}
			case GEOMETRY_BODY_CAPSULE:
			{
				return Capsule_Contain_Capsule(capsule1, (const GeometryCapsule_t*)geo_data2);
			}
		}
	}
	else if (GEOMETRY_BODY_POLYGON == geo_type1) {
		const GeometryPolygon_t* polygon1 = (const GeometryPolygon_t*)geo_data1;
		switch (geo_type2) {
			case GEOMETRY_BODY_POINT:
			{
				const CCTNum_t* point2 = (const CCTNum_t*)geo_data2;
				return Polygon_Contain_Point(polygon1, point2);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				return Polygon_Contain_Segment(polygon1, segment2->v[0], segment2->v[1]);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				return Polygon_Contain_Polygon(polygon1, (const GeometryPolygon_t*)geo_data2);
			}
		}
	}
	else if (GEOMETRY_BODY_PLANE == geo_type1) {
		const GeometryPlane_t* plane1 = (const GeometryPlane_t*)geo_data1;
		switch (geo_type2) {
			case GEOMETRY_BODY_POINT:
			{
				const CCTNum_t* point2 = (const CCTNum_t*)geo_data2;
				return Plane_Contain_Point(plane1->v, plane1->normal, point2);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				return Plane_Contain_Point(plane1->v, plane1->normal, segment2->v[0]) &&
					Plane_Contain_Point(plane1->v, plane1->normal, segment2->v[1]);
			}
			case GEOMETRY_BODY_PLANE:
			{
				const GeometryPlane_t* plane2 = (const GeometryPlane_t*)geo_data2;
				return mathPlaneEqual(plane1->v, plane1->normal, plane2->v, plane2->normal);
			}
			case GEOMETRY_BODY_POLYGON:
			{
				const GeometryPolygon_t* polygon2 = (const GeometryPolygon_t*)geo_data2;
				return mathPlaneEqual(plane1->v, plane1->normal, polygon2->v[polygon2->v_indices[0]], polygon2->normal);
			}
		}
	}
	else if (GEOMETRY_BODY_SEGMENT == geo_type1) {
		const GeometrySegment_t* segment1 = (const GeometrySegment_t*)geo_data1;
		switch (geo_type2) {
			case GEOMETRY_BODY_POINT:
			{
				const CCTNum_t* point2 = (const CCTNum_t*)geo_data2;
				return Segment_Contain_Point(segment1->v[0], segment1->v[1], point2);
			}
			case GEOMETRY_BODY_SEGMENT:
			{
				const GeometrySegment_t* segment2 = (const GeometrySegment_t*)geo_data2;
				return Segment_Contain_Segment((const CCTNum_t(*)[3])segment1->v, (const CCTNum_t(*)[3])segment2->v);
			}
		}
	}
	else if (GEOMETRY_BODY_POINT == geo_type1) {
		if (GEOMETRY_BODY_POINT == geo_type2) {
			return mathVec3Equal((const CCTNum_t*)geo_data1, (const CCTNum_t*)geo_data2);
		}
	}
	return 0;
}

#ifdef __cplusplus
}
#endif
