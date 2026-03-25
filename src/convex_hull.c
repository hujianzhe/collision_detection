#include "../inc/math_vec3.h"
#include "../inc/convex_hull.h"
#include <stdlib.h>

static CCTNum_t plane_signed_distance_unnorm(const CCTNum_t a[3],
                                             const CCTNum_t b[3],
                                             const CCTNum_t c[3],
                                             const CCTNum_t p[3]) {
	CCTNum_t ab[3], ac[3], ap[3], n[3];
	mathVec3Sub(ab, b, a);
	mathVec3Sub(ac, c, a);
	mathVec3Cross(n, ab, ac);
	mathVec3Sub(ap, p, a);
	return mathVec3Dot(n, ap);
}

static CCTNum_t cross_2d(const CCTNum_t a[2],
                         const CCTNum_t b[2],
                         const CCTNum_t c[2]) {
	CCTNum_t abx = b[0] - a[0];
	CCTNum_t aby = b[1] - a[1];
	CCTNum_t acx = c[0] - a[0];
	CCTNum_t acy = c[1] - a[1];
	return abx * acy - aby * acx;
}

static void sort_indices_by_proj_2d(const CCTNum_t *proj,
                                    unsigned int *idx,
                                    unsigned int n) {
	unsigned int i, j;
	for (i = 0; i < n; ++i) {
		idx[i] = i;
	}
	for (i = 0; i < n; ++i) {
		for (j = i + 1; j < n; ++j) {
			const CCTNum_t *Ai = proj + 2 * idx[i];
			const CCTNum_t *Aj = proj + 2 * idx[j];
			if (Ai[0] > Aj[0] || (Ai[0] == Aj[0] && Ai[1] > Aj[1])) {
				unsigned int t = idx[i];
				idx[i] = idx[j];
				idx[j] = t;
			}
		}
	}
}

static void project_points_onto_2d(const CCTNum_t (*v)[3],
                                   unsigned int v_cnt,
                                   const unsigned int i_arr[3],
                                   CCTNum_t *proj,
                                   unsigned int *idx) {
	unsigned int i0 = i_arr[0];
	unsigned int i1 = i_arr[1];
	unsigned int i2 = i_arr[2];
	CCTNum_t n[3];
	CCTNum_t v0v1[3], v0v2[3];
	CCTNum_t an, bn, cn;
	int drop_axis;
	unsigned int i;

	mathVec3Sub(v0v1, v[i1], v[i0]);
	mathVec3Sub(v0v2, v[i2], v[i0]);
	mathVec3Cross(n, v0v1, v0v2);

	an = CCTNum_abs(n[0]);
	bn = CCTNum_abs(n[1]);
	cn = CCTNum_abs(n[2]);
	if (an >= bn && an >= cn) {
		drop_axis = 0;
	}
	else if (bn >= an && bn >= cn) {
		drop_axis = 1;
	}
	else {
		drop_axis = 2;
	}

	{
		const unsigned int axis_u[3] = { 1, 0, 0 };
		const unsigned int axis_w[3] = { 2, 2, 1 };
		unsigned int au = axis_u[drop_axis];
		unsigned int aw = axis_w[drop_axis];
		for (i = 0; i < v_cnt; ++i) {
			proj[2 * i + 0] = v[i][au];
			proj[2 * i + 1] = v[i][aw];
		}
	}

	sort_indices_by_proj_2d(proj, idx, v_cnt);
}

static void init_tetra_faces(const CCTNum_t (*v)[3],
                             const unsigned int i_arr[4],
                             const CCTNum_t interior[3],
                             unsigned int *fa,
                             unsigned int *fb,
                             unsigned int *fc,
                             unsigned char *fvalid) {
	unsigned int i0 = i_arr[0];
	unsigned int i1 = i_arr[1];
	unsigned int i2 = i_arr[2];
	unsigned int i3 = i_arr[3];
	unsigned int init_faces[4][3];
	unsigned int f;

	init_faces[0][0] = i0; init_faces[0][1] = i1; init_faces[0][2] = i2;
	init_faces[1][0] = i0; init_faces[1][1] = i3; init_faces[1][2] = i1;
	init_faces[2][0] = i0; init_faces[2][1] = i2; init_faces[2][2] = i3;
	init_faces[3][0] = i1; init_faces[3][1] = i3; init_faces[3][2] = i2;

	for (f = 0; f < 4; ++f) {
		CCTNum_t d;
		fa[f] = init_faces[f][0];
		fb[f] = init_faces[f][1];
		fc[f] = init_faces[f][2];
		d = plane_signed_distance_unnorm(v[fa[f]], v[fb[f]], v[fc[f]], interior);
		if (d > CCTNum(0.0)) {
			unsigned int tmp = fb[f];
			fb[f] = fc[f];
			fc[f] = tmp;
		}
		fvalid[f] = 1;
	}
}

static int build_convex_hull_2d_chain(const CCTNum_t *proj,
                                      unsigned int v_cnt,
                                      const unsigned int *idx,
                                      unsigned int **out_hull,
                                      unsigned int *out_h_cnt) {
	unsigned int *hull;
	unsigned int h_cnt, lower_size;
	unsigned int k;

	hull = (unsigned int*)malloc(sizeof(unsigned int) * (2 * v_cnt));
	if (!hull) {
		return 0;
	}
	h_cnt = 0;

	for (k = 0; k < v_cnt; ++k) {
		unsigned int i = idx[k];
		const CCTNum_t *p = proj + 2 * i;
		while (h_cnt >= 2) {
			unsigned int j1 = hull[h_cnt - 2];
			unsigned int j2 = hull[h_cnt - 1];
			const CCTNum_t *a = proj + 2 * j1;
			const CCTNum_t *b = proj + 2 * j2;
			if (cross_2d(a, b, p) <= CCT_EPSILON) {
				--h_cnt;
			}
			else {
				break;
			}
		}
		hull[h_cnt++] = i;
	}

	lower_size = h_cnt;
	for (k = v_cnt; k > 0; --k) {
		unsigned int i = idx[k - 1];
		const CCTNum_t *p = proj + 2 * i;
		while (h_cnt > lower_size) {
			unsigned int j1 = hull[h_cnt - 2];
			unsigned int j2 = hull[h_cnt - 1];
			const CCTNum_t *a = proj + 2 * j1;
			const CCTNum_t *b = proj + 2 * j2;
			if (cross_2d(a, b, p) <= CCT_EPSILON) {
				--h_cnt;
			}
			else {
				break;
			}
		}
		hull[h_cnt++] = i;
	}
	if (h_cnt > 1) {
		--h_cnt;
	}

	if (h_cnt < 3) {
		free(hull);
		return 0;
	}

	*out_hull = hull;
	*out_h_cnt = h_cnt;
	return 1;
}

static int triangulate_convex_hull_2d(const unsigned int *hull,
                                      unsigned int h_cnt,
                                      unsigned int **out_faces,
                                      unsigned int *out_tri_v_indices_cnt) {
	unsigned int tri_cnt;
	unsigned int *faces;
	unsigned int base;
	unsigned int t;

	if (h_cnt < 3) {
		return 0;
	}

	tri_cnt = h_cnt - 2;
	faces = (unsigned int*)malloc(sizeof(unsigned int) * 3 * tri_cnt);
	if (!faces) {
		return 0;
	}

	base = hull[0];
	{
		unsigned int *p = faces;
		for (t = 0; t < tri_cnt; ++t) {
			*(p++) = base;
			*(p++) = hull[t + 1];
			*(p++) = hull[t + 2];
		}
	}
	*out_faces = faces;
	*out_tri_v_indices_cnt = tri_cnt * 3;
	return 1;
}

static int build_convex_hull_2d(const CCTNum_t (*v)[3],
                                unsigned int v_cnt,
                                const unsigned int i_arr[3],
                                unsigned int **ret_tri_v_indices,
                                unsigned int *ret_tri_v_indices_cnt) {
	CCTNum_t *proj = NULL;
	unsigned int *idx = NULL;
	unsigned int *hull = NULL;
	unsigned int h_cnt = 0;

	proj = (CCTNum_t*)malloc(sizeof(CCTNum_t) * 2 * v_cnt);
	if (!proj) {
		goto cleanup;
	}
	idx = (unsigned int*)malloc(sizeof(unsigned int) * v_cnt);
	if (!idx) {
		goto cleanup;
	}

	project_points_onto_2d(v, v_cnt, i_arr, proj, idx);

	if (!build_convex_hull_2d_chain(proj, v_cnt, idx, &hull, &h_cnt)) {
		goto cleanup;
	}

	if (!triangulate_convex_hull_2d(hull, h_cnt, ret_tri_v_indices, ret_tri_v_indices_cnt)) {
		goto cleanup;
	}

cleanup:
	free(proj);
	free(idx);
	free(hull);
	return (*ret_tri_v_indices) != NULL;
}

static int collect_convex_hull_faces(unsigned int face_cnt,
                                    const unsigned int *fa,
                                    const unsigned int *fb,
                                    const unsigned int *fc,
                                    const unsigned char *fvalid,
                                    unsigned int **ret_tri_v_indices,
                                    unsigned int *ret_tri_v_indices_cnt) {
	unsigned int n = 0;
	unsigned int f;
	unsigned int *out;

	for (f = 0; f < face_cnt; ++f) {
		if (fvalid[f]) {
			++n;
		}
	}
	if (n == 0) {
		return 0;
	}
	out = (unsigned int*)malloc(sizeof(unsigned int) * 3 * n);
	if (!out) {
		return 0;
	}
	*ret_tri_v_indices = out;
	*ret_tri_v_indices_cnt = n * 3;
	for (f = 0; f < face_cnt; ++f) {
		if (fvalid[f]) {
			*(out++) = fa[f];
			*(out++) = fb[f];
			*(out++) = fc[f];
		}
	}
	return 1;
}

static int build_convex_hull_3d(const CCTNum_t (*v)[3], unsigned int v_cnt, const unsigned int i_arr[4], unsigned int **ret_tri_v_indices, unsigned int *ret_tri_v_indices_cnt) {
	CCTNum_t interior[3];
	unsigned int i0 = i_arr[0];
	unsigned int i1 = i_arr[1];
	unsigned int i2 = i_arr[2];
	unsigned int i3 = i_arr[3];
	unsigned int face_cap;
	unsigned int *fa = NULL;
	unsigned int *fb = NULL;
	unsigned int *fc = NULL;
	unsigned char *fvalid = NULL;
	unsigned char *visible = NULL;
	unsigned int edge_cap;
	unsigned int *edge_u = NULL;
	unsigned int *edge_v = NULL;
	unsigned char *edge_used = NULL;
	unsigned int face_cnt;
	unsigned int pi;
	int ret = 0;

	face_cap = (v_cnt < 16 ? 16 : (4 * v_cnt));
	fa = (unsigned int*)malloc(sizeof(unsigned int) * face_cap);
	if (!fa) {
		goto cleanup;
	}
	fb = (unsigned int*)malloc(sizeof(unsigned int) * face_cap);
	if (!fb) {
		goto cleanup;
	}
	fc = (unsigned int*)malloc(sizeof(unsigned int) * face_cap);
	if (!fc) {
		goto cleanup;
	}
	fvalid = (unsigned char*)malloc(sizeof(unsigned char) * face_cap);
	if (!fvalid) {
		goto cleanup;
	}
	visible = (unsigned char*)malloc(sizeof(unsigned char) * face_cap);
	if (!visible) {
		goto cleanup;
	}

	interior[0] = (v[i0][0] + v[i1][0] + v[i2][0] + v[i3][0]) * CCTNum(0.25);
	interior[1] = (v[i0][1] + v[i1][1] + v[i2][1] + v[i3][1]) * CCTNum(0.25);
	interior[2] = (v[i0][2] + v[i1][2] + v[i2][2] + v[i3][2]) * CCTNum(0.25);

	init_tetra_faces(v, i_arr, interior, fa, fb, fc, fvalid);
	face_cnt = 4;

	edge_cap = 3 * face_cap;
	edge_u = (unsigned int*)malloc(sizeof(unsigned int) * edge_cap);
	if (!edge_u) {
		goto cleanup;
	}
	edge_v = (unsigned int*)malloc(sizeof(unsigned int) * edge_cap);
	if (!edge_v) {
		goto cleanup;
	}
	edge_used = (unsigned char*)malloc(sizeof(unsigned char) * edge_cap);
	if (!edge_used) {
		goto cleanup;
	}

	for (pi = 0; pi < v_cnt; ++pi) {
		unsigned int f, edge_cnt, k;
		int any_visible = 0;

		if (pi == i0 || pi == i1 || pi == i2 || pi == i3) {
			continue;
		}

		for (f = 0; f < face_cnt; ++f) {
			CCTNum_t d;
			if (!fvalid[f]) {
				visible[f] = 0;
				continue;
			}
			d = plane_signed_distance_unnorm(v[fa[f]], v[fb[f]], v[fc[f]], v[pi]);
			visible[f] = (d > CCT_EPSILON) ? 1 : 0;
			any_visible |= visible[f];
		}
		if (!any_visible) {
			continue;
		}

		edge_cnt = 0;
		for (f = 0; f < face_cnt; ++f) {
			unsigned int va[3];
			unsigned int e;

			if (!fvalid[f] || !visible[f]) {
				continue;
			}

			va[0] = fa[f];
			va[1] = fb[f];
			va[2] = fc[f];

			for (e = 0; e < 3; ++e) {
				unsigned int u = va[e];
				unsigned int vtx = va[(e + 1) % 3];
				unsigned int k;
				int cancelled = 0;

				for (k = 0; k < edge_cnt; ++k) {
					if (!edge_used[k]) {
						continue;
					}
					if (edge_u[k] == vtx && edge_v[k] == u) {
						edge_used[k] = 0;
						cancelled = 1;
						break;
					}
				}
				if (cancelled) {
					continue;
				}

				if (edge_cnt >= edge_cap) {
					unsigned int new_cap = edge_cap * 2;
					void* tmp_p;

					tmp_p = realloc(edge_u, sizeof(unsigned int) * new_cap);
					if (!tmp_p) {
						goto cleanup;
					}
					edge_u = (unsigned int*)tmp_p;

					tmp_p = realloc(edge_v, sizeof(unsigned int) * new_cap);
					if (!tmp_p) {
						goto cleanup;
					}
					edge_v = (unsigned int*)tmp_p;

					tmp_p = realloc(edge_used, sizeof(unsigned char) * new_cap);
					if (!tmp_p) {
						goto cleanup;
					}
					edge_used = (unsigned char*)tmp_p;

					edge_cap = new_cap;
				}

				edge_u[edge_cnt] = u;
				edge_v[edge_cnt] = vtx;
				edge_used[edge_cnt] = 1;
				++edge_cnt;
			}
		}

		for (f = 0; f < face_cnt; ++f) {
			if (visible[f]) {
				fvalid[f] = 0;
			}
		}

		for (k = 0; k < edge_cnt; ++k) {
			unsigned int u;
			unsigned int vtx;
			unsigned int f_idx;
			CCTNum_t d;

			if (!edge_used[k]) {
				continue;
			}

			u = edge_u[k];
			vtx = edge_v[k];

			if (face_cnt >= face_cap) {
				unsigned int new_cap = face_cap * 2;
				void* tmp_p;

				tmp_p = realloc(fa, sizeof(unsigned int) * new_cap);
				if (!tmp_p) {
					goto cleanup;
				}
				fa = (unsigned int*)tmp_p;

				tmp_p = realloc(fb, sizeof(unsigned int) * new_cap);
				if (!tmp_p) {
					goto cleanup;
				}
				fb = (unsigned int*)tmp_p;

				tmp_p = realloc(fc, sizeof(unsigned int) * new_cap);
				if (!tmp_p) {
					goto cleanup;
				}
				fc = (unsigned int*)tmp_p;

				tmp_p = realloc(fvalid, sizeof(unsigned char) * new_cap);
				if (!tmp_p) {
					goto cleanup;
				}
				fvalid = (unsigned char*)tmp_p;

				tmp_p = realloc(visible, sizeof(unsigned char) * new_cap);
				if (!tmp_p) {
					goto cleanup;
				}
				visible = (unsigned char*)tmp_p;

				face_cap = new_cap;
			}

			f_idx = face_cnt++;
			fa[f_idx] = u;
			fb[f_idx] = vtx;
			fc[f_idx] = pi;
			fvalid[f_idx] = 1;

			d = plane_signed_distance_unnorm(v[fa[f_idx]], v[fb[f_idx]], v[fc[f_idx]], interior);
			if (d > CCTNum(0.0)) {
				unsigned int tmp = fb[f_idx];
				fb[f_idx] = fc[f_idx];
				fc[f_idx] = tmp;
			}
		}
	}

	if (!collect_convex_hull_faces(face_cnt, fa, fb, fc, fvalid,
	                               ret_tri_v_indices, ret_tri_v_indices_cnt)) {
		goto cleanup;
	}
	ret = 1;

cleanup:
	free(edge_u);
	free(edge_v);
	free(edge_used);
	free(fa);
	free(fb);
	free(fc);
	free(fvalid);
	free(visible);
	return ret;
}

static int init_tetrahedron_indices(const CCTNum_t (*v)[3], unsigned int v_cnt, unsigned int i_arr[4], int *coplanar_non_degenerate) {
	unsigned int i;
	CCTNum_t maxd2 = CCTNum(0.0);
	CCTNum_t max_area2 = CCTNum(0.0);
	CCTNum_t max_dist = CCTNum(0.0);

	i_arr[0] = 0;
	i_arr[1] = 0;
	i_arr[2] = 0;
	i_arr[3] = 0;

	for (i = 1; i < v_cnt; ++i) {
		CCTNum_t d2 = mathVec3DistanceSq(v[i_arr[0]], v[i]);
		if (d2 > maxd2) {
			maxd2 = d2;
			i_arr[1] = i;
		}
	}
	if (maxd2 <= CCT_EPSILON) {
		return 0;
	}

	for (i = 1; i < v_cnt; ++i) {
		CCTNum_t ab[3], ap[3], n[3];
		CCTNum_t area2;
		if (i == i_arr[1]) {
			continue;
		}
		mathVec3Sub(ab, v[i_arr[1]], v[i_arr[0]]);
		mathVec3Sub(ap, v[i], v[i_arr[0]]);
		mathVec3Cross(n, ab, ap);
		area2 = mathVec3LenSq(n);
		if (area2 > max_area2) {
			max_area2 = area2;
			i_arr[2] = i;
		}
	}
	if (max_area2 <= CCT_EPSILON) {
		return 0;
	}

	for (i = 1; i < v_cnt; ++i) {
		CCTNum_t d;
		if (i == i_arr[1] || i == i_arr[2]) {
			continue;
		}
		d = plane_signed_distance_unnorm(v[i_arr[0]], v[i_arr[1]], v[i_arr[2]], v[i]);
		d = CCTNum_abs(d);
		if (d > max_dist) {
			max_dist = d;
			i_arr[3] = i;
		}
	}
	if (max_dist <= CCT_EPSILON) {
		*coplanar_non_degenerate = 1;
	}
	else {
		*coplanar_non_degenerate = 0;
	}

	return 1;
}

#ifdef __cplusplus
extern "C" {
#endif

int mathConvexHullBuild(const CCTNum_t (*v)[3], unsigned int v_cnt, unsigned int **ret_tri_v_indices, unsigned int *ret_tri_v_indices_cnt) {
	unsigned int i_arr[4];
	int coplanar_non_degenerate;

	*ret_tri_v_indices_cnt = 0;
	*ret_tri_v_indices = NULL;

	if (v_cnt < 3) {
		return 0;
	}

	if (v_cnt == 3) {
		unsigned int* faces;
		CCTNum_t v0v1[3], v0v2[3];
		mathVec3Sub(v0v1, v[1], v[0]);
		mathVec3Sub(v0v2, v[2], v[0]);
		if (mathVec3IsParallel(v0v1, v0v2)) {
			return 0;
		}
		faces = (unsigned int*)malloc(sizeof(unsigned int) * 3);
		if (!faces) {
			return 0;
		}
		faces[0] = 0;
		faces[1] = 1;
		faces[2] = 2;
		*ret_tri_v_indices = faces;
		*ret_tri_v_indices_cnt = 3;
		return 1;
	}

	if (!init_tetrahedron_indices(v, v_cnt, i_arr, &coplanar_non_degenerate)) {
		return 0;
	}

	if (coplanar_non_degenerate) {
		return build_convex_hull_2d(v, v_cnt, i_arr, ret_tri_v_indices, ret_tri_v_indices_cnt);
	}

	return build_convex_hull_3d(v, v_cnt, i_arr, ret_tri_v_indices, ret_tri_v_indices_cnt);
}

#ifdef __cplusplus
}
#endif
