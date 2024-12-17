//
// Created by hujianzhe
//

#include "../inc/vertex.h"
#include "../inc/math_vec3.h"

#ifdef	__cplusplus
extern "C" {
#endif

unsigned int mathVerticesMerge(const CCTNum_t(*src_v)[3], const unsigned int* src_indices, unsigned int indices_cnt, CCTNum_t(*dst_v)[3], unsigned int* dst_indices) {
	unsigned int i, dst_v_cnt = 0;
	for (i = 0; i < indices_cnt; ++i) {
		unsigned int j;
		const CCTNum_t* p = src_v[src_indices[i]];
		for (j = 0; j < dst_v_cnt; ++j) {
			if (mathVec3Equal(dst_v[j], p)) {
				break;
			}
		}
		dst_indices[i] = j;
		if (j >= dst_v_cnt) {
			mathVec3Copy(dst_v[dst_v_cnt++], p);
		}
	}
	return dst_v_cnt;
}

int mathEdgeVertexIndicesMergeEdgeVertexIndices(const CCTNum_t(*v)[3], const unsigned int a_edge_v_indices[2], const unsigned int b_edge_v_indices[2], unsigned int final_edge_v_indices[2]) {
	CCTNum_t v1[3], v2[3], N[3];
	unsigned int i, same_cnt;
	unsigned int a_same_idx, b_same_idx;
	mathVec3Sub(v1, v[a_edge_v_indices[1]], v[a_edge_v_indices[0]]);
	mathVec3Sub(v2, v[b_edge_v_indices[1]], v[b_edge_v_indices[0]]);
	mathVec3Cross(N, v1, v2);
	if (!mathVec3IsZero(N)) {
		/* not parallel */
		return 0;
	}
	mathVec3Sub(v2, v[b_edge_v_indices[0]], v[a_edge_v_indices[0]]);
	mathVec3Cross(N, v1, v2);
	if (!mathVec3IsZero(N)) {
		/* not collinear */
		return 0;
	}
	same_cnt = 0;
	for (i = 0; i < 2; ++i) {
		unsigned int j;
		for (j = 0; j < 2; ++j) {
			if (mathVec3Equal(v[a_edge_v_indices[i]], v[b_edge_v_indices[j]])) {
				++same_cnt;
				a_same_idx = i;
				b_same_idx = j;
			}
		}
	}
	if (same_cnt < 1) {
		const unsigned int* edge_v_indices_pp[2] = { a_edge_v_indices, b_edge_v_indices };
		/* test overlap, a overlap b and b overlap a */
		for (i = 0; i < 2; ++i) {
			int overlap_test[2];
			unsigned int j;
			for (j = 0; j < 2; ++j) {
				mathVec3Sub(v1, v[edge_v_indices_pp[i][0]], v[edge_v_indices_pp[!i][j]]);
				mathVec3Sub(v2, v[edge_v_indices_pp[i][1]], v[edge_v_indices_pp[!i][j]]);
				overlap_test[j] = (mathVec3Dot(v1, v2) < CCTNum(0.0));
			}
			if (overlap_test[0] && overlap_test[1]) {
				final_edge_v_indices[0] = edge_v_indices_pp[i][0];
				final_edge_v_indices[1] = edge_v_indices_pp[i][1];
				return 1;
			}
			if (overlap_test[0] || overlap_test[1]) {
				for (j = 0; j < 2; ++j) {
					mathVec3Sub(v1, v[edge_v_indices_pp[!i][0]], v[edge_v_indices_pp[i][j]]);
					mathVec3Sub(v2, v[edge_v_indices_pp[!i][1]], v[edge_v_indices_pp[i][j]]);
					if (mathVec3Dot(v1, v2) < CCTNum(0.0)) {
						final_edge_v_indices[0] = edge_v_indices_pp[i][!j];
						final_edge_v_indices[1] = edge_v_indices_pp[!i][overlap_test[1] ? 0 : 1];
						return 1;
					}
				}
			}
		}
		return 0;
	}
	if (same_cnt >= 2) {
		/* same edge */
		final_edge_v_indices[0] = a_edge_v_indices[0];
		final_edge_v_indices[1] = a_edge_v_indices[1];
		return 1;
	}
	/* only one pair indices same */
	mathVec3Sub(v1, v[a_edge_v_indices[!a_same_idx]], v[a_edge_v_indices[a_same_idx]]);
	mathVec3Sub(v2, v[b_edge_v_indices[!b_same_idx]], v[b_edge_v_indices[b_same_idx]]);
	if (mathVec3Dot(v1, v2) < CCTNum(0.0)) {
		final_edge_v_indices[0] = a_edge_v_indices[!a_same_idx];
		final_edge_v_indices[1] = b_edge_v_indices[!b_same_idx];
	}
	else if (mathVec3LenSq(v1) > mathVec3LenSq(v2)) {
		final_edge_v_indices[0] = a_edge_v_indices[0];
		final_edge_v_indices[1] = a_edge_v_indices[1];
	}
	else {
		final_edge_v_indices[0] = b_edge_v_indices[0];
		final_edge_v_indices[1] = b_edge_v_indices[1];
	}
	return 1;
}

int mathVertexIndicesFindMinMaxXYZ(const CCTNum_t(*v)[3], const unsigned int* v_indices, unsigned int v_indices_cnt, CCTNum_t v_minXYZ[3], CCTNum_t v_maxXYZ[3]) {
	unsigned int i;
	if (v_indices_cnt <= 0) {
		return 0;
	}
	mathVec3Copy(v_minXYZ, v[v_indices[0]]);
	mathVec3Copy(v_maxXYZ, v[v_indices[0]]);
	for (i = 1; i < v_indices_cnt; ++i) {
		unsigned int j;
		const CCTNum_t* cur_v = v[v_indices[i]];
		for (j = 0; j < 3; ++j) {
			if (cur_v[j] < v_minXYZ[j]) {
				v_minXYZ[j] = cur_v[j];
			}
			else if (cur_v[j] > v_maxXYZ[j]) {
				v_maxXYZ[j] = cur_v[j];
			}
		}
	}
	return 1;
}

int mathVerticesFindMinMaxXYZ(const CCTNum_t(*v)[3], unsigned int v_cnt, CCTNum_t v_minXYZ[3], CCTNum_t v_maxXYZ[3]) {
	unsigned int i;
	if (v_cnt <= 0) {
		return 0;
	}
	mathVec3Copy(v_minXYZ, v[0]);
	mathVec3Copy(v_maxXYZ, v[0]);
	for (i = 1; i < v_cnt; ++i) {
		unsigned int j;
		const CCTNum_t* cur_v = v[i];
		for (j = 0; j < 3; ++j) {
			if (cur_v[j] < v_minXYZ[j]) {
				v_minXYZ[j] = cur_v[j];
			}
			else if (cur_v[j] > v_maxXYZ[j]) {
				v_maxXYZ[j] = cur_v[j];
			}
		}
	}
	return 1;
}

int mathVertexIndicesAverageXYZ(const CCTNum_t(*v)[3], const unsigned int* v_indices, unsigned int v_indices_cnt, CCTNum_t v_avgXYZ[3]) {
	unsigned int i;
	if (v_indices_cnt <= 0) {
		return 0;
	}
	v_avgXYZ[0] = v_avgXYZ[1] = v_avgXYZ[2] = CCTNum(0.0);
	for (i = 0; i < v_indices_cnt; ++i) {
		const CCTNum_t* p = v[v_indices[i]];
		v_avgXYZ[0] += p[0];
		v_avgXYZ[1] += p[1];
		v_avgXYZ[2] += p[2];
	}
	if (v_indices_cnt > 1) {
		v_avgXYZ[0] /= v_indices_cnt;
		v_avgXYZ[1] /= v_indices_cnt;
		v_avgXYZ[2] /= v_indices_cnt;
	}
	return 1;
}

void mathTwoVertexFromCenterHalf(const CCTNum_t center_p[3], const CCTNum_t dir[3], CCTNum_t half_len, CCTNum_t start_v[3], CCTNum_t end_v[3]) {
	mathVec3Copy(start_v, center_p);
	mathVec3SubScalar(start_v, dir, half_len);
	mathVec3Copy(end_v, center_p);
	mathVec3AddScalar(end_v, dir, half_len);
}

void mathTwoVertexToCenterHalf(const CCTNum_t start_v[3], const CCTNum_t end_v[3], CCTNum_t center_p[3], CCTNum_t dir[3], CCTNum_t* half) {
	mathVec3Sub(dir, end_v, start_v);
	*half = mathVec3Normalized(dir, dir) * CCTNum(0.5);
	mathVec3Copy(center_p, start_v);
	mathVec3AddScalar(center_p, dir, *half);
}

unsigned int mathFindVertexIndex(const CCTNum_t(*v)[3], const unsigned int* v_indices, unsigned int v_indices_cnt, const CCTNum_t p[3]) {
	unsigned int i;
	for (i = 0; i < v_indices_cnt; ++i) {
		unsigned int v_idx = v_indices[i];
		if (mathVec3Equal(v[v_idx], p)) {
			return v_idx;
		}
	}
	return -1;
}

unsigned int mathFindEdgeIndexByVertexIndices(const unsigned int* edge_v_indices, unsigned int edge_v_indices_cnt, unsigned int v_idx0, unsigned int v_idx1) {
	unsigned int i;
	for (i = 0; i < edge_v_indices_cnt; ++i) {
		unsigned int idx = edge_v_indices[i++];
		if (v_idx0 == idx) {
			if (v_idx1 == edge_v_indices[i]) {
				return i >> 1;
			}
			continue;
		}
		if (v_idx1 == idx) {
			if (v_idx0 == edge_v_indices[i]) {
				return i >> 1;
			}
			continue;
		}
	}
	return -1;
}

int mathFindBorderIndexByPoint(const CCTNum_t(*v)[3], const unsigned int* edge_v_indices, unsigned int edge_v_indices_cnt, const CCTNum_t p[3], GeometryBorderIndex_t* bi) {
	unsigned int i;
	for (i = 0; i < edge_v_indices_cnt; ++i) {
		CCTNum_t l[3], r[3], N[3], dot;
		unsigned int idx[2];
		idx[0] = edge_v_indices[i++];
		idx[1] = edge_v_indices[i];
		mathVec3Sub(l, v[idx[0]], p);
		mathVec3Sub(r, v[idx[1]], p);
		dot = mathVec3Dot(l, r);
		if (dot > CCT_EPSILON) {
			continue;
		}
		if (dot >= CCT_EPSILON_NEGATE) {
			if (mathVec3IsZero(l)) {
				bi->v_idx = idx[0];
				bi->edge_idx = -1;
				return 1;
			}
			if (mathVec3IsZero(r)) {
				bi->v_idx = idx[1];
				bi->edge_idx = -1;
				return 1;
			}
			continue;
		}
		mathVec3Cross(N, l, r);
		if (!mathVec3IsZero(N)) {
			continue;
		}
		bi->v_idx = -1;
		bi->edge_idx = (i >> 1);
		return 1;
	}
	bi->v_idx = -1;
	bi->edge_idx = -1;
	return 0;
}

unsigned int mathFindEdgeIndexByVertex(const CCTNum_t(*v)[3], const unsigned int* edge_v_indices, unsigned int edge_v_indices_cnt, const CCTNum_t v0[3], const CCTNum_t v1[3]) {
	unsigned int i;
	for (i = 0; i < edge_v_indices_cnt; ++i) {
		unsigned int idx = edge_v_indices[i++];
		if (mathVec3Equal(v0, v[idx])) {
			if (mathVec3Equal(v1, v[edge_v_indices[i]])) {
				return i >> 1;
			}
			continue;
		}
		if (mathVec3Equal(v1, v[idx])) {
			if (mathVec3Equal(v0, v[edge_v_indices[i]])) {
				return i >> 1;
			}
			continue;
		}
	}
	return -1;
}

unsigned int mathFindFaceIndexByVertexIndices(const GeometryPolygon_t* faces, unsigned int faces_cnt, const unsigned int* v_idx, unsigned int v_idx_cnt) {
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

unsigned int mathFindAdjacentFaceIndexByEdgeVertexIndices(const GeometryPolygon_t* faces, unsigned int faces_cnt, unsigned int edge_v_idx0, unsigned int edge_v_idx1, unsigned int* face_idx0, unsigned int* face_idx1) {
	unsigned int i;
	*face_idx0 = *face_idx1 = -1;
	for (i = 0; i < faces_cnt; ++i) {
		const GeometryPolygon_t* face = faces + i;
		if (mathFindEdgeIndexByVertexIndices(face->edge_v_indices, face->edge_v_indices_cnt, edge_v_idx0, edge_v_idx1) == -1) {
			continue;
		}
		if (-1 == *face_idx0) {
			*face_idx0 = i;
		}
		else {
			*face_idx1 = i;
			return 2;
		}
	}
	if (-1 == *face_idx0) {
		return 0;
	}
	return 1;
}

#ifdef	__cplusplus
}
#endif
