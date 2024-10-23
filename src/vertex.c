//
// Created by hujianzhe
//

#include "../inc/vertex.h"
#include "../inc/math_vec3.h"

#ifdef	__cplusplus
extern "C" {
#endif

unsigned int mathVerticesMerge(const CCTNum_t(*src_v)[3], unsigned int v_cnt, const unsigned int* src_indices, unsigned int indices_cnt, CCTNum_t(*dst_v)[3], unsigned int* dst_indices) {
	/* allow src_v == dst_v */
	/* allow src_indices == dst_indices */
	unsigned int i, dst_v_cnt = 0;
	if (src_indices != dst_indices) {
		for (i = 0; i < indices_cnt; ++i) {
			dst_indices[i] = src_indices[i];
		}
	}
	for (i = 0; i < v_cnt; ++i) {
		unsigned int j, k;
		for (j = 0; j < dst_v_cnt; ++j) {
			if (mathVec3Equal(src_v[i], dst_v[j])) {
				break;
			}
		}
		if (j >= dst_v_cnt) {
			for (k = 0; k < indices_cnt; ++k) {
				if (src_indices[k] == i) {
					dst_indices[k] = dst_v_cnt;
				}
			}
			mathVec3Copy(dst_v[dst_v_cnt++], src_v[i]);
			continue;
		}
		for (k = 0; k < indices_cnt; ++k) {
			if (src_indices[k] == i) {
				dst_indices[k] = j;
			}
		}
	}
	return dst_v_cnt;
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

unsigned int mathFindEdgeIndexByTwoVertexIndex(const unsigned int* edge_indices, unsigned int edge_indices_cnt, unsigned short edge_stride, unsigned int v_idx0, unsigned int v_idx1) {
	unsigned int i;
	if (edge_stride != 1) {
		for (i = 0; i < edge_indices_cnt; ++i) {
			unsigned int idx = edge_indices[i++];
			if (v_idx0 == idx) {
				if (v_idx1 == edge_indices[i]) {
					return i >> 1;
				}
				continue;
			}
			if (v_idx1 == idx) {
				if (v_idx0 == edge_indices[i]) {
					return i >> 1;
				}
				continue;
			}
		}
	}
	else {
		for (i = 1; i < edge_indices_cnt; ++i) {
			if (v_idx0 == edge_indices[i]) {
				if (v_idx1 == edge_indices[i - 1]) {
					return i - 1;
				}
				continue;
			}
			if (v_idx1 == edge_indices[i]) {
				if (v_idx0 == edge_indices[i - 1]) {
					return i - 1;
				}
				continue;
			}
		}
		if (v_idx0 == edge_indices[0]) {
			if (v_idx1 == edge_indices[--i]) {
				return i;
			}
		}
		if (v_idx1 == edge_indices[0]) {
			if (v_idx0 == edge_indices[--i]) {
				return i;
			}
		}
	}
	return -1;
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

#ifdef	__cplusplus
}
#endif
