//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/aabb.h"

#ifdef __cplusplus
extern "C" {
#endif

const CCTNum_t AABB_Axis[3][3] = {
	{ CCTNums_3(1.0, 0.0, 0.0) },
	{ CCTNums_3(0.0, 1.0, 0.0) },
	{ CCTNums_3(0.0, 0.0, 1.0) }
};

const CCTNum_t AABB_Plane_Normal[6][3] = {
	{ CCTNums_3(1.0, 0.0, 0.0) }, { CCTNums_3(-1.0, 0.0, 0.0) },
	{ CCTNums_3(0.0, 1.0, 0.0) }, { CCTNums_3(0.0, -1.0, 0.0) },
	{ CCTNums_3(0.0, 0.0, 1.0) }, { CCTNums_3(0.0, 0.0, -1.0) }
};

const CCTNum_t* mathAABBPlaneNormal(unsigned int face_idx, CCTNum_t normal[3]) {
	if (face_idx >= 6) {
		return NULL;
	}
	if (normal) {
		const CCTNum_t* face_n = AABB_Plane_Normal[face_idx];
		normal[0] = face_n[0];
		normal[1] = face_n[1];
		normal[2] = face_n[2];
		return normal;
	}
	return AABB_Plane_Normal[face_idx];
}

CCTNum_t* mathAABBPlaneVertex(const CCTNum_t min_v[3], const CCTNum_t max_v[3], unsigned int face_idx, CCTNum_t v[3]) {
	if (face_idx >= 6) {
		return NULL;
	}
	if (face_idx & 0x1) {
		return mathVec3Copy(v, min_v);
	}
	else {
		return mathVec3Copy(v, max_v);
	}
	return NULL;
}

void mathAABBPlaneBoundBox(const CCTNum_t min_v[3], const CCTNum_t max_v[3], unsigned int face_idx, CCTNum_t bb_min[3], CCTNum_t bb_max[3]) {
	switch (face_idx) {
		case 0:
			bb_min[0] = max_v[0] - GEOMETRY_BODY_BOX_MIN_HALF;
			bb_min[1] = min_v[1]; bb_min[2] = min_v[2];
			bb_max[0] = max_v[0] + GEOMETRY_BODY_BOX_MIN_HALF;
			bb_max[1] = max_v[1]; bb_max[2] = max_v[2];
			return;
		case 1:
			bb_min[0] = min_v[0] - GEOMETRY_BODY_BOX_MIN_HALF;
			bb_min[1] = min_v[1]; bb_min[2] = min_v[2];
			bb_max[0] = min_v[0] + GEOMETRY_BODY_BOX_MIN_HALF;
			bb_max[1] = max_v[1]; bb_max[2] = max_v[2];
			return;
		case 2:
			bb_min[0] = min_v[0];
			bb_min[1] = max_v[1] - GEOMETRY_BODY_BOX_MIN_HALF; bb_min[2] = min_v[2];
			bb_max[0] = max_v[0];
			bb_max[1] = max_v[1] + GEOMETRY_BODY_BOX_MIN_HALF; bb_max[2] = max_v[2];
			return;
		case 3:
			bb_min[0] = min_v[0];
			bb_min[1] = min_v[1] - GEOMETRY_BODY_BOX_MIN_HALF; bb_min[2] = min_v[2];
			bb_max[0] = max_v[0];
			bb_max[1] = min_v[1] + GEOMETRY_BODY_BOX_MIN_HALF; bb_max[2] = max_v[2];
			return;
		case 4:
			bb_min[0] = min_v[0];
			bb_min[1] = min_v[1]; bb_min[2] = max_v[2] - GEOMETRY_BODY_BOX_MIN_HALF;
			bb_max[0] = max_v[0];
			bb_max[1] = max_v[1]; bb_max[2] = max_v[2] + GEOMETRY_BODY_BOX_MIN_HALF;
			return;
		case 5:
			bb_min[0] = min_v[0];
			bb_min[1] = min_v[1]; bb_min[2] = min_v[2] - GEOMETRY_BODY_BOX_MIN_HALF;
			bb_max[0] = max_v[0];
			bb_max[1] = max_v[1]; bb_max[2] = min_v[2] + GEOMETRY_BODY_BOX_MIN_HALF;
			return;
	}
}

void mathAABBVertices(const CCTNum_t min_v[3], const CCTNum_t max_v[3], CCTNum_t v[8][3]) {
	v[0][0] = min_v[0]; v[0][1] = min_v[1]; v[0][2] = min_v[2];
	v[1][0] = max_v[0]; v[1][1] = min_v[1]; v[1][2] = min_v[2];
	v[2][0] = max_v[0]; v[2][1] = max_v[1]; v[2][2] = min_v[2];
	v[3][0] = min_v[0]; v[3][1] = max_v[1]; v[3][2] = min_v[2];
	v[4][0] = min_v[0]; v[4][1] = min_v[1]; v[4][2] = max_v[2];
	v[5][0] = max_v[0]; v[5][1] = min_v[1]; v[5][2] = max_v[2];
	v[6][0] = max_v[0]; v[6][1] = max_v[1]; v[6][2] = max_v[2];
	v[7][0] = min_v[0]; v[7][1] = max_v[1]; v[7][2] = max_v[2];
}

CCTNum_t* mathAABBVertex(const CCTNum_t min_v[3], const CCTNum_t max_v[3], unsigned int v_id, CCTNum_t v[3]) {
	switch (v_id) {
		case 0:
			v[0] = min_v[0]; v[1] = min_v[1]; v[2] = min_v[2];
			return v;
		case 1:
			v[0] = max_v[0]; v[1] = min_v[1]; v[2] = min_v[2];
			return v;
		case 2:
			v[0] = max_v[0]; v[1] = max_v[1]; v[2] = min_v[2];
			return v;
		case 3:
			v[0] = min_v[0]; v[1] = max_v[1]; v[2] = min_v[2];
			return v;
		case 4:
			v[0] = min_v[0]; v[1] = min_v[1]; v[2] = max_v[2];
			return v;
		case 5:
			v[0] = max_v[0]; v[1] = min_v[1]; v[2] = max_v[2];
			return v;
		case 6:
			v[0] = max_v[0]; v[1] = max_v[1]; v[2] = max_v[2];
			return v;
		case 7:
			v[0] = min_v[0]; v[1] = max_v[1]; v[2] = max_v[2];
			return v;
	}
	return NULL;
}

void mathAABBCenterHalf(const CCTNum_t min_v[3], const CCTNum_t max_v[3], CCTNum_t center[3], CCTNum_t half[3]) {
	mathVec3Add(center, min_v, max_v);
	mathVec3MultiplyScalar(center, center, CCTNum(0.5));
	mathVec3Sub(half, max_v, min_v);
	mathVec3MultiplyScalar(half, half, CCTNum(0.5));
}

void mathAABBMergeAABB(CCTNum_t dst_min_v[3], CCTNum_t dst_max_v[3], const CCTNum_t src_min_v[3], const CCTNum_t src_max_v[3]) {
	if (dst_min_v[0] > src_min_v[0]) {
		dst_min_v[0] = src_min_v[0];
	}
	if (dst_min_v[1] > src_min_v[1]) {
		dst_min_v[1] = src_min_v[1];
	}
	if (dst_min_v[2] > src_min_v[2]) {
		dst_min_v[2] = src_min_v[2];
	}
	if (dst_max_v[0] < src_max_v[0]) {
		dst_max_v[0] = src_max_v[0];
	}
	if (dst_max_v[1] < src_max_v[1]) {
		dst_max_v[1] = src_max_v[1];
	}
	if (dst_max_v[2] < src_max_v[2]) {
		dst_max_v[2] = src_max_v[2];
	}
}

int AABB_Contain_Point(const CCTNum_t min_v[3], const CCTNum_t max_v[3], const CCTNum_t p[3]) {
	return	min_v[0] <= p[0] && p[0] <= max_v[0] &&
			min_v[1] <= p[1] && p[1] <= max_v[1] &&
			min_v[2] <= p[2] && p[2] <= max_v[2];
}

int mathAABBIntersectAABB(const CCTNum_t a_min_v[3], const CCTNum_t a_max_v[3], const CCTNum_t b_min_v[3], const CCTNum_t b_max_v[3]) {
	return	a_min_v[0] <= b_max_v[0] && a_max_v[0] >= b_min_v[0] &&
			a_min_v[1] <= b_max_v[1] && a_max_v[1] >= b_min_v[1] &&
			a_min_v[2] <= b_max_v[2] && a_max_v[2] >= b_min_v[2];
}

int AABB_Contain_AABB(const CCTNum_t a_min_v[3], const CCTNum_t a_max_v[3], const CCTNum_t b_min_v[3], const CCTNum_t b_max_v[3]) {
	return AABB_Contain_Point(a_min_v, a_max_v, b_min_v) && AABB_Contain_Point(a_min_v, a_max_v, b_max_v);
}

#ifdef __cplusplus
}
#endif
