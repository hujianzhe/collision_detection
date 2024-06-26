//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/aabb.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

const CCTNum_t AABB_Axis[3][3] = {
	{ CCTNums_3(1.0, 0.0, 0.0) },
	{ CCTNums_3(0.0, 1.0, 0.0) },
	{ CCTNums_3(0.0, 0.0, 1.0) }
};

const CCTNum_t AABB_Plane_Normal[6][3] = {
	{ CCTNums_3(0.0, 0.0, 1.0) }, { CCTNums_3(0.0, 0.0, -1.0) },
	{ CCTNums_3(1.0, 0.0, 0.0) }, { CCTNums_3(-1.0, 0.0, 0.0) },
	{ CCTNums_3(0.0, 1.0, 0.0) }, { CCTNums_3(0.0, -1.0, 0.0) }
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

void mathAABBPlaneVertices(const CCTNum_t o[3], const CCTNum_t half[3], CCTNum_t v[6][3]) {
	mathVec3Copy(v[0], o);
	v[0][2] += half[2];
	mathVec3Copy(v[1], o);
	v[1][2] -= half[2];

	mathVec3Copy(v[2], o);
	v[2][0] += half[0];
	mathVec3Copy(v[3], o);
	v[3][0] -= half[0];

	mathVec3Copy(v[4], o);
	v[4][1] += half[1];
	mathVec3Copy(v[5], o);
	v[5][1] -= half[1];
}

void mathAABBVertices(const CCTNum_t o[3], const CCTNum_t half[3], CCTNum_t v[8][3]) {
	v[0][0] = o[0] - half[0]; v[0][1] = o[1] - half[1]; v[0][2] = o[2] - half[2];
	v[1][0] = o[0] + half[0]; v[1][1] = o[1] - half[1]; v[1][2] = o[2] - half[2];
	v[2][0] = o[0] + half[0]; v[2][1] = o[1] + half[1]; v[2][2] = o[2] - half[2];
	v[3][0] = o[0] - half[0]; v[3][1] = o[1] + half[1]; v[3][2] = o[2] - half[2];
	v[4][0] = o[0] - half[0]; v[4][1] = o[1] - half[1]; v[4][2] = o[2] + half[2];
	v[5][0] = o[0] + half[0]; v[5][1] = o[1] - half[1]; v[5][2] = o[2] + half[2];
	v[6][0] = o[0] + half[0]; v[6][1] = o[1] + half[1]; v[6][2] = o[2] + half[2];
	v[7][0] = o[0] - half[0]; v[7][1] = o[1] + half[1]; v[7][2] = o[2] + half[2];
}

void mathAABBMinVertice(const CCTNum_t o[3], const CCTNum_t half[3], CCTNum_t v[3]) {
	v[0] = o[0] - half[0];
	v[1] = o[1] - half[1];
	v[2] = o[2] - half[2];
}

void mathAABBMaxVertice(const CCTNum_t o[3], const CCTNum_t half[3], CCTNum_t v[3]) {
	v[0] = o[0] + half[0];
	v[1] = o[1] + half[1];
	v[2] = o[2] + half[2];
}

void mathAABBFromTwoVertice(const CCTNum_t a[3], const CCTNum_t b[3], CCTNum_t o[3], CCTNum_t half[3]) {
	unsigned int i;
	for (i = 0; i < 3; ++i) {
		o[i] = (a[i] + b[i]) * CCTNum(0.5);
		if (a[i] > b[i]) {
			half[i] = (a[i] - b[i]) * CCTNum(0.5);
		}
		else {
			half[i] = (b[i] - a[i]) * CCTNum(0.5);
		}
		if (half[i] < GEOMETRY_BODY_BOX_MIN_HALF) {
			half[i] = GEOMETRY_BODY_BOX_MIN_HALF;
		}
	}
}

int AABB_Contain_Point(const CCTNum_t o[3], const CCTNum_t half[3], const CCTNum_t p[3]) {
	return p[0] >= o[0] - half[0] && p[0] <= o[0] + half[0] &&
		   p[1] >= o[1] - half[1] && p[1] <= o[1] + half[1] &&
		   p[2] >= o[2] - half[2] && p[2] <= o[2] + half[2];
}

void mathAABBClosestPointTo(const CCTNum_t o[3], const CCTNum_t half[3], const CCTNum_t p[3], CCTNum_t closest_p[3]) {
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

void mathAABBStretch(CCTNum_t o[3], CCTNum_t half[3], const CCTNum_t delta[3]) {
	unsigned int i;
	for (i = 0; i < 3; ++i) {
		CCTNum_t d = delta[i] * CCTNum(0.5);
		if (d > CCTNum(0.0)) {
			half[i] += d;
		}
		else {
			half[i] -= d;
		}
		o[i] += d;
	}
}

void mathAABBSplit(const CCTNum_t o[3], const CCTNum_t half[3], CCTNum_t new_o[8][3], CCTNum_t new_half[3]) {
	mathVec3MultiplyScalar(new_half, half, CCTNum(0.5));
	mathAABBVertices(o, new_half, new_o);
}

int AABB_Intersect_AABB(const CCTNum_t o1[3], const CCTNum_t half1[3], const CCTNum_t o2[3], const CCTNum_t half2[3]) {
	int i;
	for (i = 0; i < 3; ++i) {
		CCTNum_t half = half1[i] + half2[i];
		if (o2[i] - o1[i] > half || o1[i] - o2[i] > half) {
			return 0;
		}
	}
	return 1;
}

int AABB_Contain_AABB(const CCTNum_t o1[3], const CCTNum_t half1[3], const CCTNum_t o2[3], const CCTNum_t half2[3]) {
	CCTNum_t v[3];
	mathAABBMinVertice(o2, half2, v);
	if (!AABB_Contain_Point(o1, half1, v)) {
		return 0;
	}
	mathAABBMaxVertice(o2, half2, v);
	return AABB_Contain_Point(o1, half1, v);
}

#ifdef __cplusplus
}
#endif
