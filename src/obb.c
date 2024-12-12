//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/obb.h"

#ifdef __cplusplus
extern "C" {
#endif

GeometryOBB_t* mathOBBFromAABB(GeometryOBB_t* obb, const CCTNum_t o[3], const CCTNum_t half[3]) {
	mathVec3Copy(obb->o, o);
	mathVec3Copy(obb->half, half);
	mathVec3Set(obb->axis[0], CCTNums_3(1.0, 0.0, 0.0));
	mathVec3Set(obb->axis[1], CCTNums_3(0.0, 1.0, 0.0));
	mathVec3Set(obb->axis[2], CCTNums_3(0.0, 0.0, 1.0));
	return obb;
}

void mathOBBToAABB(const GeometryOBB_t* obb, CCTNum_t o[3], CCTNum_t half[3]) {
	int i;
	CCTNum_t v[8][3], min_v[3], max_v[3];
	mathOBBVertices(obb, v);
	mathVec3Copy(min_v, v[0]);
	mathVec3Copy(max_v, v[0]);
	for (i = 1; i < 8; ++i) {
		int j;
		for (j = 0; j < 3; ++j) {
			if (min_v[j] > v[i][j]) {
				min_v[j] = v[i][j];
			}
			else if (max_v[j] < v[i][j]) {
				max_v[j] = v[i][j];
			}
		}
	}
	mathVec3Copy(o, obb->o);
	for (i = 0; i < 3; ++i) {
		half[i] = CCTNum(0.5) * (max_v[i] - min_v[i]);
	}
}

void mathOBBVertices(const GeometryOBB_t* obb, CCTNum_t v[8][3]) {
	mathBoxVertices(obb->o, obb->half, (const CCTNum_t(*)[3])obb->axis, v);
}

CCTNum_t* mathOBBVertex(const GeometryOBB_t* obb, unsigned int idx, CCTNum_t v[3]) {
	return mathBoxVertex(obb->o, obb->half, (const CCTNum_t(*)[3])obb->axis, idx, v);
}

#ifdef __cplusplus
}
#endif
