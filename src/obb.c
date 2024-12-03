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
	CCTNum_t AX[3][3];
	mathVec3MultiplyScalar(AX[0], obb->axis[0], obb->half[0]);
	mathVec3MultiplyScalar(AX[1], obb->axis[1], obb->half[1]);
	mathVec3MultiplyScalar(AX[2], obb->axis[2], obb->half[2]);

	mathVec3Copy(v[0], obb->o);
	mathVec3Sub(v[0], v[0], AX[0]);
	mathVec3Sub(v[0], v[0], AX[1]);
	mathVec3Sub(v[0], v[0], AX[2]);

	mathVec3Copy(v[1], obb->o);
	mathVec3Add(v[1], v[1], AX[0]);
	mathVec3Sub(v[1], v[1], AX[1]);
	mathVec3Sub(v[1], v[1], AX[2]);

	mathVec3Copy(v[2], obb->o);
	mathVec3Add(v[2], v[2], AX[0]);
	mathVec3Add(v[2], v[2], AX[1]);
	mathVec3Sub(v[2], v[2], AX[2]);

	mathVec3Copy(v[3], obb->o);
	mathVec3Sub(v[3], v[3], AX[0]);
	mathVec3Add(v[3], v[3], AX[1]);
	mathVec3Sub(v[3], v[3], AX[2]);

	mathVec3Copy(v[4], obb->o);
	mathVec3Sub(v[4], v[4], AX[0]);
	mathVec3Sub(v[4], v[4], AX[1]);
	mathVec3Add(v[4], v[4], AX[2]);

	mathVec3Copy(v[5], obb->o);
	mathVec3Add(v[5], v[5], AX[0]);
	mathVec3Sub(v[5], v[5], AX[1]);
	mathVec3Add(v[5], v[5], AX[2]);

	mathVec3Copy(v[6], obb->o);
	mathVec3Add(v[6], v[6], AX[0]);
	mathVec3Add(v[6], v[6], AX[1]);
	mathVec3Add(v[6], v[6], AX[2]);

	mathVec3Copy(v[7], obb->o);
	mathVec3Sub(v[7], v[7], AX[0]);
	mathVec3Add(v[7], v[7], AX[1]);
	mathVec3Add(v[7], v[7], AX[2]);
}

CCTNum_t* mathOBBVertex(const GeometryOBB_t* obb, unsigned int idx, CCTNum_t v[3]) {
	CCTNum_t AX[3][3];
	if (idx >= 8) {
		return NULL;
	}
	mathVec3Copy(v, obb->o);
	mathVec3MultiplyScalar(AX[0], obb->axis[0], obb->half[0]);
	mathVec3MultiplyScalar(AX[1], obb->axis[1], obb->half[1]);
	mathVec3MultiplyScalar(AX[2], obb->axis[2], obb->half[2]);
	switch (idx) {
		case 0:
			mathVec3Sub(v, v, AX[0]);
			mathVec3Sub(v, v, AX[1]);
			mathVec3Sub(v, v, AX[2]);
			return v;
		case 1:
			mathVec3Add(v, v, AX[0]);
			mathVec3Sub(v, v, AX[1]);
			mathVec3Sub(v, v, AX[2]);
			return v;
		case 2:
			mathVec3Add(v, v, AX[0]);
			mathVec3Add(v, v, AX[1]);
			mathVec3Sub(v, v, AX[2]);
			return v;
		case 3:
			mathVec3Sub(v, v, AX[0]);
			mathVec3Add(v, v, AX[1]);
			mathVec3Sub(v, v, AX[2]);
			return v;
		case 4:
			mathVec3Sub(v, v, AX[0]);
			mathVec3Sub(v, v, AX[1]);
			mathVec3Add(v, v, AX[2]);
			return v;
		case 5:
			mathVec3Add(v, v, AX[0]);
			mathVec3Sub(v, v, AX[1]);
			mathVec3Add(v, v, AX[2]);
			return v;
		case 6:
			mathVec3Add(v, v, AX[0]);
			mathVec3Add(v, v, AX[1]);
			mathVec3Add(v, v, AX[2]);
			return v;
		case 7:
			mathVec3Sub(v, v, AX[0]);
			mathVec3Add(v, v, AX[1]);
			mathVec3Add(v, v, AX[2]);
			return v;
	}
	return NULL;
}

#ifdef __cplusplus
}
#endif
