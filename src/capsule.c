//
// Created by hujianzhe
//

#include "../inc/capsule.h"
#include "../inc/math_vec3.h"
#include "../inc/vertex.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const CCTNum_t AABB_Axis[3][3];

void mathCapsuleFindMaxMinXYZ(const GeometryCapsule_t* capsule, CCTNum_t v_minXYZ[3], CCTNum_t v_maxXYZ[3]) {
	int i;
	CCTNum_t axis_edge[2][3];
	mathTwoVertexFromCenterHalf(capsule->o, capsule->axis, capsule->half, axis_edge[0], axis_edge[1]);
	for (i = 0; i < 3; ++i) {
		if (axis_edge[0][i] < axis_edge[1][i]) {
			v_minXYZ[i] = axis_edge[0][i];
			v_maxXYZ[i] = axis_edge[1][i];
		}
		else {
			v_minXYZ[i] = axis_edge[1][i];
			v_maxXYZ[i] = axis_edge[0][i];
		}
		v_minXYZ[i] -= capsule->radius;
		v_maxXYZ[i] += capsule->radius;
	}
}

void mathCapsuleComputeExtendOBB(const GeometryCapsule_t* capsule, const CCTNum_t radius_axis1[3], GeometryOBB_t* obb) {
	mathVec3Copy(obb->o, capsule->o);
	obb->half[0] = capsule->half + capsule->radius;
	obb->half[1] = capsule->radius;
	obb->half[2] = capsule->radius;
	if (radius_axis1) {
		mathVec3Copy(obb->axis[1], radius_axis1);
	}
	else {
		mathVec3Cross(obb->axis[1], capsule->axis, AABB_Axis[0]);
		if (mathVec3IsZero(obb->axis[1])) {
			mathVec3Cross(obb->axis[1], capsule->axis, AABB_Axis[1]);
		}
		mathVec3Normalized(obb->axis[1], obb->axis[1]);
	}
	mathVec3Copy(obb->axis[0], capsule->axis);
	mathVec3Cross(obb->axis[2], obb->axis[0], obb->axis[1]);
}

#ifdef __cplusplus
}
#endif