//
// Created by hujianzhe
//

#include "../inc/capsule.h"
#include "../inc/math_vec3.h"
#include "../inc/vertex.h"

#ifdef __cplusplus
extern "C" {
#endif

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

#ifdef __cplusplus
}
#endif