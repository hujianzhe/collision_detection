//
// Created by hujianzhe
//

#ifndef UTIL_C_CRT_GEOMETRY_CAPSULE_H
#define	UTIL_C_CRT_GEOMETRY_CAPSULE_H

#include "obb.h"

typedef struct GeometryCapsuleExtra_t {
	CCTNum_t axis_edge[2][3];
	CCTNum_t axis_len;
	CCTNum_t radius_sq;
} GeometryCapsuleExtra_t;

#ifdef __cplusplus
extern "C" {
#endif

__declspec_dll void mathCapsuleFindMaxMinXYZ(const GeometryCapsule_t* capsule, CCTNum_t v_minXYZ[3], CCTNum_t v_maxXYZ[3]);
__declspec_dll void mathCapsuleComputeExtendOBB(const GeometryCapsule_t* capsule, const CCTNum_t radius_axis1[3], GeometryOBB_t* obb);

#ifdef __cplusplus
}
#endif

#endif
