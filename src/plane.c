//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/plane.h"

#ifdef __cplusplus
extern "C" {
#endif

CCTNum_t mathPointProjectionPlane(const CCTNum_t p[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3]) {
	CCTNum_t pv[3];
	mathVec3Sub(pv, plane_v, p);
	return mathVec3Dot(pv, plane_n);
}

CCTNum_t mathPlaneNormalByVertices3(const CCTNum_t v0[3], const CCTNum_t v1[3], const CCTNum_t v2[3], CCTNum_t normal[3]) {
	CCTNum_t v0v1[3], v0v2[3];
	mathVec3Sub(v0v1, v1, v0);
	mathVec3Sub(v0v2, v2, v0);
	mathVec3Cross(normal, v0v1, v0v2);
	return mathVec3Normalized(normal, normal);
}

#ifdef __cplusplus
}
#endif
