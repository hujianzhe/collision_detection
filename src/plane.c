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

CCTNum_t mathPointProjectionPlanePoint(const CCTNum_t p[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTNum_t np[3]) {
	CCTNum_t pv[3], d;
	mathVec3Sub(pv, plane_v, p);
	d = mathVec3Dot(pv, plane_n);
	mathVec3Copy(np, p);
	mathVec3AddScalar(np, plane_n, d);
	return d;
}

CCTNum_t mathPlaneNormalByVertices3(const CCTNum_t v0[3], const CCTNum_t v1[3], const CCTNum_t v2[3], CCTNum_t normal[3]) {
	CCTNum_t v0v1[3], v0v2[3];
	mathVec3Sub(v0v1, v1, v0);
	mathVec3Sub(v0v2, v2, v0);
	mathVec3Cross(normal, v0v1, v0v2);
	return mathVec3Normalized(normal, normal);
}

int mathPlaneEqual(const CCTNum_t v1[3], const CCTNum_t n1[3], const CCTNum_t v2[3], const CCTNum_t n2[3]) {
	CCTNum_t v[3], dot;
	mathVec3Sub(v, v2, v1);
	dot = mathVec3Dot(n1, v);
	if (dot < CCT_EPSILON_NEGATE || dot > CCT_EPSILON) {
		return 0;
	}
	mathVec3Cross(v, n1, n2);
	return mathVec3IsZero(v);
}

#ifdef __cplusplus
}
#endif
