//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/plane.h"

#ifdef __cplusplus
extern "C" {
#endif

void mathPointProjectionPlane(const CCTNum_t p[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTNum_t np[3], CCTNum_t* distance) {
	CCTNum_t pv[3], d;
	mathVec3Sub(pv, plane_v, p);
	d = mathVec3Dot(pv, plane_n);
	if (distance) {
		*distance = d;
	}
	if (np) {
		mathVec3Copy(np, p);
		mathVec3AddScalar(np, plane_n, d);
	}
}

CCTNum_t mathPlaneNormalByVertices3(const CCTNum_t v0[3], const CCTNum_t v1[3], const CCTNum_t v2[3], CCTNum_t normal[3]) {
	CCTNum_t v0v1[3], v0v2[3];
	mathVec3Sub(v0v1, v1, v0);
	mathVec3Sub(v0v2, v2, v0);
	mathVec3Cross(normal, v0v1, v0v2);
	return mathVec3Normalized(normal, normal);
}

int Plane_Contain_Point(const CCTNum_t plane_v[3], const CCTNum_t plane_normal[3], const CCTNum_t p[3]) {
	CCTNum_t v[3], dot;
	mathVec3Sub(v, plane_v, p);
	dot = mathVec3Dot(plane_normal, v);
	return CCT_EPSILON_NEGATE <= dot && dot <= CCT_EPSILON;
}

int Plane_Intersect_Plane(const CCTNum_t v1[3], const CCTNum_t n1[3], const CCTNum_t v2[3], const CCTNum_t n2[3]) {
	CCTNum_t n[3];
	mathVec3Cross(n, n1, n2);
	if (!mathVec3IsZero(n)) {
		return 1;
	}
	return Plane_Contain_Point(v1, n1, v2) ? 2 : 0;
}

#ifdef __cplusplus
}
#endif
