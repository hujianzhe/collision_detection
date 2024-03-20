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

int mathCircleNormalComputeHorizonAndTilt(const CCTNum_t circle_normal[3], const CCTNum_t reference_V[3], CCTNum_t horizon_dir[3], CCTNum_t tilt_dir[3]) {
	mathVec3Cross(horizon_dir, circle_normal, reference_V);
	if (mathVec3IsZero(horizon_dir)) {
		mathVec3Set(horizon_dir, CCTNums_3(0.0, 0.0, 0.0));
		mathVec3Set(tilt_dir, CCTNums_3(0.0, 0.0, 0.0));
		return 0;
	}
	mathVec3Normalized(horizon_dir, horizon_dir);
	mathVec3Cross(tilt_dir, horizon_dir, circle_normal);
	return 1;
}

#ifdef __cplusplus
}
#endif
