//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/sphere.h"

#ifdef __cplusplus
extern "C" {
#endif

int Sphere_Contain_Point(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t p[3]) {
	CCTNum_t op[3], op_lensq, radius_sq = radius * radius;
	mathVec3Sub(op, p, o);
	op_lensq = mathVec3LenSq(op);
	if (op_lensq > radius_sq + CCT_EPSILON) {
		return 0;
	}
	else if (op_lensq < radius_sq - CCT_EPSILON) {
		return 2;
	}
	return 1;
}

int Sphere_Intersect_Sphere(const CCTNum_t o1[3], CCTNum_t r1, const CCTNum_t o2[3], CCTNum_t r2, CCTNum_t p[3]) {
	CCTNum_t o1o2[3];
	CCTNum_t o1o2_lensq, radius_sum_sq = (r1 + r2) * (r1 + r2);
	mathVec3Sub(o1o2, o2, o1);
	o1o2_lensq = mathVec3LenSq(o1o2);
	if (o1o2_lensq > radius_sum_sq + CCT_EPSILON) {
		return 0;
	}
	else if (o1o2_lensq < radius_sum_sq - CCT_EPSILON) {
		return 2;
	}
	if (p) {
		mathVec3Normalized(o1o2, o1o2);
		mathVec3AddScalar(mathVec3Copy(p, o1), o1o2, r1);
	}
	return 1;
}

int Sphere_Contain_Sphere(const CCTNum_t o1[3], CCTNum_t r1, const CCTNum_t o2[3], CCTNum_t r2) {
	CCTNum_t o1o2[3], len_sq;
	if (r1 < r2) {
		return 0;
	}
	mathVec3Sub(o1o2, o2, o1);
	len_sq = mathVec3LenSq(o1o2);
	return len_sq <= (r1 - r2) * (r1 - r2);
}

#ifdef __cplusplus
}
#endif
