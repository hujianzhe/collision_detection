//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/math_matrix3.h"
#include "../inc/collision.h"
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

CCTRigidBody_t* physRigidBodyInit(CCTRigidBody_t* rigid) {
	mathVec3Set(rigid->velocity, CCTNums_3(0.0, 0.0, 0.0));
	rigid->mass = CCTNum(0.0);
	rigid->cor = CCTNum(1.0);
	return rigid;
}

void physCollisionResolveVelocity(const CCTRigidBody_t* src_rd, const CCTRigidBody_t* target_rd, const CCTNum_t hit_normal[3], CCTNum_t src_v[3], CCTNum_t target_v[3]) {
	CCTNum_t dot, total_mass = src_rd->mass + target_rd->mass;
	CCTNum_t min_cor = (src_rd->cor < target_rd->cor ? src_rd->cor : target_rd->cor);
	if (mathVec3IsZero(target_rd->velocity)) {
		mathVec3Reflect(src_v, src_rd->velocity, hit_normal);
		mathVec3Copy(target_v, hit_normal);
		dot = mathVec3Dot(src_v, target_v);
		if (dot > CCTNum(0.0)) {
			mathVec3Negate(target_v, target_v);
		}
		else {
			dot = CCTNum_abs(dot);
		}
		if (total_mass > CCT_EPSILON) {
			CCTNum_t impulse = CCTNum(2.0) * dot * min_cor / total_mass;
			CCTNum_t target_impluse = impulse;
			if (src_rd->mass > CCT_EPSILON) {
				target_impluse *= src_rd->mass;
			}
			mathVec3MultiplyScalar(target_v, target_v, target_impluse);
		}
		else {
			mathVec3MultiplyScalar(target_v, target_v, CCTNum(2.0) * dot * min_cor);
		}
		mathVec3MultiplyScalar(src_v, src_v, min_cor);
	}
	else {
		CCTNum_t delta_v[3];
		mathVec3Sub(delta_v, src_rd->velocity, target_rd->velocity);
		dot = mathVec3Dot(delta_v, hit_normal);
		dot *= min_cor;
		if (total_mass > CCT_EPSILON) {
			CCTNum_t impulse = CCTNum(2.0) * dot / total_mass;
			CCTNum_t src_impluse = impulse, target_impluse = impulse;
			if (target_rd->mass > CCT_EPSILON) {
				src_impluse *= target_rd->mass;
			}
			mathVec3MultiplyScalar(delta_v, hit_normal, src_impluse);
			mathVec3Sub(src_v, src_rd->velocity, delta_v);
			if (src_rd->mass > CCT_EPSILON) {
				target_impluse *= src_rd->mass;
			}
			mathVec3MultiplyScalar(delta_v, hit_normal, target_impluse);
			mathVec3Add(target_v, target_rd->velocity, delta_v);
		}
		else {
			mathVec3MultiplyScalar(delta_v, hit_normal, dot);
			mathVec3Sub(src_v, src_rd->velocity, delta_v);
			mathVec3Add(target_v, target_rd->velocity, delta_v);
		}
	}
}

CCTNum_t* physCollisionInertiaTensor(CCTNum_t mass, const unsigned char* geo_data, int geo_type, CCTNum_t mat44[16]) {
	memset(mat44, 0, sizeof(CCTNum_t[16]));
	if (mass < CCT_EPSILON) {
		return mat44;
	}
	switch (geo_type) {
		case GEOMETRY_BODY_AABB:
		{
			int i;
			const GeometryAABB_t* aabb = (const GeometryAABB_t*)geo_data;
			CCTNum_t size_sq[3], fraction = mass / CCTNum(12.0);
			for (i = 0; i < 3; ++i) {
				size_sq[i] = aabb->half[i] + aabb->half[i];
				size_sq[i] *= size_sq[i];
			}
			*mathMat44Element(mat44, 0, 0) = (size_sq[1] + size_sq[2]) * fraction;
			*mathMat44Element(mat44, 1, 1) = (size_sq[0] + size_sq[2]) * fraction;
			*mathMat44Element(mat44, 2, 2) = (size_sq[0] + size_sq[1]) * fraction;
			*mathMat44Element(mat44, 3, 3) = CCTNum(1.0);
			break;
		}
		case GEOMETRY_BODY_OBB:
		{
			int i;
			const GeometryOBB_t* obb = (const GeometryOBB_t*)geo_data;
			CCTNum_t size_sq[3], fraction = mass / CCTNum(12.0);
			for (i = 0; i < 3; ++i) {
				size_sq[i] = obb->half[i] + obb->half[i];
				size_sq[i] *= size_sq[i];
			}
			*mathMat44Element(mat44, 0, 0) = (size_sq[1] + size_sq[2]) * fraction;
			*mathMat44Element(mat44, 1, 1) = (size_sq[0] + size_sq[2]) * fraction;
			*mathMat44Element(mat44, 2, 2) = (size_sq[0] + size_sq[1]) * fraction;
			*mathMat44Element(mat44, 3, 3) = CCTNum(1.0);
			break;
		}
		case GEOMETRY_BODY_SPHERE:
		{
			const GeometrySphere_t* sphere = (const GeometrySphere_t*)geo_data;
			CCTNum_t e = sphere->radius * sphere->radius * mass * CCTNum(0.4);
			*mathMat44Element(mat44, 0, 0) = e;
			*mathMat44Element(mat44, 1, 1) = e;
			*mathMat44Element(mat44, 2, 2) = e;
			*mathMat44Element(mat44, 3, 3) = CCTNum(1.0);
			break;
		}
		default:
			return NULL;
	}
	return mat44;
}

#ifdef	__cplusplus
}
#endif
