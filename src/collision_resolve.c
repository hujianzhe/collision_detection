//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/collision.h"

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
		dot *= min_cor;
		if (total_mass > CCT_EPSILON) {
			CCTNum_t impulse = CCTNum(2.0) * dot / total_mass;
			CCTNum_t target_impluse = impulse;
			if (src_rd->mass > CCT_EPSILON) {
				target_impluse *= src_rd->mass;
			}
			mathVec3MultiplyScalar(target_v, target_v, target_impluse);
		}
		else {
			mathVec3MultiplyScalar(target_v, target_v, dot);
		}
		mathVec3MultiplyScalar(src_v, src_v, dot);
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

#ifdef	__cplusplus
}
#endif
