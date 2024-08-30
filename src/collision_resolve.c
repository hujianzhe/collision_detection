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
	mathVec3Set(rigid->mass_center, CCTNums_3(0.0, 0.0, 0.0));
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

static CCTNum_t* box_inertia_tensor(const CCTNum_t half[3], CCTNum_t inertia[3]) {
	const CCTNum_t fraction = CCTNum(1.0) / CCTNum(3.0);
	const CCTNum_t size_sq[3] = {
		CCTNum_sq(half[0]),
		CCTNum_sq(half[1]),
		CCTNum_sq(half[2])
	};
	inertia[0] = (size_sq[1] + size_sq[2]) * fraction;
	if (!CCTNum_chkval(inertia[0])) {
		return NULL;
	}
	inertia[1] = (size_sq[0] + size_sq[2]) * fraction;
	if (!CCTNum_chkval(inertia[1])) {
		return NULL;
	}
	inertia[2] = (size_sq[0] + size_sq[1]) * fraction;
	if (!CCTNum_chkval(inertia[2])) {
		return NULL;
	}
	return inertia;
}

CCTNum_t* physCollisionInertiaTensor(const void* geo_data, int geo_type, CCTNum_t inertia[3]) {
	switch (geo_type) {
		case GEOMETRY_BODY_OBB:
		{
			const GeometryOBB_t* obb = (const GeometryOBB_t*)geo_data;
			return box_inertia_tensor(obb->half, inertia);
		}
		case GEOMETRY_BODY_SPHERE:
		{
			const GeometrySphere_t* sphere = (const GeometrySphere_t*)geo_data;
			CCTNum_t e = CCTNum_sq(sphere->radius) * CCTNum(0.4);
			if (!CCTNum_chkval(e)) {
				return NULL;
			}
			inertia[0] = inertia[1] = inertia[2] = e;
			return inertia;
		}
		case GEOMETRY_BODY_CAPSULE:
		{
			const GeometryCapsule_t* capsule = (const GeometryCapsule_t*)geo_data;
			/* this code is copy from PhysX 4.1 */
			CCTNum_t r = capsule->radius, h = capsule->half;
			CCTNum_t r_sq = CCTNum_sq(r), h_sq = CCTNum_sq(h);
			CCTNum_t b = r_sq * r * (CCTNum(8.0) / CCTNum(15.0)) + h * r_sq;
			CCTNum_t a = b * CCTNum(1.5) + h_sq * r * (CCTNum(4.0) / CCTNum(3.0)) + h_sq * h * (CCTNum(2.0) / CCTNum(3.0));
			mathVec3Set(inertia, b, a, a);
			mathVec3MultiplyScalar(inertia, inertia, CCT_PI * r_sq);
			if (!CCTNum_chkvals(inertia, 3)) {
				return NULL;
			}
			return inertia;
		}
		case GEOMETRY_BODY_CONVEX_MESH:
		{
			const GeometryMesh_t* mesh = (const GeometryMesh_t*)geo_data;
			return box_inertia_tensor(mesh->bound_box.half, inertia);
		}
	}
	return NULL;
}

#ifdef	__cplusplus
}
#endif
