//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_AABB_H
#define	UTIL_C_CRT_GEOMETRY_AABB_H

#include "box.h"

#ifdef	__cplusplus
extern "C" {
#endif

extern const CCTNum_t AABB_Axis[3][3];
extern const CCTNum_t AABB_Plane_Normal[6][3];

__declspec_dll CCTNum_t* mathAABBPlaneVertex(const CCTNum_t o[3], const CCTNum_t half[3], unsigned int face_idx, CCTNum_t v[3]);
__declspec_dll const CCTNum_t* mathAABBPlaneNormal(unsigned int face_idx, CCTNum_t normal[3]);

__declspec_dll void mathAABBPlaneBoundBox(const CCTNum_t o[3], const CCTNum_t half[3], unsigned int face_idx, CCTNum_t bb_o[3], CCTNum_t bb_half[3]);

__declspec_dll void mathAABBVertices(const CCTNum_t o[3], const CCTNum_t half[3], CCTNum_t v[8][3]);
__declspec_dll CCTNum_t* mathAABBVertex(const CCTNum_t o[3], const CCTNum_t half[3], unsigned int v_id, CCTNum_t v[3]);
__declspec_dll void mathAABBMinVertice(const CCTNum_t o[3], const CCTNum_t half[3], CCTNum_t v[3]);
__declspec_dll void mathAABBMaxVertice(const CCTNum_t o[3], const CCTNum_t half[3], CCTNum_t v[3]);
__declspec_dll void mathAABBFromTwoVertice(const CCTNum_t a[3], const CCTNum_t b[3], CCTNum_t o[3], CCTNum_t half[3]);

__declspec_dll void mathAABBStretch(CCTNum_t o[3], CCTNum_t half[3], const CCTNum_t delta[3]);

__declspec_dll int mathAABBIntersectAABB(const CCTNum_t o1[3], const CCTNum_t half1[3], const CCTNum_t o2[3], const CCTNum_t half2[3]);
int AABB_Contain_Point(const CCTNum_t o[3], const CCTNum_t half[3], const CCTNum_t p[3]);
int AABB_Contain_AABB(const CCTNum_t o1[3], const CCTNum_t half1[3], const CCTNum_t o2[3], const CCTNum_t half2[3]);

#ifdef	__cplusplus
}
#endif

#endif
