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

__declspec_dll CCTNum_t* mathAABBPlaneVertex(const CCTNum_t min_v[3], const CCTNum_t max_v[3], unsigned int face_idx, CCTNum_t v[3]);
__declspec_dll const CCTNum_t* mathAABBPlaneNormal(unsigned int face_idx, CCTNum_t normal[3]);

__declspec_dll void mathAABBPlaneBoundBox(const CCTNum_t min_v[3], const CCTNum_t max_v[3], unsigned int face_idx, CCTNum_t bb_min[3], CCTNum_t bb_max[3]);

__declspec_dll void mathAABBVertices(const CCTNum_t min_v[3], const CCTNum_t max_v[3], CCTNum_t v[8][3]);
__declspec_dll CCTNum_t* mathAABBVertex(const CCTNum_t min_v[3], const CCTNum_t max_v[3], unsigned int v_id, CCTNum_t v[3]);

__declspec_dll void mathAABBMergePoint(CCTNum_t dst_min_v[3], CCTNum_t dst_max_v[3], const CCTNum_t p[3]);
__declspec_dll void mathAABBMergeAABB(CCTNum_t dst_min_v[3], CCTNum_t dst_max_v[3], const CCTNum_t src_min_v[3], const CCTNum_t src_max_v[3]);

__declspec_dll int mathAABBIntersectAABB(const CCTNum_t a_min_v[3], const CCTNum_t a_max_v[3], const CCTNum_t b_min_v[3], const CCTNum_t b_max_v[3]);

int AABB_Contain_Point(const CCTNum_t min_v[3], const CCTNum_t max_v[3], const CCTNum_t p[3]);
int AABB_Contain_AABB(const CCTNum_t a_min_v[3], const CCTNum_t a_max_v[3], const CCTNum_t b_min_v[3], const CCTNum_t b_max_v[3]);

#ifdef	__cplusplus
}
#endif

#endif
