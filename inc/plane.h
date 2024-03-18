//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_PLANE_H
#define	UTIL_C_CRT_GEOMETRY_PLANE_H

#include "geometry_def.h"

#ifdef	__cplusplus
extern "C" {
#endif

__declspec_dll void mathPointProjectionPlane(const CCTNum_t p[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTNum_t np[3], CCTNum_t* distance);
__declspec_dll CCTNum_t mathPlaneNormalByVertices3(const CCTNum_t v0[3], const CCTNum_t v1[3], const CCTNum_t v2[3], CCTNum_t normal[3]);

int Plane_Contain_Point(const CCTNum_t plane_v[3], const CCTNum_t plane_normal[3], const CCTNum_t p[3]);
int Plane_Intersect_Plane(const CCTNum_t v1[3], const CCTNum_t n1[3], const CCTNum_t v2[3], const CCTNum_t n2[3]);

#ifdef	__cplusplus
}
#endif

#endif
