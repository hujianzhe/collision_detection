//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_PLANE_H
#define	UTIL_C_CRT_GEOMETRY_PLANE_H

#include "geometry_def.h"

#ifdef	__cplusplus
extern "C" {
#endif

__declspec_dll CCTNum_t mathPointProjectionPlane(const CCTNum_t p[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3]);
__declspec_dll CCTNum_t mathPointProjectionPlanePoint(const CCTNum_t p[3], const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTNum_t np[3]);
__declspec_dll CCTNum_t mathSphereProjectionPlane(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t plane_v[3], const CCTNum_t plane_n[3], CCTNum_t sphere_np[3]);
__declspec_dll CCTNum_t mathPlaneNormalByVertices3(const CCTNum_t v0[3], const CCTNum_t v1[3], const CCTNum_t v2[3], CCTNum_t normal[3]);
__declspec_dll int mathPlaneEqual(const CCTNum_t v1[3], const CCTNum_t n1[3], const CCTNum_t v2[3], const CCTNum_t n2[3]);

#ifdef	__cplusplus
}
#endif

#endif
