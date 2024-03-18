//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_SPHERE_H
#define	UTIL_C_CRT_GEOMETRY_SPHERE_H

#include "geometry_def.h"

#ifdef	__cplusplus
extern "C" {
#endif

__declspec_dll int Sphere_Contain_Point(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t p[3]);
__declspec_dll int Sphere_Intersect_Sphere(const CCTNum_t o1[3], CCTNum_t r1, const CCTNum_t o2[3], CCTNum_t r2, CCTNum_t p[3]);
__declspec_dll int Sphere_Contain_Sphere(const CCTNum_t o1[3], CCTNum_t r1, const CCTNum_t o2[3], CCTNum_t r2);

#ifdef	__cplusplus
}
#endif

#endif
