//
// Created by hujianzhe
//

#ifndef UTIL_C_CRT_MATH_VEC3_H
#define	UTIL_C_CRT_MATH_VEC3_H

#include "compiler_define.h"
#include "number_define.h"

#ifdef	__cplusplus
extern "C" {
#endif

__declspec_dll CCTNum_t* mathCoordinateSystemTransform(const CCTNum_t v[3], const CCTNum_t new_origin[3], const CCTNum_t new_axies[3][3], CCTNum_t new_v[3]);

__declspec_dll int mathVec3IsZero(const CCTNum_t v[3]);
__declspec_dll int mathVec3Equal(const CCTNum_t v1[3], const CCTNum_t v2[3]);
__declspec_dll CCTNum_t mathVec3MinElement(const CCTNum_t v[3]);
__declspec_dll CCTNum_t mathVec3MaxElement(const CCTNum_t v[3]);
__declspec_dll CCTNum_t* mathVec3Set(CCTNum_t r[3], CCTNum_t x, CCTNum_t y, CCTNum_t z);
__declspec_dll CCTNum_t* mathVec3Copy(CCTNum_t r[3], const CCTNum_t v[3]);
__declspec_dll CCTNum_t mathVec3LenSq(const CCTNum_t v[3]);
__declspec_dll CCTNum_t mathVec3Len(const CCTNum_t v[3]);
__declspec_dll CCTNum_t mathVec3Normalized(CCTNum_t r[3], const CCTNum_t v[3]);
__declspec_dll CCTNum_t mathVec3Direction(const CCTNum_t end[3], const CCTNum_t start[3], CCTNum_t dir[3]);
__declspec_dll CCTNum_t mathVec3DistanceSq(const CCTNum_t p1[3], const CCTNum_t p2[3]);
__declspec_dll CCTNum_t* mathVec3Negate(CCTNum_t r[3], const CCTNum_t v[3]);
__declspec_dll CCTNum_t* mathVec3Add(CCTNum_t r[3], const CCTNum_t v1[3], const CCTNum_t v2[3]);
__declspec_dll CCTNum_t* mathVec3AddScalar(CCTNum_t r[3], const CCTNum_t v[3], CCTNum_t n);
__declspec_dll CCTNum_t* mathVec3SubScalar(CCTNum_t r[3], const CCTNum_t v[3], CCTNum_t n);
__declspec_dll CCTNum_t* mathVec3Sub(CCTNum_t r[3], const CCTNum_t v1[3], const CCTNum_t v2[3]);
__declspec_dll CCTNum_t* mathVec3MultiplyScalar(CCTNum_t r[3], const CCTNum_t v[3], CCTNum_t n);
__declspec_dll CCTNum_t* mathVec3DivisionScalar(CCTNum_t r[3], const CCTNum_t v[3], CCTNum_t n);
__declspec_dll CCTNum_t mathVec3Dot(const CCTNum_t v1[3], const CCTNum_t v2[3]);
__declspec_dll CCTNum_t mathVec3Radian(const CCTNum_t v1[3], const CCTNum_t v2[3]);
__declspec_dll CCTNum_t* mathVec3Cross(CCTNum_t r[3], const CCTNum_t v1[3], const CCTNum_t v2[3]);
__declspec_dll CCTNum_t mathVec3CrossNormalized(CCTNum_t r[3], const CCTNum_t v1[3], const CCTNum_t v2[3]);
__declspec_dll CCTNum_t* mathVec3Reflect(CCTNum_t r[3], const CCTNum_t v[3], const CCTNum_t n[3]);
__declspec_dll CCTNum_t* mathVec3Glide(CCTNum_t r[3], const CCTNum_t v[3], const CCTNum_t n[3]);
__declspec_dll void mathVec3ComputeBasis(const CCTNum_t dir[3], CCTNum_t right[3], CCTNum_t up[3]);
__declspec_dll CCTNum_t* mathVec3DelComponent(CCTNum_t r[3], const CCTNum_t v[3], const CCTNum_t dir[3]);

#ifdef	__cplusplus
}
#endif

#endif // !UTIL_C_CRT_MATH_VEC3_H
