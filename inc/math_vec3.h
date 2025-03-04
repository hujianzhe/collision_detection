//
// Created by hujianzhe
//

#ifndef UTIL_C_CRT_MATH_VEC3_H
#define	UTIL_C_CRT_MATH_VEC3_H

#include "number_define.h"

#ifdef	__cplusplus
extern "C" {
#endif

__declspec_dll int mathVec3IsZero(const CCTNum_t v[3]);
__declspec_dll int mathVec3Equal(const CCTNum_t v1[3], const CCTNum_t v2[3]);
__declspec_dll int mathVec3EqualEps(const CCTNum_t v1[3], const CCTNum_t v2[3], CCTNum_t eps);

__declspec_dll CCTNum_t mathVec3MinElement(const CCTNum_t v[3]);
__declspec_dll CCTNum_t mathVec3MaxElement(const CCTNum_t v[3]);
__declspec_dll CCTNum_t* mathVec3MergeMin(CCTNum_t r[3], const CCTNum_t v1[3], const CCTNum_t v2[3]);
__declspec_dll CCTNum_t* mathVec3MergeMax(CCTNum_t r[3], const CCTNum_t v1[3], const CCTNum_t v2[3]);
__declspec_dll CCTNum_t* mathVec3Set(CCTNum_t r[3], CCTNum_t x, CCTNum_t y, CCTNum_t z);
__declspec_dll CCTNum_t* mathVec3Copy(CCTNum_t r[3], const CCTNum_t v[3]);
__declspec_dll CCTNum_t mathVec3LenSq(const CCTNum_t v[3]);
__declspec_dll CCTNum_t mathVec3Len(const CCTNum_t v[3]);
__declspec_dll CCTNum_t mathVec3Normalized(CCTNum_t r[3], const CCTNum_t v[3]);
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
__declspec_dll CCTNum_t* mathVec3DelComponent(CCTNum_t r[3], const CCTNum_t v[3], const CCTNum_t dir[3]);

#ifdef	__cplusplus
}
#endif

#endif // !UTIL_C_CRT_MATH_VEC3_H
