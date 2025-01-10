//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_MATH_QUAT_H
#define UTIL_C_CRT_MATH_QUAT_H

#include "number_define.h"

#ifdef	__cplusplus
extern "C" {
#endif

__declspec_dll CCTNum_t* mathQuatSet(CCTNum_t q[4], CCTNum_t x, CCTNum_t y, CCTNum_t z, CCTNum_t w);
__declspec_dll CCTNum_t* mathQuatCopy(CCTNum_t r[4], const CCTNum_t q[4]);
__declspec_dll CCTNum_t* mathQuatNormalized(CCTNum_t r[4], const CCTNum_t q[4]);
__declspec_dll CCTNum_t* mathQuatFromEuler(CCTNum_t q[4], const CCTNum_t e[3], const char order[3]);
__declspec_dll CCTNum_t* mathQuatFromUnitVec3(CCTNum_t q[4], const CCTNum_t from[3], const CCTNum_t to[3]);
__declspec_dll CCTNum_t* mathQuatFromAxisRadian(CCTNum_t q[4], const CCTNum_t axis[3], CCTNum_t radian);
__declspec_dll void mathQuatToAxisRadian(const CCTNum_t q[4], CCTNum_t axis[3], CCTNum_t* radian);
__declspec_dll int mathQuatIsZero(const CCTNum_t q[4]);
__declspec_dll int mathQuatIsIdentity(CCTNum_t q[4]);
__declspec_dll int mathQuatIsZeroOrIdentity(const CCTNum_t q[4]);
__declspec_dll int mathQuatEqual(const CCTNum_t q1[4], const CCTNum_t q2[4]);
__declspec_dll CCTNum_t* mathQuatIdentity(CCTNum_t q[4]);
__declspec_dll CCTNum_t mathQuatDot(const CCTNum_t q1[4], const CCTNum_t q2[4]);
__declspec_dll CCTNum_t* mathQuatConjugate(CCTNum_t r[4], const CCTNum_t q[4]);
__declspec_dll CCTNum_t* mathQuatMultiplyScalar(CCTNum_t r[4], const CCTNum_t q[4], CCTNum_t n);
__declspec_dll CCTNum_t* mathQuatDivisionScalar(CCTNum_t r[4], const CCTNum_t q[4], CCTNum_t n);
__declspec_dll CCTNum_t* mathQuatMulQuat(CCTNum_t r[4], const CCTNum_t q1[4], const CCTNum_t q2[4]);
__declspec_dll CCTNum_t* mathQuatMulVec3(CCTNum_t r[3], const CCTNum_t q[4], const CCTNum_t v[3]);
__declspec_dll CCTNum_t* mathQuatMulVec3Inv(CCTNum_t r[3], const CCTNum_t q[4], const CCTNum_t v[3]);

#ifdef	__cplusplus
}
#endif

#endif
