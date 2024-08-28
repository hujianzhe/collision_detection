//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_MATH_MATRIX3_H
#define	UTIL_C_CRT_MATH_MATRIX3_H

#include "number_define.h"

/* 
 * note: Matrix[column][row]
 et. m[16] layout
 |m[0]	m[4]	m[8]	m[12]|
 |m[1]	m[5]	m[9]	m[13]|
 |m[2]	m[6]	m[10]	m[14]|
 |m[3]	m[7]	m[11]	m[15]|
 */

#ifdef	__cplusplus
extern "C" {
#endif

__declspec_dll void mathMat44TransformSplit(const CCTNum_t m[16], CCTNum_t T[3], CCTNum_t S[3], CCTNum_t R[9]);
__declspec_dll CCTNum_t* mathMat44SetPositionPart(CCTNum_t m[16], const CCTNum_t p[3]);
__declspec_dll CCTNum_t* mathMat44Element(const CCTNum_t m[16], unsigned int column_idx, unsigned int row_idx);
__declspec_dll CCTNum_t* mathMat44ToMat33(const CCTNum_t m44[16], CCTNum_t m33[9]);
__declspec_dll CCTNum_t* mathMat44Copy(CCTNum_t r[16], const CCTNum_t m[16]);
__declspec_dll CCTNum_t* mathMat44Identity(CCTNum_t m[16]);
__declspec_dll CCTNum_t* mathMat44Add(CCTNum_t r[16], const CCTNum_t m1[16], const CCTNum_t m2[16]);
__declspec_dll CCTNum_t* mathMat44MultiplyScalar(CCTNum_t r[16], const CCTNum_t m[16], CCTNum_t n);
__declspec_dll CCTNum_t* mathMat44MulMat44(CCTNum_t r[16], const CCTNum_t m1[16], const CCTNum_t m2[16]);
__declspec_dll CCTNum_t* mathMat44Transpose(CCTNum_t r[16], const CCTNum_t m[16]);
__declspec_dll CCTNum_t* mathMat44Inverse(CCTNum_t r[16], const CCTNum_t m[16]);
__declspec_dll CCTNum_t* mathMat44TransformVec3(CCTNum_t r[3], const CCTNum_t m[16], const CCTNum_t v[3]);
__declspec_dll CCTNum_t* mathMat44RotateVec3(CCTNum_t r[3], const CCTNum_t m[16], const CCTNum_t v[3]);
__declspec_dll CCTNum_t* mathMat44FromQuat(CCTNum_t m[16], const CCTNum_t q[4]);
__declspec_dll CCTNum_t* mathMat44ToQuat(const CCTNum_t m[16], CCTNum_t q[4]);
__declspec_dll CCTNum_t* mathMat33ToQuat(const CCTNum_t m[9], CCTNum_t q[4]);
__declspec_dll CCTNum_t* mathMat33FromQuat(CCTNum_t m[9], const CCTNum_t q[4]);

#ifdef	__cplusplus
}
#endif

#endif
