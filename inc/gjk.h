//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_GJK_H
#define	UTIL_C_CRT_GEOMETRY_GJK_H

#include "number_define.h"

typedef struct GeometryConvexGJK_t {
	const CCTNum_t(*v)[3];
	const unsigned int* v_indices;
	union {
		unsigned int v_cnt;
		unsigned int v_indices_cnt;
	};
} GeometryConvexGJK_t;

typedef struct GeometrySimplexGJK_t {
	CCTNum_t p[4][3];
	CCTNum_t dir[3];
	unsigned int cnt;
} GeometrySimplexGJK_t;

#ifdef __cplusplus
extern "C" {
#endif

__declspec_dll int mathGJK(const GeometryConvexGJK_t* geo1, const GeometryConvexGJK_t* geo2, const CCTNum_t dir[3], GeometrySimplexGJK_t* s);

#ifdef __cplusplus
}
#endif

#endif
