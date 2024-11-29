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
	unsigned int cnt;
} GeometrySimplexGJK_t;

typedef struct GeometryGJKIterator_t {
	/* read only */
	const GeometryConvexGJK_t* geo1;
	const GeometryConvexGJK_t* geo2;
	CCTNum_t dir[3];
	GeometrySimplexGJK_t simplex;
	int overlap;
	/* private */
	unsigned int left_iter_times_;
} GeometryGJKIterator_t;

#ifdef __cplusplus
extern "C" {
#endif

__declspec_dll int mathGJK(const GeometryConvexGJK_t* geo1, const GeometryConvexGJK_t* geo2, const CCTNum_t init_dir[3], GeometryGJKIterator_t* iter);
__declspec_dll void mathGJKBegin(GeometryGJKIterator_t* iter, const GeometryConvexGJK_t* geo1, const GeometryConvexGJK_t* geo2, const CCTNum_t init_dir[3]);
__declspec_dll int mathGJKNext(GeometryGJKIterator_t* iter);

#ifdef __cplusplus
}
#endif

#endif
