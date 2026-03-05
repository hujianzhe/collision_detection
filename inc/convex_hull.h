//
// Created by hujianzhe (convex hull)
//

#ifndef UTIL_C_CRT_GEOMETRY_CONVEX_HULL_H
#define UTIL_C_CRT_GEOMETRY_CONVEX_HULL_H

#include "number_define.h"

#ifdef __cplusplus
extern "C" {
#endif

__declspec_dll int mathConvexHullBuild(const CCTNum_t (*v)[3], unsigned int v_cnt, unsigned int **ret_tri_v_indices, unsigned int *ret_tri_v_indices_cnt);

#ifdef __cplusplus
}
#endif

#endif

