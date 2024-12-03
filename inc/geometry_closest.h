//
// Created by hujianzhe
//

#ifndef UTIL_C_CRT_GEOMETRY_CLOSEST_H
#define	UTIL_C_CRT_GEOMETRY_CLOSEST_H

#include "geometry_def.h"

#ifdef	__cplusplus
extern "C" {
#endif

__declspec_dll void mathLineClosestLineOpposite(const CCTNum_t lsv1[3], const CCTNum_t lsdir1[3], const CCTNum_t lsv2[3], const CCTNum_t lsdir2[3], CCTNum_t* lsdir_d1, CCTNum_t* lsdir_d2);

__declspec_dll void mathSegmentClosestPoint(const CCTNum_t ls0[3], const CCTNum_t ls1[3], const CCTNum_t p[3], CCTNum_t closest_p[3]);
__declspec_dll void mathSegmentClosestPoint_v2(const CCTNum_t ls_center_p[3], const CCTNum_t lsdir[3], const CCTNum_t ls_half_len, const CCTNum_t p[3], CCTNum_t closest_p[3]);
__declspec_dll void mathSegmentClosestPoint_v3(const CCTNum_t ls_v[3], const CCTNum_t lsdir[3], const CCTNum_t ls_len, const CCTNum_t p[3], CCTNum_t closest_p[3]);
__declspec_dll CCTNum_t mathSegmentSegmentClosestIndices(const CCTNum_t ls1[2][3], const CCTNum_t ls2[2][3], unsigned int* ls1_indices, unsigned int* ls2_indices);
__declspec_dll CCTNum_t mathSegmentClosestSegmentDistanceSq(const CCTNum_t ls1[2][3], const CCTNum_t ls1_dir[3], CCTNum_t ls1_len, const CCTNum_t ls2[2][3], const CCTNum_t ls2_dir[3], CCTNum_t ls2_len);

__declspec_dll void mathAABBClosestPoint(const CCTNum_t o[3], const CCTNum_t half[3], const CCTNum_t p[3], CCTNum_t closest_p[3]);
__declspec_dll void mathOBBClosestPoint(const GeometryOBB_t* obb, const CCTNum_t p[3], CCTNum_t closest_p[3]);

#ifdef	__cplusplus
}
#endif

#endif