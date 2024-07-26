//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_LINE_SEGMENT_H
#define	UTIL_C_CRT_GEOMETRY_LINE_SEGMENT_H

#include "geometry_def.h"

enum {
	GEOMETRY_LINE_SKEW = 0,
	GEOMETRY_LINE_PARALLEL = 1,
	GEOMETRY_LINE_CROSS = 2,
	GEOMETRY_LINE_OVERLAP = 3,

	GEOMETRY_SEGMENT_CONTACT = 1,
	GEOMETRY_SEGMENT_OVERLAP = 2,
};

typedef struct GeometrySegmentIndices_t {
	CCTNum_t(*v)[3];
	const unsigned int* indices;
	unsigned int indices_cnt;
	unsigned int stride;
	int is_convex;
} GeometrySegmentIndices_t;

#ifdef	__cplusplus
extern "C" {
#endif

__declspec_dll CCTNum_t mathPointProjectionLine(const CCTNum_t p[3], const CCTNum_t ls_v[3], const CCTNum_t lsdir[3], CCTNum_t np[3]);
__declspec_dll int mathLineClosestLine(const CCTNum_t lsv1[3], const CCTNum_t lsdir1[3], const CCTNum_t lsv2[3], const CCTNum_t lsdir2[3], CCTNum_t* min_d, CCTNum_t dir_d[2]);

__declspec_dll int mathSegmentIsSame(const CCTNum_t ls1[2][3], const CCTNum_t ls2[2][3]);
__declspec_dll void mathSegmentClosestPointTo(const CCTNum_t ls[2][3], const CCTNum_t p[3], CCTNum_t closest_p[3]);
__declspec_dll void mathSegmentClosestPointTo_v2(const CCTNum_t ls_center_p[3], const CCTNum_t lsdir[3], const CCTNum_t ls_half_len, const CCTNum_t p[3], CCTNum_t closest_p[3]);
__declspec_dll CCTNum_t mathSegmentSegmentClosestIndices(const CCTNum_t ls1[2][3], const CCTNum_t ls2[2][3], unsigned int* ls1_indices, unsigned int* ls2_indices);
__declspec_dll int mathSegmentClosestSegment(const CCTNum_t ls1[2][3], const CCTNum_t ls2[2][3], CCTNum_t closest_p[2][3]);

#ifdef	__cplusplus
}
#endif

#endif
