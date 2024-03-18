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

#ifdef	__cplusplus
extern "C" {
#endif

__declspec_dll CCTNum_t mathPointProjectionLine(const CCTNum_t p[3], const CCTNum_t ls_v[3], const CCTNum_t lsdir[3], CCTNum_t np[3]);
__declspec_dll int mathLineClosestLine(const CCTNum_t lsv1[3], const CCTNum_t lsdir1[3], const CCTNum_t lsv2[3], const CCTNum_t lsdir2[3], CCTNum_t* min_d, CCTNum_t dir_d[2]);
__declspec_dll int mathLineIntersectLine(const CCTNum_t ls1v[3], const CCTNum_t ls1dir[3], const CCTNum_t ls2v[3], const CCTNum_t ls2dir[3], CCTNum_t distance[2]);

__declspec_dll int mathSegmentIsSame(const CCTNum_t ls1[2][3], const CCTNum_t ls2[2][3]);
__declspec_dll void mathSegmentClosestPointTo(const CCTNum_t ls[2][3], const CCTNum_t p[3], CCTNum_t closest_p[3]);
__declspec_dll void mathSegmentClosestSegmentVertice(const CCTNum_t ls1[2][3], const CCTNum_t ls2[2][3], CCTNum_t closest_p[2][3]);
__declspec_dll int mathSegmentClosestSegment(const CCTNum_t ls1[2][3], const CCTNum_t ls2[2][3], CCTNum_t closest_p[2][3]);

int Segment_Contain_Point(const CCTNum_t ls[2][3], const CCTNum_t p[3]);
int Segment_Contain_Segment(const CCTNum_t ls1[2][3], const CCTNum_t ls2[2][3]);
int Segment_Intersect_Segment(const CCTNum_t ls1[2][3], const CCTNum_t ls2[2][3], CCTNum_t p[3], int* line_mask);

#ifdef	__cplusplus
}
#endif

#endif
