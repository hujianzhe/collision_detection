//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_LINE_SEGMENT_H
#define	UTIL_C_CRT_GEOMETRY_LINE_SEGMENT_H

#include "geometry_def.h"

#ifdef	__cplusplus
extern "C" {
#endif

__declspec_dll CCTNum_t mathPointProjectionLine(const CCTNum_t p[3], const CCTNum_t ls_v[3], const CCTNum_t lsdir[3], CCTNum_t np[3]);
__declspec_dll CCTNum_t mathLineCrossLine(const CCTNum_t lsv1[3], const CCTNum_t lsdir1[3], const CCTNum_t lsv2[3], const CCTNum_t lsdir2[3]);
__declspec_dll int mathPointProjectionSegment(const CCTNum_t p[3], const CCTNum_t ls0[3], const CCTNum_t ls1[3], CCTNum_t np[3], CCTNum_t epsilon);

#ifdef	__cplusplus
}
#endif

#endif
