//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_VERTEX_H
#define	UTIL_C_CRT_GEOMETRY_VERTEX_H

#include "geometry_def.h"

#ifdef	__cplusplus
extern "C" {
#endif

__declspec_dll unsigned int mathVerticesDistinctCount(const CCTNum_t(*src_v)[3], unsigned int src_v_cnt);
__declspec_dll unsigned int mathVerticesMerge(const CCTNum_t(*src_v)[3], unsigned int v_cnt, const unsigned int* src_indices, unsigned int indices_cnt, CCTNum_t(*dst_v)[3], unsigned int* dst_indices);

__declspec_dll int mathVertexIndicesFindMinMaxXYZ(const CCTNum_t(*v)[3], const unsigned int* v_indices, unsigned int v_indices_cnt, CCTNum_t v_minXYZ[3], CCTNum_t v_maxXYZ[3]);
__declspec_dll int mathVerticesFindMinMaxXYZ(const CCTNum_t(*v)[3], unsigned int v_cnt, CCTNum_t v_minXYZ[3], CCTNum_t v_maxXYZ[3]);

__declspec_dll void mathTwoVertexFromCenterHalf(const CCTNum_t center_p[3], const CCTNum_t dir[3], CCTNum_t half_len, CCTNum_t start_v[3], CCTNum_t end_v[3]);
__declspec_dll void mathTwoVertexToCenterHalf(const CCTNum_t start_v[3], const CCTNum_t end_v[3], CCTNum_t center_p[3], CCTNum_t dir[3], CCTNum_t* half);

__declspec_dll unsigned int mathFindEdgeIndexByTwoVertexIndex(const unsigned int* edge_indices, unsigned int edge_indices_cnt, unsigned short edge_stride, unsigned int v_idx0, unsigned int v_idx1);

#ifdef	__cplusplus
}
#endif

#endif
