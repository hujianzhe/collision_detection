//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_VERTEX_H
#define	UTIL_C_CRT_GEOMETRY_VERTEX_H

#include "geometry_def.h"

#ifdef	__cplusplus
extern "C" {
#endif

__declspec_dll unsigned int mathVerticesMerge(const CCTNum_t(*src_v)[3], const unsigned int* src_indices, unsigned int indices_cnt, CCTNum_t(*dst_v)[3], unsigned int* dst_indices);
__declspec_dll int mathEdgeVertexIndicesMergeEdgeVertexIndices(const CCTNum_t(*v)[3], const unsigned int a_edge_v_indices[2], const unsigned int b_edge_v_indices[2], unsigned int final_edge_v_indices[2]);

__declspec_dll int mathVertexIndicesFindMinMaxXYZ(const CCTNum_t(*v)[3], const unsigned int* v_indices, unsigned int v_indices_cnt, CCTNum_t v_minXYZ[3], CCTNum_t v_maxXYZ[3]);
__declspec_dll int mathVerticesFindMinMaxXYZ(const CCTNum_t(*v)[3], unsigned int v_cnt, CCTNum_t v_minXYZ[3], CCTNum_t v_maxXYZ[3]);

__declspec_dll int mathVertexIndicesAverageXYZ(const CCTNum_t(*v)[3], const unsigned int* v_indices, unsigned int v_indices_cnt, CCTNum_t v_avgXYZ[3]);

__declspec_dll void mathTwoVertexFromCenterHalf(const CCTNum_t center_p[3], const CCTNum_t dir[3], CCTNum_t half_len, CCTNum_t start_v[3], CCTNum_t end_v[3]);
__declspec_dll void mathTwoVertexToCenterHalf(const CCTNum_t start_v[3], const CCTNum_t end_v[3], CCTNum_t center_p[3], CCTNum_t dir[3], CCTNum_t* half);

__declspec_dll unsigned int mathFindVertexId(const CCTNum_t(*v)[3], const unsigned int* v_indices, unsigned int v_indices_cnt, const CCTNum_t p[3]);
__declspec_dll unsigned int mathFindEdgeIdByVertexIndices(const unsigned int* edge_v_indices_flat, unsigned int edge_v_indices_cnt, unsigned int v_idx0, unsigned int v_idx1);
__declspec_dll unsigned int mathFindEdgeIdByVertices(const CCTNum_t(*v)[3], const unsigned int* edge_v_indices_flat, unsigned int edge_v_indices_cnt, const CCTNum_t p0[3], const CCTNum_t p1[3]);
__declspec_dll int mathFindBorderIdByPoint(const CCTNum_t(*v)[3], const unsigned int* v_indices, const unsigned int* edge_v_ids_flat, unsigned int edge_v_indices_cnt, const CCTNum_t p[3], GeometryBorderId_t* bi);
__declspec_dll unsigned int mathFindFaceIdByVertexIndices(const GeometryPolygon_t* faces, unsigned int faces_cnt, const unsigned int* v_idx, unsigned int v_idx_cnt);
__declspec_dll unsigned int mathFindAdjacentFaceIdByEdgeVertexIndices(const GeometryPolygon_t* faces, unsigned int faces_cnt, unsigned int edge_v_idx0, unsigned int edge_v_idx1, unsigned int* face_idx0, unsigned int* face_idx1);

#ifdef	__cplusplus
}
#endif

#endif
