//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_COOKING_H
#define	UTIL_C_CRT_GEOMETRY_COOKING_H

#include "polygon.h"

#ifdef	__cplusplus
extern "C" {
#endif

int mathCookingStage1(const CCTNum_t(*v)[3], const unsigned int* tri_v_indices, unsigned int tri_v_indices_cnt, CCTNum_t(**ret_v)[3], unsigned int* ret_v_cnt, unsigned int** ret_tri_v_indices);
int mathCookingStage2(const CCTNum_t(*v)[3], const unsigned int* tri_v_indices, unsigned int tri_v_indices_cnt, GeometryPolygon_t** ret_polygons, unsigned int* ret_polygons_cnt);
int mathCookingStage3(const CCTNum_t(*v)[3], const unsigned int* tri_v_indices, unsigned int tri_v_indices_cnt, const CCTNum_t plane_n[3], unsigned int** ret_edge_v_indices, unsigned int* ret_edge_v_indices_cnt);
int mathCookingStage4(const unsigned int* edge_v_indices_flat, unsigned int edge_v_indices_cnt, unsigned int** ret_v_indices, unsigned int* ret_v_indices_cnt, unsigned int** ret_edge_v_ids_flat);

__declspec_dll GeometryPolygon_t* mathCookingPolygon(const CCTNum_t(*v)[3], const unsigned int* tri_v_indices, unsigned int tri_v_indices_cnt, GeometryPolygon_t* polygon);
__declspec_dll GeometryMesh_t* mathCookingMesh(const CCTNum_t(*v)[3], const unsigned int* tri_v_indices, unsigned int tri_v_indices_cnt, GeometryMesh_t* mesh);

#ifdef	__cplusplus
}
#endif

#endif
