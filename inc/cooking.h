//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_COOKING_H
#define	UTIL_C_CRT_GEOMETRY_COOKING_H

#include "polygon.h"

typedef struct GeometryCookingOption_t {
	int tri_merged;
	CCTNum_t tri_edge_min_len;
} GeometryCookingOption_t;

typedef struct GeometryCookingOutput_t {
	GeometryMesh_t* mesh_ptr;
	GeometryPolygon_t* polygon_ptr;
	int error_code;
	int has_invalid_tri;
	CCTNum_t invalid_tri_v[3][3];
} GeometryCookingOutput_t;

#ifdef	__cplusplus
extern "C" {
#endif

__declspec_dll const GeometryCookingOutput_t* mathCookingPolygon(const CCTNum_t(*v)[3], const unsigned int* tri_v_indices, unsigned int tri_v_indices_cnt, const GeometryCookingOption_t* opt, GeometryCookingOutput_t* output);
__declspec_dll const GeometryCookingOutput_t* mathCookingMesh(const CCTNum_t(*v)[3], const unsigned int* tri_v_indices, unsigned int tri_v_indices_cnt, const GeometryCookingOption_t* opt, GeometryCookingOutput_t* output);

#ifdef	__cplusplus
}
#endif

#endif
