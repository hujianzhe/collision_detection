//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_COOKING_H
#define	UTIL_C_CRT_GEOMETRY_COOKING_H

#include "polygon.h"

#ifdef	__cplusplus
extern "C" {
#endif

__declspec_dll GeometryPolygon_t* mathCookingPolygon(const CCTNum_t(*v)[3], const unsigned int* tri_v_indices, unsigned int tri_v_indices_cnt, GeometryPolygon_t* polygon);
__declspec_dll GeometryMesh_t* mathCookingMesh(const CCTNum_t(*v)[3], const unsigned int* tri_v_indices, unsigned int tri_v_indices_cnt, GeometryMesh_t* mesh);

#ifdef	__cplusplus
}
#endif

#endif
