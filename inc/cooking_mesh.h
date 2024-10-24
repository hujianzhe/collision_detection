//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_MESH_COOKING_H
#define	UTIL_C_CRT_GEOMETRY_MESH_COOKING_H

#include "mesh.h"

#ifdef	__cplusplus
extern "C" {
#endif

__declspec_dll GeometryMesh_t* mathCookingMeshDirect(const CCTNum_t(*v)[3], unsigned int v_cnt, const unsigned int* tri_indices, unsigned int tri_indices_cnt, GeometryMesh_t* mesh);
__declspec_dll GeometryMesh_t* mathCookingMesh(const CCTNum_t(*v)[3], unsigned int v_cnt, const unsigned int* tri_indices, unsigned int tri_indices_cnt, GeometryMesh_t* mesh);

#ifdef	__cplusplus
}
#endif

#endif
