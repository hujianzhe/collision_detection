//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_COOKING_H
#define	UTIL_C_CRT_GEOMETRY_COOKING_H

#include "geometry_def.h"

typedef struct MeshCookingOption_t {
	int tri_merged;
	CCTNum_t tri_edge_min_len;
	CCTNum_t tri_min_degree;
} MeshCookingOption_t;

enum {
	MESH_COOKING_ERR_INVALID_TRIANGLES = 1,
	MESH_COOKING_ERR_ISOLATE_VERTICES = 2,
	MESH_COOKING_ERR_UNKNOW
};

typedef struct MeshCookingOutput_t {
	GeometryMesh_t* mesh_ptr;
	int err;
	union {
		CCTNum_t invalid_tri_v[3][3];
		CCTNum_t isolate_v[3];
	};
} MeshCookingOutput_t;

#ifdef	__cplusplus
extern "C" {
#endif

__declspec_dll int mathCookingConvexHull(const CCTNum_t(*v)[3], unsigned int v_cnt, unsigned int** ret_tri_v_indices, unsigned int* ret_tri_v_indices_cnt, const CCTAllocator_t* ac);
__declspec_dll const MeshCookingOutput_t* mathCookingMesh(const CCTNum_t(*v)[3], const unsigned int* tri_v_indices, unsigned int tri_v_indices_cnt, const MeshCookingOption_t* opt, MeshCookingOutput_t* output, const CCTAllocator_t* ac);

#ifdef	__cplusplus
}
#endif

#endif
