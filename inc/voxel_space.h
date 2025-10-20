//
// Created by hujianzhe on 25-10-16
//

#ifndef	UTIL_C_CRT_AABB_PARTITION_H
#define	UTIL_C_CRT_AABB_PARTITION_H

#include "number_define.h"

struct VoxelSpaceObject_t;
typedef struct VoxelSpaceNode_t {
	struct VoxelSpaceObject_t** objs;
	unsigned int objs_cnt;
	unsigned int _objs_arr_cap;
} VoxelSpaceNode_t;

typedef struct VoxelSpaceObject_t {
	union {
		void* ptr;
		unsigned long long u64;
	} udata;
	struct VoxelSpaceNode_t** locate_nodes;
	unsigned int locate_nodes_cnt;
	unsigned int _locate_nodes_arr_cap;
} VoxelSpaceObject_t;

typedef struct VoxelSpace_t {
	CCTNum_t min_v[3];
	CCTNum_t max_v[3];
	CCTNum_t split_size[3];
	CCTNum_t epsilon;
	struct VoxelSpaceNode_t* nodes;
	unsigned int nodes_cnt;
	unsigned int _dimension_node_max_sz[3];
	unsigned int _dimension_stride0;
	unsigned int _cap_expand;
} VoxelSpace_t;

typedef struct VoxelSpaceFinder_t {
	const VoxelSpace_t* vs;
	unsigned int start_idx[3];
	unsigned int end_idx[3];
	unsigned int cur_idx[3];
} VoxelSpaceFinder_t;

#ifdef __cplusplus
extern "C" {
#endif

__declspec_dll VoxelSpace_t* voxelspaceInit(VoxelSpace_t* vs, const CCTNum_t min_v[3], const CCTNum_t max_v[3], const CCTNum_t split_size[3]);
__declspec_dll VoxelSpaceObject_t* voxelspaceUpdateObject(VoxelSpace_t* vs, VoxelSpaceObject_t* obj, const CCTNum_t min_v[3], const CCTNum_t max_v[3]);
__declspec_dll void voxelspaceRemoveObject(VoxelSpaceObject_t* obj);
__declspec_dll const VoxelSpaceNode_t* voxelspaceFindBegin(const VoxelSpace_t* vs, const CCTNum_t min_v[3], const CCTNum_t max_v[3], VoxelSpaceFinder_t* finder);
__declspec_dll const VoxelSpaceNode_t* voxelspaceFindNext(VoxelSpaceFinder_t* finder);

#ifdef __cplusplus
}
#endif

#endif
