//
// Created by hujianzhe on 25-10-16
//

#ifndef	UTIL_C_CRT_AABB_PARTITION_H
#define	UTIL_C_CRT_AABB_PARTITION_H

#include "number_define.h"

struct VoxelSpaceObject_t;
typedef struct VoxelSpaceNode_t {
	struct VoxelSpaceObject_t** objs;
	size_t objs_cnt;
	size_t _objs_arr_cap;
	union {
		void* ptr;
		unsigned long long u64;
	} udata;
} VoxelSpaceNode_t;

typedef struct VoxelSpaceObject_t {
	union {
		void* ptr;
		unsigned long long u64;
	} udata;
	struct VoxelSpaceNode_t** locate_nodes;
	size_t locate_nodes_cnt;
	size_t _locate_nodes_arr_cap;
} VoxelSpaceObject_t;

typedef struct VoxelSpace_t {
	long long min_v[3];
	long long max_v[3];
	unsigned long long split_size[3];
	CCTNum_t epsilon;
	struct VoxelSpaceNode_t* nodes;
	size_t nodes_cnt;
	size_t _dimension_node_max_sz[3];
	size_t _dimension_stride0;
	unsigned int _cap_expand;
} VoxelSpace_t;

typedef struct VoxelSpaceFinder_t {
	const VoxelSpace_t* _vs;
	size_t _start_idx[3];
	size_t _end_idx[3];
	size_t _cur_idx[3];
} VoxelSpaceFinder_t;

#ifdef __cplusplus
extern "C" {
#endif

__declspec_dll VoxelSpace_t* voxelspaceInit(VoxelSpace_t* vs, const CCTNum_t min_v[3], const CCTNum_t max_v[3], const CCTNum_t split_size[3]);

__declspec_dll void voxelspaceNodeIndexToXYZ(const VoxelSpace_t* vs, size_t node_index, size_t* x, size_t* y, size_t* z);
__declspec_dll void voxelspaceNodeBoundingBox(const VoxelSpace_t* vs, size_t x, size_t y, size_t z, CCTNum_t min_v[3], CCTNum_t max_v[3]);
__declspec_dll const VoxelSpaceNode_t* voxelspaceGetNodeByXYZ(const VoxelSpace_t* vs, size_t x, size_t y, size_t z);

__declspec_dll VoxelSpaceObject_t* voxelspaceUpdate(VoxelSpace_t* vs, VoxelSpaceObject_t* obj, const CCTNum_t min_v[3], const CCTNum_t max_v[3]);
__declspec_dll VoxelSpaceObject_t* voxelspaceUpdateEx(VoxelSpace_t* vs, VoxelSpaceObject_t* obj, const CCTNum_t boundbox_min_v[3], const CCTNum_t boundbox_max_v[3], const void* geo_data, int geo_type, int(*fn_check_intersect)(const void*, int, const CCTNum_t[3], const CCTNum_t[3]));
__declspec_dll void voxelspaceRemove(VoxelSpaceObject_t* obj);

__declspec_dll const VoxelSpaceNode_t* voxelspaceFindBegin(const VoxelSpace_t* vs, const CCTNum_t min_v[3], const CCTNum_t max_v[3], VoxelSpaceFinder_t* finder);
__declspec_dll const VoxelSpaceNode_t* voxelspaceFindNext(VoxelSpaceFinder_t* finder);

__declspec_dll void voxelspaceClear(VoxelSpace_t* vs);
__declspec_dll void voxelspaceDestroy(VoxelSpace_t* vs);

#ifdef __cplusplus
}
#endif

#endif
