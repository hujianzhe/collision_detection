//
// Created by hujianzhe on 21-12-29
//

#ifndef	UTIL_C_CRT_OCTREE_H
#define	UTIL_C_CRT_OCTREE_H

#include "number_define.h"

struct OctreeObject_t;
typedef struct OctreeNode_t {
	CCTNum_t min_v[3];
	CCTNum_t max_v[3];
	struct OctreeObject_t* obj_list_head;
	unsigned int obj_cnt;
	unsigned int deep_num; /* from 1,2,3... */
	struct OctreeNode_t* parent;
	struct OctreeNode_t* childs;
} OctreeNode_t;

typedef struct OctreeObject_t {
	struct OctreeObject_t* prev;
	struct OctreeObject_t* next;
	const CCTNum_t* min_v; /* point to your CCTNum_t[3] */
	const CCTNum_t* max_v;	/* point to your CCTNum_t[3] */
	OctreeNode_t* oct;
	union {
		void* ptr;
		unsigned long long u64;
	} udata;
} OctreeObject_t;

typedef struct Octree_t {
	OctreeNode_t* nodes;
	unsigned int nodes_cnt;
	unsigned int max_deep_num; /* must >= 1 */
	unsigned int split_cnt_per_node; /*/ must >= 1 */
} Octree_t;

typedef struct OctreeFinder_t {
	const OctreeNode_t** nodes;
	unsigned int cnt;
	unsigned int cap;
} OctreeFinder_t;

#ifdef __cplusplus
extern "C" {
#endif

__declspec_dll unsigned int octreeCalculateDeepNumByCellSize(const CCTNum_t half_size[3], CCTNum_t cell_size);

__declspec_dll Octree_t* octreeInit(Octree_t* tree, const CCTNum_t pos[3], const CCTNum_t half[3], unsigned int max_deep_num, unsigned int split_cnt_per_node);
__declspec_dll void octreeUpdate(Octree_t* tree, OctreeObject_t* obj);
__declspec_dll void octreeRemove(OctreeObject_t* obj);

__declspec_dll OctreeFinder_t* octreeFinderAlloc(const Octree_t* tree, OctreeFinder_t* finder);
__declspec_dll void octreeFinderDestroy(OctreeFinder_t* finder);

__declspec_dll void octreeFindNodes(const Octree_t* tree, const CCTNum_t min_v[3], const CCTNum_t max_v[3], OctreeFinder_t* finder);
__declspec_dll void octreeClear(Octree_t* tree);
__declspec_dll void octreeDestroy(Octree_t* tree);

#ifdef __cplusplus
}
#endif

#endif
