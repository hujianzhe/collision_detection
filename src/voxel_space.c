//
// Created by hujianzhe on 25-10-16
//

#include "../inc/voxel_space.h"
#include <stdlib.h>

static void node_indices_floor(const VoxelSpace_t* vs, const CCTNum_t p[3], unsigned int index_values[3]) {
	unsigned int i;
	for (i = 0; i < 3; ++i) {
		CCTNum_t delta = p[i] - vs->min_v[i] - vs->epsilon;
		if (delta <= CCTNum(0.0)) {
			index_values[i] = 0;
			continue;
		}
		index_values[i] = (unsigned int)CCTNum_floor(delta / vs->split_size[i]);
		if (index_values[i] > vs->_dimension_node_max_sz[i]) {
			index_values[i] = vs->_dimension_node_max_sz[i];
		}
	}
}

static void node_indices_ceil(const VoxelSpace_t* vs, const CCTNum_t p[3], unsigned int index_values[3]) {
	unsigned int i;
	for (i = 0; i < 3; ++i) {
		CCTNum_t delta = p[i] - vs->min_v[i] + vs->epsilon;
		if (delta <= CCTNum(0.0)) {
			index_values[i] = 0;
			continue;
		}
		index_values[i] = (unsigned int)CCTNum_ceil(delta / vs->split_size[i]);
		if (index_values[i] > vs->_dimension_node_max_sz[i]) {
			index_values[i] = vs->_dimension_node_max_sz[i];
		}
	}
}

static VoxelSpaceNode_t* get_node(const VoxelSpace_t* vs, size_t x, size_t y, size_t z) {
	size_t idx = x * vs->_dimension_stride0 + y * vs->_dimension_node_max_sz[2] + z;
	return vs->nodes + idx;
}

static void node_remove_obj(VoxelSpaceNode_t* node, VoxelSpaceObject_t* obj) {
	size_t i;
	for (i = 0; i < node->objs_cnt; ++i) {
		if (node->objs[i] == obj) {
			node->objs[i] = node->objs[--node->objs_cnt];
			return;
		}
	}
}

#ifdef __cplusplus
extern "C" {
#endif

VoxelSpace_t* voxelspaceInit(VoxelSpace_t* vs, const CCTNum_t min_v[3], const CCTNum_t max_v[3], const CCTNum_t split_size[3]) {
	size_t i, cnt[3], nodes_cnt;
	VoxelSpaceNode_t* nodes;
	for (i = 0; i < 3; ++i) {
		CCTNum_t delta = max_v[i] - min_v[i];
		if (delta <= CCTNum(0.0)) {
			return NULL;
		}
		if (delta <= split_size[i]) {
			cnt[i] = 1;
		}
		else {
			cnt[i] = (size_t)CCTNum_ceil(delta / split_size[i]);
		}
	}
	nodes_cnt = cnt[0] * cnt[1] * cnt[2];
	nodes = (VoxelSpaceNode_t*)calloc(1, sizeof(VoxelSpaceNode_t) * nodes_cnt);
	if (!nodes) {
		return NULL;
	}
	vs->nodes_cnt = nodes_cnt;
	vs->nodes = nodes;
	vs->epsilon = CCT_EPSILON;
	vs->min_v[0] = min_v[0];
	vs->min_v[1] = min_v[1];
	vs->min_v[2] = min_v[2];
	vs->max_v[0] = max_v[0];
	vs->max_v[1] = max_v[1];
	vs->max_v[2] = max_v[2];
	vs->split_size[0] = split_size[0];
	vs->split_size[1] = split_size[1];
	vs->split_size[2] = split_size[2];
	vs->_dimension_node_max_sz[0] = cnt[0];
	vs->_dimension_node_max_sz[1] = cnt[1];
	vs->_dimension_node_max_sz[2] = cnt[2];
	vs->_dimension_stride0 = cnt[1] * cnt[2];
	vs->_cap_expand = 8;
	return vs;
}

VoxelSpaceObject_t* voxelspaceUpdateObject(VoxelSpace_t* vs, VoxelSpaceObject_t* obj, const CCTNum_t min_v[3], const CCTNum_t max_v[3]) {
	size_t i, x, y, z, cnt;
	unsigned int start_idx[3], end_idx[3];

	node_indices_floor(vs, min_v, start_idx);
	node_indices_ceil(vs, max_v, end_idx);
	cnt = (end_idx[0] - start_idx[0]) * (end_idx[1] - start_idx[1]) * (end_idx[2] - start_idx[2]);
	if (cnt > obj->_locate_nodes_arr_cap) {
		unsigned int cap = cnt + vs->_cap_expand;
		VoxelSpaceNode_t** p = (VoxelSpaceNode_t**)realloc(obj->locate_nodes, sizeof(VoxelSpaceNode_t*) * cap);
		if (!p) {
			return NULL;
		}
		obj->locate_nodes = p;
		obj->_locate_nodes_arr_cap = cap;
	}
	for (x = start_idx[0]; x < end_idx[0]; ++x) {
		for (y = start_idx[1]; y < end_idx[1]; ++y) {
			for (z = start_idx[2]; z < end_idx[2]; ++z) {
				VoxelSpaceNode_t* node = get_node(vs, x, y, z);
				cnt = node->objs_cnt + 1;
				if (cnt > node->_objs_arr_cap) {
					unsigned int cap = cnt + vs->_cap_expand;
					VoxelSpaceObject_t** p = (VoxelSpaceObject_t**)realloc(node->objs, sizeof(VoxelSpaceObject_t*) * cap);
					if (!p) {
						return NULL;
					}
					node->objs = p;
					node->_objs_arr_cap = cap;
				}
			}
		}
	}

	for (i = 0; i < obj->locate_nodes_cnt; ++i) {
		node_remove_obj(obj->locate_nodes[i], obj);
	}
	obj->locate_nodes_cnt = 0;
	for (x = start_idx[0]; x < end_idx[0]; ++x) {
		for (y = start_idx[1]; y < end_idx[1]; ++y) {
			for (z = start_idx[2]; z < end_idx[2]; ++z) {
				VoxelSpaceNode_t* node = get_node(vs, x, y, z);
				node->objs[node->objs_cnt++] = obj;
				obj->locate_nodes[obj->locate_nodes_cnt++] = node;
			}
		}
	}
	return obj;
}

void voxelspaceRemoveObject(VoxelSpaceObject_t* obj) {
	size_t i;
	for (i = 0; i < obj->locate_nodes_cnt; ++i) {
		node_remove_obj(obj->locate_nodes[i], obj);
	}
	if (obj->locate_nodes) {
		free(obj->locate_nodes);
		obj->locate_nodes = NULL;
		obj->_locate_nodes_arr_cap = 0;
	}
	obj->locate_nodes_cnt = 0;
}

const VoxelSpaceNode_t* voxelspaceFindBegin(const VoxelSpace_t* vs, const CCTNum_t min_v[3], const CCTNum_t max_v[3], VoxelSpaceFinder_t* finder) {
	node_indices_floor(vs, min_v, finder->start_idx);
	node_indices_ceil(vs, max_v, finder->end_idx);
	if (finder->start_idx[0] >= finder->end_idx[0]) {
		return NULL;
	}
	if (finder->start_idx[1] >= finder->end_idx[1]) {
		return NULL;
	}
	if (finder->start_idx[2] >= finder->end_idx[2]) {
		return NULL;
	}
	finder->vs = vs;
	finder->cur_idx[0] = finder->start_idx[0];
	finder->cur_idx[1] = finder->start_idx[1];
	finder->cur_idx[2] = finder->start_idx[2];
	return get_node(vs, finder->cur_idx[0], finder->cur_idx[1], finder->cur_idx[2]);
}

const VoxelSpaceNode_t* voxelspaceFindNext(VoxelSpaceFinder_t* finder) {
	++finder->cur_idx[2];
	if (finder->cur_idx[2] < finder->end_idx[2]) {
		return get_node(finder->vs, finder->cur_idx[0], finder->cur_idx[1], finder->cur_idx[2]);
	}
	finder->cur_idx[2] = finder->start_idx[2];
	++finder->cur_idx[1];
	if (finder->cur_idx[1] < finder->end_idx[1]) {
		return get_node(finder->vs, finder->cur_idx[0], finder->cur_idx[1], finder->cur_idx[2]);
	}
	finder->cur_idx[1] = finder->start_idx[1];
	++finder->cur_idx[0];
	if (finder->cur_idx[0] < finder->end_idx[0]) {
		return get_node(finder->vs, finder->cur_idx[0], finder->cur_idx[1], finder->cur_idx[2]);
	}
	return NULL;
}

void voxelspaceClear(VoxelSpace_t* vs) {
	size_t i, j;
	for (i = 0; i < vs->nodes_cnt; ++i) {
		VoxelSpaceNode_t* node = vs->nodes + i;
		for (j = 0; j < node->objs_cnt; ++j) {
			VoxelSpaceObject_t* obj = node->objs[j];
			if (obj->locate_nodes) {
				free(obj->locate_nodes);
				obj->locate_nodes = NULL;
				obj->locate_nodes_cnt = 0;
			}
		}
		free(node->objs);
		node->objs = NULL;
		node->objs_cnt = 0;
	}
}

void voxelspaceDestroy(VoxelSpace_t* vs) {
	size_t i, j;
	for (i = 0; i < vs->nodes_cnt; ++i) {
		VoxelSpaceNode_t* node = vs->nodes + i;
		for (j = 0; j < node->objs_cnt; ++j) {
			VoxelSpaceObject_t* obj = node->objs[j];
			if (obj->locate_nodes) {
				free(obj->locate_nodes);
				obj->locate_nodes = NULL;
				obj->locate_nodes_cnt = 0;
			}
		}
		free(node->objs);
	}
	free(vs->nodes);
	vs->nodes = NULL;
	vs->nodes_cnt = 0;
}

#ifdef __cplusplus
}
#endif
