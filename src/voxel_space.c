//
// Created by hujianzhe on 25-10-16
//

#include "../inc/voxel_space.h"
#include <limits.h>
#include <stdint.h>
#include <stdlib.h>

static void calculate_index(long long min_v, unsigned long long sz, size_t count, CCTNum_t v, size_t* p_floor_idx, size_t* p_ceil_idx) {
	long long d;
	long long vl = CCTNum_floor(v);
	long long vh = CCTNum_ceil(v);
	if (vl < min_v) {
		*p_floor_idx = *p_ceil_idx = 0;
		return;
	}
	if (vl == vh) {
		d = vl - min_v;
		*p_floor_idx = d / sz;
		*p_ceil_idx = *p_floor_idx + 1;
		if (*p_floor_idx > 0 && 0 == d % sz) {
			*p_floor_idx -= 1;
		}
	}
	else {
		d = vl - min_v;
		*p_floor_idx = d / sz;
		d = vh - min_v;
		*p_ceil_idx = d / sz;
		if (*p_ceil_idx <= *p_floor_idx) {
			*p_ceil_idx = *p_floor_idx + 1;
		}
	}
	if (*p_floor_idx > count) {
		*p_floor_idx = count;
	}
	if (*p_ceil_idx > count) {
		*p_ceil_idx = count;
	}
}

static void node_range_indices(const VoxelSpace_t* vs, const CCTNum_t p1[3], const CCTNum_t p2[3], size_t start_idx[3], size_t end_idx[3]) {
	size_t i;
	for (i = 0; i < 3; ++i) {
		size_t p1_start_idx, p1_end_idx, p2_start_idx, p2_end_idx;
		calculate_index(vs->min_v[i], vs->split_size[i], vs->_dimension_node_max_sz[i], p1[i], &p1_start_idx, &p1_end_idx);
		calculate_index(vs->min_v[i], vs->split_size[i], vs->_dimension_node_max_sz[i], p2[i], &p2_start_idx, &p2_end_idx);
		start_idx[i] = (p1_start_idx < p2_start_idx ? p1_start_idx : p2_start_idx);
		end_idx[i] = (p1_end_idx > p2_end_idx ? p1_end_idx : p2_end_idx);
	}
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

static int voxelspace_update_prepare_memory(VoxelSpace_t* vs, VoxelSpaceObject_t* obj, const size_t start_idx[3], const size_t end_idx[3]) {
	size_t x, y, z;
	size_t cnt = (end_idx[0] - start_idx[0]) * (end_idx[1] - start_idx[1]) * (end_idx[2] - start_idx[2]);
	if (cnt > obj->_locate_nodes_arr_cap) {
		size_t cap = cnt + vs->_cap_expand;
		VoxelSpaceNode_t** p = (VoxelSpaceNode_t**)realloc(obj->locate_nodes, sizeof(VoxelSpaceNode_t*) * cap);
		if (!p) {
			return 0;
		}
		obj->locate_nodes = p;
		obj->_locate_nodes_arr_cap = cap;
	}
	for (x = start_idx[0]; x < end_idx[0]; ++x) {
		for (y = start_idx[1]; y < end_idx[1]; ++y) {
			for (z = start_idx[2]; z < end_idx[2]; ++z) {
				VoxelSpaceNode_t* node = vs->nodes + voxelspaceNodeIndexFromXYZ(vs, x, y, z);
				cnt = node->objs_cnt + 1;
				if (cnt > node->_objs_arr_cap) {
					size_t cap = cnt + vs->_cap_expand;
					VoxelSpaceObject_t** p = (VoxelSpaceObject_t**)realloc(node->objs, sizeof(VoxelSpaceObject_t*) * cap);
					if (!p) {
						return 0;
					}
					node->objs = p;
					node->_objs_arr_cap = cap;
				}
			}
		}
	}
	return 1;
}

#ifdef __cplusplus
extern "C" {
#endif

VoxelSpace_t* voxelspaceInit(VoxelSpace_t* vs, const CCTNum_t min_v_[3], const CCTNum_t max_v_[3], const CCTNum_t split_size_[3], int alloc_nodes) {
	size_t i, cnt[3], nodes_cnt;
	vs->min_v[0] = CCTNum_floor(min_v_[0]);
	vs->min_v[1] = CCTNum_floor(min_v_[1]);
	vs->min_v[2] = CCTNum_floor(min_v_[2]);
	vs->max_v[0] = CCTNum_ceil(max_v_[0]);
	vs->max_v[1] = CCTNum_ceil(max_v_[1]);
	vs->max_v[2] = CCTNum_ceil(max_v_[2]);
	vs->split_size[0] = CCTNum_ceil(split_size_[0]);
	vs->split_size[1] = CCTNum_ceil(split_size_[1]);
	vs->split_size[2] = CCTNum_ceil(split_size_[2]);
	for (i = 0; i < 3; ++i) {
		long long delta = vs->max_v[i] - vs->min_v[i];
		if (delta <= 0 || vs->split_size[i] <= 0) {
			return NULL;
		}
		cnt[i] = delta / vs->split_size[i];
		if (cnt[i] * vs->split_size[i] < delta) {
			++cnt[i];
		}
		if ((long long)(cnt[i] * vs->split_size[i]) <= 0) {
			return NULL;
		}
	}
	if (cnt[1] > SIZE_MAX / cnt[0] || cnt[0] * cnt[1] > SIZE_MAX / cnt[2]) {
		return NULL;
	}
	nodes_cnt = cnt[0] * cnt[1] * cnt[2];
	if (nodes_cnt > SIZE_MAX / sizeof(VoxelSpaceNode_t)) {
		return NULL;
	}
	if (alloc_nodes) {
		vs->nodes = (VoxelSpaceNode_t*)calloc(1, sizeof(VoxelSpaceNode_t) * nodes_cnt);
		if (!vs->nodes) {
			return NULL;
		}
	}
	else {
		vs->nodes = NULL;
	}
	vs->_alloc_nodes = alloc_nodes;
	vs->_dimension_node_max_sz[0] = cnt[0];
	vs->_dimension_node_max_sz[1] = cnt[1];
	vs->_dimension_node_max_sz[2] = cnt[2];
	vs->_dimension_stride0 = cnt[1] * cnt[2];
	vs->_cap_expand = 8;
	return vs;
}

void voxelspaceNodeIndexToXYZ(const VoxelSpace_t* vs, size_t node_index, size_t* x, size_t* y, size_t* z) {
	*x = node_index / vs->_dimension_stride0;
	node_index %= vs->_dimension_stride0;
	*y = node_index / vs->_dimension_node_max_sz[2];
	*z = node_index % vs->_dimension_node_max_sz[2];
}

void voxelspaceNodeBoundingBox(const VoxelSpace_t* vs, size_t x, size_t y, size_t z, CCTNum_t min_v[3], CCTNum_t max_v[3]) {
	const long long v[3] = {
		vs->min_v[0] + (long long)(x * vs->split_size[0]),
		vs->min_v[1] + (long long)(y * vs->split_size[1]),
		vs->min_v[2] + (long long)(z * vs->split_size[2])
	};
	min_v[0] = v[0];
	min_v[1] = v[1];
	min_v[2] = v[2];
	max_v[0] = v[0] + (long long)vs->split_size[0];
	max_v[1] = v[1] + (long long)vs->split_size[1];
	max_v[2] = v[2] + (long long)vs->split_size[2];
}

const VoxelSpaceNode_t* voxelspaceGetNodeByXYZ(const VoxelSpace_t* vs, size_t x, size_t y, size_t z) {
	if (x >= vs->_dimension_node_max_sz[0] || y >= vs->_dimension_node_max_sz[1] || z >= vs->_dimension_node_max_sz[2]) {
		return NULL;
	}
	return vs->nodes + voxelspaceNodeIndexFromXYZ(vs, x, y, z);
}

size_t voxelspaceNodeIndexFromXYZ(const VoxelSpace_t* vs, size_t x, size_t y, size_t z) {
	return x * vs->_dimension_stride0 + y * vs->_dimension_node_max_sz[2] + z;
}

VoxelSpaceObject_t* voxelspaceUpdate(VoxelSpace_t* vs, VoxelSpaceObject_t* obj, const CCTNum_t min_v[3], const CCTNum_t max_v[3]) {
	size_t i, x, y, z;
	size_t start_idx[3], end_idx[3];
	node_range_indices(vs, min_v, max_v, start_idx, end_idx);
	if (!voxelspace_update_prepare_memory(vs, obj, start_idx, end_idx)) {
		return NULL;
	}

	for (i = 0; i < obj->locate_nodes_cnt; ++i) {
		node_remove_obj(obj->locate_nodes[i], obj);
	}
	obj->locate_nodes_cnt = 0;
	for (x = start_idx[0]; x < end_idx[0]; ++x) {
		for (y = start_idx[1]; y < end_idx[1]; ++y) {
			for (z = start_idx[2]; z < end_idx[2]; ++z) {
				VoxelSpaceNode_t* node = vs->nodes + voxelspaceNodeIndexFromXYZ(vs, x, y, z);
				node->objs[node->objs_cnt++] = obj;
				obj->locate_nodes[obj->locate_nodes_cnt++] = node;
			}
		}
	}
	return obj;
}

VoxelSpaceObject_t* voxelspaceUpdateEx(VoxelSpace_t* vs, VoxelSpaceObject_t* obj, const CCTNum_t boundbox_min_v[3], const CCTNum_t boundbox_max_v[3], const void* geo_data, int geo_type, int(*fn_check_intersect)(const void*, int, const CCTNum_t[3], const CCTNum_t[3])) {
	size_t i, x, y, z;
	size_t start_idx[3], end_idx[3];
	node_range_indices(vs, boundbox_min_v, boundbox_max_v, start_idx, end_idx);
	if (!voxelspace_update_prepare_memory(vs, obj, start_idx, end_idx)) {
		return NULL;
	}

	for (i = 0; i < obj->locate_nodes_cnt; ++i) {
		node_remove_obj(obj->locate_nodes[i], obj);
	}
	obj->locate_nodes_cnt = 0;
	for (x = start_idx[0]; x < end_idx[0]; ++x) {
		for (y = start_idx[1]; y < end_idx[1]; ++y) {
			for (z = start_idx[2]; z < end_idx[2]; ++z) {
				VoxelSpaceNode_t* node;
				CCTNum_t voxel_min_v[3], voxel_max_v[3];
				voxelspaceNodeBoundingBox(vs, x, y, z, voxel_min_v, voxel_max_v);
				if (!fn_check_intersect(geo_data, geo_type, voxel_min_v, voxel_max_v)) {
					continue;
				}
				node = vs->nodes + voxelspaceNodeIndexFromXYZ(vs, x, y, z);
				node->objs[node->objs_cnt++] = obj;
				obj->locate_nodes[obj->locate_nodes_cnt++] = node;
			}
		}
	}
	return obj;
}

void voxelspaceRemove(VoxelSpaceObject_t* obj) {
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

VoxelSpaceFinder_t* voxelspaceFindBegin(const VoxelSpace_t* vs, const CCTNum_t min_v[3], const CCTNum_t max_v[3], VoxelSpaceFinder_t* finder) {
	node_range_indices(vs, min_v, max_v, finder->_start_idx, finder->_end_idx);
	finder->_vs = vs;
	finder->_cur_idx[0] = finder->_start_idx[0];
	finder->_cur_idx[1] = finder->_start_idx[1];
	finder->_cur_idx[2] = finder->_start_idx[2];
	finder->_node_idx = voxelspaceNodeIndexFromXYZ(vs, finder->_cur_idx[0], finder->_cur_idx[1], finder->_cur_idx[2]);
	if (finder->_start_idx[0] >= finder->_end_idx[0]) {
		return NULL;
	}
	if (finder->_start_idx[1] >= finder->_end_idx[1]) {
		return NULL;
	}
	if (finder->_start_idx[2] >= finder->_end_idx[2]) {
		return NULL;
	}
	return finder;
}

VoxelSpaceFinder_t* voxelspaceFindNext(VoxelSpaceFinder_t* finder) {
	const VoxelSpace_t* vs = finder->_vs;
	++finder->_cur_idx[2];
	if (finder->_cur_idx[2] < finder->_end_idx[2]) {
		finder->_node_idx = voxelspaceNodeIndexFromXYZ(vs, finder->_cur_idx[0], finder->_cur_idx[1], finder->_cur_idx[2]);
		return finder;
	}
	finder->_cur_idx[2] = finder->_start_idx[2];
	++finder->_cur_idx[1];
	if (finder->_cur_idx[1] < finder->_end_idx[1]) {
		finder->_node_idx = voxelspaceNodeIndexFromXYZ(vs, finder->_cur_idx[0], finder->_cur_idx[1], finder->_cur_idx[2]);
		return finder;
	}
	finder->_cur_idx[1] = finder->_start_idx[1];
	++finder->_cur_idx[0];
	if (finder->_cur_idx[0] < finder->_end_idx[0]) {
		finder->_node_idx = voxelspaceNodeIndexFromXYZ(vs, finder->_cur_idx[0], finder->_cur_idx[1], finder->_cur_idx[2]);
		return finder;
	}
	return NULL;
}

void voxelspaceClear(VoxelSpace_t* vs) {
	size_t i, j, nodes_cnt;
	nodes_cnt = vs->_dimension_node_max_sz[0] * vs->_dimension_node_max_sz[1] * vs->_dimension_node_max_sz[2];
	for (i = 0; i < nodes_cnt; ++i) {
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
	size_t i, j, nodes_cnt;
	if (!vs->nodes) {
		return;
	}
	nodes_cnt = vs->_dimension_node_max_sz[0] * vs->_dimension_node_max_sz[1] * vs->_dimension_node_max_sz[2];
	for (i = 0; i < nodes_cnt; ++i) {
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
	if (vs->_alloc_nodes) {
		free(vs->nodes);
	}
	vs->nodes = NULL;
}

#ifdef __cplusplus
}
#endif
