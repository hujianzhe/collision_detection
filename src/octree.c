//
// Created by hujianzhe on 21-12-29
//

#include "../inc/math_vec3.h"
#include "../inc/aabb.h"
#include "../inc/octree.h"
#include <stdlib.h>

static void insert_obj_to_list(OctreeNode_t* oct_node, OctreeObject_t* oct_obj) {
	if (oct_node->obj_list_head) {
		oct_node->obj_list_head->prev = oct_obj;
	}
	oct_obj->next = oct_node->obj_list_head;
	oct_obj->prev = NULL;
	oct_node->obj_list_head = oct_obj;
	++oct_node->obj_cnt;
	oct_obj->oct = oct_node;
}

static void del_obj_from_list(OctreeNode_t* oct_node, OctreeObject_t* oct_obj) {
	if (oct_node->obj_list_head == oct_obj) {
		oct_node->obj_list_head = oct_obj->next;
	}
	if (oct_obj->prev) {
		oct_obj->prev->next = oct_obj->next;
	}
	if (oct_obj->next) {
		oct_obj->next->prev = oct_obj->prev;
	}
	--oct_node->obj_cnt;
	oct_obj->oct = NULL;
}

static size_t octree_level_nodes_cnt(unsigned int deep_num) {
	size_t cnt = 1;
	if (deep_num > 1) {
		size_t i;
		deep_num--;
		for (i = 0; i < deep_num; ++i) {
			cnt *= 8;
		}
	}
	return cnt;
}

static size_t octree_total_nodes_cnt(unsigned int max_deep_num) {
	size_t total_cnt = 0;
	size_t i;
	for (i = 1; i <= max_deep_num; ++i) {
		total_cnt += octree_level_nodes_cnt(i);
	}
	return total_cnt;
}

static void octree_node_init(OctreeNode_t* root, const CCTNum_t pos[3], const CCTNum_t half[3]) {
	mathVec3Sub(root->min_v, pos, half);
	mathVec3Add(root->max_v, pos, half);
	root->obj_list_head = NULL;
	root->obj_cnt = 0;
	root->deep_num = 0;
	root->parent = NULL;
	root->childs = NULL;
}

static void octree_node_split(Octree_t* tree, OctreeNode_t* root) {
	int i;
	OctreeObject_t* obj, *obj_next;
	CCTNum_t new_o[8][3], new_min_v[3], new_max_v[3], root_pos[3], new_half[3];

	if (root->childs) {
		return;
	}
	root->childs = &tree->nodes[1 + ((root - tree->nodes) << 3)];

	mathVec3Sub(new_half, root->max_v, root->min_v);
	mathVec3MultiplyScalar(new_half, new_half, CCTNum(0.5));
	mathVec3Add(root_pos, root->min_v, new_half);
	mathVec3MultiplyScalar(new_half, new_half, CCTNum(0.5));
	mathVec3Sub(new_min_v, root_pos, new_half);
	mathVec3Add(new_max_v, root_pos, new_half);

	mathAABBVertices(new_min_v, new_max_v, new_o);
	for (i = 0; i < 8; ++i) {
		OctreeNode_t* child = root->childs + i;
		octree_node_init(child, new_o[i], new_half);
		child->parent = root;
		child->deep_num = root->deep_num + 1;
	}
	for (obj = root->obj_list_head; obj; obj = obj_next) {
		obj_next = obj->next;
		for (i = 0; i < 8; ++i) {
			OctreeNode_t* child = root->childs + i;
			if (!AABB_Contain_AABB(child->min_v, child->max_v, obj->min_v, obj->max_v)) {
				continue;
			}
			del_obj_from_list(root, obj);
			insert_obj_to_list(child, obj);
			break;
		}
	}
}

#ifdef __cplusplus
extern "C" {
#endif

unsigned int octreeCalculateDeepNumByCellSize(const CCTNum_t half_size[3], CCTNum_t cell_size) {
	CCTNum_t min_half_value;
	if (half_size[0] < half_size[1]) {
		if (half_size[0] < half_size[2]) {
			min_half_value = half_size[0];
		}
		else {
			min_half_value = half_size[2];
		}
	}
	else if (half_size[1] < half_size[2]) {
		min_half_value = half_size[1];
	}
	else {
		min_half_value = half_size[2];
	}
	if (min_half_value <= cell_size) {
		return 1;
	}
	return CCTNum_floor(CCTNum_log(min_half_value / cell_size, CCTNum(2.0)) + CCTNum(1.0));
}

Octree_t* octreeInit(Octree_t* tree, const CCTNum_t pos[3], const CCTNum_t half[3], unsigned int max_deep_num, unsigned int split_cnt_per_node) {
	OctreeNode_t* nodes;
	size_t nodes_cnt;

	if (max_deep_num <= 0 || split_cnt_per_node <= 0) {
		return NULL;
	}
	nodes_cnt = octree_total_nodes_cnt(max_deep_num);
	nodes = (OctreeNode_t*)calloc(1, sizeof(OctreeNode_t) * nodes_cnt);
	if (!nodes) {
		return NULL;
	}
	tree->nodes_cnt = nodes_cnt;
	tree->nodes = nodes;
	octree_node_init(&tree->nodes[0], pos, half);
	tree->nodes[0].deep_num = 1;
	tree->max_deep_num = max_deep_num;
	tree->split_cnt_per_node = split_cnt_per_node;
	return tree;
}

void octreeRemove(OctreeObject_t* obj) {
	OctreeNode_t* oct = obj->oct;
	if (!oct) {
		return;
	}
	del_obj_from_list(oct, obj);
}

void octreeUpdate(Octree_t* tree, OctreeObject_t* obj) {
	OctreeNode_t* root = &tree->nodes[0];
	OctreeNode_t* obj_oct = obj->oct;
	OctreeNode_t* oct = obj_oct ? obj_oct : root;
	while (oct) {
		if (oct->childs) {
			int i, find = 0;
			for (i = 0; i < 8; ++i) {
				OctreeNode_t* child = oct->childs + i;
				if (!AABB_Contain_AABB(child->min_v, child->max_v, obj->min_v, obj->max_v)) {
					continue;
				}
				if (child->childs) {
					oct = child;
					break;
				}
				if (obj_oct) {
					del_obj_from_list(obj_oct, obj);
				}
				insert_obj_to_list(child, obj);
				find = 1;
				break;
			}
			if (find) {
				break;
			}
			if (i != 8) {
				continue;
			}
		}
		if (AABB_Contain_AABB(oct->min_v, oct->max_v, obj->min_v, obj->max_v)) {
			if (oct == obj_oct) {
				return;
			}
			if (obj_oct) {
				del_obj_from_list(obj_oct, obj);
			}
			insert_obj_to_list(oct, obj);
			break;
		}
		oct = oct->parent;
	}
	if (!oct) {
		if (obj_oct == root) {
			return;
		}
		if (obj_oct) {
			del_obj_from_list(obj_oct, obj);
		}
		insert_obj_to_list(root, obj);
		return;
	}
	if (obj->oct->obj_cnt > tree->split_cnt_per_node && obj->oct->deep_num < tree->max_deep_num) {
		octree_node_split(tree, obj->oct);
	}
}

OctreeFinder_t* octreeFinderAlloc(const Octree_t* tree, OctreeFinder_t* finder) {
	if (finder->cap < tree->nodes_cnt) {
		const OctreeNode_t** p = (const OctreeNode_t**)realloc(finder->nodes, sizeof(OctreeNode_t*) * tree->nodes_cnt);
		if (!p) {
			return NULL;
		}
		finder->nodes = p;
		finder->cap = tree->nodes_cnt;
	}
	finder->cnt = 0;
	return finder;
}

void octreeFinderDestroy(OctreeFinder_t* finder) {
	if (finder->nodes) {
		free(finder->nodes);
		finder->nodes = NULL;
		finder->cap = 0;
	}
	finder->cnt = 0;
}

void octreeFindNodes(const Octree_t* tree, const CCTNum_t min_v[3], const CCTNum_t max_v[3], OctreeFinder_t* finder) {
	size_t pop_idx;
	const OctreeNode_t* root = tree->nodes;
	finder->cnt = 0;
	if (!mathAABBIntersectAABB(root->min_v, root->max_v, min_v, max_v)) {
		return;
	}
	finder->nodes[finder->cnt++] = root;
	pop_idx = 0;
	while (pop_idx < finder->cnt) {
		int i;
		const OctreeNode_t* oct = finder->nodes[pop_idx++];
		if (!oct->childs) {
			continue;
		}
		for (i = 0; i < 8; ++i) {
			const OctreeNode_t* child = oct->childs + i;
			if (!mathAABBIntersectAABB(child->min_v, child->max_v, min_v, max_v)) {
				continue;
			}
			finder->nodes[finder->cnt++] = child;
		}
	}
}

void octreeClear(Octree_t* tree) {
	size_t i;
	for (i = 0; i < tree->nodes_cnt; ++i) {
		OctreeNode_t* node = tree->nodes + i;
		OctreeObject_t* obj;
		for (obj = node->obj_list_head; obj; obj = obj->next) {
			obj->oct = NULL;
		}
		node->obj_list_head = NULL;
		node->obj_cnt = 0;
		node->parent = node->childs = NULL;
	}
}

void octreeDestroy(Octree_t* tree) {
	size_t i;
	for (i = 0; i < tree->nodes_cnt; ++i) {
		OctreeNode_t* node = tree->nodes + i;
		OctreeObject_t* obj;
		for (obj = node->obj_list_head; obj; obj = obj->next) {
			obj->oct = NULL;
		}
	}
	free(tree->nodes);
	tree->nodes = NULL;
	tree->nodes_cnt = 0;
}

#ifdef __cplusplus
}
#endif
