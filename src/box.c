//
// Created by hujianzhe
//

#include "../inc/box.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

const unsigned int Box_Vertice_Indices_Default[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };

const unsigned int Box_Edge_Indices[24] = {
	0, 1,	1, 2,	2, 3,	3, 0,
	7, 6,	6, 5,	5, 4,	4, 7,
	1, 5,	6, 2,
	3, 7,	4, 0
};

const unsigned int Box_Vertice_Indices_Adjacent[8][3] = {
	{ 1, 3, 4 },
	{ 0, 2, 5 },
	{ 3, 1, 6 },
	{ 2, 0, 7 },
	{ 5, 7, 0 },
	{ 4, 6, 1 },
	{ 7, 5, 2 },
	{ 6, 4, 3 }
};

const unsigned int Box_Face_Indices[6][4] = {
	{ 4, 5, 6, 7 },
	{ 0, 1, 2, 3 },
	{ 1, 2, 6, 5 },
	{ 0, 3, 7, 4 },
	{ 3, 2, 6, 7 },
	{ 0, 1, 5, 4 }
};

/*
const unsigned int Box_Triangle_Vertices_Indices[36] = {
	0, 1, 2,	2, 3, 0,
	7, 6, 5,	5, 4, 7,
	1, 5, 6,	6, 2, 1,
	3, 7, 4,	4, 0, 3,
	3, 7, 6,	6, 2, 3,
	0, 4, 5,	5, 1, 0
};
*/

const unsigned int* mathBoxEdgeVertexIndices(unsigned int edge_idx, unsigned int indices[2]) {
	unsigned int idx;
	if (edge_idx >= 12) {
		return NULL;
	}
	idx = edge_idx + edge_idx;
	if (indices) {
		indices[0] = Box_Edge_Indices[idx];
		indices[1] = Box_Edge_Indices[idx + 1];
		return indices;
	}
	return Box_Edge_Indices + idx;
}

const unsigned int* mathBoxVertexIndicesAdjacent(unsigned int indices, unsigned int adj_indices[3]) {
	const unsigned int* pi;
	if (indices >= 8) {
		return NULL;
	}
	pi = Box_Vertice_Indices_Adjacent[indices];
	if (adj_indices) {
		adj_indices[0] = pi[0];
		adj_indices[1] = pi[1];
		adj_indices[2] = pi[2];
		return adj_indices;
	}
	return pi;
}

const unsigned int* mathBoxFaceVertexIndices(unsigned int face_idx, unsigned int indices[4]) {
	const unsigned int* pi;
	if (face_idx >= 6) {
		return NULL;
	}
	pi = Box_Face_Indices[face_idx];
	if (indices) {
		indices[0] = pi[0];
		indices[1] = pi[1];
		indices[2] = pi[2];
		indices[3] = pi[3];
		return indices;
	}
	return pi;
}

#ifdef __cplusplus
}
#endif
