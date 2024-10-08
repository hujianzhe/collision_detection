//
// Created by hujianzhe
//

#include "../inc/box.h"
#include "../inc/math_vec3.h"
#include "../inc/vertex.h"
#include "../inc/aabb.h"

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
	{ 1, 2, 6, 5 },
	{ 0, 3, 7, 4 },
	{ 3, 2, 6, 7 },
	{ 0, 1, 5, 4 },
	{ 4, 5, 6, 7 },
	{ 0, 1, 2, 3 }
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

void mathBoxVertices(const CCTNum_t o[3], const CCTNum_t half[3], const CCTNum_t axis[3][3], CCTNum_t v[8][3]) {
	CCTNum_t AX[3][3];
	mathVec3MultiplyScalar(AX[0], axis[0], half[0]);
	mathVec3MultiplyScalar(AX[1], axis[1], half[1]);
	mathVec3MultiplyScalar(AX[2], axis[2], half[2]);

	mathVec3Copy(v[0], o);
	mathVec3Sub(v[0], v[0], AX[0]);
	mathVec3Sub(v[0], v[0], AX[1]);
	mathVec3Sub(v[0], v[0], AX[2]);

	mathVec3Copy(v[1], o);
	mathVec3Add(v[1], v[1], AX[0]);
	mathVec3Sub(v[1], v[1], AX[1]);
	mathVec3Sub(v[1], v[1], AX[2]);

	mathVec3Copy(v[2], o);
	mathVec3Add(v[2], v[2], AX[0]);
	mathVec3Add(v[2], v[2], AX[1]);
	mathVec3Sub(v[2], v[2], AX[2]);

	mathVec3Copy(v[3], o);
	mathVec3Sub(v[3], v[3], AX[0]);
	mathVec3Add(v[3], v[3], AX[1]);
	mathVec3Sub(v[3], v[3], AX[2]);

	mathVec3Copy(v[4], o);
	mathVec3Sub(v[4], v[4], AX[0]);
	mathVec3Sub(v[4], v[4], AX[1]);
	mathVec3Add(v[4], v[4], AX[2]);

	mathVec3Copy(v[5], o);
	mathVec3Add(v[5], v[5], AX[0]);
	mathVec3Sub(v[5], v[5], AX[1]);
	mathVec3Add(v[5], v[5], AX[2]);

	mathVec3Copy(v[6], o);
	mathVec3Add(v[6], v[6], AX[0]);
	mathVec3Add(v[6], v[6], AX[1]);
	mathVec3Add(v[6], v[6], AX[2]);

	mathVec3Copy(v[7], o);
	mathVec3Sub(v[7], v[7], AX[0]);
	mathVec3Add(v[7], v[7], AX[1]);
	mathVec3Add(v[7], v[7], AX[2]);
}

CCTNum_t* mathBoxFaceNormal(const CCTNum_t axis[3][3], unsigned int face_idx, CCTNum_t normal[3]) {
	if (face_idx < 2) {
		if (0 == face_idx) {
			return mathVec3Copy(normal, axis[0]);
		}
		else {
			return mathVec3Negate(normal, axis[0]);
		}
	}
	else if (face_idx < 4) {
		if (2 == face_idx) {
			return mathVec3Copy(normal, axis[1]);
		}
		else {
			return mathVec3Negate(normal, axis[1]);
		}
	}
	else if (4 == face_idx) {
		return mathVec3Copy(normal, axis[2]);
	}
	else if (5 == face_idx) {
		return mathVec3Negate(normal, axis[2]);
	}
	return NULL;
}

GeometryPolygon_t* mathBoxFace(const CCTNum_t v[8][3], const CCTNum_t axis[3][3], unsigned int face_idx, GeometryPolygon_t* polygon) {
	if (!mathBoxFaceNormal(axis, face_idx, polygon->normal)) {
		return NULL;
	}
	mathVec3Add(polygon->center, v[Box_Face_Indices[face_idx][0]], v[Box_Face_Indices[face_idx][2]]);
	mathVec3MultiplyScalar(polygon->center, polygon->center, CCTNum(0.5));
	mathVec3Set(polygon->o, CCTNums_3(0.0, 0.0, 0.0));
	polygon->v = (CCTNum_t(*)[3])v;
	polygon->v_indices = Box_Face_Indices[face_idx];
	polygon->v_indices_cnt = sizeof(Box_Face_Indices[0]) / sizeof(Box_Face_Indices[0][0]);
	polygon->tri_indices = NULL;
	polygon->tri_indices_cnt = 0;
	polygon->is_convex = 1;
	return polygon;
}

void mathBoxMesh(GeometryBoxMesh_t* bm, const CCTNum_t v[8][3], const CCTNum_t axis[3][3]) {
	unsigned int i;
	CCTNum_t min_v[3], max_v[3];
	GeometryMesh_t* mesh = &bm->mesh;

	mesh->v = (CCTNum_t(*)[3])v;
	mesh->v_indices = Box_Vertice_Indices_Default;
	mesh->v_indices_cnt = sizeof(Box_Vertice_Indices_Default) / sizeof(Box_Vertice_Indices_Default[0]);
	mesh->edge_indices = Box_Edge_Indices;
	mesh->edge_indices_cnt = sizeof(Box_Edge_Indices) / sizeof(Box_Edge_Indices[0]);
	mesh->edge_stride = 2;
	mesh->is_convex = 1;
	mesh->polygons = bm->faces;
	mesh->polygons_cnt = sizeof(bm->faces) / sizeof(bm->faces[0]);
	mathVerticesFindMinMaxXYZ(v, 8, min_v, max_v);
	mathAABBFromTwoVertice(min_v, max_v, mesh->bound_box.o, mesh->bound_box.half);
	mathVec3Add(mesh->o, v[0], v[6]);
	mathVec3MultiplyScalar(mesh->o, mesh->o, CCTNum(0.5));
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		mathBoxFace(v, axis, i, mesh->polygons + i);
		mathVec3Copy(mesh->polygons[i].o, mesh->o);
	}
}

#ifdef __cplusplus
}
#endif
