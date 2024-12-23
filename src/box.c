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

const unsigned int Box_Edge_VertexIndices[24] = {
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

const unsigned int Box_Face_Vertice_Indices[6][4] = {
	{ 1, 2, 6, 5 },
	{ 0, 3, 7, 4 },
	{ 3, 2, 6, 7 },
	{ 0, 1, 5, 4 },
	{ 4, 5, 6, 7 },
	{ 0, 1, 2, 3 }
};

const unsigned int Box_Face_Edge_VertexIndices[6][8] = {
	{ 1,2, 2,6, 6,5, 5,1 },
	{ 0,3, 3,7, 7,4, 4,0 },
	{ 3,2, 2,6, 6,7, 7,3 },
	{ 0,1, 1,5, 5,4, 4,0 },
	{ 4,5, 5,6, 6,7, 7,4 },
	{ 0,1, 1,2, 2,3, 3,0 }
};

const unsigned int Box_Face_MeshEdge_Index[6][4] = {
	{ 1,9,5,8 },
	{ 3,10,7,11 },
	{ 2,9,4,10 },
	{ 0,8,6,11 },
	{ 6,5,4,7 },
	{ 0,1,2,3 }
};

const unsigned int Box_Edge_Adjacent_FaceIndex[24] = {
	5,3,	5,0,	5,2,	5,1,	4,2,	4,0,
	4,3,	4,1,	3,0,	2,0,	2,1,	3,1
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
		indices[0] = Box_Edge_VertexIndices[idx];
		indices[1] = Box_Edge_VertexIndices[idx + 1];
		return indices;
	}
	return Box_Edge_VertexIndices + idx;
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
	pi = Box_Face_Vertice_Indices[face_idx];
	if (indices) {
		indices[0] = pi[0];
		indices[1] = pi[1];
		indices[2] = pi[2];
		indices[3] = pi[3];
		return indices;
	}
	return pi;
}

const unsigned int* mathBoxEdgeAdjacentFaces(unsigned int edge_idx, unsigned int adjacent_face_idx[2]) {
	unsigned int idx;
	if (edge_idx >= 12) {
		return NULL;
	}
	idx = edge_idx + edge_idx;
	if (adjacent_face_idx) {
		adjacent_face_idx[0] = Box_Edge_Adjacent_FaceIndex[idx];
		adjacent_face_idx[1] = Box_Edge_Adjacent_FaceIndex[idx + 1];
		return adjacent_face_idx;
	}
	return Box_Edge_Adjacent_FaceIndex + idx;
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

CCTNum_t* mathBoxVertex(const CCTNum_t o[3], const CCTNum_t half[3], const CCTNum_t axis[3][3], unsigned int idx, CCTNum_t v[3]) {
	CCTNum_t AX[3][3];
	if (idx >= 8) {
		return NULL;
	}
	mathVec3Copy(v, o);
	mathVec3MultiplyScalar(AX[0], axis[0], half[0]);
	mathVec3MultiplyScalar(AX[1], axis[1], half[1]);
	mathVec3MultiplyScalar(AX[2], axis[2], half[2]);
	switch (idx) {
		case 0:
			mathVec3Sub(v, v, AX[0]);
			mathVec3Sub(v, v, AX[1]);
			mathVec3Sub(v, v, AX[2]);
			return v;
		case 1:
			mathVec3Add(v, v, AX[0]);
			mathVec3Sub(v, v, AX[1]);
			mathVec3Sub(v, v, AX[2]);
			return v;
		case 2:
			mathVec3Add(v, v, AX[0]);
			mathVec3Add(v, v, AX[1]);
			mathVec3Sub(v, v, AX[2]);
			return v;
		case 3:
			mathVec3Sub(v, v, AX[0]);
			mathVec3Add(v, v, AX[1]);
			mathVec3Sub(v, v, AX[2]);
			return v;
		case 4:
			mathVec3Sub(v, v, AX[0]);
			mathVec3Sub(v, v, AX[1]);
			mathVec3Add(v, v, AX[2]);
			return v;
		case 5:
			mathVec3Add(v, v, AX[0]);
			mathVec3Sub(v, v, AX[1]);
			mathVec3Add(v, v, AX[2]);
			return v;
		case 6:
			mathVec3Add(v, v, AX[0]);
			mathVec3Add(v, v, AX[1]);
			mathVec3Add(v, v, AX[2]);
			return v;
		case 7:
			mathVec3Sub(v, v, AX[0]);
			mathVec3Add(v, v, AX[1]);
			mathVec3Add(v, v, AX[2]);
			return v;
	}
	return NULL;
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
	mathVec3Add(polygon->center, v[Box_Face_Vertice_Indices[face_idx][0]], v[Box_Face_Vertice_Indices[face_idx][2]]);
	mathVec3MultiplyScalar(polygon->center, polygon->center, CCTNum(0.5));
	polygon->v = (CCTNum_t(*)[3])v;
	polygon->v_indices = Box_Face_Vertice_Indices[face_idx];
	polygon->v_indices_cnt = sizeof(Box_Face_Vertice_Indices[0]) / sizeof(Box_Face_Vertice_Indices[0][0]);
	polygon->edge_v_indices = Box_Face_Edge_VertexIndices[face_idx];
	polygon->edge_v_indices_cnt = sizeof(Box_Face_Edge_VertexIndices[0]) / sizeof(Box_Face_Edge_VertexIndices[0][0]);
	polygon->mesh_edge_index = Box_Face_MeshEdge_Index[face_idx];
	polygon->tri_v_indices = NULL;
	polygon->tri_v_indices_cnt = 0;
	polygon->is_convex = 1;
	return polygon;
}

void mathBoxMesh(GeometryBoxMesh_t* bm, const CCTNum_t center[3], const CCTNum_t half[3], const CCTNum_t axis[3][3]) {
	unsigned int i;
	CCTNum_t min_v[3], max_v[3];
	GeometryMesh_t* mesh = &bm->mesh;
	const CCTNum_t(*v)[3] = (const CCTNum_t(*)[3])bm->v;

	mathBoxVertices(center, half, axis, bm->v);
	mesh->v = bm->v;
	mesh->v_indices = Box_Vertice_Indices_Default;
	mesh->v_indices_cnt = sizeof(Box_Vertice_Indices_Default) / sizeof(Box_Vertice_Indices_Default[0]);
	mesh->edge_v_indices = Box_Edge_VertexIndices;
	mesh->edge_v_indices_cnt = sizeof(Box_Edge_VertexIndices) / sizeof(Box_Edge_VertexIndices[0]);
	mesh->is_convex = 1;
	mesh->is_closed = 1;
	mesh->polygons = bm->faces;
	mesh->polygons_cnt = sizeof(bm->faces) / sizeof(bm->faces[0]);
	mathVerticesFindMinMaxXYZ(v, 8, min_v, max_v);
	mathAABBFromTwoVertice(min_v, max_v, mesh->bound_box.o, mesh->bound_box.half);
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		mathBoxFace(v, axis, i, mesh->polygons + i);
	}
}

#ifdef __cplusplus
}
#endif
