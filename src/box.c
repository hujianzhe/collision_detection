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

const unsigned int Box_Face_MeshVerticeIds[6][4] = {
	{ 1, 2, 6, 5 },
	{ 0, 3, 7, 4 },
	{ 3, 2, 6, 7 },
	{ 0, 1, 5, 4 },
	{ 4, 5, 6, 7 },
	{ 0, 1, 2, 3 }
};

static const unsigned int Box_Edge_VertexIds[24] = {
	0, 1,	1, 2,	2, 3,	3, 0,
	7, 6,	6, 5,	5, 4,	4, 7,
	1, 5,	6, 2,
	3, 7,	4, 0
};

static const unsigned int Box_Vertex_Adjacent_VertexIds[8][3] = {
	{ 1, 3, 4 },
	{ 0, 2, 5 },
	{ 3, 1, 6 },
	{ 2, 0, 7 },
	{ 5, 7, 0 },
	{ 4, 6, 1 },
	{ 7, 5, 2 },
	{ 6, 4, 3 }
};

static const unsigned int Box_Vertex_Adjacent_EdgeIds[8][3] = {
	{ 0, 3, 11 },
	{ 0, 1, 8 },
	{ 2, 1, 9 },
	{ 2, 3, 10 },
	{ 6, 7, 11 },
	{ 6, 5, 8 },
	{ 4, 5, 9 },
	{ 4, 7, 10 }
};

static const unsigned int Box_Vertex_Adjacent_FaceIds[8][3] = {
	{ 5, 1, 3 },
	{ 5, 0, 3 },
	{ 2, 0, 5 },
	{ 1, 2, 5 },
	{ 1, 3, 4 },
	{ 0, 3, 4 },
	{ 0, 2, 4 },
	{ 1, 2, 4 }
};

static const GeometryMeshVertexAdjacentInfo_t Box_Vertex_AdjacentInfos[8] = {
	{ 3,3,3, Box_Vertex_Adjacent_VertexIds[0], Box_Vertex_Adjacent_EdgeIds[0], Box_Vertex_Adjacent_FaceIds[0] },
	{ 3,3,3, Box_Vertex_Adjacent_VertexIds[1], Box_Vertex_Adjacent_EdgeIds[1], Box_Vertex_Adjacent_FaceIds[1] },
	{ 3,3,3, Box_Vertex_Adjacent_VertexIds[2], Box_Vertex_Adjacent_EdgeIds[2], Box_Vertex_Adjacent_FaceIds[2] },
	{ 3,3,3, Box_Vertex_Adjacent_VertexIds[3], Box_Vertex_Adjacent_EdgeIds[3], Box_Vertex_Adjacent_FaceIds[3] },
	{ 3,3,3, Box_Vertex_Adjacent_VertexIds[4], Box_Vertex_Adjacent_EdgeIds[4], Box_Vertex_Adjacent_FaceIds[4] },
	{ 3,3,3, Box_Vertex_Adjacent_VertexIds[5], Box_Vertex_Adjacent_EdgeIds[5], Box_Vertex_Adjacent_FaceIds[5] },
	{ 3,3,3, Box_Vertex_Adjacent_VertexIds[6], Box_Vertex_Adjacent_EdgeIds[6], Box_Vertex_Adjacent_FaceIds[6] },
	{ 3,3,3, Box_Vertex_Adjacent_VertexIds[7], Box_Vertex_Adjacent_EdgeIds[7], Box_Vertex_Adjacent_FaceIds[7] }
};

static const unsigned int Box_Face_Edge_MeshVertexIds[6][8] = {
	{ 1,2, 2,6, 6,5, 5,1 },
	{ 0,3, 3,7, 7,4, 4,0 },
	{ 3,2, 2,6, 6,7, 7,3 },
	{ 0,1, 1,5, 5,4, 4,0 },
	{ 4,5, 5,6, 6,7, 7,4 },
	{ 0,1, 1,2, 2,3, 3,0 }
};

static const unsigned int Box_Face_MeshEdgeIds[6][4] = {
	{ 1,9,5,8 },
	{ 3,10,7,11 },
	{ 2,9,4,10 },
	{ 0,8,6,11 },
	{ 6,5,4,7 },
	{ 0,1,2,3 }
};

static const unsigned int Box_Edge_Adjacent_FaceIds[24] = {
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

const unsigned int* mathBoxEdgeVertexIds(unsigned int edge_id, unsigned int v_ids[2]) {
	unsigned int idx;
	if (edge_id >= 12) {
		return NULL;
	}
	idx = edge_id + edge_id;
	if (v_ids) {
		v_ids[0] = Box_Edge_VertexIds[idx];
		v_ids[1] = Box_Edge_VertexIds[idx + 1];
		return v_ids;
	}
	return Box_Edge_VertexIds + idx;
}

const unsigned int* mathBoxVertexAdjacentVertexIds(unsigned int v_id, unsigned int adj_v_ids[3]) {
	const unsigned int* pi;
	if (v_id >= 8) {
		return NULL;
	}
	pi = Box_Vertex_Adjacent_VertexIds[v_id];
	if (adj_v_ids) {
		adj_v_ids[0] = pi[0];
		adj_v_ids[1] = pi[1];
		adj_v_ids[2] = pi[2];
		return adj_v_ids;
	}
	return pi;
}

const unsigned int* mathBoxFaceMeshVertexIds(unsigned int face_id, unsigned int v_ids[4]) {
	const unsigned int* pi;
	if (face_id >= 6) {
		return NULL;
	}
	pi = Box_Face_MeshVerticeIds[face_id];
	if (v_ids) {
		v_ids[0] = pi[0];
		v_ids[1] = pi[1];
		v_ids[2] = pi[2];
		v_ids[3] = pi[3];
		return v_ids;
	}
	return pi;
}

const unsigned int* mathBoxEdgeAdjacentFaceIds(unsigned int edge_id, unsigned int adjacent_face_ids[2]) {
	unsigned int idx = edge_id + edge_id;
	if (edge_id >= 12) {
		return NULL;
	}
	if (adjacent_face_ids) {
		adjacent_face_ids[0] = Box_Edge_Adjacent_FaceIds[idx];
		adjacent_face_ids[1] = Box_Edge_Adjacent_FaceIds[idx + 1];
		return adjacent_face_ids;
	}
	return Box_Edge_Adjacent_FaceIds + idx;
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

CCTNum_t* mathBoxVertex(const CCTNum_t o[3], const CCTNum_t half[3], const CCTNum_t axis[3][3], unsigned int v_id, CCTNum_t v[3]) {
	CCTNum_t AX[3][3];
	if (v_id >= 8) {
		return NULL;
	}
	mathVec3Copy(v, o);
	mathVec3MultiplyScalar(AX[0], axis[0], half[0]);
	mathVec3MultiplyScalar(AX[1], axis[1], half[1]);
	mathVec3MultiplyScalar(AX[2], axis[2], half[2]);
	switch (v_id) {
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

CCTNum_t* mathBoxFaceNormal(const CCTNum_t axis[3][3], unsigned int face_id, CCTNum_t normal[3]) {
	if (face_id < 2) {
		if (0 == face_id) {
			return mathVec3Copy(normal, axis[0]);
		}
		else {
			return mathVec3Negate(normal, axis[0]);
		}
	}
	else if (face_id < 4) {
		if (2 == face_id) {
			return mathVec3Copy(normal, axis[1]);
		}
		else {
			return mathVec3Negate(normal, axis[1]);
		}
	}
	else if (4 == face_id) {
		return mathVec3Copy(normal, axis[2]);
	}
	else if (5 == face_id) {
		return mathVec3Negate(normal, axis[2]);
	}
	return NULL;
}

GeometryPolygon_t* mathBoxFace(const CCTNum_t v[8][3], const CCTNum_t axis[3][3], unsigned int face_id, GeometryPolygon_t* polygon) {
	static const unsigned int Box_Face_Edge_VertexIds[8] = {
		0,1, 1,2, 2,3, 3,0
	};
	static const GeometryPolygonVertexAdjacentInfo_t Box_Face_VertexAdjacentInfos[4] = {
		{ {1,3}, {0,3} },
		{ {2,0}, {1,0} },
		{ {3,1}, {2,1} },
		{ {0,2}, {3,2} },
	};

	if (!mathBoxFaceNormal(axis, face_id, polygon->normal)) {
		return NULL;
	}
	mathVec3Add(polygon->center, v[Box_Face_MeshVerticeIds[face_id][0]], v[Box_Face_MeshVerticeIds[face_id][2]]);
	mathVec3MultiplyScalar(polygon->center, polygon->center, CCTNum(0.5));
	polygon->v = (CCTNum_t(*)[3])v;
	polygon->v_indices = Box_Face_MeshVerticeIds[face_id];
	polygon->v_indices_cnt = 4;
	polygon->edge_v_ids_flat = Box_Face_Edge_VertexIds;
	polygon->edge_v_indices_flat = Box_Face_Edge_MeshVertexIds[face_id];
	polygon->edge_cnt = 4;
	polygon->mesh_v_ids = Box_Face_MeshVerticeIds[face_id];
	polygon->mesh_edge_ids = Box_Face_MeshEdgeIds[face_id];
	polygon->v_adjacent_infos = Box_Face_VertexAdjacentInfos;
	polygon->tri_v_indices_flat = NULL;
	polygon->concave_tri_v_ids_flat = NULL;
	polygon->concave_tri_edge_ids_flat = NULL;
	polygon->tri_cnt = 0;
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
	mesh->v_indices_cnt = 8;
	mesh->v_adjacent_infos = Box_Vertex_AdjacentInfos;
	mesh->edge_adjacent_face_ids_flat = (const unsigned int*)Box_Edge_Adjacent_FaceIds;
	mesh->edge_v_ids_flat = Box_Edge_VertexIds;
	mesh->edge_v_indices_flat = Box_Edge_VertexIds;
	mesh->edge_cnt = 12;
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
