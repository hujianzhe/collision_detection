//
// Created by hujianzhe
//

#include "../inc/box.h"
#include "../inc/const_data.h"
#include "../inc/math_vec3.h"
#include "../inc/vertex.h"
#include "../inc/aabb.h"

extern const CCTConstVal_t CCTConstVal_;

#ifdef __cplusplus
extern "C" {
#endif

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

static void box_mesh_fill_common_fields(GeometryBoxMesh_t* bm) {
	GeometryMesh_t* mesh = &bm->mesh;
	mesh->v = bm->v;
	mesh->v_indices = CCTConstVal_.Box_Vertices_Indices;
	mesh->v_indices_cnt = 8;
	mesh->v_adjacent_infos = Box_Vertex_AdjacentInfos;
	mesh->edge_adjacent_face_ids_flat = CCTConstVal_.Box_Edge_Adjacent_FaceIds;
	mesh->edge_v_ids_flat = CCTConstVal_.Box_Edge_VertexIds;
	mesh->edge_v_indices_flat = CCTConstVal_.Box_Edge_VertexIds;
	mesh->edge_cnt = 12;
	mesh->is_convex = 1;
	mesh->is_closed = 1;
	mesh->polygons = bm->faces;
	mesh->polygons_cnt = 6;
}

static CCTNum_t* box_face_normal(const CCTNum_t axis[3][3], unsigned int face_id, CCTNum_t normal[3]) {
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

const unsigned int* mathBoxEdgeVertexIds(unsigned int edge_id, unsigned int v_ids[2]) {
	unsigned int idx;
	if (edge_id >= 12) {
		return NULL;
	}
	idx = edge_id + edge_id;
	if (v_ids) {
		v_ids[0] = CCTConstVal_.Box_Edge_VertexIds[idx];
		v_ids[1] = CCTConstVal_.Box_Edge_VertexIds[idx + 1];
		return v_ids;
	}
	return CCTConstVal_.Box_Edge_VertexIds + idx;
}

const unsigned int* mathBoxVertexAdjacentVertexIds(unsigned int v_id, unsigned int adj_v_ids[3]) {
	const unsigned int* pi;
	if (v_id >= 8) {
		return NULL;
	}
	pi = &CCTConstVal_.Box_Vertex_Adjacent_VertexIds[v_id * 3];
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
	pi = &CCTConstVal_.Box_Face_MeshVertexIds[face_id * 4];
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
		adjacent_face_ids[0] = CCTConstVal_.Box_Edge_Adjacent_FaceIds[idx];
		adjacent_face_ids[1] = CCTConstVal_.Box_Edge_Adjacent_FaceIds[idx + 1];
		return adjacent_face_ids;
	}
	return CCTConstVal_.Box_Edge_Adjacent_FaceIds + idx;
}

void mathAABBFixSize(CCTNum_t min_v[3], CCTNum_t max_v[3]) {
	const CCTNum_t min_sz = GEOMETRY_BODY_BOX_MIN_HALF + GEOMETRY_BODY_BOX_MIN_HALF;
	if (max_v[0] - min_v[0] < min_sz) {
		max_v[0] += GEOMETRY_BODY_BOX_MIN_HALF;
		min_v[0] -= GEOMETRY_BODY_BOX_MIN_HALF;
	}
	if (max_v[1] - min_v[1] < min_sz) {
		max_v[1] += GEOMETRY_BODY_BOX_MIN_HALF;
		min_v[1] -= GEOMETRY_BODY_BOX_MIN_HALF;
	}
	if (max_v[2] - min_v[2] < min_sz) {
		max_v[2] += GEOMETRY_BODY_BOX_MIN_HALF;
		min_v[2] -= GEOMETRY_BODY_BOX_MIN_HALF;
	}
}

void mathBoxFixAxis3(CCTNum_t axis0[3], CCTNum_t axis1[3], CCTNum_t axis2[3]) {
	CCTNum_t new_axis1[3], new_axis2[3];
	mathVec3Normalized(axis0, axis0);
	mathVec3Cross(new_axis2, axis0, axis1);
	if (mathVec3Dot(new_axis2, axis2) < CCTNum(0.0)) {
		mathVec3Negate(new_axis2, new_axis2);
	}
	mathVec3Normalized(axis2, new_axis2);
	mathVec3Cross(new_axis1, axis0, axis2);
	if (mathVec3Dot(new_axis1, axis1) < CCTNum(0.0)) {
		mathVec3Negate(new_axis1, new_axis1);
	}
	mathVec3Normalized(axis1, new_axis1);
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

GeometryPolygon_t* mathBoxFace(const CCTNum_t v[8][3], const CCTNum_t axis[3][3], unsigned int face_id, GeometryPolygon_t* polygon) {
	static const GeometryPolygonVertexAdjacentInfo_t Box_Face_VertexAdjacentInfos[4] = {
		{ {1,3}, {0,3} },
		{ {2,0}, {1,0} },
		{ {3,1}, {2,1} },
		{ {0,2}, {3,2} },
	};

	if (!box_face_normal(axis, face_id, polygon->normal)) {
		return NULL;
	}
	mathVec3Midpoint(polygon->center, v[CCTConstVal_.Box_Face_MeshVertexIds[face_id * 4]], v[CCTConstVal_.Box_Face_MeshVertexIds[face_id * 4 + 2]]);
	polygon->v = (CCTNum_t(*)[3])v;
	polygon->v_indices = &CCTConstVal_.Box_Face_MeshVertexIds[face_id * 4];
	polygon->v_indices_cnt = 4;
	polygon->edge_v_ids_flat = CCTConstVal_.Rect_Edge_VertexIds;
	polygon->edge_v_indices_flat = &CCTConstVal_.Box_Face_Edge_MeshVertexIds[face_id * 8];
	polygon->edge_cnt = 4;
	polygon->mesh_v_ids = &CCTConstVal_.Box_Face_MeshVertexIds[face_id * 4];
	polygon->mesh_edge_ids = &CCTConstVal_.Box_Face_MeshEdgeIds[face_id * 4];
	polygon->v_adjacent_infos = Box_Face_VertexAdjacentInfos;
	polygon->tri_v_indices_flat = &CCTConstVal_.Box_Triangle_Vertices_Indices[face_id * 6];
	polygon->concave_tri_v_ids_flat = NULL;
	polygon->concave_tri_edge_ids_flat = NULL;
	polygon->tri_cnt = 2;
	polygon->is_convex = 1;
	return polygon;
}

void mathBoxMesh(GeometryBoxMesh_t* bm, const CCTNum_t center[3], const CCTNum_t half[3], const CCTNum_t axis[3][3]) {
	unsigned int i;
	GeometryMesh_t* mesh = &bm->mesh;
	const CCTNum_t(*v)[3] = (const CCTNum_t(*)[3])bm->v;
	box_mesh_fill_common_fields(bm);
	mathBoxVertices(center, half, axis, bm->v);
	mathVerticesFindMinMaxXYZ(v, 8, mesh->bound_box.min_v, mesh->bound_box.max_v);
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		mathBoxFace(v, axis, i, mesh->polygons + i);
	}
}

void mathAABBMesh(GeometryBoxMesh_t* bm, const CCTNum_t min_v[3], const CCTNum_t max_v[3]) {
	unsigned int i;
	GeometryMesh_t* mesh = &bm->mesh;
	const CCTNum_t(*v)[3] = (const CCTNum_t(*)[3])bm->v;
	box_mesh_fill_common_fields(bm);
	mathAABBVertices(min_v, max_v, bm->v);
	mathVec3Copy(mesh->bound_box.min_v, min_v);
	mathVec3Copy(mesh->bound_box.max_v, max_v);
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		mathBoxFace(v, CCTConstVal_.AABB_Axis, i, mesh->polygons + i);
	}
}

#ifdef __cplusplus
}
#endif
