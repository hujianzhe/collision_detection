//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_BOX_H
#define	UTIL_C_CRT_GEOMETRY_BOX_H

#include "geometry_def.h"

/*
		 3+------+2			0 = ---
		 /|     /|			1 = +--
		/ |    / |			2 = ++-
	   / 0+---/--+1			3 = -+-
	 7+------+6 /       	4 = --+
	  | /    | /     y  	5 = +-+
	  |/     |/      |		6 = +++
	 4+------+5      *--x	7 = -++
                    /
                   z

	face_0 = [1, 2, 6, 5]	// +x
	face_1 = [0, 3, 7, 4]	// -x
	face_2 = [3, 2, 6, 7]	// +y
	face_3 = [0, 1, 5, 4]	// -y
	face_4 = [4, 5, 6, 7]	// +z
	face_5 = [0, 1, 2, 3]	// -z

	edge_0  = [0, 1]
	edge_1  = [1, 2]
	edge_2  = [2, 3]
	edge_3  = [3, 0]
	edge_4  = [7, 6]
	edge_5  = [6, 5]
	edge_6  = [5, 4]
	edge_7  = [4, 7]
	edge_8  = [1, 5]
	edge_9  = [6, 2]
	edge_10 = [3, 7]
	edge_11 = [4, 0]
*/

#define	GEOMETRY_BODY_BOX_MIN_HALF	(CCTNum(1e-4))

#define	GEOMETRY_BODY_BOX_MIN_VERTICE_INDICE	0
#define	GEOMETRY_BODY_BOX_MAX_VERTICE_INDICE	6

typedef struct GeometryBoxMesh_t {
	GeometryMesh_t mesh;
	GeometryPolygon_t faces[6];
	CCTNum_t v[8][3];
} GeometryBoxMesh_t;

#ifdef __cplusplus
extern "C" {
#endif

extern const unsigned int Box_Vertice_Indices_Default[8];
extern const unsigned int Box_Face_MeshVerticeIds[6][4];

__declspec_dll const unsigned int* mathBoxEdgeVertexIds(unsigned int edge_id, unsigned int v_ids[2]);
__declspec_dll const unsigned int* mathBoxVertexAdjacentVertexIds(unsigned int v_id, unsigned int adj_v_ids[3]);
__declspec_dll const unsigned int* mathBoxFaceMeshVertexIds(unsigned int face_id, unsigned int v_ids[4]);
__declspec_dll const unsigned int* mathBoxEdgeAdjacentFaceIds(unsigned int edge_id, unsigned int adjacent_face_ids[2]);

__declspec_dll void mathBoxFixAxis3(CCTNum_t axis0[3], CCTNum_t axis1[3], CCTNum_t axis2[3]);
__declspec_dll void mathBoxVertices(const CCTNum_t o[3], const CCTNum_t half[3], const CCTNum_t axis[3][3], CCTNum_t v[8][3]);
__declspec_dll CCTNum_t* mathBoxVertex(const CCTNum_t o[3], const CCTNum_t half[3], const CCTNum_t axis[3][3], unsigned int v_id, CCTNum_t v[3]);
__declspec_dll CCTNum_t* mathBoxFaceNormal(const CCTNum_t axis[3][3], unsigned int face_id, CCTNum_t normal[3]);
__declspec_dll GeometryPolygon_t* mathBoxFace(const CCTNum_t v[8][3], const CCTNum_t axis[3][3], unsigned int face_id, GeometryPolygon_t* polygon);
__declspec_dll void mathBoxMesh(GeometryBoxMesh_t* bm, const CCTNum_t center[3], const CCTNum_t half[3], const CCTNum_t axis[3][3]);
__declspec_dll void mathAABBMesh(GeometryBoxMesh_t* bm, const CCTNum_t min_v[3], const CCTNum_t max_v[3]);

#ifdef __cplusplus
}
#endif

#endif
