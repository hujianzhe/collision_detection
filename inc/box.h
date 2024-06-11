//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_BOX_H
#define	UTIL_C_CRT_GEOMETRY_BOX_H

#include "geometry_def.h"

/*
		 7+------+6			0 = ---
		 /|     /|			1 = +--
		/ |    / |			2 = ++-
	   / 4+---/--+5			3 = -+-
	 3+------+2 /    y   z	4 = --+
	  | /    | /     |  /	5 = +-+
	  |/     |/      |/		6 = +++
	 0+------+1      *---x	7 = -++

	face_0 = [4, 5, 6, 7]	// +z
	face_1 = [0, 1, 2, 3]	// -z
	face_2 = [1, 2, 6, 5]	// +x
	face_3 = [0, 3, 7, 4]	// -x
	face_4 = [3, 2, 6, 7]	// +y
	face_5 = [0, 1, 5, 4]	// -y
*/

#ifndef GEOMETRY_BODY_BOX_MIN_HALF
	#define	GEOMETRY_BODY_BOX_MIN_HALF	(CCTNum(1e-5) + CCTNum(1e-5))
#endif

#define	GEOMETRY_BODY_BOX_MIN_VERTICE_INDICE	0
#define	GEOMETRY_BODY_BOX_MAX_VERTICE_INDICE	6

typedef struct GeometryBoxMesh_t {
	GeometryMesh_t mesh;
	CCTNum_t v[8][3];
	GeometryPolygon_t faces[6];
} GeometryBoxMesh_t;

typedef struct GeometryBoxFace_t {
	GeometryPolygon_t polygon;
	CCTNum_t v[8][3];
} GeometryBoxFace_t;

#ifdef __cplusplus
extern "C" {
#endif

__declspec_dll const unsigned int* mathBoxVertexIndicesDefault();
__declspec_dll const unsigned int* mathBoxEdgeVertexIndices(unsigned int edge_idx, unsigned int indices[2]);
__declspec_dll const unsigned int* mathBoxVertexIndicesAdjacent(unsigned int indices, unsigned int adj_indices[3]);
__declspec_dll const unsigned int* mathBoxFaceVertexIndices(unsigned int face_idx, unsigned int indices[4]);

#ifdef __cplusplus
}
#endif

#endif
