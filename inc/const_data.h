//
// Created by hujianzhe
//

#ifndef	UTIL_CCT_CONST_DATA_H
#define	UTIL_CCT_CONST_DATA_H

#include "geometry_def.h"

typedef struct CCTConstVal_t {
	union {
		struct {
			const CCTNum_t Axis_X[3];
			const CCTNum_t Axis_Y[3];
			const CCTNum_t Axis_Z[3];
		};
		const CCTNum_t AABB_Axis[3][3];
	};
	const CCTNum_t AABB_Face_Normals[6][3];
	const CCTNum_t Vec3_Zero[3];
	const CCTNum_t Quat_Zero[4];
	const CCTNum_t Quat_Identity[4];
	union {
		const unsigned int Segment_VertexIds[2];
		const unsigned int Segment_Edge_VertexIds_Flat[2];
	};
	const unsigned int Triangle_VertexIds[3];
	const unsigned int Triangle_Edge_VertexIds_Flat[6];
	const unsigned int Rect_VertexIds[4];
	const unsigned int Rect_Edge_VertexIds_Flat[8];
	const unsigned int Box_VertexIds[8];
	const unsigned int Box_Triangle_VertexIds_Flat[36];
	const unsigned int Box_Face_MeshVertexIds_Flat[24];
	const unsigned int Box_Edge_VertexIds_Flat[24];
	const unsigned int Box_Face_Edge_MeshVertexIds_Flat[48];
	const unsigned int Box_Face_MeshEdgeIds_Flat[24];
	const GeometryMeshVertexAdjacentInfo_t Box_Vertex_AdjacentInfos[8];
	const GeometryPolygonVertexAdjacentInfo_t Box_Face_Vertex_AdjacentInfos[4];
	union {
		const unsigned int Box_Edge_Adjacent_FaceIds_Flat[24];
		const unsigned int Box_Edge_Adjacent_FaceIds[12][2];
	};
} CCTConstVal_t;

#ifdef	__cplusplus
extern "C" {
#endif

__declspec_dll const CCTConstVal_t* CCT_ConstVal();

#ifdef	__cplusplus
}
#endif

#endif
