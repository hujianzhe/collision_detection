//
// Created by hujianzhe
//

#include "../inc/const_data.h"

const CCTConstVal_t CCTConstVal_ = {
	/* Axis_X, Axis_Y, Axis_Z, AABB_Axis */{{
		{ CCTNums_3(1.0, 0.0, 0.0) },
		{ CCTNums_3(0.0, 1.0, 0.0) },
		{ CCTNums_3(0.0, 0.0, 1.0) }
	}},
	/* AABB_Face_Normals */{
		{ CCTNums_3(1.0, 0.0, 0.0) }, { CCTNums_3(-1.0, 0.0, 0.0) },
		{ CCTNums_3(0.0, 1.0, 0.0) }, { CCTNums_3(0.0, -1.0, 0.0) },
		{ CCTNums_3(0.0, 0.0, 1.0) }, { CCTNums_3(0.0, 0.0, -1.0) }
	},
	/* Vec3_Zero */{ CCTNums_3(0.0, 0.0, 0.0) },
	/* Quat_Zero */{ CCTNums_4(0.0, 0.0, 0.0, 0.0) },
	/* Quat_Identity */{ CCTNums_4(0.0, 0.0, 0.0, 1.0) },
	/* Segment_VertexIds, Segment_Edge_VertexIds */{
		{ 0, 1 }
	},
	/* Triangle_VertexIds */{ 0, 1, 2 },
	/* Triangle_Edge_VertexIds */{ 0,1, 1,2, 2,0 },
	/* Rect_VertexIds */{ 0, 1, 2, 3 },
	/* Rect_Edge_VertexIds */ { 0,1, 1,2, 2,3, 3,0 },
	/* Box_Vertices_Indices */{ 0, 1, 2, 3, 4, 5, 6, 7 },
	/* Box_Triangle_Vertices_Indices */{
		1, 5, 6,	6, 2, 1,
		3, 7, 4,	4, 0, 3,
		3, 7, 6,	6, 2, 3,
		0, 4, 5,	5, 1, 0,
		7, 6, 5,	5, 4, 7,
		0, 1, 2,	2, 3, 0
	},
	/* Box_Face_MeshVertexIds */{
		1, 2, 6, 5,
		0, 3, 7, 4,
		3, 2, 6, 7,
		0, 1, 5, 4,
		4, 5, 6, 7,
		0, 1, 2, 3 
	},
	/* Box_Edge_VertexIds */{
		0, 1,	1, 2,	2, 3,	3, 0,
		7, 6,	6, 5,	5, 4,	4, 7,
		1, 5,	6, 2,
		3, 7,	4, 0
	},
	/* Box_Vertex_Adjacent_VertexIds */{
		1, 3, 4,
		0, 2, 5,
		3, 1, 6,
		2, 0, 7,
		5, 7, 0,
		4, 6, 1,
		7, 5, 2,
		6, 4, 3
	},
	/* Box_Vertex_Adjacent_EdgeIds */{
		0, 3, 11,
		0, 1, 8,
		2, 1, 9,
		2, 3, 10,
		6, 7, 11,
		6, 5, 8,
		4, 5, 9,
		4, 7, 10
	},
	/* Box_Vertex_Adjacent_FaceIds */{
		5, 1, 3,
		5, 0, 3,
		2, 0, 5,
		1, 2, 5,
		1, 3, 4,
		0, 3, 4,
		0, 2, 4,
		1, 2, 4
	},
	/* Box_Face_Edge_MeshVertexIds */{
		1,2, 2,6, 6,5, 5,1,
		0,3, 3,7, 7,4, 4,0,
		3,2, 2,6, 6,7, 7,3,
		0,1, 1,5, 5,4, 4,0,
		4,5, 5,6, 6,7, 7,4,
		0,1, 1,2, 2,3, 3,0
	},
	/* Box_Face_MeshEdgeIds */{
		1,9,5,8,
		3,10,7,11,
		2,9,4,10,
		0,8,6,11,
		6,5,4,7,
		0,1,2,3
	},
	/* Box_Edge_Adjacent_FaceIds */{
		5,3,	5,0,	5,2,	5,1,	4,2,	4,0,
		4,3,	4,1,	3,0,	2,0,	2,1,	3,1
	},
};

#ifdef	__cplusplus
extern "C" {
#endif

const CCTConstVal_t* CCT_ConstVal() { return &CCTConstVal_; }

#ifdef	__cplusplus
}
#endif
