//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_GEOMETRY_DEF_H
#define	UTIL_C_CRT_GEOMETRY_GEOMETRY_DEF_H

#include "number_define.h"

/*********************************************************************/

typedef struct GeometryBorderId_t {
	unsigned int v_id;
	unsigned int edge_id;
} GeometryBorderId_t;

typedef struct GeometryPolygonVertexAdjacentInfo_t {
	/* v_ids[i] must locate in edge_ids[i] */

	unsigned int v_ids[2];
	unsigned int edge_ids[2];
} GeometryPolygonVertexAdjacentInfo_t;

typedef struct GeometryMeshVertexAdjacentInfo_t {
	/* v_ids[i] must locate in edge_ids[i] */
	/* v_cnt must equal edge_cnt */

	unsigned int v_cnt;
	unsigned int edge_cnt;
	unsigned int face_cnt;
	const unsigned int* v_ids;
	const unsigned int* edge_ids;
	const unsigned int* face_ids;
} GeometryMeshVertexAdjacentInfo_t;

/*********************************************************************/

typedef struct GeometrySegment_t {
	CCTNum_t v[2][3];
} GeometrySegment_t;

typedef struct GeometryPlane_t {
	CCTNum_t v[3];
	CCTNum_t normal[3];
} GeometryPlane_t;

typedef struct GeometrySphere_t {
	CCTNum_t o[3];
	CCTNum_t radius;
} GeometrySphere_t;

typedef struct GeometryCapsule_t {
	CCTNum_t o[3];
	CCTNum_t axis[3];
	CCTNum_t radius;
	CCTNum_t half;
} GeometryCapsule_t;

typedef struct GeometryAABB_t {
	CCTNum_t o[3];
	CCTNum_t half[3];
} GeometryAABB_t;

typedef struct GeometryOBB_t {
	CCTNum_t o[3];
	CCTNum_t half[3];
	CCTNum_t axis[3][3];
} GeometryOBB_t;

typedef struct GeometryPolygon_t {
	CCTNum_t (*v)[3]; /* vertices buffer */
	CCTNum_t center[3]; /* center position */
	CCTNum_t normal[3]; /* plane normal */
	short is_convex;
	unsigned int v_indices_cnt; /* number of vertices index, also number of vertices */
	unsigned int edge_cnt; /* number of edges */
	unsigned int tri_cnt;  /* number of triangle */
	const unsigned int* v_indices; /* vertices index */
	union {
		const unsigned int* edge_v_indices_flat;
		const unsigned int(*edge_v_indices)[2]; /* edge vertices index */
	};
	union {
		const unsigned int* edge_v_ids_flat;
		const unsigned int(*edge_v_ids)[2]; /* edge vertices logic id */
	};
	union {
		const unsigned int* tri_v_indices_flat;
		const unsigned int(*tri_v_indices)[3]; /* triangle vertices index */
	};
	union {
		const unsigned int* concave_tri_edge_ids_flat;
		const unsigned int(*concave_tri_edge_ids)[3]; /* if the polygon is concave, thid variable stores the edge id of the current face corresponding to each inner triangle */
	};
	const unsigned int* mesh_v_ids; /* if the polygon is a face of the mesh object, this variable stores the vertex id of the corresponding mesh object */
	const unsigned int* mesh_edge_ids; /* if the polygon is a face of the mesh object, this variable stores the edge id of the corresponding mesh object */
	const GeometryPolygonVertexAdjacentInfo_t* v_adjacent_infos; /* vertex adjacent infos */
} GeometryPolygon_t;

typedef struct GeometryMesh_t {
	CCTNum_t (*v)[3]; /* vertices buffer */
	GeometryAABB_t bound_box; /* AABB bound box, AABB.o is center position */
	short is_convex;
	short is_closed;
	unsigned int v_indices_cnt; /* number of vertices index, also number of vertices */
	unsigned int edge_cnt; /* number of edges */
	unsigned int polygons_cnt; /* number of polygen plane */
	const unsigned int* v_indices; /* vertices index */
	union {
		const unsigned int* edge_v_indices_flat;
		const unsigned int(*edge_v_indices)[2]; /* edge vertices index */
	};
	union {
		const unsigned int* edge_v_ids_flat;
		const unsigned int(*edge_v_ids)[2]; /* edge vertices logic id */
	};
	GeometryPolygon_t* polygons; /* array of polygens */
	const GeometryMeshVertexAdjacentInfo_t* v_adjacent_infos; /* vertex adjacent infos */
	union {
		const unsigned int* edge_adjacent_face_ids_flat;
		const unsigned int(*edge_adjacent_face_ids)[2]; /* edge adjacent face logic id */
	};
} GeometryMesh_t;

enum {
	GEOMETRY_BODY_POINT = 1,
	GEOMETRY_BODY_SEGMENT = 2,
	GEOMETRY_BODY_PLANE = 3,
	GEOMETRY_BODY_SPHERE = 4,
	GEOMETRY_BODY_AABB = 5,
	GEOMETRY_BODY_OBB = 6,
	GEOMETRY_BODY_POLYGON = 7,
	GEOMETRY_BODY_MESH = 8,
	GEOMETRY_BODY_CAPSULE = 9,
};

typedef struct GeometryBody_t {
	union {
		unsigned char data;
		CCTNum_t point[3];
		GeometrySegment_t segment;
		GeometryPlane_t plane;
		GeometrySphere_t sphere;
		GeometryAABB_t aabb;
		GeometryOBB_t obb;
		GeometryPolygon_t polygon;
		GeometryMesh_t mesh;
		GeometryCapsule_t capsule;
	};
	int type;
} GeometryBody_t;

#endif
