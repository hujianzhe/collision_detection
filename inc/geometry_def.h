//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_GEOMETRY_DEF_H
#define	UTIL_C_CRT_GEOMETRY_GEOMETRY_DEF_H

#include "number_define.h"

/*********************************************************************/

typedef struct GeometrySegment_t {
	CCTNum_t v[2][3];
	CCTNum_t o[3];
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
	CCTNum_t (*v)[3]; /* vertices vec3 */
	CCTNum_t o[3]; /* module origin position */
	CCTNum_t center[3]; /* center position */
	CCTNum_t normal[3]; /* plane normal */
	short is_convex;
	unsigned int edge_v_indices_cnt; /* number of edge vertices index */
	unsigned int v_indices_cnt; /* number of edge vertices index */
	unsigned int tri_indices_cnt;  /* number of triangle vertices index */
	const unsigned int* edge_v_indices; /* edge vertices index */
	const unsigned int* v_indices; /* vertices index */
	const unsigned int* tri_indices; /* triangle vertices index */
	const unsigned int* mesh_edge_index; /* if the polygon is a face of the mesh object, this variable stores the edge index of the corresponding mesh object */
} GeometryPolygon_t;

typedef struct GeometryMesh_t {
	CCTNum_t (*v)[3]; /* vertices vec3 */
	CCTNum_t o[3]; /* module origin position */
	GeometryAABB_t bound_box; /* AABB bound box, AABB.o is center position */
	short is_convex;
	short is_closed;
	unsigned int polygons_cnt; /* number of polygen plane */
	unsigned int edge_v_indices_cnt; /* number of edge vertices index */
	unsigned int v_indices_cnt; /* number of vertices index */
	GeometryPolygon_t* polygons; /* array of polygens */
	const unsigned int* edge_v_indices; /* edge vertices index */
	const unsigned int* v_indices; /* vertices index */
} GeometryMesh_t;

enum {
	GEOMETRY_BODY_POINT = 1,
	GEOMETRY_BODY_SEGMENT = 2,
	GEOMETRY_BODY_PLANE = 3,
	GEOMETRY_BODY_SPHERE = 4,
	GEOMETRY_BODY_AABB = 5,
	GEOMETRY_BODY_OBB = 6,
	GEOMETRY_BODY_POLYGON = 7,
	GEOMETRY_BODY_CONVEX_MESH = 8,
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

/*********************************************************************/

typedef struct GeometryBorderIndex_t {
	unsigned int v_idx;
	unsigned int edge_idx;
} GeometryBorderIndex_t;

#endif
