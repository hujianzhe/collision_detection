//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_GEOMETRY_DEF_H
#define	UTIL_C_CRT_GEOMETRY_GEOMETRY_DEF_H

#include "compiler_define.h"
#include "number_define.h"

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

typedef struct GeometryCircle_t {
	CCTNum_t o[3];
	CCTNum_t radius;
	CCTNum_t normal[3];
} GeometryCircle_t;

typedef struct GeometryAABB_t {
	CCTNum_t o[3];
	CCTNum_t half[3];
} GeometryAABB_t;

#ifndef GEOMETRY_BODY_BOX_MIN_HALF
	#define	GEOMETRY_BODY_BOX_MIN_HALF	(CCTNum(1e-5) + CCTNum(1e-5))
#endif

typedef struct GeometryOBB_t {
	CCTNum_t o[3];
	CCTNum_t half[3];
	CCTNum_t axis[3][3];
} GeometryOBB_t;

typedef struct GeometryPolygon_t {
	CCTNum_t (*v)[3]; /* vertices vec3 */
	CCTNum_t o[3]; /* origin position */
	CCTNum_t normal[3]; /* plane normal */
	unsigned int v_indices_cnt; /* number of edge vertices index */
	unsigned int tri_indices_cnt;  /* number of triangle vertices index */
	const unsigned int* v_indices; /* edge vertices index, must be ordered(clockwise or counterclockwise) */
	const unsigned int* tri_indices; /* triangle vertices index */
} GeometryPolygon_t;

typedef struct GeometryMesh_t {
	CCTNum_t (*v)[3]; /* vertices vec3 */
	CCTNum_t o[3]; /* origin position */
	GeometryAABB_t bound_box; /* AABB bound box */
	unsigned int polygons_cnt; /* number of polygen plane */
	unsigned int edge_indices_cnt; /* number of edge vertices index */
	unsigned int v_indices_cnt; /* number of vertices index */
	GeometryPolygon_t* polygons; /* array of polygens */
	const unsigned int* edge_indices; /* edge vertices index */
	const unsigned int* v_indices; /* vertices index */
} GeometryMesh_t;

/*********************************************************************/

enum {
	GEOMETRY_BODY_POINT = 1,
	GEOMETRY_BODY_SEGMENT = 2,
	GEOMETRY_BODY_PLANE = 3,
	GEOMETRY_BODY_SPHERE = 4,
	GEOMETRY_BODY_AABB = 5,
	GEOMETRY_BODY_POLYGON = 6,
	GEOMETRY_BODY_OBB = 7,
	GEOMETRY_BODY_CONVEX_MESH = 8,
};

typedef struct GeometryBody_t {
	union {
		char data;
		CCTNum_t point[3];
		GeometrySegment_t segment;
		GeometryPlane_t plane;
		GeometrySphere_t sphere;
		GeometryAABB_t aabb;
		GeometryPolygon_t polygon;
		GeometryOBB_t obb;
		GeometryMesh_t mesh;
	};
	int type;
} GeometryBody_t;

typedef struct GeometryBodyRef_t {
	union {
		void* data;
		CCTNum_t* point; /* CCTNum_t[3] */
		GeometrySegment_t* segment;
		GeometryPlane_t* plane;
		GeometrySphere_t* sphere;
		GeometryAABB_t* aabb;
		GeometryPolygon_t* polygon;
		GeometryOBB_t* obb;
		GeometryMesh_t* mesh;
	};
	int type;
} GeometryBodyRef_t;

#endif
