//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/obb.h"
#include "../inc/vertex.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

GeometryOBB_t* mathOBBFromAABB(GeometryOBB_t* obb, const CCTNum_t o[3], const CCTNum_t half[3]) {
	mathVec3Copy(obb->o, o);
	mathVec3Copy(obb->half, half);
	mathVec3Set(obb->axis[0], CCTNums_3(1.0, 0.0, 0.0));
	mathVec3Set(obb->axis[1], CCTNums_3(0.0, 1.0, 0.0));
	mathVec3Set(obb->axis[2], CCTNums_3(0.0, 0.0, 1.0));
	return obb;
}

void mathOBBToAABB(const GeometryOBB_t* obb, CCTNum_t o[3], CCTNum_t half[3]) {
	int i;
	CCTNum_t v[8][3], min_v[3], max_v[3];
	mathOBBVertices(obb, v);
	mathVec3Copy(min_v, v[0]);
	mathVec3Copy(max_v, v[0]);
	for (i = 1; i < 8; ++i) {
		int j;
		for (j = 0; j < 3; ++j) {
			if (min_v[j] > v[i][j]) {
				min_v[j] = v[i][j];
			}
			else if (max_v[j] < v[i][j]) {
				max_v[j] = v[i][j];
			}
		}
	}
	mathVec3Copy(o, obb->o);
	for (i = 0; i < 3; ++i) {
		half[i] = CCTNum(0.5) * (max_v[i] - min_v[i]);
	}
}

void mathOBBVertices(const GeometryOBB_t* obb, CCTNum_t v[8][3]) {
	CCTNum_t AX[3][3];
	mathVec3MultiplyScalar(AX[0], obb->axis[0], obb->half[0]);
	mathVec3MultiplyScalar(AX[1], obb->axis[1], obb->half[1]);
	mathVec3MultiplyScalar(AX[2], obb->axis[2], obb->half[2]);

	mathVec3Copy(v[0], obb->o);
	mathVec3Sub(v[0], v[0], AX[0]);
	mathVec3Sub(v[0], v[0], AX[1]);
	mathVec3Sub(v[0], v[0], AX[2]);

	mathVec3Copy(v[1], obb->o);
	mathVec3Add(v[1], v[1], AX[0]);
	mathVec3Sub(v[1], v[1], AX[1]);
	mathVec3Sub(v[1], v[1], AX[2]);

	mathVec3Copy(v[2], obb->o);
	mathVec3Add(v[2], v[2], AX[0]);
	mathVec3Add(v[2], v[2], AX[1]);
	mathVec3Sub(v[2], v[2], AX[2]);

	mathVec3Copy(v[3], obb->o);
	mathVec3Sub(v[3], v[3], AX[0]);
	mathVec3Add(v[3], v[3], AX[1]);
	mathVec3Sub(v[3], v[3], AX[2]);

	mathVec3Copy(v[4], obb->o);
	mathVec3Sub(v[4], v[4], AX[0]);
	mathVec3Sub(v[4], v[4], AX[1]);
	mathVec3Add(v[4], v[4], AX[2]);

	mathVec3Copy(v[5], obb->o);
	mathVec3Add(v[5], v[5], AX[0]);
	mathVec3Sub(v[5], v[5], AX[1]);
	mathVec3Add(v[5], v[5], AX[2]);

	mathVec3Copy(v[6], obb->o);
	mathVec3Add(v[6], v[6], AX[0]);
	mathVec3Add(v[6], v[6], AX[1]);
	mathVec3Add(v[6], v[6], AX[2]);

	mathVec3Copy(v[7], obb->o);
	mathVec3Sub(v[7], v[7], AX[0]);
	mathVec3Add(v[7], v[7], AX[1]);
	mathVec3Add(v[7], v[7], AX[2]);
}

void mathOBBMinVertice(const GeometryOBB_t* obb, CCTNum_t v[3]) {
	CCTNum_t AX[3][3];
	mathVec3MultiplyScalar(AX[0], obb->axis[0], obb->half[0]);
	mathVec3MultiplyScalar(AX[1], obb->axis[1], obb->half[1]);
	mathVec3MultiplyScalar(AX[2], obb->axis[2], obb->half[2]);

	mathVec3Copy(v, obb->o);
	mathVec3Sub(v, v, AX[0]);
	mathVec3Sub(v, v, AX[1]);
	mathVec3Sub(v, v, AX[2]);
}

void mathOBBMaxVertice(const GeometryOBB_t* obb, CCTNum_t v[3]) {
	CCTNum_t AX[3][3];
	mathVec3MultiplyScalar(AX[0], obb->axis[0], obb->half[0]);
	mathVec3MultiplyScalar(AX[1], obb->axis[1], obb->half[1]);
	mathVec3MultiplyScalar(AX[2], obb->axis[2], obb->half[2]);

	mathVec3Copy(v, obb->o);
	mathVec3Add(v, v, AX[0]);
	mathVec3Add(v, v, AX[1]);
	mathVec3Add(v, v, AX[2]);
}

void mathOBBClosestPointTo(const GeometryOBB_t* obb, const CCTNum_t p[3], CCTNum_t closest_p[3]) {
	int i;
	CCTNum_t v[3];

	mathVec3Sub(v, p, obb->o);
	mathVec3Copy(closest_p, obb->o);
	for (i = 0; i < 3; ++i) {
		CCTNum_t d = mathVec3Dot(v, obb->axis[i]);
		if (d > obb->half[i]) {
			d = obb->half[i];
		}
		else if (d < -obb->half[i]) {
			d = -obb->half[i];
		}
		mathVec3AddScalar(closest_p, obb->axis[i], d);
	}
}

extern const unsigned int Box_Vertice_Indices_Default[8];
extern const unsigned int Box_Edge_Indices[24];
extern const unsigned int Box_Face_Indices[6][4];

GeometryBoxMesh_t* mathOBBMesh(GeometryBoxMesh_t* bm, const GeometryOBB_t* obb) {
	unsigned int i;
	GeometryMesh_t* mesh = &bm->mesh;
	CCTNum_t min_v[3], max_v[3];
	mathOBBVertices(obb, bm->v);
	mesh->v = bm->v;
	mesh->v_indices = Box_Vertice_Indices_Default;
	mesh->v_indices_cnt = sizeof(Box_Vertice_Indices_Default) / sizeof(Box_Vertice_Indices_Default[0]);
	mesh->edge_indices = Box_Edge_Indices;
	mesh->edge_indices_cnt = sizeof(Box_Edge_Indices) / sizeof(Box_Edge_Indices[0]);
	mesh->polygons = bm->faces;
	mesh->polygons_cnt = sizeof(bm->faces) / sizeof(bm->faces[0]);
	mathVec3Set(mesh->o, CCTNums_3(0.0, 0.0, 0.0));
	mathVerticesFindMinMaxXYZ((const CCTNum_t(*)[3])bm->v, 8, min_v, max_v);
	mathVec3Copy(mesh->bound_box.o, obb->o);
	for (i = 0; i < 3; ++i) {
		mesh->bound_box.half[i] = CCTNum(0.5) * (max_v[i] - min_v[i]);
	}
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		GeometryPolygon_t* polygon = mesh->polygons + i;
		polygon->v = bm->v;
		polygon->v_indices = Box_Face_Indices[i];
		polygon->v_indices_cnt = sizeof(Box_Face_Indices[i]) / sizeof(Box_Face_Indices[i][0]);
		polygon->tri_indices = NULL;
		polygon->tri_indices_cnt = 0;
		mathVec3Set(polygon->o, CCTNums_3(0.0, 0.0, 0.0));
		if (i < 2) {
			if (0 == i) {
				mathVec3Copy(polygon->normal, obb->axis[2]);
			}
			else {
				mathVec3Negate(polygon->normal, obb->axis[2]);
			}
		}
		else if (i < 4) {
			if (2 == i) {
				mathVec3Copy(polygon->normal, obb->axis[0]);
			}
			else {
				mathVec3Negate(polygon->normal, obb->axis[0]);
			}
		}
		else {
			if (4 == i) {
				mathVec3Copy(polygon->normal, obb->axis[1]);
			}
			else {
				mathVec3Negate(polygon->normal, obb->axis[2]);
			}
		}
	}
	return bm;
}

#ifdef __cplusplus
}
#endif
