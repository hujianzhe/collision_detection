//
// Created by hujianzhe
//

#include "../inc/math_vec3.h"
#include "../inc/polygon.h"
#include "../inc/mesh.h"
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

GeometryMesh_t* mathMeshDeepCopy(GeometryMesh_t* dst, const GeometryMesh_t* src) {
	unsigned int i, dup_v_cnt = 0;
	CCTNum_t(*dup_v)[3] = NULL;
	unsigned int* dup_v_indices = NULL;
	unsigned int* dup_edge_indices = NULL;
	GeometryPolygon_t* dup_polygons = NULL;
	/* find max vertex index, dup_v_cnt */
	for (i = 0; i < src->v_indices_cnt; ++i) {
		if (src->v_indices[i] >= dup_v_cnt) {
			dup_v_cnt = src->v_indices[i] + 1;
		}
	}
	for (i = 0; i < src->edge_indices_cnt; ++i) {
		if (src->edge_indices[i] >= dup_v_cnt) {
			dup_v_cnt = src->edge_indices[i] + 1;
		}
	}
	for (i = 0; i < src->polygons_cnt; ++i) {
		const GeometryPolygon_t* src_polygon = src->polygons + i;
		unsigned int j;
		for (j = 0; j < src_polygon->v_indices_cnt; ++j) {
			if (src_polygon->v_indices[j] >= dup_v_cnt) {
				dup_v_cnt = src_polygon->v_indices[j] + 1;
			}
		}
		for (j = 0; j < src_polygon->tri_indices_cnt; ++j) {
			if (src_polygon->tri_indices[j] >= dup_v_cnt) {
				dup_v_cnt = src_polygon->tri_indices[j] + 1;
			}
		}
	}
	if (dup_v_cnt < 3) {
		return NULL;
	}
	/* deep copy */
	dup_v = (CCTNum_t(*)[3])malloc(sizeof(dup_v[0]) * dup_v_cnt);
	if (!dup_v) {
		goto err_0;
	}
	dup_v_indices = (unsigned int*)malloc(sizeof(dup_v_indices[0]) * dup_v_cnt);
	if (!dup_v_indices) {
		goto err_0;
	}
	dup_edge_indices = (unsigned int*)malloc(sizeof(dup_edge_indices[0]) * src->edge_indices_cnt);
	if (!dup_edge_indices) {
		goto err_0;
	}
	dup_polygons = (GeometryPolygon_t*)malloc(sizeof(dup_polygons[0]) * src->polygons_cnt);
	if (!dup_polygons) {
		goto err_0;
	}
	for (i = 0; i < src->polygons_cnt; ++i) {
		unsigned int j;
		const GeometryPolygon_t* src_polygon = src->polygons + i;
		unsigned int* dup_v_indices = NULL, *dup_tri_indices = NULL;
		dup_v_indices = (unsigned int*)malloc(sizeof(dup_v_indices[0]) * src_polygon->v_indices_cnt);
		if (!dup_v_indices) {
			goto err_1;
		}
		dup_tri_indices = (unsigned int*)malloc(sizeof(dup_tri_indices[0]) * src_polygon->tri_indices_cnt);
		if (!dup_tri_indices) {
			goto err_1;
		}
		for (j = 0; j < src_polygon->v_indices_cnt; ++j) {
			dup_v_indices[j] = src_polygon->v_indices[j];
		}
		for (j = 0; j < src_polygon->tri_indices_cnt; ++j) {
			dup_tri_indices[j] = src_polygon->tri_indices[j];
		}
		mathVec3Copy(dup_polygons[i].o, src_polygon->o);
		mathVec3Copy(dup_polygons[i].center, src_polygon->center);
		mathVec3Copy(dup_polygons[i].normal, src_polygon->normal);
		dup_polygons[i].v = dup_v;
		dup_polygons[i].v_indices_cnt = src_polygon->v_indices_cnt;
		dup_polygons[i].tri_indices_cnt = src_polygon->tri_indices_cnt;
		dup_polygons[i].v_indices = dup_v_indices;
		dup_polygons[i].tri_indices = dup_tri_indices;
		dup_polygons[i].is_convex = src_polygon->is_convex;
		continue;
	err_1:
		free(dup_v_indices);
		free(dup_tri_indices);
		for (j = 0; j < i; ++j) {
			free((void*)dup_polygons[j].v_indices);
			free((void*)dup_polygons[j].tri_indices);
		}
		goto err_0;
	}
	for (i = 0; i < src->v_indices_cnt; ++i) {
		unsigned int idx = src->v_indices[i];
		dup_v_indices[i] = idx;
		mathVec3Copy(dup_v[idx], src->v[idx]);
	}
	for (i = 0; i < src->edge_indices_cnt; ++i) {
		dup_edge_indices[i] = src->edge_indices[i];
	}
	mathVec3Copy(dst->o, src->o);
	dst->v = dup_v;
	dst->bound_box = src->bound_box;
	dst->polygons_cnt = src->polygons_cnt;
	dst->edge_indices_cnt = src->edge_indices_cnt;
	dst->edge_stride = src->edge_stride;
	dst->is_convex = src->is_convex;
	dst->v_indices_cnt = src->v_indices_cnt;
	dst->polygons = dup_polygons;
	dst->edge_indices = dup_edge_indices;
	dst->v_indices = dup_v_indices;
	return dst;
err_0:
	free(dup_v);
	free(dup_v_indices);
	free(dup_edge_indices);
	free(dup_polygons);
	return NULL;
}

void mathMeshFreeData(GeometryMesh_t* mesh) {
	unsigned int i;
	if (!mesh) {
		return;
	}
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		mesh->polygons[i].v = NULL;
		mathPolygonFreeData(mesh->polygons + i);
	}
	if (mesh->polygons) {
		free(mesh->polygons);
		mesh->polygons = NULL;
		mesh->polygons_cnt = 0;
	}
	if (mesh->edge_indices) {
		free((void*)mesh->edge_indices);
		mesh->edge_indices = NULL;
		mesh->edge_indices_cnt = 0;
	}
	if (mesh->v_indices) {
		free((void*)mesh->v_indices);
		mesh->v_indices = NULL;
		mesh->v_indices_cnt = 0;
	}
	if (mesh->v) {
		free(mesh->v);
		mesh->v = NULL;
	}
}

int mathMeshIsClosed(const GeometryMesh_t* mesh) {
	unsigned int i;
	if (mesh->v_indices_cnt < 3) {
		return 0;
	}
	for (i = 0; i < mesh->v_indices_cnt; ++i) {
		unsigned int j, cnt = 0;
		unsigned int vi = mesh->v_indices[i];
		for (j = 0; j < mesh->polygons_cnt; ++j) {
			unsigned int k;
			const GeometryPolygon_t* polygon = mesh->polygons + j;
			for (k = 0; k < polygon->v_indices_cnt; ++k) {
				unsigned int vk = polygon->v_indices[k];
				if (vi == vk || mathVec3Equal(mesh->v[vi], polygon->v[vk])) {
					++cnt;
					break;
				}
			}
			if (cnt >= 3) {
				break;
			}
		}
		if (cnt < 3) {
			return 0;
		}
	}
	return 1;
}

int mathMeshIsConvex(const GeometryMesh_t* mesh, CCTNum_t epsilon) {
	unsigned int i;
	if (mesh->polygons_cnt < 4) {
		return 0;
	}
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		const GeometryPolygon_t* polygon = mesh->polygons + i;
		int flag_sign = 0;
		unsigned int j;
		for (j = 0; j < mesh->v_indices_cnt; ++j) {
			CCTNum_t vj[3], dot;
			mathVec3Sub(vj, mesh->v[mesh->v_indices[j]], polygon->v[polygon->v_indices[0]]);
			dot = mathVec3Dot(polygon->normal, vj);
			/* some module needed epsilon */
			if (dot > epsilon) {
				if (flag_sign < 0) {
					return 0;
				}
				flag_sign = 1;
			}
			else if (dot < -epsilon) {
				if (flag_sign > 0) {
					return 0;
				}
				flag_sign = -1;
			}
		}
	}
	return 1;
}

void mathConvexMeshMakeFacesOut(GeometryMesh_t* mesh) {
	CCTNum_t p[2][3], po[3];
	unsigned int i;
	for (i = 0; i < 2; ++i) {
		CCTNum_t tri[3][3];
		const GeometryPolygon_t* polygon = mesh->polygons + i;
		mathVec3Copy(tri[0], polygon->v[polygon->v_indices[0]]);
		mathVec3Copy(tri[1], polygon->v[polygon->v_indices[1]]);
		mathVec3Copy(tri[2], polygon->v[polygon->v_indices[2]]);
		mathTriangleGetPoint((const CCTNum_t(*)[3])tri, CCTNum(0.5), CCTNum(0.5), p[i]);
	}
	mathVec3Add(po, p[0], p[1]);
	mathVec3MultiplyScalar(po, po, CCTNum(0.5));
	for (i = 0; i < mesh->polygons_cnt; ++i) {
		CCTNum_t v[3], dot;
		GeometryPolygon_t* polygon = mesh->polygons + i;
		mathVec3Sub(v, po, polygon->v[polygon->v_indices[0]]);
		dot = mathVec3Dot(v, polygon->normal);
		if (dot > CCTNum(0.0)) {
			mathVec3Negate(polygon->normal, polygon->normal);
		}
	}
}

#ifdef __cplusplus
}
#endif
