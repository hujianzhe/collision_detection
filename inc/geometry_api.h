//
// Created by hujianzhe
//

#ifndef UTIL_C_CRT_GEOMETRY_API_H
#define	UTIL_C_CRT_GEOMETRY_API_H

#include "geometry_def.h"

typedef struct CCTSweepHitInfo_t {
	int hit_part;
	unsigned int id;
} CCTSweepHitInfo_t;

typedef struct CCTSweepResult_t {
	CCTNum_t distance;
	CCTNum_t hit_plane_v[3];
	CCTNum_t hit_plane_n[3];
	short overlap;
	short hit_unique_point;
	CCTSweepHitInfo_t peer[2];
} CCTSweepResult_t;

enum {
	CCT_SWEEP_HIT_POINT = 1,
	CCT_SWEEP_HIT_EDGE = 2,
	CCT_SWEEP_HIT_FACE = 4,
	CCT_SWEEP_HIT_SPHERE = 8,
};

#ifdef __cplusplus
extern "C" {
#endif

__declspec_dll size_t mathGeometrySize(int geo_type);
__declspec_dll int mathGeometryCheckParametersValid(const void* geo_data, int geo_type);

__declspec_dll void* mathGeometryClone(void* dst_data, int* dst_type, const void* src_geo_data, int src_geo_type);
__declspec_dll void mathGeometryFree(void* geo_data, int geo_type);
__declspec_dll void mathGeometryFreeBody(GeometryBody_t* b);

__declspec_dll const CCTNum_t* mathGeometryGetPosition(const void* geo_data, int geo_type, CCTNum_t v[3]);
__declspec_dll void mathGeometrySetPosition(void* geo_data, int geo_type, const CCTNum_t v[3]);

__declspec_dll GeometryAABB_t* mathGeometryBoundingBox(const void* geo_data, int geo_type, GeometryAABB_t* aabb);
__declspec_dll GeometryBody_t* mathGeometryInflate(const void* geo_data, int geo_type, CCTNum_t inflate, GeometryBody_t* geo_inflate);
__declspec_dll int mathGeometryRotate(void* geo_data, int geo_type, const CCTNum_t base_p[3], const CCTNum_t q[4]);
__declspec_dll int mathGeometryRotateAxisRadian(void* geo_data, int geo_type, const CCTNum_t base_p[3], const CCTNum_t axis[3], CCTNum_t radian);

__declspec_dll CCTNum_t mathGeometrySeparateDistance(const void* geo_data, int geo_type, const CCTNum_t plane_v[3], const CCTNum_t separate_dir[3]);

__declspec_dll int mathGeometryContain(const void* geo_data1, int geo_type1, const void* geo_data2, int geo_type2);
__declspec_dll int mathGeometryIntersect(const void* geo_data1, int geo_type1, const void* geo_data2, int geo_type2);
__declspec_dll CCTSweepResult_t* mathGeometrySweep(const void* geo_data1, int geo_type1, const CCTNum_t dir[3], const void* geo_data2, int geo_type2, CCTSweepResult_t* result);

#ifdef	__cplusplus
}
#endif

#endif
