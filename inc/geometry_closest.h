//
// Created by hujianzhe
//

#ifndef UTIL_C_CRT_GEOMETRY_CLOSEST_H
#define	UTIL_C_CRT_GEOMETRY_CLOSEST_H

#include "geometry_def.h"

#ifdef	__cplusplus
extern "C" {
#endif

__declspec_dll void mathLineClosestLine_opposite(const CCTNum_t lsv1[3], const CCTNum_t lsdir1[3], const CCTNum_t lsv2[3], const CCTNum_t lsdir2[3], CCTNum_t* lsdir_d1, CCTNum_t* lsdir_d2);
__declspec_dll void mathLineClosestLine_opposite_v2(const CCTNum_t lsv1[3], const CCTNum_t lsdir1[3], const CCTNum_t lsv2[3], const CCTNum_t lsdir2[3], CCTNum_t closest_p1[3], CCTNum_t closest_p2[3]);

__declspec_dll void mathSegmentClosestPoint(const CCTNum_t ls0[3], const CCTNum_t ls1[3], const CCTNum_t p[3], CCTNum_t closest_p[3]);
__declspec_dll void mathSegmentClosestPoint_v2(const CCTNum_t ls_center_p[3], const CCTNum_t lsdir[3], const CCTNum_t ls_half_len, const CCTNum_t p[3], CCTNum_t closest_p[3]);
__declspec_dll void mathSegmentClosestPoint_v3(const CCTNum_t ls_v[3], const CCTNum_t lsdir[3], const CCTNum_t ls_len, const CCTNum_t p[3], CCTNum_t closest_p[3]);
__declspec_dll void mathSegmentIndicesClosestPoint(const CCTNum_t(*v)[3], const unsigned int* edge_v_indices_flat, unsigned int edge_v_indices_cnt, const CCTNum_t p[3], CCTNum_t closest_p[3]);

__declspec_dll void mathAABBClosestPoint(const CCTNum_t min_v[3], const CCTNum_t max_v[3], const CCTNum_t p[3], CCTNum_t closest_p[3]);
__declspec_dll void mathOBBClosestPoint(const GeometryOBB_t* obb, const CCTNum_t p[3], CCTNum_t closest_p[3]);

__declspec_dll void mathSphereClosestPoint(const CCTNum_t o[3], CCTNum_t radius, const CCTNum_t p[3], CCTNum_t closest_p[3]);
__declspec_dll void mathCapsuleClosestPoint(const GeometryCapsule_t* capsule, const CCTNum_t p[3], CCTNum_t closest_p[3]);

__declspec_dll void mathPolygonClosestPoint(const GeometryPolygon_t* polygon, const CCTNum_t p[3], CCTNum_t closest_p[3]);
__declspec_dll void mathMeshClosestPoint(const GeometryMesh_t* mesh, const CCTNum_t p[3], CCTNum_t closest_p[3]);

#ifdef	__cplusplus
}
#endif

#endif
