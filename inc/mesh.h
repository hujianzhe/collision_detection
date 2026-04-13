//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_MESH_H
#define	UTIL_C_CRT_GEOMETRY_MESH_H

#include "geometry_def.h"

#ifdef	__cplusplus
extern "C" {
#endif

__declspec_dll GeometryMesh_t* mathMeshClone(GeometryMesh_t* dst, const GeometryMesh_t* src, const CCTAllocator_t* ac);
__declspec_dll GeometryMesh_t* mathMeshDupFaces(GeometryMesh_t* dup_mesh, const GeometryPolygon_t* faces, unsigned int face_cnt, const CCTAllocator_t* ac);
__declspec_dll void mathMeshClear(GeometryMesh_t* mesh);

#ifdef	__cplusplus
}
#endif

#endif
