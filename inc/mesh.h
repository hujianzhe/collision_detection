//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_MESH_H
#define	UTIL_C_CRT_GEOMETRY_MESH_H

#include "geometry_def.h"

#ifdef	__cplusplus
extern "C" {
#endif

__declspec_dll GeometryMesh_t* mathMeshDeepCopy(GeometryMesh_t* dst, const GeometryMesh_t* src, const CCTAllocator_t* ac);
__declspec_dll void mathMeshClear(GeometryMesh_t* mesh);

__declspec_dll int mathMeshIsClosed(const GeometryMesh_t* mesh);
__declspec_dll int mathMeshIsConvex(const GeometryMesh_t* mesh);

__declspec_dll size_t mathMeshBinaryStreamView(GeometryMesh_t* mesh, const void* buffer, size_t len, const CCTAllocator_t* ac);

#ifdef	__cplusplus
}
#endif

#endif
