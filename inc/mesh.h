//
// Created by hujianzhe
//

#ifndef	UTIL_C_CRT_GEOMETRY_MESH_H
#define	UTIL_C_CRT_GEOMETRY_MESH_H

#include "geometry_def.h"

#ifdef	__cplusplus
extern "C" {
#endif

__declspec_dll GeometryMesh_t* mathMeshDeepCopy(GeometryMesh_t* dst, const GeometryMesh_t* src);
__declspec_dll void mathMeshClear(GeometryMesh_t* mesh);

__declspec_dll int mathMeshIsClosed(const GeometryMesh_t* mesh);
__declspec_dll int mathMeshIsConvex(const GeometryMesh_t* mesh);

__declspec_dll size_t mathMeshBinarySize(const GeometryMesh_t* mesh);
__declspec_dll size_t mathMeshSaveBinary(const GeometryMesh_t* mesh, void* buffer);
__declspec_dll GeometryMesh_t* mathMeshLoadBinary(const void* buffer, size_t len, GeometryMesh_t* mesh);

#ifdef	__cplusplus
}
#endif

#endif
