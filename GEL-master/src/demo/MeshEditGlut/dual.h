#ifndef __DUAL_H__
#define __DUAL_H__

#include <GEL/HMesh/HMesh.h>



/// Compute the mesh dual where every vertex is a face and vice versa.
void dual(HMesh::Manifold&);

#endif