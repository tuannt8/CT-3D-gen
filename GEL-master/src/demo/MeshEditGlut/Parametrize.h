//
//  Parametrize.h
//  MeshEditE
//
//  Created by J. Andreas Bærentzen on 25/05/15.
//  Copyright (c) 2015 J. Andreas Bærentzen. All rights reserved.
//

#ifndef __MeshEditE__Parametrize__
#define __MeshEditE__Parametrize__

#include <stdio.h>
#include <GEL/HMesh/Manifold.h>
#include <GEL/HMesh/AttributeVector.h>

enum WeightScheme {FLOATER_W, HARMONIC_W, BARYCENTRIC_W};


/** Parametrization using the LSCM method as described in Polygonal Mesh Processing by
    Botsch et al. */
void parametrize_LSCM(HMesh::Manifold& m, HMesh::VertexAttributeVector<int>& selected);

/** Iterative parametrization using one of several edge weight schemes. */
void parametrize_iterative(HMesh::Manifold& m, WeightScheme ws, HMesh::VertexAttributeVector<int>& selected);

/** Given a parametrization, flatten the mesh */
void flatten(HMesh::Manifold& m, double scale);

#endif /* defined(__MeshEditE__Parametrize__) */
