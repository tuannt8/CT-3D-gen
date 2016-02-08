//
//  curvature.h
//  MeshEditGLUT
//
//  Created by J. Andreas Bærentzen on 04/09/15.
//  Copyright (c) 2015 J. Andreas Bærentzen. All rights reserved.
//

#ifndef __MeshEditGLUT__curvature__
#define __MeshEditGLUT__curvature__

#include <GEL/HMesh/HMesh.h>

/// Compute the mixed area - Voronoi area approximation around vertex
double mixed_area(const HMesh::Manifold& m, HMesh::VertexID v);

/** Compute Gaussian curvature for all vertices. The second argument contains the gaussian
 curvatures upon return. The function returns the integral gaussian curvature */

double compute_gaussian_curvature(const HMesh::Manifold& m, HMesh::VertexAttributeVector<double>& vcv);

/** Compute mean curvature for all vertices. The second argument contains the mean curvature
 upon return. */
void compute_mean_curvature(const HMesh::Manifold& m, HMesh::VertexAttributeVector<double>& vcv);

#endif /* defined(__MeshEditGLUT__curvature__) */
