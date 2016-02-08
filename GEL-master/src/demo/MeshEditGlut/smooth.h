//
//  smooth.h
//  MeshEditGLUT
//
//  Created by J. Andreas Bærentzen on 07/09/15.
//  Copyright (c) 2015 J. Andreas Bærentzen. All rights reserved.
//

#ifndef __MeshEditGLUT__smooth__
#define __MeshEditGLUT__smooth__

#include <GEL/HMesh/HMesh.h>

/** Smooth mesh. The first argument is the mesh, and the second argument indicates
 whether implicit or explicit smoothing is desired. The final argument is the step length. */
void mean_curvature_smooth(HMesh::Manifold& m, bool implicit, double step);


#endif /* defined(__MeshEditGLUT__smooth__) */
