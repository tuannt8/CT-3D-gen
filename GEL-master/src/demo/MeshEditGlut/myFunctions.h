//
//  myFunctions.hpp
//  MeshEditGLUT
//
//  Created by Tuan Nguyen Trung on 10/29/15.
//  Copyright © 2015 J. Andreas Bærentzen. All rights reserved.
//

#ifndef myFunctions_hpp
#define myFunctions_hpp


#include <GEL/HMesh/HMesh.h>

/** Subdivision */
void doo_sabin(HMesh::Manifold& m);

/** Register m1 to m2 using ICP*/
void iterative_closest_point(HMesh::Manifold& m1, HMesh::Manifold& m2);

/** Construct image from csene*/
void cross_section_image_gen(HMesh::Manifold& m, std::string out_path, int res);

#endif /* myFunctions_hpp */
