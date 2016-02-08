//
//  smooth.cpp
//  MeshEditGLUT
//
//  Created by J. Andreas Bærentzen on 07/09/15.
//  Copyright (c) 2015 J. Andreas Bærentzen. All rights reserved.
//

#include "smooth.h"

#include "curvature.h"

#include <Eigen/Sparse>

#include <GEL/HMesh/HMesh.h>
#include <GEL/GLGraphics/ManifoldRenderer.h>

using namespace std;
using namespace CGLA;
using namespace HMesh;
using namespace GLGraphics;
using namespace Eigen;

void mean_curvature_smooth(Manifold& m, bool implicit, double lambda)
{
    using EigMat = SparseMatrix<double>;
    using EigVec = VectorXd;
    
    int N = (int)m.no_vertices();
    VertexAttributeVector<int> indices(m.allocated_vertices());
    VertexAttributeVector<double> areas(m.allocated_vertices());
    int i=0;
    for(auto v: m.vertices()) {
        indices[v] = i++;
        areas[v] = mixed_area(m, v);
    }
    

    EigMat K(N,N);      // Sparse matrix initialized with 0
    EigVec X(N),Y(N),Z(N);
    EigVec Xp(N), Yp(N), Zp(N);
    
    //-----------------------------------------------------------
    // Student implementation
    //-----------------------------------------------------------
    double epsilon = 1e-5;
    for (auto vkey : m.vertices())
    {
        int i = indices[vkey];
        for (auto w = m.walker(vkey); !w.full_circle(); w = w.circulate_vertex_ccw())
        {
            int j = indices[w.vertex()];
            assert(i != j);
            
            if (i > j
                or w.face() == HMesh::InvalidFaceID
                or w.opp().face() == HMesh::InvalidFaceID)
            {
                continue; // Avoid recomputation
            }
            
            auto pi = m.pos(w.opp().vertex());
            auto pj = m.pos(w.vertex());
            auto pl = m.pos(w.opp().next().vertex());
            auto pk = m.pos(w.next().vertex());
            
            double cot_alpha_ij = dot(pj - pk, pi - pk) /
                                ( cross(pi - pk, pj - pk).length() + epsilon);
            double cot_beta_ij = dot(pj - pl, pi - pl) /
                                ( cross(pi - pl, pj - pl).length() + epsilon);
            
            double Ai = areas[w.opp().vertex()];
            double Aj = areas[w.vertex()];
            
            double Lij = (cot_alpha_ij + cot_beta_ij)
                        / sqrt(Ai*Aj + epsilon);
            
            K.coeffRef(i, j) = Lij;
            K.coeffRef(j, i) = Lij;
            K.coeffRef(i, i) -= Lij;
            K.coeffRef(j, j) -= Lij;
            
        }
    }
    
    EigMat I(N,N);
    for (int i = 0; i < N; i++)
    {
        I.coeffRef(i, i) = 1;
    }
    
    K = I - K*lambda;
    
    
    for (auto vkey : m.vertices())
    {
        auto p = m.pos(vkey);
        int i = indices[vkey];
        X.coeffRef(i) = p[0];
        Y.coeffRef(i) = p[1];
        Z.coeffRef(i) = p[2];
    }
    
    // Solve
    SimplicialLLT<EigMat> solver(K);
    Xp = solver.solve(X);
    Yp = solver.solve(Y);
    Zp = solver.solve(Z);
    
    // End student implementation
    //-----------------------------------------------------------
        
    for(auto v: m.vertices())
    {
        int i = indices[v];
        m.pos(v) = Vec3d(Xp[i], Yp[i], Zp[i]);
    }
}

