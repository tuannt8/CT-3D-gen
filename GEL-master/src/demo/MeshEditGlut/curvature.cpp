//
//  curvature.cpp
//  MeshEditGLUT
//
//  Created by J. Andreas Bærentzen on 04/09/15.
//  Copyright (c) 2015 J. Andreas Bærentzen. All rights reserved.
//

#include "curvature.h"

#include <Eigen/Sparse>

#include <GEL/HMesh/HMesh.h>
#include <GEL/GLGraphics/ManifoldRenderer.h>

using namespace std;
using namespace CGLA;
using namespace HMesh;
using namespace GLGraphics;

using namespace Eigen;

bool is_obtuse(Vec3d p0, Vec3d p1, Vec3d p2)
{
    return (dot(p1-p0, p2-p0) < 0)
            or (dot(p0-p1, p2-p1) < 0)
            or (dot(p1-p2, p0-p2) < 0);
}

double mixed_area(const Manifold& m, VertexID v)
{
    double area_mixed = 0;

    for (auto w = m.walker(v); !w.full_circle(); w = w.circulate_vertex_ccw())
    {
        if(w.face() != HMesh::InvalidFaceID)
        {
            double A_f = abs(area(m, w.face()));

            auto pi = m.pos(w.opp().vertex());
            auto pj = m.pos(w.opp().next().vertex());
            auto pk = m.pos(w.opp().next().next().vertex());

            double theta = acos(dot(pj-pi, pk-pi)
                                / (pj-pi).length()
                                / (pk-pi).length() );

            if(!is_obtuse(pi, pj, pk))
            {
                double cos_alpha_ij = dot(pi - pj, pk - pj)
                                        / (pi-pj).length()
                                        / (pk-pj).length();
                double cos_beta_jk = dot(pi - pk, pj - pk)
                                        /(pi-pk).length()
                                        /(pj - pk).length();
                area_mixed += 1./8. * ( cos_alpha_ij * pow((pi-pj).length(), 2)
                                        + cos_beta_jk * pow((pi-pk).length(), 2));
            }
            else if (theta > M_PI_2)
            {
                area_mixed += A_f / 2;
            }
            else
            {
                area_mixed += A_f / 4;

            }
        }
    }
    
    
    return area_mixed;
}

Vec3d mean_curvature_normal(const Manifold& m, VertexID v)
{
    Vec3d curv_normal(0);
    
    double total_area = 0.;
    for (auto w = m.walker(v); !w.full_circle(); w = w.circulate_vertex_ccw()) {
        
        total_area += area(m, w.face());
        
        // alpha
        auto pai = m.pos(w.vertex()) - m.pos(w.next().vertex());
        auto paj = m.pos(w.opp().vertex()) - m.pos(w.next().vertex());
        auto cota = dot(pai, paj) / cross(pai, paj).length();
        
        // beta
        auto pbi = m.pos(w.vertex()) - m.pos(w.opp().next().vertex());
        auto pbj = m.pos(w.opp().vertex()) - m.pos(w.opp().next().vertex());
        auto cotb = dot(pbi, pbj) / cross(pbi, pbj).length();
        
        auto pij = m.pos(w.vertex()) - m.pos(w.opp().vertex());
        curv_normal += pij*(cota + cotb);
    }
    
    curv_normal /= (4*total_area);
    
    return curv_normal;
}

double compute_gaussian_curvature(const Manifold& m, VertexAttributeVector<double>& vcv)
{
    double integral_gaussian_curvature = 0;
    
    vcv.resize(m.no_vertices());
    
    for (auto vkey : m.vertices())
    {
        double total_theta = 0.;
        for (auto w = m.walker(vkey); !w.full_circle(); w = w.circulate_vertex_ccw())
        {
            if(w.face() != HMesh::InvalidFaceID)
            {
                auto pi = m.pos(w.opp().vertex());
                auto pj = m.pos(w.opp().next().vertex());
                auto pk = m.pos(w.opp().next().next().vertex());

                double theta = acos(dot(pj-pi, pk-pi)
                                    / (pj-pi).length()
                                    / (pk-pi).length() );
                
                total_theta += theta;
            }
        }
        
        auto area_mix = mixed_area(m, vkey);
        assert(area_mix > 0);
        
        vcv[vkey] = (2*M_PI - total_theta) / area_mix;
        
        integral_gaussian_curvature += (2*M_PI - total_theta);
    }

    return integral_gaussian_curvature;
    
}



void compute_mean_curvature(const Manifold& m, VertexAttributeVector<double>& curvature)
{
    for(VertexIDIterator v = m.vertices_begin(); v != m.vertices_end(); ++v)
        if(!boundary(m,*v))
        {
            Vec3d N = -mean_curvature_normal(m, *v);
            curvature[*v] = length(N) * sign(dot(N,Vec3d(normal(m, *v))));
        }
}
