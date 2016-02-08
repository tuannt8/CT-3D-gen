//
//  additional_console_funcs.cpp
//  MeshEditE
//
//  Created by J. Andreas Bærentzen on 14/10/13.
//  Copyright (c) 2013 J. Andreas Bærentzen. All rights reserved.
//

#include <strstream>
#include <istream>
#include <fstream>
#include <string>
#include <regex>
#include <set>
#include <queue>

#include <GEL/HMesh/Manifold.h>
#include <GEL/HMesh/AttributeVector.h>
#include <GEL/HMesh/mesh_optimization.h>
#include <GEL/HMesh/triangulate.h>
#include <GEL/HMesh/refine_edges.h>
#include <GEL/GLGraphics/MeshEditor.h>

#include "additional_console_funcs.h"
#include "Parametrize.h"
#include "dual.h"
#include "curvature.h"
#include "smooth.h"
#include "myFunctions.h"

using namespace std;
using namespace CGLA;
using namespace GLGraphics;
using namespace HMesh;


void console_parametrize_iterative(MeshEditor* me, const std::vector<std::string> & args)
{
    me->save_active_mesh();
    
    WeightScheme ws = BARYCENTRIC_W;
	if (args.size() > 0){
		if (args[0] == "floater")
			ws = FLOATER_W;
		else if (args[0] == "harmonic")
			ws = HARMONIC_W;
	}
	//else
	//	ws = DEFULD_W;
    //    return;
    
    parametrize_iterative(me->active_mesh(), ws, me->get_vertex_selection());
    
    return;
}

void console_parametrize_lscm(MeshEditor* me, const std::vector<std::string> & args)
{
    me->save_active_mesh();
    parametrize_LSCM(me->active_mesh(), me->get_vertex_selection());
    return;
}


void console_parametrize_flatten(MeshEditor* me, const std::vector<std::string> & args)
{
    double disk_size=1;
    
    if(args.size() > 0){
        istringstream a0(args[0]);
        a0 >> disk_size;
    }
    me->save_active_mesh();
    flatten(me->active_mesh(), disk_size);
    return;
}

void console_dual(MeshEditor* me, const std::vector<std::string> & args)
{
    me->save_active_mesh();
    dual(me->active_mesh());
    return;
}

void console_compute_gaussian_curvature(MeshEditor* me, const std::vector<std::string> & args)
{
    double igc = compute_gaussian_curvature(me->active_mesh(),
                                            me->active_visobj().get_scalar_field_attrib_vector());

    me->printf("Integral Gaussian Curvature: %F\n", igc);
}

void console_compute_mean_curvature(MeshEditor* me, const std::vector<std::string> & args)
{
    compute_mean_curvature(me->active_mesh(),
                           me->active_visobj().get_scalar_field_attrib_vector());
}

void console_mean_curvature_smooth(MeshEditor* me, const std::vector<std::string> & args)
{
    me->save_active_mesh();

    bool implicit = true;
    if(args.size() > 0){
        istringstream a0(args[0]);
        a0 >> implicit;
    }

    double step = 0.0001;
    if(args.size() > 1){
        istringstream a0(args[1]);
        a0 >> step;
    }

    mean_curvature_smooth(me->active_mesh(), implicit, step);
    return;
}

void console_subdivide_doo_sabin(MeshEditor* me, const std::vector<std::string> & args)
{
    doo_sabin(me->active_mesh());
}

void cross_section_construct(MeshEditor* me, const std::vector<std::string> & args)
{
    int im_res = 128;
    if (args.size() > 0)
    {
        istringstream a0(args[0]);
        a0 >> im_res;
    }
    
    string out = "cross_section";
    if (args.size() > 1)
    {
        out = args[1];
    }
    
    cross_section_image_gen(me->active_mesh(), out, im_res);
}

void console_icp(MeshEditor* me, const std::vector<std::string> & args)
{
    int m0_id = 1, m1_id = 2;
    if(args.size()>0) {
        istringstream a0(args[0]);
        a0 >> m0_id;
    }
    if(args.size()>1) {
        istringstream a1(args[1]);
        a1 >> m1_id;
    }
    me->save_active_mesh();
    me->printf("registering mesh %i to %i\n", m0_id, m1_id);
    iterative_closest_point(me->get_mesh(m0_id-1),me->get_mesh(m1_id-1));
    return;
}

void register_console_funcs(MeshEditor* me)
{
    me->register_console_function("dual", console_dual, "");
    me->register_console_function("parametrize.lscm", console_parametrize_lscm, "");
    me->register_console_function("parametrize.iterative", console_parametrize_iterative, "");
    me->register_console_function("parametrize.flatten", console_parametrize_flatten, "");
    me->register_console_function("curvature.gaussian", console_compute_gaussian_curvature,"");
    me->register_console_function("curvature.mean", console_compute_mean_curvature,"");
    me->register_console_function("smooth.mean_curvature", console_mean_curvature_smooth,"");
    me->register_console_function("my.doo_sabin", console_subdivide_doo_sabin,"");
    me->register_console_function("icp", console_icp,"");
    
    me->register_console_function("cross_section.construct", cross_section_construct,"");
}
