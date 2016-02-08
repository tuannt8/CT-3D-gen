/*
 *  VisObj.cpp
 *  GEL
 *
 *  Created by J. Andreas Bærentzen on 20/09/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include "VisObj.h"

#include <GLGraphics/Console.h>
#include <HMesh/Manifold.h>
#include <HMesh/AttributeVector.h>
#include <HMesh/load.h>

#include <CGLA/Mat3x3d.h>
#include <CGLA/Vec3d.h>
#include <CGLA/Vec4d.h>

using namespace std;
using namespace CGLA;
using namespace HMesh;
using namespace GLGraphics;

int WINX=800, WINY=800;

namespace GLGraphics {
    
    void VisObj::refit()
    {
        bsphere(mani, bsphere_center, bsphere_radius);
        
        view_ctrl.set_centre(Vec3f(bsphere_center));
        view_ctrl.set_eye_dist(2*bsphere_radius);
    }
    
    bool VisObj::reload(string _file)
    {
        if(_file != "") file = _file;
        mani.clear();
        if(!load(file, mani))
            return false;
        refit();
        return true;
    }
    
    bool VisObj::add_mesh(string file)
    {
        if(!load(file, mani))
            return false;
        refit();
        return true;
    }
    
    
    bool VisObj::select_vertex(const CGLA::Vec2i& pos)
    {
        float d;
        if(depth_pick(pos[0], pos[1], d))
        {
            Vec3d c;
            float r;
            bsphere(mani, c, r);
            VertexID closest = InvalidVertexID;
            double min_dist = DBL_MAX;
            for(auto vid : mani.vertices())
            {
                Vec3d wp = world2screen(mani.pos(vid));
                if(sqr_length(Vec2d(wp[0],wp[1])-Vec2d(pos))<100)
                {
                    double dist = sqr_length(screen2world(pos[0], pos[1], d)-mani.pos(vid));
                    if(dist < min_dist)
                    {
                        min_dist = dist;
                        closest = vid;
                    }
                }
            }
            if(closest != InvalidVertexID) {
                vertex_selection.resize(mani.allocated_vertices(),0);
                vertex_selection[closest] = !vertex_selection[closest];
                active_selection = true;
                post_create_display_list();
                return true;
            }
        }
        return false;
        
    }

    void VisObj::produce_renderer(const std::string& display_method , Console& cs, bool smooth, float gamma)
    {
        delete renderer;
        
        string short_name = display_method.substr(0,3);
        
        static Console::variable<int> use_shading(0);
        static Console::variable<int> use_stripes(0);
        static Console::variable<int> color_sign(0);
        if(short_name=="mea"||short_name=="gau"||short_name=="sca")
        {
            use_shading.reg(cs, "display.scalar_field.use_shading", "use shading for scalar field visualization");
            use_stripes.reg(cs, "display.scalar_field.use_stripes", "use stripes for scalar field visualization");
            color_sign.reg(cs, "display.scalar_field.color_sign", "color according to sign when scalar field visualizing");

        }
        
        if(short_name== "wir")
            renderer = new WireframeRenderer(mani, smooth);
        
        else if(short_name == "iso") {
            renderer = new IsophoteLineRenderer();
            renderer->compile_display_list(mani,smooth);
        }
        else if(short_name == "ref") {
            renderer = new ReflectionLineRenderer();
            renderer->compile_display_list(mani,smooth);
        }
        else if(short_name == "gla") {
            renderer = new GlazedRenderer();
            dynamic_cast<GlazedRenderer*>(renderer)->compile_display_list(mani,smooth);
        }
        
        
        else if(short_name == "too") {
            renderer = new ToonRenderer();
            renderer->compile_display_list(mani,smooth);
        }
        else if(short_name == "deb")
        {
            static Console::variable<float> debug_renderer_ball_radius(0.001);
            debug_renderer_ball_radius.reg(cs, "display.debug_renderer.radius","");
            renderer = new DebugRenderer;
            dynamic_cast<DebugRenderer*>(renderer)->compile_display_list(mani, smooth, debug_renderer_ball_radius);
        }
        else if(short_name == "che")
        {
            renderer = new CheckerBoardRenderer;
            renderer->compile_display_list(mani, smooth);
        }
        else if(short_name == "sca")
        {
            double max_G = scalar_field[*mani.vertices_begin()];
            double min_G = max_G;
            for(auto v: mani.vertices()) {
                max_G = max(scalar_field[v], max_G);
                min_G = min(scalar_field[v], min_G);
            }
            renderer = new ScalarFieldRenderer();
            dynamic_cast<ScalarFieldRenderer*>(renderer)->compile_display_list(mani, smooth,scalar_field, min_G, max_G, gamma,use_stripes,color_sign,use_shading);
        }
        else if(short_name == "lin")
        {
            renderer = new LineFieldRenderer();
            dynamic_cast<LineFieldRenderer*>(renderer)->compile_display_list(mani, get_line_field_attrib_vector());
        }
        else {
            renderer = new NormalRenderer();
            renderer->compile_display_list(mani, smooth);
        }

    
    }
    
    void VisObj::draw_selection()
    {
        float r = view_ctrl.get_eye_dist();
        r *= 0.003;
        glDisable(GL_LIGHTING);
        for(auto vid : mani.vertices())
        {
            Vec3d p = mani.pos(vid);
            if(vertex_selection[vid])
                glColor3f(1,1,0);
            else
                glColor3f(0, 0, 0.3);
            glPushMatrix();
            glTranslated(p[0], p[1], p[2]);
            glScalef(r, r, r);
            draw_ball();
            glPopMatrix();
        }
        glEnable(GL_LIGHTING);
    }

    
    void VisObj::display(const std::string& display_method , Console& cs, bool smooth, float gamma)
    {
        if(create_display_list){
            create_display_list = false;
            produce_renderer(display_method, cs, smooth, gamma);
        }
        view_ctrl.set_gl_modelview();
        renderer->draw();
        if(active_selection)
            draw_selection();
    }
}