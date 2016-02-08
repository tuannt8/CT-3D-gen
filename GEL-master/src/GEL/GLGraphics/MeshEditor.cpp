//
//  MeshEditor.cpp
//  GEL
//
//  Created by J. Andreas Bærentzen on 09/10/13.
//
//
#include <thread>
#if !defined (WIN32)
#include <stdarg.h>
#include <unistd.h>
#endif
#include <GL/glew.h>
#include <functional>
#include "MeshEditor.h"
#include <string>
#include <iostream>
#include <vector>
#include <algorithm>
#include <queue>

#include <GLGraphics/Console.h>
#include <GLGraphics/glsl_shader.h>
#include <GLGraphics/ShadowBuffer.h>

#include <CGLA/eigensolution.h>
#include <CGLA/Vec2d.h>
#include <CGLA/Vec3d.h>
#include <CGLA/Mat3x3d.h>
#include <CGLA/Mat2x2d.h>
#include <CGLA/Mat2x3d.h>
#include <CGLA/Mat4x4d.h>

#include <HMesh/Manifold.h>
#include <HMesh/AttributeVector.h>
#include <HMesh/mesh_optimization.h>
#include <HMesh/triangulate.h>
#include <HMesh/load.h>
#include <HMesh/quadric_simplify.h>
#include <HMesh/x3d_save.h>
#include <HMesh/obj_save.h>
#include <HMesh/off_save.h>
#include <HMesh/triangulate.h>
#include <HMesh/cleanup.h>
#include <HMesh/refine_edges.h>

#include <Util/Timer.h>

#include "VisObj.h"

#include "SOIL.h"

using namespace std;
using namespace CGLA;
using namespace HMesh;
using namespace Util;

namespace GLGraphics {
    
    namespace {
        
        bool wantshelp(const std::vector<std::string> & args)
        {
            if(args.size() == 0)
                return false;
            
            string str = args[0];
            
            if(str=="help" || str=="HELP" || str=="Help" || str=="?")
                return true;
            
            return false;
        }

        
        bool wantshelp(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(args.size() == 0)
                return false;
            
            string str = args[0];
            
            if(str=="help" || str=="HELP" || str=="Help" || str=="?")
                return true;
            
            return false;
        }
        
        /// Function that aligns two meshes.
        void console_align(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args)) {
                me->printf("usage: align <dest> <src>");
                me->printf("This function aligns dest mesh with src");
                me->printf("In practice the GLViewController of src is copied to dst.");
                me->printf("both arguments are mandatory and must be numbers between 1 and 9.");
                me->printf("Note that results might be unexpexted if the meshes are not on the same scale");
            }
            
            int dest = 0;
            
            if(args.size()>0){
                istringstream a0(args[0]);
                a0 >> dest;
                --dest;
                
                if(dest <0 || dest>8)
                {
                    me->printf("dest mesh out of range (1-9)");
                    return;
                }
            }
            else
            {
                me->printf("neither source nor destination mesh?!");
                return;
            }
            
            int src = 0;
            if(args.size()>1){
                istringstream a1(args[1]);
                a1 >> src;
                --src;
                
                if(src <0 || src>8)
                {
                    me->printf("src mesh out of range (1-9)");
                    return;
                }
            }
            else
            {
                me->printf("no src mesh?");
                return;
            }
            me->align(src,dest);
        }
    
        void transform_mesh(Manifold& mani, const Mat4x4d& m)
        {
            for(VertexIDIterator vid = mani.vertices_begin(); vid != mani.vertices_end(); ++vid)
                mani.pos(*vid) = m.mul_3D_point(mani.pos(*vid));
        }
        
        void console_scale(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args)) {
                me->printf("usage: scale sx sy sz");
                return;
            }
            
            Vec3d s;
            
            if(args.size() > 0){
                istringstream a0(args[0]);
                a0 >> s[0];
            }
            if(args.size() > 1){
                istringstream a0(args[0]);
                a0 >> s[1];
            }
            if(args.size() > 2){
                istringstream a0(args[0]);
                a0 >> s[2];
            }
            
            me->save_active_mesh();
            transform_mesh(me->active_mesh(),scaling_Mat4x4d(s));
            me->refit();
        }
        
        void console_merge_1_ring(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args)) {
                me->printf("usage: edit.selected.merge_1_ring");
                return;
            }
            me->save_active_mesh();
            Manifold& m = me->active_mesh();
            auto sel = me->get_vertex_selection();
            for(auto v: m.vertices())
                if(m.in_use(v) && sel[v]==1)
                    m.merge_one_ring(v);
        }
        
        
        void console_refit_trackball(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args)) {
                me->printf("display.refit_trackball");
                return;
            }
            me->refit();
        }

        
        void console_test(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args)) {
                me->printf("usage: test");
                return;
            }
            
            me->save_active_mesh();
            me->active_mesh().slit_edges(me->get_vertex_selection());
        }

        void console_save(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args)) {
                me->printf("usage: save <name->x3d|name->obj> ");
                
                return;
            }
            const string& file_name = args[0];
            if(args.size() == 1){
                if(file_name.substr(file_name.length()-4,file_name.length())==".obj"){
                    obj_save(file_name, me->active_mesh());
                    
                    return;
                }
                else if(file_name.substr(file_name.length()-4,file_name.length())==".off"){
                    off_save(file_name, me->active_mesh());
                    
                    return;
                }
                else if(file_name.substr(file_name.length()-4,file_name.length())==".x3d"){
                    x3d_save(file_name, me->active_mesh());
                    
                    return;
                }
                me->printf("unknown format");
                return;
            }
            me->printf("usage: save <name->x3d|name->obj> ");
        }
        
        
        void console_refine_edges(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args)) {
                me->printf("usage: refine.split_edges <length>");
                me->printf("splits edges longer than <length>; default is 0.5 times average length");
                return;
            }
            
            me->save_active_mesh();
            
            float thresh = 0.5f;
            
            if(args.size() > 0){
                istringstream a0(args[0]);
                a0 >> thresh;
            }
            
            float avg_length = average_edge_length(me->active_mesh());
            
            refine_edges(me->active_mesh(), thresh * avg_length);
            
            return;
            
        }
        
        void console_refine_faces(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args)) {
                me->printf("usage: refine.split_faces ");
                me->printf("usage:  Takes no arguments. Inserts a vertex at the centre of each face.");
                
                return;
            }
            me->save_active_mesh();
            
            triangulate_by_vertex_face_split(me->active_mesh());
            
            return;
            
        }
        
        void console_stitch(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args)) {
                me->printf("usage: cleanup.stitch <rad>");
                me->printf("Stitches faces");
                
                return;
            }
            double r = 0.001;
            
            if(args.size() > 0){
                istringstream a0(args[0]);
                a0 >> r;
            }
            
            me->save_active_mesh();
            Manifold& m = me->active_mesh();
            Vec3d c;
            float rad;
            bsphere(m, c, rad);
            stitch_mesh(me->active_mesh(), r * rad);
            return;
        }
        
        void console_remove_duplicates(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args)) {
                me->printf("usage: cleanup.remove_duplicates <rad>");
                me->printf("Removes duplicate vertices and incident faces");
                
                return;
            }
            double r = 0.001;
            
            if(args.size() > 0){
                istringstream a0(args[0]);
                a0 >> r;
            }
            
            me->save_active_mesh();
            Manifold& m = me->active_mesh();
            Vec3d c;
            float rad;
            bsphere(m, c, rad);
            remove_duplicates(me->active_mesh(), r * rad);
            return;
        }

        
        void console_remove_val2(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args)) {
                me->printf("usage: cleanup.remove_val2");
                me->printf("Removes valence 2 vertices");
                
                return;
            }
            me->save_active_mesh();
            Manifold& m = me->active_mesh();
            remove_valence_two_vertices(m);
            return;
        }
        
        
        void console_minimize_dihedral(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args))
            {
                me->printf("usage: optimize.minimize_dihedral <iter> <anneal> <use_alpha> <gamma> ");
                me->printf("Flip edges to minimize dihedral angles.");
                me->printf("Iter is the max number of iterations. anneal tells us whether to use ");
                me->printf("simulated annealing and not greedy optimization. use_alpha (default=true) ");
                me->printf("means to use angle and not cosine of anglegamma (default=4) is the power ");
                me->printf("to which we raise the dihedral angle");
                return;
            }
            me->save_active_mesh();
            
            int iter = 1000;
            if(args.size() > 0)
            {
                istringstream a0(args[0]);
                a0 >> iter;
            }
            
            bool anneal = false;
            if(args.size() > 1)
            {
                istringstream a0(args[1]);
                a0 >> anneal;
            }
            
            bool use_alpha = true;
            if(args.size() > 2)
            {
                istringstream a0(args[2]);
                a0 >> use_alpha;
            }
            
            float gamma = 4.0f;
            if(args.size() > 3)
            {
                istringstream a0(args[3]);
                a0 >> gamma;
            }
            
            
            minimize_dihedral_angle(me->active_mesh(), iter, anneal, use_alpha, gamma);
            return;
        }
        
        void console_maximize_min_angle(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args))
            {
                me->printf("usage: optimize.maximize_min_angle <thresh> <anneal>");
                me->printf("Flip edges to maximize min angle - to make mesh more Delaunay.");
                me->printf("If the dot product of the normals between adjacent faces < thresh");
                me->printf("no flip will be made. anneal selects simulated annealing rather ");
                me->printf("nthan greedy optimization.");
                return;
            }
            me->save_active_mesh();
            
            float thresh = 0.0f;
            if(args.size() > 0)
            {
                istringstream a0(args[0]);
                a0 >> thresh;
            }
            bool anneal = false;
            if(args.size() > 1)
            {
                istringstream a0(args[1]);
                a0 >> anneal;
            }
            maximize_min_angle(me->active_mesh(),thresh,anneal);
            return;
        }
        
        
        void console_optimize_valency(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args))
            {
                me->printf("usage: optimize.valency <anneal> ");
                me->printf("Optimizes valency for triangle meshes. Anneal selects simulated annealing rather than greedy optim.");
                return;
            }
            me->save_active_mesh();
            
            bool anneal = false;
            if(args.size() > 0)
            {
                istringstream a0(args[0]);
                a0 >> anneal;
            }
            optimize_valency(me->active_mesh(), anneal);
            return;
        }

        void console_close_holes(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args))
            {
                me->printf("usage: cleanup.close_holes");
                me->printf("This function closes holes. It simply follows the loop of halfvectors which");
                me->printf("enclose the hole and add a face to which they all point.");
                return;
            }
            me->save_active_mesh();
            
            close_holes(me->active_mesh());
            return;
        }
        
        void console_reload(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args))
            {
                me->printf("usage:  load <file>");
                me->printf("(Re)loads the current file if no argument is given, but");
                me->printf("if an argument is given, then that becomes the current file");
                return;
            }
            me->save_active_mesh();
            
            if(!me->reload_active_from_file(args.size() > 0 ? args[0]:""))
                me->printf("failed to load");
            
            return;
        }
        
        
        void console_add_mesh(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args))
            {
                me->printf("usage:  add_mesh <file>");
                me->printf("Loads the file but without clearing the mesh. Thus, the loaded mesh is added to the");
                me->printf("current model.");
                return;
            }
            me->save_active_mesh();
            
            if(!me->add_to_active_from_file(args.size() > 0 ? args[0]:""))
                me->printf("failed to load");
            
            return;
        }
        
        void console_valid(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args))
            {
                me->printf("usage:  validity");
                me->printf("Tests validity of Manifold");
                return;
            }
            if(valid(me->active_mesh()))
                me->printf("Mesh is valid");
            else
                me->printf("Mesh is invalid - check console output");
            return;
        }
        
        void console_Dijkstra(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args))
            {
                me->printf("usage:  Dijkstra");
                return;
            }
            
            Manifold& m = me->active_mesh();
            
            
            VertexAttributeVector<double> dist(m.allocated_vertices(), DBL_MAX);
            VertexAttributeVector<int> visited(m.allocated_vertices(), 0);
            VertexID v = *m.vertices_begin();
            dist[v]=0;
            priority_queue<pair<double,VertexID>> pq;
            pq.push(make_pair(-dist[v], v));
            double max_dist;
            while(!pq.empty())
            {
                VertexID v = pq.top().second;
                max_dist = dist[v];
                pq.pop();
                
                if(!visited[v]){
                    visited[v]=1;
                    
                    for(Walker w = m.walker(v); !w.full_circle(); w = w.circulate_vertex_ccw())
                        if(!visited[w.vertex()])
                        {
                            double d = dist[v] + length(m, w.halfedge());
                            if(d<dist[w.vertex()]) {
                                dist[w.vertex()] = d;
                                pq.push(make_pair(-d, w.vertex()));
                            }
                        }
                }
            }
            
            for(auto vid : m.vertices()) {
                DebugRenderer::vertex_colors[vid] = Vec3f(1-dist[vid]/max_dist,0,0);
                cout << dist[vid] << endl;
            }
            for(auto fid : m.faces())
                DebugRenderer::face_colors[fid] = Vec3f(0.3);
            
            for(auto hid : m.halfedges()) {
                Walker w = m.walker(hid);
                DebugRenderer::edge_colors[hid] = Vec3f(1.0-max(dist[w.vertex()],dist[w.opp().vertex()])/max_dist,0,0);
            }
            return;
        }

        const Vec3f& get_color(int i)
        {
            static Vec3f ctable[100000];
            static bool was_here;
            gel_srand(0);
            if(!was_here)
            {
                was_here = true;
                ctable[0] = Vec3f(0);
                for(int j=1;j<100000;++j)
                    ctable[j] = Vec3f(0.3)+0.7*normalize(Vec3f(gel_rand(),gel_rand(),gel_rand()));
                ctable[3] = Vec3f(1,0,0);
                ctable[4] = Vec3f(0,1,0);
                ctable[5] = Vec3f(0,0,1);
                ctable[6] = Vec3f(1,0,1);
            }
            return ctable[i%100000];
        }

        void console_info(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args))
            {
                me->printf("usage:  info");
                me->printf("Provides information about mesh.");
                return;
            }
            Vec3d p0, p7;
            bbox(me->active_mesh(), p0, p7);
            stringstream bbox_corners;
            bbox_corners << p0 << " - " << p7 << endl;
            me->printf("Bounding box corners : %s", bbox_corners.str().c_str());
            map<int,int> val_hist;
            
            Manifold& m = me->active_mesh();
            
            double avg_len = 0;
            for(HalfEdgeID h: m.halfedges()) {
                DebugRenderer::edge_colors[h] = Vec3f(0.3);
                avg_len += length(m,h);
            }
            avg_len /= m.no_halfedges();
            me->printf("Avg. edge length: %f", avg_len);
            for(VertexID v: m.vertices())
            {
                int val = valency(m,v);
                DebugRenderer::vertex_colors[v] = get_color(val);
                ++val_hist[val];
                
                if(val != 4)
                    circulate_vertex_ccw(m, v, static_cast<std::function<void(HalfEdgeID)>>([&](HalfEdgeID h){
                        Walker w = m.walker(h);
                        DebugRenderer::edge_colors[h] = Vec3f(1);
                        DebugRenderer::edge_colors[w.opp().halfedge()] = Vec3f(1);
                        while(valency(m, w.vertex())==4) {
                            w = w.next().opp().next();
                            DebugRenderer::edge_colors[w.halfedge()] = Vec3f(1);
                            DebugRenderer::edge_colors[w.opp().halfedge()] = Vec3f(1);
                        }
                    }));
            }
            map<int, int> ngon_hist;
            for(FaceID f: m.faces()) {
                int ne = no_edges(m, f);
                ++ngon_hist[ne];
                DebugRenderer::face_colors[f] = 0.7*get_color(ne);
            }
            
            me->printf("Valency histogram");
            for(map<int,int>::iterator iter = val_hist.begin(); iter != val_hist.end(); ++iter)
            {
                me->printf("%d, %d", iter->first, iter->second);
            }
            
            me->printf("Ngon histogram");
            for(map<int,int>::iterator iter = ngon_hist.begin(); iter != ngon_hist.end(); ++iter)
            {
                me->printf("%d, %d", iter->first, iter->second);
            }

            
            me->printf("Mesh contains %d faces", me->active_mesh().no_faces());
            me->printf("Mesh contains %d halfedges", me->active_mesh().no_halfedges());
            me->printf("Mesh contains %d vertices", me->active_mesh().no_vertices());
            return;
        }
        
        
        void console_simplify(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args))
            {
                me->printf("usage: simplify <fraction> ");
                me->printf("Performs Garland Heckbert (quadric based) mesh simplification.");
                me->printf("The only argument is the fraction of vertices to keep.");
                return;
            }
            me->save_active_mesh();
            
            float keep_fraction;
            if(args.size() == 0)
            {
                me->printf("you must specify fraction of vertices to keep");
                return;
            }
            istringstream a0(args[0]);
            a0 >> keep_fraction;

            bool optimal_positions = true;
            if(args.size() == 2)
            {
                istringstream a1(args[1]);
                a1 >> optimal_positions;
            }
            
            Vec3d p0, p7;
            bbox(me->active_mesh(), p0, p7);
            Vec3d d = p7-p0;
            float s = 1.0/d.max_coord();
            Vec3d pcentre = (p7+p0)/2.0;
            for(VertexIDIterator vi = me->active_mesh().vertices_begin(); vi != me->active_mesh().vertices_end(); ++vi){
                me->active_mesh().pos(*vi) = (me->active_mesh().pos(*vi) - pcentre) * s;
            }
            cout << "Timing the Garland Heckbert (quadric based) mesh simplication..." << endl;
            Timer timer;
            timer.start();
            
            //simplify
            quadric_simplify(me->active_mesh(),keep_fraction,0.0001f,optimal_positions);
            
            cout << "Simplification complete, process time: " << timer.get_secs() << " seconds" << endl;
            
            //clean up the mesh, a lot of edges were just collapsed
            me->active_mesh().cleanup();
            
            for(VertexIDIterator vi = me->active_mesh().vertices_begin(); vi != me->active_mesh().vertices_end(); ++vi)
                me->active_mesh().pos(*vi) = me->active_mesh().pos(*vi)*d.max_coord() + pcentre;
            return;
        }
        
        void console_vertex_noise(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args))
            {
                me->printf("usage: noise.perturb_vertices <amplitude>");
                me->printf("adds a random vector to each vertex. A random vector in the unit cube is generated and");
                me->printf("to ensure an isotropic distribution, vectors outside the unit ball are discarded.");
                me->printf("The vector is multiplied by the average edge length and then by the amplitude specified.");
                me->printf("If no amplitude is specified, the default (0.5) is used.");
                return;
            }
            me->save_active_mesh();
            
            float avg_length = average_edge_length(me->active_mesh());
            
            float noise_amplitude = 0.5f;
            if(args.size() > 0) {
                istringstream a0(args[0]);
                a0 >> noise_amplitude;
            }
            
            gel_srand(0);
            for(VertexIDIterator vi = me->active_mesh().vertices_begin(); vi != me->active_mesh().vertices_end(); ++vi){
                Vec3d v;
                do{
                    v = Vec3d(gel_rand(),gel_rand(),gel_rand());
                    v /= (float)(GEL_RAND_MAX);
                    v -= Vec3d(0.5);
                    v *= 2.0;
                }
                while(sqr_length(v) > 1.0);
                
                v *= noise_amplitude;
                v *= avg_length;
                me->active_mesh().pos(*vi) += v;
            }
            return;
        }
        
        void console_perpendicular_vertex_noise(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args)) {
                me->printf("usage: noise.perturb_vertices_perpendicular <amplitude>");
                me->printf("adds the normal times a random scalar times amplitude times");
                me->printf("times average edge length to the vertex. (default amplitude=0.5)");
                return;
            }
            me->save_active_mesh();
            
            float avg_length = average_edge_length(me->active_mesh());
            
            float noise_amplitude = 0.5;
            if(args.size() > 0)
            {
                istringstream a0(args[0]);
                a0 >> noise_amplitude;
            }
            
            VertexAttributeVector<Vec3d> normals(me->active_mesh().allocated_vertices());
            for(VertexIDIterator vi = me->active_mesh().vertices_begin(); vi != me->active_mesh().vertices_end(); ++vi)
                normals[*vi] = normal(me->active_mesh(), *vi);
            
            gel_srand(0);
            for(VertexIDIterator vi = me->active_mesh().vertices_begin(); vi != me->active_mesh().vertices_end(); ++vi)
            {
                float rval = 0.5-gel_rand() / float(GEL_RAND_MAX);
                me->active_mesh().pos(*vi) += normals[*vi]*rval*noise_amplitude*avg_length*2.0;
            }
            return;
        }
        
        void console_noisy_flips(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args)){
                me->printf("usage:  noise.perturb_topology <iter>");
                me->printf("Perform random flips. iter (default=1) is the number of iterations.");
                me->printf("mostly for making nasty synthetic test cases.");
                return;
            }
            me->save_active_mesh();
            
            int iter = 1;
            if(args.size() > 0){
                istringstream a0(args[0]);
                a0 >> iter;
            }
            
            randomize_mesh(me->active_mesh(),  iter);
            return;
        }

        void console_triangulate(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args)) {
                me->printf("usage:  triangulate");
                me->printf("This function triangulates all non triangular faces of the mesh.");
                me->printf("you may want to call it after hole closing. For a polygon it simply connects");
                me->printf("the two closest vertices in a recursive manner until only triangles remain");
                return;
            }
            me->save_active_mesh();
            
            shortest_edge_triangulate(me->active_mesh());
            me->active_mesh().cleanup();
            valid(me->active_mesh());
            return;
        }
        
        
        void console_remove_caps(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args)) {
                me->printf("usage:  cleanup.remove_caps thresh");
                me->printf("Remove caps (triangles with one very big angle). The thresh argument is the fraction of PI to");
                me->printf("use as threshold for big angle. Default is 0.85. Caps are removed by flipping.");
                return;
            }
            me->save_active_mesh();
            
            float t = 0.85f;
            if(args.size() > 0){
                istringstream a0(args[0]);
                a0 >> t;
            }
            remove_caps(me->active_mesh(), static_cast<float>(M_PI) *t);
            me->active_mesh().cleanup();
            
            return;
        }
        
        void console_remove_needles(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args)){
                me->printf("usage: cleanup.remove_needles <thresh>");
                me->printf("Removes very short edges by collapse. thresh is multiplied by the average edge length");
                me->printf("to get the length shorter than which we collapse. Default = 0.1");
                return;
            }
            me->save_active_mesh();
            
            float thresh = 0.1f;
            if(args.size() > 0){
                istringstream a0(args[0]);
                a0 >> thresh;
            }
            float avg_length = average_edge_length(me->active_mesh());
            remove_needles(me->active_mesh(), thresh * avg_length);
            me->active_mesh().cleanup();
            
            return;
        }
        
        void console_flip_orientation(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args)){
                me->printf("usage: cleanup.flip_orientation");
                me->printf("reorients all faces - flipping normal direction");
                return;
            }
            me->save_active_mesh();
            flip_orientation(me->active_mesh());
            return;
        }

        
        void console_undo(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args)) {
                me->printf("usage:  undo");
                me->printf("This function undoes one operation. Repeated undo does nothing");
                return;
            }
            me->restore_active_mesh();
            return;
        }
        
        void console_save_trackball(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args)) {
                me->printf("usage:  display.save_trackball");
                me->printf("This function saves the trackball to disk");
                return;
            }
            me->save_ball();
            return;
        }

        void console_load_trackball(MeshEditor* me, const std::vector<std::string> & args)
        {
            if(wantshelp(args)) {
                me->printf("usage:  display.load_trackball");
                me->printf("This function loads the trackball from disk");
                return;
            }
            me->load_ball();
            return;
        }

        
    }
    
    
    void MeshEditor::register_console_function(const std::string& name,
                                               const std::function<void(MeshEditor*, const std::vector<std::string>&)>& con_fun,
                                               const std::string& help_txt)
    {
        std::function<void (const std::vector<std::string>&)> f = bind(con_fun, this, placeholders::_1);
        theConsole.reg_cmdN(name, f, help_txt);
    }
    
    void save_painting_no_overwite(int width, int height, std::string folder){
        std::ostringstream s;
        if (folder.length() == 0) {
            s << "scr";
        }
        else {
            s << folder << "/scr";
        }
        
        // protect old file
        int count = 0;
        while (1) {
            count ++;
            std::ostringstream temp, name;
            name << "_" << count << ".png";
            temp << s.str() << name.str();
            FILE *f = fopen(temp.str().c_str(), "r");
            if (f) { // Existed
                fclose(f);
                continue;
            }
            else{
                s << name.str();
                break;
            }
            
            if (count > 100) {
                std::cout << "Name identical" << std::endl;
                return;
            }
        }
        
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        int success = SOIL_save_screenshot(s.str().c_str(), SOIL_SAVE_TYPE_PNG, 0, 0, width, height);
        if(!success)
        {
            std::cout << "ERROR: Failed to take screen shot: " << s.str().c_str() << std::endl;
            return;
        }
    }
    
    void MeshEditor::keyparse(unsigned short key){
        //toggle console with ESC
        if (key == 27)
        {
            console_visible = !console_visible;
            return;
        }
        
        if (console_visible)
        {
            static Timer tim;
            if(key==13)
                tim.start();
            theConsole.keyboard(key);
            if(key == 13)
            {
                active_visobj().post_create_display_list();
                double t = tim.get_secs();
                printf("%f seconds",t);
            }
            return;
        }
        else {
            
            switch(key) {
                    case 'R':
                {
                    theConsole.key_up();
                    theConsole.keyboard(13);
                    active_visobj().post_create_display_list();
                }
                    break;
                case 'q': exit(0);
                case '\033':
                    console_visible = false;
                    break;
                case '1':
                case '2':
                case '3':
                case '4':
                case '5':
                case '6':
                case '7':
                case '8':
                case '9':
                {
                    int w[4];
                    glGetIntegerv(GL_VIEWPORT, w);
                    active = key - '1';
                    active_view_control().reshape(w[2],w[3]);
                }
                    break;
                case 'f': display_smooth_shading = !display_smooth_shading; break;
                case 'w':
                    display_render_mode = "wire"; break;
                case 'n':
                    display_render_mode = "normal"; break;
                case 'i':
                    display_render_mode = "isophotes"; break;
                case 'r':
                    display_render_mode = "reflection"; break;
//                case 'h':
//                    display_render_mode = "harmonics"; break;
                case 't':
                    display_render_mode = "toon"; break;
                case 'g':
                    display_render_mode = "glazed"; break;
                case 'a':
                    display_render_mode = "ambient_occlusion"; break;
                case 'c':
                    display_render_mode = "copper"; break;
                case 'C':
                    display_render_mode = "curvature_lines"; break;
                case 'M':
                    display_render_mode = "mean_curvature"; break;
                case 'G':
                    display_render_mode = "gaussian_curvature"; break;
                case ' ':
                    active_visobj().clear_selection();
                    break;
                case 'P':
                {
                    int w[4];
                    glGetIntegerv(GL_VIEWPORT, w);
                    save_painting_no_overwite(w[2], w[3], "log");
                }
                    break;
                
            }
            
            if(key != '\033') post_create_display_list();
        }
        
    }
    
    void MeshEditor::printf(const char* format, ...)
    {
        //format text
        char buffer[1024];
        va_list args;
        va_start(args, format);
        vsprintf(buffer, format, args);
        va_end(args);
        theConsole.print(buffer);
    }

    void MeshEditor::key_up(){theConsole.key_up();}
    void MeshEditor::key_down(){theConsole.key_down();}
    void MeshEditor::key_left(){theConsole.key_left();}
    void MeshEditor::key_right(){theConsole.key_right();}
    void MeshEditor::key_home(){theConsole.key_home();}
    void MeshEditor::key_end(){theConsole.key_end();}
    
    
    
    void MeshEditor::grab_ball(TrackBallAction action, const CGLA::Vec2i& pos){
        active_view_control().grab_ball(action, pos);
    }
    void MeshEditor::roll_ball(const CGLA::Vec2i& pos){
        active_view_control().roll_ball(pos);
    }
    void MeshEditor::release_ball(){
        active_view_control().release_ball();
    }
    bool MeshEditor::try_spinning_ball(){
        return active_view_control().try_spin();
    }
    
    void MeshEditor::save_ball() {
        ofstream ofs("trackball.bin", ios_base::binary);
        active_view_control().save(ofs);
        
    }
    void MeshEditor::load_ball() {
        ifstream ifs("trackball.bin", ios_base::binary);
        active_view_control().load(ifs);
    
    }

    
    bool MeshEditor::grab_mesh(const CGLA::Vec2i& pos)
    {
        if(depth_pick(pos[0], pos[1], depth))
        {
            dragging = true;
            mouse_x = pos[0];
            mouse_y = pos[1];
            Vec3d p0 = screen2world(mouse_x, mouse_y, depth);
            Manifold& m = active_mesh();
            active_visobj().save_old();
            Vec3d c;
            float r;
            bsphere(m, c, r);
            for(auto vid : m.vertices())
            {
                double l = sqr_length(p0-m.pos(vid));
                weight_vector[vid] = exp(-l/(brush_size*r*r));
            }
            return true;
        }
        return false;
    }
    
    bool MeshEditor::drag_mesh(const CGLA::Vec2i& pos)
    {
        if(dragging)
        {
            Vec3d p0 = screen2world(mouse_x, mouse_y, depth);
            Vec3d p1 = screen2world(pos[0], pos[1], depth);
            Vec3d v = p1-p0;
            Manifold& m = active_mesh();
            for(auto vid : m.vertices())
                m.pos(vid) = active_visobj().mesh_old().pos(vid) + weight_vector[vid] * v;
            post_create_display_list();
            return true;
        }
        return false;
    }
    
    void MeshEditor::release_mesh()
    {
        dragging = false;
    }
    
    
    
    
    mutex parallel_work;
    void MeshEditor::display(int scale){

        // Clear screen.
        glClearColor(0.2, 0.2, 0.2, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        //        parallel_work.lock();
        //        active_visobj().post_create_display_list();
        
        
        // Display object.
        active_visobj().display(display_render_mode, theConsole, display_smooth_shading, display_gamma);
        
        // Draw console
        if(console_visible)
        {
            glUseProgram(0);
            theConsole.display(scale);
        }
        
        // Static variable controlling whether we render shadow at all.
        static Console::variable<int> shadow_enable(0);
        shadow_enable.reg(theConsole, "display.shadow.enable", "");
        if(shadow_enable) {
            // Static variables that control the shadow display created and linked to console
            static Console::variable<float> zenith(1.571);
            static Console::variable<float> azimuth(0);
            static Console::variable<float> shadow_alpha(0.3);
            azimuth.reg(theConsole, "display.shadow.azimuth", "");
            zenith.reg(theConsole, "display.shadow.zenith", "");
            shadow_alpha.reg(theConsole, "display.shadow.shadow_alpha", "");

            
            // Shadow buffer (really a frame buffer object)
            static ShadowBuffer sb;
            
            // String containing fragment program for shadow rendering
            static string shadow_shdr_fp = "#version 120\n"
            "    uniform sampler2DShadow shadow_map;\n"
            "    uniform mat4 Mat;\n"
            "   uniform float shadow_alpha;\n"
            "    varying vec4 ep;\n"
            "    void main()\n"
            "    {\n"
            "        vec4 light_pos =  Mat * ep;\n"
            "        light_pos.z = max(0.001,min(0.999,light_pos.z-0.003));\n"
            "        if(light_pos.x <=0 || light_pos.x >=1 || light_pos.y <=0 || light_pos.y>=1) gl_FragColor = vec4(1);"
            "   else      gl_FragColor= vec4(1.0-shadow_alpha)+shadow_alpha*0.25*(shadow2D(shadow_map, light_pos.xyz+vec3(0.001,-0.001,0))+shadow2D(shadow_map, light_pos.xyz+vec3(0.001,0.001,0))+shadow2D(shadow_map, light_pos.xyz-vec3(0.001,0.001,0))+shadow2D(shadow_map, light_pos.xyz+vec3(-0.001,0.001,0)));\n"
            "    }\n";
            
            // Shader program for shadow rendering is compiled and linked.
            static GLuint prog = 0;
            if(!prog)
            {
                GLuint vp = create_glsl_shader(GL_VERTEX_SHADER,"#version 120\n varying vec4 ep; void main(){ep = gl_Vertex; gl_Position = ftransform();}");
                GLuint fp = create_glsl_shader(GL_FRAGMENT_SHADER, shadow_shdr_fp);
                prog = glCreateProgram();
                
                glAttachShader(prog, vp);
                glAttachShader(prog, fp);
                glLinkProgram(prog);
            }
            
            
            // Setup OpenGL state for lighting - used when rendering 3D object casting shadow
            Vec4f lpos = Vec4f(cos(azimuth)*cos(zenith),sin(zenith),sin(azimuth)*cos(zenith),0);
            glLightfv(GL_LIGHT0, GL_POSITION, lpos.get());
            Vec4f mamb(.8,.8,.8,0);
            Vec4f lamb(.4,.4,.5,0);
            glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mamb.get());
            glLightfv(GL_LIGHT0, GL_AMBIENT, lamb.get());

            // Get old viewport dimensions. Setup rendering to FBO and set viewport
            // dimensions for rendering shadows.
            GLint viewp[4];
            glGetIntegerv(GL_VIEWPORT, viewp);
            sb.enable();
            glViewport(0, 0, 1024, 1024);
            
            // Setup object transformations using old school GL.
            Mat4x4f m;
            float r = active_visobj().get_bsphere_radius();
            glMatrixMode(GL_MODELVIEW);
            glPushMatrix();
            glLoadIdentity();
            glMatrixMode(GL_PROJECTION);
            glPushMatrix();
            glLoadIdentity();
            glOrtho(-2*r, 2*r, -2*r, 2*r, -2*r, 2*r);
            gluLookAt(0.1*r*cos(azimuth)*cos(zenith),0.1*r*sin(zenith),0.1*r*sin(azimuth)*cos(zenith), 0,0,0, 0,1,0);
            
            // Copy the transformation matrix to user code.
            glGetFloatv(GL_PROJECTION_MATRIX, m.get());
            
            // Draw the object in light space.
            draw(active_visobj().mesh());
            
            // Restore transformation matrices.
            glPopMatrix();
            glMatrixMode(GL_MODELVIEW);
            glPopMatrix();
            
            // Restore the usual on screen framebuffer.
            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
            glDrawBuffer(GL_BACK);
            glViewport(0, 0, viewp[2], viewp[3]);
            
            // Use shadow: bind the shadow texture to unit 0.
            sb.bind_textures(0);
            
            // Use the shadow rendering shader program.
            glUseProgram(prog);
            active_visobj().view_control().set_gl_modelview();
            glUniform1i(glGetUniformLocation(prog, "shadow_map"), 0);
            glUniform1f(glGetUniformLocation(prog, "shadow_alpha"), shadow_alpha);
            m = translation_Mat4x4f(Vec3f(0.5)) * scaling_Mat4x4f(Vec3f(0.5)) * transpose(m);
            glUniformMatrix4fv(glGetUniformLocation(prog, "Mat"), 1, 1, m.get());
            
            
            // Setup blending such that the shadow is alpha blended with model.
            glDepthFunc(GL_LEQUAL);
            glEnable(GL_BLEND);
            glBlendFunc(GL_ZERO, GL_SRC_COLOR);

            // Draw ground plane for shadows.
            Vec3d p0, p7;
            bbox(active_visobj().mesh(), p0, p7);
            glBegin(GL_QUADS);
            glVertex3f(-100*r, p0[1],-100*r);
            glVertex3f(-100*r, p0[1],100*r);
            glVertex3f(100*r, p0[1],100*r);
            glVertex3f(100*r, p0[1],-100*r);
            glEnd();
            
            // Draw model again ... just to add shadow.
            draw(active_visobj().mesh());
            
            // Disable blending and shader program.
            glDisable(GL_BLEND);
            glUseProgram(0);
        }
        //        parallel_work.unlock();
        //        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    void MeshEditor::reshape(int w, int h) {
        for(VisObj& v : vo)
            v.view_control().reshape(w, h);
    }

    void MeshEditor::init() {
        glewInit();
        
        GLint vp[4];
        glGetIntegerv(GL_VIEWPORT, vp);
        for(VisObj& vis_obj : vo)
            vis_obj.view_control().reshape(vp[2], vp[3]);
        
//        glEnable(GL_CULL_FACE);
//        glCullFace(GL_BACK);
        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
        glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1);
        
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glClearColor(1,1,0, 0.f);
        glColor4f(1.0f, 1.0f, 1.0f, 0.f);
        float material[4] = {1,1,1,1};
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, material);
        glEnable(GL_DEPTH_TEST);
        
        register_console_function("simplify", console_simplify,"");
        
        
        register_console_function("optimize.valency", console_optimize_valency,"");
        register_console_function("optimize.minimize_dihedral_angles", console_minimize_dihedral,"");
        register_console_function("optimize.maximize_min_angle", console_maximize_min_angle,"");
        register_console_function("cleanup.close_holes", console_close_holes,"");
        register_console_function("load_mesh", console_reload,"");
        register_console_function("add_mesh", console_add_mesh,"");
        
        register_console_function("cleanup.stitch", console_stitch,"");
        register_console_function("cleanup.remove_duplicates", console_remove_duplicates,"");
        register_console_function("cleanup.remove_val2", console_remove_val2, "");
        register_console_function("cleanup.flip_orientation", console_flip_orientation,"");
        register_console_function("cleanup.remove_caps", console_remove_caps,"");
        register_console_function("cleanup.remove_needles", console_remove_needles,"");
        register_console_function("triangulate", console_triangulate,"");
        register_console_function("refine.split_edges", console_refine_edges,"");
        register_console_function("refine.split_faces", console_refine_faces,"");
        register_console_function("save_mesh", console_save,"");
        register_console_function("noise.perturb_vertices", console_vertex_noise,"");
        register_console_function("noise.perturb_vertices_perpendicular", console_perpendicular_vertex_noise,"");
        register_console_function("noise.perturb_topology", console_noisy_flips,"");
        
        register_console_function("align", console_align,"");
        register_console_function("undo", console_undo,"");
        
        register_console_function("validity", console_valid,"");
        register_console_function("info", console_info,"");
        
        register_console_function("Dijkstra", console_Dijkstra,"");
        
        register_console_function("display.refit_trackball", console_refit_trackball, "Resets trackball");
        register_console_function("display.save_trackball", console_save_trackball, "Saves trackball to disk");
        register_console_function("display.load_trackball", console_load_trackball, "Load trackball to disk");
        
        register_console_function("transform.scale", console_scale, "Scale mesh");
        register_console_function("edit.selected.merge_1_ring", console_merge_1_ring, "Merge 1-ring of selected vertices");
        
        register_console_function("test", console_test, "Test some shit");
        active.reg(theConsole, "active_mesh", "The active mesh");
        display_render_mode.reg(theConsole, "display.render_mode", "Display render mode");
        brush_size.reg(theConsole, "brush_size", "Size of brush used for editing");
        display_smooth_shading.reg(theConsole, "display.smooth_shading", "1 for smooth shading 0 for flat");
        display_gamma.reg(theConsole, "display.gamma", "The gamma setting for the display");

        theConsole.print("Welcome to MeshEdit");
        theConsole.newline();
    }
    
    bool MeshEditor::add_file(const std::string& str)
    {
        while (active_mesh().no_vertices()>0 && active<NO_MESHES)
            active  = active + 1;
        if(active == NO_MESHES)
            active = 0;
        if(active_visobj().reload(str)) {
            active_visobj().post_create_display_list();
            return true;
        }
        return false;
    }

    bool MeshEditor::reload_active_from_file(const std::string& str)
    {
        if(active_visobj().reload(str)) {
            active_visobj().post_create_display_list();
            return true;
        }
        return false;
    }
    
    bool MeshEditor::add_to_active_from_file(const std::string& str)
    {
        if(active_visobj().add_mesh(str)) {
            active_visobj().post_create_display_list();
            return  true;
        }
        return false;
    }

    
}
