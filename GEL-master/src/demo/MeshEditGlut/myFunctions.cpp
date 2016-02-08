//
//  myFunctions.cpp
//  MeshEditGLUT
//
//  Created by Tuan Nguyen Trung on 10/29/15.
//  Copyright © 2015 J. Andreas Bærentzen. All rights reserved.
//

#include "myFunctions.h"
#include "dual.h"
#include <GEL/Geometry/KDTree.h>
#include <Eigen/SVD>
#include <Eigen/LU>
#include <GEL/Geometry/build_bbtree.h>
#include "CImg/CImg.h"
#include <sys/stat.h>

using namespace std;
using namespace HMesh;
using namespace CGLA;
using namespace Geometry;
using namespace Eigen;

void doo_sabin(HMesh::Manifold& m)
{
    HMesh::Manifold newMesh;
    
    for (auto fkey : m.faces())
    {
        Vec3d center(0.0);
        int num = 0;
        for (auto hew = m.walker(fkey); !hew.full_circle(); hew = hew.circulate_face_cw())
        {
            center += m.pos(hew.vertex());
            num++;
        }
        center = center / num;
        
        for (auto hew = m.walker(fkey); !hew.full_circle(); hew = hew.circulate_face_ccw())
        {
            auto nextE = hew.circulate_face_ccw();
            vector<Vec3d> pts;
            pts.push_back(m.pos(hew.vertex()));
            pts.push_back((m.pos(nextE.vertex()) + m.pos(nextE.opp().vertex()))/2.0);
            pts.push_back(center);
            pts.push_back((m.pos(hew.vertex()) + m.pos(hew.opp().vertex()))/2.0);
            
            newMesh.add_face(pts);
        }
    }
    
    stitch_mesh(newMesh, 0.01);
    
    dual(newMesh);
    
    m = newMesh;
}

void iterative_closest_point(Manifold& m1, Manifold& m2)
{
    cout << "---" << endl << "Object vertices: ";
    cout << m1.no_vertices() << " " << m2.no_vertices() << endl;
    
    // Build Kd tree on m2. Move m1 (P)
    KDTree<Vec3d, string> tree;
    for (auto vid2 : m2.vertices())
    {
        tree.insert(m2.pos(vid2), "");
    }
    tree.build();

    
    for (int iter = 0; iter < 100; iter++)
    {
        // Fixed P
        MatrixXf P = MatrixXf::Random(3, m1.no_vertices());
        Vec3d center1(0.0);
        for (auto vid1 : m1.vertices())
        {
            auto p = m1.pos(vid1);
            center1 += p;
        }
        center1 /= m1.no_vertices();
        
        int i = 0;
        for (auto vid1 : m1.vertices())
        {
            auto p = m1.pos(vid1) - center1;
            P(0,i) = p[0];
            P(1,i) = p[1];
            P(2,i) = p[2];
            i++;
        }
        P.transposeInPlace();
        Vector3f center_p(center1[0], center1[1], center1[2]);
        
        // Center of m2
        Vec3d center2(0.0);
        for (auto vid2 : m2.vertices())
        {
            center2 += m2.pos(vid2);
        }
        center2 /= m2.no_vertices();
        Vector3f center_q(center2[0], center2[1], center2[2]);
        
        // Finding closest point
        MatrixXf Q = MatrixXf::Random(3, m2.no_vertices());
        i = 0;
        for (auto vid1 : m1.vertices())
        {
            
            auto p1 = m1.pos(vid1);
            Vec3d p_closest;
            
            string temp_s; double dis=100;
//            assert(tree.closest_point(p1, dis, p_closest, temp_s));
            
            // Find closest point
            double shortest_dis = INFINITY;
            for (auto vid2 : m2.vertices())
            {
                auto p2 = m2.pos(vid2);
                double dis = (p1 - p2).length();
                if (dis < shortest_dis)
                {
                    shortest_dis = dis;
                    p_closest = p2;
                }
            }
            assert(shortest_dis != INFINITY);
            
            // add to matrix Q
            p_closest = p_closest - center2;
            Q(0,i) = p_closest[0];
            Q(1,i) = p_closest[1];
            Q(2,i) = p_closest[2];
            
            i++;
        }
        
        // Solve transformation matrix
        MatrixXf S = Q*P;
        JacobiSVD<MatrixXf> svd(S, ComputeFullU|ComputeFullV);
        auto U = svd.matrixU();
        auto V = svd.matrixV();
        
        Matrix3f ttm;
        float det_u_vt = (U*V.transpose()).determinant();
        ttm << 1, 0, 0,
                0, 1, 0,
                0, 0, det_u_vt;
        
        Matrix3f R = U*ttm*V.transpose();
        Vector3f t = center_q - R*center_p;
        
        // Perform translation
        for (auto vid1 : m1.vertices())
        {
            auto pt = m1.pos(vid1);
            Vector3f pt_e(pt[0], pt[1], pt[2]);
            Vector3f new_p_e = R*pt_e + t;
            
            Vec3d new_p(new_p_e(0), new_p_e(1), new_p_e(2));
            m1.pos(vid1) = new_p;
        }
    }

    
}

vector<float> intersect(AABBTree &tree, Vec3f origin, Vec3f direct)
{
    vector<float> intersects;
    
    return intersects;
}

typedef cimg_library::CImg<float> cimg_float;
/** Construct image from csene*/
void cross_section_image_gen(HMesh::Manifold& m, std::string out_path, int res)
{
    // Create directory
    mkdir(out_path.c_str(), 0775);
    
    // Construct AABB tree
    AABBTree tree;
    build_AABBTree(m, tree);
    
    // Image bounding
    auto ld = tree.root->get_pmin();
    auto ru = tree.root->get_pmax();
    
    auto diag = ru - ld;

    double gap = 0.1;
    ld -= diag * gap;
    ru += diag * gap;
    diag = ru - ld;
    
    // Scan resolution
    double min_size = min(min(diag[0], diag[1]), diag[2]);
    double delta = min_size / (double)res;
    Vec3i out_res;
    out_res[0] = round(diag[0] / delta);
    out_res[1] = round(diag[1] / delta);
    out_res[2] = round(diag[2] / delta);
    
//    Vec3f pt = ld;// + diag/2.0;
//    Vec3f n(1.0, 0.0, 0.0);
//    auto ts = tree.intersect_all(pt, n);
//    cout << "Intersection " << ts.size() << ": " ;
//    for (auto t : ts)
//        cout << " " << t;
    
    float * img = (float*) malloc(out_res[0] * out_res[1] * sizeof(float));
    for (int zt = 0; zt < out_res[2]; zt++)
    {
        // Each cross section
        memset(img, 0, out_res[0] * out_res[1] * sizeof(float));
        
        float * y_pointer = img;
        
        for (int yt = 0; yt < out_res[1]; yt++)
        {
            Vec3f pt = ld + Vec3f(0, yt*delta, zt*delta);
        //    float * x_pointer = y_pointer;
            Vec3f x_(1.0, 0.0, 0.0);

            auto ts = tree.intersect_all(pt, x_);
            assert(ts.size() % 2 == 0);
            for (int i = 0; i < ts.size() / 2; i++)
            {
                int pt1 = ts[2*i] / delta;
                int pt2 = ts[2*i+1] / delta;
                for (int j = pt1; j < pt2; j++)
                {
                    y_pointer[j] = 255;
                }
            }
            
            y_pointer += out_res[0];
        }
        using namespace cimg_library;
        CImg<float> _img(out_res[0],out_res[1]);
        float * ptr_img = img;
        cimg_for(_img,ptr,float) { *ptr= *(ptr_img++); }
        
        ostringstream out_name;
        out_name << out_path << "/im_" << zt << ".png";
        _img.save(out_name.str().c_str());
    }
}

