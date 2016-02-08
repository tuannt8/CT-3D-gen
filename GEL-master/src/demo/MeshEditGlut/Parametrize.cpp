//
//  Parametrize.cpp
//  MeshEditE
//
//  Created by J. Andreas Bærentzen on 25/05/15.
//  Copyright (c) 2015 J. Andreas Bærentzen. All rights reserved.
//

#include "Parametrize.h"


#include <string>
#include <fstream>
#include <vector>
#include <GEL/CGLA/Vec3d.h>

#include <GEL/HMesh/Manifold.h>
#include <GEL/HMesh/AttributeVector.h>
#include <GEL/GLGraphics/ManifoldRenderer.h>
#include <Eigen/Sparse>
#include <GEL/CGLA/Mat2x3d.h>
#include <GEL/CGLA/Mat2x2d.h>


using namespace std;
using namespace CGLA;
using namespace HMesh;
using namespace GLGraphics;
using namespace Eigen;

/** Global variable that stores the uv coords. This is not desirable.
    A mechanism for storing a uv coordinate set with each mesh should be devised */
VertexAttributeVector<Vec2d> uv_coords;

/** Given valid uv coords, this function flattens the mesh by copying UV to XY of vertex 
    positions and setting Z to 0 */
void flatten(Manifold& m, double scale) {
    for(auto v: m.vertices())
    {
        m.pos(v)[0] = uv_coords[v][0]*scale;
        m.pos(v)[1] = uv_coords[v][1]*scale;
        m.pos(v)[2] = 0;
    }
}


/** Compute stretch according to the Sander et al. 2002 method */
double compute_stretch(Manifold& m){
    double vis_data_max = 0;
    double A_sum = 0;
    double At_sum = 0;
    FaceAttributeVector<double> stretch(m.no_faces(), 0);
    for(FaceID f: m.faces()) {
        double s[3],t[3];
        Vec3d q[3];
        int l=0;
        
        // Record s and t coords for each vertex
        // also position q.
        circulate_face_ccw(m, f, static_cast<std::function<void(VertexID)>>([&](VertexID v){
            s[l] = uv_coords[v][0];
            t[l] = uv_coords[v][1];
            q[l] = m.pos(v);
            ++l;
        }));
        
        // Compute L2 stretch
        double At = area(m, f);
        
        // STUDENT TASK BEGIN (compute stretch)

        double A = abs(((s[1]-s[0])*(t[2]-t[0])-(s[2]-s[0])*(t[1]-t[0]))/2.0);
        Vec3d Ss = (q[0]*(t[1]-t[2])+q[1]*(t[2]-t[0])+q[2]*(t[0]-t[1]))/(2.0*A);
        Vec3d St = (q[0]*(s[2]-s[1])+q[1]*(s[0]-s[2])+q[2]*(s[1]-s[0]))/(2.0*A);
        double a = dot(Ss,Ss);
        double b = dot(Ss,St);
        double c = dot(St,St);
        double G = sqrt(((a+c)+sqrt((a-c)*(a-c)+4*b*b))/2);
        double g = sqrt(((a+c)-sqrt((a-c)*(a-c)+4*b*b))/2);
        double L2 = sqrt((a+c)/2);
        
        // STUDENT TASK END
        stretch[f] = L2;
        A_sum += A;
        At_sum += At;
    }
    double stretch_norm = sqrt(A_sum/At_sum);
    for(FaceID f: m.faces())
        stretch[f] *= stretch_norm;
    for(FaceID f: m.faces()) {
        float s = stretch[f];
        if(s>1.0f) {
            float gb = max(0.0f,2.0f-s);
            DebugRenderer::face_colors[f] = Vec3f(1, gb, gb);
        }
        else {
            float rg = 2.0f*max(0.0f,s-0.5f);
            DebugRenderer::face_colors[f] = Vec3f(rg,rg,1);
        }
    }
    
    return A_sum;
}

void log_mat(SparseMatrix<double> a)
{
    return;
    cout << "Mat log" << endl;
    for (int i = 0; i<a.rows(); i++)
    {
        for (int j = 0; j<a.cols(); j++)
        {
            cout << a.coeff(i, j) << " ";
        }
        cout << endl;
    }
}

void log_vec(VectorXd b)
{
    return;
    cout << "Vector log " << endl;
    for (int i = 0; i < b.size(); i++)
    {
        cout << b[i] << " ";
    }
    cout << endl;
}

void parametrize_LSCM(Manifold& m, VertexAttributeVector<int>& selected)
{
    /// We use Eigen, typedef the vector and matrix types used below.
    using EigMat = SparseMatrix<double>;
    using EigVec = VectorXd;
    
    const int CONSTRAINTS = 2;                           // Number of boundary constraints we will be using: 2.
    const int COL_DIM = 2*m.no_vertices();               // Number of columns in linear system we need to solve
    const int ROW_DIM = 2*(m.no_faces()+CONSTRAINTS);    // Number of rows.
    
    EigMat M(ROW_DIM,COL_DIM);       // Matrix for the linear system we need to solve
    EigVec B(ROW_DIM);               // Right hand side.
    VertexAttributeVector<int> num;  // This attribute vector is used to enumerate vertices
    
    // This initialization may not need in Windows
    for (int i = 0; i < ROW_DIM; i++)
    {
        B[i] = 0.0;
    }
    
    // Create a matrix row for each constraint. These two rows fix the position of the two selected vertices.
    int constraint_no=0, idx=0;
    for(auto v: m.vertices()) {
        num[v] = idx++;
        //printf("indx: %d", (num[v]));
        if(selected[v] && constraint_no<CONSTRAINTS) {
            
            int row = 2*(m.no_faces()+constraint_no);
            // STUDENT TASK BEGIN (Compute constraint rows of LSCM lin system)
            
            B[row] = -1.0 + (2.0 * constraint_no);
            B[row + 1] = 0.0;
            
            M.insert(row, num[v] * 2) = 1;
            //M.insert(row, 2 * num[v] + 1) = 1;
            
            //M.insert(row + 1, num[v] * 2) = 1;
            M.insert(row + 1, 2 * num[v] + 1) = 1;
            
            // STUDENT TASK END
            ++constraint_no;
            
        }
    }
    
    log_vec(B);
    
    // Create the rest of the linear system which expresses that the gradients of the u parameter coordinate must be orthogonal
    // to the gradient of the v parameter coordinate.
    int row = 0;
    for(FaceID f: m.faces())
    {
        Vec3d x[3]; // 3D positions of triangle vertices
        Vec3i idx;  // Indices of triangle vertices
        int l=0;
        // For each vertex of a face we store the vertex coordinates and indices.
        circulate_face_ccw(m, f, static_cast<std::function<void(VertexID)>> ([&](VertexID vf){
            x[l]=m.pos(vf);
            idx[l] = num[vf];
            ++l;
        }));
        
        // Create rows of linear system.
        
        // STUDENT TASK BEGIN (Compute rows of LSCM lin sys)
        
        // sub space
        Vec3d X = cond_normalize(x[1] - x[0]);
        Vec3d n = normal(m, f);
        Vec3d Y = cond_normalize( cross(n, X) );
        // Projected points
        Vec3d p_i = ((dot(x[0], X)/ dot(X,X)) * Vec3d(1.0,0.0,0.0)) + ((dot(x[0], Y) / dot(Y, Y)) * Vec3d(0.0, 1.0, 0.0));
        Vec3d p_j = ((dot(x[1], X) / dot(X, X))* Vec3d(1.0, 0.0, 0.0)) + ((dot(x[1], Y) / dot(Y, Y)) * Vec3d(0.0, 1.0, 0.0));
        Vec3d p_k = ((dot(x[2], X) / dot(X, X))* Vec3d(1.0, 0.0, 0.0)) + ((dot(x[2], Y) / dot(Y, Y)) * Vec3d(0.0, 1.0, 0.0));
        // Calculate and insert into M
        double a = 1.0 / (2.0 * area(m, f));
        
        M.insert(row, 2 * idx[0]) = a * (p_j[1] - p_k[1]);
        M.insert(row, 2 * idx[0] + 1) = -(a * (p_k[0] - p_j[0]));
        
        M.insert(row, 2 * idx[1]) = a * (p_k[1] - p_i[1]);
        M.insert(row, 2 * idx[1] + 1) = -(a * (p_i[0] - p_k[0]));
        
        M.insert(row, 2 * idx[2]) = a * (p_i[1] - p_j[1]);
        M.insert(row, 2 * idx[2] + 1) = -(a * (p_j[0] - p_i[0]));
        
        M.insert(row + 1, 2 * idx[0]) = a * (p_k[0] - p_j[0]);
        M.insert(row + 1, 2 * idx[0] + 1) = a * (p_j[1] - p_k[1]);
        
        M.insert(row + 1, 2 * idx[1]) = a * (p_i[0] - p_k[0]);
        M.insert(row + 1, 2 * idx[1] + 1) = a * (p_k[1] - p_i[1]);
        
        M.insert(row + 1, 2 * idx[2]) = a * (p_j[0] - p_i[0]);
        M.insert(row + 1, 2 * idx[2] + 1) = a * (p_i[1] - p_j[1]);
        
        // STUDENT TASK END
        
        row += 2;
    }
    
    // Solve the linear system in the least squares sense by constructing the
    // normal equations and solving them using the conjugate gradient method
    EigMat MtM = M.transpose()*M;
    ConjugateGradient<EigMat> cg(MtM);
    EigVec MtB = M.transpose()*B;
    EigVec X = cg.solve(MtB);
    
    // Extract parametrization from solution (trivial)
    for(auto v: m.vertices())
        uv_coords[v] = Vec2d(X[2*num[v]], X[2*num[v]+1]);
    
    // Compute the stretch for each triangle.
    double area = compute_stretch(m);
    
    // Create checkerboard texture
    for(auto v: m.vertices())
        CheckerBoardRenderer::param[v] = 10.0f*Vec2f(uv_coords[v])/sqrt(area/M_PI);
}

void parametrize_iterative(Manifold& m, WeightScheme ws, VertexAttributeVector<int>& selected)
{
    // Set up edge weights according to scheme.
    HalfEdgeAttributeVector<double> edge_weights(m.allocated_halfedges(), 0);
    for(FaceIDIterator f = m.faces_begin(); f != m.faces_end(); ++f)
    {
        for(Walker wv = m.walker(*f); !wv.full_circle(); wv = wv.circulate_face_ccw())
        {
            HalfEdgeID h = wv.halfedge();
            Vec3d p0(m.pos(wv.opp().vertex()));
            Vec3d p1(m.pos(wv.vertex()));
            Vec3d p2(m.pos(wv.next().vertex()));
            
			if (ws == FLOATER_W){
				double ang = acos(min(1.0, max(-1.0, dot(normalize(p1 - p0), normalize(p2 - p0)))));
				double ang_opp = acos(min(1.0, max(-1.0, dot(normalize(p2 - p1), normalize(p0 - p1)))));
				double l = (p1 - p0).length();
				edge_weights[h] += tan(ang / 2) / l;
				edge_weights[wv.opp().halfedge()] += tan(ang_opp / 2) / l;
			}
			else if (ws == HARMONIC_W){
				double a = acos(min(1.0, max(-1.0, dot(normalize(p0 - p2), normalize(p1 - p2)))));
				double w = max(0.0000001, 0.5 / tan(a));
				edge_weights[h] += w;
				edge_weights[wv.opp().halfedge()] += w;
			}
			else{
				edge_weights[h] = valency(m, wv.opp().vertex());
				edge_weights[wv.opp().halfedge()] = valency(m, wv.vertex());
			}
        }
        
    }
    
    // Enumerate boundary vertices and pin them to unit circle.
    VertexID v0;
    for(auto v: m.vertices())
        if(selected[v])
        {
            v0 = v;
            break;
        }
    // Count number of boundary vertices
    int n = 1;
    Walker boundary_walker = m.walker(v0);
    do{
        ++n;
        boundary_walker = boundary_walker.next();
    }
    while(boundary_walker.vertex() != v0);
    
    // Pin boundary vertices to circle
    int i = 0;
    do{
        double a = 2.0*M_PI*double(i)/n;
        uv_coords[boundary_walker.vertex()] = Vec2d(cos(a), sin(a));
        ++i;
        boundary_walker = boundary_walker.next();
    }
    while(boundary_walker.vertex() != v0);
    
    // Reset all non-boundary texture coords to the origin.
    for(auto v: m.vertices())
        if(!boundary(m, v))
            uv_coords[v] = Vec2d(0.0);
    
    // Solve iteratively
    VertexAttributeVector<Vec2d> new_uv_coords(m.no_vertices());
    for(int i = 0; i < 15000; ++i) {
		for (auto v : m.vertices()) {
			if (boundary(m, v))
				new_uv_coords[v] = uv_coords[v];
			else
			{
				new_uv_coords[v] = Vec2d(0);
				double w_sum = 0;
				std::function<void(Walker&)> f = [&](Walker& wv){
					double w = edge_weights[wv.halfedge()];
					new_uv_coords[v] += w * uv_coords[wv.vertex()];
					w_sum += w;
				};
				circulate_vertex_ccw(m, v, f);
				new_uv_coords[v] /= (w_sum + 1e-10);
			}
		}
		uv_coords = new_uv_coords;
    }
    
    // Checkerboard texture
    for(auto v: m.vertices())
        CheckerBoardRenderer::param[v] = 10.0*Vec2f(uv_coords[v]);
    
    // Compute the stretch for each triangle
    compute_stretch(m);
}
