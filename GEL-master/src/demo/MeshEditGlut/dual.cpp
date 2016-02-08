#include "dual.h"

using namespace std;
using namespace HMesh;
using namespace CGLA;

void dual(HMesh::Manifold& m)
{
    FaceAttributeVector<Vec3d> face_center(m.no_faces());
    for (auto f : m.faces()) {
        // find mid point
        Vec3d mpt(0.0, 0.0, 0.0);
        int nb_p = 0;
        for (auto hw = m.walker(f); !hw.full_circle(); hw = hw.circulate_face_ccw()) {
            mpt += m.pos(hw.vertex());
            nb_p++;
        }
        mpt = mpt / nb_p;
        
        
        face_center[f] = mpt;
    }
    
    Manifold newMesh;
    for (auto v : m.vertices()) {
        vector<Vec3d> pts;
        for (auto hw = m.walker(v); !hw.full_circle(); hw = hw.circulate_vertex_ccw()) {
//            if (hw.opp().face() == InvalidFaceID)
//            {
//                pts.push_back((m.pos(hw.vertex()) + m.pos(hw.opp().vertex()))/2.0);
//                pts.push_back(face_center[hw.face()]);
//            }
//            else if(hw.face() == InvalidFaceID)
//            {
//                pts.push_back((m.pos(hw.vertex()) + m.pos(hw.opp().vertex()))/2.0);
//            }
//            else
            if (m.in_use(hw.face()))
            {
                pts.push_back(face_center[hw.face()]);
            }
        }
        
        newMesh.add_face(pts);
    }
    
    stitch_mesh(newMesh, 0.01);
    
    m = newMesh;
 
}