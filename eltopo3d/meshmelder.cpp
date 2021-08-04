#include <meshmelder.h>

#include <broadphase.h>
#include <collisionpipeline.h>
#include <collisionqueries.h>
#include <queue>
#include <runstats.h>
#include <surftrack.h>
#include <vector>

#include <solve_intersections.h>
#include <igl/copyleft/cgal/mesh_boolean.h>

igl::MeshBooleanType boolean_type(
    igl::MESH_BOOLEAN_TYPE_UNION);

Eigen::MatrixXd p_verts;
Eigen::MatrixXi p_faces;

void MeshMelder::process_mesh()
{

    std::vector<Intersection> intersections;
    m_surf.m_collision_pipeline.get_intersections(false, false, intersections);

    if (intersections.empty())
    {
        //return;
    }

    const std::vector<Vec3d> &positions = m_surf.get_positions();
    const std::vector<Vec3st> &triangles = m_surf.m_mesh.get_triangles();
    
    std::vector<double> coords(positions.size() * 3);
    std::vector<uint> tris(triangles.size() * 3);

    std::vector<double> out_coords;
    std::vector<uint> out_tris;
    solveIntersections(coords, tris, out_coords, out_tris);

    Eigen::MatrixXd verts(out_coords.size(), 3);
    Eigen::MatrixXi faces(out_tris.size(), 3);

    Eigen::MatrixXd r_verts;
    Eigen::MatrixXi r_faces;

    for (int i = 0; i < out_coords.size(); i += 1)
    {
        for (int j = 0; j < 3; j += 1)
        {
            verts(i, j) = out_coords[i * 3 + j];
        }
    }

    for (int i = 0; i < out_tris.size(); i += 1)
    {
        for (int j = 0; j < 3; j += 1)
        {
            faces(i, j) = out_tris[i * 3 + j];
        }
    }

    igl::copyleft::cgal::mesh_boolean(verts, faces, p_verts, p_faces, boolean_type, r_verts, r_faces);

    std::vector<Vec3d> new_pos;
    std::vector<Vec3st> new_tris;

    new_pos.resize(r_verts.rows());
    new_tris.resize(r_faces.rows());

    for (int i = 0; i < new_pos.size(); i += 1)
    {
        for (int j = 0; j < 3; j += 1)
        {
            new_pos[i][j] = r_verts(i, j);
        }
    }

    for (int i = 0; i < new_tris.size(); i += 1)
    {
        for (int j = 0; j < 3; j += 1)
        {
            new_tris[i][j] = r_faces(i, j);
        }
    }

    m_surf.set_all_newpositions(new_pos);

    m_surf.set_positions_to_newpositions();

    m_surf.m_mesh.set_num_vertices(m_surf.get_num_vertices());
    m_surf.m_mesh.replace_all_triangles(new_tris);
    
}