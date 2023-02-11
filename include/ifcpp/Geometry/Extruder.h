#pragma once

#include "carve/input.hpp"
#include "carve/triangulator.hpp"
#include "ifcpp/Geometry/Utils.h"
#include "ifcpp/Geometry/csgjs.h"
#include <vector>

namespace ifcpp {

void FindEnclosedLoops( const std::vector<std::vector<csgjscpp::Vector>>& face_loops_input,
                        std::vector<std::vector<std::vector<csgjscpp::Vector>>>& profile_paths_enclosed ) {
    if( face_loops_input.size() > 1 ) {
        const std::vector<csgjscpp::Vector>& loop1 = face_loops_input[ 0 ];
        std::vector<std::vector<csgjscpp::Vector>> enclosed_in_loop1;
        std::vector<std::vector<csgjscpp::Vector>> separate_loops;
        enclosed_in_loop1.push_back( loop1 );

        for( size_t ii = 1; ii < face_loops_input.size(); ++ii ) {
            const std::vector<csgjscpp::Vector>& loop = face_loops_input[ ii ];
            bool loop_enclosed_in_loop1 = IsEnclosed( loop, loop1 );

            if( loop_enclosed_in_loop1 ) {
                enclosed_in_loop1.push_back( loop );
            } else {
                separate_loops.push_back( loop );
            }
        }

        profile_paths_enclosed.push_back( enclosed_in_loop1 );
        if( !separate_loops.empty() ) {
            profile_paths_enclosed.push_back( separate_loops );
        }
    } else {
        profile_paths_enclosed.push_back( face_loops_input );
    }
}

void TriangulateLoops( const std::vector<std::vector<csgjscpp::Vector>>& profile_paths_input,
                       std::vector<std::vector<csgjscpp::Vector>>& face_loops_used_for_triangulation, std::vector<int>& face_indexes_out ) {
    // TODO: complete and test
    if( profile_paths_input.empty() ) {
        // TODO: Log error
        return;
    }

    // figure 1: loops and indexes
    //  3----------------------------2
    //  |                            |
    //  |   1-------------------2    |3---------2
    //  |   |                   |    |          |
    //  |   |                   |    |          |face_loops[2]   // TODO: handle combined profiles
    //  |   0---face_loops[1]---3    |0---------1
    //  |                            |
    //  0-------face_loops[0]--------1

    //csgjscpp::Vector normal_first_loop;
    //bool warning_small_loop_detected = false;
    //bool polyline_created = false;

    for( size_t i_face_loops = 0; i_face_loops < profile_paths_input.size(); ++i_face_loops ) {
        const std::vector<csgjscpp::Vector>& loop_input = profile_paths_input[ i_face_loops ];
        // GeomUtils::copyClosedLoopSkipDuplicates( loop_input, loop_2d );
        std::vector<csgjscpp::Vector> loop_2d = loop_input;
        UnCloseLoop( loop_2d );

        if( loop_2d.size() < 3 ) {
            if( profile_paths_input.size() == 1 ) {
                // Cross section is just a point or a line. Create a face with one index
                face_indexes_out.push_back( static_cast<int>( loop_2d.size() ) ); // num points
                for( size_t ii = 0; ii < loop_2d.size(); ++ii ) {
                    face_indexes_out.push_back( (int)ii ); // point index
                }
                // Nothing to triangulate, so return
                return;
            }
            continue;
        }

        // check winding order
        bool reverse_loop = false;
        csgjscpp::Vector normal_2d = ComputePolygon2DNormal( loop_2d );
        if( i_face_loops == 0 ) {
            //normal_first_loop = normal_2d;
            if( normal_2d.z < 0 ) {
                reverse_loop = true;
                //normal_first_loop = -normal_first_loop;
            }
        } else {
            if( normal_2d.z > 0 ) {
                reverse_loop = true;
            }
        }
        if( reverse_loop ) {
            std::reverse( loop_2d.begin(), loop_2d.end() );
        }

        if( loop_2d.size() < 3 ) {
            // TODO: Log error
        }

        /*
         * TODO: Calculate area
        double signed_area = carve::geom2d::signedArea( loop_2d );
        double min_loop_area = 1.e-10; // m_geom_settings->m_min_face_area
        if( std::abs( signed_area ) < min_loop_area ) {
            warning_small_loop_detected = true;
            continue;
        }
        */
        face_loops_used_for_triangulation.push_back( loop_2d );
    }

    /*
    if( warning_small_loop_detected ) {
        // TODO: Log error
    }
     */

    /*
    if( face_loops_used_for_triangulation.empty() ) {
        if( polyline_created ) {
            // already handled as curve
            return;
        }
        // TODO: Log error
        return;
    }
     */

    size_t num_vertices_in_loops = 0;
    for( auto& loop: face_loops_used_for_triangulation ) {
        num_vertices_in_loops += loop.size();
    }

    // triangulate
    std::vector<csgjscpp::Vector> merged_path;
    std::vector<carve::triangulate::tri_idx> triangulated;
    std::vector<std::pair<size_t, size_t>> path_incorporated_holes;
    try {
        path_incorporated_holes = carve::triangulate::incorporateHolesIntoPolygon( face_loops_used_for_triangulation );
        // figure 2: path which incorporates holes, described by path_all_loops
        // (0/0) -> (1/3) -> (1/0) -> (1/1) -> (1/2) -> (1/3) -> (0/0) -> (0/1) -> (0/2) -> (0/3)

        //  0/3<----------------------------0/2
        //  |                                ^
        //  |  1/0------------------>1/1     |
        //  |   ^                    |       |
        //  |   |                    v       |  path_incorporated_holes
        //  |  1/3  1/3<-------------1/2     |
        //  v /    /                         |
        //  0/0  0/0----------------------->0/1

        merged_path.reserve( path_incorporated_holes.size() );
        for( auto& path_incorporated_hole: path_incorporated_holes ) {
            size_t loop_number = path_incorporated_hole.first;
            size_t index_in_loop = path_incorporated_hole.second;

            if( loop_number >= face_loops_used_for_triangulation.size() ) {
                // TODO: Log error
                continue;
            }
            std::vector<csgjscpp::Vector>& loop = face_loops_used_for_triangulation[ loop_number ];

            if( index_in_loop >= loop.size() ) {
                // TODO: Log error
                continue;
            }
            csgjscpp::Vector& point_in_loop = loop[ index_in_loop ];
            merged_path.push_back( point_in_loop );
        }
        // figure 3: merged path for triangulation
        //  9<--------------------------8
        //  |                           ^
        //  |  2------------------>3    |
        //  |  ^                   |    |
        //  |  |                   v    |  merged_path
        //  |  1  5<---------------4    |
        //  | /  /                      |
        //  0  6----------------------->7
        carve::triangulate::triangulate( merged_path, triangulated );
        carve::triangulate::improve( merged_path, triangulated );
        // triangles: (3,8,9) (2,0,1) (4,6,7)  (4,5,6)  (9,0,2)  (9,2,3)  (7,8,3)  (3,4,7)
    } catch( ... ) {
        return;
    }

    for( auto& triangle: triangulated ) {
        size_t a = triangle.a;
        size_t b = triangle.b;
        size_t c = triangle.c;

        size_t loop_number_a = path_incorporated_holes[ a ].first;
        size_t index_in_loop_a = path_incorporated_holes[ a ].second;

        size_t vertex_id_a = index_in_loop_a;
        for( size_t jj = 0; jj < loop_number_a; ++jj ) {
            if( face_loops_used_for_triangulation.size() > jj ) {
                vertex_id_a += face_loops_used_for_triangulation[ jj ].size();
            }
        }

        size_t loop_number_b = path_incorporated_holes[ b ].first;
        size_t index_in_loop_b = path_incorporated_holes[ b ].second;
        size_t vertex_id_b = index_in_loop_b;
        for( size_t jj = 0; jj < loop_number_b; ++jj ) {
            if( face_loops_used_for_triangulation.size() > jj ) {
                vertex_id_b += face_loops_used_for_triangulation[ jj ].size();
            }
        }

        size_t loop_number_c = path_incorporated_holes[ c ].first;
        size_t index_in_loop_c = path_incorporated_holes[ c ].second;
        size_t vertex_id_c = index_in_loop_c;
        for( size_t jj = 0; jj < loop_number_c; ++jj ) {
            if( face_loops_used_for_triangulation.size() > jj ) {
                vertex_id_c += face_loops_used_for_triangulation[ jj ].size();
            }
        }

        if( vertex_id_a == vertex_id_b || vertex_id_a == vertex_id_c || vertex_id_b == vertex_id_c ) {
            continue;
        }

        face_indexes_out.push_back( 3 );
        face_indexes_out.push_back( (int)vertex_id_a );
        face_indexes_out.push_back( (int)vertex_id_b );
        face_indexes_out.push_back( (int)vertex_id_c );
    }
}


csgjscpp::Model SweepArea( const std::vector<csgjscpp::Vector>& curve_points, const std::vector<std::vector<csgjscpp::Vector>>& profile_paths_input ) {
    typedef carve::geom::vector<2> vec2;
    typedef carve::geom::vector<3> vec3;
    if( profile_paths_input.empty() ) {
        // TODO: Log error
        return {};
    }
    const size_t num_curve_points = profile_paths_input.size();
    if( num_curve_points < 2 ) {
        // TODO: Log error
        return {};
    }

    // figure 1: loops and indexes
    //  3----------------------------2
    //  |                            |
    //  |   1-------------------2    |3---------2
    //  |   |                   |    |          |
    //  |   |                   |    |          |face_loops[2]   // TODO: handle combined profiles
    //  |   0---face_loops[1]---3    |0---------1
    //  |                            |
    //  0-------face_loops[0]--------1


    std::vector<std::vector<std::vector<csgjscpp::Vector>>> profile_paths_enclosed;
    FindEnclosedLoops( profile_paths_input, profile_paths_enclosed );

    for( auto& profile_paths: profile_paths_enclosed ) {
        for( size_t ii = 0; ii < profile_paths.size(); ++ii ) {
            std::vector<int> face_indexes;
            std::vector<std::vector<csgjscpp::Vector>> face_loops_used_for_triangulation;
            TriangulateLoops( profile_paths, face_loops_used_for_triangulation, face_indexes );

            size_t num_points_in_all_loops = 0;
            for( auto& loop: face_loops_used_for_triangulation ) {
                num_points_in_all_loops += loop.size();
            }

            std::shared_ptr<carve::input::PolyhedronData> poly_data( new carve::input::PolyhedronData() );
            poly_data->points.resize( num_points_in_all_loops * curve_points.size() );

            const auto& curve_point_first = curve_points[ 0 ];
            const auto& curve_point_second = curve_points[ 1 ];
            const auto curve_normal = ComputePolygonNormal( curve_points );

            // rotate face loops into first direction
            auto section_local_y = curve_normal;
            auto section_local_z = curve_point_first - curve_point_second;
            auto section_local_x = csgjscpp::cross( section_local_y, section_local_z );
            section_local_y = csgjscpp::cross( section_local_x, section_local_z );
            section_local_x = csgjscpp::unit( section_local_x );
            section_local_y = csgjscpp::unit( section_local_y );
            section_local_z = csgjscpp::unit( section_local_z );

            carve::math::Matrix matrix_first_direction =
                carve::math::Matrix( section_local_x.x, section_local_y.x, section_local_z.x, 0, section_local_x.y, section_local_y.y, section_local_z.y, 0,
                                     section_local_x.z, section_local_y.z, section_local_z.z, 0, 0, 0, 0, 1 );

            std::vector<vec3>& polyhedron_points = poly_data->points;
            size_t polyhedron_point_index = 0;
            for(auto & loop : face_loops_used_for_triangulation) {
                for(auto & vec_2d : loop) {
                    vec3 vec_3d( carve::geom::VECTOR( vec_2d.x, vec_2d.y, 0 ) );

                    // cross section is defined in XY plane
                    vec_3d = matrix_first_direction * vec_3d + carve::geom::VECTOR( curve_point_first.x, curve_point_first.y, curve_point_first.z );
                    polyhedron_points[ polyhedron_point_index++ ] = vec_3d;
                }
            }

            for( size_t iii = 1; iii < num_curve_points; ++iii ) {
                auto curve_point_current = curve_points[ iii ];
                csgjscpp::Vector curve_point_next;
                csgjscpp::Vector curve_point_before;
                if( iii == 0 ) {
                    // first point
                    curve_point_next = curve_points[ iii + 1 ];
                    auto delta_element = curve_point_next - curve_point_current;
                    curve_point_before = curve_point_current - ( delta_element );
                } else if( iii == num_curve_points - 1 ) {
                    // last point
                    curve_point_before = curve_points[ iii - 1 ];
                    auto delta_element = curve_point_current - curve_point_before;
                    curve_point_next = curve_point_before + ( delta_element );
                } else {
                    // inner point
                    curve_point_next = curve_points[ iii + 1 ];
                    curve_point_before = curve_points[ iii - 1 ];
                }

                auto bisecting_normal = BisectingPlane( curve_point_before, curve_point_current, curve_point_next );

                auto section1 = csgjscpp::unit( curve_point_current - curve_point_before );
                if( iii == num_curve_points - 1 ) {
                    bisecting_normal = -bisecting_normal;
                }

                carve::geom::plane<3> bisecting_plane( carve::geom::VECTOR( bisecting_normal.x, bisecting_normal.y, bisecting_normal.z ),
                                                       carve::geom::VECTOR( curve_point_current.x, curve_point_current.y, curve_point_current.z ) );
                for( size_t jj = 0; jj < num_points_in_all_loops; ++jj ) {
                    vec3& section_point_3d = polyhedron_points[ polyhedron_point_index ];
                    vec3& previous_section_point_3d = polyhedron_points[ polyhedron_point_index - num_points_in_all_loops ];

                    polyhedron_point_index++;

                    vec3 v;
                    double t;
                    carve::IntersectionClass intersect =
                        carve::geom3d::rayPlaneIntersection( bisecting_plane, previous_section_point_3d,
                                                             previous_section_point_3d + carve::geom::VECTOR( section1.x, section1.y, section1.z ), v, t );
                    if( intersect > 0 ) {
                        section_point_3d = v;
                    } else {
                        // messageCallback( "no intersection found", StatusCallback::MESSAGE_TYPE_WARNING, __FUNC__, ifc_entity );
                        //  TODO: Log error
                    }
                }
            }

            // add face loops for all sections
            const size_t num_poly_points = polyhedron_points.size();
            size_t loop_offset = 0;
            for(auto & loop : face_loops_used_for_triangulation) {
                for( size_t jj = 0; jj < loop.size(); ++jj ) {
                    for( size_t kk = 0; kk < num_curve_points - 1; ++kk ) {
                        size_t tri_idx_a = num_points_in_all_loops * kk + jj + loop_offset;

                        size_t tri_idx_next = tri_idx_a + 1;
                        if( jj == loop.size() - 1 ) {
                            tri_idx_next -= loop.size();
                        }
                        size_t tri_idx_up = tri_idx_a + num_points_in_all_loops; // next section
                        size_t tri_idx_next_up = tri_idx_next + num_points_in_all_loops; // next section


                        if( tri_idx_a >= num_poly_points || tri_idx_next >= num_poly_points || tri_idx_up >= num_poly_points ||
                            tri_idx_next_up >= num_poly_points ) {
                            // messageCallback( "invalid triangle index", StatusCallback::MESSAGE_TYPE_WARNING, __FUNC__, ifc_entity );
                            //  TODO: Log error
                            continue;
                        }

                        poly_data->addFace( ( int )tri_idx_a, ( int )tri_idx_next, ( int )tri_idx_next_up );
                        poly_data->addFace( ( int )tri_idx_next_up, ( int )tri_idx_up, ( int )tri_idx_a );
                    }
                }

                loop_offset += loop.size();
            }

            // add front and back cap
            for( size_t iii = 0; iii < face_indexes.size(); ++iii ) {
                size_t num_face_vertices = face_indexes[ iii ];

                if( num_face_vertices == 3 ) {
                    size_t tri_idx_a = face_indexes[ iii + 1 ];
                    size_t tri_idx_b = face_indexes[ iii + 2 ];
                    size_t tri_idx_c = face_indexes[ iii + 3 ];
                    if( tri_idx_a >= num_poly_points || tri_idx_b >= num_poly_points || tri_idx_c >= num_poly_points ) {
                        // messageCallback( "invalid triangle index", StatusCallback::MESSAGE_TYPE_WARNING, __FUNC__, ifc_entity );
                        //  TODO: Log error
                        iii += num_face_vertices;
                        continue;
                    }

                    poly_data->addFace( ( int )tri_idx_a, ( int )tri_idx_c, ( int )tri_idx_b );

                    size_t tri_idx_a_back_cap = tri_idx_a + num_poly_points - num_points_in_all_loops;
                    size_t tri_idx_b_back_cap = tri_idx_b + num_poly_points - num_points_in_all_loops;
                    size_t tri_idx_c_back_cap = tri_idx_c + num_poly_points - num_points_in_all_loops;
                    if( tri_idx_a_back_cap >= num_poly_points || tri_idx_b_back_cap >= num_poly_points || tri_idx_c_back_cap >= num_poly_points ) {
                        // messageCallback( "invalid triangle index", StatusCallback::MESSAGE_TYPE_WARNING, __FUNC__, ifc_entity );
                        //  TODO: Log error
                        iii += num_face_vertices;
                        continue;
                    }

                    poly_data->addFace( ( int )tri_idx_a_back_cap, ( int )tri_idx_b_back_cap, ( int )tri_idx_c_back_cap );
                } /*else if( num_face_vertices == 2 ) {
                    // add polyline
                    // poly_data->addFace( face_indexes[ii+1], face_indexes[ii+2] );
                } else if( num_face_vertices == 1 ) {
                    // add polyline
                    // poly_data->addFace( face_indexes[ii+1], face_indexes[ii+2] );
                } else {
                    // messageCallback( "num_face_vertices != 3", StatusCallback::MESSAGE_TYPE_WARNING, __FUNC__, ifc_entity );
                    //  TODO: Log error
                }*/
                iii += num_face_vertices;
            }

            // try {
            //     item_data->addClosedPolyhedron( poly_data );
            // } catch( BuildingException& exception ) {
            //     messageCallback( exception.what(), StatusCallback::MESSAGE_TYPE_WARNING, "", ifc_entity ); // calling function already in e.what()
            // }
        }
    }
}

}