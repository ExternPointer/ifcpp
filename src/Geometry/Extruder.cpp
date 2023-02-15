#include "ifcpp/Geometry/Extruder.h"
#include "ifcpp/Geometry/Utils.h"
#include "carve/input.hpp"
#include "carve/triangulator.hpp"
#include "ifcpp/Geometry/csgjs.h"
#include "mapbox/earcut.hpp"

namespace ifcpp {

double signedArea(const std::vector<std::array<double, 2> >& points)
{
    size_t l = points.size();
    double A = 0.0;
    if( l > 2 )
    {
        for( size_t i = 0; i < l - 1; i++ )
        {
            A += (points[i + 1][1] + points[i][1]) * (points[i + 1][0] - points[i][0]);
        }
    }
    A += (points[0][1] + points[l - 1][1]) * (points[0][0] - points[l - 1][0]);

    return A / 2.0;
}



void polygons2flatVec(const std::vector<std::vector<std::array<double, 2> > >& polyIn, std::vector<std::array<double, 2> >& polyOut)
{
    for( const auto& vec : polyIn )
    {
        for( const auto& point : vec )
        {
            polyOut.push_back({ point[0], point[1] });
        }
    }
}

bool bisectingPlane( const carve::geom::vector<3>& v1, const carve::geom::vector<3>& v2, const carve::geom::vector<3>& v3, carve::geom::vector<3>& normal )
{
    typedef carve::geom::vector<3> vec3;
    bool valid = false;
    vec3 v21 = v2 - v1;
    vec3 v32 = v3 - v2;
    double len21_square = v21.length2();
    double len32_square = v32.length2();

    if( len21_square <= carve::CARVE_EPSILON * len32_square )
    {
        if( len32_square == 0.0 )
        {
            // all three points lie ontop of one-another
            normal = carve::geom::VECTOR( 0.0, 0.0, 0.0 );
            valid = false;
        }
        else
        {
            // return a normalized copy of v32 as bisector
            len32_square = 1.0 / len32_square;
            normal = v32*len32_square;
            normal.normalize();
            valid = true;
        }

    }
    else
    {
        valid = true;
        if( len32_square <= carve::CARVE_EPSILON * len21_square )
        {
            // return v21 as bisector
            v21.normalize();
            normal = v21;
        }
        else
        {
            v21.normalize();
            v32.normalize();

            double dot_product = dot( v32, v21 );
            double dot_product_abs = std::abs( dot_product );

            if( dot_product_abs > ( 1.0 + carve::CARVE_EPSILON ) || dot_product_abs < ( 1.0 - carve::CARVE_EPSILON ) )
            {
                normal = ( v32 + v21 )*dot_product - v32 - v21;
                normal.normalize();
            }
            else
            {
                // dot == 1 or -1, points are colinear
                normal = -v21;
            }
        }
    }
    return valid;
}


void AddFacesFromCarvePolyhedron( const std::shared_ptr<carve::input::PolyhedronData>& polyhedron, std::vector<csgjscpp::Polygon>& polygons ) {
    for( int i = 0; i < polyhedron->faceIndices.size(); ) {
        int verticesCount = polyhedron->faceIndices[ i ];
        std::vector<csgjscpp::Vertex> vertices;
        for( int j = i + 1; j < i + 1 + verticesCount; j++ ) {
            auto carveV = polyhedron->getVertex( polyhedron->faceIndices[ j ] );
            csgjscpp::Vertex v;
            v.pos = { (float)carveV.x, (float)carveV.y, (float)carveV.z };
            vertices.push_back( v );
        }
        i += verticesCount + 1;
        polygons.emplace_back( vertices );
    }
}

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

    // csgjscpp::Vector normal_first_loop;
    // bool warning_small_loop_detected = false;
    // bool polyline_created = false;

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
            // normal_first_loop = normal_2d;
            if( normal_2d.z < 0 ) {
                reverse_loop = true;
                // normal_first_loop = -normal_first_loop;
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

csgjscpp::Model Extrude( const std::vector<std::vector<csgjscpp::Vector>>& faceLoopsInput, const csgjscpp::Vector& extrusionVector ) {
    typedef std::array<double, 2> array2d;
    typedef carve::geom::vector<2> vec2;
    typedef carve::geom::vector<3> vec3;
    // TODO: complete and test
    if( faceLoopsInput.empty() ) {
        // messageCallback("faceLoopsInput.size() == 0", StatusCallback::MESSAGE_TYPE_WARNING, __FUNC__, ifc_entity);
        //  TODO: Log error
        return {};
    }

    // loops and indexes
    //  3----------------------------2
    //  |                            |
    //  |   1-------------------2    |3---------2
    //  |   |                   |    |          |
    //  |   |                   |    |          |face_loops[2]
    //  |   0---face_loops[1]---3    |0---------1
    //  |                            |
    //  0-------face_loops[0]--------1

    if( csgjscpp::lengthsquared( extrusionVector ) < csgjscpp::csgjs_EPSILON ) {
        // maybe still use it as flat surface instead of extruded volume
        return {};
    }

    std::vector<std::vector<array2d>> faceLoopsTriangulate;
    std::vector<double> polygon3DArea;
    bool face_loop_reversed = false;
    bool warning_small_loop_detected = false;
    bool errorOccured = false;
    csgjscpp::Vector normal( 0, 0, 1 );

    for( auto it_bounds = faceLoopsInput.begin(); it_bounds != faceLoopsInput.end(); ++it_bounds ) {
        auto loopPointsInput = *it_bounds;

        if( loopPointsInput.size() < 3 ) {
            if( it_bounds == faceLoopsInput.begin() ) {
                break;
            } else {
                continue;
            }
        }
        bool mergeAlignedEdges = true;
        //GeomUtils::simplifyPolygon( loopPointsInput, mergeAlignedEdges );
        UnCloseLoop( loopPointsInput );
        normal = ComputePolygon2DNormal( loopPointsInput );

        if( loopPointsInput.size() > 3 ) {
            // rare case: edges between points n -> 0 -> 1 could be in a straight line
            // this leads to an open mesh after triangulation

            const auto& p0 = loopPointsInput.back();
            const auto& p1 = loopPointsInput[ 0 ];
            const auto& p2 = loopPointsInput[ 1 ];
            const float dx1 = p1.x - p0.x;
            const float dx2 = p2.x - p1.x;
            const float dy1 = p1.y - p0.y;
            const float dy2 = p2.y - p1.y;


            float scalar = dx1 * dx2 + dy1 * dy2;
            float check = scalar * scalar - ( dx1 * dx1 + dy1 * dy1 ) * ( dx2 * dx2 + dy2 * dy2 );

            if( std::fabsf( check ) < csgjscpp::csgjs_EPSILON ) {
                loopPointsInput.erase( loopPointsInput.begin() );
            }
        }

        std::vector<array2d> path_loop_2d;
        for( size_t i = 0; i < loopPointsInput.size(); ++i ) {
            const auto& point = loopPointsInput[ i ];
            path_loop_2d.push_back( { (double)point.x, (double)point.y } );
        }

        if( path_loop_2d.size() < 3 ) {
            // std::cout << __FUNC__ << ": #" << face_id <<  "=IfcFace: path_loop.size() < 3" << std::endl;
            continue;
        }

        double loop_area = std::abs( signedArea( path_loop_2d ) );
        double min_loop_area = csgjscpp::csgjs_EPSILON; // m_geom_settings->m_min_face_area
        if( loop_area < min_loop_area ) {
            warning_small_loop_detected = true;
            continue;
        }

        // outer loop (biggest area) needs to come first
        bool insertPositionFound = false;
        for( size_t iiArea = 0; iiArea < polygon3DArea.size(); ++iiArea ) {
            double existingLoopArea = polygon3DArea[ iiArea ];

            // existingArea[i]  < loop_area < existingArea[i+1]

            if( loop_area > existingLoopArea ) {
                faceLoopsTriangulate.insert( faceLoopsTriangulate.begin() + iiArea, path_loop_2d );
                polygon3DArea.insert( polygon3DArea.begin() + iiArea, loop_area );
                insertPositionFound = true;
                break;
            }
        }

        if( !insertPositionFound ) {
            faceLoopsTriangulate.push_back( path_loop_2d );
            polygon3DArea.push_back( loop_area );
        }
    }


    // check winding order in 2D
    for( size_t ii = 0; ii < faceLoopsTriangulate.size(); ++ii ) {
        std::vector<array2d>& loop2D = faceLoopsTriangulate[ ii ];


        auto normal_2d = computePolygon2DNormal( loop2D );
        if( ii == 0 ) {
            if( normal_2d.z < 0 ) {
                std::reverse( loop2D.begin(), loop2D.end() );
                face_loop_reversed = true;
            }
        } else {
            if( normal_2d.z > 0 ) {
                std::reverse( loop2D.begin(), loop2D.end() );
            }
        }
    }

    if( warning_small_loop_detected ) {
        std::stringstream err;
        //err << "std::abs( signed_area ) < 1.e-10";
        //messageCallback( err.str().c_str(), StatusCallback::MESSAGE_TYPE_MINOR_WARNING, __FUNC__, ifc_entity );
        // TODO: Log error
    }

    if( faceLoopsTriangulate.size() == 0 ) {
        return {};
    }

    std::vector<uint32_t> triangulated = mapbox::earcut<uint32_t>( faceLoopsTriangulate );


    std::vector<array2d> polygons2dFlatVector;
    polygons2flatVec( faceLoopsTriangulate, polygons2dFlatVector );
    size_t numPointsInAllLoops = polygons2dFlatVector.size();

    // PolyInputCache3D meshOut;
    std::shared_ptr<carve::input::PolyhedronData> meshOut( new carve::input::PolyhedronData() );

    // add points bottom
    for( size_t ii = 0; ii < polygons2dFlatVector.size(); ++ii ) {
        array2d& point2D = polygons2dFlatVector[ ii ];
        vec3 point3D = carve::geom::VECTOR( point2D[ 0 ], point2D[ 1 ], 0 );
        meshOut->addVertex( point3D );
    }

    // add points top
    for( size_t ii = 0; ii < polygons2dFlatVector.size(); ++ii ) {
        array2d& point2D = polygons2dFlatVector[ ii ];
        vec3 point3D = carve::geom::VECTOR( point2D[ 0 ], point2D[ 1 ], 0 );
        vec3 point3D_top = point3D + carve::geom::VECTOR(extrusionVector.x, extrusionVector.y, extrusionVector.z);
        meshOut->addVertex( point3D_top );
    }


    // triangles along the extruded loops
    size_t idxLoopOffset = 0;
    for( size_t ii = 0; ii < faceLoopsTriangulate.size(); ++ii ) {
        std::vector<array2d>& loop2D = faceLoopsTriangulate[ ii ];


        bool createQuadsIfPossible = false;
        bool flipFaces = false;

        if( extrusionVector.z < 0 ) {
            flipFaces = !flipFaces;
        }

        const size_t numLoopPoints = loop2D.size();
        for( size_t jj = 0; jj < numLoopPoints; ++jj ) {
            array2d& point2D = loop2D[ jj ];
            array2d& point2D_next = loop2D[ ( jj + 1 ) % numLoopPoints ];

            vec3 point3D = carve::geom::VECTOR( point2D[ 0 ], point2D[ 1 ], 0 );
            vec3 point3D_next = carve::geom::VECTOR( point2D_next[ 0 ], point2D_next[ 1 ], 0 );

            vec3 point3D_top = point3D + carve::geom::VECTOR(extrusionVector.x, extrusionVector.y, extrusionVector.z);
            vec3 point3D_top_next = point3D_next + carve::geom::VECTOR(extrusionVector.x, extrusionVector.y, extrusionVector.z);

            size_t idx = jj + idxLoopOffset;
            size_t idx_next = ( jj + 1 ) % numLoopPoints + idxLoopOffset;
            size_t idx_top = idx + numPointsInAllLoops;
            size_t idx_top_next = idx_next + numPointsInAllLoops;

            if( createQuadsIfPossible ) {
                if( flipFaces ) {
                    meshOut->addFace( idx, idx_top, idx_top_next, idx_next );
                } else {
                    meshOut->addFace( idx, idx_next, idx_top_next, idx_top );
                }
            } else {
                if( flipFaces ) {
                    meshOut->addFace( idx, idx_top_next, idx_next );
                    meshOut->addFace( idx_top_next, idx, idx_top );
                } else {
                    meshOut->addFace( idx, idx_next, idx_top_next );
                    meshOut->addFace( idx_top_next, idx_top, idx );
                }
            }
        }

        idxLoopOffset += numLoopPoints;
    }

    // front and back cap
    for( int ii = 0; ii < triangulated.size(); ii += 3 ) {
        size_t idxA = triangulated[ ii + 0 ];
        size_t idxB = triangulated[ ii + 1 ];
        size_t idxC = triangulated[ ii + 2 ];

        size_t idxAtop = idxA + numPointsInAllLoops;
        size_t idxBtop = idxB + numPointsInAllLoops;
        size_t idxCtop = idxC + numPointsInAllLoops;

        if( extrusionVector.z < 0 ) {
            meshOut->addFace( idxA, idxB, idxC );
            meshOut->addFace( idxAtop, idxCtop, idxBtop );
        } else {
            meshOut->addFace( idxA, idxC, idxB );
            meshOut->addFace( idxAtop, idxBtop, idxCtop );
        }
    }

    std::vector<csgjscpp::Polygon> polygons;
    AddFacesFromCarvePolyhedron(meshOut, polygons);
    //itemData->addClosedPolyhedron( meshOut );
    return csgjscpp::modelfrompolygons(polygons);
}

csgjscpp::Model sweepDisk( const std::vector<csgjscpp::Vector>& curve_points, const size_t nvc, const double radius, const double radius_inner )
{
    typedef std::array<double, 2> array2d;
    typedef carve::geom::vector<2> vec2;
    typedef carve::geom::vector<3> vec3;
    const size_t num_curve_points = curve_points.size();
    if( num_curve_points < 2 )
    {
        //messageCallback( "num curve points < 2", StatusCallback::MESSAGE_TYPE_WARNING, __FUNC__, ifc_entity );
        // TODO: Log error
        return {};
    }

    if( radius < 0.001 )
    {
        /*
        // Cross section is just a point. Create a polyline
        std::shared_ptr<carve::input::PolylineSetData> polyline_data( new carve::input::PolylineSetData() );
        polyline_data->beginPolyline();
        for( size_t i_polyline = 0; i_polyline < curve_points.size(); ++i_polyline )
        {
            const auto& curve_pt = curve_points[i_polyline];
            polyline_data->addVertex( carve::geom::VECTOR( curve_pt ) );
            polyline_data->addPolylineIndex( 0 );
            polyline_data->addPolylineIndex( i_polyline );
        }
        item_data->m_polylines.push_back( polyline_data );
        return;
         */
        return {};
    }

    double use_radius_inner = radius_inner;
    if( radius_inner > radius )
    {
        //messageCallback( "radius_inner > radius", StatusCallback::MESSAGE_TYPE_WARNING, __FUNC__, ifc_entity );
        // TODO: Log error
        use_radius_inner = radius;
    }

    vec3 local_z( carve::geom::VECTOR( 0, 0, 1 ) );
    vec3 curve_point_first = carve::geom::VECTOR(curve_points[0]);
    vec3 curve_point_second = carve::geom::VECTOR(curve_points[1]);

    bool bend_found = false;
    if( num_curve_points > 3 )
    {
        // compute local z vector by dot product of the first bend of the reference line
        vec3 vertex_back2 = curve_point_first;
        vec3 vertex_back1 = curve_point_second;
        for( size_t i = 2; i<num_curve_points; ++i )
        {
            const vec3& vertex_current = carve::geom::VECTOR(curve_points[i]);
            vec3 section1 = vertex_back1 - vertex_back2;
            vec3 section2 = vertex_current - vertex_back1;
            section1.normalize();
            section2.normalize();

            double dot_product = dot( section1, section2 );
            double dot_product_abs = std::abs(dot_product);

            // if dot == 1 or -1, then points are colinear
            if( dot_product_abs < (1.0-0.0001) || dot_product_abs > (1.0+0.0001) )
            {
                // bend found, compute cross product
                vec3 lateral_vec = cross( section1, section2 );
                local_z = cross( lateral_vec, section1 );
                local_z.normalize();
                bend_found = true;
                break;
            }
        }
    }

    if( !bend_found )
    {
        // sweeping curve is linear. assume any local z vector
        vec3 sweep_dir = curve_point_second - curve_point_first;
        if( sweep_dir.length2() > 0.1 )
        {
            sweep_dir.normalize();

            double dot_sweep_dir = dot(sweep_dir, carve::geom::VECTOR(0, 0, 1));
            if( std::abs(dot_sweep_dir-1.0) > 0.0001 )
            {
                local_z = cross(carve::geom::VECTOR(0, 0, 1), sweep_dir);
                if( local_z.length2() < 0.001 )
                {
                    local_z = cross(carve::geom::VECTOR(0, 1, 0), sweep_dir);
                    local_z.normalize();
                }
                else
                {
                    local_z.normalize();
                }
                double dot_normal_local_z = dot(sweep_dir, local_z);
                if( std::abs(dot_normal_local_z-1.0) < 0.0001 )
                {
                    local_z = cross(carve::geom::VECTOR(0, 1, 0), sweep_dir);
                    local_z.normalize();

                    dot_normal_local_z = dot(sweep_dir, local_z);
                    if( std::abs(dot_normal_local_z-1.0) < 0.0001 )
                    {
                        local_z = cross(carve::geom::VECTOR(1, 0, 0), sweep_dir);
                        local_z.normalize();
                    }
                }
            }
        }
    }

    // rotate disk into first direction
    vec3  section_local_y = local_z;
    vec3  section_local_z = curve_point_first - curve_point_second;
    vec3  section_local_x = carve::geom::cross( section_local_y, section_local_z );
    section_local_y = carve::geom::cross( section_local_x, section_local_z );
    std::vector<vec3> inner_shape_points;

    section_local_x.normalize();
    section_local_y.normalize();
    section_local_z.normalize();

    carve::math::Matrix matrix_first_direction = carve::math::Matrix(
        section_local_x.x,		section_local_y.x,		section_local_z.x,	0,
        section_local_x.y,		section_local_y.y,		section_local_z.y,	0,
        section_local_x.z,		section_local_y.z,		section_local_z.z,	0,
        0,				0,				0,			1 );

    double angle = 0;
    double delta_angle = 2.0*M_PI/double(nvc);	// TODO: adapt to model size and complexity
    std::vector<vec3> circle_points(nvc);
    std::vector<vec3> circle_points_inner(nvc);
    for( size_t ii = 0; ii < nvc; ++ii )
    {
        // cross section (circle) is defined in XY plane
        double x = sin(angle);
        double y = cos(angle);
        vec3 vertex( carve::geom::VECTOR( x*radius, y*radius, 0.0 ) );
        vertex = matrix_first_direction*vertex + curve_point_first;
        circle_points[ii] = vertex;

        if( use_radius_inner > 0 )
        {
            vec3 vertex_inner( carve::geom::VECTOR( x*use_radius_inner, y*use_radius_inner, 0.0 ) );
            vertex_inner = matrix_first_direction*vertex_inner + curve_point_first;
            circle_points_inner[ii] = vertex_inner;
        }
        angle += delta_angle;
    }

    std::shared_ptr<carve::input::PolyhedronData> poly_data( new carve::input::PolyhedronData() );

    for( size_t ii = 0; ii<num_curve_points; ++ii )
    {
        const vec3& vertex_current = carve::geom::VECTOR(curve_points[ii]);
        vec3 vertex_next;
        vec3 vertex_before;
        if( ii == 0 )
        {
            // first point
            vertex_next	= carve::geom::VECTOR(curve_points[ii+1]);
            vec3 delta_element = vertex_next - vertex_current;
            vertex_before = vertex_current - (delta_element);
        }
        else if( ii == num_curve_points-1 )
        {
            // last point
            vertex_before	= carve::geom::VECTOR(curve_points[ii-1]);
            vec3 delta_element = vertex_current - vertex_before;
            vertex_next = vertex_before + (delta_element);
        }
        else
        {
            // inner point
            vertex_next		= carve::geom::VECTOR(curve_points[ii+1]);
            vertex_before	= carve::geom::VECTOR(curve_points[ii-1]);
        }

        vec3 bisecting_normal;
        bisectingPlane( vertex_before, vertex_current, vertex_next, bisecting_normal );

        vec3 section1 = vertex_current - vertex_before;
        vec3 section2 = vertex_next - vertex_current;
        if (section1.length2() > 0.0001)
        {
            section1.normalize();
            if (section2.length2() > 0.0001)
            {
                section2.normalize();
                double dot_product = dot(section1, section2);
                double dot_product_abs = std::abs(dot_product);

                if (dot_product_abs < (1.0-0.0001) || dot_product_abs >(1.0+0.0001))
                {
                    // bend found, compute next local z vector
                    vec3 lateral_vec = cross(section1, section2);
                    if (lateral_vec.length2() > 0.0001)
                    {
                        local_z = cross(lateral_vec, section1);
                        local_z.normalize();
                    }
                }
            }
        }

        if( ii == num_curve_points -1 )
        {
            bisecting_normal *= -1.0;
        }

        carve::geom::plane<3> bisecting_plane( bisecting_normal, vertex_current );
        for( size_t jj = 0; jj < nvc; ++jj )
        {
            vec3& vertex = circle_points[jj];

            vec3 v;
            double t;
            carve::IntersectionClass intersect = carve::geom3d::rayPlaneIntersection( bisecting_plane, vertex, vertex + section1, v, t);
            if( intersect > 0 )
            {
                vertex = v;
            }
            else
            {
                //messageCallback( "no intersection found", StatusCallback::MESSAGE_TYPE_WARNING, __FUNC__, ifc_entity );
                // TODO: Log error
            }

            poly_data->addVertex( vertex );
        }

        if( use_radius_inner > 0 )
        {
            for( size_t jj = 0; jj < nvc; ++jj )
            {
                vec3& vertex = circle_points_inner[jj];

                vec3 v;
                double t;
                carve::IntersectionClass intersect = carve::geom3d::rayPlaneIntersection( bisecting_plane, vertex, vertex + section1, v, t);
                if( intersect > 0 )
                {
                    vertex = v;
                }
                else
                {
                    //messageCallback( "no intersection found", StatusCallback::MESSAGE_TYPE_WARNING, __FUNC__, ifc_entity );
                    // TODO: Log error
                }

                //inner_shape_points[jj] = vertex;
                inner_shape_points.push_back( vertex );
            }
        }
    }

    // outer shape
    size_t num_vertices_outer = poly_data->getVertexCount();
    for( size_t i=0; i<num_curve_points- 1; ++i )
    {
        size_t i_offset = i*nvc;
        size_t i_offset_next = ( i + 1 )*nvc;
        for( size_t jj = 0; jj < nvc; ++jj )
        {
            size_t current_loop_pt1 = jj + i_offset;
            size_t current_loop_pt2 = ( jj + 1 ) % nvc + i_offset;

            size_t next_loop_pt1 = jj + i_offset_next;
            size_t next_loop_pt2 = ( jj + 1 ) % nvc + i_offset_next;
            poly_data->addFace( current_loop_pt1,	next_loop_pt1,		next_loop_pt2 );
            poly_data->addFace( next_loop_pt2,		current_loop_pt2,	current_loop_pt1  );
        }
    }

    if( use_radius_inner > 0 )
    {
        if( inner_shape_points.size() != num_vertices_outer )
        {
            //messageCallback( "inner_shape_points.size() != num_vertices_outer", StatusCallback::MESSAGE_TYPE_WARNING, __FUNC__, ifc_entity );
            // TODO: Log error
        }

        // add points for inner shape
        for( size_t i=0; i<inner_shape_points.size(); ++i )
        {
            poly_data->addVertex( inner_shape_points[i] );
        }

        // faces of inner shape
        for( size_t i=0; i<num_curve_points- 1; ++i )
        {
            size_t i_offset = i*nvc + num_vertices_outer;
            size_t i_offset_next = ( i + 1 )*nvc + num_vertices_outer;
            for( size_t jj = 0; jj < nvc; ++jj )
            {
                size_t current_loop_pt1 = jj + i_offset;
                size_t current_loop_pt2 = ( jj + 1 ) % nvc + i_offset;

                size_t next_loop_pt1 = jj + i_offset_next;
                size_t next_loop_pt2 = ( jj + 1 ) % nvc + i_offset_next;

                poly_data->addFace( current_loop_pt1,	current_loop_pt2,	next_loop_pt2 );
                poly_data->addFace( next_loop_pt2,		next_loop_pt1,		current_loop_pt1  );
            }
        }

        // front cap
        for( size_t jj = 0; jj < nvc; ++jj )
        {
            size_t outer_rim_next = ( jj + 1 ) % nvc;
            size_t inner_rim_next = outer_rim_next + num_vertices_outer;

            poly_data->addFace( jj,					outer_rim_next,		num_vertices_outer+jj );
            poly_data->addFace( outer_rim_next,		inner_rim_next,		num_vertices_outer+jj );
        }

        // back cap
        size_t back_offset = ( num_curve_points - 1 )*nvc;
        for( size_t jj = 0; jj < nvc; ++jj )
        {
            size_t outer_rim_next = ( jj + 1 ) % nvc + back_offset;
            size_t inner_rim_next = outer_rim_next + num_vertices_outer;

            poly_data->addFace( jj+back_offset,		num_vertices_outer+jj+back_offset,	outer_rim_next );
            poly_data->addFace( outer_rim_next,		num_vertices_outer+jj+back_offset,	inner_rim_next );
        }
    }
    else
    {
        // front cap, full pipe, create triangle fan
        for( size_t jj = 0; jj < nvc - 2; ++jj )
        {
            poly_data->addFace( 0, jj+1, jj+2 );
        }

        // back cap
        size_t back_offset = ( num_curve_points - 1 )*nvc;
        for( size_t jj = 0; jj < nvc - 2; ++jj )
        {
            poly_data->addFace( back_offset, back_offset+jj+2, back_offset+jj+1 );
        }
    }
    /*
        try
        {
            item_data->addClosedPolyhedron( poly_data );
        }
        catch( BuildingException& exception )
        {
            messageCallback( exception.what(), StatusCallback::MESSAGE_TYPE_WARNING, "", ifc_entity );  // calling function already in e.what()
        }
        */
    std::vector<csgjscpp::Polygon> polygons;
    AddFacesFromCarvePolyhedron(poly_data, polygons);
    return csgjscpp::modelfrompolygons(polygons);
}


csgjscpp::Model SweepArea( const std::vector<csgjscpp::Vector>& curve_points, const std::vector<std::vector<csgjscpp::Vector>>& profile_paths_input ) {
    std::vector<csgjscpp::Polygon> polygons;
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
            for( auto& loop: face_loops_used_for_triangulation ) {
                for( auto& vec_2d: loop ) {
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
            for( auto& loop: face_loops_used_for_triangulation ) {
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

                        poly_data->addFace( (int)tri_idx_a, (int)tri_idx_next, (int)tri_idx_next_up );
                        poly_data->addFace( (int)tri_idx_next_up, (int)tri_idx_up, (int)tri_idx_a );
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

                    poly_data->addFace( (int)tri_idx_a, (int)tri_idx_c, (int)tri_idx_b );

                    size_t tri_idx_a_back_cap = tri_idx_a + num_poly_points - num_points_in_all_loops;
                    size_t tri_idx_b_back_cap = tri_idx_b + num_poly_points - num_points_in_all_loops;
                    size_t tri_idx_c_back_cap = tri_idx_c + num_poly_points - num_points_in_all_loops;
                    if( tri_idx_a_back_cap >= num_poly_points || tri_idx_b_back_cap >= num_poly_points || tri_idx_c_back_cap >= num_poly_points ) {
                        // messageCallback( "invalid triangle index", StatusCallback::MESSAGE_TYPE_WARNING, __FUNC__, ifc_entity );
                        //  TODO: Log error
                        iii += num_face_vertices;
                        continue;
                    }

                    poly_data->addFace( (int)tri_idx_a_back_cap, (int)tri_idx_b_back_cap, (int)tri_idx_c_back_cap );
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

            AddFacesFromCarvePolyhedron( poly_data, polygons );
        }
    }

    return csgjscpp::modelfrompolygons( polygons );
}

}