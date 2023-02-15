#pragma once

#include "csgjs.h"
#include <array>
#include <cmath>
#include <memory>
#include <vector>
#include "Matrix.h"

namespace ifcpp {

inline void UnCloseLoop( std::vector<csgjscpp::Vector>& loop ) {
    if( loop.size() > 1 ) {
        if( loop.front() == loop.back() ) {
            loop.pop_back();
        }
    }
}

inline csgjscpp::Polygon CreatePolygon( const std::vector<csgjscpp::Vector>& loop /*, style data*/ ) {
    if( loop.size() < 3 ) {
        return {};
    }
    // const auto normal = csgjscpp::unit( csgjscpp::cross( loop[ 1 ] - loop[ 0 ], loop[ 2 ] - loop[ 0 ] ) );
    std::vector<csgjscpp::Vertex> vertices;
    for( const auto& position: loop ) {
        vertices.push_back( { position } );
    }
    return csgjscpp::Polygon( vertices );
}

inline csgjscpp::Model CreateModel( const std::vector<csgjscpp::Polygon> polygons ) {
    return csgjscpp::modelfrompolygons( polygons );
}

template<typename T, typename U>
bool FindFirstInVector( std::vector<std::shared_ptr<U>> vec, std::shared_ptr<T>& ptr ) {
    for( auto& item: vec ) {
        ptr = std::dynamic_pointer_cast<T>( item );
        if( ptr ) {
            return true;
        }
    }
    return false;
}

inline void AppendPointsToCurve( const std::vector<csgjscpp::Vector>& points_vec_src, std::vector<csgjscpp::Vector>& target_vec ) {
    if( points_vec_src.empty() ) {
        return;
    }

    // sometimes, sense agreement is not given correctly. try to correct sense of segment if necessary
    std::vector<csgjscpp::Vector> points_vec( points_vec_src );
    if( !target_vec.empty() && points_vec.size() > 1 ) {
        csgjscpp::Vector first_target_point = target_vec.front();
        csgjscpp::Vector last_target_point = target_vec.back();

        csgjscpp::Vector first_segment_point = points_vec.front();
        csgjscpp::Vector last_segment_point = points_vec.back();

        if( last_target_point == first_segment_point ) {
            // segment order is as expected, nothing to do
        } else {
            if( last_target_point == last_segment_point ) {
                // current segment seems to be in wrong order
                std::reverse( points_vec.begin(), points_vec.end() );
            } else {
                // maybe the current segment fits to the beginning of the target vector
                if( first_target_point == first_segment_point ) {
                    std::reverse( target_vec.begin(), target_vec.end() );
                } else {
                    if( first_target_point == last_segment_point ) {
                        std::reverse( target_vec.begin(), target_vec.end() );
                        std::reverse( points_vec.begin(), points_vec.end() );
                    }
                }
            }
        }
    }

    bool omit_first = false;
    if( !target_vec.empty() ) {
        csgjscpp::Vector last_point = target_vec.back();
        csgjscpp::Vector first_point_current_segment = points_vec.front();
        if( last_point == first_point_current_segment ) {
            omit_first = true;
        }
    }

    if( omit_first ) {
        target_vec.insert( target_vec.end(), points_vec.begin() + 1, points_vec.end() );
    } else {
        target_vec.insert( target_vec.end(), points_vec.begin(), points_vec.end() );
    }
    // TODO: handle all segments separately: std::vector<std::vector<vec3> >& target_vec
}

inline csgjscpp::Vector ClosestPointOnLine( const csgjscpp::Vector& point, const csgjscpp::Vector& line_origin, const csgjscpp::Vector& line_direction ) {
    const float denom = point.x * line_direction.x + point.y * line_direction.y + point.z * line_direction.z - line_direction.x * line_origin.x -
        line_direction.y * line_origin.y - line_direction.z * line_origin.z;
    const float numer = line_direction.x * line_direction.x + line_direction.y * line_direction.y + line_direction.z * line_direction.z;
    if( numer == 0.0 ) {
        // TODO: Log error
        return {};
    }
    const float lambda = denom / numer;
    return { line_origin.x + lambda * line_direction.x, line_origin.y + lambda * line_direction.y, line_origin.z + lambda * line_direction.z };
}

inline bool IsPointInPolySimple( const std::vector<csgjscpp::Vector>& points, const csgjscpp::Vector& p ) {
    if( points.empty() ) {
        return false;
    }
    size_t l = points.size();
    double s = 0.0;
    double rp, r0, d;

    rp = r0 = std::atan2( points[ 0 ].y - p.y, points[ 0 ].x - p.x );

    for( size_t i = 1; i < l; i++ ) {
        double r = atan2( points[ i ].y - p.y, points[ i ].x - p.x );
        d = r - rp;
        if( d > M_PI ) {
            d -= ( M_PI + M_PI );
        }
        if( d < -M_PI ) {
            d += ( M_PI + M_PI );
        }
        s = s + d;
        rp = r;
    }

    d = r0 - rp;
    if( d > M_PI ) {
        d -= ( M_PI + M_PI );
    }
    if( d < -M_PI ) {
        d += ( M_PI + M_PI );
    }
    s = s + d;

    bool is_zero = fabs( s ) < csgjscpp::csgjs_EPSILON;
    return !is_zero;
}

inline bool IsEnclosed( const std::vector<csgjscpp::Vector>& loop1, const std::vector<csgjscpp::Vector>& loop2 ) {
    bool all_points_inside = true;
    for( auto p1: loop1 ) {
        if( !IsPointInPolySimple( loop2, p1 ) ) {
            all_points_inside = false;
            break;
        }
    }
    return all_points_inside;
}
inline csgjscpp::Vector computePolygon2DNormal( const std::vector<std::array<double, 2>>& polygon ) {
    const size_t num_points = polygon.size();
    csgjscpp::Vector polygon_normal;
    for( int k = 0; k < num_points; ++k ) {
        const std::array<double, 2>& vertex_current = polygon[ k ];
        const std::array<double, 2>& vertex_next = polygon[ ( k + 1 ) % num_points ];
        polygon_normal.z += ( vertex_current[ 0 ] - vertex_next[ 0 ] ) * ( vertex_current[ 1 ] + vertex_next[ 1 ] );
    }
    return csgjscpp::unit( polygon_normal );
}
inline csgjscpp::Vector ComputePolygon2DNormal( const std::vector<csgjscpp::Vector>& polygon ) {
    const int num_points = (int)polygon.size();
    csgjscpp::Vector polygon_normal( 0, 0, 0 );
    for( int k = 0; k < num_points; ++k ) {
        const csgjscpp::Vector& vertex_current = polygon[ k ];
        const csgjscpp::Vector& vertex_next = polygon[ ( k + 1 ) % num_points ];
        polygon_normal.z += ( vertex_current.x - vertex_next.x ) * ( vertex_current.y + vertex_next.y );
    }
    return csgjscpp::unit( polygon_normal );
}

inline csgjscpp::Vector ComputePolygonNormal( const std::vector<csgjscpp::Vector>& polygon ) {
    csgjscpp::Vector polygon_normal;
    bool last_loop = false;
    for( auto it = polygon.begin();; ) {
        const auto& vertex_current = ( *it );
        ++it;
        if( it == polygon.end() ) {
            it = polygon.begin();
            last_loop = true;
        }
        const auto& vertex_next = ( *it );
        polygon_normal.x += ( vertex_current.y - vertex_next.y ) * ( vertex_current.z + vertex_next.z );
        polygon_normal.y += ( vertex_current.z - vertex_next.z ) * ( vertex_current.x + vertex_next.x );
        polygon_normal.z += ( vertex_current.x - vertex_next.x ) * ( vertex_current.y + vertex_next.y );
        if( last_loop ) {
            break;
        }
    }
    return csgjscpp::unit( polygon_normal );
}

inline csgjscpp::Vector BisectingPlane( const csgjscpp::Vector& v1, const csgjscpp::Vector& v2, const csgjscpp::Vector& v3 ) {
    auto v21 = v2 - v1;
    auto v32 = v3 - v2;
    float len21_square = csgjscpp::lengthsquared( v21 );
    float len32_square = csgjscpp::lengthsquared( v32 );

    if( len21_square <= csgjscpp::csgjs_EPSILON * len32_square ) {
        if( len32_square == 0.0 ) {
            // all three points lie ontop of one-another
            return {};
        } else {
            // return a normalized copy of v32 as bisector
            len32_square = 1.0 / len32_square;
            return csgjscpp::unit( v32 * len32_square );
        }

    } else {
        if( len32_square <= csgjscpp::csgjs_EPSILON * len21_square ) {
            // return v21 as bisector
            return csgjscpp::unit( v21 );
        } else {
            v21 = csgjscpp::unit( v21 );
            v32 = csgjscpp::unit( v32 );

            float dot_product = csgjscpp::dot( v32, v21 );
            float dot_product_abs = std::fabsf( dot_product );

            if( dot_product_abs > ( 1.0 + csgjscpp::csgjs_EPSILON ) || dot_product_abs < ( 1.0 - csgjscpp::csgjs_EPSILON ) ) {
                return csgjscpp::unit( ( v32 + v21 ) * dot_product - v32 - v21 );
            } else {
                // dot == 1 or -1, points are colinear
                return -v21;
            }
        }
    }
}

inline Matrix ConvertPlane2Matrix( const csgjscpp::Vector& plane_normal, const csgjscpp::Vector& plane_position, const csgjscpp::Vector& local_z ) {
    auto local_normal = csgjscpp::unit( plane_normal );
    auto local_z_new = local_z;

    auto local_y = csgjscpp::unit( csgjscpp::cross( local_normal, local_z_new ) );
    local_z_new = csgjscpp::unit( csgjscpp::cross( local_y, local_normal ) );

    return Matrix::CreateFromAxis( local_normal, local_y, local_z_new, plane_position );
}

template<typename T>
bool AllPointersValid( const std::vector<std::shared_ptr<T>>& vec ) {
    for( size_t ii = 0; ii < vec.size(); ++ii ) {
        const std::shared_ptr<T>& ptr = vec[ ii ];
        if( !ptr ) {
            return false;
        }
    }
    return true;
}
}


// CARVE TO CSGJSCPP HACKS
#include "Curve.h"
#include "Extruder.h"
#include "carve/input.hpp"
#include "ifcpp/Ifc/IfcAxis2Placement2D.h"
#include "ifcpp/Ifc/IfcAxis2Placement3D.h"
#include "ifcpp/Ifc/IfcCartesianPoint.h"
#include "ifcpp/Ifc/IfcCurve.h"
#include "ifcpp/Ifc/IfcDirection.h"
#include "ifcpp/Ifc/IfcLengthMeasure.h"
#include "ifcpp/Ifc/IfcReal.h"
namespace GeomUtils {
using namespace IFC4X3;
typedef carve::geom::vector<2> vec2;
typedef carve::geom::vector<3> vec3;
inline vec3 computePolygon2DNormal( const std::vector<vec2>& polygon ) {
    const int num_points = polygon.size();
    vec3 polygon_normal( carve::geom::VECTOR( 0, 0, 0 ) );
    for( int k = 0; k < num_points; ++k ) {
        const vec2& vertex_current = polygon[ k ];
        const vec2& vertex_next = polygon[ ( k + 1 ) % num_points ];
        polygon_normal[ 2 ] += ( vertex_current.x - vertex_next.x ) * ( vertex_current.y + vertex_next.y );
    }
    polygon_normal.normalize();
    return polygon_normal;
}
inline void closestPointOnLine( const vec3& point, const vec3& line_origin, const vec3& line_direction, vec3& closest ) {
    const double denom = point.x * line_direction.x + point.y * line_direction.y + point.z * line_direction.z - line_direction.x * line_origin.x -
        line_direction.y * line_origin.y - line_direction.z * line_origin.z;
    const double numer = line_direction.x * line_direction.x + line_direction.y * line_direction.y + line_direction.z * line_direction.z;
    if( numer == 0.0 ) {
        // throw BuildingException( "Line is degenerated: the line's direction vector is a null vector!", __FUNC__ );
        //  TODO: log error, throw smth...
    }
    const double lambda = denom / numer;
    closest =
        carve::geom::VECTOR( line_origin.x + lambda * line_direction.x, line_origin.y + lambda * line_direction.y, line_origin.z + lambda * line_direction.z );
}
inline void removeDuplicates( std::vector<carve::geom::vector<3>>& loop ) {
    typedef carve::geom::vector<2> vec2;
    typedef carve::geom::vector<3> vec3;
    if( loop.size() > 1 ) {
        auto it_loop = loop.begin();
        double previous_x = ( *it_loop ).x;
        double previous_y = ( *it_loop ).y;
        double previous_z = ( *it_loop ).z;
        ++it_loop;

        while( it_loop != loop.end() ) {
            vec3& current_point = *it_loop;
            if( std::abs( current_point.x - previous_x ) < csgjscpp::csgjs_EPSILON ) {
                if( std::abs( current_point.y - previous_y ) < csgjscpp::csgjs_EPSILON ) {
                    if( std::abs( current_point.z - previous_z ) < csgjscpp::csgjs_EPSILON ) {
                        previous_x = current_point.x;
                        previous_y = current_point.y;
                        previous_z = current_point.z;
                        it_loop = loop.erase( it_loop );
                        continue;
                    }
                }
            }
            previous_x = current_point.x;
            previous_y = current_point.y;
            previous_z = current_point.z;
            ++it_loop;
        }
    }
}

inline void removeDuplicates( std::vector<carve::geom::vector<2>>& loop ) {
    typedef carve::geom::vector<2> vec2;
    typedef carve::geom::vector<3> vec3;
    if( loop.size() > 1 ) {
        auto it_loop = loop.begin();
        double previous_x = ( *it_loop ).x;
        double previous_y = ( *it_loop ).y;
        ++it_loop;

        while( it_loop != loop.end() ) {
            vec2& current_point = *it_loop;
            if( std::abs( current_point.x - previous_x ) < csgjscpp::csgjs_EPSILON ) {
                if( std::abs( current_point.y - previous_y ) < csgjscpp::csgjs_EPSILON ) {
                    previous_x = current_point.x;
                    previous_y = current_point.y;
                    it_loop = loop.erase( it_loop );
                    continue;
                }
            }
            previous_x = current_point.x;
            previous_y = current_point.y;
            ++it_loop;
        }
    }
}

inline void removeDuplicates( std::vector<std::vector<carve::geom::vector<2>>>& paths ) {
    typedef carve::geom::vector<2> vec2;
    typedef carve::geom::vector<3> vec3;
    for( size_t ii = 0; ii < paths.size(); ++ii ) {
        std::vector<vec2>& loop = paths[ ii ];
        removeDuplicates( loop );
    }
}
inline void AddFacesFromCarvePolyhedron( const std::shared_ptr<carve::input::PolyhedronData>& polyhedron, std::vector<csgjscpp::Polygon>& polygons ) {
    int pos = 0;
    for( int i = 0; i < polyhedron->getFaceCount(); i++ ) {
        int verticesCount = polyhedron->faceIndices[ pos ];
        std::vector<csgjscpp::Vertex> vertices;
        for( int j = pos + 1; j < verticesCount; j++ ) {
            auto carveV = polyhedron->getVertex( polyhedron->faceIndices[ j ] );
            csgjscpp::Vertex v;
            v.pos = { (float)carveV.x, (float)carveV.y, (float)carveV.z };
            vertices.push_back( v );
        }
        polygons.emplace_back( vertices );
    }
}
inline vec3 computePolygonNormal( const std::vector<vec3>& polygon ) {
    vec3 polygon_normal( carve::geom::VECTOR( 0, 0, 0 ) );
    bool last_loop = false;
    for( std::vector<vec3>::const_iterator it = polygon.begin();; ) {
        const vec3& vertex_current = ( *it );
        ++it;
        if( it == polygon.end() ) {
            it = polygon.begin();
            last_loop = true;
        }
        const vec3& vertex_next = ( *it );
        polygon_normal[ 0 ] += ( vertex_current.y - vertex_next.y ) * ( vertex_current.z + vertex_next.z );
        polygon_normal[ 1 ] += ( vertex_current.z - vertex_next.z ) * ( vertex_current.x + vertex_next.x );
        polygon_normal[ 2 ] += ( vertex_current.x - vertex_next.x ) * ( vertex_current.y + vertex_next.y );
        if( last_loop ) {
            break;
        }
    }
    polygon_normal.normalize();
    return polygon_normal;
}

inline void getPlane( const std::shared_ptr<IFC4X3::IfcAxis2Placement3D>& axis2placement3d, carve::geom::plane<3>& plane, vec3& translate ) {
    const double length_factor = 1.0;
    vec3 location( carve::geom::VECTOR( 0.0, 0.0, 0.0 ) );
    vec3 local_x( carve::geom::VECTOR( 1.0, 0.0, 0.0 ) );
    vec3 local_y( carve::geom::VECTOR( 0.0, 1.0, 0.0 ) );
    vec3 local_z( carve::geom::VECTOR( 0.0, 0.0, 1.0 ) );
    vec3 ref_direction( carve::geom::VECTOR( 1.0, 0.0, 0.0 ) );

    if( axis2placement3d->m_Location ) {
        std::shared_ptr<IFC4X3::IfcCartesianPoint> cartesianPoint = dynamic_pointer_cast<IFC4X3::IfcCartesianPoint>( axis2placement3d->m_Location );
        if( cartesianPoint ) {
            std::vector<std::shared_ptr<IFC4X3::IfcLengthMeasure>>& coords = cartesianPoint->m_Coordinates;
            if( coords.size() > 2 ) {
                location =
                    carve::geom::VECTOR( coords[ 0 ]->m_value * length_factor, coords[ 1 ]->m_value * length_factor, coords[ 2 ]->m_value * length_factor );
            } else if( coords.size() > 1 ) {
                location = carve::geom::VECTOR( coords[ 0 ]->m_value * length_factor, coords[ 1 ]->m_value * length_factor, 0.0 );
            }
        } else {
        }
    }

    if( axis2placement3d->m_Axis ) {
        // local z-axis
        std::vector<shared_ptr<IFC4X3::IfcReal>>& axis = axis2placement3d->m_Axis->m_DirectionRatios;
        if( axis.size() > 2 ) {
            local_z = carve::geom::VECTOR( axis[ 0 ]->m_value, axis[ 1 ]->m_value, axis[ 2 ]->m_value );
        }
    }
    local_z.normalize();

    carve::geom::plane<3> p( local_z, location );
    plane.d = p.d;
    plane.N = local_z;
    translate = location;
}
inline void convertIfcAxis2Placement3D( const std::shared_ptr<IFC4X3::IfcAxis2Placement3D>& axis2placement3d, carve::math::Matrix& resulting_matrix,
                                        bool only_rotation = false ) {
    const double length_factor = 1.0;
    vec3 translate( carve::geom::VECTOR( 0.0, 0.0, 0.0 ) );
    vec3 local_x( carve::geom::VECTOR( 1.0, 0.0, 0.0 ) );
    vec3 local_y( carve::geom::VECTOR( 0.0, 1.0, 0.0 ) );
    vec3 local_z( carve::geom::VECTOR( 0.0, 0.0, 1.0 ) );
    vec3 ref_direction( carve::geom::VECTOR( 1.0, 0.0, 0.0 ) );

    if( !only_rotation ) {
        if( axis2placement3d->m_Location ) {
            shared_ptr<IFC4X3::IfcCartesianPoint> cartesianPoint = dynamic_pointer_cast<IFC4X3::IfcCartesianPoint>( axis2placement3d->m_Location );
            if( cartesianPoint ) {
                std::vector<std::shared_ptr<IFC4X3::IfcLengthMeasure>>& coords = cartesianPoint->m_Coordinates;
                if( coords.size() > 2 ) {
                    translate =
                        carve::geom::VECTOR( coords[ 0 ]->m_value * length_factor, coords[ 1 ]->m_value * length_factor, coords[ 2 ]->m_value * length_factor );
                } else if( coords.size() > 1 ) {
                    translate = carve::geom::VECTOR( coords[ 0 ]->m_value * length_factor, coords[ 1 ]->m_value * length_factor, 0.0 );
                }
            } else {
            }
        }
    }

    if( axis2placement3d->m_Axis ) {
        // local z-axis
        std::vector<shared_ptr<IFC4X3::IfcReal>>& axis = axis2placement3d->m_Axis->m_DirectionRatios;
        if( axis.size() > 2 ) {
            local_z = carve::geom::VECTOR( axis[ 0 ]->m_value, axis[ 1 ]->m_value, axis[ 2 ]->m_value );
        }
    }

    if( axis2placement3d->m_RefDirection ) {
        if( axis2placement3d->m_RefDirection->m_DirectionRatios.size() > 2 ) {
            ref_direction.x = axis2placement3d->m_RefDirection->m_DirectionRatios[ 0 ]->m_value;
            ref_direction.y = axis2placement3d->m_RefDirection->m_DirectionRatios[ 1 ]->m_value;
            ref_direction.z = axis2placement3d->m_RefDirection->m_DirectionRatios[ 2 ]->m_value;
        }
    }

    local_x = ref_direction;
    local_y = carve::geom::cross( local_z, local_x );
    // ref_direction can be just in the x-z-plane, not perpendicular to y and z. so re-compute local x
    local_x = carve::geom::cross( local_y, local_z );

    local_x.normalize();
    local_y.normalize();
    local_z.normalize();


    resulting_matrix = carve::math::Matrix( local_x.x, local_y.x, local_z.x, translate.x, local_x.y, local_y.y, local_z.y, translate.y, local_x.z, local_y.z,
                                            local_z.z, translate.z, 0, 0, 0, 1 );
}
inline void convertIfcAxis2Placement2D( const shared_ptr<IfcAxis2Placement2D>& axis2placement2d, carve::math::Matrix& resulting_matrix, bool only_rotation = false ) {
    typedef carve::geom::vector<2> vec2;
    typedef carve::geom::vector<3> vec3;
    vec3 translate( carve::geom::VECTOR( 0.0, 0.0, 0.0 ) );
    vec3 local_x( carve::geom::VECTOR( 1.0, 0.0, 0.0 ) );
    vec3 local_y( carve::geom::VECTOR( 0.0, 1.0, 0.0 ) );
    vec3 local_z( carve::geom::VECTOR( 0.0, 0.0, 1.0 ) );
    vec3 ref_direction( carve::geom::VECTOR( 1.0, 0.0, 0.0 ) );

    if( !only_rotation ) {
        if( axis2placement2d->m_Location ) {
            shared_ptr<IfcCartesianPoint> cartesianPoint = dynamic_pointer_cast<IfcCartesianPoint>( axis2placement2d->m_Location );
            if( cartesianPoint ) {
                std::vector<shared_ptr<IfcLengthMeasure>>& coords = cartesianPoint->m_Coordinates;
                if( coords.size() > 1 ) {
                    translate = carve::geom::VECTOR( coords[ 0 ]->m_value, coords[ 1 ]->m_value, 0.0 );
                }
            } else {
            }
        }
    }

    if( axis2placement2d->m_RefDirection ) {
        if( axis2placement2d->m_RefDirection->m_DirectionRatios.size() > 1 ) {
            if( axis2placement2d->m_RefDirection->m_DirectionRatios[ 0 ] ) {
                ref_direction.x = axis2placement2d->m_RefDirection->m_DirectionRatios[ 0 ]->m_value;
            }
            if( axis2placement2d->m_RefDirection->m_DirectionRatios[ 1 ] ) {
                ref_direction.y = axis2placement2d->m_RefDirection->m_DirectionRatios[ 1 ]->m_value;
            }
            ref_direction.z = 0;
        }
    }

    local_x = ref_direction;
    vec3 z_axis( carve::geom::VECTOR( 0.0, 0.0, 1.0 ) );
    local_y = carve::geom::cross( z_axis, local_x );
    // ref_direction can be just in the x-z-plane, not perpendicular to y and z. so re-compute local x
    local_x = carve::geom::cross( local_y, local_z );

    local_x.normalize();
    local_y.normalize();
    local_z.normalize();

    resulting_matrix = carve::math::Matrix( local_x.x, local_y.x, local_z.x, translate.x, local_x.y, local_y.y, local_z.y, translate.y, local_x.z, local_y.z,
                                            local_z.z, translate.z, 0, 0, 0, 1 );
}
inline carve::geom::aabb<3> computeBbox( const csgjscpp::Model& model ) {
    float minx = 0, miny = 0, minz = 0;
    float maxx = 0, maxy = 0, maxz = 0;
    if( !model.vertices.empty() ) {
        minx = model.vertices[ 0 ].pos.x;
        miny = model.vertices[ 0 ].pos.y;
        minz = model.vertices[ 0 ].pos.z;
        maxx = model.vertices[ 0 ].pos.x;
        maxy = model.vertices[ 0 ].pos.y;
        maxz = model.vertices[ 0 ].pos.z;
        for( const auto& v: model.vertices ) {
            minx = std::min( minx, v.pos.x );
            miny = std::min( miny, v.pos.y );
            minz = std::min( minz, v.pos.z );
            maxx = std::max( maxx, v.pos.x );
            maxy = std::max( maxy, v.pos.y );
            maxz = std::max( maxz, v.pos.z );
        }
    }
    return carve::geom::aabb<3>( carve::geom::VECTOR( maxx - minx, maxy - miny, maxz - minz ) * 0.5,
                                 carve::geom::VECTOR( maxx + minx, maxy + miny, maxz + minz ) * 0.5 );
}
inline void convertIfcCurve2D( const shared_ptr<IfcCurve>& ifc_curve, std::vector<carve::geom::vector<2>>& target_vec,
                               std::vector<carve::geom::vector<2>>& segment_start_points, std::vector<shared_ptr<IfcTrimmingSelect>>& trim1_vec,
                               std::vector<shared_ptr<IfcTrimmingSelect>>& trim2_vec, bool senseAgreement, float angleFactor ) {
    typedef carve::geom::vector<2> vec2;
    typedef carve::geom::vector<3> vec3;
    std::vector<csgjscpp::Vector> target_vec_3d;
    std::vector<csgjscpp::Vector> segment_start_points_3d;
    ifcpp::ConvertCurveInternal( ifc_curve, target_vec_3d, segment_start_points_3d, senseAgreement, angleFactor );

    for( size_t i = 0; i < target_vec_3d.size(); ++i ) {
        const auto& point_3d = target_vec_3d[ i ];
        target_vec.push_back( carve::geom::VECTOR( point_3d.x, point_3d.y ) );
    }
    for( size_t i = 0; i < segment_start_points_3d.size(); ++i ) {
        const auto& point_3d = segment_start_points_3d[ i ];
        segment_start_points.push_back( carve::geom::VECTOR( point_3d.x, point_3d.y ) );
    }
}

inline void convertIfcCurve2D( const shared_ptr<IfcCurve>& ifc_curve, std::vector<carve::geom::vector<2>>& loops,
                               std::vector<carve::geom::vector<2>>& segment_start_points, bool senseAgreement, float angleFactor ) {
    typedef carve::geom::vector<2> vec2;
    std::vector<shared_ptr<IfcTrimmingSelect>> trim1_vec;
    std::vector<shared_ptr<IfcTrimmingSelect>> trim2_vec;
    convertIfcCurve2D( ifc_curve, loops, segment_start_points, trim1_vec, trim2_vec, senseAgreement, angleFactor );
}
inline csgjscpp::Model Extrude( const std::vector<std::vector<vec2>>& faceLoopsInput, const vec3& extrusionVector ) {
    std::vector<std::vector<csgjscpp::Vector>> loops;
    csgjscpp::Vector ev( extrusionVector.x, extrusionVector.y, extrusionVector.z );
    for( const auto& l: faceLoopsInput ) {
        std::vector<csgjscpp::Vector> loop;
        for( const auto& v: l ) {
            loop.push_back( { (float)v.x, (float)v.y, 0.0f } );
        }
        loops.push_back( loop );
    }
    return ifcpp::Extrude( loops, ev );
}
}
