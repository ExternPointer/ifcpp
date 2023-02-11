#pragma once

#include "csgjs.h"
#include <cmath>
#include <memory>
#include <vector>

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
    const auto normal = csgjscpp::unit( csgjscpp::cross( loop[ 1 ] - loop[ 0 ], loop[ 2 ] - loop[ 0 ] ) );
    std::vector<csgjscpp::Vertex> vertices;
    for( const auto& position: loop ) {
        vertices.push_back( { position, normal, 4278190335 /*style data*/ } );
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

}