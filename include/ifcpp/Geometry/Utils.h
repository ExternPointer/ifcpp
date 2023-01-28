#pragma once

#include "csgjs.h"
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

}