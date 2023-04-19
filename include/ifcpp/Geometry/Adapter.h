#pragma once

#include "../third_party/earcut/include/mapbox/earcut.hpp"
#include <vector>

#include "csgjs.h"
#include "ifcpp/Ifc/IfcObjectDefinition.h"

namespace ifcpp {

class Material {
public:
    unsigned int m_color;
};

class Polyline {
public:
    std::vector<csgjscpp::Vector> m_points;
    Material m_material;
};

class Entity {
public:
    std::shared_ptr<IFC4X3::IfcObjectDefinition> m_ifcObject;
    std::vector<csgjscpp::Polygon> m_polygons;
    std::vector<Polyline> m_polylines;
};


class Adapter {
public:
    using TEntity = Entity;
    using TPolygon = csgjscpp::Polygon;
    using TPolyline = Polyline;
    using TVector = csgjscpp::Vector;
    using TMaterial = Material;

    inline TPolyline CreatePolyline( const std::vector<TVector>& vertices ) {
        return { vertices };
    }

    inline TEntity CreateEntity( const std::shared_ptr<IFC4X3::IfcObjectDefinition>& ifcObject, const std::vector<TPolygon>& meshes,
                                 const std::vector<TPolyline>& polylines ) {
        return { ifcObject, meshes, polylines };
    }
    inline void Transform( std::vector<TPolygon>* polygons, Matrix<TVector> matrix ) {
        for( auto& p: *polygons ) {
            for( auto& v: p.vertices ) {
                matrix.Transform( &v.pos );
            }
        }
    }
    inline void Transform( std::vector<TPolyline>* polylines, Matrix<TVector> matrix ) {
        for( auto& p: *polylines ) {
            for( auto& v: p.m_points ) {
                matrix.Transform( &v );
            }
        }
    }

    inline TPolygon CreatePolygon( std::vector<TVector> vertices, std::vector<int> indices ) {
        if( indices.size() != 3 ) {
            // TODO: Log error
            // WTF???
            return {};
        }
        return csgjscpp::Polygon( { { vertices[ indices[ 0 ] ] }, { vertices[ indices[ 1 ] ] }, { vertices[ indices[ 2 ] ] } } )
    }

    inline std::vector<int> Triangulate( std::vector<TVector> loop ) {
        if( loop.size() < 3 ) {
            // WTF, TODO: Log error
            return {};
        }
        const auto normal = csgjscpp::unit( -csgjscpp::cross( loop[ 0 ] - loop[ 1 ], loop[ 2 ] - loop[ 1 ] ) );
        auto right = csgjscpp::cross( { 0.0f, 0.0f, 1.0f }, normal );
        if( csgjscpp::lengthsquared( right ) < csgjscpp::csgjs_EPSILON ) {
            right = csgjscpp::cross( { 1.0f, 0.0f, 0.0f }, normal );
        }
        if( csgjscpp::lengthsquared( right ) < csgjscpp::csgjs_EPSILON ) {
            // WTF, |normal| == 0 ?????
            return {};
        }
        auto up = csgjscpp::cross( normal, right );
        right = csgjscpp::unit( right );
        up = csgjscpp::unit( up );
        std::vector<std::vector<std::tuple<float, float>>> polygon;
        std::vector<std::tuple<float, float>> outer;
        for( const auto& p: loop ) {
            outer.emplace_back( csgjscpp::dot( right, p ), csgjscpp::dot( up, p ) );
        }
        polygon.push_back( outer );
        return mapbox::earcut<int>( polygon );
    }

    inline std::vector<TPolygon> ComputeUnion( const std::vector<TPolygon>& operand1, const std::vector<TPolygon>& operand2 ) {
        return csgjscpp::csgunion( operand1, operand2 );
    }

    inline std::vector<TPolygon> ComputeIntersection( const std::vector<TPolygon>& operand1, const std::vector<TPolygon>& operand2 ) {
        return csgjscpp::csgintersection( operand1, operand2 );
    }

    inline std::vector<TPolygon> ComputeDifference( const std::vector<TPolygon>& operand1, const std::vector<TPolygon>& operand2 ) {
        return csgjscpp::csgsubtract( operand1, operand2 );
    }

    // inline TPolyline CreatePolyline( const std::vector<TVector>& vertices, const TMaterial& material ) {
    //     return { vertices, material };
    // }
    // inline TPolygon CreatePolygon( const std::vector<TVector>& vertices ) {
    //     std::vector<csgjscpp::Vertex> csgjsVertices;
    //     for( const auto& v: vertices ) {
    //         csgjsVertices.push_back( csgjscpp::Vertex { v } );
    //     }
    //     return { csgjsVertices };
    // }
    //  inline std::vector<TMesh> ComputeUnion(const std::vector<TMesh>& operand1, const std::vector<TMesh>& operand2) {
    //      std::vector<TMesh> result;
    //      for(const auto& a: operand1) {
    //          for(const auto& b: operand2) {

    //        }
    //    }
    //}
};

}