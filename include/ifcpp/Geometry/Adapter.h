#pragma once

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