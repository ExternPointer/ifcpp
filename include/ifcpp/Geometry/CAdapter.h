#pragma once

#include "csgjs.h"
#include "ifcpp/Ifc/IfcObjectDefinition.h"
#include <concepts>
#include <memory>

namespace ifcpp {

template<typename TVector>
concept CVector = requires( TVector v, float f ) {
                      { TVector() } -> std::same_as<TVector>;
                      { v.x } -> std::convertible_to<float>;
                      { v.y } -> std::convertible_to<float>;
                      { v.z } -> std::convertible_to<float>;
                      { v + v } -> std::same_as<TVector>;
                      { v - v } -> std::same_as<TVector>;
                      { -v } -> std::same_as<TVector>;
                      { v * f } -> std::same_as<TVector>;
                  };

template<typename TAdapter>
concept CAdapter = CVector<typename TAdapter::TVector> &&
    requires( TAdapter adapter, std::vector<typename TAdapter::TPolygon>& polygons, std::vector<typename TAdapter::TPolyline>& polylines,
              typename TAdapter::TVector& vector, std::vector<typename TAdapter::TVector>& vertices, typename TAdapter::TMaterial& material,
              typename TAdapter::TMatrix matrix, std::shared_ptr<IFC4X3::IfcObjectDefinition>& ifcObject ) {
        { adapter.CreateEntity( ifcObject, polygons, polylines ) } -> std::same_as<typename TAdapter::TEntity>;
        { adapter.Transform( &polygons, matrix ) } -> std::same_as<void>;
        { adapter.Transform( &polylines, matrix ) } -> std::same_as<void>;

        { adapter.CreatePolyline( vertices ) } -> std::same_as<typename TAdapter::TPolyline>;
        //{ adapter.CreatePolygon( vertices, material ) } -> std::same_as<typename TAdapter::TPolygon>;
        //{ adapter.ComputeUnion( polygons, polygons ) } -> std::same_as<std::vector<typename TAdapter::Polygon>>;
        //{ adapter.ComputeIntersection( polygons, polygons ) } -> std::same_as<std::vector<typename TAdapter::Polygon>>;
        //{ adapter.ComputeSubtraction( polygons, polygons ) } -> std::same_as<std::vector<typename TAdapter::TPolygon>>;
    };
}
