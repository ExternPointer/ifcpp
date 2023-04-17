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
    { v* f } -> std::same_as<TVector>;
};

template<typename TAdapter>
concept CAdapter = CVector<typename TAdapter::TVector> &&
    requires( TAdapter adapter, std::vector<typename TAdapter::TVector>& vertices ) {
        { adapter.CreatePolyline( vertices ) } -> std::same_as<typename TAdapter::TPolyline>;
    };
}
