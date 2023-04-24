#pragma once

#include <concepts>

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