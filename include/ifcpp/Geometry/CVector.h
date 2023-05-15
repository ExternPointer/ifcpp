#pragma once

#include <concepts>

template<typename TVector>
concept CVector = requires( TVector v, double f ) {
    { TVector() } -> std::same_as<TVector>;
    { v.x } -> std::convertible_to<double>;
    { v.y } -> std::convertible_to<double>;
    { v.z } -> std::convertible_to<double>;
    { v + v } -> std::same_as<TVector>;
    { v - v } -> std::same_as<TVector>;
    { -v } -> std::same_as<TVector>;
    { v* f } -> std::same_as<TVector>;
};