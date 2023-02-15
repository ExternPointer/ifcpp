#pragma once


#include <array>
#include <vector>
#include "carve/vector.hpp"
#include "carve/input.hpp"

namespace csgjscpp {
struct Vector;
struct Model;
}

namespace ifcpp {

double signedArea( const std::vector<std::array<double, 2>>& points );
void polygons2flatVec( const std::vector<std::vector<std::array<double, 2>>>& polyIn, std::vector<std::array<double, 2>>& polyOut );
bool bisectingPlane( const carve::geom::vector<3>& v1, const carve::geom::vector<3>& v2, const carve::geom::vector<3>& v3, carve::geom::vector<3>& normal );
void AddFacesFromCarvePolyhedron( const std::shared_ptr<carve::input::PolyhedronData>& polyhedron, std::vector<csgjscpp::Polygon>& polygons );

void FindEnclosedLoops( const std::vector<std::vector<csgjscpp::Vector>>& face_loops_input,
                        std::vector<std::vector<std::vector<csgjscpp::Vector>>>& profile_paths_enclosed );

void TriangulateLoops( const std::vector<std::vector<csgjscpp::Vector>>& profile_paths_input,
                       std::vector<std::vector<csgjscpp::Vector>>& face_loops_used_for_triangulation, std::vector<int>& face_indexes_out );
csgjscpp::Model Extrude( const std::vector<std::vector<csgjscpp::Vector>>& faceLoopsInput, const csgjscpp::Vector& extrusionVector );
csgjscpp::Model sweepDisk( const std::vector<csgjscpp::Vector>& curve_points, size_t nvc, double radius, double radius_inner = -1 );
csgjscpp::Model SweepArea( const std::vector<csgjscpp::Vector>& curve_points, const std::vector<std::vector<csgjscpp::Vector>>& profile_paths_input );

}