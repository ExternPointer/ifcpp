#pragma once

#include <concepts>
#include <memory>

#include "ifcpp/Ifc/IfcObjectDefinition.h"

#include "ifcpp/Geometry/CVector.h"
#include "ifcpp/Geometry/Matrix.h"


namespace ifcpp {

template<typename TAdapter>
concept CAdapter = CVector<typename TAdapter::TVector> &&
    requires( TAdapter adapter, std::vector<typename TAdapter::TVector>& vertices, std::vector<int> indices,
              std::shared_ptr<IFC4X3::IfcObjectDefinition> ifcObjectDifinition, std::vector<typename TAdapter::TTriangle> triangles,
              std::vector<typename TAdapter::TMesh> meshes, std::vector<typename TAdapter::TPolyline> polylines, Matrix<typename TAdapter::TVector> matrix,
              std::vector<typename TAdapter::TVector> loop, std::vector<typename TAdapter::TMesh> operand1, std::vector<typename TAdapter::TMesh> operand2 ) {
        { adapter.CreatePolyline( vertices ) } -> std::same_as<typename TAdapter::TPolyline>;
        { adapter.CreateTriangle( vertices, indices ) } -> std::same_as<typename TAdapter::TTriangle>;
        { adapter.CreateMesh( triangles ) } -> std::same_as<typename TAdapter::TMesh>;
        { adapter.CreateEntity( ifcObjectDifinition, meshes, polylines ) } -> std::same_as<typename TAdapter::TEntity>;
        { adapter.Transform( &meshes, matrix ) } -> std::same_as<void>;
        { adapter.Transform( &polylines, matrix ) } -> std::same_as<void>;
        { adapter.Triangulate( loop ) } -> std::same_as<std::vector<int>>;
        { adapter.ComputeUnion( operand1, operand2 ) } -> std::same_as<std::vector<typename TAdapter::TMesh>>;
        { adapter.ComputeIntersection( operand1, operand2 ) } -> std::same_as<std::vector<typename TAdapter::TMesh>>;
        { adapter.ComputeDifference( operand1, operand2 ) } -> std::same_as<std::vector<typename TAdapter::TMesh>>;
    };

}
