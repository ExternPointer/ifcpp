#pragma once

#include <memory>

namespace IFC4X3{
class IfcSolidModel;
class IfcBooleanResult;
class IfcCsgPrimitive3D;
}

namespace csgjscpp {
struct Model;
}

namespace ifcpp {
csgjscpp::Model convertIfcSolidModel( const std::shared_ptr<IFC4X3::IfcSolidModel>& solid_model, float angleFactor );
csgjscpp::Model convertIfcBooleanResult( const std::shared_ptr<IFC4X3::IfcBooleanResult>& bool_result, float angleFactor );
csgjscpp::Model convertIfcCsgPrimitive3D( const std::shared_ptr<IFC4X3::IfcCsgPrimitive3D>& csg_primitive, float angleFactor );
}