#pragma once

#include <vector>
#include <memory>
#include "csgjs.h"

class BuildingModel;
namespace IFC4X3 {
class IfcObjectDefinition;
class IfcGeometricRepresentationItem;
class IfcFace;
class IfcBooleanResult;
class IfcSolidModel;
class IfcSurface;
class IfcLoop;
}

namespace ifcpp {

struct Geometry {
    std::shared_ptr<IFC4X3::IfcObjectDefinition> m_object;
    std::vector<csgjscpp::Model> m_meshes;
};

std::vector<std::shared_ptr<Geometry>> GenerateGeometry( std::shared_ptr<BuildingModel> ifcModel );

namespace details {
    std::shared_ptr<Geometry> GenerateGeometryFromObject( std::shared_ptr<IFC4X3::IfcObjectDefinition> ifc_object_def );
    csgjscpp::Model ConvertGeometryRepresentation( std::shared_ptr<IFC4X3::IfcGeometricRepresentationItem> geom_item );

    csgjscpp::Model ConvertFaceList( const std::vector<std::shared_ptr<IFC4X3::IfcFace> >& vec_faces /*, style data*/ );
    csgjscpp::Model ConvertBooleanResult( const std::shared_ptr<IFC4X3::IfcBooleanResult>& bool_result /*, style data*/ );
    csgjscpp::Model ConvertSolidModel( const std::shared_ptr<IFC4X3::IfcSolidModel>& bool_result /*, style data*/ );
    csgjscpp::Model ConvertSurface( const std::shared_ptr<IFC4X3::IfcSurface>& surface /*, style data*/ );
    std::vector<csgjscpp::Vector> ConvertLoop( const std::shared_ptr<IFC4X3::IfcLoop> loop );
}

}
