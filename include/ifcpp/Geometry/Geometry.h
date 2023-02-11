#pragma once

#include <memory>
#include <vector>

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
class IfcFaceBasedSurfaceModel;
class IfcShellBasedSurfaceModel;
class IfcObjectPlacement;
class IfcPlacement;
class IfcCartesianPoint;
class IfcEdge;
class IfcVertex;
class IfcCurve;
class IfcTrimmingSelect;
}

namespace ifcpp {

class Matrix;

class Geometry {
public:
    std::shared_ptr<IFC4X3::IfcObjectDefinition> m_object;
    std::vector<csgjscpp::Model> m_meshes;
    void ApplyTransformation( const Matrix& matrix );
};

std::vector<std::shared_ptr<Geometry>> GenerateGeometry( const std::shared_ptr<BuildingModel>& ifcModel );


std::shared_ptr<Geometry> GenerateGeometryFromObject( const std::shared_ptr<IFC4X3::IfcObjectDefinition>& object, float lengthFactor, float angleFactor );
csgjscpp::Model ConvertGeometryRepresentation( const std::shared_ptr<IFC4X3::IfcGeometricRepresentationItem>& geometric, float angleFactor );


csgjscpp::Model ConvertFaceBasedSurfaceModel( const std::shared_ptr<IFC4X3::IfcFaceBasedSurfaceModel>& surfaceModel, float angleFactor /* style data */ );
csgjscpp::Model ConvertShellBasedSurfaceModel( const std::shared_ptr<IFC4X3::IfcShellBasedSurfaceModel>& shellModel, float angleFactor /* style data */ );
csgjscpp::Model ConvertFaceList( const std::vector<std::shared_ptr<IFC4X3::IfcFace>>& faces, float angleFactor /*, style data*/ );
csgjscpp::Model ConvertBooleanResult( const std::shared_ptr<IFC4X3::IfcBooleanResult>& bool_result /*, style data*/ );
csgjscpp::Model ConvertSolidModel( const std::shared_ptr<IFC4X3::IfcSolidModel>& bool_result /*, style data*/ );
csgjscpp::Model ConvertSurface( const std::shared_ptr<IFC4X3::IfcSurface>& surface /*, style data*/ );
std::vector<csgjscpp::Vector> ConvertLoop( const std::shared_ptr<IFC4X3::IfcLoop>& loop, float angleFactor );
std::vector<csgjscpp::Vector> ConvertEdge( const std::shared_ptr<IFC4X3::IfcEdge>& edge, float angleFactor );

}
