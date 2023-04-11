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


/*
std::vector<std::shared_ptr<Entity>> GenerateGeometry( const std::shared_ptr<BuildingModel>& ifcModel );


std::shared_ptr<Entity> GenerateGeometryFromObject( const std::shared_ptr<IFC4X3::IfcObjectDefinition>& object, float lengthFactor, float angleFactor );
void SubtractOpenings( const std::shared_ptr<Entity>& geometry, float lengthFactor, float angleFactor );
csgjscpp::Model ConvertGeometryRepresentation( const std::shared_ptr<IFC4X3::IfcGeometricRepresentationItem>& geometric, float angleFactor );


csgjscpp::Model ConvertFaceBasedSurfaceModel( const std::shared_ptr<IFC4X3::IfcFaceBasedSurfaceModel>& surfaceModel, float angleFactor );
csgjscpp::Model ConvertShellBasedSurfaceModel( const std::shared_ptr<IFC4X3::IfcShellBasedSurfaceModel>& shellModel, float angleFactor );
csgjscpp::Model ConvertBooleanResult( const std::shared_ptr<IFC4X3::IfcBooleanResult>& bool_result, float angleFactor );
csgjscpp::Model ConvertSolidModel( const std::shared_ptr<IFC4X3::IfcSolidModel>& bool_result, float angleFactor );
csgjscpp::Model ConvertSurface( const std::shared_ptr<IFC4X3::IfcSurface>& surface );
*/

}
