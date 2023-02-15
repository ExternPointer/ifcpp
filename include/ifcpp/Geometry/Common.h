#pragma once

#include <memory>
#include <vector>

namespace csgjscpp {
struct Vector;
struct Model;
}

namespace IFC4X3 {
class IfcVertex;
class IfcCartesianPoint;
class IfcObjectPlacement;
class IfcPlacement;
class IfcLengthMeasure;
class IfcAxis2Placement3D;
class IfcFace;
class IfcLoop;
class IfcEdge;
}

namespace ifcpp {

class Matrix;

csgjscpp::Vector ConvertPoint( const std::vector<std::shared_ptr<IFC4X3::IfcLengthMeasure>>& coords );
csgjscpp::Vector ConvertVertex( const std::shared_ptr<IFC4X3::IfcVertex>& vertex );
csgjscpp::Vector ConvertCartesianPoint( const std::shared_ptr<IFC4X3::IfcCartesianPoint>& point );
std::vector<csgjscpp::Vector> ConvertCartesianPoints( const std::vector<std::shared_ptr<IFC4X3::IfcCartesianPoint>>& points );
Matrix ConvertObjectPlacement( const std::shared_ptr<IFC4X3::IfcObjectPlacement>& objectPlacement );
Matrix ConvertAxis2Placement3D( const std::shared_ptr<IFC4X3::IfcAxis2Placement3D>& axis2placement3d );
Matrix ConvertPlacement( const std::shared_ptr<IFC4X3::IfcPlacement>& placement );
std::vector<csgjscpp::Vector> ConvertPoints( const std::vector<std::vector<std::shared_ptr<IFC4X3::IfcLengthMeasure>>>& pointList );
csgjscpp::Model ConvertFaceList( const std::vector<std::shared_ptr<IFC4X3::IfcFace>>& faces, float angleFactor /*, style data*/ );
std::vector<csgjscpp::Vector> ConvertLoop( const std::shared_ptr<IFC4X3::IfcLoop>& loop, float angleFactor );
std::vector<csgjscpp::Vector> ConvertEdge( const std::shared_ptr<IFC4X3::IfcEdge>& edge, float angleFactor );
}