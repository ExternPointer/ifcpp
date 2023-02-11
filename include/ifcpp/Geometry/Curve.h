#pragma once

#include <memory>
#include <vector>

namespace csgjscpp {
struct Vector;
}

namespace IFC4X3 {
class IfcCurve;
class IfcTrimmingSelect;
class IfcBSplineCurve;
}

namespace ifcpp {

class Matrix;

std::vector<csgjscpp::Vector> ConvertCurve( const std::shared_ptr<IFC4X3::IfcCurve>& ifc_curve, bool senseAgreement, float planeAngleFactor );
std::vector<csgjscpp::Vector> ConvertCurve( const std::shared_ptr<IFC4X3::IfcCurve>& ifc_curve,
                                            std::vector<std::shared_ptr<IFC4X3::IfcTrimmingSelect>>& trim1_vec,
                                            std::vector<std::shared_ptr<IFC4X3::IfcTrimmingSelect>>& trim2_vec, bool senseAgreement, float planeAngleFactor );

void ConvertCurveInternal( const std::shared_ptr<IFC4X3::IfcCurve>& ifc_curve, std::vector<csgjscpp::Vector>& loops,
                   std::vector<csgjscpp::Vector>& segment_start_points, bool senseAgreement, float planeAngleFactor );
void ConvertCurveInternal( const std::shared_ptr<IFC4X3::IfcCurve>& ifc_curve, std::vector<csgjscpp::Vector>& target_vec,
                   std::vector<csgjscpp::Vector>& segment_start_points, std::vector<std::shared_ptr<IFC4X3::IfcTrimmingSelect>>& trim1_vec,
                   std::vector<std::shared_ptr<IFC4X3::IfcTrimmingSelect>>& trim2_vec, bool senseAgreement, float planeAngleFactor );
void getTrimAngles( const std::vector<std::shared_ptr<IFC4X3::IfcTrimmingSelect>>& trim1_vec,
                    const std::vector<std::shared_ptr<IFC4X3::IfcTrimmingSelect>>& trim2_vec, csgjscpp::Vector circle_center, float circle_radius,
                    bool senseAgreement, float& trimAngle1, float& trimAngle2, float& startAngle, float& openingAngle, const Matrix& circlePlacement,
                    const Matrix& circlePlacementInverse, float planeAngleFactor );
void getTrimPoints( const std::vector<std::shared_ptr<IFC4X3::IfcTrimmingSelect>>& trim1_vec,
                    const std::vector<std::shared_ptr<IFC4X3::IfcTrimmingSelect>>& trim2_vec, const Matrix& circlePosition, float circleRadius,
                    float circleRadius2, bool senseAgreement, csgjscpp::Vector& trimPoint1, csgjscpp::Vector& trimPoint2, float planeAngleFactor );
float GetAngleOnCircle( const csgjscpp::Vector& circleCenter, float radius, csgjscpp::Vector& trimPoint, const Matrix& circlePosition,
                        const Matrix& circlePositionInverse );
float trimPointCircleDistance( float angle, float radius, const Matrix& circlePosition, const csgjscpp::Vector& trimPoint );
float regula( float x0, float x1, float fx0, float fx1 );
void ConvertBSplineCurve( const std::shared_ptr<IFC4X3::IfcBSplineCurve>& bspline_curve, std::vector<csgjscpp::Vector>& target_vec,
                          std::vector<csgjscpp::Vector>& segment_start_points );
void ComputeRationalBSpline( const size_t order, const size_t numCurvePoints, const std::vector<csgjscpp::Vector>& controlPoints, std::vector<float>& weights,
                             std::vector<float>& knotVec, std::vector<float>& curvePoints );
void ComputeKnotVector( const size_t numControlPoints, const size_t order, std::vector<float>& knotVector );
static void ComputRationalBasisFunctions( const size_t order, const float t, const size_t numControlPoints, const std::vector<float>& knotVec,
                                          std::vector<float>& weights, std::vector<float>& basisFunc );
}