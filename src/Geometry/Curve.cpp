#include "ifcpp/Geometry/Curve.h"
#include "ifcpp/Geometry/Common.h"
#include "ifcpp/Geometry/Matrix.h"
#include "ifcpp/Geometry/Utils.h"
#include "ifcpp/Geometry/csgjs.h"

#include "ifcpp/Ifc/IfcArcIndex.h"
#include "ifcpp/Ifc/IfcAxis2Placement.h"
#include "ifcpp/Ifc/IfcAxis2Placement3D.h"
#include "ifcpp/Ifc/IfcBSplineCurve.h"
#include "ifcpp/Ifc/IfcBSplineCurveWithKnots.h"
#include "ifcpp/Ifc/IfcBoolean.h"
#include "ifcpp/Ifc/IfcCartesianPoint.h"
#include "ifcpp/Ifc/IfcCartesianPointList.h"
#include "ifcpp/Ifc/IfcCartesianPointList2D.h"
#include "ifcpp/Ifc/IfcCartesianPointList3D.h"
#include "ifcpp/Ifc/IfcCircle.h"
#include "ifcpp/Ifc/IfcCompositeCurve.h"
#include "ifcpp/Ifc/IfcCompositeCurveSegment.h"
#include "ifcpp/Ifc/IfcConnectedFaceSet.h"
#include "ifcpp/Ifc/IfcCurveSegment.h"
#include "ifcpp/Ifc/IfcDirection.h"
#include "ifcpp/Ifc/IfcEllipse.h"
#include "ifcpp/Ifc/IfcGeometricSet.h"
#include "ifcpp/Ifc/IfcIndexedPolyCurve.h"
#include "ifcpp/Ifc/IfcLine.h"
#include "ifcpp/Ifc/IfcLineIndex.h"
#include "ifcpp/Ifc/IfcLocalPlacement.h"
#include "ifcpp/Ifc/IfcOffsetCurve2D.h"
#include "ifcpp/Ifc/IfcOffsetCurve3D.h"
#include "ifcpp/Ifc/IfcParameterValue.h"
#include "ifcpp/Ifc/IfcPcurve.h"
#include "ifcpp/Ifc/IfcPlacement.h"
#include "ifcpp/Ifc/IfcPolyline.h"
#include "ifcpp/Ifc/IfcPositiveLengthMeasure.h"
#include "ifcpp/Ifc/IfcRationalBSplineCurveWithKnots.h"
#include "ifcpp/Ifc/IfcReal.h"
#include "ifcpp/Ifc/IfcTrimmedCurve.h"
#include "ifcpp/Ifc/IfcVector.h"


using namespace IFC4X3;

namespace ifcpp {

int numVerticesPerControlPoint = 4;
int numVerticesPerCircle = 14;
int minNumVerticesPerArc = 5;


std::vector<csgjscpp::Vector> ConvertCurve( const std::shared_ptr<IFC4X3::IfcCurve>& ifc_curve, bool senseAgreement, float planeAngleFactor ) {
    std::vector<csgjscpp::Vector> result;
    std::vector<csgjscpp::Vector> segment_start_points;
    ConvertCurveInternal( ifc_curve, result, segment_start_points, senseAgreement, planeAngleFactor );
    return result;
}
std::vector<csgjscpp::Vector> ConvertCurve( const std::shared_ptr<IFC4X3::IfcCurve>& ifc_curve,
                                            std::vector<std::shared_ptr<IFC4X3::IfcTrimmingSelect>>& trim1_vec,
                                            std::vector<std::shared_ptr<IFC4X3::IfcTrimmingSelect>>& trim2_vec, bool senseAgreement, float planeAngleFactor ) {
    std::vector<csgjscpp::Vector> result;
    std::vector<csgjscpp::Vector> segment_start_points;
    ConvertCurveInternal( ifc_curve, result, segment_start_points, trim1_vec, trim2_vec, senseAgreement, planeAngleFactor );
    return result;
}

void ConvertCurveInternal( const std::shared_ptr<IFC4X3::IfcCurve>& ifc_curve, std::vector<csgjscpp::Vector>& loops,
                   std::vector<csgjscpp::Vector>& segment_start_points, bool senseAgreement, float planeAngleFactor ) {
    std::vector<shared_ptr<IfcTrimmingSelect>> trim1_vec;
    std::vector<shared_ptr<IfcTrimmingSelect>> trim2_vec;
    ConvertCurveInternal( ifc_curve, loops, segment_start_points, trim1_vec, trim2_vec, senseAgreement, planeAngleFactor );
}
void ConvertCurveInternal( const std::shared_ptr<IFC4X3::IfcCurve>& ifc_curve, std::vector<csgjscpp::Vector>& target_vec,
                   std::vector<csgjscpp::Vector>& segment_start_points, std::vector<std::shared_ptr<IFC4X3::IfcTrimmingSelect>>& trim1_vec,
                   std::vector<std::shared_ptr<IFC4X3::IfcTrimmingSelect>>& trim2_vec, bool senseAgreement, float planeAngleFactor ) {
    //	ENTITY IfcCurve ABSTRACT SUPERTYPE OF	(ONEOF(IfcBoundedCurve, IfcConic, IfcLine, IfcOffsetCurve2D, IfcOffsetCurve3D, IfcPCurve))
    shared_ptr<IfcBoundedCurve> bounded_curve = dynamic_pointer_cast<IfcBoundedCurve>( ifc_curve );
    if( bounded_curve ) {
        shared_ptr<IfcCompositeCurve> composite_curve = dynamic_pointer_cast<IfcCompositeCurve>( bounded_curve );
        if( composite_curve ) {
            // ENTITY IfcBoundedCurve ABSTRACT SUPERTYPE OF	(ONEOF(IfcCompositeCurve, IfcPolyline, IfcTrimmedCurve, IfcBSplineCurve))
            // std::vector<shared_ptr<IfcSegment> >					m_Segments;
            std::vector<shared_ptr<IfcSegment>>& vec_segements = composite_curve->m_Segments;
            for( auto segement: vec_segements ) {
                // ENTITY IfcSegment ABSTRACT SUPERTYPE OF (ONEOF (IfcCompositeCurveSegment ,IfcCurveSegment))
                shared_ptr<IfcCompositeCurveSegment> compositeCurveSegment = dynamic_pointer_cast<IfcCompositeCurveSegment>( segement );
                if( compositeCurveSegment ) {
                    shared_ptr<IfcCurve> segement_curve = compositeCurveSegment->m_ParentCurve;

                    std::vector<csgjscpp::Vector> segment_vec;
                    ConvertCurveInternal( segement_curve, segment_vec, segment_start_points, senseAgreement, planeAngleFactor );
                    if( !segment_vec.empty() ) {
                        AppendPointsToCurve( segment_vec, target_vec );
                    }
                    continue;
                }

                shared_ptr<IfcCurveSegment> curveSegment = dynamic_pointer_cast<IfcCurveSegment>( segement );
                if( curveSegment ) {
                    // shared_ptr<IfcPlacement>								m_Placement;
                    // shared_ptr<IfcCurveMeasureSelect>						m_SegmentStart;
                    // shared_ptr<IfcCurveMeasureSelect>						m_SegmentLength;
                    // shared_ptr<IfcCurve>									m_ParentCurve;

                    shared_ptr<IfcCurve> segement_curve = curveSegment->m_ParentCurve;

                    std::vector<csgjscpp::Vector> segment_vec;
                    ConvertCurveInternal( segement_curve, segment_vec, segment_start_points, senseAgreement, planeAngleFactor );
                    if( !segment_vec.empty() ) {
                        AppendPointsToCurve( segment_vec, target_vec );
                    }
                    continue;
                }
            }
            return;
        }

        shared_ptr<IfcPolyline> poly_line = dynamic_pointer_cast<IfcPolyline>( ifc_curve );
        if( poly_line ) {
            std::vector<shared_ptr<IfcCartesianPoint>>& points = poly_line->m_Points;
            if( !points.empty() ) {
                auto convertedPoints = ConvertCartesianPoints( points );
                std::copy( convertedPoints.begin(), convertedPoints.end(), std::back_inserter( target_vec ) );
                shared_ptr<IfcCartesianPoint>& first_point = points[ 0 ];
                if( first_point->m_Coordinates.size() > 1 ) {
                    if( first_point->m_Coordinates[ 0 ] && first_point->m_Coordinates[ 1 ] ) {
                        segment_start_points.emplace_back( static_cast<float>( first_point->m_Coordinates[ 0 ]->m_value ),
                                                           static_cast<float>( first_point->m_Coordinates[ 1 ]->m_value ), 0.0f );
                    }
                }
            }
            return;
        }

        shared_ptr<IfcTrimmedCurve> trimmed_curve = dynamic_pointer_cast<IfcTrimmedCurve>( bounded_curve );
        if( trimmed_curve ) {
            shared_ptr<IfcCurve> basis_curve = trimmed_curve->m_BasisCurve;
            if( basis_curve ) {
                std::vector<csgjscpp::Vector> basis_curve_points;
                std::vector<csgjscpp::Vector> basis_curve_start_points;
                std::vector<shared_ptr<IfcTrimmingSelect>>& curve_trim1_vec = trimmed_curve->m_Trim1;
                std::vector<shared_ptr<IfcTrimmingSelect>>& curve_trim2_vec = trimmed_curve->m_Trim2;
                bool trimmed_senseAgreement = true;
                if( trimmed_curve->m_SenseAgreement ) {
                    trimmed_senseAgreement = trimmed_curve->m_SenseAgreement->m_value;
                }
                if( !senseAgreement ) {
                    trimmed_senseAgreement = !trimmed_senseAgreement;
                }

                ConvertCurveInternal( basis_curve, basis_curve_points, basis_curve_start_points, curve_trim1_vec, curve_trim2_vec, trimmed_senseAgreement,
                                      planeAngleFactor );

                AppendPointsToCurve( basis_curve_points, target_vec );
                AppendPointsToCurve( basis_curve_start_points, segment_start_points );
            }
            return;
        }

        shared_ptr<IfcBSplineCurve> bspline_curve = dynamic_pointer_cast<IfcBSplineCurve>( bounded_curve );
        if( bspline_curve ) {
            ConvertBSplineCurve( bspline_curve, target_vec, segment_start_points );

            // TODO: handle trim points

            return;
        }

        shared_ptr<IfcIndexedPolyCurve> indexed_poly_curve = dynamic_pointer_cast<IfcIndexedPolyCurve>( bounded_curve );
        if( indexed_poly_curve ) {
            shared_ptr<IfcCartesianPointList>& pointList = indexed_poly_curve->m_Points;
            if( !pointList ) {
                return;
            }

            // IfcIndexedPolyCurve -----------------------------------------------------------
            std::vector<csgjscpp::Vector> pointVec;

            shared_ptr<IfcCartesianPointList2D> pointList2D = dynamic_pointer_cast<IfcCartesianPointList2D>( pointList );
            if( pointList2D ) {
                pointVec = ConvertPoints( pointList2D->m_CoordList );
            } else {
                shared_ptr<IfcCartesianPointList3D> pointList3D = dynamic_pointer_cast<IfcCartesianPointList3D>( pointList );
                if( pointList3D ) {
                    pointVec = ConvertPoints( pointList3D->m_CoordList );
                }
            }

            const std::vector<shared_ptr<IfcSegmentIndexSelect>>& segments = indexed_poly_curve->m_Segments; // optional
            if( !segments.empty() ) {
                for( const auto& segment: segments ) {
                    shared_ptr<IfcLineIndex> lineIdx = dynamic_pointer_cast<IfcLineIndex>( segment );
                    if( lineIdx ) {
                        if( lineIdx->m_vec.size() > 1 ) {
                            if( lineIdx->m_vec[ 0 ] && lineIdx->m_vec[ 1 ] ) {
                                int idx0 = lineIdx->m_vec[ 0 ]->m_value - 1;
                                int idx1 = lineIdx->m_vec[ 1 ]->m_value - 1;
                                if( idx0 < pointVec.size() && idx1 < pointVec.size() ) {
                                    const auto& pt0 = pointVec[ idx0 ];
                                    const auto& pt1 = pointVec[ idx1 ];

                                    target_vec.push_back( pt0 );
                                    target_vec.push_back( pt1 );
                                    segment_start_points.push_back( pt0 );
                                }
                            }
                        }
                        continue;
                    }

                    shared_ptr<IfcArcIndex> arcIdx = dynamic_pointer_cast<IfcArcIndex>( segment );
                    if( arcIdx ) {
                        if( arcIdx->m_vec.size() < 3 ) {
                            continue;
                        }

                        if( arcIdx->m_vec[ 0 ] && arcIdx->m_vec[ 1 ] && arcIdx->m_vec[ 2 ] ) {
                            int idx0 = arcIdx->m_vec[ 0 ]->m_value - 1;
                            int idx1 = arcIdx->m_vec[ 1 ]->m_value - 1;
                            int idx2 = arcIdx->m_vec[ 2 ]->m_value - 1;
                            if( idx0 >= 0 && idx1 >= 0 && idx2 >= 0 ) {
                                if( idx0 < pointVec.size() && idx1 < pointVec.size() && idx2 < pointVec.size() ) {
                                    const auto& p0 = pointVec[ idx0 ];
                                    const auto& p1 = pointVec[ idx1 ];
                                    const auto& p2 = pointVec[ idx2 ];

                                    const auto t = p1 - p0;
                                    const auto u = p2 - p0;
                                    const auto v = p2 - p1;

                                    const auto w = csgjscpp::cross( t, u );
                                    const float wsl = csgjscpp::lengthsquared( w );
                                    if( wsl > 10e-14 ) {
                                        const float iwsl2 = 1.0f / ( 2.0f * wsl );
                                        const float tt = csgjscpp::dot( t, t );
                                        const float uu = csgjscpp::dot( u, u );

                                        auto circ_center = p0 + ( u * tt * ( csgjscpp::dot( u, v ) ) - t * uu * ( csgjscpp::dot( t, v ) ) ) * iwsl2;
                                        auto circAxis = w / sqrt( wsl );
                                        auto center_p0 = p0 - circ_center;
                                        auto center_p1 = p1 - circ_center;
                                        auto center_p2 = p2 - circ_center;
                                        auto center_p0_normalized = csgjscpp::unit( center_p0 );
                                        auto center_p2_normalized = csgjscpp::unit( center_p2 );

                                        const float openingAngle = std::acos( csgjscpp::dot( center_p0_normalized, center_p2_normalized ) );
                                        size_t n = numVerticesPerCircle * openingAngle / ( M_PI * 2.0f );
                                        if( n < minNumVerticesPerArc ) {
                                            n = minNumVerticesPerArc;
                                        }

                                        const float deltaAngle = openingAngle / (float)( n - 1 );
                                        float angle = 0;
                                        std::vector<csgjscpp::Vector> circle_points_3d;
                                        for( size_t kk = 0; kk < n; ++kk ) {
                                            auto m = Matrix::GetRotation( -angle, circAxis );

                                            auto p_rotated = center_p0;
                                            p_rotated = m.GetTransformed( p_rotated ) + circ_center;

                                            circle_points_3d.push_back( p_rotated );
                                            angle += deltaAngle;
                                        }
                                        AppendPointsToCurve( circle_points_3d, target_vec );
                                        segment_start_points.push_back( circle_points_3d[ 0 ] );
                                    }
                                }
                            }
                        }
                    }
                }
            } else {
                // no segments, take all points from CoordList
                if( !pointVec.empty() ) {
                    const auto& pt0 = pointVec[ 0 ];
                    AppendPointsToCurve( pointVec, target_vec );
                    segment_start_points.push_back( pt0 );
                }
            }

            return;
        }
        // throw UnhandledRepresentationException( bounded_curve );
        //  TODO: Log error
        return;
    }

    shared_ptr<IfcConic> conic = dynamic_pointer_cast<IfcConic>( ifc_curve );
    if( conic ) {
        // ENTITY IfcConic ABSTRACT SUPERTYPE OF(ONEOF(IfcCircle, IfcEllipse))
        auto conic_position_matrix = Matrix::GetIdentity();
        shared_ptr<IfcPlacement> conic_placement = dynamic_pointer_cast<IfcPlacement>( conic->m_Position );
        if( conic_placement ) {
            conic_position_matrix = ConvertPlacement( conic_placement );
        }

        auto circle_center = conic_position_matrix.GetTransformed( { 0.0f, 0.0f, 0.0f } );

        float circle_radius = -1;
        float circle_radius2 = -1;
        const auto ellipse = dynamic_pointer_cast<IfcEllipse>( conic );
        if( ellipse ) {
            if( ellipse->m_SemiAxis1 ) {
                if( ellipse->m_SemiAxis2 ) {
                    circle_radius = (float)ellipse->m_SemiAxis1->m_value;
                    circle_radius2 = (float)ellipse->m_SemiAxis2->m_value;
                }
            }
        }

        shared_ptr<IfcCircle> circle = dynamic_pointer_cast<IfcCircle>( conic );
        if( circle ) {
            circle_radius = 0.0;
            if( circle->m_Radius ) {
                circle_radius = (float)circle->m_Radius->m_value;
            }
        }

        auto circlePositionInverse = conic_position_matrix;
        circlePositionInverse.Invert();
        float maxRadius = std::max( circle_radius, circle_radius2 );
        float trimAngle1 = 0.0;
        float trimAngle2 = (float)( M_PI * 2.0f );
        float startAngle = 0;
        float openingAngle = (float)( M_PI * 2.0f );
        // bool senseAgreement = true;
        getTrimAngles( trim1_vec, trim2_vec, circle_center, maxRadius, senseAgreement, trimAngle1, trimAngle2, startAngle, openingAngle, conic_position_matrix,
                       circlePositionInverse, planeAngleFactor );

        csgjscpp::Vector trimPoint1;
        csgjscpp::Vector trimPoint2;
        getTrimPoints( trim1_vec, trim2_vec, conic_position_matrix, circle_radius, circle_radius2, senseAgreement, trimPoint1, trimPoint2, planeAngleFactor );

        // FIXME: num_segments should be calculated from radius
        // int num_segments = m_geom_settings->getNumVerticesPerCircleWithRadius( circle_radius ) * ( std::fabsf( openingAngle ) / ( 2.0f * M_PI ) );
        int num_segments = ( (int)( numVerticesPerCircle * 1.5f ) ) * ( std::fabsf( openingAngle ) / ( 2.0f * M_PI ) );
        if( num_segments < minNumVerticesPerArc )
            num_segments = minNumVerticesPerArc;
        const float circle_center_x = 0.0;
        const float circle_center_y = 0.0;
        std::vector<csgjscpp::Vector> circle_segment_points3D;

        if( circle_radius > 0 ) {
            float angle = startAngle;
            float angle_delta = openingAngle / (float)( num_segments - 1 );

            for( int i = 0; i < num_segments; ++i ) {
                csgjscpp::Vector circlePoint( circle_radius * cosf( angle ), circle_radius * sinf( angle ), 0.0f );
                if( circle_radius2 > 0 ) {
                    circlePoint = csgjscpp::Vector( circle_radius * cosf( angle ), circle_radius2 * sinf( angle ), 0.0f );
                }

                // apply position
                conic_position_matrix.Transform( &circlePoint );

                circle_segment_points3D.push_back( circlePoint );
                angle += angle_delta;
            }
        } else {
            circle_segment_points3D.emplace_back( circle_center_x, circle_center_y, 0.0f );
        }

        AppendPointsToCurve( circle_segment_points3D, target_vec );
        segment_start_points.push_back( circle_segment_points3D[ 0 ] );

        return;
    }

    const auto line = dynamic_pointer_cast<IfcLine>( ifc_curve );
    if( line ) {
        shared_ptr<IfcCartesianPoint> ifc_line_point = line->m_Pnt;
        csgjscpp::Vector line_origin = ConvertCartesianPoint( ifc_line_point );

        // line: lambda(u) = line_point + u*line_direction
        shared_ptr<IfcVector> line_vec = line->m_Dir;
        if( !line_vec ) {
            return;
        }
        shared_ptr<IfcDirection> ifc_line_direction = line_vec->m_Orientation;

        std::vector<shared_ptr<IfcReal>>& direction_ratios = ifc_line_direction->m_DirectionRatios;
        csgjscpp::Vector line_direction;
        if( direction_ratios.size() > 1 ) {
            if( direction_ratios.size() > 2 ) {
                line_direction = { (float)direction_ratios[ 0 ]->m_value, (float)direction_ratios[ 1 ]->m_value, (float)direction_ratios[ 2 ]->m_value };
            } else {
                line_direction = { (float)direction_ratios[ 0 ]->m_value, (float)direction_ratios[ 1 ]->m_value, 0.0f };
            }
        }
        line_direction = csgjscpp::unit( line_direction );

        // line_vec->m_Magnitude;  can be ignored here, since it is a direction


        // check for trimming at beginning of line
        float start_parameter = 0.0f;
        shared_ptr<IfcParameterValue> trim_par1;
        if( FindFirstInVector( trim1_vec, trim_par1 ) ) {
            start_parameter = (float)trim_par1->m_value;
            line_origin = line_origin + line_direction * start_parameter;
        } else {
            shared_ptr<IfcCartesianPoint> ifc_trim_point;
            if( FindFirstInVector( trim1_vec, ifc_trim_point ) ) {
                csgjscpp::Vector trim_point;
                trim_point = ConvertCartesianPoint( ifc_trim_point );
                line_origin = trim_point;

                auto closest_point_on_line = ClosestPointOnLine( trim_point, line_origin, line_direction );

                if( closest_point_on_line == trim_point ) {
                    // trimming point is on the line
                    line_origin = trim_point;
                } else {
                    line_origin = closest_point_on_line;
                }
            }
        }
        // check for trimming at end of line
        csgjscpp::Vector line_end;
        shared_ptr<IfcParameterValue> trim_par2;
        if( FindFirstInVector( trim2_vec, trim_par2 ) ) {
            line_end = line_origin + line_direction * (float)trim_par2->m_value;
        } else {
            shared_ptr<IfcCartesianPoint> ifc_trim_point;
            if( FindFirstInVector( trim2_vec, ifc_trim_point ) ) {
                csgjscpp::Vector trim_point;
                trim_point = ConvertCartesianPoint( ifc_trim_point );
                line_end = trim_point;

                auto closest_point_on_line = ClosestPointOnLine( trim_point, line_origin, line_direction );

                if( closest_point_on_line == trim_point ) {
                    // trimming point is on the line
                    line_end = trim_point;
                } else {
                    line_end = closest_point_on_line;
                }
            }
        }

        std::vector<csgjscpp::Vector> points_vec;
        points_vec.push_back( line_origin );
        points_vec.push_back( line_end );

        AppendPointsToCurve( points_vec, target_vec );
        segment_start_points.push_back( line_origin );

        return;
    }


    const auto offset_curve_2d = dynamic_pointer_cast<IfcOffsetCurve2D>( ifc_curve );
    if( offset_curve_2d ) {
        // TODO: implement
        return;
    }

    const auto offset_curve_3d = dynamic_pointer_cast<IfcOffsetCurve3D>( ifc_curve );
    if( offset_curve_3d ) {
        // TODO: implement
        return;
    }

    const auto pcurve = dynamic_pointer_cast<IfcPcurve>( ifc_curve );
    if( pcurve ) {
        // TODO: implement
        return;
    }

    // TODO: Log error
    // throw UnhandledRepresentationException( ifc_curve );
}

void getTrimAngles( const std::vector<shared_ptr<IfcTrimmingSelect>>& trim1_vec, const std::vector<shared_ptr<IfcTrimmingSelect>>& trim2_vec,
                    csgjscpp::Vector circle_center, float circle_radius, bool senseAgreement, float& trimAngle1, float& trimAngle2, float& startAngle,
                    float& openingAngle, const Matrix& circlePlacement, const Matrix& circlePlacementInverse, float planeAngleFactor ) {
    trimAngle1 = 0.0;
    trimAngle2 = M_PI * 2.0;
    startAngle = 0;
    openingAngle = 0;

    // check for trimming begin
    shared_ptr<IfcParameterValue> trim_par1;
    if( FindFirstInVector( trim1_vec, trim_par1 ) ) {
        trimAngle1 = (float)trim_par1->m_value * planeAngleFactor;
    } else {
        shared_ptr<IfcCartesianPoint> trim_point1;
        if( FindFirstInVector( trim1_vec, trim_point1 ) ) {
            csgjscpp::Vector trim_point;
            trim_point = ConvertCartesianPoint( trim_point1 );
            // TODO: get direction of trim_point to circle_center, get angle. This is more robust in case the trim_point is not exactly on the circle
            trimAngle1 = GetAngleOnCircle( circle_center, circle_radius, trim_point, circlePlacement, circlePlacementInverse );
        }
    }

    // check for trimming end
    shared_ptr<IfcParameterValue> trim_par2;
    if( FindFirstInVector( trim2_vec, trim_par2 ) ) {
        trimAngle2 = (float)trim_par2->m_value * planeAngleFactor;
    } else {
        shared_ptr<IfcCartesianPoint> ifc_trim_point;
        if( FindFirstInVector( trim2_vec, ifc_trim_point ) ) {
            csgjscpp::Vector trim_point;
            trim_point = ConvertCartesianPoint( ifc_trim_point );
            trimAngle2 = GetAngleOnCircle( circle_center, circle_radius, trim_point, circlePlacement, circlePlacementInverse );
        }
    }

    startAngle = trimAngle1;
    openingAngle = 0;

    if( senseAgreement ) {
        if( trimAngle1 < trimAngle2 ) {
            openingAngle = trimAngle2 - trimAngle1;
        } else {
            // circle passes 0 angle
            openingAngle = (float)( trimAngle2 - trimAngle1 + 2.0f * M_PI );
        }
    } else {
        if( trimAngle1 > trimAngle2 ) {
            openingAngle = trimAngle2 - trimAngle1;
        } else {
            // circle passes 0 angle
            openingAngle = (float)( trimAngle2 - trimAngle1 - 2.0 * M_PI );
        }
    }

    while( openingAngle > 2.0 * M_PI ) {
        openingAngle -= 2.0 * M_PI;
    }
    while( openingAngle < -2.0 * M_PI ) {
        openingAngle += 2.0 * M_PI;
    }
}

void getTrimPoints( const std::vector<std::shared_ptr<IfcTrimmingSelect>>& trim1_vec, const std::vector<shared_ptr<IfcTrimmingSelect>>& trim2_vec,
                    const Matrix& circlePosition, float circleRadius, float circleRadius2, bool senseAgreement, csgjscpp::Vector& trimPoint1,
                    csgjscpp::Vector& trimPoint2, float planeAngleFactor ) {
    // check for trimming begin
    std::shared_ptr<IfcCartesianPoint> ifc_trim_point1;
    if( FindFirstInVector( trim1_vec, ifc_trim_point1 ) ) {
        trimPoint1 = ConvertCartesianPoint( ifc_trim_point1 );
    } else {
        shared_ptr<IfcParameterValue> trim_par1;
        if( FindFirstInVector( trim1_vec, trim_par1 ) ) {
            float trimAngle1 = (float)trim_par1->m_value * planeAngleFactor;
            trimPoint1 = circlePosition.GetTransformed( csgjscpp::Vector( circleRadius * cosf( trimAngle1 ), circleRadius * sinf( trimAngle1 ), 0 ) );
            if( circleRadius2 > 0 ) {
                trimPoint1 = csgjscpp::Vector( circleRadius * cosf( trimAngle1 ), circleRadius2 * sinf( trimAngle1 ), 0 );
            }
        }
    }

    // check for trimming end
    shared_ptr<IfcCartesianPoint> ifc_trim_point2;
    if( FindFirstInVector( trim2_vec, ifc_trim_point2 ) ) {
        trimPoint2 = ConvertCartesianPoint( ifc_trim_point2 );
    } else {
        shared_ptr<IfcParameterValue> trim_par;
        if( FindFirstInVector( trim2_vec, trim_par ) ) {
            float trimAngle = (float)trim_par->m_value * planeAngleFactor;
            trimPoint2 = circlePosition.GetTransformed( csgjscpp::Vector( circleRadius * cosf( trimAngle ), circleRadius * sinf( trimAngle ), 0 ) );
            if( circleRadius2 > 0 ) {
                // FIXME. Was:
                // trimPoint1 = csgjscpp::Vector( circleRadius * cosf( trimAngle ), circleRadius2 * sinf( trimAngle ), 0 );
                trimPoint2 = csgjscpp::Vector( circleRadius * cosf( trimAngle ), circleRadius2 * sinf( trimAngle ), 0 );
            }
        }
    }
}

float GetAngleOnCircle( const csgjscpp::Vector& circleCenter, float radius, csgjscpp::Vector& trimPoint, const Matrix& circlePosition,
                        const Matrix& circlePositionInverse ) {
    float resultAngle = -1.0;
    auto center2trimPoint = trimPoint - circleCenter;
    auto center2trimPointDirection = csgjscpp::unit( center2trimPoint );

    bool checkDistanceToCircleCenter = false;
    if( checkDistanceToCircleCenter ) {
        if( std::fabsf( csgjscpp::lengthsquared( center2trimPoint ) - radius * radius ) > csgjscpp::csgjs_EPSILON ) {
            trimPoint = circleCenter + center2trimPointDirection * radius;
        }
    }

    auto circleCenter2D = circlePositionInverse.GetTransformed( circleCenter );
    auto trimPoint2D = circlePositionInverse.GetTransformed( trimPoint );
    auto trimPointRelative2D = trimPoint2D - circleCenter2D;
    if( std::fabsf( trimPointRelative2D.z ) < csgjscpp::csgjs_EPSILON ) {
        resultAngle = atan2( trimPointRelative2D.y - circleCenter2D.y, trimPointRelative2D.x - circleCenter2D.x );

        auto circleCenter3DCheck = circlePosition.GetTransformed( { 0, 0, 0 } );
        csgjscpp::Vector circlePoint( radius * cos( resultAngle ), radius * sin( resultAngle ), 0 );
        circlePoint = circlePosition.GetTransformed( circlePoint );

        auto check = circlePoint - trimPoint;
        float dist = csgjscpp::length( check );
        if( dist < csgjscpp::csgjs_EPSILON ) {
            return resultAngle;
        }
    }

    {
        // try parabola
        float angle = resultAngle;
        float angleStep = 0.1;
        float x[ 4 ], y[ 4 ];
        x[ 1 ] = resultAngle - angleStep;
        x[ 2 ] = resultAngle;
        x[ 3 ] = resultAngle + angleStep;

        for( size_t ii = 0; ii < 40; ++ii ) {
            y[ 1 ] = trimPointCircleDistance( x[ 1 ], radius, circlePosition, trimPoint );
            y[ 2 ] = trimPointCircleDistance( x[ 2 ], radius, circlePosition, trimPoint );
            y[ 3 ] = trimPointCircleDistance( x[ 3 ], radius, circlePosition, trimPoint );

            // zero of parabola
            x[ 0 ] = ( x[ 2 ] * x[ 2 ] * ( y[ 3 ] - y[ 1 ] ) - x[ 1 ] * ( y[ 3 ] - y[ 2 ] ) - x[ 3 ] * ( y[ 2 ] - y[ 1 ] ) ) /
                ( 2 * ( x[ 2 ] * ( y[ 3 ] - y[ 1 ] ) - x[ 1 ] * ( y[ 3 ] - y[ 2 ] ) - x[ 3 ] * ( y[ 2 ] - y[ 1 ] ) ) );
            y[ 0 ] = trimPointCircleDistance( x[ 0 ], radius, circlePosition, trimPoint );
            bool improvementFound = false;

            for( size_t jj = 1; jj <= 3; ++jj ) {
                if( y[ jj ] < csgjscpp::csgjs_EPSILON ) {
                    resultAngle = x[ jj ];
                    return resultAngle;
                }

                if( y[ jj ] < y[ 0 ] ) {
                    // improvement found
                    angleStep *= 0.7;
                    x[ 1 ] = x[ jj ] - angleStep;
                    x[ 2 ] = x[ jj ];
                    x[ 3 ] = x[ jj ] + angleStep;
                    resultAngle = x[ jj ];
                    x[ 0 ] = x[ jj ];
                    y[ 0 ] = y[ jj ];
                    improvementFound = true;
                }
            }

            if( !improvementFound ) {
                float bestX = x[ 2 ];
                float bestY = y[ 2 ];

                if( y[ 1 ] < bestY ) {
                    bestX = x[ 1 ];
                    bestY = y[ 1 ];
                }
                if( y[ 3 ] < bestY ) {
                    bestX = x[ 3 ];
                    bestY = y[ 3 ];
                }

                if( angleStep < 3 ) {
                    angleStep += 0.03;
                }
                x[ 1 ] = bestX - angleStep;
                x[ 2 ] = bestX;
                x[ 3 ] = bestX + angleStep;
            }
        }
    }


    float angle = resultAngle;
    size_t smallestDistanceAngle = resultAngle;
    float angleStep = 0.1; // deltaAngle * 0.2;
    float x0 = smallestDistanceAngle;
    float x1 = smallestDistanceAngle + angleStep;
    float x2 = x1;
    float x3 = x1;
    float f0 = trimPointCircleDistance( x0, radius, circlePosition, trimPoint );
    float f1 = trimPointCircleDistance( x1, radius, circlePosition, trimPoint );
    float f2 = f1;
    x2 = regula( x0, x1, f0, f1 );

    for( size_t ii = 0; ii < 40; ++ii ) {
        f0 = trimPointCircleDistance( x0, radius, circlePosition, trimPoint );
        f2 = trimPointCircleDistance( x2, radius, circlePosition, trimPoint );
        if( f0 < f2 ) {
            // no improvement found
            x1 = x2;
        } else {
            // improvement found
            x0 = x2;
            resultAngle = x2;
        }

        if( f0 < csgjscpp::csgjs_EPSILON ) {
            resultAngle = x0;
            break;
        }

        if( f2 < csgjscpp::csgjs_EPSILON ) {
            resultAngle = x2;
            break;
        }

        f0 = trimPointCircleDistance( x0, radius, circlePosition, trimPoint );
        f1 = trimPointCircleDistance( x1, radius, circlePosition, trimPoint );
        x3 = regula( x0, x1, f0, f1 );
        if( fabs( x3 - x2 ) < csgjscpp::csgjs_EPSILON ) {
            break;
        }

        if( f0 < csgjscpp::csgjs_EPSILON ) {
            resultAngle = x0;
            break;
        }
        if( f1 < csgjscpp::csgjs_EPSILON ) {
            resultAngle = x1;
            break;
        }
        x2 = x3;
    }

    if( resultAngle < 0 ) {
        resultAngle += 2.0 * M_PI;
    }

    if( resultAngle > 2.0 * M_PI ) {
        resultAngle -= 2.0 * M_PI;
    }
    return resultAngle;
}

float trimPointCircleDistance( float angle, float radius, const Matrix& circlePosition, const csgjscpp::Vector& trimPoint ) {
    csgjscpp::Vector circlePoint( radius * cosf( angle ), radius * sinf( angle ), 0.0f );
    circlePosition.Transform( &circlePoint );
    float distance2 = csgjscpp::lengthsquared( trimPoint - circlePoint );
    return distance2;
}

float regula( float x0, float x1, float fx0, float fx1 ) {
    return x0 - ( ( x1 - x0 ) / ( fx1 - fx0 ) ) * fx0;
}

void ConvertBSplineCurve( const shared_ptr<IfcBSplineCurve>& bspline_curve, std::vector<csgjscpp::Vector>& target_vec,
                          std::vector<csgjscpp::Vector>& segment_start_points ) {
    if( !bspline_curve ) {
        return;
    }
    if( !bspline_curve->m_Degree ) {
        return;
    }
    const int degree = bspline_curve->m_Degree->m_value;
    const std::vector<shared_ptr<IfcCartesianPoint>>& control_points_ifc = bspline_curve->m_ControlPointsList;
    // const shared_ptr<IfcBSplineCurveForm>&				curve_form = bspline_curve->m_CurveForm;
    // const LogicalEnum									closed_curve = bspline_curve->m_ClosedCurve;
    // const LogicalEnum									self_intersect = bspline_curve->m_ClosedCurve;

    std::vector<csgjscpp::Vector> controlPoints;
    controlPoints = ConvertCartesianPoints( control_points_ifc );

    if( controlPoints.size() < 2 ) {
        return;
    }

    std::vector<float> weights;
    std::vector<float> curvePointsCoords;

    const size_t numControlPoints = controlPoints.size();
    const size_t order = degree + 1; // the order of the curve is the degree of the resulting polynomial + 1
    const size_t numCurvePoints = numControlPoints * numVerticesPerControlPoint;
    std::vector<float> knotVec;

    //	set weighting factors to 1.0 in case of homogeneous curve
    weights.resize( numControlPoints + 1, 1.0 );

    const auto bspline_curve_with_knots = dynamic_pointer_cast<IfcBSplineCurveWithKnots>( bspline_curve );
    if( bspline_curve_with_knots ) {
        std::vector<shared_ptr<IfcInteger>>& ifc_knot_mult = bspline_curve_with_knots->m_KnotMultiplicities;
        std::vector<shared_ptr<IfcParameterValue>>& ifc_knots = bspline_curve_with_knots->m_Knots;
        // shared_ptr<IfcKnotType>&						ifc_knot_spec = bspline_curve_with_knots->m_KnotSpec;

        for( size_t ii = 0; ii < ifc_knots.size(); ++ii ) {
            shared_ptr<IfcParameterValue>& knot_parameter = ifc_knots[ ii ];
            double knot_value = knot_parameter->m_value;

            int num_multiply_knot_value = 1;
            if( ifc_knot_mult.size() == ifc_knots.size() ) {
                num_multiply_knot_value = ifc_knot_mult[ ii ]->m_value;
            }

            for( int jj = 0; jj < num_multiply_knot_value; ++jj ) {
                knotVec.push_back( knot_value );
            }
        }

        const auto r_bspline_curve_with_knots = dynamic_pointer_cast<IfcRationalBSplineCurveWithKnots>( bspline_curve_with_knots );
        if( r_bspline_curve_with_knots ) {
            std::vector<shared_ptr<IfcReal>>& ifc_vec_weigths = r_bspline_curve_with_knots->m_WeightsData;
            weights.resize( ifc_vec_weigths.size() );
            for( size_t i_weight = 0; i_weight < ifc_vec_weigths.size(); ++i_weight ) {
                weights[ i_weight ] = ifc_vec_weigths[ i_weight ]->m_value;
            }
        }
    }

    curvePointsCoords.resize( 3 * numCurvePoints, 0.0 );
    ComputeRationalBSpline( order, numCurvePoints, controlPoints, weights, knotVec, curvePointsCoords );

    if( target_vec.size() > 2 ) {
        segment_start_points.emplace_back( curvePointsCoords[ 0 ], curvePointsCoords[ 1 ], curvePointsCoords[ 2 ] );
    }

    for( size_t ii = 0; ii < 3 * numCurvePoints; ii = ii + 3 ) {
        target_vec.emplace_back( curvePointsCoords[ ii ], curvePointsCoords[ ii + 1 ], curvePointsCoords[ ii + 2 ] );
    }
}

void ComputeRationalBSpline( const size_t order, const size_t numCurvePoints, const std::vector<csgjscpp::Vector>& controlPoints, std::vector<float>& weights,
                             std::vector<float>& knotVec, std::vector<float>& curvePoints ) {
    // order: order of the BSpline basis function
    std::vector<float> basis_func; // basis function for parameter value t
    basis_func.resize( controlPoints.size() + 1, 0.0 );

    const size_t n_plus_order = controlPoints.size() + order; // number of knot values
    if( knotVec.size() != n_plus_order ) {
        // generate a uniform open knot vector
        knotVec.resize( n_plus_order, 0.0 );
        ComputeKnotVector( controlPoints.size(), order, knotVec );
    }

    double t = 0; // parameter value 0 <= t <= npts - k + 1
    double step = knotVec[ knotVec.size() - 1 ] / ( (double)( numCurvePoints - 1 ) );

    std::vector<double> control_points_coords;
    for( auto controlPoint: controlPoints ) {
        control_points_coords.push_back( controlPoint.x );
        control_points_coords.push_back( controlPoint.y );
        control_points_coords.push_back( controlPoint.z );
    }

    size_t offset_i = 0;
    double temp;
    for( size_t ii = 0; ii < numCurvePoints; ++ii ) {
        if( t > knotVec[ knotVec.size() - 1 ] - 0.000001 ) {
            t = knotVec[ knotVec.size() - 1 ];
        }

        // generate the basis function for this value of t
        ComputRationalBasisFunctions( order, t, controlPoints.size(), knotVec, weights, basis_func );

        for( size_t jj = 0; jj < 3; ++jj ) {
            curvePoints[ offset_i + jj ] = 0.;

            for( size_t kk = 0; kk < controlPoints.size(); ++kk ) {
                // matrix multiplication
                temp = basis_func[ kk ] * control_points_coords[ jj + kk * 3 ];
                curvePoints[ offset_i + jj ] = curvePoints[ offset_i + jj ] + temp;
            }
        }

        offset_i = offset_i + 3;
        t = t + step;
    }
}

void ComputeKnotVector( const size_t numControlPoints, const size_t order, std::vector<float>& knotVector ) {
    const size_t n_plus_order = numControlPoints + order;
    const size_t n_plus_1 = numControlPoints + 1;

    // example: order=2, numControlPoints=4,  n_plus_1=5,  n_plus_c=6
    //                       order                            nplus1    nplusc
    //  knot[0]   knot[1]    knot[2]    knot[3]    knot[4]    knot[5]   knot[6]
    //  0         0          1          2          3          4         4

    knotVector[ 0 ] = 0; // start the knot vector with 0
    for( size_t ii = 1; ii < n_plus_order; ++ii ) {
        if( ( ii >= order ) && ( ii < n_plus_1 ) ) {
            knotVector[ ii ] = knotVector[ ii - 1 ] + 1.0;
        } else {
            knotVector[ ii ] = knotVector[ ii - 1 ];
        }
    }
}

static void ComputRationalBasisFunctions( const size_t order, const float t, const size_t numControlPoints, const std::vector<float>& knotVec,
                                          std::vector<float>& weights, std::vector<float>& basisFunc ) {
    const size_t n_plus_order = numControlPoints + order; // maximum number of knot values

    // first order nonrational basis function
    std::vector<float> temp;
    temp.resize( n_plus_order + 1 );
    for( size_t ii = 0; ii < n_plus_order - 1; ii++ ) {
        if( ( t >= knotVec[ ii ] ) && ( t < knotVec[ ii + 1 ] ) ) {
            temp[ ii ] = 1;
        } else {
            temp[ ii ] = 0;
        }
    }

    // higher order nonrational basis function
    float basis_func_part1; // first term of the basis function recursion relation
    float basis_func_part2; // second term of the basis function recursion relation
    for( size_t kk = 2; kk <= order; ++kk ) {
        for( size_t ii = 0; ii <= n_plus_order - kk + 1; ++ii ) {
            // skip if the lower order basis function is zero
            if( temp[ ii ] != 0 ) {
                basis_func_part1 = ( ( t - knotVec[ ii ] ) * temp[ ii ] ) / ( knotVec[ ii + kk - 1 ] - knotVec[ ii ] );
            } else {
                basis_func_part1 = 0;
            }

            // skip if the lower order basis function is zero
            if( temp[ ii + 1 ] != 0 ) {
                basis_func_part2 = ( ( knotVec[ ii + kk ] - t ) * temp[ ii + 1 ] ) / ( knotVec[ ii + kk ] - knotVec[ ii + 1 ] );
            } else {
                basis_func_part2 = 0;
            }

            temp[ ii ] = basis_func_part1 + basis_func_part2;
        }
    }

    if( t == knotVec[ n_plus_order - 1 ] ) {
        // pick up last point
        temp[ numControlPoints - 1 ] = 1;
    }

    if( weights.size() < numControlPoints + 1 ) {
        float resizeValue = 1.0;
        weights.resize( numControlPoints + 1, resizeValue );
    }

    // compute sum for denominator of rational basis function
    float sum = 0.0;
    for( size_t ii = 0; ii < numControlPoints; ++ii ) {
        sum = sum + temp[ ii ] * weights[ ii + 1 ];
    }

    // compute the rational basis function
    for( size_t ii = 0; ii < numControlPoints; ii++ ) {
        if( sum != 0.0 ) {
            basisFunc[ ii ] = temp[ ii ] * weights[ ii + 1 ] / sum;
        } else {
            basisFunc[ ii ] = 0;
        }
    }
}

}
