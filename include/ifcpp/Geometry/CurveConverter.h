#pragma once

#include "ifcpp/Geometry/CAdapter.h"
#include "ifcpp/Geometry/GeomUtils.h"
#include "ifcpp/Geometry/Matrix.h"
#include "ifcpp/Geometry/Parameters.h"
#include "ifcpp/Geometry/PrimitivesConverter.h"
#include "ifcpp/Geometry/Vector.h"

#include "ifcpp/Ifc/IfcArcIndex.h"
#include "ifcpp/Ifc/IfcBSplineCurve.h"
#include "ifcpp/Ifc/IfcBoolean.h"
#include "ifcpp/Ifc/IfcBoundedCurve.h"
#include "ifcpp/Ifc/IfcCartesianPointList2D.h"
#include "ifcpp/Ifc/IfcCartesianPointList3D.h"
#include "ifcpp/Ifc/IfcCircle.h"
#include "ifcpp/Ifc/IfcCompositeCurve.h"
#include "ifcpp/Ifc/IfcCompositeCurveSegment.h"
#include "ifcpp/Ifc/IfcConic.h"
#include "ifcpp/Ifc/IfcCurve.h"
#include "ifcpp/Ifc/IfcCurveSegment.h"
#include "ifcpp/Ifc/IfcEllipse.h"
#include "ifcpp/Ifc/IfcIndexedPolyCurve.h"
#include "ifcpp/Ifc/IfcLine.h"
#include "ifcpp/Ifc/IfcLineIndex.h"
#include "ifcpp/Ifc/IfcOffsetCurve2D.h"
#include "ifcpp/Ifc/IfcOffsetCurve3D.h"
#include "ifcpp/Ifc/IfcParameterValue.h"
#include "ifcpp/Ifc/IfcPcurve.h"
#include "ifcpp/Ifc/IfcPolyline.h"
#include "ifcpp/Ifc/IfcPositiveLengthMeasure.h"
#include "ifcpp/Ifc/IfcSegment.h"
#include "ifcpp/Ifc/IfcTrimmedCurve.h"
#include "ifcpp/Ifc/IfcTrimmingSelect.h"
#include "ifcpp/Ifc/IfcVector.h"


namespace ifcpp {

template<CVector TVector>
class CurveConverter {
    using TCurve = std::vector<TVector>;
    using AVector = VectorAdapter<TVector>;
    using TMatrix = Matrix<TVector>;

    std::shared_ptr<PrimitivesConverter<TVector>> m_primitivesConverter;
    std::shared_ptr<GeomUtils<TVector>> m_geomUtils;
    std::shared_ptr<Parameters> m_parameters;

public:
    CurveConverter( const std::shared_ptr<PrimitivesConverter<TVector>>& primitivesConverter, const std::shared_ptr<GeomUtils<TVector>>& geomUtils,
                    const std::shared_ptr<Parameters>& parameters )
        : m_primitivesConverter( primitivesConverter )
        , m_geomUtils( geomUtils )
        , m_parameters( parameters ) {
    }

    TCurve ConvertCurve( const std::shared_ptr<IfcCurve>& curve ) {
        // ENTITY IfcCurve ABSTRACT SUPERTYPE OF (ONEOF(IfcBoundedCurve, IfcConic, IfcLine, IfcOffsetCurve2D, IfcOffsetCurve3D, IfcPCurve))

        const auto boundedCurve = dynamic_pointer_cast<IfcBoundedCurve>( curve );
        if( boundedCurve ) {
            // ENTITY IfcBoundedCurve ABSTRACT SUPERTYPE OF (ONEOF(IfcCompositeCurve, IfcPolyline, IfcTrimmedCurve, IfcBSplineCurve))

            const auto compositeCurve = dynamic_pointer_cast<IfcCompositeCurve>( boundedCurve );
            if( compositeCurve ) {
                TCurve result;
                for( const auto& segment: compositeCurve->m_Segments ) {
                    TCurve segmentPoints;
                    const auto compositeCurveSegment = dynamic_pointer_cast<IfcCompositeCurveSegment>( segment );
                    if( compositeCurveSegment ) {
                        segmentPoints = this->ConvertCurve( compositeCurveSegment->m_ParentCurve );
                        // FIXME: concatenation
                        if( compositeCurveSegment->m_SameSense && !compositeCurveSegment->m_SameSense->m_value ) {
                            std::back_inserter( std::begin( segmentPoints ), std::end( segmentPoints ), result );
                        } else {
                            std::back_inserter( std::rbegin( segmentPoints ), std::rend( segmentPoints ), result );
                        }
                    } else {
                        // TODO: Log error
                    }
                }
                return result;
            }

            const auto polyLine = dynamic_pointer_cast<IfcPolyline>( curve );
            if( polyLine ) {
                return this->m_primitivesConverter->ConvertPoints( polyLine->m_Points );
            }

            const auto trimmedCurve = dynamic_pointer_cast<IfcTrimmedCurve>( boundedCurve );
            if( trimmedCurve ) {
                if( trimmedCurve->m_BasisCurve ) {
                    const auto conic = dynamic_pointer_cast<IfcConic>( curve );
                    if( conic ) {
                        // ENTITY IfcConic ABSTRACT SUPERTYPE OF(ONEOF(IfcCircle, IfcEllipse))
                        const auto placement = this->m_primitivesConverter->ConvertPlacement( conic->m_Position );
                        const auto circleCenter = placement.GetTransformed( AVector::New() );
                        float radius1 = 0.0f;
                        float radius2 = 0.0f;

                        const auto ellipse = dynamic_pointer_cast<IfcEllipse>( conic );
                        if( ellipse && ellipse->m_SemiAxis1 && ellipse->m_SemiAxis2 ) {
                            radius1 = (float)ellipse->m_SemiAxis1->m_value;
                            radius2 = (float)ellipse->m_SemiAxis2->m_value;
                        }
                        shared_ptr<IfcCircle> circle = dynamic_pointer_cast<IfcCircle>( conic );
                        if( circle && circle->m_Radius ) {
                            radius1 = radius2 = (float)circle->m_Radius->m_value;
                        }

                        TCurve result;
                        int n = this->m_parameters->m_numVerticesPerCircle; // TODO: should be calculated from radius
                        n = std::max( n, this->m_parameters->m_minNumVerticesPerArc );
                        auto [ startAngle, openingAngle ] =
                            this->GetTrimmingsForCircle( trimmedCurve->m_Trim1, trimmedCurve->m_Trim2, circleCenter, trimmedCurve->m_SenseAgreement );
                        if( radius1 > this->m_parameters->m_epsilon && radius2 > this->m_parameters->m_epsilon ) {
                            result = this->m_geomUtils->BuildEllipse( radius1, radius2, startAngle, openingAngle, n, circleCenter );
                        } else {
                            result.push_back( circleCenter );
                        }
                        return result;
                    }

                    const auto line = dynamic_pointer_cast<IfcLine>( curve );
                    if( line && line->m_Pnt && line->m_Dir ) {
                        const auto origin = this->m_primitivesConverter->ConvertPoint( line->m_Pnt );
                        const auto direction = AVector::Normalized( this->m_primitivesConverter->ConvertVector( line->m_Dir ) );
                        const auto[ l, r ] = this->GetTrimmingsForLine( trimmedCurve->m_Trim1, trimmedCurve->m_Trim2, origin, direction, trimmedCurve->m_SenseAgreement );
                        return {
                            origin + direction * l,
                            origin + direction * r,
                        };
                    }
                }
            }

            const auto bsplineCurve = dynamic_pointer_cast<IfcBSplineCurve>( boundedCurve );
            if( bsplineCurve ) {
                // TODO: Implement
                return this->m_primitivesConverter->ConvertPoints( bsplineCurve->m_ControlPointsList );
            }

            const auto indexedPolyCurve = dynamic_pointer_cast<IfcIndexedPolyCurve>( boundedCurve );
            if( indexedPolyCurve ) {
                std::vector<TVector> points;
                TCurve result;
                const auto pointList2d = dynamic_pointer_cast<IfcCartesianPointList2D>( indexedPolyCurve->m_Points );
                if( pointList2d ) {
                    points = this->m_primitivesConverter->ConvertPoints( pointList2d->m_CoordList );
                }
                const auto pointList3d = dynamic_pointer_cast<IfcCartesianPointList3D>( indexedPolyCurve->m_Points );
                if( pointList3d ) {
                    points = this->m_primitivesConverter->ConvertPoints( pointList3d->m_CoordList );
                }
                for( const auto& segment: indexedPolyCurve->m_Segments ) {
                    const auto lineIdx = dynamic_pointer_cast<IfcLineIndex>( segment );
                    if( lineIdx && lineIdx->m_vec.size() > 1 && lineIdx->m_vec[ 0 ] && lineIdx->m_vec[ 1 ] ) {
                        int idx0 = lineIdx->m_vec[ 0 ]->m_value - 1;
                        int idx1 = lineIdx->m_vec[ 1 ]->m_value - 1;
                        if( 0 <= idx0 && idx0 < points.size() && 0 <= idx1 && idx1 < points.size() ) {
                            result.push_back( points[ idx0 ] );
                            result.push_back( points[ idx1 ] );
                        }
                    }
                    const auto arcIdx = dynamic_pointer_cast<IfcArcIndex>( segment );
                    if( arcIdx && arcIdx->m_vec.size() > 2 && arcIdx->m_vec[ 0 ] && arcIdx->m_vec[ 1 ] && arcIdx->m_vec[ 2 ] ) {
                        int idx0 = arcIdx->m_vec[ 0 ]->m_value - 1;
                        int idx1 = arcIdx->m_vec[ 1 ]->m_value - 1;
                        int idx2 = arcIdx->m_vec[ 2 ]->m_value - 1;
                        if( 0 <= idx0 && idx0 < points.size() && 0 <= idx1 && idx1 < points.size() && 0 <= idx2 && idx2 < points.size() ) {
                            const auto arcPoints = this->m_geomUtils->BuildArc( points[ idx0 ], points[ idx1 ], points[ idx2 ] );
                            std::back_inserter( std::begin( arcPoints ), std::end( arcPoints ), result );
                        }
                    }
                }
                return result;
            }
        }

        const auto conic = dynamic_pointer_cast<IfcConic>( curve );
        if( conic ) {
            // ENTITY IfcConic ABSTRACT SUPERTYPE OF(ONEOF(IfcCircle, IfcEllipse))
            const auto placement = this->m_primitivesConverter->ConvertPlacement( conic->m_Position );
            const auto circleCenter = placement.GetTransformed( AVector::New() );
            float radius1 = 0.0f;
            float radius2 = 0.0f;

            const auto ellipse = dynamic_pointer_cast<IfcEllipse>( conic );
            if( ellipse && ellipse->m_SemiAxis1 && ellipse->m_SemiAxis2 ) {
                radius1 = (float)ellipse->m_SemiAxis1->m_value;
                radius2 = (float)ellipse->m_SemiAxis2->m_value;
            }
            shared_ptr<IfcCircle> circle = dynamic_pointer_cast<IfcCircle>( conic );
            if( circle && circle->m_Radius ) {
                radius1 = radius2 = (float)circle->m_Radius->m_value;
            }

            TCurve result;
            int n = this->m_parameters->m_numVerticesPerCircle; // TODO: should be calculated from radius
            n = std::max( n, this->m_parameters->m_minNumVerticesPerArc );
            if( radius1 > this->m_parameters->m_epsilon && radius2 > this->m_parameters->m_epsilon ) {
                result = this->m_geomUtils->BuildEllipse( radius1, radius2, 0.0f, (float)( M_PI * 2 ), n, circleCenter );
            } else {
                result.push_back( circleCenter );
            }
            return result;
        }

        const auto line = dynamic_pointer_cast<IfcLine>( curve );
        if( line && line->m_Pnt && line->m_Dir ) {
            const auto origin = this->m_primitivesConverter->ConvertPoint( line->m_Pnt );
            const auto direction = AVector::Normalized( this->m_primitivesConverter->ConvertVector( line->m_Dir ) );
            return {
                origin - direction * this->m_parameters->m_modelMaxSize * 0.5f,
                origin + direction * this->m_parameters->m_modelMaxSize * 0.5f,
            };
        }

        const auto offset_curve_2d = dynamic_pointer_cast<IfcOffsetCurve2D>( curve );
        if( offset_curve_2d ) {
            // TODO: implement
            return {};
        }

        const auto offset_curve_3d = dynamic_pointer_cast<IfcOffsetCurve3D>( curve );
        if( offset_curve_3d ) {
            // TODO: implement
            return {};
        }

        const auto pcurve = dynamic_pointer_cast<IfcPcurve>( curve );
        if( pcurve ) {
            // TODO: implement
            return {};
        }
    }

    TCurve TrimCurve( const TCurve& curve, const TVector& t1, const TVector& t2 ) {
        const auto preprocessedCurve = curve;
        std::back_inserter( std::begin( curve ), std::end( curve ), preprocessedCurve );
        auto l = this->IsPointOnCurve( curve, t1 );
        auto r = this->IsPointOnCurve( curve, t2 );
        int lidx = -1;
        int ridx = -1;
        for( int i = 1; i < preprocessedCurve.size(); i++ ) {
            if( lidx < 0 ) {
                if( this->m_geomUtils->IsPointOnEdge( preprocessedCurve[ i - 1 ], preprocessedCurve[ i ], l ) ) {
                    lidx = i;
                }
            } else if( ridx < 0 ) {
                if( this->m_geomUtils->IsPointOnEdge( preprocessedCurve[ i - 1 ], preprocessedCurve[ i ], r ) ) {
                    ridx = i;
                }
            } else {
                break;
            }
        }
        if( lidx < 0 ) {
            lidx = 0;
        }
        if( ridx < 0 ) {
            ridx = curve.size();
        }
        TCurve result;
        result.push_back( t1 );
        std::back_inserter( std::begin( preprocessedCurve ) + lidx, std::begin( preprocessedCurve ) + ridx, result );
        result.push_back( t2 );
        return this->m_geomUtils->SimplifyCurve( result );
    }

private:
    std::tuple<float, float> GetTrimmingsForCircle( const std::shared_ptr<IfcTrimmingSelect>& t1, const std::shared_ptr<IfcTrimmingSelect>& t2, TVector center,
                                                    bool senseArgement ) {
        float l = this->GetTrimmingForCircle( t1, center );
        float r = this->GetTrimmingForCircle( t2, center );
        float opening = 0.0f;
        if( l > r ) {
            std::swap( l, r );
        }
        if( senseArgement ) {
            opening = r - l;
        } else {
            opening = (float)( l - r + M_PI * 2 );
        }
        return { l, opening };
    }

    float GetTrimmingForCircle( const std::shared_ptr<IfcTrimmingSelect>& t, TVector center ) {
        const auto cartesianPoint = dynamic_pointer_cast<IfcCartesianPoint>( t );
        if( cartesianPoint ) {
            const auto dir = AVector::Normalized( this->m_primitivesConverter->ConvertPoint( cartesianPoint ) - center );
            return asinf( AVector::Len( AVector::Cross( AVector::New( 1, 0, 0 ), dir ) ) );
        }
        const auto parameterValue = std::dynamic_pointer_cast<IfcParameterValue>( t );
        if( parameterValue ) {
            return (float)( parameterValue->m_value * this->m_parameters->m_angleFactor );
        }
        // TODO: Log error
        return 0;
    }

    std::tuple<float, float> GetTrimmingsForLine( const std::shared_ptr<IfcTrimmingSelect>& t1, const std::shared_ptr<IfcTrimmingSelect>& t2, TVector origin,
                                                  TVector dir, bool senseArgement ) {
        float l = this->GetTrimmingForLine( t1, origin, dir );
        float r = this->GetTrimmingForLine( t2, origin, dir );
        if( l > r ) {
            std::swap( l, r );
        }
        if( senseArgement ) {
            return { l, r };
        }
        return { r, l };
    }

    float GetTrimmingForLine( const std::shared_ptr<IfcTrimmingSelect>& t, TVector origin, TVector dir ) {
        const auto cartesianPoint = dynamic_pointer_cast<IfcCartesianPoint>( t );
        if( cartesianPoint ) {
            return AVector::Dot( dir, this->m_primitivesConverter->ConvertPoint( cartesianPoint ) - origin );
        }
        const auto parameterValue = std::dynamic_pointer_cast<IfcParameterValue>( t );
        if( parameterValue ) {
            return (float)parameterValue->m_value;
        }
        // TODO: Log error
        return 0;
    }

    TVector IsPointOnCurve( const TCurve& curve, const TVector& point ) {
        TVector result = curve[ 0 ];
        float distance2 = AVector::Len2( point - result );
        for( int i = 1; i < curve.size(); i++ ) {
            TVector r = this->m_geomUtils->ClosestPointOnEdge( curve[ i - 1 ], curve[ i ] );
            float d2 = AVector::Len2( point - r );
            if( d2 < distance2 ) {
                result = r;
                distance2 = d2;
            }
        }
        return result;
    }
};

}