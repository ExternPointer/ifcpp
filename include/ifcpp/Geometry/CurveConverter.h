#pragma once

#include <map>
#include <cmath>

#include "ifcpp/Model/OpenMPIncludes.h"

#include "ifcpp/Geometry/CAdapter.h"
#include "ifcpp/Geometry/GeomUtils.h"
#include "ifcpp/Geometry/Matrix.h"
#include "ifcpp/Geometry/Parameters.h"
#include "ifcpp/Geometry/PrimitiveTypesConverter.h"
#include "ifcpp/Geometry/SplineConverter.h"
#include "ifcpp/Geometry/VectorAdapter.h"

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

    std::shared_ptr<PrimitiveTypesConverter<TVector>> m_primitivesConverter;
    std::shared_ptr<GeomUtils<TVector>> m_geomUtils;
    std::shared_ptr<SplineConverter<TVector>> m_splineConverter;
    std::shared_ptr<Parameters> m_parameters;

    std::map<std::shared_ptr<IfcCurve>, TCurve> m_curveToPointsMap;
#ifdef ENABLE_OPENMP
    Mutex m_curveToPointsMapMutex;
#endif

public:
    CurveConverter( const std::shared_ptr<PrimitiveTypesConverter<TVector>>& primitivesConverter, const std::shared_ptr<GeomUtils<TVector>>& geomUtils,
                    const std::shared_ptr<SplineConverter<TVector>> splineConverter, const std::shared_ptr<Parameters>& parameters )
        : m_primitivesConverter( primitivesConverter )
        , m_geomUtils( geomUtils )
        , m_splineConverter( splineConverter )
        , m_parameters( parameters ) {
    }

    void ResetCaches() {
#ifdef ENABLE_OPENMP
        ScopedLock lock( this->m_curveToPointsMapMutex );
#endif
        this->m_curveToPointsMap = {};
    }

    TCurve ConvertCurve( const std::shared_ptr<IfcCurve>& curve ) {
        // ENTITY IfcCurve ABSTRACT SUPERTYPE OF (ONEOF(IfcBoundedCurve, IfcConic, IfcLine, IfcOffsetCurve2D, IfcOffsetCurve3D, IfcPCurve))

        TCurve* cached = nullptr;
        {
#ifdef ENABLE_OPENMP
            ScopedLock lock( this->m_curveToPointsMapMutex );
#endif
            if( this->m_curveToPointsMap.contains( curve ) ) {
                cached = &this->m_curveToPointsMap[ curve ];
            }
        }
        if( cached ) {
            return *cached;
        }

        TCurve result;

        const auto boundedCurve = dynamic_pointer_cast<IfcBoundedCurve>( curve );
        if( boundedCurve ) {
            // ENTITY IfcBoundedCurve ABSTRACT SUPERTYPE OF (ONEOF(IfcCompositeCurve, IfcPolyline, IfcTrimmedCurve, IfcBSplineCurve))

            const auto compositeCurve = dynamic_pointer_cast<IfcCompositeCurve>( boundedCurve );
            if( compositeCurve ) {
                for( const auto& segment: compositeCurve->m_Segments ) {
                    TCurve segmentPoints;
                    const auto compositeCurveSegment = dynamic_pointer_cast<IfcCompositeCurveSegment>( segment );
                    if( compositeCurveSegment ) {
                        segmentPoints = this->ConvertCurve( compositeCurveSegment->m_ParentCurve );
                        // FIXME: concatenation
                        if( compositeCurveSegment->m_SameSense && !compositeCurveSegment->m_SameSense->m_value ) {
                            std::copy( std::rbegin( segmentPoints ), std::rend( segmentPoints ), std::back_inserter( result ) );
                        } else {
                            std::copy( std::begin( segmentPoints ), std::end( segmentPoints ), std::back_inserter( result ) );
                        }
                    } else {
                        // TODO: Log error
                    }
                }
                result = this->m_geomUtils->SimplifyCurve( result );
            }

            const auto polyLine = dynamic_pointer_cast<IfcPolyline>( curve );
            if( polyLine ) {
                result = this->m_geomUtils->SimplifyCurve( this->m_primitivesConverter->ConvertPoints( polyLine->m_Points ) );
            }

            const auto trimmedCurve = dynamic_pointer_cast<IfcTrimmedCurve>( boundedCurve );
            if( trimmedCurve ) {
                if( trimmedCurve->m_BasisCurve ) {
                    const auto conic = dynamic_pointer_cast<IfcConic>( trimmedCurve->m_BasisCurve );
                    if( conic ) {
                        // ENTITY IfcConic ABSTRACT SUPERTYPE OF(ONEOF(IfcCircle, IfcEllipse))
                        const auto placement = this->m_primitivesConverter->ConvertPlacement( conic->m_Position );
                        double radius1 = 0.0;
                        double radius2 = 0.0;

                        const auto ellipse = dynamic_pointer_cast<IfcEllipse>( conic );
                        if( ellipse && ellipse->m_SemiAxis1 && ellipse->m_SemiAxis2 ) {
                            radius1 = ellipse->m_SemiAxis1->m_value;
                            radius2 = ellipse->m_SemiAxis2->m_value;
                        }
                        const auto circle = dynamic_pointer_cast<IfcCircle>( conic );
                        if( circle && circle->m_Radius ) {
                            radius1 = radius2 = circle->m_Radius->m_value;
                        }

                        int n = this->m_parameters->m_numVerticesPerCircle; // TODO: should be calculated from radius
                        n = std::max( n, this->m_parameters->m_minNumVerticesPerArc );
                        bool senseAgreement = !trimmedCurve->m_SenseAgreement || trimmedCurve->m_SenseAgreement->m_value;
                        auto [ startAngle, openingAngle ] = this->GetTrimmingsForCircle( trimmedCurve->m_Trim1, trimmedCurve->m_Trim2,
                                                                                         placement.GetTransformed( AVector::New() ), senseAgreement );
                        if( radius1 > this->m_parameters->m_epsilon && radius2 > this->m_parameters->m_epsilon ) {
                            result = this->m_geomUtils->BuildEllipse( radius1, radius2, startAngle, openingAngle, n );
                        } else {
                            result.push_back( AVector::New() );
                        }
                        placement.TransformLoop( &result );
                    }

                    const auto line = dynamic_pointer_cast<IfcLine>( trimmedCurve->m_BasisCurve );
                    if( line && line->m_Pnt && line->m_Dir ) {
                        const auto origin = this->m_primitivesConverter->ConvertPoint( line->m_Pnt );
                        const auto direction = AVector::Normalized( this->m_primitivesConverter->ConvertVector( line->m_Dir ) );
                        bool senseAgreement = !trimmedCurve->m_SenseAgreement || trimmedCurve->m_SenseAgreement->m_value;
                        const auto [ l, r ] = this->GetTrimmingsForLine( trimmedCurve->m_Trim1, trimmedCurve->m_Trim2, origin, direction, senseAgreement );
                        result = {
                            origin + direction * l,
                            origin + direction * r,
                        };
                    }
                }
            }

            const auto bsplineCurve = dynamic_pointer_cast<IfcBSplineCurve>( boundedCurve );
            if( bsplineCurve ) {
                result = this->m_splineConverter->ConvertBSplineCurve( bsplineCurve );
            }

            const auto indexedPolyCurve = dynamic_pointer_cast<IfcIndexedPolyCurve>( boundedCurve );
            if( indexedPolyCurve ) {
                std::vector<TVector> points;
                const auto pointList2d = dynamic_pointer_cast<IfcCartesianPointList2D>( indexedPolyCurve->m_Points );
                if( pointList2d ) {
                    points = this->m_primitivesConverter->ConvertPoints( pointList2d->m_CoordList );
                }
                const auto pointList3d = dynamic_pointer_cast<IfcCartesianPointList3D>( indexedPolyCurve->m_Points );
                if( pointList3d ) {
                    points = this->m_primitivesConverter->ConvertPoints( pointList3d->m_CoordList );
                }
                if( !indexedPolyCurve->m_Segments.empty() ) {
                    for( const auto& segment: indexedPolyCurve->m_Segments ) {
                        const auto lineIdx = dynamic_pointer_cast<IfcLineIndex>( segment );
                        if( lineIdx && lineIdx->m_vec.size() > 1 && lineIdx->m_vec[ 0 ] && lineIdx->m_vec[ 1 ] ) {
                            for( const auto& idx: lineIdx->m_vec ) {
                                if( idx && idx->m_value > 0 && idx->m_value <= points.size() ) {
                                    result.push_back( points[ idx->m_value - 1 ] );
                                }
                            }
                        }
                        const auto arcIdx = dynamic_pointer_cast<IfcArcIndex>( segment );
                        if( arcIdx && arcIdx->m_vec.size() > 2 && arcIdx->m_vec[ 0 ] && arcIdx->m_vec[ 1 ] && arcIdx->m_vec[ 2 ] ) {
                            int idx0 = arcIdx->m_vec[ 0 ]->m_value - 1;
                            int idx1 = arcIdx->m_vec[ 1 ]->m_value - 1;
                            int idx2 = arcIdx->m_vec[ 2 ]->m_value - 1;
                            if( 0 <= idx0 && idx0 < points.size() && 0 <= idx1 && idx1 < points.size() && 0 <= idx2 && idx2 < points.size() ) {
                                const auto arcPoints = this->m_geomUtils->BuildArc( points[ idx0 ], points[ idx1 ], points[ idx2 ] );
                                std::copy( std::begin( arcPoints ), std::end( arcPoints ), std::back_inserter( result ) );
                            }
                        }
                    }
                } else {
                    result = std::move( points );
                }
            }
        }

        const auto conic = dynamic_pointer_cast<IfcConic>( curve );
        if( conic ) {
            // ENTITY IfcConic ABSTRACT SUPERTYPE OF(ONEOF(IfcCircle, IfcEllipse))
            const auto placement = this->m_primitivesConverter->ConvertPlacement( conic->m_Position );
            double radius1 = 0.0;
            double radius2 = 0.0;

            const auto ellipse = dynamic_pointer_cast<IfcEllipse>( conic );
            if( ellipse && ellipse->m_SemiAxis1 && ellipse->m_SemiAxis2 ) {
                radius1 = ellipse->m_SemiAxis1->m_value;
                radius2 = ellipse->m_SemiAxis2->m_value;
            }
            shared_ptr<IfcCircle> circle = dynamic_pointer_cast<IfcCircle>( conic );
            if( circle && circle->m_Radius ) {
                radius1 = radius2 = circle->m_Radius->m_value;
            }

            int n = this->m_parameters->m_numVerticesPerCircle; // TODO: should be calculated from radius
            n = std::max( n, this->m_parameters->m_minNumVerticesPerArc );
            if( radius1 > this->m_parameters->m_epsilon && radius2 > this->m_parameters->m_epsilon ) {
                result = this->m_geomUtils->BuildEllipse( radius1, radius2, 0.0, ( M_PI * 2 ), n );
            } else {
                result.push_back( AVector::New() );
            }
            placement.TransformLoop( &result );
        }

        const auto line = dynamic_pointer_cast<IfcLine>( curve );
        if( line && line->m_Pnt && line->m_Dir ) {
            const auto origin = this->m_primitivesConverter->ConvertPoint( line->m_Pnt );
            const auto direction = AVector::Normalized( this->m_primitivesConverter->ConvertVector( line->m_Dir ) );
            result = {
                origin - direction * this->m_parameters->m_modelMaxSize * 0.5,
                origin + direction * this->m_parameters->m_modelMaxSize * 0.5,
            };
        }

        const auto offset_curve_2d = dynamic_pointer_cast<IfcOffsetCurve2D>( curve );
        if( offset_curve_2d ) {
            // TODO: implement
            result = {};
        }

        const auto offset_curve_3d = dynamic_pointer_cast<IfcOffsetCurve3D>( curve );
        if( offset_curve_3d && offset_curve_3d->m_RefDirection && offset_curve_3d->m_Distance ) {
            auto baseCurve = this->ConvertCurve( offset_curve_3d->m_BasisCurve );
            auto offset = this->m_primitivesConverter->ConvertPoint( offset_curve_3d->m_RefDirection->m_DirectionRatios ) * offset_curve_3d->m_Distance->m_value;
            for( auto& p: baseCurve ) {
                p = p + offset;
            }
            result = std::move( baseCurve );
        }

        const auto pcurve = dynamic_pointer_cast<IfcPcurve>( curve );
        if( pcurve ) {
            // TODO: implement
            result = {};
        }

        auto resultCopy = result;
        {
#ifdef ENABLE_OPENMP
            ScopedLock lock( this->m_curveToPointsMapMutex );
#endif
            if( !this->m_curveToPointsMap.contains( curve ) ) {
                this->m_curveToPointsMap[ curve ] = std::move( resultCopy );
            }
        }
        return result;
    }

    TCurve TrimCurve( const TCurve& curve, const TVector& t1, const TVector& t2 ) {
        auto preprocessedCurve = curve;
        if( AVector::IsNearlyEqual( preprocessedCurve[ 0 ], preprocessedCurve[ preprocessedCurve.size() - 1 ] ) ) {
            std::copy( std::begin( curve ), std::end( curve ), std::back_inserter( preprocessedCurve ) );
        }
        auto l = this->GetClosestPointOnCurve( curve, t1 );
        auto r = this->GetClosestPointOnCurve( curve, t2 );
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
        std::copy( std::begin( preprocessedCurve ) + lidx, std::begin( preprocessedCurve ) + ridx, std::back_inserter( result ) );
        result.push_back( t2 );
        return this->m_geomUtils->SimplifyCurve( result );
    }

private:
    std::tuple<double, double> GetTrimmingsForCircle( const std::vector<std::shared_ptr<IfcTrimmingSelect>>& t1,
                                                      const std::vector<std::shared_ptr<IfcTrimmingSelect>>& t2, TVector center, bool senseAgreement ) {
        double l = this->GetTrimmingForCircle( t1, center );
        double r = this->GetTrimmingForCircle( t2, center );

        if( senseAgreement && l < r ) {
            return { l, r - l };
        }
        if( senseAgreement && l > r ) {
            return { l, M_PI * 2 - l + r };
        }
        if( !senseAgreement && l < r ) {
            return { l, -l - ( M_PI * 2 - r ) };
        }
        if( !senseAgreement && l > r ) {
            return { l, l - r };
        }
        return { l, 0.0 };
    }

    double GetTrimmingForCircle( std::vector<std::shared_ptr<IfcTrimmingSelect>> t, TVector center ) {
        if( t.size() > 1 && std::dynamic_pointer_cast<IfcParameterValue>( t[ 1 ] ) ) {
            std::swap( t[ 0 ], t[ 1 ] );
        }
        const auto cartesianPoint = dynamic_pointer_cast<IfcCartesianPoint>( t[ 0 ] );
        if( cartesianPoint ) {
            const auto dir = AVector::Normalized( this->m_primitivesConverter->ConvertPoint( cartesianPoint ) - center );
            // TODO: Verify code
            if( dir.x >= this->m_parameters->m_epsilon && dir.y >= this->m_parameters->m_epsilon ) {
                return acos( AVector::Dot( AVector::New( 1, 0, 0 ), dir ) );
            } else if( dir.x < -this->m_parameters->m_epsilon && dir.y >= this->m_parameters->m_epsilon ) {
                return M_PI_2 + acos( AVector::Dot( AVector::New( 0, 1, 0 ), dir ) );
            } else if( dir.x < -this->m_parameters->m_epsilon && dir.y < -this->m_parameters->m_epsilon ) {
                return M_PI + acos( AVector::Dot( AVector::New( -1, 0, 0 ), dir ) );
            } else {
                return M_PI + M_PI_2 + acos( AVector::Dot( AVector::New( 0, -1, 0 ), dir ) );
            }
        }
        const auto parameterValue = std::dynamic_pointer_cast<IfcParameterValue>( t[ 0 ] );
        if( parameterValue ) {
            return ( parameterValue->m_value * this->m_parameters->m_angleFactor );
        }
        // TODO: Log error
        return 0;
    }

    std::tuple<double, double> GetTrimmingsForLine( const std::vector<std::shared_ptr<IfcTrimmingSelect>>& t1,
                                                    const std::vector<std::shared_ptr<IfcTrimmingSelect>>& t2, TVector origin, TVector dir,
                                                    bool senseArgement ) {
        double l = this->GetTrimmingForLine( t1, origin, dir );
        double r = this->GetTrimmingForLine( t2, origin, dir );
        if( senseArgement && l > r || !senseArgement && l < r ) {
            std::swap( l, r );
        }
        return { l, r };
    }

    double GetTrimmingForLine( std::vector<std::shared_ptr<IfcTrimmingSelect>> t, TVector origin, TVector dir ) {
        if( t.size() > 1 && std::dynamic_pointer_cast<IfcParameterValue>( t[ 1 ] ) ) {
            std::swap( t[ 0 ], t[ 1 ] );
        }
        const auto cartesianPoint = dynamic_pointer_cast<IfcCartesianPoint>( t[ 0 ] );
        if( cartesianPoint ) {
            return AVector::Dot( dir, this->m_primitivesConverter->ConvertPoint( cartesianPoint ) - origin );
        }
        const auto parameterValue = std::dynamic_pointer_cast<IfcParameterValue>( t[ 0 ] );
        if( parameterValue ) {
            return parameterValue->m_value;
        }
        // TODO: Log error
        return 0;
    }

    TVector GetClosestPointOnCurve( const TCurve& curve, const TVector& point ) {
        TVector result = curve[ 0 ];
        double distance2 = AVector::Len2( point - result );
        for( int i = 1; i < curve.size(); i++ ) {
            TVector r = this->m_geomUtils->ClosestPointOnEdge( curve[ i - 1 ], curve[ i ], point );
            double d2 = AVector::Len2( point - r );
            if( d2 < distance2 ) {
                result = r;
                distance2 = d2;
            }
        }
        return result;
    }
};

}