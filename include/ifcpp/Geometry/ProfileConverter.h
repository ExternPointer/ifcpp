#pragma once

#include <map>
#include <vector>

#include "ifcpp/Model/OpenMPIncludes.h"

#include "ifcpp/Geometry/CAdapter.h"
#include "ifcpp/Geometry/CurveConverter.h"
#include "ifcpp/Geometry/GeomUtils.h"
#include "ifcpp/Geometry/Matrix.h"
#include "ifcpp/Geometry/Parameters.h"
#include "ifcpp/Geometry/PrimitiveTypesConverter.h"
#include "ifcpp/Geometry/VectorAdapter.h"

#include "ifcpp/Ifc/IfcArbitraryClosedProfileDef.h"
#include "ifcpp/Ifc/IfcArbitraryOpenProfileDef.h"
#include "ifcpp/Ifc/IfcArbitraryProfileDefWithVoids.h"
#include "ifcpp/Ifc/IfcAsymmetricIShapeProfileDef.h"
#include "ifcpp/Ifc/IfcCShapeProfileDef.h"
#include "ifcpp/Ifc/IfcCenterLineProfileDef.h"
#include "ifcpp/Ifc/IfcCircleHollowProfileDef.h"
#include "ifcpp/Ifc/IfcCircleProfileDef.h"
#include "ifcpp/Ifc/IfcCompositeProfileDef.h"
#include "ifcpp/Ifc/IfcDerivedProfileDef.h"
#include "ifcpp/Ifc/IfcEllipseProfileDef.h"
#include "ifcpp/Ifc/IfcIShapeProfileDef.h"
#include "ifcpp/Ifc/IfcLShapeProfileDef.h"
#include "ifcpp/Ifc/IfcNonNegativeLengthMeasure.h"
#include "ifcpp/Ifc/IfcParameterizedProfileDef.h"
#include "ifcpp/Ifc/IfcPlaneAngleMeasure.h"
#include "ifcpp/Ifc/IfcProfileDef.h"
#include "ifcpp/Ifc/IfcRectangleHollowProfileDef.h"
#include "ifcpp/Ifc/IfcRectangleProfileDef.h"
#include "ifcpp/Ifc/IfcRoundedRectangleProfileDef.h"
#include "ifcpp/Ifc/IfcTShapeProfileDef.h"
#include "ifcpp/Ifc/IfcTrapeziumProfileDef.h"
#include "ifcpp/Ifc/IfcUShapeProfileDef.h"
#include "ifcpp/Ifc/IfcZShapeProfileDef.h"


namespace ifcpp {
using namespace IFC4X3;

template<CVector TVector>
class ProfileConverter {
    using TPath = std::vector<TVector>;
    using AVector = VectorAdapter<TVector>;

    std::shared_ptr<CurveConverter<TVector>> m_curveConverter;
    std::shared_ptr<PrimitiveTypesConverter<TVector>> m_primitivesConverter;
    std::shared_ptr<GeomUtils<TVector>> m_geomUtils;
    std::shared_ptr<Parameters> m_parameters;

    std::map<std::shared_ptr<IfcProfileDef>, std::vector<TVector>> m_profileToPointsMap;
#ifdef ENABLE_OPENMP
    Mutex m_profileToPointsMapMutex;
#endif

public:
    ProfileConverter( const std::shared_ptr<CurveConverter<TVector>>& curveConverter, const std::shared_ptr<GeomUtils<TVector>> geomUtils,
                      const std::shared_ptr<PrimitiveTypesConverter<TVector>>& primitivesConverter, const std::shared_ptr<Parameters>& parameters )
        : m_curveConverter( curveConverter )
        , m_geomUtils( geomUtils )
        , m_primitivesConverter( primitivesConverter )
        , m_parameters( parameters ) {
    }

    void ResetCaches() {
#ifdef ENABLE_OPENMP
        ScopedLock lock( this->m_profileToPointsMapMutex );
#endif
        this->m_profileToPointsMap = {};
    }

    std::vector<TVector> ConvertProfile( const std::shared_ptr<IfcProfileDef>& profile ) {
        // ENTITY IfcProfileDef SUPERTYPE OF(ONEOF(IfcArbitraryClosedProfileDef, IfcArbitraryOpenProfileDef, IfcCompositeProfileDef, IfcDerivedProfileDef,
        // IfcParameterizedProfileDef));

        {
#ifdef ENABLE_OPENMP
            ScopedLock lock( this->m_profileToPointsMapMutex );
#endif
            if( this->m_profileToPointsMap.contains( profile ) ) {
                return this->m_profileToPointsMap[ profile ];
            }
        }

        std::vector<TVector> result;

        const auto parameterized = dynamic_pointer_cast<IfcParameterizedProfileDef>( profile );
        if( parameterized ) {
            result = this->ConvertParameterizedProfileDef( parameterized );
        }

        const auto arbitraryClosed = dynamic_pointer_cast<IfcArbitraryClosedProfileDef>( profile );
        if( arbitraryClosed ) {
            result = this->ConvertArbitraryClosedProfileDef( arbitraryClosed );
        }

        const auto arbitraryOpen = dynamic_pointer_cast<IfcArbitraryOpenProfileDef>( profile );
        if( arbitraryOpen ) {
            result = this->ConvertArbitraryOpenProfileDef( arbitraryOpen );
        }

        const auto composite = dynamic_pointer_cast<IfcCompositeProfileDef>( profile );
        if( composite ) {
            result = this->ConvertCompositeProfileDef( composite );
        }

        const auto derived = dynamic_pointer_cast<IfcDerivedProfileDef>( profile );
        if( derived ) {
            result = this->ConvertDerivedProfileDef( derived );
        }


        auto resultCopy = result;
        {
#ifdef ENABLE_OPENMP
            ScopedLock lock( this->m_profileToPointsMapMutex );
#endif
            this->m_profileToPointsMap[ profile ] = std::move( resultCopy );
        }

        return result;
    }

private:
    std::vector<TVector> ConvertArbitraryClosedProfileDef( const shared_ptr<IfcArbitraryClosedProfileDef>& profile ) {
        if( !profile || !profile->m_OuterCurve ) {
            return {};
        }

        const auto outer = this->m_geomUtils->SimplifyCurve( this->m_curveConverter->ConvertCurve( profile->m_OuterCurve ) );
        std::vector<TPath> holes;

        shared_ptr<IfcArbitraryProfileDefWithVoids> profileWithVoids = dynamic_pointer_cast<IfcArbitraryProfileDefWithVoids>( profile );
        if( profileWithVoids ) {
            for( const auto& inner: profileWithVoids->m_InnerCurves ) {
                holes.push_back( this->m_geomUtils->SimplifyCurve( this->m_curveConverter->ConvertCurve( inner ) ) );
            }
        }

        auto result = this->m_geomUtils->IncorporateHoles( outer, holes );
        if( !result.empty() ) {
            result.push_back( result[ 0 ] );
        }
        return result;
    }

    TPath ConvertArbitraryOpenProfileDef( const shared_ptr<IfcArbitraryOpenProfileDef>& profile ) {
        shared_ptr<IfcCenterLineProfileDef> centerLineProfileDef = dynamic_pointer_cast<IfcCenterLineProfileDef>( profile );
        if( centerLineProfileDef ) {
            double thickness = 0.0;
            if( centerLineProfileDef->m_Thickness ) {
                thickness = centerLineProfileDef->m_Thickness->m_value;
            }
            const auto curve = this->m_geomUtils->SimplifyCurve( this->m_curveConverter->ConvertCurve( centerLineProfileDef->m_Curve ) );
            if( curve.size() <= 1 ) {
                return {};
            }
            std::vector<TVector> normals;
            normals.push_back( this->m_geomUtils->Normal2d( curve[ 0 ], curve[ 1 ] ) );
            for( int i = 1; i < curve.size() - 1; i++ ) {
                const auto t = this->m_geomUtils->Normal2d( curve[ i - 1 ], curve[ i ] ) + this->m_geomUtils->Normal2d( curve[ i ], curve[ i + 1 ] );
                const auto l = AVector::Len( t );
                normals.push_back( AVector::Normalized( t ) * ( 1.0 / sin( ( acos( l / 2 ) + M_PI ) / 2.0 ) ) );
            }
            normals.push_back( this->m_geomUtils->Normal2d( curve[ curve.size() - 2 ], curve[ curve.size() - 1 ] ) );
            std::vector<TVector> left, right;
            for( int i = 0; i < curve.size(); i++ ) {
                left.push_back( curve[ i ] - normals[ i ] * thickness * 0.5 );
                right.push_back( curve[ i ] + normals[ i ] * thickness * 0.5 );
            }
            auto result = left;
            std::copy( std::rbegin( right ), std::rend( right ), std::back_inserter( result ) );
            if( !result.empty() ) {
                result.push_back( result[ 0 ] );
            }
            return this->m_geomUtils->SimplifyCurve( result );
        }
        return this->m_geomUtils->SimplifyCurve( this->m_curveConverter->ConvertCurve( profile->m_Curve ) );
    }

    TPath ConvertCompositeProfileDef( const shared_ptr<IfcCompositeProfileDef>& profile ) {
        std::vector<TPath> paths;
        for( const auto& p: profile->m_Profiles ) {
            paths.push_back( this->ConvertProfile( p ) );
        }
        auto result = this->m_geomUtils->CombineLoops( paths );
        if( !result.empty() ) {
            result.push_back( result[ 0 ] );
        }
        return result;
    }

    TPath ConvertDerivedProfileDef( const shared_ptr<IfcDerivedProfileDef>& profile ) {
        auto path = this->ConvertProfile( profile->m_ParentProfile );
        const auto m = this->m_primitivesConverter->ConvertTransformationOperator( profile->m_Operator );

        for( auto& p: path ) {
            m.Transform( &p );
        }

        return path;
    }

    TPath ConvertParameterizedProfileDef( const shared_ptr<IfcParameterizedProfileDef>& profile ) {
        auto path = this->ConvertParameterizedProfileDefWithoutPosition( profile );

        if( profile->m_Position ) {
            const auto m = this->m_primitivesConverter->ConvertPlacement( profile->m_Position );
            for( auto& p: path ) {
                m.Transform( &p );
            }
        }

        return path;
    }

    TPath ConvertParameterizedProfileDefWithoutPosition( const shared_ptr<IfcParameterizedProfileDef>& profile ) {
        // IfcParameterizedProfileDef ABSTRACT SUPERTYPE OF (ONEOF
        //	(IfcCShapeProfileDef, IfcCircleProfileDef, IfcEllipseProfileDef, IfcIShapeProfileDef, IfcLShapeProfileDef,
        //	IfcRectangleProfileDef, IfcTShapeProfileDef, IfcTrapeziumProfileDef, IfcUShapeProfileDef, IfcZShapeProfileDef))

        const auto rectangle = dynamic_pointer_cast<IfcRectangleProfileDef>( profile );
        if( rectangle && rectangle->m_XDim && rectangle->m_YDim ) {
            auto xDim = rectangle->m_XDim->m_value;
            auto yDim = rectangle->m_YDim->m_value;

            const auto hollow = dynamic_pointer_cast<IfcRectangleHollowProfileDef>( rectangle );
            if( hollow && hollow->m_WallThickness ) {
                TPath outer;
                TPath inner;

                const auto t = hollow->m_WallThickness->m_value;
                const auto outerR = hollow->m_OuterFilletRadius->m_value;
                const auto innerR = hollow->m_InnerFilletRadius->m_value;

                if( outerR > this->m_parameters->m_epsilon ) {
                    this->AddArc( &outer, outerR, 0, M_PI_2, xDim * 0.5 - outerR, yDim * 0.5 - outerR );
                    this->AddArc( &outer, outerR, M_PI_2, M_PI_2, -xDim * 0.5 + outerR, yDim * 0.5 - outerR );
                    this->AddArc( &outer, outerR, M_PI, M_PI_2, -xDim * 0.5 + outerR, -yDim * 0.5 + outerR );
                    this->AddArc( &outer, outerR, 3 * M_PI_2, M_PI_2, xDim * 0.5 - outerR, -yDim * 0.5 + outerR );
                } else {
                    outer.push_back( AVector::New( -xDim * 0.5, -yDim * 0.5 ) );
                    outer.push_back( AVector::New( xDim * 0.5, -yDim * 0.5 ) );
                    outer.push_back( AVector::New( xDim * 0.5, yDim * 0.5 ) );
                    outer.push_back( AVector::New( -xDim * 0.5, yDim * 0.5 ) );
                }

                xDim -= 2 * t;
                yDim -= 2 * t;
                if( innerR > this->m_parameters->m_epsilon ) {
                    this->AddArc( &inner, innerR, 0, M_PI_2, xDim * 0.5 - innerR, yDim * 0.5 - innerR );
                    this->AddArc( &inner, innerR, M_PI_2, M_PI_2, -xDim * 0.5 + innerR, yDim * 0.5 - innerR );
                    this->AddArc( &inner, innerR, M_PI, M_PI_2, -xDim * 0.5 + innerR, -yDim * 0.5 + innerR );
                    this->AddArc( &inner, innerR, 3 * M_PI_2, M_PI_2, xDim * 0.5 - innerR, -yDim * 0.5 + innerR );
                } else {
                    inner.push_back( AVector::New( -xDim * 0.5, -yDim * 0.5 ) );
                    inner.push_back( AVector::New( xDim * 0.5, -yDim * 0.5 ) );
                    inner.push_back( AVector::New( xDim * 0.5, yDim * 0.5 ) );
                    inner.push_back( AVector::New( -xDim * 0.5, yDim * 0.5 ) );
                }
                std::reverse( std::begin( inner ), std::end( inner ) );
                auto result = this->m_geomUtils->CombineLoops( { outer, inner } );
                if( !result.empty() ) {
                    result.push_back( result[ 0 ] );
                }
                return result;
            }

            const auto rounded_rectangle = dynamic_pointer_cast<IfcRoundedRectangleProfileDef>( rectangle );
            if( rounded_rectangle && rounded_rectangle->m_RoundingRadius ) {
                TPath result;
                const auto radius = rounded_rectangle->m_RoundingRadius->m_value;
                this->AddArc( &result, radius, 0, M_PI_2, xDim * 0.5 - radius, yDim * 0.5 - radius );
                this->AddArc( &result, radius, M_PI_2, M_PI_2, -xDim * 0.5 + radius, yDim * 0.5 - radius );
                this->AddArc( &result, radius, M_PI, M_PI_2, -xDim * 0.5 + radius, -yDim * 0.5 + radius );
                this->AddArc( &result, radius, 3 * M_PI_2, M_PI_2, xDim * 0.5 - radius, -yDim * 0.5 + radius );
                if( !result.empty() ) {
                    result.push_back( result[ 0 ] );
                }
                return result;
            }

            return {
                AVector::New( -xDim * 0.5, -yDim * 0.5 ), AVector::New( xDim * 0.5, -yDim * 0.5 ),  AVector::New( xDim * 0.5, yDim * 0.5 ),
                AVector::New( -xDim * 0.5, yDim * 0.5 ),  AVector::New( -xDim * 0.5, -yDim * 0.5 ),
            };
        }

        const auto trapezium = dynamic_pointer_cast<IfcTrapeziumProfileDef>( profile );
        if( trapezium && trapezium->m_BottomXDim && trapezium->m_TopXDim && trapezium->m_TopXOffset && trapezium->m_YDim ) {
            const auto x_bottom = trapezium->m_BottomXDim->m_value;
            const auto x_top = trapezium->m_TopXDim->m_value;
            const auto x_offset = trapezium->m_TopXOffset->m_value;
            const auto y_dim = trapezium->m_YDim->m_value;
            return {
                AVector::New( -x_bottom * 0.5, -y_dim * 0.5 ),
                AVector::New( x_bottom * 0.5, -y_dim * 0.5 ),
                AVector::New( -x_bottom * 0.5 + x_offset + x_top, y_dim * 0.5 ),
                AVector::New( -x_bottom * 0.5 + x_offset, y_dim * 0.5 ),
                AVector::New( -x_bottom * 0.5, -y_dim * 0.5 ),
            };
        }

        shared_ptr<IfcCircleProfileDef> circle_profile_def = dynamic_pointer_cast<IfcCircleProfileDef>( profile );
        if( circle_profile_def && circle_profile_def->m_Radius ) {
            TPath outer, inner;
            auto radius = circle_profile_def->m_Radius->m_value;
            if( radius < this->m_parameters->m_epsilon ) {
                return {};
            }
            // TODO: getNumVerticesPerCircleWithRadius
            outer = this->m_geomUtils->BuildCircle( radius, 0.0, M_PI * 2, this->m_parameters->m_numVerticesPerCircle );

            const auto hollow = dynamic_pointer_cast<IfcCircleHollowProfileDef>( profile );
            if( hollow ) {
                radius -= hollow->m_WallThickness->m_value;
                // TODO: getNumVerticesPerCircleWithRadius
                inner = this->m_geomUtils->BuildCircle( radius, 0.0, M_PI * 2, this->m_parameters->m_numVerticesPerCircle );
            }
            std::reverse( std::begin( inner ), std::end( inner ) );
            auto result = this->m_geomUtils->CombineLoops( { outer, inner } );
            if( !result.empty() ) {
                result.push_back( result[ 0 ] );
            }
            return result;
        }

        const auto ellipse_profile_def = dynamic_pointer_cast<IfcEllipseProfileDef>( profile );
        if( ellipse_profile_def && ellipse_profile_def->m_SemiAxis1 && ellipse_profile_def->m_SemiAxis2 ) {
            const auto x_radius = ellipse_profile_def->m_SemiAxis1->m_value;
            const auto y_radius = ellipse_profile_def->m_SemiAxis2->m_value;
            // TODO: getNumVerticesPerCircleWithRadius
            return this->m_geomUtils->BuildEllipse( x_radius, y_radius, 0.0, ( M_PI * 2 ), this->m_parameters->m_numVerticesPerCircle );
        }

        const auto i_shape = dynamic_pointer_cast<IfcIShapeProfileDef>( profile );
        if( i_shape && i_shape->m_OverallDepth && i_shape->m_OverallWidth && i_shape->m_WebThickness && i_shape->m_FlangeThickness ) {
            const auto h = i_shape->m_OverallDepth->m_value;
            const auto width = i_shape->m_OverallWidth->m_value;
            const auto tw = i_shape->m_WebThickness->m_value;
            const auto tf = i_shape->m_FlangeThickness->m_value;
            const auto fillet_radius = i_shape->m_FilletRadius->m_value;
            auto flange_edge_radius = 0.0;
            if( i_shape->m_FlangeEdgeRadius ) {
                flange_edge_radius = i_shape->m_FlangeEdgeRadius->m_value;
            }
            TPath result;

            result.push_back( AVector::New( width * 0.5, -h * 0.5 ) );

            // TODO: implement flange slope
            if( flange_edge_radius > this->m_parameters->m_epsilon ) {
                this->m_geomUtils->AppendToLoop( &result,
                                                 this->m_geomUtils->BuildCircle( flange_edge_radius, 0, M_PI_2, this->m_parameters->m_numVerticesPerCircle,
                                                                                 width * 0.5 - flange_edge_radius, -h * 0.5 + tf - flange_edge_radius ) );
            } else {
                result.push_back( AVector::New( width * 0.5, -h * 0.5 + tf ) );
            }
            if( fillet_radius > this->m_parameters->m_epsilon ) {
                this->m_geomUtils->AppendToLoop( &result,
                                                 this->m_geomUtils->BuildCircle( fillet_radius, 3 * M_PI_2, -M_PI_2, this->m_parameters->m_numVerticesPerCircle,
                                                                                 tw * 0.5 + fillet_radius, -h * 0.5 + tf + fillet_radius ) );
            } else {
                result.push_back( AVector::New( tw * 0.5, ( -h * 0.5 + tf ) ) );
            }

            const auto asym_I_profile = dynamic_pointer_cast<IfcAsymmetricIShapeProfileDef>( i_shape );
            if( asym_I_profile && asym_I_profile->m_TopFlangeWidth ) {
                const auto width_top = asym_I_profile->m_TopFlangeWidth->m_value;
                double tfTop = tf;

                if( asym_I_profile->m_TopFlangeThickness ) {
                    tfTop = asym_I_profile->m_TopFlangeThickness->m_value;
                }
                double rTop = fillet_radius;
                if( asym_I_profile->m_TopFlangeFilletRadius ) {
                    rTop = asym_I_profile->m_TopFlangeFilletRadius->m_value;
                }

                if( rTop > this->m_parameters->m_epsilon ) {
                    this->AddArc( &result, rTop, M_PI, -M_PI_2, tw * 0.5 + rTop, h * 0.5 - tfTop - rTop );
                } else {
                    result.push_back( AVector::New( tw * 0.5, ( h * 0.5 - tfTop ) ) );
                }
                result.push_back( AVector::New( width_top * 0.5, ( h * 0.5 - tfTop ) ) );
                result.push_back( AVector::New( width_top * 0.5, h * 0.5 ) );
            } else {
                this->MirrorCopyPathReverse( &result, false, true );
            }

            this->MirrorCopyPathReverse( &result, true, false );
            if( !result.empty() ) {
                result.push_back( result[ 0 ] );
            }
            return result;
        }

        const auto l_shape = dynamic_pointer_cast<IfcLShapeProfileDef>( profile );
        if( l_shape && l_shape->m_Depth && l_shape->m_Thickness ) {
            TPath result;
            const auto h = l_shape->m_Depth->m_value;
            double w = h;

            if( l_shape->m_Width ) {
                w = l_shape->m_Width->m_value;
            }

            auto t = l_shape->m_Thickness->m_value;

            double fillet_radius = 0;
            if( l_shape->m_FilletRadius ) {
                fillet_radius = l_shape->m_FilletRadius->m_value;
            }

            double edge_radius = 0;
            if( l_shape->m_EdgeRadius ) {
                edge_radius = l_shape->m_EdgeRadius->m_value;
            }

            double leg_slope = 0;
            if( l_shape->m_LegSlope ) {
                leg_slope = l_shape->m_LegSlope->m_value * this->m_parameters->m_angleFactor;
            }

            result.push_back( AVector::New( -w * 0.5, -h * 0.5 ) );
            result.push_back( AVector::New( w * 0.5, -h * 0.5 ) );

            if( edge_radius > this->m_parameters->m_epsilon ) {
                double y_edge_radius_start = -h * 0.5 + t - edge_radius;
                this->AddArc( &result, edge_radius, 0, M_PI_2 - leg_slope, w * 0.5 - edge_radius, y_edge_radius_start );
            } else {
                result.push_back( AVector::New( w * 0.5, ( -h * 0.5 + t ) ) );
            }

            const double s = sin( leg_slope );
            const double c = cos( leg_slope );
            const double z1 = ( -s * ( ( c - s ) * ( fillet_radius + edge_radius + t ) - c * w + s * h ) ) / ( 2 * c * c - 1 );
            const double z2 = ( -s * ( ( c - s ) * ( fillet_radius + edge_radius + t ) - c * h + s * w ) ) / ( 2 * c * c - 1 );
            if( fillet_radius > this->m_parameters->m_epsilon ) {
                this->AddArc( &result, fillet_radius, 3 * M_PI_2 - leg_slope, -M_PI_2 + 2 * leg_slope, -w * 0.5 + t + z2 + fillet_radius,
                              -h * 0.5 + t + z1 + fillet_radius );
            } else {
                result.push_back( AVector::New( ( -w * 0.5 + t + z2 ), ( -h * 0.5 + t + z1 ) ) );
            }

            if( edge_radius > this->m_parameters->m_epsilon ) {
                this->AddArc( &result, edge_radius, leg_slope, M_PI_2 - leg_slope, -w * 0.5 + t - edge_radius, h * 0.5 - edge_radius );
            } else {
                result.push_back( AVector::New( ( -w * 0.5 + t ), h * 0.5 ) );
            }

            result.push_back( AVector::New( -w * 0.5, h * 0.5 ) );
            if( !result.empty() ) {
                result.push_back( result[ 0 ] );
            }
            return result;
        }

        const auto u_shape = dynamic_pointer_cast<IfcUShapeProfileDef>( profile );
        if( u_shape && u_shape->m_Depth && u_shape->m_FlangeWidth && u_shape->m_WebThickness && u_shape->m_FlangeThickness ) {
            TPath result;
            const auto height = u_shape->m_Depth->m_value;
            const auto width = u_shape->m_FlangeWidth->m_value;
            const auto tw = u_shape->m_WebThickness->m_value;
            const auto tf = u_shape->m_FlangeThickness->m_value;
            double fillet_radius = 0;
            if( u_shape->m_FilletRadius ) {
                fillet_radius = u_shape->m_FilletRadius->m_value;
            }
            double edge_radius = 0;
            if( u_shape->m_EdgeRadius ) {
                edge_radius = u_shape->m_EdgeRadius->m_value;
            }
            double fs = 0;
            if( u_shape->m_FlangeSlope ) {
                fs = u_shape->m_FlangeSlope->m_value * this->m_parameters->m_angleFactor;
            }

            result.push_back( AVector::New( -width * 0.5, -height * 0.5 ) );
            result.push_back( AVector::New( width * 0.5, -height * 0.5 ) );

            double z = tan( fs ) * ( width * 0.5 - edge_radius );
            if( edge_radius > this->m_parameters->m_epsilon ) {
                this->AddArc( &result, edge_radius, 0, M_PI_2 - fs, width * 0.5 - edge_radius, -height * 0.5 + tf - z - edge_radius );
            } else {
                result.push_back( AVector::New( width * 0.5, ( -height * 0.5 + tf - z ) ) );
            }

            z = tan( fs ) * ( width * 0.5 - tw - fillet_radius );
            if( fillet_radius > this->m_parameters->m_epsilon ) {
                this->AddArc( &result, fillet_radius, 3 * M_PI_2 - fs, -M_PI_2 + fs, -width * 0.5 + tw + fillet_radius,
                              -height * 0.5 + tf + z + fillet_radius );
            } else {
                result.push_back( AVector::New( ( -width * 0.5 + tw ), ( -height * 0.5 + tf + z ) ) );
            }

            this->MirrorCopyPathReverse( &result, false, true );
            if( !result.empty() ) {
                result.push_back( result[ 0 ] );
            }
            return result;
        }

        const auto c_shape = dynamic_pointer_cast<IfcCShapeProfileDef>( profile );
        if( c_shape && c_shape->m_Depth && c_shape->m_Width && c_shape->m_Girth && c_shape->m_WallThickness ) {
            TPath result;
            const auto h = c_shape->m_Depth->m_value;
            const auto width = c_shape->m_Width->m_value;
            const auto g = c_shape->m_Girth->m_value;
            const auto t = c_shape->m_WallThickness->m_value;
            double fillet_radius = 0;
            if( c_shape->m_InternalFilletRadius ) {
                fillet_radius = c_shape->m_InternalFilletRadius->m_value;
            }

            if( fillet_radius > this->m_parameters->m_epsilon ) {
                this->AddArc( &result, fillet_radius + t, M_PI, M_PI_2, -width * 0.5 + t + fillet_radius, -h * 0.5 + t + fillet_radius );
            } else {
                result.push_back( AVector::New( -width * 0.5, -h * 0.5 ) );
            }

            if( fillet_radius != 0 ) {
                this->AddArc( &result, fillet_radius + t, 3 * M_PI_2, M_PI_2, width * 0.5 - t - fillet_radius, -h * 0.5 + t + fillet_radius );
            } else {
                result.push_back( AVector::New( width * 0.5, -h * 0.5 ) );
            }

            result.push_back( AVector::New( width * 0.5, ( -h * 0.5 + g ) ) );
            result.push_back( AVector::New( ( width * 0.5 - t ), ( -h * 0.5 + g ) ) );

            if( fillet_radius > this->m_parameters->m_epsilon ) {
                this->AddArc( &result, fillet_radius, 0, -M_PI_2, width * 0.5 - t - fillet_radius, -h * 0.5 + t + fillet_radius );
            } else {
                result.push_back( AVector::New( ( width * 0.5 - t ), ( -h * 0.5 + t ) ) );
            }

            if( fillet_radius > this->m_parameters->m_epsilon ) {
                this->AddArc( &result, fillet_radius, 3 * M_PI_2, -M_PI_2, -width * 0.5 + t + fillet_radius, -h * 0.5 + t + fillet_radius );
            } else {
                result.push_back( AVector::New( ( -width * 0.5 + t ), ( -h * 0.5 + t ) ) );
            }
            this->MirrorCopyPathReverse( &result, false, true );
            if( !result.empty() ) {
                result.push_back( result[ 0 ] );
            }
            return result;
        }

        const auto z_shape = dynamic_pointer_cast<IfcZShapeProfileDef>( profile );
        if( z_shape && z_shape->m_Depth && z_shape->m_FlangeWidth && z_shape->m_WebThickness && z_shape->m_FlangeThickness ) {
            TPath result;
            const auto h = z_shape->m_Depth->m_value;
            const auto width = z_shape->m_FlangeWidth->m_value;
            const auto tw = z_shape->m_WebThickness->m_value;
            const auto tf = z_shape->m_FlangeThickness->m_value;
            double fillet_radius = 0;
            if( z_shape->m_FilletRadius ) {
                fillet_radius = z_shape->m_FilletRadius->m_value;
            }

            double edge_radius = 0;
            if( z_shape->m_EdgeRadius ) {
                edge_radius = z_shape->m_EdgeRadius->m_value;
            }

            result.push_back( AVector::New( ( -tw * 0.5 ), -h * 0.5 ) );
            result.push_back( AVector::New( ( width - tw * 0.5 ), -h * 0.5 ) );

            if( edge_radius > this->m_parameters->m_epsilon ) {
                this->AddArc( &result, edge_radius, 0, M_PI_2, width - tw * 0.5 - edge_radius, -h * 0.5 + tf - edge_radius );
            } else {
                result.push_back( AVector::New( ( width - tw * 0.5 ), ( -h * 0.5 + tf ) ) );
            }

            if( fillet_radius > this->m_parameters->m_epsilon ) {
                this->AddArc( &result, fillet_radius, 3 * M_PI_2, -M_PI_2, tw * 0.5 + fillet_radius, -h * 0.5 + tf + fillet_radius );
            } else {
                result.push_back( AVector::New( ( tw * 0.5 ), ( -h * 0.5 + tf ) ) );
            }

            this->MirrorCopyPath( &result, true, true );
            if( !result.empty() ) {
                result.push_back( result[ 0 ] );
            }
            return result;
        }

        shared_ptr<IfcTShapeProfileDef> t_shape = dynamic_pointer_cast<IfcTShapeProfileDef>( profile );
        if( t_shape ) {
            TPath result;
            const auto h = t_shape->m_Depth->m_value;
            const auto width = t_shape->m_FlangeWidth->m_value;
            const auto tw = t_shape->m_WebThickness->m_value * 0.5;
            const auto tf = t_shape->m_FlangeThickness->m_value;

            double fillet_radius = 0;
            if( t_shape->m_FilletRadius ) {
                fillet_radius = t_shape->m_FilletRadius->m_value;
            }

            double flange_edge_radius = 0;
            if( t_shape->m_FlangeEdgeRadius ) {
                flange_edge_radius = t_shape->m_FlangeEdgeRadius->m_value;
            }

            double web_edge_radius = 0;
            if( t_shape->m_WebEdgeRadius ) {
                web_edge_radius = t_shape->m_WebEdgeRadius->m_value;
            }
            double flange_slope = 0;

            if( t_shape->m_FlangeSlope ) {
                flange_slope = t_shape->m_FlangeSlope->m_value * this->m_parameters->m_angleFactor;
            }

            double web_slope = 0;
            if( t_shape->m_WebSlope ) {
                web_slope = t_shape->m_WebSlope->m_value * this->m_parameters->m_angleFactor;
            }

            result.push_back( AVector::New( -width * 0.5, h * 0.5 ) );

            const double zf = tan( flange_slope ) * ( width * 0.25 - flange_edge_radius );
            const double zw = tan( web_slope ) * ( h * 0.5 - web_edge_radius );
            if( flange_edge_radius > this->m_parameters->m_epsilon ) {
                this->AddArc( &result, flange_edge_radius, M_PI, M_PI_2 - flange_slope, -width * 0.5 + flange_edge_radius,
                              h * 0.5 - tf + zf + flange_edge_radius );
            } else {
                result.push_back( AVector::New( -width * 0.5, ( h * 0.5 - tf + zf ) ) );
            }

            const double cf = cos( flange_slope );
            const double sf = sin( flange_slope );
            const double cw = cos( web_slope );
            const double sw = sin( web_slope );
            const double z1 =
                ( sf *
                  ( ( width - 2.0 * ( fillet_radius + flange_edge_radius + tw - zw ) ) * cw - 2.0 * ( h - web_edge_radius - fillet_radius - tf + zf ) * sw ) ) /
                ( 2.0 * ( cf * cw - sf * sw ) );
            const double z2 = tan( web_slope ) * ( h - web_edge_radius - fillet_radius - z1 - tf + zf );
            if( fillet_radius > this->m_parameters->m_epsilon ) {
                this->AddArc( &result, fillet_radius, M_PI_2 - flange_slope, -M_PI_2 + flange_slope + web_slope, -tw + zw - z2 - fillet_radius,
                              h * 0.5 - tf + zf - z1 - fillet_radius );
            } else {
                result.push_back( AVector::New( ( -tw + zw - z2 ), ( h * 0.5 - tf + zf - z1 ) ) );
            }

            if( web_edge_radius > this->m_parameters->m_epsilon ) {
                double x_center = -tw + zw + web_edge_radius;
                if( x_center > 0 ) {
                    x_center = 0;
                }
                this->AddArc( &result, web_edge_radius, M_PI + web_slope, M_PI_2 - web_slope, x_center, -h * 0.5 + web_edge_radius );
                while( result.size() > 0 ) {
                    if( result.back().x < 0 ) {
                        break;
                    }
                    result.pop_back();
                }
                result.push_back( AVector::New( 0, -h * 0.5 ) );
            } else {
                result.push_back( AVector::New( ( -tw + zw ), -h * 0.5 ) );
            }

            // mirror vertically along y-Axis
            this->MirrorCopyPathReverse( &result, true, false );
            if( !result.empty() ) {
                result.push_back( result[ 0 ] );
            }
            return result;
        }


        // TODO: Log error (unknown profile)
        return {};
    }

    void AddArc( TPath* path, double radius, double startAngle, double openningAngle, double x, double y ) {
        this->m_geomUtils->AppendToLoop(
            path, this->m_geomUtils->BuildCircle( radius, startAngle, openningAngle, this->m_parameters->m_numVerticesPerCircle, x, y ) );
    }

    static void MirrorCopyPath( TPath* coords, bool mirror_on_y_axis, bool mirror_on_x_axis ) {
        int points_count = coords->size();
        double x, y;
        for( int i = 0; i < points_count; ++i ) {
            const auto& p = ( *coords )[ i ];
            if( mirror_on_y_axis ) {
                x = -p.x;
            } else {
                x = p.x;
            }
            if( mirror_on_x_axis ) {
                y = -p.y;
            } else {
                y = p.y;
            }
            coords->push_back( AVector::New( x, y ) );
        }
    }

    static void MirrorCopyPathReverse( TPath* path, bool mirrorOnY, bool mirrorOnX ) {
        int points_count = path->size();
        double x, y;
        for( int i = points_count - 1; i >= 0; --i ) {
            const auto& p = ( *path )[ i ];
            if( mirrorOnY ) {
                x = -p.x;
            } else {
                x = p.x;
            }
            if( mirrorOnX ) {
                y = -p.y;
            } else {
                y = p.y;
            }

            path->push_back( AVector::New( x, y ) );
        }
    }
};
}