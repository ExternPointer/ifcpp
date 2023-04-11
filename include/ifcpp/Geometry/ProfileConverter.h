#pragma once

#include <vector>

#include "ifcpp/Geometry/CAdapter.h"
#include "ifcpp/Geometry/CurveConverter.h"
#include "ifcpp/Geometry/GeomUtils.h"
#include "ifcpp/Geometry/Matrix.h"
#include "ifcpp/Geometry/Parameters.h"
#include "ifcpp/Geometry/PrimitivesConverter.h"
#include "ifcpp/Geometry/Vector.h"

#include "ifcpp/Ifc/IfcArbitraryClosedProfileDef.h"
#include "ifcpp/Ifc/IfcArbitraryOpenProfileDef.h"
#include "ifcpp/Ifc/IfcArbitraryProfileDefWithVoids.h"
#include "ifcpp/Ifc/IfcCenterLineProfileDef.h"
#include "ifcpp/Ifc/IfcCompositeProfileDef.h"
#include "ifcpp/Ifc/IfcDerivedProfileDef.h"
#include "ifcpp/Ifc/IfcParameterizedProfileDef.h"
#include "ifcpp/Ifc/IfcProfileDef.h"


namespace ifcpp {
using namespace IFC4X3;

template<CVector TVector>
class ProfileConverter {
    using TPath = std::vector<TVector>;
    using AVector = VectorAdapter<TVector>;

    std::shared_ptr<CurveConverter<TVector>> m_curveConverter;
    std::shared_ptr<PrimitivesConverter<TVector>> m_primitivesConverter;
    std::shared_ptr<GeomUtils<TVector>> m_geomUtils;
    std::shared_ptr<Parameters> m_parameters;


public:
    ProfileConverter( const std::shared_ptr<CurveConverter<TVector>>& curveConverter, const std::shared_ptr<GeomUtils<TVector>> geomUtils,
                      const std::shared_ptr<PrimitivesConverter<TVector>>& primitivesConverter, const std::shared_ptr<Parameters>& parameters )
        : m_curveConverter( curveConverter )
        , m_geomUtils( geomUtils )
        , m_primitivesConverter( primitivesConverter )
        , m_parameters( parameters ) {
    }

    std::vector<TVector> ConvertProfile( const std::shared_ptr<IfcProfileDef>& profile ) {
        // ENTITY IfcProfileDef SUPERTYPE OF(ONEOF(IfcArbitraryClosedProfileDef, IfcArbitraryOpenProfileDef, IfcCompositeProfileDef, IfcDerivedProfileDef,
        // IfcParameterizedProfileDef));

        const auto parameterized = dynamic_pointer_cast<IfcParameterizedProfileDef>( profile );
        if( parameterized ) {
            return this->ConvertParameterizedProfileDef( parameterized );
        }

        const auto arbitraryClosed = dynamic_pointer_cast<IfcArbitraryClosedProfileDef>( profile );
        if( arbitraryClosed ) {
            return this->ConvertArbitraryClosedProfileDef( arbitraryClosed );
        }

        const auto arbitraryOpen = dynamic_pointer_cast<IfcArbitraryOpenProfileDef>( profile );
        if( arbitraryOpen ) {
            return this->ConvertArbitraryOpenProfileDef( arbitraryOpen );
        }

        const auto composite = dynamic_pointer_cast<IfcCompositeProfileDef>( profile );
        if( composite ) {
            return this->ConvertCompositeProfileDef( composite );
        }

        const auto derived = dynamic_pointer_cast<IfcDerivedProfileDef>( profile );
        if( derived ) {
            return this->ConvertDerivedProfileDef( derived );
        }

        // TODO: Log error
    }

private:
    std::vector<TVector> ConvertArbitraryClosedProfileDef( const shared_ptr<IfcArbitraryClosedProfileDef>& profile ) {
        if( !profile || !profile->m_OuterCurve ) {
            return {};
        }

        const auto outer = this->m_geomUtils->SimplifyLoop( this->m_curveConverter->ConvertCurve( profile->m_OuterCurve ) );
        std::vector<TPath> holes;

        shared_ptr<IfcArbitraryProfileDefWithVoids> profileWithVoids = dynamic_pointer_cast<IfcArbitraryProfileDefWithVoids>( profile );
        if( profileWithVoids ) {
            for( const auto& inner: profileWithVoids->m_InnerCurves ) {
                holes.push_back( this->m_geomUtils->SimplifyLoop( this->m_curveConverter->ConvertCurve( inner ) ) );
            }
        }

        // TODO: Incorporate holes

        return outer;
    }

    TPath ConvertArbitraryOpenProfileDef( const shared_ptr<IfcArbitraryOpenProfileDef>& profile ) {
        shared_ptr<IfcCenterLineProfileDef> centerLineProfileDef = dynamic_pointer_cast<IfcCenterLineProfileDef>( profile );
        if( centerLineProfileDef && centerLineProfileDef->m_Thickness ) {
            const auto curve = this->m_geomUtils->SimplifyLoop( this->m_curveConverter->ConvertCurve( centerLineProfileDef->m_Curve ) );
            if( curve.size() <= 1 ) {
                return {};
            }
            std::vector<TVector> normals;
            normals.push_back( this->m_geomUtils->Normal2d( curve[ 0 ], curve[ 1 ] ) );
            for( int i = 1; i < curve.size() - 1; i++ ) {
                const auto t = this->m_geomUtils->Normal2d( curve[ i - 1 ], curve[ i ] ) + this->m_geomUtils->Normal2d( curve[ i ], curve[ i + 1 ] );
                const auto l = AVector::Len( t );
                normals.push_back( AVector::Normalized( t ) * ( 1.0f / sinf( ( acosf( l / 2 ) + M_PI ) / 2.0f ) ) );
            }
            normals.push_back( this->m_geomUtils->Normal2d( curve[ curve.size() - 2 ], curve[ curve.size() - 1 ] ) );
            std::vector<TVector> left, right;
            for( int i = 0; i < curve.size(); i++ ) {
                left.push_back( curve[ i ] - normals[ i ] * (float)centerLineProfileDef->m_Thickness->m_value * 0.5f );
                right.push_back( curve[ i ] + normals[ i ] * (float)centerLineProfileDef->m_Thickness->m_value * 0.5f );
            }
            auto result = left;
            std::back_inserter( std::rbegin( right ), std::rend( right ), result );
            return result;
        }
        return this->m_geomUtils->SimplifyLoop( this->m_curveConverter->ConvertCurve( profile->m_Curve ) );
    }

    TPath ConvertCompositeProfileDef( const shared_ptr<IfcCompositeProfileDef>& profile ) {
        std::vector<TPath> paths;
        for( const auto& p: profile->m_Profiles ) {
            paths.push_back( this->ConvertProfile( p ) );
        }
        // TODO: Merge profiles
        return {};
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
        //IfcParameterizedProfileDef ABSTRACT SUPERTYPE OF (ONEOF
        //	(IfcCShapeProfileDef, IfcCircleProfileDef, IfcEllipseProfileDef, IfcIShapeProfileDef, IfcLShapeProfileDef,
        //	IfcRectangleProfileDef, IfcTShapeProfileDef, IfcTrapeziumProfileDef, IfcUShapeProfileDef, IfcZShapeProfileDef))


    }

};

}