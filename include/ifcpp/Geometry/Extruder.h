#pragma once

#include <memory>
#include <vector>

#include <ifcpp/Geometry/CAdapter.h>
#include <ifcpp/Geometry/GeomUtils.h>
#include <ifcpp/Geometry/Matrix.h>
#include <ifcpp/Geometry/Parameters.h>
#include <ifcpp/Geometry/VectorAdapter.h>

namespace ifcpp {

template<CVector TVector>
class Extruder {
    using TLoop = std::vector<TVector>;
    using AVector = VectorAdapter<TVector>;
    using TMatrix = Matrix<TVector>;

    std::shared_ptr<GeomUtils<TVector>> m_geomUtils;
    std::shared_ptr<Parameters> m_parameters;

public:
    Extruder( const std::shared_ptr<GeomUtils<TVector>>& geomUtils, const std::shared_ptr<Parameters>& parameters )
        : m_geomUtils( geomUtils )
        , m_parameters( parameters ) {
    }

    std::vector<TLoop> Extrude( TLoop profile, TVector extrusion, bool asClosed = true ) {
        if( profile.empty() ) {
            return {};
        }
        // TODO: It is not important to simplify profile heres
        if( asClosed ) {
            profile = this->m_geomUtils->SimplifyLoop( profile );
            profile.push_back( profile[ 0 ] );
        } else {
            profile = this->m_geomUtils->SimplifyCurve( profile );
        }
        std::vector<TLoop> result;

        // TODO: Rework
        if( AVector::Dot( extrusion, this->m_geomUtils->ComputePolygonNormal( profile ) ) < 0 ) {
            for( auto& point: profile ) {
                point = point + extrusion;
            }
            extrusion = -extrusion;
        }

        TLoop back = profile;
        TLoop front = profile;
        for( auto& point: front ) {
            point = point + extrusion;
        }

        const auto connection = this->ConnectLoops( back, front );
        std::copy( std::begin( connection ), std::end( connection ), std::back_inserter( result ) );

        if( asClosed ) {
            std::reverse( std::begin( back ), std::end( back ) );
            result.push_back( this->m_geomUtils->SimplifyLoop( back ) );
            result.push_back( this->m_geomUtils->SimplifyLoop( front ) );
        }

        return result;
    }
    std::vector<TLoop> Sweep( TLoop profile, const std::vector<TVector>& sweepPoints, bool asClosed = true ) {
        if( profile.empty() ) {
            return {};
        }
        // TODO: It is not important to simplify profile here
        if( asClosed ) {
            profile = this->m_geomUtils->SimplifyLoop( profile );
            profile.push_back( profile[ 0 ] );
        } else {
            profile = this->m_geomUtils->SimplifyCurve( profile );
        }

        if( sweepPoints.size() < 2 ) {
            return {};
        }

        std::vector<TVector> rDirs, fDirs, uDirs;
        std::vector<float> k;

        fDirs.push_back( AVector::Normalized( sweepPoints[ 1 ] - sweepPoints[ 0 ] ) );
        k.push_back( 1.0f );
        for( int i = 1; i < sweepPoints.size() - 1; i++ ) {
            fDirs.push_back( AVector::Normalized( AVector::Normalized( sweepPoints[ i ] - sweepPoints[ i - 1 ] ) +
                                                  AVector::Normalized( sweepPoints[ i + 1 ] - sweepPoints[ i ] ) ) );
            float angle = std::acosf( AVector::Dot( AVector::Normalized( sweepPoints[ i ] - sweepPoints[ i - 1 ] ),
                                                    AVector::Normalized( sweepPoints[ i ] - sweepPoints[ i + 1 ] ) ) );
            k.push_back( 1.0f / std::sinf( angle / 2.0f ) );
        }
        fDirs.push_back( AVector::Normalized( sweepPoints[ sweepPoints.size() - 1 ] - sweepPoints[ sweepPoints.size() - 2 ] ) );
        k.push_back( 1.0f );

        auto up = AVector::New( 0, 0, 1 );
        for( int i = 0; i < sweepPoints.size(); i++ ) {
            auto [ r, f, u ] = this->CalculateBasis( fDirs[ i ], up );
            up = u;
            rDirs.push_back( r * k[ i ] );
            uDirs.push_back( u * k[ i ] );
        }

        std::vector<TLoop> result;

        result.push_back( this->m_geomUtils->SimplifyLoop( UnProjectLoop( profile, sweepPoints[ 0 ], rDirs[ 0 ], uDirs[ 0 ] ) ) );
        for( int i = 1; i < sweepPoints.size(); i++ ) {
            const auto unProjected = this->UnProjectLoop( profile, sweepPoints[ i ], rDirs[ i ], uDirs[ i ] );
            const auto connection = this->ConnectLoops( result[ i - 1 ], unProjected );
            std::copy( std::begin( connection ), std::end( connection ), std::back_inserter( result ) );
        }
        result.push_back( this->m_geomUtils->SimplifyLoop(
            UnProjectLoop( profile, sweepPoints[ sweepPoints.size() - 1 ], rDirs[ sweepPoints.size() - 1 ], uDirs[ sweepPoints.size() - 1 ] ) ) );
        std::reverse( std::begin( result[ 0 ] ), std::end( result[ 0 ] ) );

        if( !asClosed ) {
            result.pop_back();
            result.erase( std::begin( result ) );
        }

        return result;
    }
    std::vector<TLoop> Revolve( TLoop profile, const TVector& axisLocation, const TVector& axisDirection, float revolveAngle, bool asClosed = true ) {
        if( profile.empty() ) {
            return {};
        }
        // TODO: It is not important to simplify profile here
        if( asClosed ) {
            profile = this->m_geomUtils->SimplifyLoop( profile );
            profile.push_back( profile[ 0 ] );
        } else {
            profile = this->m_geomUtils->SimplifyCurve( profile );
        }

        std::vector<TLoop> result;
        for( auto& p: profile ) {
            p = p - axisLocation;
        }
        // TODO: Calculate n with profile sizes

        result.push_back( this->m_geomUtils->SimplifyLoop( profile ) );
        int n = this->m_parameters->m_numVerticesPerCircle;
        float deltaAngle = revolveAngle / (float)( n - 1 );
        for( int i = 1; i < n; i++ ) {
            auto m = TMatrix::GetRotation( deltaAngle * (float)i, axisDirection );
            auto p = m.GetTransformedLoop( profile );
            const auto connection = this->ConnectLoops( result[ i - 1 ], p );
            std::copy( std::begin( connection ), std::end( connection ), std::back_inserter( result ) );
            if( i == n - 2 && asClosed ) {
                result.push_back( this->m_geomUtils->SimplifyLoop( p ) );
            }
        }
        std::reverse( std::begin( result[ 0 ] ), std::end( result[ 0 ] ) );

        if( !asClosed ) {
            result.pop_back();
            result.erase( std::begin( result ) );
        }

        for( auto& l: result ) {
            for( auto& p: l ) {
                p = p + axisLocation;
            }
        }

        return result;
    }

private:
    //         right,   forward, up
    std::tuple<TVector, TVector, TVector> CalculateBasis( const TVector& forward, const TVector& up ) {
        auto r = AVector::Cross( forward, up );
        if( AVector::Len2( r ) < this->m_parameters->m_epsilon ) {
            r = AVector::Cross( forward, AVector::New( 0, 0, 1 ) );
            if( AVector::Len2( r ) < this->m_parameters->m_epsilon ) {
                r = AVector::Cross( forward, AVector::New( -1, 0, 0 ) );
            }
        }
        auto u = AVector::Cross( r, forward );
        return { AVector::Normalized( r ), AVector::Normalized( forward ), AVector::Normalized( u ) };
    }

    TLoop UnProjectLoop( const TLoop& loop, const TVector& origin, const TVector& r, const TVector& u ) {
        TLoop result;
        for( const auto& point: loop ) {
            result.push_back( origin + r * point.x + u * point.y );
        }
        return result;
    }

    std::vector<TLoop> ConnectLoops( TLoop back, TLoop front ) {
        if( front.size() != back.size() ) {
            // TODO: Log error
            return {};
        }
        // FIXME: Points order
        std::vector<TLoop> result;
        for( int i = 1; i < front.size(); i++ ) {
            result.push_back( { back[ i - 1 ], back[ i ], front[ i ], front[ i - 1 ] } );
        }
        return result;
    }
};

}
