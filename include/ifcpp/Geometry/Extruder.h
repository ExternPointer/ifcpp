#pragma once

#include <memory>
#include <vector>

#include <ifcpp/Geometry/CAdapter.h>
#include <ifcpp/Geometry/Matrix.h>
#include <ifcpp/Geometry/Parameters.h>
#include <ifcpp/Geometry/VectorAdapter.h>

namespace ifcpp {

template<CVector TVector>
class Extruder {
    using TLoop = std::vector<TVector>;
    using AVector = VectorAdapter<TVector>;
    using TMatrix = Matrix<TVector>;

    std::shared_ptr<Parameters> m_parameters;

public:
    explicit Extruder( const std::shared_ptr<Parameters>& parameters )
        : m_parameters( parameters ) {
    }

    std::vector<TLoop> Extrude( const TLoop& profile, const TVector& extrusion ) {
        std::vector<TLoop> result;

        TLoop back = profile;
        TLoop front = profile;
        for( auto& point: front ) {
            front = front + extrusion;
        }

        const auto connection = this->ConnectLoops( back, front );
        std::back_inserter( std::begin( connection ), std::end( connection ), result );

        std::reverse( std::begin( back ), std::end( back ) );
        result.push_back( back );
        result.push_back( front );

        return result;
    }
    std::vector<TLoop> Sweep( const TLoop& profile, const std::vector<TVector>& sweepPoints ) {
        if( sweepPoints.size() <= 1 ) {
            return {};
        }

        std::vector<TVector> rDirs, fDirs, uDirs;
        std::vector<float> k;

        fDirs.push_back( AVector::Normalized( sweepPoints[ 1 ] - sweepPoints[ 0 ] ) );
        k.push_back( 1.0f );
        for( int i = 1; i < sweepPoints.size() - 1; i++ ) {
            fDirs.push_back( AVector::Normalized( AVector::Normalized( sweepPoints[ i ] - sweepPoints[ i - 1 ] ) +
                                                  AVector::Normalized( sweepPoints[ i + 1 ] - sweepPoints[ i ] ) ) );
            float angle = std::acosf( AVector::DotProduct( AVector::Normalized( sweepPoints[ i ] - sweepPoints[ i - 1 ] ),
                                                           AVector::Normalized( sweepPoints[ i ] - sweepPoints[ i + 1 ] ) ) );
            k.push_back( 1.0f / std::sinf( angle / 2.0f ) );
        }
        fDirs.push_back( AVector::Normalized( sweepPoints[ sweepPoints.size() - 1 ] - sweepPoints[ sweepPoints.size() - 2 ] ) );
        k.push_back( 1.0f );

        auto up = AVector::New( 0, 0, 1 );
        for( int i = 0; i < sweepPoints.size(); i++ ) {
            auto [ r, f, u ] = this->CalculateBasis( fDirs, up );
            up = u;
            rDirs.push_back( r * k[ i ] );
            uDirs.push_back( u * k[ i ] );
        }

        std::vector<TLoop> result;

        result.push_back( UnProjectLoop( profile, sweepPoints[ 0 ], rDirs[ 0 ], uDirs[ 0 ] ) );
        for( int i = 1; i < sweepPoints.size(); i++ ) {
            const auto unProjected = this->UnProjectLoop( profile, sweepPoints[ i ], rDirs[ i ], uDirs[ i ] );
            const auto connection = this->ConnectLoops( result[ i - 1 ], unProjected );
            std::back_inserter( std::begin( connection ), std::end( connection ), result );
        }
        result.push_back( UnProjectLoop( profile, sweepPoints[ sweepPoints.size() - 1 ], rDirs[ sweepPoints.size() - 1 ], uDirs[ sweepPoints.size() - 1 ] ) );
        std::reverse( std::begin( result[ 0 ] ), std::end( result[ 0 ] ) );

        return result;
    }
    std::vector<TLoop> Revolve( TLoop profile, const TVector& axisLocation, const TVector& axisDirection, float revolveAngle ) {
        std::vector<TLoop> result;
        for( auto& p: profile ) {
            p = p - axisLocation;
        }
        // TODO: Calculate n with profile sizes

        result.push_back( profile );
        int n = this->m_parameters->m_NumVerticesPerCircle;
        float deltaAngle = revolveAngle / (float)( n - 1 );
        for( int i = 1; i < n; i++ ) {
            auto m = TMatrix::GetRotation( deltaAngle * (float)i, axisDirection );
            auto p = m.GetTransformedLoop( profile );
            const auto connection = this->ConnectLoops( result[ i - 1 ], p );
            std::back_inserter( std::begin( connection ), std::end( connection ), result );
            if( i == n - 2 ) {
                result.push_back( p );
            }
        }
        std::reverse( std::begin( result[ 0 ] ), std::end( result[ 0 ] ) );

        for( const auto& l: result ) {
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
            r = AVector::Cross( forward, AVector( 0, 0, 1 ) );
            if( AVector::Len2( r ) < this->m_parameters->m_epsilon ) {
                r = AVector::Cross( forward, AVector( -1, 0, 0 ) );
            }
        }
        auto u = AVector::Cross( r, forward );
        return { AVector::Normalized( r ), AVector::Normalized( forward ), AVector::Normalized( u ) };
    }

    TLoop UnProjectLoop( const std::vector<TLoop>& loop, const TVector& origin, const TVector& r, const TVector& u ) {
        TLoop result;
        for( const auto& point: loop ) {
            result.push_back( origin + r * point.x + u * point.y );
        }
        return result;
    }

    std::vector<TLoop> ConnectLoops( TLoop front, TLoop back ) {
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
