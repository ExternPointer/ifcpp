#pragma once


#include "ifcpp/Geometry/CAdapter.h"
#include "ifcpp/Geometry/Vector.h"
#include "ifcpp/Geometry/Matrix.h"


namespace ifcpp {

template<CVector TVector>
class GeomUtils {
    using AVector = VectorAdapter<TVector>;
    using TMatrix = Matrix<TVector>;

public:
    TVector Normal2d( const TVector& v ) {
        return AVector::Normalized( AVector::New( v.y, -v.x, 0 ) );
    }
    TVector Normal2d( const TVector& v1, const TVector& v2 ) {
        return Normal2d( v2 - v1 );
    }
    std::vector<TVector> SimplifyLoop( const std::vector<TVector> loop ) {
        std::vector<TVector> result;
        result.reserve( loop.size() );
        if( loop.empty() ) {
            return result;
        }
        result.push_back( loop[ 0 ] );
        for( const auto& point: loop ) {
            if( !AVector::IsNearlyEqual( point, result[ result.size() - 1 ] ) ) {
                result.push_back( point );
            }
        }
        if( result.size() > 2 && AVector::IsNearlyEqual( result[ 0 ], result[ result.size() - 1 ] ) ) {
            result.pop_back();
        }
        // TODO: Remove collinear edges
        return result;
    }
    std::vector<TVector> BuildEllipse( float radius1, float radius2, float startAngle, float openingAngle, int verticesCount, TVector center ) const {
        std::vector<TVector> points;
        float angle = startAngle;
        float delta = openingAngle / (float)( verticesCount - 1 );
        for( int i = 0; i < verticesCount; ++i ) {
            points.push_back( AVector::New( radius1 * cos( angle ), radius2 * sin( angle ) ) + center );
            angle += delta;
        }
        return points;
    }
    std::vector<TVector> BuildCircle( float radius, float startAngle, float openingAngle, int verticesCount, TVector center ) const {
        return this->BuildEllipse( radius, radius, startAngle, openingAngle, verticesCount, center );
    }
    // p0, p1, p2 - points on arc
    std::vector<TVector> BuildArc( TVector p0, TVector p1, TVector p2 ) const {
        std::vector<TVector> result;
        const auto t = p1 - p0;
        const auto u = p2 - p0;
        const auto v = p2 - p1;

        const auto w = AVector::Cross( t, u );
        const float wsl = AVector::Len2( w );
        if( wsl < this->m_parameters->m_epsilon ) {
            result.push_back( p0 );
            result.push_back( p1 );
            result.push_back( p2 );
        } else {
            const float iwsl2 = 1.0f / ( 2.0f * wsl );
            const float tt = AVector::Dot( t, t );
            const float uu = AVector::Dot( u, u );

            const auto circ_center = p0 + ( u * tt * ( AVector::Dot( u, v ) ) - t * uu * ( AVector::Dot( t, v ) ) ) * iwsl2;
            const auto circAxis = w * ( 1.0f / sqrtf( wsl ) );
            const auto center_p0 = p0 - circ_center;
            const auto center_p2 = p2 - circ_center;
            const auto center_p0_normalized = AVector::Normalized( center_p0 );
            const auto center_p2_normalized = AVector::Normalized( center_p2 );

            const float openingAngle = std::acos( AVector::Dot( center_p0_normalized, center_p2_normalized ) );
            int n = (int)( this->m_parameters->m_NumVerticesPerCircle * openingAngle / ( M_PI * 2.0f ) );
            n = std::max( n, this->m_parameters->minNumVerticesPerArc );

            const float deltaAngle = openingAngle / (float)( n - 1 );
            double angle = 0;
            for( size_t kk = 0; kk < n; ++kk ) {
                const auto m = TMatrix::GetRotation( -angle, circAxis );
                const auto p_rotated = m.GetTransformed( center_p0 ) + circ_center;

                result.push_back( p_rotated );
                angle += deltaAngle;
            }
        }
        return result;
    }
};

}