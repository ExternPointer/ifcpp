#pragma once

#include <limits>
#include <set>

#include "ifcpp/Geometry/CAdapter.h"
#include "ifcpp/Geometry/Matrix.h"
#include "ifcpp/Geometry/Parameters.h"
#include "ifcpp/Geometry/VectorAdapter.h"


namespace ifcpp {

template<CVector TVector>
class GeomUtils {
    using AVector = VectorAdapter<TVector>;
    using TMatrix = Matrix<TVector>;

    std::shared_ptr<Parameters> m_parameters;

public:
    explicit GeomUtils( const std::shared_ptr<Parameters>& parameters )
        : m_parameters( parameters ) {
    }

    TVector Normal2d( const TVector& v ) {
        return AVector::Normalized( AVector::New( v.y, -v.x, 0 ) );
    }
    TVector Normal2d( const TVector& v1, const TVector& v2 ) {
        return Normal2d( v2 - v1 );
    }
    struct Intersection {
        TVector left;
        TVector right;
        bool isIntersects = false;
        bool isParallel = false;
        bool isOnOneLine = false;
    };
    struct Line {
        float a = 0, b = 0, c = 0;
        Line( float _a, float _b, float _c )
            : a( _a )
            , b( _b )
            , c( _c ) {
        }
        Line( TVector p, TVector q ) {
            a = p.y - q.y;
            b = q.x - p.x;
            c = -a * p.x - b * p.y;
            Normalize();
        }
        void Normalize() {
            double z = sqrt( a * a + b * b );
            if( z > 0 ) {
                a /= z, b /= z, c /= z;
            }
        }
        float Distance( TVector p ) {
            return fabsf( a * p.x + b * p.y + c );
        }
    };
    std::tuple<float, float> ProjectEdge( const TVector& a, const TVector& b, const TVector& axis ) {
        float l = AVector::Dot( axis, a );
        float r = AVector::Dot( axis, b );
        if( l > r ) {
            std::swap( l, r );
        }
        return { l, r };
    }
    TVector ClosestPointOnLine( const TVector& a, const TVector& b, const TVector& point ) {
        const auto dir = AVector::Normalized( b - a );
        return a + dir * AVector::Dot( dir, point );
    }
    TVector ClosestPointOnEdge( const TVector& a, const TVector& b, const TVector& point ) {
        const auto pointOnLine = this->ClosestPointOnLine( a, b, point );
        if( this->IsPointOnEdge( a, b, pointOnLine ) ) {
            return pointOnLine;
        }
        if( AVector::Len2( a - pointOnLine ) < AVector::Len2( b - pointOnLine ) ) {
            return a;
        }
        return b;
    }
    bool IsPointOnEdge( const TVector& a, const TVector& b, const TVector& point ) {
        const auto f = AVector::Normalized( b - a );
        const auto [ l, r ] = this->ProjectEdge( a, b, f );
        const auto p = AVector::Dot( f, point );
        Line line( a, b );
        return line.Distance( point ) < this->m_parameters->m_epsilon && l <= p && p <= r;
    }
    Intersection Intersect( const TVector& a1, const TVector& b1, const TVector& a2, const TVector& b2 ) {
        Intersection result;

        TVector f1 = AVector::Normalized( b1 - a1 );
        TVector f2 = AVector::Normalized( b2 - a2 );
        if( AVector::Dot( f1, f2 ) < 0 ) {
            f2 = -f2;
        }
        auto [ lp1, rp1 ] = this->ProjectEdge( a1, b1, f1 );
        auto [ lp2, rp2 ] = this->ProjectEdge( a2, b2, f2 );

        result.isParallel = AVector::IsNearlyEqual( f1, f2, this->m_parameters->m_epsilon );
        result.isOnOneLine = AVector::Len2( AVector::Cross( b1 - a1, a2 - a1 ) ) < this->m_parameters->m_epsilon &&
            AVector::Len2( AVector::Cross( b1 - a1, b2 - a1 ) ) < this->m_parameters->m_epsilon;

        if( result.isOnOneLine ) {
            if( lp1 > lp2 ) {
                std::swap( lp1, lp2 );
                std::swap( rp1, rp2 );
            }
            if( rp1 > lp2 ) {
                result.isIntersects = true;
                result.left = f1 * lp2;
                result.right = f1 * std::min( rp1, rp2 );
            }
        } else if( !result.isParallel ) {
            Line m( a1, b1 );
            Line n( a2, b2 );
            float zn = m.a * n.b - m.b * n.a;
            result.left.x = result.right.x = -( m.c * n.b - m.b * n.c ) / zn;
            result.left.y = result.right.y = -( m.a * n.c - m.c * n.a ) / zn;
            result.isIntersects = this->IsPointOnEdge( a1, b1, result.left ) && this->IsPointOnEdge( a2, b2, result.left );
        }
        return result;
    }
    std::vector<TVector> SimplifyLoop( const std::vector<TVector>& loop ) {
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
        if( result.size <= 1 ) {
            return result;
        }
        result.push_back( result[ 0 ] );
        result.insert( std::begin( result ), result[ result.size() - 2 ] );
        for( int i = 1; i < result.size() - 1; i++ ) {
            const auto a = result[ i - 1 ] - result[ i ];
            const auto b = result[ i + 1 ] - result[ i ];
            const auto c = AVector::Cross( a, b );
            if( AVector::Len2( c.z ) < this->m_parameters->m_epsilon ) {
                result.erase( std::begin( result ) + i );
                i--;
            }
        }
        result.erase( std::begin( result ) );
        result.pop_back();
        return result;
    }
    std::vector<TVector> SimplifyCurve( const std::vector<TVector>& loop ) {
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
      if( result.size <= 1 ) {
        return result;
      }
      for( int i = 1; i < result.size() - 1; i++ ) {
        const auto a = result[ i - 1 ] - result[ i ];
        const auto b = result[ i + 1 ] - result[ i ];
        const auto c = AVector::Cross( a, b );
        if( AVector::Len2( c.z ) < this->m_parameters->m_epsilon ) {
          result.erase( std::begin( result ) + i );
          i--;
        }
      }
      return result;
    }
    void AppendToLoop( std::vector<TVector>* loop, const std::vector<TVector>& toAppend ) {
        // TODO: Check order, remove duplicates, etc...
        std::back_inserter( std::begin( toAppend ), std::end( toAppend ), *loop );
    }
    std::vector<TVector> CombineLoops( std::vector<std::vector<TVector>> loops ) {
        std::vector<TVector> result = loops[ 0 ];
        loops.erase( std::begin( loops ) );

        while( loops.size() ) {
            int inResultIdx = -1;
            int loopIdx = -1;
            int pointIdx = -1;
            float dist2 = std::numeric_limits<float>::max();

            for( int ridx = 0; ridx < result.size(); ridx++ ) {
                const auto& p1 = result[ ridx ];
                for( int lidx = 0; lidx < loops.size(); lidx++ ) {
                    const auto& loop = loops[ lidx ];
                    for( int pidx = 0; pidx < loop.size(); pidx++ ) {
                        const auto p2 = loop[ pidx ];

                        bool isIntersects = false;
                        for( int i = 1; i < result.size(); i++ ) {
                            if( i == ridx || i - 1 == ridx ) {
                                continue;
                            }
                            if( this->Intersect( result[ i - 1 ], result[ i ], p1, p2 ).isIntersects ) {
                                isIntersects = true;
                            }
                        }
                        for( int j = 0; j < loops.size(); j++ ) {
                            for( int i = 1; i < result.size(); i++ ) {
                                if( j == lidx && ( i == pidx || i - 1 == pidx ) ) {
                                    continue;
                                }
                                if( this->Intersect( loops[ j ][ i - 1 ], loops[ j ][ i ], p1, p2 ).isIntersects ) {
                                    isIntersects = true;
                                }
                            }
                        }
                        if( isIntersects ) {
                            continue;
                        }

                        const auto d2 = AVector::Len2( p2 - p1 );
                        if( d2 < dist2 ) {
                            inResultIdx = ridx;
                            loopIdx = lidx;
                            pointIdx = pidx;
                        }
                    }
                }
            }

            if( inResultIdx < 0 || loopIdx < 0 || pointIdx < 0 ) {
                // TODO: Log error
                return result;
            }

            std::vector<TVector> loopToInsert( std::begin( loops[ loopIdx ] ) + pointIdx, std::end( loops[ loopIdx ] ) );
            std::back_inserter( std::begin( loops[ loopIdx ] ), std::begin( loops[ loopIdx ] ) + pointIdx, loopToInsert );
            loops.erase( std::begin( loops ) + loopIdx );
            result.insert( std::begin( result ) + inResultIdx, result[ inResultIdx ] );
            result.insert( std::begin( result ) + inResultIdx + 1, loopToInsert );
        }
        return this->SimplifyLoop( result );
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
    std::vector<TVector> BuildEllipse( float radius1, float radius2, float startAngle, float openingAngle, int verticesCount, float x, float y ) {
        return this->BuildEllipse( radius1, radius2, startAngle, openingAngle, verticesCount, AVector::New( x, y ) );
    }
    std::vector<TVector> BuildCircle( float radius, float startAngle, float openingAngle, int verticesCount, TVector center = AVector::New() ) const {
        return this->BuildEllipse( radius, radius, startAngle, openingAngle, verticesCount, center );
    }
    std::vector<TVector> BuildCircle( float radius, float startAngle, float openingAngle, int verticesCount, float x, float y ) const {
        return this->BuildCircle( radius, radius, startAngle, openingAngle, verticesCount, AVector::New( x, y ) );
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