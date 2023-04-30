#pragma once

#include <limits>
#include <set>

#include "ifcpp/Geometry/CAdapter.h"
#include "ifcpp/Geometry/Matrix.h"
#include "ifcpp/Geometry/Parameters.h"
#include "ifcpp/Geometry/VectorAdapter.h"


namespace ifcpp {

template<CVector TVector>
class BoundingBox {
    using AVector = VectorAdapter<TVector>;

public:
    TVector m_min = AVector::New( std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() );
    TVector m_max = AVector::New( -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max() );
    BoundingBox() = default;
    explicit BoundingBox( const std::vector<TVector>& points ) {
        for( const auto v: points ) {
            this->m_min.x = std::min( this->m_min.x, v.x );
            this->m_min.y = std::min( this->m_min.y, v.y );
            this->m_min.z = std::min( this->m_min.z, v.z );
            this->m_max.x = std::max( this->m_max.x, v.x );
            this->m_max.y = std::max( this->m_max.y, v.y );
            this->m_max.z = std::max( this->m_max.z, v.z );
        }
    }
    void AddPoints( const std::vector<TVector>& points ) {
        for( const auto v: points ) {
            this->m_min.x = std::min( this->m_min.x, v.x );
            this->m_min.y = std::min( this->m_min.y, v.y );
            this->m_min.z = std::min( this->m_min.z, v.z );
            this->m_max.x = std::max( this->m_max.x, v.x );
            this->m_max.y = std::max( this->m_max.y, v.y );
            this->m_max.z = std::max( this->m_max.z, v.z );
        }
    }
    TVector GetExtents() const {
        return this->m_max - this->m_min;
    }
};

template<CVector TVector>
class Plane {
    using AVector = VectorAdapter<TVector>;

public:
    TVector m_origin;
    TVector m_normal;
    TVector m_up;
    TVector m_right;
    Plane( const TVector& origin, const TVector& normal ) {
        this->m_origin = origin;
        this->m_normal = normal;
        if( AVector::Len2( normal ) < 1e-6 ) {
            this->m_normal = AVector::Normalized( AVector::New( 1, 1, 1 ) );
        }
        this->m_right = AVector::Normalized( AVector::Cross( AVector::New( 0, 0, 1 ), this->m_normal ) );
        if( AVector::Len2( this->m_right ) < 1e-6 ) {
            this->m_right = AVector::Normalized( AVector::Cross( this->m_normal, AVector::New( 0, -1, 0 ) ) );
        }
        this->m_up = AVector::Cross( this->m_normal, this->m_right );
    }
    std::vector<TVector> GetProjection( const std::vector<TVector>& points ) const {
        std::vector<TVector> result;
        for( const auto& v: points ) {
            result.push_back( AVector::New( AVector::Dot( this->m_right, v - this->m_origin ), AVector::Dot( this->m_up, v - this->m_origin ) ) );
        }
        return result;
    }
    std::vector<TVector> GetUnProjected( const std::vector<TVector>& points ) const {
        std::vector<TVector> result;
        for( const auto& v: points ) {
            result.push_back( this->m_right * v.x + this->m_up * v.y + this->m_origin );
        }
        return result;
    }
};

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
    struct Line2d {
        float a = 0, b = 0, c = 0;
        Line2d( float _a, float _b, float _c )
            : a( _a )
            , b( _b )
            , c( _c ) {
        }
        Line2d( TVector p, TVector q ) {
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
        Line2d line( a, b );
        return line.Distance( point ) < this->m_parameters->m_epsilon && l <= p && p <= r;
    }
    Intersection Intersect2d( const TVector& a1, const TVector& b1, const TVector& a2, const TVector& b2 ) {
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
            Line2d m( a1, b1 );
            Line2d n( a2, b2 );
            float zn = m.a * n.b - m.b * n.a;
            result.left.x = result.right.x = -( m.c * n.b - m.b * n.c ) / zn;
            result.left.y = result.right.y = -( m.a * n.c - m.c * n.a ) / zn;
            result.isIntersects = this->IsPointOnEdge( a1, b1, result.left ) && this->IsPointOnEdge( a2, b2, result.left );
        }
        return result;
    }
    std::vector<TVector> SimplifyLoop( std::vector<TVector> loop ) {
        const auto restoreInfo = this->MoveToOriginAndScale( &loop );

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

        if( result.size() < 3 ) {
            this->Restore( &result, restoreInfo );
            return result;
        }

        result.push_back( result[ 0 ] );
        result.insert( std::begin( result ), result[ result.size() - 2 ] );
        for( int i = 1; i < result.size() - 1; i++ ) {
            const auto a = result[ i - 1 ] - result[ i ];
            const auto b = result[ i + 1 ] - result[ i ];
            const auto c = AVector::Cross( a, b );
            if( AVector::Len2( c ) < this->m_parameters->m_epsilon ) {
                result.erase( std::begin( result ) + i );
                i--;
            }
        }
        result.erase( std::begin( result ) );
        result.pop_back();

        this->Restore( &result, restoreInfo );
        return result;
    }
    std::vector<TVector> SimplifyCurve( std::vector<TVector> curve ) {
        const auto restoreInfo = this->MoveToOriginAndScale( &curve );
        std::vector<TVector> result;
        result.reserve( curve.size() );
        if( curve.empty() ) {
            return result;
        }
        result.push_back( curve[ 0 ] );
        for( const auto& point: curve ) {
            if( !AVector::IsNearlyEqual( point, result[ result.size() - 1 ] ) ) {
                result.push_back( point );
            }
        }
        if( result.size() < 3 ) {
            this->Restore( &result, restoreInfo );
            return result;
        }
        for( int i = 1; i < result.size() - 1; i++ ) {
            const auto a = result[ i - 1 ] - result[ i ];
            const auto b = result[ i + 1 ] - result[ i ];
            const auto c = AVector::Cross( a, b );
            if( AVector::Len2( c ) < this->m_parameters->m_epsilon ) {
                result.erase( std::begin( result ) + i );
                i--;
            }
        }
        this->Restore( &result, restoreInfo );
        return result;
    }
    void AppendToLoop( std::vector<TVector>* loop, const std::vector<TVector>& toAppend ) {
        // TODO: Check order, remove duplicates, etc...
        std::copy( std::begin( toAppend ), std::end( toAppend ), std::back_inserter( *loop ) );
    }
    std::vector<TVector> CombineLoops( std::vector<std::vector<TVector>> loops ) {
        const auto restoreInfo = this->MoveToOriginAndScale( &loops );

        while( !loops.empty() && loops[ 0 ].size() < 3 ) {
            loops.erase( std::begin( loops ) );
        }

        if( loops.empty() ) {
            return {};
        }

        auto planeNormal = this->ComputePolygonNormal( loops[ 0 ] );
        const Plane<TVector> p( loops[ 0 ][ 0 ], planeNormal );


        for( auto& l: loops ) {
            l = p.GetProjection( l );
        }


        for( auto& l: loops ) {
            if( !l.empty() ) {
                l.push_back( l[ 0 ] );
            }
        }

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

                        const auto d2 = AVector::Len2( p2 - p1 );
                        if( d2 >= dist2 ) {
                            continue;
                        }

                        bool isIntersects = false;
                        //                        for( int i = 1; i < result.size(); i++ ) {
                        //                            const auto intersection = this->Intersect2d( result[ i - 1 ], result[ i ], p1, p2 );
                        //
                        //                            if( intersection.isOnOneLine || AVector::IsNearlyEqual( intersection.left, p1 ) ||
                        //                                AVector::IsNearlyEqual( intersection.left, p2 ) || AVector::IsNearlyEqual( intersection.left, result[
                        //                                i - 1 ] ) || AVector::IsNearlyEqual( intersection.left, result[ i ] ) ) { continue;
                        //                            }
                        //                            if( intersection.isIntersects ) {
                        //                                isIntersects = true;
                        //                            }
                        //                        }
                        //                        for( int j = 0; j < loops.size(); j++ ) {
                        //                            for( int i = 1; i < loops[ j ].size(); i++ ) {
                        //                                const auto intersection = this->Intersect2d( loops[ j ][ i - 1 ], loops[ j ][ i ], p1, p2 );
                        //                                if( intersection.isOnOneLine || AVector::IsNearlyEqual( intersection.left, p1 ) ||
                        //                                    AVector::IsNearlyEqual( intersection.left, p2 ) || AVector::IsNearlyEqual( intersection.left,
                        //                                    loops[ j ][ i - 1 ] ) || AVector::IsNearlyEqual( intersection.left, loops[ j ][ i ] ) ) {
                        //                                    continue;
                        //                                }
                        //                                if( intersection.isIntersects ) {
                        //                                    isIntersects = true;
                        //                                }
                        //                            }
                        //                        }
                        if( isIntersects ) {
                            continue;
                        }

                        // const auto d2 = AVector::Len2( p2 - p1 );
                        if( d2 < dist2 ) {
                            dist2 = d2;
                            inResultIdx = ridx;
                            loopIdx = lidx;
                            pointIdx = pidx;
                        }
                    }
                }
            }

            if( inResultIdx < 0 || loopIdx < 0 || pointIdx < 0 ) {
                // LOG ERROR
                break;
            }

            loops[ loopIdx ].pop_back();
            if( pointIdx == loops[ loopIdx ].size() ) {
                pointIdx = 0;
            }

            std::vector<TVector> loopToInsert( std::begin( loops[ loopIdx ] ) + pointIdx, std::end( loops[ loopIdx ] ) );
            std::copy( std::begin( loops[ loopIdx ] ), std::begin( loops[ loopIdx ] ) + pointIdx, std::back_inserter( loopToInsert ) );

            loopToInsert.push_back( loopToInsert[0] );

            loops.erase( std::begin( loops ) + loopIdx );
            result.insert( std::begin( result ) + inResultIdx, result[ inResultIdx ] );
            result.insert( std::begin( result ) + inResultIdx + 1, std::begin( loopToInsert ), std::end( loopToInsert ) );
        }
        result = p.GetUnProjected( result );
        this->Restore( &result, restoreInfo );
        return this->SimplifyLoop( result );
    }
    std::vector<TVector> IncorporateHoles( const std::vector<TVector>& outer, std::vector<std::vector<TVector>> inners ) {
        if( inners.empty() ) {
            return outer;
        }
        const auto outerNormal = this->ComputePolygonNormal( outer );
        for( auto& inner: inners ) {
            const auto innerNormal = this->ComputePolygonNormal( inner );
            if( AVector::Dot( outerNormal, innerNormal ) > 0 ) {
                std::reverse( inner.begin(), inner.end() );
            }
        }
        inners.insert( inners.begin(), outer );
        auto result = this->CombineLoops( inners );

        std::vector<char> outerChar;
        std::vector<char> innerChar;

        auto in = inners[ 1 ];
        for( int i = 0; i < outer.size(); i++ ) {
            outerChar.push_back( 'A' + i );
        }
        for( int i = 0; i < in.size(); i++ ) {
            innerChar.push_back( 'a' + i );
        }

        std::string str;

        for( int i = 0; i < result.size(); i++ ) {
            bool isOk = false;
            for( int j = 0; j < outerChar.size(); j++ ) {
                if( result[ i ] == outer[ j ] ) {
                    str += std::string( { outerChar[ j ] } );
                    isOk = true;
                    break;
                }
            }
            if( isOk )
                continue;

            for( int j = 0; j < innerChar.size(); j++ ) {
                if( result[ i ] == in[ j ] ) {
                    str += std::string( { innerChar[ j ] } );
                    isOk = true;
                    break;
                }
            }

            if( isOk )
                continue;

            str += std::string( { '-' } );
        }

        std::cout << str << "\n";

        return result;
    }
    std::vector<TVector> BuildEllipse( float radius1, float radius2, float startAngle, float openingAngle, int verticesCount,
                                       TVector center = AVector::New() ) const {
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
        return this->BuildCircle( radius, startAngle, openingAngle, verticesCount, AVector::New( x, y ) );
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
            int n = (int)( this->m_parameters->m_numVerticesPerCircle * openingAngle / ( M_PI * 2.0f ) );
            n = std::max( n, this->m_parameters->m_minNumVerticesPerArc );

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

    TVector ComputePolygonNormal( std::vector<TVector> loop ) {
//        TVector normal;
//        for( int i = 1; i < loop.size() - 1; i++ ) {
//            normal = normal + AVector::Cross( ( loop[ i - 1 ] - loop[ i ] ), ( loop[ i + 1 ] - loop[ i ] ) );
//        }
//
//        if( AVector::Len2( normal ) < 1e-12 ) {
//            return AVector::New();
//        }
//        normal = -AVector::Normalized( normal );
//        return normal;

        // this->MoveToOriginAndScale( &loop, true );

        if( loop.size() < 3 ) {
            // TODO: Log error
            return AVector::New();
        }

        TVector planeNormal = AVector::New();

        for( const auto& a: loop ) {
            for( const auto& b: loop ) {
                for( const auto& c: loop ) {
                    const auto normal = AVector::Cross( b - a, c - b );
                    if( AVector::Len2( normal ) > AVector::Len2( planeNormal ) ) {
                        planeNormal = normal;
                    }
                }
            }
        }

        planeNormal = AVector::Normalized( planeNormal );

        Plane<TVector> p( loop[ 0 ], planeNormal );
        loop = p.GetProjection( loop );
        this->MoveToOriginAndScale( &loop );
        loop.push_back( loop[ 0 ] );

        float s = 0.0f;
        for( int i = 1; i < loop.size(); i++ ) {
            const auto& v1 = loop[ i - 1 ];
            const auto& v2 = loop[ i ];
            s += ( v1.y + v2.y ) * 0.5f * ( v1.x - v2.x );
        }

        if( s < 0 ) {
            planeNormal = -planeNormal;
        }

        return planeNormal;
    }

    //         offset   scale
    std::tuple<TVector, TVector> MoveToOriginAndScale( std::vector<TVector>* points, bool saveProportions = false ) {
        BoundingBox<TVector> bbox( *points );
        const auto offset = -bbox.m_min;
        const auto e = bbox.GetExtents();
        const float t = 9.0f;
        auto scale = AVector::New( e.x > 0 ? t / e.x : 1.0f, e.y > 0 ? t / e.y : 1.0f, e.z > 0 ? t / e.z : 1.0f );

        if( saveProportions ) {
            scale.x = std::max( std::max( scale.x, scale.y ), scale.z );
            scale.y = scale.z = scale.x;
        }

        for( auto& v: *points ) {
            v = ( v + offset );
            v.x *= scale.x;
            v.y *= scale.y;
            v.z *= scale.z;
        }

        return { offset, scale };
    }

    void Restore( std::vector<TVector>* points, const std::tuple<TVector, TVector>& offsetAndScale ) {
        auto [ offset, scale ] = offsetAndScale;
        offset = -offset;
        scale.x = 1.0f / scale.x;
        scale.y = 1.0f / scale.y;
        scale.z = 1.0f / scale.z;

        for( auto& v: *points ) {
            v.x *= scale.x;
            v.y *= scale.y;
            v.z *= scale.z;
            v = ( v + offset );
        }
    }

    std::tuple<TVector, TVector> MoveToOriginAndScale( std::vector<std::vector<TVector>>* points, bool saveProportions = false ) {
        BoundingBox<TVector> bbox;
        for( const auto& l: *points ) {
            bbox.AddPoints( l );
        }
        const auto offset = -bbox.m_min;
        const auto e = bbox.GetExtents();
        const float t = 9.0f;
        auto scale = AVector::New( e.x > 0 ? t / e.x : 1.0f, e.y > 0 ? t / e.y : 1.0f, e.z > 0 ? t / e.z : 1.0f );

        if( saveProportions ) {
            scale.x = std::max( std::max( scale.x, scale.y ), scale.z );
            scale.y = scale.z = scale.x;
        }

        for( auto& l: *points ) {
            for( auto& v: l ) {
                v = ( v + offset );
                v.x *= scale.x;
                v.y *= scale.y;
                v.z *= scale.z;
            }
        }

        return { offset, scale };
    }

    void Restore( std::vector<std::vector<TVector>>* points, const std::tuple<TVector, TVector>& offsetAndScale ) {
        auto [ offset, scale ] = offsetAndScale;
        offset = -offset;
        scale.x = 1.0f / scale.x;
        scale.y = 1.0f / scale.y;
        scale.z = 1.0f / scale.z;

        for( auto& l: *points ) {
            for( auto& v: l ) {
                v.x *= scale.x;
                v.y *= scale.y;
                v.z *= scale.z;
                v = ( v + offset );
            }
        }
    }
};

}