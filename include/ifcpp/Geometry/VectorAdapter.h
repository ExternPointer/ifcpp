#pragma once

#include <cmath>

#include "ifcpp/Geometry/CVector.h"


namespace ifcpp {

template<CVector TVector>
class VectorAdapter {
public:
    static TVector New( float x = 0.0f, float y = 0.0f, float z = 0.0f ) {
        TVector v;
        v.x = x;
        v.y = y;
        v.z = z;
        return v;
    }

    static float Len( const TVector& v ) {
        return sqrtf( v.x * v.x + v.y * v.y + v.z * v.z );
    }

    static float Len2( const TVector& v ) {
        return v.x * v.x + v.y * v.y + v.z * v.z;
    }

    static TVector Normalized( const TVector& v ) {
        float len = VectorAdapter::Len( v );
        if( len <= 1e-6 ) {
            return v;
        }
        float invLength = 1.0f / len;
        return VectorAdapter::New( v.x * invLength, v.y * invLength, v.z * invLength );
    }

    static void Normalize( TVector* v ) {
        float len = VectorAdapter::Len( *v );
        if( len <= 1e-12 ) {
            return;
        }
        float invLength = 1.0f / len;
        v->x *= invLength;
        v->y *= invLength;
        v->z *= invLength;
    }

    static float Dot( const TVector& v1, const TVector& v2 ) {
        return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
    }

    static TVector Cross( const TVector& v1, const TVector& v2 ) {
        return VectorAdapter::New( v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x );
    }

    static bool IsNearlyEqual( const TVector& v1, const TVector& v2, float preccission = 1e-3f ) {
        return fabsf( v1.x - v2.x ) < preccission && fabsf( v1.y - v2.y ) < preccission && fabsf( v1.z - v2.z ) < preccission;
    }
};

}