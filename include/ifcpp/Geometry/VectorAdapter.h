#pragma once

#include <cmath>

#include "ifcpp/Geometry/CVector.h"


namespace ifcpp {

template<CVector TVector>
class VectorAdapter {
public:
    static TVector New( double x = 0.0, double y = 0.0, double z = 0.0 ) {
        TVector v;
        v.x = x;
        v.y = y;
        v.z = z;
        return v;
    }

    static double Len( const TVector& v ) {
        return sqrt( Len2( v ) );
    }

    static double Len2( const TVector& v ) {
        return v.x * v.x + v.y * v.y + v.z * v.z;
    }

    static TVector Normalized( const TVector& v ) {
        TVector result = v;
        VectorAdapter::Normalize( &result );
        return result;
    }

    static void Normalize( TVector* v ) {
        double len = VectorAdapter::Len( *v );
        if( len <= 1e-3 ) {
            return;
        }
        double invLength = 1.0 / len;
        v->x *= invLength;
        v->y *= invLength;
        v->z *= invLength;
    }

    static double Dot( const TVector& v1, const TVector& v2 ) {
        return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
    }

    static TVector Cross( const TVector& v1, const TVector& v2 ) {
        return VectorAdapter::New( v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x );
    }

    static bool IsNearlyEqual( const TVector& v1, const TVector& v2, double preccission = 1e-6 ) {
        return fabs( v1.x - v2.x ) < preccission && fabs( v1.y - v2.y ) < preccission && fabs( v1.z - v2.z ) < preccission;
    }
};

}