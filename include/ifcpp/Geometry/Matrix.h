#pragma once

#include <memory>

#include "ifcpp/Geometry/CAdapter.h"

namespace ifcpp {

template<CVector TVector>
class Matrix {
public:
    float data[ 4 ][ 4 ];
    static Matrix<TVector> GetIdentity() {
        Matrix<TVector> matrix {};
        memset( matrix.data, 0, sizeof( float ) * 16 );
        matrix.data[ 0 ][ 0 ] = 1;
        matrix.data[ 1 ][ 1 ] = 1;
        matrix.data[ 2 ][ 2 ] = 1;
        matrix.data[ 3 ][ 3 ] = 1;
        return matrix;
    }
    static Matrix<TVector> GetScale( float x, float y, float z ) {
        auto matrix = Matrix::GetIdentity();
        matrix.data[ 0 ][ 0 ] = x;
        matrix.data[ 1 ][ 1 ] = y;
        matrix.data[ 2 ][ 2 ] = z;
        return matrix;
    }
    static Matrix<TVector> CreateFromAxises( const TVector& x, const TVector& y, const TVector& z, const TVector& t ) {
        Matrix<TVector> matrix {};

        matrix.data[ 0 ][ 0 ] = x.x;
        matrix.data[ 1 ][ 0 ] = x.y;
        matrix.data[ 2 ][ 0 ] = x.z;
        matrix.data[ 3 ][ 0 ] = 0;

        matrix.data[ 0 ][ 1 ] = y.x;
        matrix.data[ 1 ][ 1 ] = y.y;
        matrix.data[ 2 ][ 1 ] = y.z;
        matrix.data[ 3 ][ 1 ] = 0;

        matrix.data[ 0 ][ 2 ] = z.x;
        matrix.data[ 1 ][ 2 ] = z.y;
        matrix.data[ 2 ][ 2 ] = z.z;
        matrix.data[ 3 ][ 2 ] = 0;

        matrix.data[ 0 ][ 3 ] = t.x;
        matrix.data[ 1 ][ 3 ] = t.y;
        matrix.data[ 2 ][ 3 ] = t.z;
        matrix.data[ 3 ][ 3 ] = 1;

        return matrix;
    }
    static Matrix<TVector> GetRotation( float angle, const TVector& v ) {
        auto result = Matrix::GetIdentity();

        auto a = angle;
        auto c = cosf( a );
        auto s = sinf( a );
        auto axis = csgjscpp::unit( v );

        result.data[ 0 ][ 0 ] = c + ( 1.0f - c ) * axis.x * axis.x;
        result.data[ 0 ][ 1 ] = ( 1.0f - c ) * axis.x * axis.y + s * axis.z;
        result.data[ 0 ][ 2 ] = ( 1.0f - c ) * axis.x * axis.z - s * axis.y;
        result.data[ 0 ][ 3 ] = 0.0f;

        result.data[ 1 ][ 0 ] = ( 1.0f - c ) * axis.y * axis.x - s * axis.z;
        result.data[ 1 ][ 1 ] = c + ( 1.0f - c ) * axis.y * axis.y;
        result.data[ 1 ][ 2 ] = ( 1.0f - c ) * axis.y * axis.z + s * axis.x;
        result.data[ 1 ][ 3 ] = 0.0f;

        result.data[ 2 ][ 0 ] = ( 1.0f - c ) * axis.z * axis.x + s * axis.y;
        result.data[ 2 ][ 1 ] = ( 1.0f - c ) * axis.z * axis.y - s * axis.x;
        result.data[ 2 ][ 2 ] = c + ( 1.0f - c ) * axis.z * axis.z;
        result.data[ 2 ][ 3 ] = 0.0f;

        return result;
    }
    void Transform( TVector* v ) const {
        float rw = 1.0f / ( this->data[ 3 ][ 0 ] * v->x + this->data[ 3 ][ 1 ] * v->y + this->data[ 3 ][ 2 ] * v->z + this->data[ 3 ][ 3 ] );
        *v = { ( this->data[ 0 ][ 0 ] * v->x + this->data[ 0 ][ 1 ] * v->y + this->data[ 0 ][ 2 ] * v->z + this->data[ 0 ][ 3 ] ) * rw,
               ( this->data[ 1 ][ 0 ] * v->x + this->data[ 1 ][ 1 ] * v->y + this->data[ 1 ][ 2 ] * v->z + this->data[ 1 ][ 3 ] ) * rw,
               ( this->data[ 2 ][ 0 ] * v->x + this->data[ 2 ][ 1 ] * v->y + this->data[ 2 ][ 2 ] * v->z + this->data[ 2 ][ 3 ] ) * rw };
    }
    [[nodiscard]] TVector GetTransformed( const TVector& v ) const {
        auto result = v;
        this->Transform( &result );
        return result;
    }
    static void Multiply( Matrix<TVector>* m1, const Matrix<TVector>& m2 ) {
        Matrix<TVector>& out = *m1;
        const Matrix<TVector> a = *m1;
        const Matrix<TVector>& b = m2;
        out.data[ 0 ][ 0 ] = a.data[ 0 ][ 0 ] * b.data[ 0 ][ 0 ] + a.data[ 1 ][ 0 ] * b.data[ 0 ][ 1 ] + a.data[ 2 ][ 0 ] * b.data[ 0 ][ 2 ] +
            a.data[ 3 ][ 0 ] * b.data[ 0 ][ 3 ];
        out.data[ 1 ][ 0 ] = a.data[ 0 ][ 0 ] * b.data[ 1 ][ 0 ] + a.data[ 1 ][ 0 ] * b.data[ 1 ][ 1 ] + a.data[ 2 ][ 0 ] * b.data[ 1 ][ 2 ] +
            a.data[ 3 ][ 0 ] * b.data[ 1 ][ 3 ];
        out.data[ 2 ][ 0 ] = a.data[ 0 ][ 0 ] * b.data[ 2 ][ 0 ] + a.data[ 1 ][ 0 ] * b.data[ 2 ][ 1 ] + a.data[ 2 ][ 0 ] * b.data[ 2 ][ 2 ] +
            a.data[ 3 ][ 0 ] * b.data[ 2 ][ 3 ];
        out.data[ 3 ][ 0 ] = a.data[ 0 ][ 0 ] * b.data[ 3 ][ 0 ] + a.data[ 1 ][ 0 ] * b.data[ 3 ][ 1 ] + a.data[ 2 ][ 0 ] * b.data[ 3 ][ 2 ] +
            a.data[ 3 ][ 0 ] * b.data[ 3 ][ 3 ];
        out.data[ 0 ][ 1 ] = a.data[ 0 ][ 1 ] * b.data[ 0 ][ 0 ] + a.data[ 1 ][ 1 ] * b.data[ 0 ][ 1 ] + a.data[ 2 ][ 1 ] * b.data[ 0 ][ 2 ] +
            a.data[ 3 ][ 1 ] * b.data[ 0 ][ 3 ];
        out.data[ 1 ][ 1 ] = a.data[ 0 ][ 1 ] * b.data[ 1 ][ 0 ] + a.data[ 1 ][ 1 ] * b.data[ 1 ][ 1 ] + a.data[ 2 ][ 1 ] * b.data[ 1 ][ 2 ] +
            a.data[ 3 ][ 1 ] * b.data[ 1 ][ 3 ];
        out.data[ 2 ][ 1 ] = a.data[ 0 ][ 1 ] * b.data[ 2 ][ 0 ] + a.data[ 1 ][ 1 ] * b.data[ 2 ][ 1 ] + a.data[ 2 ][ 1 ] * b.data[ 2 ][ 2 ] +
            a.data[ 3 ][ 1 ] * b.data[ 2 ][ 3 ];
        out.data[ 3 ][ 1 ] = a.data[ 0 ][ 1 ] * b.data[ 3 ][ 0 ] + a.data[ 1 ][ 1 ] * b.data[ 3 ][ 1 ] + a.data[ 2 ][ 1 ] * b.data[ 3 ][ 2 ] +
            a.data[ 3 ][ 1 ] * b.data[ 3 ][ 3 ];
        out.data[ 0 ][ 2 ] = a.data[ 0 ][ 2 ] * b.data[ 0 ][ 0 ] + a.data[ 1 ][ 2 ] * b.data[ 0 ][ 1 ] + a.data[ 2 ][ 2 ] * b.data[ 0 ][ 2 ] +
            a.data[ 3 ][ 2 ] * b.data[ 0 ][ 3 ];
        out.data[ 1 ][ 2 ] = a.data[ 0 ][ 2 ] * b.data[ 1 ][ 0 ] + a.data[ 1 ][ 2 ] * b.data[ 1 ][ 1 ] + a.data[ 2 ][ 2 ] * b.data[ 1 ][ 2 ] +
            a.data[ 3 ][ 2 ] * b.data[ 1 ][ 3 ];
        out.data[ 2 ][ 2 ] = a.data[ 0 ][ 2 ] * b.data[ 2 ][ 0 ] + a.data[ 1 ][ 2 ] * b.data[ 2 ][ 1 ] + a.data[ 2 ][ 2 ] * b.data[ 2 ][ 2 ] +
            a.data[ 3 ][ 2 ] * b.data[ 2 ][ 3 ];
        out.data[ 3 ][ 2 ] = a.data[ 0 ][ 2 ] * b.data[ 3 ][ 0 ] + a.data[ 1 ][ 2 ] * b.data[ 3 ][ 1 ] + a.data[ 2 ][ 2 ] * b.data[ 3 ][ 2 ] +
            a.data[ 3 ][ 2 ] * b.data[ 3 ][ 3 ];
        out.data[ 0 ][ 3 ] = a.data[ 0 ][ 3 ] * b.data[ 0 ][ 0 ] + a.data[ 1 ][ 3 ] * b.data[ 0 ][ 1 ] + a.data[ 2 ][ 3 ] * b.data[ 0 ][ 2 ] +
            a.data[ 3 ][ 3 ] * b.data[ 0 ][ 3 ];
        out.data[ 1 ][ 3 ] = a.data[ 0 ][ 3 ] * b.data[ 1 ][ 0 ] + a.data[ 1 ][ 3 ] * b.data[ 1 ][ 1 ] + a.data[ 2 ][ 3 ] * b.data[ 1 ][ 2 ] +
            a.data[ 3 ][ 3 ] * b.data[ 1 ][ 3 ];
        out.data[ 2 ][ 3 ] = a.data[ 0 ][ 3 ] * b.data[ 2 ][ 0 ] + a.data[ 1 ][ 3 ] * b.data[ 2 ][ 1 ] + a.data[ 2 ][ 3 ] * b.data[ 2 ][ 2 ] +
            a.data[ 3 ][ 3 ] * b.data[ 2 ][ 3 ];
        out.data[ 3 ][ 3 ] = a.data[ 0 ][ 3 ] * b.data[ 3 ][ 0 ] + a.data[ 1 ][ 3 ] * b.data[ 3 ][ 1 ] + a.data[ 2 ][ 3 ] * b.data[ 3 ][ 2 ] +
            a.data[ 3 ][ 3 ] * b.data[ 3 ][ 3 ];
    }
};
}