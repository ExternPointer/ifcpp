#pragma once

#include "csgjs.h"
#include <memory>


namespace ifcpp {
class Matrix {
public:
    float data[ 4 ][ 4 ];
    inline static Matrix GetIdentity() {
        Matrix matrix {};
        memset( matrix.data, 0, sizeof( float ) * 16 );
        matrix.data[ 0 ][ 0 ] = 1;
        matrix.data[ 1 ][ 1 ] = 1;
        matrix.data[ 2 ][ 2 ] = 1;
        matrix.data[ 3 ][ 3 ] = 1;
        return matrix;
    }
    inline static Matrix GetScale( float x, float y, float z ) {
        auto matrix = Matrix::GetIdentity();
        matrix.data[ 0 ][ 0 ] = x;
        matrix.data[ 1 ][ 1 ] = y;
        matrix.data[ 2 ][ 2 ] = z;
        return matrix;
    }
    inline static Matrix CreateFromAxis( csgjscpp::Vector x, csgjscpp::Vector y, csgjscpp::Vector z, csgjscpp::Vector t ) {
        Matrix matrix {};

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
    inline static Matrix GetRotation( float angle, const csgjscpp::Vector& v ) {
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
    inline void Transform( csgjscpp::Vector* v ) const {
        float rw = 1.0f / ( this->data[ 3 ][ 0 ] * v->x + this->data[ 3 ][ 1 ] * v->y + this->data[ 3 ][ 2 ] * v->z + this->data[ 3 ][ 3 ] );
        *v = { ( this->data[ 0 ][ 0 ] * v->x + this->data[ 0 ][ 1 ] * v->y + this->data[ 0 ][ 2 ] * v->z + this->data[ 0 ][ 3 ] ) * rw,
               ( this->data[ 1 ][ 0 ] * v->x + this->data[ 1 ][ 1 ] * v->y + this->data[ 1 ][ 2 ] * v->z + this->data[ 1 ][ 3 ] ) * rw,
               ( this->data[ 2 ][ 0 ] * v->x + this->data[ 2 ][ 1 ] * v->y + this->data[ 2 ][ 2 ] * v->z + this->data[ 2 ][ 3 ] ) * rw };
    }
    [[nodiscard]] inline csgjscpp::Vector GetTransformed( const csgjscpp::Vector& v ) const {
        auto result = v;
        this->Transform( &result );
        return result;
    }
    inline void Transform( csgjscpp::Model* m ) const {
        for(auto& v: m->vertices) {
            this->Transform(&v.pos);
        }
    }
    [[nodiscard]] inline csgjscpp::Model GetTransformed( const csgjscpp::Model& m ) const {
        auto result = m;
        this->Transform( &result );
        return result;
    }
    inline static void Multiply( Matrix* m1, const Matrix& m2 ) {
        Matrix& out = *m1;
        const Matrix a = *m1;
        const Matrix& b = m2;
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
    inline bool Invert() {
        float in[ 4 ][ 4 ];
        memcpy( in, this->data, sizeof( float ) * 16 );
        float* temp;
        float* r[ 4 ];
        float rtemp[ 4 ][ 8 ];
        float m[ 4 ];
        float s;

        r[ 0 ] = rtemp[ 0 ];
        r[ 1 ] = rtemp[ 1 ];
        r[ 2 ] = rtemp[ 2 ];
        r[ 3 ] = rtemp[ 3 ];

        r[ 0 ][ 0 ] = in[ 0 ][ 0 ];
        r[ 0 ][ 1 ] = in[ 0 ][ 1 ];
        r[ 0 ][ 2 ] = in[ 0 ][ 2 ];
        r[ 0 ][ 3 ] = in[ 0 ][ 3 ];
        r[ 0 ][ 4 ] = 1.0f;
        r[ 0 ][ 5 ] = 0.0f;
        r[ 0 ][ 6 ] = 0.0f;
        r[ 0 ][ 7 ] = 0.0f;

        r[ 1 ][ 0 ] = in[ 1 ][ 0 ];
        r[ 1 ][ 1 ] = in[ 1 ][ 1 ];
        r[ 1 ][ 2 ] = in[ 1 ][ 2 ];
        r[ 1 ][ 3 ] = in[ 1 ][ 3 ];
        r[ 1 ][ 5 ] = 1.0f;
        r[ 1 ][ 4 ] = 0.0f;
        r[ 1 ][ 6 ] = 0.0f;
        r[ 1 ][ 7 ] = 0.0f;

        r[ 2 ][ 0 ] = in[ 2 ][ 0 ];
        r[ 2 ][ 1 ] = in[ 2 ][ 1 ];
        r[ 2 ][ 2 ] = in[ 2 ][ 2 ];
        r[ 2 ][ 3 ] = in[ 2 ][ 3 ];
        r[ 2 ][ 6 ] = 1.0f;
        r[ 2 ][ 4 ] = 0.0f;
        r[ 2 ][ 5 ] = 0.0f;
        r[ 2 ][ 7 ] = 0.0f;

        r[ 3 ][ 0 ] = in[ 3 ][ 0 ];
        r[ 3 ][ 1 ] = in[ 3 ][ 1 ];
        r[ 3 ][ 2 ] = in[ 3 ][ 2 ];
        r[ 3 ][ 3 ] = in[ 3 ][ 3 ];
        r[ 3 ][ 4 ] = 0.0f;
        r[ 3 ][ 5 ] = 0.0f;
        r[ 3 ][ 6 ] = 0.0f;
        r[ 3 ][ 7 ] = 1.0f;

        if( fabs( r[ 3 ][ 0 ] ) > fabs( r[ 2 ][ 0 ] ) ) {
            temp = r[ 3 ];
            r[ 3 ] = r[ 2 ];
            r[ 2 ] = temp;
        }

        if( fabs( r[ 2 ][ 0 ] ) > fabs( r[ 1 ][ 0 ] ) ) {
            temp = r[ 2 ];
            r[ 2 ] = r[ 1 ];
            r[ 1 ] = temp;
        }

        if( fabs( r[ 1 ][ 0 ] ) > fabs( r[ 0 ][ 0 ] ) ) {
            temp = r[ 1 ];
            r[ 1 ] = r[ 0 ];
            r[ 0 ] = temp;
        }

        if( r[ 0 ][ 0 ] != 0.0f ) {
            m[ 1 ] = r[ 1 ][ 0 ] / r[ 0 ][ 0 ];
            m[ 2 ] = r[ 2 ][ 0 ] / r[ 0 ][ 0 ];
            m[ 3 ] = r[ 3 ][ 0 ] / r[ 0 ][ 0 ];

            s = r[ 0 ][ 1 ];
            r[ 1 ][ 1 ] -= m[ 1 ] * s;
            r[ 2 ][ 1 ] -= m[ 2 ] * s;
            r[ 3 ][ 1 ] -= m[ 3 ] * s;

            s = r[ 0 ][ 2 ];
            r[ 1 ][ 2 ] -= m[ 1 ] * s;
            r[ 2 ][ 2 ] -= m[ 2 ] * s;
            r[ 3 ][ 2 ] -= m[ 3 ] * s;

            s = r[ 0 ][ 3 ];
            r[ 1 ][ 3 ] -= m[ 1 ] * s;
            r[ 2 ][ 3 ] -= m[ 2 ] * s;
            r[ 3 ][ 3 ] -= m[ 3 ] * s;

            s = r[ 0 ][ 4 ];
            if( s != 0.0f ) {
                r[ 1 ][ 4 ] -= m[ 1 ] * s;
                r[ 2 ][ 4 ] -= m[ 2 ] * s;
                r[ 3 ][ 4 ] -= m[ 3 ] * s;
            }

            s = r[ 0 ][ 5 ];
            if( s != 0.0f ) {
                r[ 1 ][ 5 ] -= m[ 1 ] * s;
                r[ 2 ][ 5 ] -= m[ 2 ] * s;
                r[ 3 ][ 5 ] -= m[ 3 ] * s;
            }

            s = r[ 0 ][ 6 ];
            if( s != 0.0f ) {
                r[ 1 ][ 6 ] -= m[ 1 ] * s;
                r[ 2 ][ 6 ] -= m[ 2 ] * s;
                r[ 3 ][ 6 ] -= m[ 3 ] * s;
            }

            s = r[ 0 ][ 7 ];
            if( s != 0.0f ) {
                r[ 1 ][ 7 ] -= m[ 1 ] * s;
                r[ 2 ][ 7 ] -= m[ 2 ] * s;
                r[ 3 ][ 7 ] -= m[ 3 ] * s;
            }

            if( fabs( r[ 3 ][ 1 ] ) > fabs( r[ 2 ][ 1 ] ) ) {
                temp = r[ 3 ];
                r[ 3 ] = r[ 2 ];
                r[ 2 ] = temp;
            }

            if( fabs( r[ 2 ][ 1 ] ) > fabs( r[ 1 ][ 1 ] ) ) {
                temp = r[ 2 ];
                r[ 2 ] = r[ 1 ];
                r[ 1 ] = temp;
            }

            if( r[ 1 ][ 1 ] != 0.0f ) {
                m[ 2 ] = r[ 2 ][ 1 ] / r[ 1 ][ 1 ];
                m[ 3 ] = r[ 3 ][ 1 ] / r[ 1 ][ 1 ];
                r[ 2 ][ 2 ] -= m[ 2 ] * r[ 1 ][ 2 ];
                r[ 3 ][ 2 ] -= m[ 3 ] * r[ 1 ][ 2 ];
                r[ 2 ][ 3 ] -= m[ 2 ] * r[ 1 ][ 3 ];
                r[ 3 ][ 3 ] -= m[ 3 ] * r[ 1 ][ 3 ];

                s = r[ 1 ][ 4 ];
                if( s != 0.0f ) {
                    r[ 2 ][ 4 ] -= m[ 2 ] * s;
                    r[ 3 ][ 4 ] -= m[ 3 ] * s;
                }

                s = r[ 1 ][ 5 ];
                if( s != 0.0f ) {
                    r[ 2 ][ 5 ] -= m[ 2 ] * s;
                    r[ 3 ][ 5 ] -= m[ 3 ] * s;
                }

                s = r[ 1 ][ 6 ];
                if( s != 0.0f ) {
                    r[ 2 ][ 6 ] -= m[ 2 ] * s;
                    r[ 3 ][ 6 ] -= m[ 3 ] * s;
                }

                s = r[ 1 ][ 7 ];
                if( s != 0.0f ) {
                    r[ 2 ][ 7 ] -= m[ 2 ] * s;
                    r[ 3 ][ 7 ] -= m[ 3 ] * s;
                }

                if( fabs( r[ 3 ][ 2 ] ) > fabs( r[ 2 ][ 2 ] ) ) {
                    temp = r[ 3 ];
                    r[ 3 ] = r[ 2 ];
                    r[ 2 ] = temp;
                }

                if( r[ 2 ][ 2 ] != 0.0f ) {
                    m[ 3 ] = r[ 3 ][ 2 ] / r[ 2 ][ 2 ];
                    r[ 3 ][ 3 ] -= m[ 3 ] * r[ 2 ][ 3 ];
                    r[ 3 ][ 4 ] -= m[ 3 ] * r[ 2 ][ 4 ];
                    r[ 3 ][ 5 ] -= m[ 3 ] * r[ 2 ][ 5 ];
                    r[ 3 ][ 6 ] -= m[ 3 ] * r[ 2 ][ 6 ];
                    r[ 3 ][ 7 ] -= m[ 3 ] * r[ 2 ][ 7 ];

                    if( r[ 3 ][ 3 ] != 0.0f ) {
                        s = 1.0f / r[ 3 ][ 3 ];
                        r[ 3 ][ 4 ] *= s;
                        r[ 3 ][ 5 ] *= s;
                        r[ 3 ][ 6 ] *= s;
                        r[ 3 ][ 7 ] *= s;

                        m[ 2 ] = r[ 2 ][ 3 ];
                        s = 1.0f / r[ 2 ][ 2 ];
                        r[ 2 ][ 4 ] = s * ( r[ 2 ][ 4 ] - r[ 3 ][ 4 ] * m[ 2 ] );
                        r[ 2 ][ 5 ] = s * ( r[ 2 ][ 5 ] - r[ 3 ][ 5 ] * m[ 2 ] );
                        r[ 2 ][ 6 ] = s * ( r[ 2 ][ 6 ] - r[ 3 ][ 6 ] * m[ 2 ] );
                        r[ 2 ][ 7 ] = s * ( r[ 2 ][ 7 ] - r[ 3 ][ 7 ] * m[ 2 ] );

                        m[ 1 ] = r[ 1 ][ 3 ];
                        r[ 1 ][ 4 ] -= r[ 3 ][ 4 ] * m[ 1 ];
                        r[ 1 ][ 5 ] -= r[ 3 ][ 5 ] * m[ 1 ];
                        r[ 1 ][ 6 ] -= r[ 3 ][ 6 ] * m[ 1 ];
                        r[ 1 ][ 7 ] -= r[ 3 ][ 7 ] * m[ 1 ];

                        m[ 0 ] = r[ 0 ][ 3 ];
                        r[ 0 ][ 4 ] -= r[ 3 ][ 4 ] * m[ 0 ];
                        r[ 0 ][ 5 ] -= r[ 3 ][ 5 ] * m[ 0 ];
                        r[ 0 ][ 6 ] -= r[ 3 ][ 6 ] * m[ 0 ];
                        r[ 0 ][ 7 ] -= r[ 3 ][ 7 ] * m[ 0 ];

                        m[ 1 ] = r[ 1 ][ 2 ];
                        s = 1.0f / r[ 1 ][ 1 ];
                        r[ 1 ][ 4 ] = s * ( r[ 1 ][ 4 ] - r[ 2 ][ 4 ] * m[ 1 ] );
                        r[ 1 ][ 5 ] = s * ( r[ 1 ][ 5 ] - r[ 2 ][ 5 ] * m[ 1 ] );
                        r[ 1 ][ 6 ] = s * ( r[ 1 ][ 6 ] - r[ 2 ][ 6 ] * m[ 1 ] );
                        r[ 1 ][ 7 ] = s * ( r[ 1 ][ 7 ] - r[ 2 ][ 7 ] * m[ 1 ] );

                        m[ 0 ] = r[ 0 ][ 2 ];
                        r[ 0 ][ 4 ] -= r[ 2 ][ 4 ] * m[ 0 ];
                        r[ 0 ][ 5 ] -= r[ 2 ][ 5 ] * m[ 0 ];
                        r[ 0 ][ 6 ] -= r[ 2 ][ 6 ] * m[ 0 ];
                        r[ 0 ][ 7 ] -= r[ 2 ][ 7 ] * m[ 0 ];

                        m[ 0 ] = r[ 0 ][ 1 ];
                        s = 1.0f / r[ 0 ][ 0 ];
                        r[ 0 ][ 4 ] = s * ( r[ 0 ][ 4 ] - r[ 1 ][ 4 ] * m[ 0 ] );
                        r[ 0 ][ 5 ] = s * ( r[ 0 ][ 5 ] - r[ 1 ][ 5 ] * m[ 0 ] );
                        r[ 0 ][ 6 ] = s * ( r[ 0 ][ 6 ] - r[ 1 ][ 6 ] * m[ 0 ] );
                        r[ 0 ][ 7 ] = s * ( r[ 0 ][ 7 ] - r[ 1 ][ 7 ] * m[ 0 ] );

                        this->data[ 0 ][ 0 ] = r[ 0 ][ 4 ];
                        this->data[ 0 ][ 1 ] = r[ 0 ][ 5 ];
                        this->data[ 0 ][ 2 ] = r[ 0 ][ 6 ];
                        this->data[ 0 ][ 3 ] = r[ 0 ][ 7 ];
                        this->data[ 1 ][ 0 ] = r[ 1 ][ 4 ];
                        this->data[ 1 ][ 1 ] = r[ 1 ][ 5 ];
                        this->data[ 1 ][ 2 ] = r[ 1 ][ 6 ];
                        this->data[ 1 ][ 3 ] = r[ 1 ][ 7 ];
                        this->data[ 2 ][ 0 ] = r[ 2 ][ 4 ];
                        this->data[ 2 ][ 1 ] = r[ 2 ][ 5 ];
                        this->data[ 2 ][ 2 ] = r[ 2 ][ 6 ];
                        this->data[ 2 ][ 3 ] = r[ 2 ][ 7 ];
                        this->data[ 3 ][ 0 ] = r[ 3 ][ 4 ];
                        this->data[ 3 ][ 1 ] = r[ 3 ][ 5 ];
                        this->data[ 3 ][ 2 ] = r[ 3 ][ 6 ];
                        this->data[ 3 ][ 3 ] = r[ 3 ][ 7 ];

                        return true;
                    }
                }
            }
        }
        return false;
    }
};
}