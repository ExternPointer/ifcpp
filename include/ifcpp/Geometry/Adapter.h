#pragma once

#include "../third_party/earcut/include/mapbox/earcut.hpp"
#include <vector>

#include "Matrix.h"
#include "VectorAdapter.h"
#include "csgjs.h"
#include "ifcpp/Ifc/IfcObjectDefinition.h"

namespace ifcpp {

class Polyline {
public:
    std::vector<csgjscpp::Vector> m_points;
};

class Entity {
public:
    std::shared_ptr<IFC4X3::IfcObjectDefinition> m_ifcObject;
    std::vector<csgjscpp::Polygon> m_polygons;
    std::vector<Polyline> m_polylines;
};


class Adapter {
public:
    using TEntity = Entity;
    using TPolygon = csgjscpp::Polygon;
    using TPolyline = Polyline;
    using TVector = csgjscpp::Vector;

    inline TPolyline CreatePolyline( const std::vector<TVector>& vertices ) {
        return { vertices };
    }
    inline TPolygon CreatePolygon( std::vector<TVector> vertices, std::vector<int> indices ) {
        if( indices.size() != 3 ) {
            // TODO: Log error
            // WTF???
            return {};
        }
        return csgjscpp::Polygon( { { vertices[ indices[ 0 ] ] }, { vertices[ indices[ 1 ] ] }, { vertices[ indices[ 2 ] ] } } );
    }
    inline TEntity CreateEntity( const std::shared_ptr<IFC4X3::IfcObjectDefinition>& ifcObject, const std::vector<TPolygon>& meshes,
                                 const std::vector<TPolyline>& polylines ) {
        return { ifcObject, meshes, polylines };
    }

    inline void Transform( std::vector<TPolygon>* polygons, Matrix<TVector> matrix ) {
        for( auto& p: *polygons ) {
            for( auto& v: p.vertices ) {
                matrix.Transform( &v.pos );
            }
            p = csgjscpp::Polygon( p.vertices );
        }
    }
    inline void Transform( std::vector<TPolyline>* polylines, Matrix<TVector> matrix ) {
        for( auto& p: *polylines ) {
            for( auto& v: p.m_points ) {
                matrix.Transform( &v );
            }
        }
    }

    //    inline bool IsPointInTriangle( const TVector& a, const TVector& b, const TVector& c, const TVector& p ) const {
    //        return ( c.x - p.x ) * ( a.y - p.y ) - ( a.x - p.x ) * ( c.y - p.y ) > 1e-6 && ( a.x - p.x ) * ( b.y - p.y ) - ( b.x - p.x ) * ( a.y - p.y ) >
    //        1e-6 &&
    //            ( b.x - p.x ) * ( c.y - p.y ) - ( c.x - p.x ) * ( b.y - p.y ) > 1e-6;
    //    }

    inline std::vector<int> Triangulate( std::vector<TVector> loop ) {
        if( loop.size() < 3 ) {
            // WTF, TODO: Log error
            return {};
        }


        //        std::vector<int> indices;
        //        for( int i = 0; i < loop.size(); i++ ) {
        //            indices.push_back( i );
        //        }
        //        std::vector<int> result;
        //
        //        TVector normal;
        //        for( int i = 1; i < loop.size() - 1; i++ ) {
        //            normal = normal - csgjscpp::cross( loop[ i - 1 ] - loop[ i ], loop[ i + 1 ] - loop[ i ] );
        //        }
        //        normal = csgjscpp::unit( normal );
        //        auto right = csgjscpp::cross( { 0.0f, 0.0f, 1.0f }, normal );
        //        if( csgjscpp::lengthsquared( right ) < csgjscpp::csgjs_EPSILON ) {
        //            right = csgjscpp::cross( { 1.0f, 0.0f, 0.0f }, normal );
        //        }
        //        if( csgjscpp::lengthsquared( right ) < csgjscpp::csgjs_EPSILON ) {
        //            // WTF, |normal| == 0 ?????
        //            return {};
        //        }
        //        auto up = csgjscpp::cross( normal, right );
        //        right = csgjscpp::unit( right );
        //        up = csgjscpp::unit( up );
        //
        //        for( auto& p: loop ) {
        //            p = TVector { csgjscpp::dot( right, p ), csgjscpp::dot( up, p ), 0 };
        //        }
        //
        //        while( loop.size() > 3 ) {
        //            for( int i = 1; i < loop.size() - 1; i++ ) {
        //                const auto& a = loop[ i - 1 ];
        //                const auto& b = loop[ i ];
        //                const auto& c = loop[ i + 1 ];
        //                if( csgjscpp::cross( b - a, c - b ).z >= 0 ) {
        //                    bool ok = true;
        //                    for( const auto& p: loop ) {
        //                        if( !VectorAdapter<TVector>::IsNearlyEqual( a, p ) && !VectorAdapter<TVector>::IsNearlyEqual( b, p ) &&
        //                            !VectorAdapter<TVector>::IsNearlyEqual( c, p ) && this->IsPointInTriangle( a, b, c, p ) ) {
        //                            ok = false;
        //                            break;
        //                        }
        //                    }
        //                    if( ok ) {
        //                        result.push_back( indices[ i - 1 ] );
        //                        result.push_back( indices[ i ] );
        //                        result.push_back( indices[ i + 1 ] );
        //
        //                        indices.erase( indices.begin() + i );
        //                        loop.erase( loop.begin() + i );
        //                        i--;
        //                    }
        //                }
        //            }
        //        }
        //
        //        if( loop.size() == 3 ) {
        //            result.push_back( indices[ 0 ] );
        //            result.push_back( indices[ 1 ] );
        //            result.push_back( indices[ 2 ] );
        //        }
        //
        //        return result;
        //
        //
        TVector normal;
        for( int i = 1; i < loop.size() - 1; i++ ) {
            normal = normal - csgjscpp::cross( loop[ i - 1 ] - loop[ i ], loop[ i + 1 ] - loop[ i ] );
        }
        normal = csgjscpp::unit( normal );
        auto right = csgjscpp::cross( { 0.0f, 0.0f, 1.0f }, normal );
        if( csgjscpp::lengthsquared( right ) < csgjscpp::csgjs_EPSILON ) {
            right = csgjscpp::cross( { 1.0f, 0.0f, 0.0f }, normal );
        }
        if( csgjscpp::lengthsquared( right ) < csgjscpp::csgjs_EPSILON ) {
            // WTF, |normal| == 0 ?????
            return {};
        }
        auto up = csgjscpp::cross( normal, right );
        right = csgjscpp::unit( right );
        up = csgjscpp::unit( up );
        std::vector<std::vector<std::tuple<float, float>>> polygon;
        std::vector<std::tuple<float, float>> outer;
        for( const auto& p: loop ) {
            outer.emplace_back( csgjscpp::dot( right, p ), csgjscpp::dot( up, p ) );
        }


        polygon.push_back( outer );
        const auto result = mapbox::earcut<int>( polygon );
        return result;
    }

    inline std::vector<TPolygon> ComputeUnion( const std::vector<TPolygon>& operand1, const std::vector<TPolygon>& operand2 ) {
        if( operand1.empty() ) {
            return operand2;
        }
        if( operand2.empty() ) {
            return operand1;
        }
        return csgjscpp::modeltopolygons( csgjscpp::csgunion( csgjscpp::modelfrompolygons( operand1 ), csgjscpp::modelfrompolygons( operand2 ) ) );
    }

    inline std::vector<TPolygon> ComputeIntersection( const std::vector<TPolygon>& operand1, const std::vector<TPolygon>& operand2 ) {
        if( operand1.empty() || operand2.empty() ) {
            return {};
        }
        return csgjscpp::modeltopolygons( csgjscpp::csgintersection( csgjscpp::modelfrompolygons( operand1 ), csgjscpp::modelfrompolygons( operand2 ) ) );
    }

    inline std::vector<TPolygon> ComputeDifference( const std::vector<TPolygon>& operand1, const std::vector<TPolygon>& operand2 ) {
        if( operand1.empty() || operand2.empty() ) {
            return operand1;
        }
        return csgjscpp::modeltopolygons( csgjscpp::csgsubtract( csgjscpp::modelfrompolygons( operand1 ), csgjscpp::modelfrompolygons( operand2 ) ) );
    }
};

}