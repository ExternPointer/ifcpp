#pragma once

#include <vector>

#include "ifcpp/Geometry/CAdapter.h"


namespace ifcpp {

class Helpers {
public:
    template<typename T>
    static std::vector<T> GetConcatenated( const std::vector<T>& v1, const std::vector<T>& v2 ) {
        std::vector<T> result = v1;
        result.reserve( v1.size() + v2.size() );
        std::copy( v2.begin(), v2.end(), std::back_inserter( result ) );
        return result;
    }

    template<typename T>
    static void AppendTo( std::vector<T>* target, const std::vector<T>& toAppend ) {
        target->reserve( target->size() + toAppend.size() );
        std::copy( toAppend.begin(), toAppend.end(), std::back_inserter( *target ) );
    }

    template<CAdapter TAdapter>
    static std::vector<typename TAdapter::TTriangle> CreateTriangles( const std::shared_ptr<TAdapter> adapter,
                                                                      const std::vector<std::vector<typename TAdapter::TVector>>& loops ) {
        std::vector<typename TAdapter::TTriangle> result;
        for( const auto& l: loops ) {
            if( l.size() < 3 ) {
                // WTF
                continue;
            }
            const auto indices = adapter->Triangulate( l );
            if( indices.size() < 3 ) {
                continue;
            }
            for( int i = 0; i < indices.size() - 2; i += 3 ) {
                result.push_back( adapter->CreateTriangle( l, { indices[ i ], indices[ i + 1 ], indices[ i + 2 ] } ) );
            }
        }
        return result;
    }

    template<CAdapter TAdapter>
    static typename TAdapter::TMesh CreateMesh( const std::shared_ptr<TAdapter> adapter, const std::vector<std::vector<typename TAdapter::TVector>>& loops ) {
        return adapter->CreateMesh( Helpers::CreateTriangles( adapter, loops ) );
    }
};

}