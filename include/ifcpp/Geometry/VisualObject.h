#pragma once

#include <memory>
#include <vector>

#include "ifcpp/Geometry/CAdapter.h"
#include "ifcpp/Geometry/Style.h"

namespace ifcpp {

template<CAdapter TAdapter>
class VisualObject {
    using TMesh = typename TAdapter::TMesh;
    using TPolyline = typename TAdapter::TPolyline;

public:
    std::vector<TMesh> m_meshes;
    std::vector<TPolyline> m_polylines;
    std::vector<std::shared_ptr<Style>> m_styles;
    void AddStyles( const std::vector<std::shared_ptr<Style>>& styles ) {
        std::copy( styles.begin(), styles.end(), std::back_inserter( this->m_styles ) );
    }

    std::shared_ptr<VisualObject<TAdapter>> Copy( const std::shared_ptr<TAdapter>& adapter ) const {
        auto result = VisualObject::Create();
        result->m_meshes.reserve( this->m_meshes.size() );
        result->m_polylines.reserve( this->m_polylines.size() );
        for( const auto& m: this->m_meshes ) {
            result->m_meshes.push_back( adapter->CreateMesh( m ) );
        }
        for( const auto& p: this->m_polylines ) {
            result->m_polylines.push_back( adapter->CreatePolyline( p ) );
        }
        result->m_styles = this->m_styles;
        return result;
    }
    static std::vector<std::shared_ptr<VisualObject<TAdapter>>> Copy( const std::shared_ptr<TAdapter>& adapter,
                                                                      const std::vector<std::shared_ptr<VisualObject<TAdapter>>>& visualObjects ) {
        std::vector<std::shared_ptr<VisualObject<TAdapter>>> result;
        result.reserve( visualObjects.size() );
        for( const auto& vo: visualObjects ) {
            result.push_back( vo->Copy( adapter ) );
        }
        return result;
    }

    static std::shared_ptr<VisualObject<TAdapter>> Create( const std::vector<TMesh>& meshes = {}, const std::vector<TPolyline>& polylines = {},
                                                           const std::vector<std::shared_ptr<Style>>& styles = {} ) {
        return std::make_shared<VisualObject<TAdapter>>( VisualObject<TAdapter> { meshes, polylines, styles } );
    }
    static std::shared_ptr<VisualObject<TAdapter>> CreateEmpty() {
        return VisualObject::Create();
    }
};

}
