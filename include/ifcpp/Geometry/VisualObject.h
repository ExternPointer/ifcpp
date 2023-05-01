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

    static std::shared_ptr<VisualObject<TAdapter>> Create( const std::vector<TMesh>& meshes = {}, const std::vector<TPolyline>& polylines = {},
                                                           const std::vector<std::shared_ptr<Style>>& styles = {} ) {
        return std::make_shared<VisualObject<TAdapter>>( VisualObject<TAdapter> { meshes, polylines, styles } );
    }
    static std::shared_ptr<VisualObject<TAdapter>> CreateEmpty() {
        return VisualObject::Create();
    }
};

}