#pragma once

#include <map>

#include "ifcpp/Model/OpenMPIncludes.h"

#include "ifcpp/Geometry/Style.h"

#include "ifcpp/Ifc/IfcColourRgb.h"
#include "ifcpp/Ifc/IfcColourSpecification.h"
#include "ifcpp/Ifc/IfcCurveStyle.h"
#include "ifcpp/Ifc/IfcDraughtingPreDefinedColour.h"
#include "ifcpp/Ifc/IfcExternallyDefinedSurfaceStyle.h"
#include "ifcpp/Ifc/IfcFillAreaStyle.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcNormalisedRatioMeasure.h"
#include "ifcpp/Ifc/IfcPreDefinedColour.h"
#include "ifcpp/Ifc/IfcPresentationLayerWithStyle.h"
#include "ifcpp/Ifc/IfcRepresentation.h"
#include "ifcpp/Ifc/IfcRepresentationItem.h"
#include "ifcpp/Ifc/IfcStyledItem.h"
#include "ifcpp/Ifc/IfcSurfaceSide.h"
#include "ifcpp/Ifc/IfcSurfaceStyle.h"
#include "ifcpp/Ifc/IfcSurfaceStyleLighting.h"
#include "ifcpp/Ifc/IfcSurfaceStyleRefraction.h"
#include "ifcpp/Ifc/IfcSurfaceStyleRendering.h"
#include "ifcpp/Ifc/IfcSurfaceStyleShading.h"
#include "ifcpp/Ifc/IfcSurfaceStyleWithTextures.h"
#include "ifcpp/Ifc/IfcTextStyle.h"


namespace ifcpp {
using namespace IFC4X3;

class StyleConverter {

    std::map<std::shared_ptr<IfcPresentationStyle>, std::vector<std::shared_ptr<Style>>> m_presentationStyleToStylesMap;
#ifdef ENABLE_OPENMP
    Mutex m_presentationStyleToStylesMapMutex;
#endif

public:
    std::vector<std::shared_ptr<Style>> GetStyles( const shared_ptr<IfcRepresentation>& representation ) {
        return this->GetStyles( representation->m_LayerAssignments_inverse );
    }

    std::vector<std::shared_ptr<Style>> GetStyles( const shared_ptr<IfcRepresentationItem>& item ) {
        std::vector<std::shared_ptr<Style>> result;

        for( const auto& itemRef: item->m_StyledByItem_inverse ) {
            const auto styles = this->ConvertStyledItem( itemRef.lock() );
            std::copy( styles.begin(), styles.end(), std::back_inserter( result ) );
        }

        const auto layerStyles = this->GetStyles( item->m_LayerAssignment_inverse );
        std::copy( layerStyles.begin(), layerStyles.end(), std::back_inserter( result ) );

        return result;
    }

    std::vector<std::shared_ptr<Style>> GetStyles( const std::vector<weak_ptr<IfcPresentationLayerAssignment>>& layers ) {
        std::vector<std::shared_ptr<Style>> result;

        for( auto& layer_assignment_weak: layers ) {
            const auto layer_assignment = layer_assignment_weak.lock();
            if( !layer_assignment ) {
                continue;
            }

            const auto layer_assignment_with_style = dynamic_pointer_cast<IfcPresentationLayerWithStyle>( layer_assignment );
            if( layer_assignment_with_style ) {
                for( const auto& presentation_style: layer_assignment_with_style->m_LayerStyles ) {
                    if( presentation_style ) {
                        const auto styles = this->ConvertPresentationStyle( presentation_style );
                        std::copy( styles.begin(), styles.end(), std::back_inserter( result ) );
                    }
                }
            }
        }

        return result;
    }

    std::vector<std::shared_ptr<Style>> ConvertStyledItem( const std::shared_ptr<IfcStyledItem>& styled_item ) {
        if( !styled_item ) {
            // TODO: Log error
            return {};
        }

        std::vector<std::shared_ptr<Style>> result;

        for( const auto& presentationStyle: styled_item->m_Styles ) {
            if( !presentationStyle ) {
                continue;
            }

            const auto styles = this->ConvertPresentationStyle( presentationStyle );
            std::copy( styles.begin(), styles.end(), std::back_inserter( result ) );
        }

        return result;
    }


    std::vector<std::shared_ptr<Style>> ConvertPresentationStyle( const shared_ptr<IfcPresentationStyle>& presentationStyle ) {
        // ENTITY IfcPresentationStyle	ABSTRACT SUPERTYPE OF(ONEOF(IfcCurveStyle, IfcFillAreaStyle, IfcSurfaceStyle, IfcSymbolStyle, IfcTextStyle));

        {
#ifdef ENABLE_OPENMP
            ScopedLock lock( this->m_presentationStyleToStylesMapMutex );
#endif
            if( this->m_presentationStyleToStylesMap.contains( presentationStyle ) ) {
                return this->m_presentationStyleToStylesMap[ presentationStyle ];
            }
        }

        std::vector<std::shared_ptr<Style>> result;

        const auto curveStyle = dynamic_pointer_cast<IfcCurveStyle>( presentationStyle );
        if( curveStyle ) {
            result = this->ConvertCurveStyle( curveStyle );
        }

        const auto fillAreaStyle = dynamic_pointer_cast<IfcFillAreaStyle>( presentationStyle );
        if( fillAreaStyle ) {
            // TODO: Implement
            result = {};
        }

        const auto surfaceStyle = dynamic_pointer_cast<IfcSurfaceStyle>( presentationStyle );
        if( surfaceStyle ) {
            result = this->ConvertSurfaceStyle( surfaceStyle );
        }

        const auto textStyle = dynamic_pointer_cast<IfcTextStyle>( presentationStyle );
        if( textStyle ) {
            // TODO: Implement
            result = {};
        }

        {
#ifdef ENABLE_OPENMP
            ScopedLock lock( this->m_presentationStyleToStylesMapMutex );
#endif
            this->m_presentationStyleToStylesMap[ presentationStyle ] = result;
        }


        return result;
    }

    std::vector<std::shared_ptr<Style>> ConvertSurfaceStyle( const shared_ptr<IfcSurfaceStyle>& surface_style ) {
        if( !surface_style ) {
            return {};
        }

        const auto result = std::make_shared<Style>();
        result->m_type = Style::SURFACE_FRONT;
        if( surface_style->m_Side && surface_style->m_Side->m_enum == IfcSurfaceSide::ENUM_NEGATIVE ) {
            result->m_type = Style::SURFACE_BACK;
        }
        if( surface_style->m_Side && surface_style->m_Side->m_enum == IfcSurfaceSide::ENUM_BOTH ) {
            result->m_type = Style::SURFACE_BOTH;
        }


        result->m_color = { 1, 1, 1, 1 };

        for( const auto& surf_style_element_select: surface_style->m_Styles ) {
            // TYPE IfcSurfaceStyleElementSelect = SELECT	(IfcExternallyDefinedSurfaceStyle	,IfcSurfaceStyleLighting
            // ,IfcSurfaceStyleRefraction ,IfcSurfaceStyleShading	,IfcSurfaceStyleWithTextures);

            if( !surf_style_element_select ) {
                continue;
            }
            const auto surface_style_shading = dynamic_pointer_cast<IfcSurfaceStyleShading>( surf_style_element_select );
            if( surface_style_shading ) {
                if( surface_style_shading->m_SurfaceColour ) {
                    result->m_color = this->ConvertColorRgb( surface_style_shading->m_SurfaceColour );
                }
                if( surface_style_shading->m_Transparency ) {
                    result->m_color.a = std::max( 0.0f, ( std::min( 1.0f, 1.0f - (float)surface_style_shading->m_Transparency->m_value ) ) );
                }
                continue;
            }

            const auto ext_surf_style = dynamic_pointer_cast<IfcExternallyDefinedSurfaceStyle>( surf_style_element_select );
            if( ext_surf_style ) {
                // TODO: Implement
                continue;
            }

            const auto style_lighting = dynamic_pointer_cast<IfcSurfaceStyleLighting>( surf_style_element_select );
            if( style_lighting ) {
                // TODO: Implement
                continue;
            }

            const auto style_refraction = dynamic_pointer_cast<IfcSurfaceStyleRefraction>( surf_style_element_select );
            if( style_refraction ) {
                // TODO: Implement
                continue;
            }

            const auto style_texture = dynamic_pointer_cast<IfcSurfaceStyleWithTextures>( surf_style_element_select );
            if( style_texture ) {
                // TODO: Implement
                continue;
            }
        }

        return { result };
    }


    std::vector<std::shared_ptr<Style>> ConvertCurveStyle( const std::shared_ptr<IfcCurveStyle>& curve_style ) {
        if( !curve_style ) {
            return {};
        }

        auto result = std::make_shared<Style>();
        result->m_type = Style::CURVE;
        result->m_color = this->ConvertColor( curve_style->m_CurveColour );
        return { result };
    }

    Style::Color ConvertColor( const std::shared_ptr<IfcColour>& ifc_color_select ) {
        // IfcColour = SELECT ( IfcColourSpecification, IfcPreDefinedColour );

        const auto color_spec = dynamic_pointer_cast<IfcColourSpecification>( ifc_color_select );
        if( color_spec ) {
            // ENTITY IfcColourSpecification ABSTRACT SUPERTYPE OF(IfcColourRgb);
            shared_ptr<IfcColourRgb> color_rgb = dynamic_pointer_cast<IfcColourRgb>( color_spec );
            if( color_rgb ) {
                return this->ConvertColorRgb( color_rgb );
            }
        }

        const auto predefined_color = dynamic_pointer_cast<IfcPreDefinedColour>( ifc_color_select );
        if( predefined_color ) {
            // ENTITY IfcPreDefinedColour ABSTRACT SUPERTYPE OF(IfcDraughtingPreDefinedColour)
            const auto draughting_predefined_color = dynamic_pointer_cast<IfcDraughtingPreDefinedColour>( predefined_color );
            if( draughting_predefined_color ) {
                if( draughting_predefined_color->m_Name ) {
                    std::string name = draughting_predefined_color->m_Name->m_value;
                    if( std_iequal( name, "black" ) ) {
                        return { 0.0f, 0.0f, 0.0f, 1.0f };
                    } else if( std_iequal( name, "red" ) ) {
                        return { 1.0, 0.0, 0.0, 1.0 };
                    } else if( std_iequal( name, "green" ) ) {
                        return { 0.0, 1.0, 0.0, 1.0 };
                    } else if( std_iequal( name, "blue" ) ) {
                        return { 0.0, 0.0, 1.0, 1.0 };
                    } else if( std_iequal( name, "yellow" ) ) {
                        return { 1.0, 1.0, 0.0, 1.0 };
                    } else if( std_iequal( name, "magenta" ) ) {
                        return { 1.0, 0.0, 1.0, 1.0 };
                    } else if( std_iequal( name, "cyan" ) ) {
                        return { 0.0, 1.0, 1.0, 1.0 };
                    } else if( std_iequal( name, "white" ) ) {
                        return { 1.0, 1.0, 1.0, 1.0 };
                    }
                }
            }
        }

        // TODO: Log error
        return {};
    }

    Style::Color ConvertColorRgb( const shared_ptr<IfcColourRgb>& color_rgb ) {
        Style::Color color;
        if( color_rgb->m_Red ) {
            color.r = (float)color_rgb->m_Red->m_value;
        }
        if( color_rgb->m_Green ) {
            color.g = (float)color_rgb->m_Green->m_value;
        }
        if( color_rgb->m_Blue ) {
            color.b = (float)color_rgb->m_Blue->m_value;
        }
        color.a = 1.0f;
        return color;
    }

private:
    static bool std_iequal( const std::string& a, const std::string& b ) {
        if( a.size() == b.size() ) {
            return std::equal( a.begin(), a.end(), b.begin(), []( const char l, const char r ) { return std::toupper( l ) == std::toupper( r ); } );
        }
        return false;
    }
};

}