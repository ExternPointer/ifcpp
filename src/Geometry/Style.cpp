#include "ifcpp/Geometry/Style.h"

#include "ifcpp/Ifc/IfcColour.h"
#include "ifcpp/Ifc/IfcColourRgb.h"
#include "ifcpp/Ifc/IfcColourSpecification.h"
#include "ifcpp/Ifc/IfcDraughtingPreDefinedColour.h"
#include "ifcpp/Ifc/IfcExternallyDefinedHatchStyle.h"
#include "ifcpp/Ifc/IfcFillAreaStyle.h"
#include "ifcpp/Ifc/IfcFillAreaStyleHatching.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcNormalisedRatioMeasure.h"
#include "ifcpp/Ifc/IfcPreDefinedColour.h"
#include "ifcpp/Ifc/IfcPresentationStyle.h"
#include "ifcpp/Ifc/IfcRepresentationItem.h"
#include "ifcpp/Ifc/IfcStyledItem.h"
#include "ifcpp/Ifc/IfcSurfaceStyle.h"
#include "ifcpp/Ifc/IfcSurfaceStyleRendering.h"
#include "ifcpp/Ifc/IfcSurfaceStyleShading.h"

namespace ifcpp {
using namespace IFC4X3;
bool std_iequal( const std::string& a, const std::string& b ) {
    if( a.size() == b.size() ) {
        return std::equal( a.begin(), a.end(), b.begin(), []( const char l, const char r ) { return std::toupper( l ) == std::toupper( r ); } );
    }
    return false;
}

unsigned int rgbaToColor( float r, float g, float b, float a ) {
    return ( int( r * 255 ) << 24 ) | ( int( g * 255 ) << 16 ) | ( int( b * 255 ) << 8 ) | int( a * 255 );
}
unsigned int convertIfcColour( const std::shared_ptr<IfcColour>& ifc_color_select ) {
    unsigned int color = ( int( 0.2 * 255 ) << 24 ) | ( int( 0.25 * 255 ) << 16 ) | ( int( 0.3 * 255 ) << 8 ) | int( 1.0 * 255 );
    // IfcColour = SELECT ( IfcColourSpecification, IfcPreDefinedColour );
    shared_ptr<IfcColourSpecification> color_spec = dynamic_pointer_cast<IfcColourSpecification>( ifc_color_select );
    if( color_spec ) {
        // ENTITY IfcColourSpecification ABSTRACT SUPERTYPE OF(IfcColourRgb);
        shared_ptr<IfcColourRgb> color_rgb = dynamic_pointer_cast<IfcColourRgb>( color_spec );
        if( color_rgb ) {
            color = ( int( (float)color_rgb->m_Red->m_value * 255 ) << 24 ) | ( int( (float)color_rgb->m_Green->m_value * 255 ) << 16 ) |
                ( int( (float)color_rgb->m_Blue->m_value * 255 ) << 8 ) | int( 1.0 * 255 );
        }
    }

    shared_ptr<IfcPreDefinedColour> predefined_color = dynamic_pointer_cast<IfcPreDefinedColour>( ifc_color_select );
    if( predefined_color ) {
        // ENTITY IfcPreDefinedColour ABSTRACT SUPERTYPE OF(IfcDraughtingPreDefinedColour)
        shared_ptr<IfcDraughtingPreDefinedColour> draughting_predefined_color = dynamic_pointer_cast<IfcDraughtingPreDefinedColour>( predefined_color );
        if( draughting_predefined_color ) {
            if( draughting_predefined_color->m_Name ) {
                std::string predefined_name = draughting_predefined_color->m_Name->m_value;
                if( std_iequal( predefined_name, "black" ) )
                    color = rgbaToColor( 0.0, 0.0, 0.0, 1.0 );
                else if( std_iequal( predefined_name, "red" ) )
                    color = rgbaToColor( 1.0, 0.0, 0.0, 1.0 );
                else if( std_iequal( predefined_name, "green" ) )
                    color = rgbaToColor( 0.0, 1.0, 0.0, 1.0 );
                else if( std_iequal( predefined_name, "blue" ) )
                    color = rgbaToColor( 0.0, 0.0, 1.0, 1.0 );
                else if( std_iequal( predefined_name, "yellow" ) )
                    color = rgbaToColor( 1.0, 1.0, 0.0, 1.0 );
                else if( std_iequal( predefined_name, "magenta" ) )
                    color = rgbaToColor( 1.0, 0.0, 1.0, 1.0 );
                else if( std_iequal( predefined_name, "cyan" ) )
                    color = rgbaToColor( 0.0, 1.0, 1.0, 1.0 );
                else if( std_iequal( predefined_name, "white" ) )
                    color = rgbaToColor( 1.0, 1.0, 1.0, 1.0 );
            }
        }
    }
    return color;
}
unsigned int convertIfcSurfaceStyleShading( const std::shared_ptr<IfcSurfaceStyleShading>& surface_style_shading ) {
    if( surface_style_shading ) {
        unsigned int color = 0;
        if( surface_style_shading->m_SurfaceColour ) {
            shared_ptr<IfcColourRgb> surf_color = surface_style_shading->m_SurfaceColour;
            color = convertIfcColour( surf_color );
        }


        shared_ptr<IfcSurfaceStyleRendering> surf_style_rendering = dynamic_pointer_cast<IfcSurfaceStyleRendering>( surface_style_shading );
        if( surf_style_rendering ) {

            if( surf_style_rendering->m_Transparency ) {
                // in IFC 1 is transparent, 0 is opaque. if not given, the value 0 (opaque) is assumed
                // in osg, 1 is opaque, 0 is transparent
                float transparency = 1.f - (float)surf_style_rendering->m_Transparency->m_value;
                if( transparency < 0.1f ) {
                    transparency = 0.1f;
                }

                if( transparency > 1.f ) {
                    transparency = 1.f;
                }

                color = ( ( color >> 8 ) << 8 ) | int( transparency * 255 );
            }
        }

        return color;
    }
    return 0;
}

unsigned int convertIfcSurfaceStyle( const std::shared_ptr<IfcSurfaceStyle>& surface_style ) {
    if( !surface_style ) {
        return 0;
    }
    const int style_id = surface_style->m_tag;

    std::vector<shared_ptr<IfcSurfaceStyleElementSelect>>& vec_styles = surface_style->m_Styles;
    if( vec_styles.empty() ) {
        return 0;
    }

    for(const auto& surf_style_element_select : vec_styles) {
        if( !surf_style_element_select ) {
            continue;
        }
        // TYPE IfcSurfaceStyleElementSelect = SELECT	(IfcExternallyDefinedSurfaceStyle	,IfcSurfaceStyleLighting	,IfcSurfaceStyleRefraction
        // ,IfcSurfaceStyleShading	,IfcSurfaceStyleWithTextures);
        shared_ptr<IfcSurfaceStyleShading> surface_style_shading = dynamic_pointer_cast<IfcSurfaceStyleShading>( surf_style_element_select );
        if( surface_style_shading ) {
            return convertIfcSurfaceStyleShading( surface_style_shading );
        }
    }
    return 0;
}

unsigned int convertIfcPresentationStyle( const std::shared_ptr<IfcPresentationStyle>& presentation_style ) {
    int style_id = presentation_style->m_tag;


    shared_ptr<IfcFillAreaStyle> fill_area_style = dynamic_pointer_cast<IfcFillAreaStyle>( presentation_style );
    if( fill_area_style ) {
        for( shared_ptr<IfcFillStyleSelect>& fillStyle: fill_area_style->m_FillStyles ) {

            shared_ptr<IfcExternallyDefinedHatchStyle> externalStyle = dynamic_pointer_cast<IfcExternallyDefinedHatchStyle>( fillStyle );
            if( externalStyle ) {
                continue;
            }

            shared_ptr<IfcColour> AreaColour = dynamic_pointer_cast<IfcColour>( fillStyle );
            if( AreaColour ) {
                return convertIfcColour( AreaColour );
            }
        }
    }

    shared_ptr<IfcSurfaceStyle> surface_style = dynamic_pointer_cast<IfcSurfaceStyle>( presentation_style );
    if( surface_style ) {
        return convertIfcSurfaceStyle( surface_style );
    }

    return 0;
}

unsigned int convertIfcStyledItem( const weak_ptr<IfcStyledItem>& styled_item_weak ) {
    if( styled_item_weak.expired() ) {
        return 0;
    }
    shared_ptr<IfcStyledItem> styled_item( styled_item_weak );
    const int style_id = styled_item->m_tag;


    std::vector<shared_ptr<IfcPresentationStyle>>& vec_style_assigns = styled_item->m_Styles;
    for( const auto& presentationStyle: vec_style_assigns ) {
        if( !presentationStyle ) {
            continue;
        }

        auto color = convertIfcPresentationStyle( presentationStyle );
        if( color ) {
            return color;
        }
    }
    return 0;
}

unsigned int convertRepresentationStyle( const shared_ptr<IfcRepresentationItem>& representation_item ) {
    std::vector<weak_ptr<IfcStyledItem>>& vec_StyledByItem_inverse = representation_item->m_StyledByItem_inverse;
    for( const auto& styled_item_weak: vec_StyledByItem_inverse ) {
        shared_ptr<IfcStyledItem> styled_item = shared_ptr<IfcStyledItem>( styled_item_weak );
        return convertIfcStyledItem( styled_item );
    }
    return 0;
}
}