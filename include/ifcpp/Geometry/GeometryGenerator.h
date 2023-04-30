#pragma once

#include <memory>
#include <vector>

#include "ifcpp/Geometry/CAdapter.h"
#include "ifcpp/Geometry/CurveConverter.h"
#include "ifcpp/Geometry/Extruder.h"
#include "ifcpp/Geometry/GeomUtils.h"
#include "ifcpp/Geometry/GeometryConverter.h"
#include "ifcpp/Geometry/Helpers.h"
#include "ifcpp/Geometry/Matrix.h"
#include "ifcpp/Geometry/Parameters.h"
#include "ifcpp/Geometry/PrimitiveTypesConverter.h"
#include "ifcpp/Geometry/ProfileConverter.h"
#include "ifcpp/Geometry/SolidConverter.h"
#include "ifcpp/Geometry/SplineConverter.h"
#include "ifcpp/Geometry/StyleConverter.h"

#include "ifcpp/Model/BuildingModel.h"
#include "ifcpp/Model/UnitConverter.h"

#include "ifcpp/Ifc/IfcAnnotationFillArea.h"
#include "ifcpp/Ifc/IfcBooleanResult.h"
#include "ifcpp/Ifc/IfcBuiltElement.h"
#include "ifcpp/Ifc/IfcConnectedFaceSet.h"
#include "ifcpp/Ifc/IfcFace.h"
#include "ifcpp/Ifc/IfcFaceBasedSurfaceModel.h"
#include "ifcpp/Ifc/IfcFeatureElementSubtraction.h"
#include "ifcpp/Ifc/IfcGeometricCurveSet.h"
#include "ifcpp/Ifc/IfcGeometricSet.h"
#include "ifcpp/Ifc/IfcMappedItem.h"
#include "ifcpp/Ifc/IfcObjectDefinition.h"
#include "ifcpp/Ifc/IfcOpenShell.h"
#include "ifcpp/Ifc/IfcProductRepresentation.h"
#include "ifcpp/Ifc/IfcRelVoidsElement.h"
#include "ifcpp/Ifc/IfcRepresentation.h"
#include "ifcpp/Ifc/IfcRepresentationMap.h"
#include "ifcpp/Ifc/IfcSectionedSpine.h"
#include "ifcpp/Ifc/IfcShellBasedSurfaceModel.h"
#include "ifcpp/Ifc/IfcSlab.h"
#include "ifcpp/Ifc/IfcSpace.h"
#include "ifcpp/Ifc/IfcTessellatedItem.h"
#include "ifcpp/Ifc/IfcTextLiteral.h"
#include "ifcpp/Ifc/IfcWall.h"
#include "ifcpp/Ifc/IfcWindow.h"
#include "ifcpp/Ifc/IfcDoor.h"


namespace ifcpp {

using namespace IFC4X3;

template<CAdapter TAdapter>
class GeometryGenerator {
    using TEntity = typename TAdapter::TEntity;
    using TVector = typename TAdapter::TVector;
    using TTriangle = typename TAdapter::TTriangle;
    using TPolyline = typename TAdapter::TPolyline;
    using TMesh = typename TAdapter::TMesh;
    using TMatrix = Matrix<TVector>;

    std::shared_ptr<BuildingModel> m_ifcModel;
    std::shared_ptr<TAdapter> m_adapter;

    std::shared_ptr<CurveConverter<TVector>> m_curveConverter;
    std::shared_ptr<Extruder<TVector>> m_extruder;
    std::shared_ptr<GeometryConverter<TVector>> m_geometryConverter;
    std::shared_ptr<GeomUtils<TVector>> m_geomUtils;
    std::shared_ptr<PrimitiveTypesConverter<TVector>> m_primitivesConverter;
    std::shared_ptr<ProfileConverter<TVector>> m_profileConverter;
    std::shared_ptr<SolidConverter<TAdapter>> m_solidConverter;
    std::shared_ptr<SplineConverter<TVector>> m_splineConverter;
    std::shared_ptr<StyleConverter> m_styleConverter;
    std::shared_ptr<Parameters> m_parameters;

public:
    GeometryGenerator( const std::shared_ptr<BuildingModel>& ifcModel, const std::shared_ptr<TAdapter> adapter,
                       const std::shared_ptr<CurveConverter<TVector>>& curveConverter, const std::shared_ptr<Extruder<TVector>>& extruder,
                       const std::shared_ptr<GeometryConverter<TVector>>& geometryConverter, const std::shared_ptr<GeomUtils<TVector>>& geomUtils,
                       const std::shared_ptr<PrimitiveTypesConverter<TVector>>& primitivesConverter,
                       const std::shared_ptr<ProfileConverter<TVector>>& profileConverter, const std::shared_ptr<SolidConverter<TAdapter>>& solidConverter,
                       const std::shared_ptr<SplineConverter<TVector>>& splineConverter, const std::shared_ptr<StyleConverter>& styleConverter,
                       const std::shared_ptr<Parameters>& parameters )
        : m_ifcModel( ifcModel )
        , m_adapter( adapter )
        , m_curveConverter( curveConverter )
        , m_extruder( extruder )
        , m_geometryConverter( geometryConverter )
        , m_geomUtils( geomUtils )
        , m_primitivesConverter( primitivesConverter )
        , m_profileConverter( profileConverter )
        , m_solidConverter( solidConverter )
        , m_splineConverter( splineConverter )
        , m_styleConverter( styleConverter )
        , m_parameters( parameters ) {
        this->m_parameters->m_lengthFactor = (float)ifcModel->getUnitConverter()->getLengthInMeterFactor();
        this->m_parameters->m_angleFactor = (float)ifcModel->getUnitConverter()->getAngleInRadiantFactor();
    }

    std::vector<TEntity> GenerateGeometry() {
        std::vector<TEntity> entities;
        for( const auto& idEntityPair: this->m_ifcModel->getMapIfcEntities() ) {
            auto object = std::dynamic_pointer_cast<IfcObjectDefinition>( idEntityPair.second );
            if( !object ) {
                continue;
            }
            if( std::dynamic_pointer_cast<IfcSpace>( object ) ) {
                continue;
            }
            if( std::dynamic_pointer_cast<IfcFeatureElementSubtraction>( object ) ) {
                continue;
            }
            entities.push_back( GenerateGeometryFromObject( object ) );
        }
        return entities;
    }

private:
    TEntity GenerateGeometryFromObject( const std::shared_ptr<IFC4X3::IfcObjectDefinition>& object ) {
        const auto product = dynamic_pointer_cast<IfcProduct>( object );
        if( !product || !product->m_Representation ) {
            return this->m_adapter->CreateEntity( object, {}, {} );
        }

        std::vector<TMesh> meshes;
        std::vector<TPolyline> polylines;

        for( const auto& item: product->m_Representation->m_Representations ) {
            const auto [ m, p ] = this->ConvertRepresentation( item );
            Helpers::AppendTo( &meshes, m );
            Helpers::AppendTo( &polylines, p );
        }

        auto matrix = this->m_primitivesConverter->ConvertPlacement( product->m_ObjectPlacement );
        TMatrix::Multiply( &matrix,
                           TMatrix::GetScale( this->m_parameters->m_lengthFactor, this->m_parameters->m_lengthFactor, this->m_parameters->m_lengthFactor ) );
        this->m_adapter->Transform( &meshes, matrix );
        this->m_adapter->Transform( &polylines, matrix );

        const auto opening = this->ConvertRelatedOpening( object );
        //return { this->m_adapter->CreateEntity( object, opening, {} ) };
        if( !opening.empty() ) {
            meshes = this->m_adapter->ComputeDifference( meshes, opening );
        }

        if( std::dynamic_pointer_cast<IfcWall>( object ) ) {
            auto style = std::make_shared<Style>();
            style->m_type = Style::SURFACE_FRONT;
            style->m_color = { 231.0f / 255.0f, 219.0f / 255.0f, 169.0f / 255.0f, 1.0f };
            this->m_adapter->AddStyles( &meshes, { style } );
        }
        if( std::dynamic_pointer_cast<IfcSlab>( object ) ) {
            auto style = std::make_shared<Style>();
            style->m_type = Style::SURFACE_FRONT;
            style->m_color = { 140.0f / 255.0f, 140.0f / 255.0f, 140.0f / 255.0f, 1.0f };
            this->m_adapter->AddStyles( &meshes, { style } );
        }

        return this->m_adapter->CreateEntity( object, meshes, polylines );
    }

    std::tuple<std::vector<TMesh>, std::vector<TPolyline>> ConvertRepresentation( const std::shared_ptr<IfcRepresentation>& representation ) {
        std::vector<TMesh> meshes;
        std::vector<TPolyline> polylines;

        for( const auto& item: representation->m_Items ) {
            const auto [ m, p ] = this->ConvertRepresentationItem( item );
            Helpers::AppendTo( &meshes, m );
            Helpers::AppendTo( &polylines, p );
        }

        const auto styles = this->m_styleConverter->GetStyles( representation );
        this->m_adapter->AddStyles( &meshes, styles );
        this->m_adapter->AddStyles( &polylines, styles );

        return { meshes, polylines };
    }

    std::tuple<std::vector<TMesh>, std::vector<TPolyline>> ConvertRepresentationItem( const std::shared_ptr<IfcRepresentationItem>& item ) {
        // ENTITY IfcRepresentationItem  ABSTRACT SUPERTYPE OF(ONEOF(IfcGeometricRepresentationItem,
        // IfcMappedItem, IfcStyledItem, IfcTopologicalRepresentationItem));

        std::vector<TMesh> meshes;
        std::vector<TPolyline> polylines;

        const auto geometric = dynamic_pointer_cast<IfcGeometricRepresentationItem>( item );
        if( geometric ) {
            std::tie( meshes, polylines ) = this->ConvertGeometryRepresentation( geometric );
        }

        const auto mappedItem = dynamic_pointer_cast<IfcMappedItem>( item );
        if( mappedItem ) {
            std::tie( meshes, polylines ) = this->ConvertMappedItem( mappedItem );
        }

        // FIXME: IfcTopologicalRepresentationItem (maybe)

        const auto styles = this->m_styleConverter->GetStyles( item );
        this->m_adapter->AddStyles( &meshes, styles );
        this->m_adapter->AddStyles( &polylines, styles );

        return { meshes, polylines };
    }

    std::tuple<std::vector<TMesh>, std::vector<TPolyline>> ConvertMappedItem( const std::shared_ptr<IfcMappedItem>& mappedItem ) {
        if( !mappedItem->m_MappingSource || !mappedItem->m_MappingSource->m_MappedRepresentation ) {
            // TODO: Log error
            return {};
        }
        const auto transformation = this->m_primitivesConverter->ConvertTransformationOperator( mappedItem->m_MappingTarget );
        const auto origin = this->m_primitivesConverter->ConvertPlacement( mappedItem->m_MappingSource->m_MappingOrigin );
        const auto m = TMatrix::GetMultiplied( transformation, origin );

        auto [ meshes, polylines ] = this->ConvertRepresentation( mappedItem->m_MappingSource->m_MappedRepresentation );

        this->m_adapter->Transform( &meshes, m );
        this->m_adapter->Transform( &polylines, m );

        const auto styles = this->m_styleConverter->GetStyles( mappedItem->m_LayerAssignment_inverse );
        this->m_adapter->AddStyles( &meshes, styles );
        this->m_adapter->AddStyles( &polylines, styles );

        return { meshes, polylines };
    }

    std::vector<TMesh> ConvertRelatedOpening( const std::shared_ptr<IFC4X3::IfcObjectDefinition>& object ) {
        std::vector<TMesh> resultMeshes;
        if( const auto element = std::dynamic_pointer_cast<IfcElement>( object ) ) {
            for( const auto& openingRef: element->m_HasOpenings_inverse ) {
                std::vector<TMesh> meshes;
                const auto openingRel = openingRef.lock();
                if( openingRel ) {
                    const auto opening = openingRel->m_RelatedOpeningElement;

                    if( !opening || !opening->m_Representation ) {
                        continue;
                    }
                    for( const auto& representation: opening->m_Representation->m_Representations ) {
                        auto [ m, p ] = this->ConvertRepresentation( representation );
                        Helpers::AppendTo( &meshes, m );
                    }
                    auto m = this->m_primitivesConverter->ConvertPlacement( opening->m_ObjectPlacement );
                    TMatrix::Multiply(
                        &m, TMatrix::GetScale( this->m_parameters->m_lengthFactor, this->m_parameters->m_lengthFactor, this->m_parameters->m_lengthFactor ) );
                    this->m_adapter->Transform( &meshes, m );
                    Helpers::AppendTo( &resultMeshes, meshes );
                }
            }
        }
        return resultMeshes;
    }

    std::tuple<std::vector<TMesh>, std::vector<TPolyline>>
    ConvertGeometryRepresentation( const std::shared_ptr<IFC4X3::IfcGeometricRepresentationItem>& geometricRepresentation ) {
        // ENTITY IfcGeometricRepresentationItem
        // ABSTRACT SUPERTYPE OF(ONEOF(IfcAnnotationFillArea, IfcBooleanResult, IfcBoundingBox, IfcCartesianPointList, IfcCartesianTransformationOperator,
        // IfcCompositeCurveSegment, IfcCsgPrimitive3D, IfcCurve, IfcDirection, IfcFaceBasedSurfaceModel, IfcFillAreaStyleHatching, IfcFillAreaStyleTiles,
        // IfcGeometricSet, IfcHalfSpaceSolid, IfcLightSource, IfcPlacement, IfcPlanarExtent, IfcPoint, IfcSectionedSpine, IfcShellBasedSurfaceModel,
        // IfcSolidModel, IfcSurface, IfcTessellatedItem, IfcTextLiteral, IfcVector))

        const auto surfaceModel = dynamic_pointer_cast<IfcFaceBasedSurfaceModel>( geometricRepresentation );
        if( surfaceModel ) {
            return this->ConvertSurfaceModel( surfaceModel );
        }

        const auto booleanResult = dynamic_pointer_cast<IfcBooleanResult>( geometricRepresentation );
        if( booleanResult ) {
            return { this->m_solidConverter->ConvertBooleanResult( booleanResult ), {} };
        }

        const auto solidModel = dynamic_pointer_cast<IfcSolidModel>( geometricRepresentation );
        if( solidModel ) {
            return { this->m_solidConverter->ConvertSolidModel( solidModel ), {} };
        }

        const auto curve = dynamic_pointer_cast<IfcCurve>( geometricRepresentation );
        if( curve ) {
            return { {}, { this->m_adapter->CreatePolyline( this->m_curveConverter->ConvertCurve( curve ) ) } };
        }

        const auto shellModel = dynamic_pointer_cast<IfcShellBasedSurfaceModel>( geometricRepresentation );
        if( shellModel ) {
            return this->ConvertShellBasedSurfaceModel( shellModel );
        }

        const auto tessellatedItem = dynamic_pointer_cast<IfcTessellatedItem>( geometricRepresentation );
        if( tessellatedItem ) {
            // TODO: implement
            return {};
        }

        const auto surface = dynamic_pointer_cast<IfcSurface>( geometricRepresentation );
        if( surface ) {
            return this->ConvertSurface( surface );
        }

        const auto geometricSet = dynamic_pointer_cast<IfcGeometricSet>( geometricRepresentation );
        if( geometricSet ) {
            std::vector<TMesh> meshes;
            std::vector<TPolyline> polylines;

            for( const auto& geom_select: geometricSet->m_Elements ) {
                // TYPE IfcGeometricSetSelect = SELECT (IfcPoint, IfcCurve, IfcSurface);
                if( !geom_select ) {
                    continue;
                }

                shared_ptr<IfcPoint> point = dynamic_pointer_cast<IfcPoint>( geom_select );
                if( point ) {
                    continue;
                }

                shared_ptr<IfcCurve> select_curve = dynamic_pointer_cast<IfcCurve>( geom_select );
                if( select_curve ) {
                    const auto [ m, p ] = this->ConvertGeometryRepresentation( select_curve );
                    Helpers::AppendTo( &meshes, m );
                    Helpers::AppendTo( &polylines, p );
                }

                shared_ptr<IfcSurface> select_surface = dynamic_pointer_cast<IfcSurface>( geom_select );
                if( select_surface ) {
                    const auto [ m, p ] = this->ConvertGeometryRepresentation( select_surface );
                    Helpers::AppendTo( &meshes, m );
                    Helpers::AppendTo( &polylines, p );
                }
            }

            shared_ptr<IfcGeometricCurveSet> geometric_curve_set = dynamic_pointer_cast<IfcGeometricCurveSet>( geometricSet );
            if( geometric_curve_set ) {
                // no additional attributes
            }
            return { meshes, polylines };
        }

        const auto sectioned_spine = dynamic_pointer_cast<IfcSectionedSpine>( geometricRepresentation );
        if( sectioned_spine ) {
            // TODO: Implement
            return {};
        }

        const auto text_literal = dynamic_pointer_cast<IfcTextLiteral>( geometricRepresentation );
        if( text_literal ) {
            // TODO: Implement
            return {};
        }

        const auto annotation_fill_area = dynamic_pointer_cast<IfcAnnotationFillArea>( geometricRepresentation );
        if( annotation_fill_area ) {
            // TODO: Implement
            return {};
        }

        shared_ptr<IfcPoint> ifc_point = dynamic_pointer_cast<IfcPoint>( geometricRepresentation );
        if( ifc_point ) {
            // TODO: Implement
            return {};
        }

        // TODO: Implement all
        return {};
    }

    std::tuple<std::vector<TMesh>, std::vector<TPolyline>> ConvertSurfaceModel( const std::shared_ptr<IfcFaceBasedSurfaceModel>& surfaceModel ) {
        std::vector<std::shared_ptr<IfcFace>> faces;
        for( const auto& face_set: surfaceModel->m_FbsmFaces ) {
            // FIXME: Per face materials???
            Helpers::AppendTo( &faces, face_set->m_CfsFaces );
        }
        const auto loops = this->m_geometryConverter->ConvertFaces( faces );
        // TODO: Check points order
        return { { Helpers::CreateMesh( this->m_adapter, loops ) }, {} };
    }

    std::tuple<std::vector<TMesh>, std::vector<TPolyline>> ConvertShellBasedSurfaceModel( const std::shared_ptr<IfcShellBasedSurfaceModel>& surfaceModel ) {
        std::vector<std::shared_ptr<IfcFace>> faces;
        for( const auto& shell: surfaceModel->m_SbsmBoundary ) {
            // FIXME: Per face materials???
            // FIXME: Maybe handle separately IfcClosedShell and IfcOpenShell
            // TYPE IfcShell = SELECT	(IfcClosedShell	,IfcOpenShell)
            shared_ptr<IfcClosedShell> closed_shell = dynamic_pointer_cast<IfcClosedShell>( shell );
            if( closed_shell ) {
                Helpers::AppendTo( &faces, closed_shell->m_CfsFaces );
            }
            shared_ptr<IfcOpenShell> open_shell = dynamic_pointer_cast<IfcOpenShell>( shell );
            if( open_shell ) {
                Helpers::AppendTo( &faces, open_shell->m_CfsFaces );
            }
        }
        const auto loops = this->m_geometryConverter->ConvertFaces( faces );
        // TODO: Check points order
        return { { Helpers::CreateMesh( this->m_adapter, loops ) }, {} };
    }

    std::tuple<std::vector<TMesh>, std::vector<TPolyline>> ConvertSurface( const std::shared_ptr<IfcSurface>& surface ) {
        auto loops = this->m_geometryConverter->ConvertSurface( surface );
        return { { Helpers::CreateMesh( this->m_adapter, loops ) }, {} };
    }
};

}
