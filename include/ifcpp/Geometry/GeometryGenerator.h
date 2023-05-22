#pragma once

#include <map>
#include <memory>
#include <vector>
#include <functional>
#include <atomic>

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
#include "ifcpp/Geometry/VisualObject.h"

#include "ifcpp/Model/BuildingModel.h"
#include "ifcpp/Model/OpenMPIncludes.h"
#include "ifcpp/Model/UnitConverter.h"

#include "ifcpp/Ifc/IfcAnnotationFillArea.h"
#include "ifcpp/Ifc/IfcBooleanResult.h"
#include "ifcpp/Ifc/IfcBuiltElement.h"
#include "ifcpp/Ifc/IfcConnectedFaceSet.h"
#include "ifcpp/Ifc/IfcDoor.h"
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
    using TVisualObject = VisualObject<TAdapter>;

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

    std::map<std::shared_ptr<IfcRepresentation>, std::vector<std::shared_ptr<TVisualObject>>> m_representationToVisualObjectMap;
    std::map<std::shared_ptr<IfcRepresentationItem>, std::vector<std::shared_ptr<TVisualObject>>> m_representationItemToVisualObjectMap;
#ifdef ENABLE_OPENMP
    Mutex m_representationToVisualObjectMapMutex;
    Mutex m_representationItemToVisualObjectMapMutex;
#endif

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
        this->m_parameters->m_lengthFactor = ifcModel->getUnitConverter()->getLengthInMeterFactor();
        this->m_parameters->m_angleFactor = ifcModel->getUnitConverter()->getAngleInRadiantFactor();
    }

    std::vector<TEntity> GenerateGeometry( const std::function<void(double)>& onProgressChanged, const std::atomic<bool>& isCancellationRequested = false ) {
        this->ResetCaches();
        std::vector<TEntity> entities;
        entities.reserve( this->m_ifcModel->getMapIfcEntities().size() );

        std::vector<std::shared_ptr<BuildingEntity>> ifcEntities;
        ifcEntities.reserve( this->m_ifcModel->getMapIfcEntities().size() );
        for( const auto& idEntityPair: this->m_ifcModel->getMapIfcEntities() ) {
            ifcEntities.push_back( idEntityPair.second );
        }

        const auto ifcEntitiesPtr = &ifcEntities;
        const auto ifcEntitiesCount = ifcEntities.size();
        const auto entitiesPtr = &entities;
        std::atomic<size_t> processedCount = 0;

#ifdef ENABLE_OPENMP
        Mutex entitiesPtrMutex;
#pragma omp parallel default( none ) shared( entitiesPtrMutex, ifcEntitiesCount, ifcEntitiesPtr, entitiesPtr, onProgressChanged, processedCount, isCancellationRequested )
        {
            std::vector<TEntity> entitiesPerThread;
            entitiesPerThread.reserve( 1000 );
#pragma omp for schedule( dynamic, 1000 ) nowait
#endif
            for( int i = 0; i < ifcEntitiesCount; i++ ) {
#ifdef ENABLE_OPENMP
                if( isCancellationRequested ) {
                    continue;
                }
#else
            if( isCancellationRequested ) {
                return {};
            }
#endif
                if( !(++processedCount % 100000) ) {
                    auto progress = (double)processedCount / (double)ifcEntitiesCount;
                    if( progress < 1 ) {
                        onProgressChanged( (double)processedCount / (double)ifcEntitiesCount );
                    }
                }

                const auto object = std::dynamic_pointer_cast<IfcObjectDefinition>( ifcEntitiesPtr->at( i ) );
                if( !object ) {
                    continue;
                }
                if( std::dynamic_pointer_cast<IfcFeatureElementSubtraction>( object ) ) {
                    continue;
                }

                const auto entity = GenerateGeometryFromObject( object );

#ifdef ENABLE_OPENMP
                entitiesPerThread.push_back( entity );
#else
            entitiesPtr->push_back( entity );
#endif
            }

#ifdef ENABLE_OPENMP
            {
                ScopedLock lock( entitiesPtrMutex );
                Helpers::AppendTo( entitiesPtr, entitiesPerThread );
            }
        }
#endif
        if( isCancellationRequested ) {
            return {};
        }
        onProgressChanged( 1 );
        return entities;
    }

    void ResetCaches() {
        {
#ifdef ENABLE_OPENMP
            ScopedLock lock1( this->m_representationToVisualObjectMapMutex );
            ScopedLock lock2( this->m_representationItemToVisualObjectMapMutex );
#endif
            this->m_representationToVisualObjectMap = {};
            this->m_representationItemToVisualObjectMap = {};
        }
        this->m_solidConverter->ResetCaches();
        this->m_profileConverter->ResetCaches();
        this->m_curveConverter->ResetCaches();
        this->m_styleConverter->ResetCaches();
    }

private:
    TEntity GenerateGeometryFromObject( const std::shared_ptr<IFC4X3::IfcObjectDefinition>& object ) {
        const auto product = dynamic_pointer_cast<IfcProduct>( object );
        if( std::dynamic_pointer_cast<IfcSpace>( object ) || !product || !product->m_Representation ) {
            return this->m_adapter->CreateEntity( object, {}, {} );
        }

        std::vector<std::shared_ptr<TVisualObject>> visualObjects;

        for( const auto& item: product->m_Representation->m_Representations ) {
            const auto vo = this->ConvertRepresentation( item );
            Helpers::AppendTo( &visualObjects, vo );
        }

        auto matrix = this->m_primitivesConverter->ConvertPlacement( product->m_ObjectPlacement );

        TMatrix::Multiply( &matrix,
                           TMatrix::GetScale( this->m_parameters->m_lengthFactor, this->m_parameters->m_lengthFactor, this->m_parameters->m_lengthFactor ) );

        const auto opening = this->ConvertRelatedOpening( object );

        std::vector<TMesh> resultMeshes;
        std::vector<TPolyline> resultPolylines;

        auto defaultStyle = this->GetDefaultStyleForObject( object );
        for( auto& vo: visualObjects ) {
            this->m_adapter->Transform( &vo->m_meshes, matrix );
            this->m_adapter->Transform( &vo->m_polylines, matrix );
            vo->m_meshes = this->m_adapter->ComputeDifference( vo->m_meshes, opening );
            vo->AddStyles( { defaultStyle } );
            this->m_adapter->AddStyles( &vo->m_meshes, vo->m_styles );
            this->m_adapter->AddStyles( &vo->m_polylines, vo->m_styles );
            Helpers::AppendTo( &resultMeshes, vo->m_meshes );
            Helpers::AppendTo( &resultPolylines, vo->m_polylines );
        }


        return this->m_adapter->CreateEntity( object, resultMeshes, resultPolylines );
    }

    std::vector<std::shared_ptr<TVisualObject>> ConvertRepresentation( const std::shared_ptr<IfcRepresentation>& representation ) {
        std::vector<std::shared_ptr<TVisualObject>> visualObjects;

        std::vector<std::shared_ptr<TVisualObject>>* cached = nullptr;
        {
#ifdef ENABLE_OPENMP
            ScopedLock lock( this->m_representationToVisualObjectMapMutex );
#endif
            if( this->m_representationToVisualObjectMap.contains( representation ) ) {
                cached = &this->m_representationToVisualObjectMap[ representation ];
            }
        }

        if( cached ) {
            return TVisualObject::Copy( this->m_adapter, *cached );
        }

        for( const auto& item: representation->m_Items ) {
            const auto vo = this->ConvertRepresentationItem( item );
            Helpers::AppendTo( &visualObjects, vo );
        }

        const auto styles = this->m_styleConverter->GetStyles( representation );
        for( auto& vo: visualObjects ) {
            vo->AddStyles( styles );
        }

        auto resultCopy = TVisualObject::Copy( this->m_adapter, visualObjects );
        {
#ifdef ENABLE_OPENMP
            ScopedLock lock( this->m_representationToVisualObjectMapMutex );
#endif
            if( !this->m_representationToVisualObjectMap.contains( representation ) ) {
                this->m_representationToVisualObjectMap[ representation ] = std::move( resultCopy );
            }
        }
        return visualObjects;
    }

    std::vector<std::shared_ptr<TVisualObject>> ConvertRepresentationItem( const std::shared_ptr<IfcRepresentationItem>& item ) {
        // ENTITY IfcRepresentationItem  ABSTRACT SUPERTYPE OF(ONEOF(IfcGeometricRepresentationItem,
        // IfcMappedItem, IfcStyledItem, IfcTopologicalRepresentationItem));


        std::vector<std::shared_ptr<TVisualObject>>* cached = nullptr;
        {
#ifdef ENABLE_OPENMP
            ScopedLock lock( this->m_representationItemToVisualObjectMapMutex );
#endif
            if( this->m_representationItemToVisualObjectMap.contains( item ) ) {
                cached = &this->m_representationItemToVisualObjectMap[ item ];
            }
        }

        if( cached ) {
            return TVisualObject::Copy( this->m_adapter, *cached );
        }

        std::vector<std::shared_ptr<TVisualObject>> visualObjects;

        const auto geometric = dynamic_pointer_cast<IfcGeometricRepresentationItem>( item );
        if( geometric ) {
            visualObjects = this->ConvertGeometryRepresentation( geometric );
        }

        const auto mappedItem = dynamic_pointer_cast<IfcMappedItem>( item );
        if( mappedItem ) {
            visualObjects = this->ConvertMappedItem( mappedItem );
        }

        // FIXME: IfcTopologicalRepresentationItem (maybe)

        const auto styles = this->m_styleConverter->GetStyles( item->m_LayerAssignment_inverse );
        for( auto& vo: visualObjects ) {
            vo->AddStyles( styles );
        }

        auto resultCopy = TVisualObject::Copy( this->m_adapter, visualObjects );
        {
#ifdef ENABLE_OPENMP
            ScopedLock lock( this->m_representationItemToVisualObjectMapMutex );
#endif
            if( !this->m_representationItemToVisualObjectMap.contains( item ) ) {
                this->m_representationItemToVisualObjectMap[ item ] = std::move( resultCopy );
            }
        }
        return visualObjects;
    }

    std::vector<std::shared_ptr<TVisualObject>> ConvertMappedItem( const std::shared_ptr<IfcMappedItem>& mappedItem ) {
        if( !mappedItem->m_MappingSource || !mappedItem->m_MappingSource->m_MappedRepresentation ) {
            // TODO: Log error
            return {};
        }
        const auto transformation = this->m_primitivesConverter->ConvertTransformationOperator( mappedItem->m_MappingTarget );
        const auto origin = this->m_primitivesConverter->ConvertPlacement( mappedItem->m_MappingSource->m_MappingOrigin );
        const auto m = TMatrix::GetMultiplied( transformation, origin );

        auto visualObjects = this->ConvertRepresentation( mappedItem->m_MappingSource->m_MappedRepresentation );

        const auto styles = this->m_styleConverter->GetStyles( mappedItem->m_LayerAssignment_inverse );
        for( auto& vo: visualObjects ) {
            this->m_adapter->Transform( &vo->m_meshes, m );
            this->m_adapter->Transform( &vo->m_polylines, m );
            vo->AddStyles( styles );
        }

        return visualObjects;
    }

    std::vector<TMesh> ConvertRelatedOpening( const std::shared_ptr<IFC4X3::IfcObjectDefinition>& object ) {
        std::vector<TMesh> resultMeshes;
        if( const auto element = std::dynamic_pointer_cast<IfcElement>( object ) ) {
            for( const auto& openingRef: element->m_HasOpenings_inverse ) {
                const auto openingRel = openingRef.lock();
                if( openingRel ) {
                    const auto opening = openingRel->m_RelatedOpeningElement;

                    if( !opening || !opening->m_Representation ) {
                        continue;
                    }
                    std::vector<TMesh> meshes;
                    for( const auto& representation: opening->m_Representation->m_Representations ) {
                        auto visualObjects = this->ConvertRepresentation( representation );
                        for( const auto& vo: visualObjects ) {
                            Helpers::AppendTo( &meshes, vo->m_meshes );
                        }
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

    std::vector<std::shared_ptr<TVisualObject>>
    ConvertGeometryRepresentation( const std::shared_ptr<IFC4X3::IfcGeometricRepresentationItem>& geometricRepresentation ) {
        // ENTITY IfcGeometricRepresentationItem
        // ABSTRACT SUPERTYPE OF(ONEOF(IfcAnnotationFillArea, IfcBooleanResult, IfcBoundingBox, IfcCartesianPointList, IfcCartesianTransformationOperator,
        // IfcCompositeCurveSegment, IfcCsgPrimitive3D, IfcCurve, IfcDirection, IfcFaceBasedSurfaceModel, IfcFillAreaStyleHatching, IfcFillAreaStyleTiles,
        // IfcGeometricSet, IfcHalfSpaceSolid, IfcLightSource, IfcPlacement, IfcPlanarExtent, IfcPoint, IfcSectionedSpine, IfcShellBasedSurfaceModel,
        // IfcSolidModel, IfcSurface, IfcTessellatedItem, IfcTextLiteral, IfcVector))

        std::vector<std::shared_ptr<TVisualObject>> result;

        const auto surfaceModel = dynamic_pointer_cast<IfcFaceBasedSurfaceModel>( geometricRepresentation );
        if( surfaceModel ) {
            result = this->ConvertSurfaceModel( surfaceModel );
        }

        const auto booleanResult = dynamic_pointer_cast<IfcBooleanResult>( geometricRepresentation );
        if( booleanResult ) {
            result = this->m_solidConverter->ConvertBooleanResult( booleanResult );
        }

        const auto solidModel = dynamic_pointer_cast<IfcSolidModel>( geometricRepresentation );
        if( solidModel ) {
            result = this->m_solidConverter->ConvertSolidModel( solidModel );
        }

        const auto curve = dynamic_pointer_cast<IfcCurve>( geometricRepresentation );
        if( curve ) {
            result = { TVisualObject::Create( {}, { this->m_adapter->CreatePolyline( this->m_curveConverter->ConvertCurve( curve ) ) } ) };
        }

        const auto shellModel = dynamic_pointer_cast<IfcShellBasedSurfaceModel>( geometricRepresentation );
        if( shellModel ) {
            result = this->ConvertShellBasedSurfaceModel( shellModel );
        }

        const auto tessellatedItem = dynamic_pointer_cast<IfcTessellatedItem>( geometricRepresentation );
        if( tessellatedItem ) {
            // TODO: implement
            result = {};
        }

        const auto surface = dynamic_pointer_cast<IfcSurface>( geometricRepresentation );
        if( surface ) {
            result = this->ConvertSurface( surface );
        }

        const auto geometricSet = dynamic_pointer_cast<IfcGeometricSet>( geometricRepresentation );
        if( geometricSet ) {
            std::vector<std::shared_ptr<TVisualObject>> visualObjects;

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
                    const auto vo = this->ConvertGeometryRepresentation( select_curve );
                    Helpers::AppendTo( &visualObjects, vo );
                }

                shared_ptr<IfcSurface> select_surface = dynamic_pointer_cast<IfcSurface>( geom_select );
                if( select_surface ) {
                    const auto vo = this->ConvertGeometryRepresentation( select_surface );
                    Helpers::AppendTo( &visualObjects, vo );
                }
            }

            shared_ptr<IfcGeometricCurveSet> geometric_curve_set = dynamic_pointer_cast<IfcGeometricCurveSet>( geometricSet );
            if( geometric_curve_set ) {
                // no additional attributes
            }
            result = visualObjects;
        }

        const auto sectioned_spine = dynamic_pointer_cast<IfcSectionedSpine>( geometricRepresentation );
        if( sectioned_spine ) {
            // TODO: Implement
            result = {};
        }

        const auto text_literal = dynamic_pointer_cast<IfcTextLiteral>( geometricRepresentation );
        if( text_literal ) {
            // TODO: Implement
            result = {};
        }

        const auto annotation_fill_area = dynamic_pointer_cast<IfcAnnotationFillArea>( geometricRepresentation );
        if( annotation_fill_area ) {
            // TODO: Implement
            result = {};
        }

        shared_ptr<IfcPoint> ifc_point = dynamic_pointer_cast<IfcPoint>( geometricRepresentation );
        if( ifc_point ) {
            // TODO: Implement
            result = {};
        }

        const auto styles = this->m_styleConverter->GetStyles( geometricRepresentation );
        for( auto& r: result ) {
            r->AddStyles( styles );
        }

        // TODO: Implement all
        return result;
    }

    std::vector<std::shared_ptr<TVisualObject>> ConvertSurfaceModel( const std::shared_ptr<IfcFaceBasedSurfaceModel>& surfaceModel ) {
        std::vector<std::shared_ptr<IfcFace>> faces;
        for( const auto& face_set: surfaceModel->m_FbsmFaces ) {
            // FIXME: Per face materials???
            Helpers::AppendTo( &faces, face_set->m_CfsFaces );
        }
        const auto loops = this->m_geometryConverter->ConvertFaces( faces );
        // TODO: Check points order
        return { TVisualObject::Create( { Helpers::CreateMesh( this->m_adapter, loops ) } ) };
    }

    std::vector<std::shared_ptr<TVisualObject>> ConvertShellBasedSurfaceModel( const std::shared_ptr<IfcShellBasedSurfaceModel>& surfaceModel ) {
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
        return { TVisualObject::Create( { Helpers::CreateMesh( this->m_adapter, loops ) } ) };
    }

    std::vector<std::shared_ptr<TVisualObject>> ConvertSurface( const std::shared_ptr<IfcSurface>& surface ) {
        auto loops = this->m_geometryConverter->ConvertSurface( surface );
        return { TVisualObject::Create( { Helpers::CreateMesh( this->m_adapter, loops ) } ) };
    }

    std::shared_ptr<Style> GetDefaultStyleForObject( const std::shared_ptr<IfcObjectDefinition>& object ) {
        auto style = std::make_shared<Style>();
        style->m_type = Style::SURFACE_BOTH;
        style->m_color = { 1.0, 1.0, 1.0, 1.0 };

        if( std::dynamic_pointer_cast<IfcWall>( object ) ) {
            style->m_color = { 231.0 / 255.0, 219.0 / 255.0, 169.0 / 255.0, 1.0 };
        }
        if( std::dynamic_pointer_cast<IfcSlab>( object ) ) {
            style->m_color = { 140.0 / 255.0, 140.0 / 255.0, 140.0 / 255.0, 1.0 };
        }
        return style;
    }
};
}
