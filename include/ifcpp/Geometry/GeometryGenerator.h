#pragma once

#include <memory>
#include <vector>

#include "ifcpp/Geometry/CAdapter.h"
#include "ifcpp/Geometry/GeometryConverter.h"
#include "ifcpp/Geometry/Matrix.h"
#include "ifcpp/Geometry/Parameters.h"
#include "ifcpp/Geometry/PrimitivesConverter.h"
#include "ifcpp/Geometry/CurveConverter.h"
#include "ifcpp/Model/BuildingModel.h"
#include "ifcpp/Model/UnitConverter.h"

#include "ifcpp/Ifc/IfcConnectedFaceSet.h"
#include "ifcpp/Ifc/IfcFace.h"
#include "ifcpp/Ifc/IfcFaceBasedSurfaceModel.h"
#include "ifcpp/Ifc/IfcFeatureElementSubtraction.h"
#include "ifcpp/Ifc/IfcObjectDefinition.h"
#include "ifcpp/Ifc/IfcProductRepresentation.h"
#include "ifcpp/Ifc/IfcRepresentation.h"
#include "ifcpp/Ifc/IfcSpace.h"


namespace ifcpp {

using namespace IFC4X3;

template<CAdapter TAdapter>
class GeometryGenerator {
    using TEntity = typename TAdapter::TEntity;
    using TVector = typename TAdapter::TVector;
    using TPolygon = typename TAdapter::TPolygon;
    using TPolyline = typename TAdapter::TPolyline;
    using TMatrix = Matrix<TVector>;

    std::shared_ptr<BuildingModel> m_ifcModel;
    std::shared_ptr<TAdapter> m_adapter;
    std::shared_ptr<PrimitivesConverter<TVector>> m_primitivesConverter;
    std::shared_ptr<GeometryConverter<TVector>> m_geometryConverter;
    std::shared_ptr<GeometryConverter<TVector>> m_curveConverter;
    std::shared_ptr<Parameters> m_parameters;

public:
    explicit GeometryGenerator( const std::shared_ptr<BuildingModel>& ifcModel, const std::shared_ptr<TAdapter> adapter )
        : m_ifcModel( ifcModel )
        , m_adapter( adapter ) {
        this->m_parameters = std::make_shared<Parameters>( Parameters {
            (float)ifcModel->getUnitConverter()->getLengthInMeterFactor(),
            (float)ifcModel->getUnitConverter()->getAngleInRadiantFactor(),
            1e-6,
            14,
            5
        } );
        this->m_primitivesConverter = std::make_shared<PrimitivesConverter<TVector>>();
        this->m_curveConverter = std::make_shared<CurveConverter<TVector>>( this->m_primitivesConverter, this->m_parameters );
        this->m_geometryConverter = std::make_shared<GeometryConverter<TVector>>( this->m_curveConverter, this->m_primitivesConverter, this->m_parameters );
    }

    std::vector<TEntity> GenerateGeometry() {
        std::vector<TEntity> entities;
        for( const auto& idEntityPair: this->m_ifcModel->getMapIfcEntities() ) {
            auto object = dynamic_pointer_cast<IfcObjectDefinition>( idEntityPair.second );
            if( !object ) {
                continue;
            }
            if( dynamic_pointer_cast<IfcFeatureElementSubtraction>( object ) ) {
                continue;
            }
            if( dynamic_pointer_cast<IfcSpace>( object ) ) {
                continue;
            }
            auto g = GenerateGeometryFromObject( object );
            // if( !g->m_meshes.empty() ) {
            // SubtractOpenings( g, lengthFactor, angleFactor );
            entities.push_back( g );
            //}
        }
        return entities;
    }

private:
    TEntity GenerateGeometryFromObject( const std::shared_ptr<IFC4X3::IfcObjectDefinition>& object ) {
        std::vector<TPolygon> resultPolygons;
        std::vector<TPolyline> resultPolylines;
        const auto product = dynamic_pointer_cast<IfcProduct>( object );
        if( !product ) {
            return this->m_adapter->CreateEntity( object, resultPolygons, resultPolylines );
        }
        const auto& productRepresentation = product->m_Representation;
        if( !productRepresentation ) {
            return this->m_adapter->CreateEntity( object, resultPolygons, resultPolylines );
        }

        for( const auto& representation: productRepresentation->m_Representations ) {
            for( const auto& item: representation->m_Items ) {
                // ENTITY IfcRepresentationItem  ABSTRACT SUPERTYPE OF(ONEOF(IfcGeometricRepresentationItem,
                // IfcMappedItem, IfcStyledItem, IfcTopologicalRepresentationItem));
                const auto geometric = dynamic_pointer_cast<IfcGeometricRepresentationItem>( item );
                if( geometric ) {
                    auto [ polygons, polylines ] = ConvertGeometryRepresentation( geometric );
                    std::copy( polygons.begin(), polygons.end(), std::back_inserter( resultPolygons ) );
                    std::copy( polylines.begin(), polylines.end(), std::back_inserter( resultPolylines ) );
                }
            }
        }

        auto matrix = TMatrix::GetScale( this->m_parameters->m_lengthFactor, this->m_parameters->m_lengthFactor, this->m_parameters->m_lengthFactor );
        TMatrix::Multiply( &matrix, this->m_geometryConverter->ConvertObjectPlacement( product->m_ObjectPlacement ) );
        this->m_adapter.Transform( &resultPolygons, matrix );
        this->m_adapter.Tranform( &resultPolylines, matrix );

        return this->m_adapter->CreateEntity( object, resultPolygons, resultPolylines );
    }

    std::tuple<std::vector<TPolygon>, std::vector<TPolyline>>
    ConvertGeometryRepresentation( const std::shared_ptr<IFC4X3::IfcGeometricRepresentationItem>& geometricRepresentation ) {
        // ENTITY IfcGeometricRepresentationItem
        // ABSTRACT SUPERTYPE OF(ONEOF(IfcAnnotationFillArea, IfcBooleanResult, IfcBoundingBox, IfcCartesianPointList, IfcCartesianTransformationOperator,
        // IfcCompositeCurveSegment, IfcCsgPrimitive3D, IfcCurve, IfcDirection, IfcFaceBasedSurfaceModel, IfcFillAreaStyleHatching, IfcFillAreaStyleTiles,
        // IfcGeometricSet, IfcHalfSpaceSolid, IfcLightSource, IfcPlacement, IfcPlanarExtent, IfcPoint, IfcSectionedSpine, IfcShellBasedSurfaceModel,
        // IfcSolidModel, IfcSurface, IfcTessellatedItem, IfcTextLiteral, IfcVector))

        // TODO: MATERIAL

        const auto surfaceModel = dynamic_pointer_cast<IfcFaceBasedSurfaceModel>( geometricRepresentation );
        if( surfaceModel ) {
            std::vector<std::shared_ptr<IfcFace>> faces;
            for( const auto& face_set: surfaceModel->m_FbsmFaces ) {
                std::copy( face_set->m_CfsFaces.begin(), face_set->m_CfsFaces.end(), std::back_inserter( faces ) );
            }
            return this->m_geometryConverter->ConvertFaces( faces );
        }

        const auto booleanResult = dynamic_pointer_cast<IfcBooleanResult>( geometricRepresentation );
        if( booleanResult ) {
            return this->ConvertBooleanResult( booleanResult );
        }

        const auto solidModel = dynamic_pointer_cast<IfcSolidModel>( geometricRepresentation );
        if( solidModel ) {
            return this->ConvertSolidModel( solidModel );
        }

        const auto curve = dynamic_pointer_cast<IfcCurve>( geometricRepresentation );
        if( curve ) {
            return { {}, this->m_adapter->CreatePolyline( this->m_curveConverter->ConvertCurve( curve ) ) };
        }

        const auto shellModel = dynamic_pointer_cast<IfcShellBasedSurfaceModel>( geometricRepresentation );
        if( shellModel ) {
            return this->ConvertShellBasedSurfaceModel( shellModel );
        }

        const auto surface = dynamic_pointer_cast<IfcSurface>( geometricRepresentation );
        if( surface ) {
            return this->ConvertSurface( surface );
        }
    }
};

}
