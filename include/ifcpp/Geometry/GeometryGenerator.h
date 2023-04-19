#pragma once

#include <memory>
#include <vector>

#include "ifcpp/Geometry/CAdapter.h"
#include "ifcpp/Geometry/CurveConverter.h"
#include "ifcpp/Geometry/GeometryConverter.h"
#include "ifcpp/Geometry/Matrix.h"
#include "ifcpp/Geometry/Parameters.h"
#include "ifcpp/Geometry/PrimitivesConverter.h"
#include "ifcpp/Geometry/SolidConverter.h"

#include "ifcpp/Model/BuildingModel.h"
#include "ifcpp/Model/UnitConverter.h"

#include "ifcpp/Ifc/IfcBooleanResult.h"
#include "ifcpp/Ifc/IfcConnectedFaceSet.h"
#include "ifcpp/Ifc/IfcFace.h"
#include "ifcpp/Ifc/IfcFaceBasedSurfaceModel.h"
#include "ifcpp/Ifc/IfcFeatureElementSubtraction.h"
#include "ifcpp/Ifc/IfcObjectDefinition.h"
#include "ifcpp/Ifc/IfcOpenShell.h"
#include "ifcpp/Ifc/IfcProductRepresentation.h"
#include "ifcpp/Ifc/IfcRelVoidsElement.h"
#include "ifcpp/Ifc/IfcRepresentation.h"
#include "ifcpp/Ifc/IfcShellBasedSurfaceModel.h"
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
    std::shared_ptr<SolidConverter<TAdapter>> m_solidConverter;
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
            5,
            10000,
            4,
        } );
        this->m_primitivesConverter = std::make_shared<PrimitivesConverter<TVector>>();
        this->m_curveConverter = std::make_shared<CurveConverter<TVector>>( this->m_primitivesConverter, this->m_parameters );
        this->m_geometryConverter = std::make_shared<GeometryConverter<TVector>>( this->m_curveConverter, this->m_primitivesConverter, this->m_parameters );
        this->m_solidConverter =
            std::make_shared<SolidConverter<TAdapter>>( this->m_primitivesConverter, this->m_curveConverter, this->m_profileConverter, this->m_extruder,
                                                        this->m_geometryConverter, this->m_adapter, this->m_geomUtils, this->m_parameters );
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

        // Subtract openings
        if( const auto element = std::dynamic_pointer_cast<IfcElement>( object ) ) {
            for( const auto& openingRef: element->m_HasOpenings_inverse ) {
                const auto openingRel = openingRef.lock();
                if( openingRel ) {
                    const auto opening = openingRel->m_RelatedOpeningElement;
                    auto [ voidPolygons, voidPolylines ] = ConvertGeometryRepresentation( opening );
                    resultPolygons = this->m_adapter->ComputeDifference( resultPolygons, voidPolygons );
                }
            }
        }

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

    std::tuple<std::vector<TPolygon>, std::vector<TPolyline>> ConvertSurfaceModel( const std::shared_ptr<IfcFaceBasedSurfaceModel>& surfaceModel ) {
        std::vector<std::shared_ptr<IfcFace>> faces;
        for( const auto& face_set: surfaceModel->m_FbsmFaces ) {
            std::copy( face_set->m_CfsFaces.begin(), face_set->m_CfsFaces.end(), std::back_inserter( faces ) );
        }
        std::vector<std::vector<TVector>> loops = this->m_geometryConverter->ConvertFaces( faces );
        return { this->CreatePolygons( loops ), {} };
    }

    std::tuple<std::vector<TPolygon>, std::vector<TPolyline>>
    ConvertShellBasedSurfaceModel( const std::shared_ptr<IfcShellBasedSurfaceModel>& shell_based_surface_model ) {
        std::vector<std::shared_ptr<IfcFace>> faces;
        for( const auto& shell_select: shell_based_surface_model->m_SbsmBoundary ) {
            // TYPE IfcShell = SELECT	(IfcClosedShell	,IfcOpenShell)
            shared_ptr<IfcClosedShell> closed_shell = dynamic_pointer_cast<IfcClosedShell>( shell_select );
            if( closed_shell ) {
                std::copy( closed_shell->m_CfsFaces.begin(), closed_shell->m_CfsFaces.end(), std::back_inserter( faces ) );
            }
            shared_ptr<IfcOpenShell> open_shell = dynamic_pointer_cast<IfcOpenShell>( shell_select );
            if( open_shell ) {
                std::copy( open_shell->m_CfsFaces.begin(), open_shell->m_CfsFaces.end(), std::back_inserter( faces ) );
            }
        }
        std::vector<std::vector<TVector>> loops = this->m_geometryConverter->ConvertFaces( faces );
        return { this->CreatePolygons( loops ), {} };
    }

    std::tuple<std::vector<TPolygon>, std::vector<TPolyline>> ConvertSurface( const std::shared_ptr<IfcSurface>& surface ) {
        const auto loops = this->m_geometryConverter->ConvertSurface( surface );
        return { this->CreatePolygons( loops ), {} };
    }

    std::vector<TPolygon> CreatePolygons( std::vector<std::vector<TVector>> loops ) {
        std::vector<TPolygon> result;
        for( const auto& l: loops ) {
            const auto indices = this->m_adapter->Triangulate( l );
            for( int i = 0; i < indices.size() - 2; i++ ) {
                result.push_back( this->m_adapter->CreatePolygon( l, { i, i + 1, i + 2 } ) );
            }
        }
        return result;
    }
};

}
