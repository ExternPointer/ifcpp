#pragma once

#include <memory>
#include <vector>

#include "ifcpp/Geometry/CAdapter.h"
#include "ifcpp/Geometry/CurveConverter.h"
#include "ifcpp/Geometry/Extruder.h"
#include "ifcpp/Geometry/GeomUtils.h"
#include "ifcpp/Geometry/GeometryConverter.h"
#include "ifcpp/Geometry/Matrix.h"
#include "ifcpp/Geometry/Parameters.h"
#include "ifcpp/Geometry/PrimitivesConverter.h"
#include "ifcpp/Geometry/ProfileConverter.h"
#include "ifcpp/Geometry/SolidConverter.h"
#include "ifcpp/Geometry/SplineConverter.h"

#include "ifcpp/Model/BuildingModel.h"
#include "ifcpp/Model/UnitConverter.h"

#include "ifcpp/Ifc/IfcBooleanResult.h"
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
#include "ifcpp/Ifc/IfcSpace.h"
#include "ifcpp/Ifc/IfcTessellatedItem.h"
#include "ifcpp/Ifc/IfcTextLiteral.h"
#include "ifcpp/Ifc/IfcAnnotationFillArea.h"


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

    std::shared_ptr<CurveConverter<TVector>> m_curveConverter;
    std::shared_ptr<Extruder<TVector>> m_extruder;
    std::shared_ptr<GeometryConverter<TVector>> m_geometryConverter;
    std::shared_ptr<GeomUtils<TVector>> m_geomUtils;
    std::shared_ptr<PrimitivesConverter<TVector>> m_primitivesConverter;
    std::shared_ptr<ProfileConverter<TVector>> m_profileConverter;
    std::shared_ptr<SolidConverter<TAdapter>> m_solidConverter;
    std::shared_ptr<SplineConverter<TVector>> m_splineConverter;
    std::shared_ptr<Parameters> m_parameters;

public:
    GeometryGenerator( const std::shared_ptr<BuildingModel>& ifcModel, const std::shared_ptr<TAdapter> adapter,
                       const std::shared_ptr<CurveConverter<TVector>>& curveConverter, const std::shared_ptr<Extruder<TVector>>& extruder,
                       const std::shared_ptr<GeometryConverter<TVector>>& geometryConverter, const std::shared_ptr<GeomUtils<TVector>>& geomUtils,
                       const std::shared_ptr<PrimitivesConverter<TVector>>& primitivesConverter,
                       const std::shared_ptr<ProfileConverter<TVector>>& profileConverter, const std::shared_ptr<SolidConverter<TAdapter>>& solidConverter,
                       const std::shared_ptr<SplineConverter<TVector>>& splineConverter, const std::shared_ptr<Parameters>& parameters )
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
        , m_parameters( parameters ) {
        this->m_parameters->m_lengthFactor = (float)ifcModel->getUnitConverter()->getLengthInMeterFactor();
        this->m_parameters->m_angleFactor = (float)ifcModel->getUnitConverter()->getAngleInRadiantFactor();
    }

    std::vector<TEntity> GenerateGeometry() {

        //        std::vector<TVector> outer = {
        //            VectorAdapter<TVector>::New( 0, 0 ),
        //            VectorAdapter<TVector>::New( 5, 0 ),
        //            VectorAdapter<TVector>::New( 5, 3 ),
        //            VectorAdapter<TVector>::New( 0, 3 ),
        //        };
        //
        //        std::vector<std::vector<TVector>> inners = {
        //            {
        //                VectorAdapter<TVector>::New( 1, 1 ),
        //                VectorAdapter<TVector>::New( 2, 1 ),
        //                VectorAdapter<TVector>::New( 2, 2 ),
        //                VectorAdapter<TVector>::New( 1, 2 ),
        //            },
        //            {
        //                VectorAdapter<TVector>::New( 3, 1 ),
        //                VectorAdapter<TVector>::New( 4, 1 ),
        //                VectorAdapter<TVector>::New( 4, 2 ),
        //                VectorAdapter<TVector>::New( 3, 2 ),
        //            },
        //        };
        //
        //        auto inner = this->m_geomUtils->CombineLoops( inners );
        //        std::reverse( inner.begin(), inner.end() );
        //        outer = this->m_geomUtils->CombineLoops( { outer, inner} );
        //
        //        auto loops = this->m_extruder->Extrude( outer, VectorAdapter<TVector>::New( 0, 0, 1) );
        //
        //        return { this->m_adapter->CreateEntity( {}, this->CreatePolygons( loops ), {} ) };

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

        std::vector<TPolygon> polygons;
        std::vector<TPolyline> polylines;
        for( const auto& item: product->m_Representation->m_Representations ) {
            const auto [ p, l ] = this->ConvertRepresentation( item );
            std::copy( p.begin(), p.end(), std::back_inserter( polygons ) );
            std::copy( l.begin(), l.end(), std::back_inserter( polylines ) );
        }

        auto matrix = this->m_primitivesConverter->ConvertPlacement( product->m_ObjectPlacement );
        TMatrix::Multiply( &matrix, TMatrix::GetScale( this->m_parameters->m_lengthFactor, this->m_parameters->m_lengthFactor, this->m_parameters->m_lengthFactor ) );
        this->m_adapter->Transform( &polygons, matrix );
        this->m_adapter->Transform( &polylines, matrix );

        const auto opening = this->ConvertRelatedOpening( object );
        polygons = this->m_adapter->ComputeDifference( polygons, opening );

        return this->m_adapter->CreateEntity( object, polygons, polylines );
    }

    std::tuple<std::vector<TPolygon>, std::vector<TPolyline>> ConvertRepresentation( const std::shared_ptr<IfcRepresentation>& representation ) {
        std::vector<TPolygon> polygons;
        std::vector<TPolyline> polylines;

        for( const auto& item: representation->m_Items ) {
            // ENTITY IfcRepresentationItem  ABSTRACT SUPERTYPE OF(ONEOF(IfcGeometricRepresentationItem,
            // IfcMappedItem, IfcStyledItem, IfcTopologicalRepresentationItem));
            const auto geometric = dynamic_pointer_cast<IfcGeometricRepresentationItem>( item );
            if( geometric ) {
                auto [ p, l ] = this->ConvertGeometryRepresentation( geometric );
                std::copy( p.begin(), p.end(), std::back_inserter( polygons ) );
                std::copy( l.begin(), l.end(), std::back_inserter( polylines ) );
            }

            const auto mappedItem = dynamic_pointer_cast<IfcMappedItem>( item );
            if( mappedItem ) {
                auto [ p, l ] = this->ConvertMappedItem( mappedItem );
                std::copy( p.begin(), p.end(), std::back_inserter( polygons ) );
                std::copy( l.begin(), l.end(), std::back_inserter( polylines ) );
            }

            const auto topologicalItem = std::dynamic_pointer_cast<IfcTopologicalRepresentationItem>( item );
            if( topologicalItem ) {
                auto [ p, l ] = this->ConvertTopologicalRepresentationItem( topologicalItem );
                std::copy( p.begin(), p.end(), std::back_inserter( polygons ) );
                std::copy( l.begin(), l.end(), std::back_inserter( polylines ) );
            }
        }

        return { polygons, polylines };
    }

    std::tuple<std::vector<TPolygon>, std::vector<TPolyline>>
    ConvertTopologicalRepresentationItem( const shared_ptr<IfcTopologicalRepresentationItem>& topological_item ) {
        // IfcTopologicalRepresentationItem ABSTRACT SUPERTYPE OF(ONEOF(IfcConnectedFaceSet, IfcEdge, IfcFace, IfcFaceBound, IfcLoop, IfcPath, IfcVertex))

        const auto topo_connected_face_set = dynamic_pointer_cast<IfcConnectedFaceSet>( topological_item );
        if( topo_connected_face_set ) {
            std::vector<std::vector<TVector>> loops = this->m_geometryConverter->ConvertFaces( topo_connected_face_set->m_CfsFaces );

            // TODO: Rework (try to fix points order)
            auto reversed = loops;
            for( auto& l: reversed ) {
                std::reverse( l.begin(), l.end() );
            }
            std::copy( std::begin( reversed ), std::end( reversed ), std::back_inserter( loops ) );

            return { this->CreatePolygons( loops ), {} };
        }

        const auto topo_edge = dynamic_pointer_cast<IfcEdge>( topological_item );
        if( topo_edge ) {
            return { {}, { this->m_adapter->CreatePolyline( this->m_geometryConverter->ConvertEdge( topo_edge ) ) } };
        }

        const shared_ptr<IfcFace> topo_face = dynamic_pointer_cast<IfcFace>( topological_item );
        if( topo_face ) {
            auto loop = this->m_geometryConverter->ConvertFace( topo_face );
            auto rloop = loop;
            std::reverse( rloop.begin(), rloop.end() );
            return { this->CreatePolygons( { loop, rloop } ), {} };
        }

        const auto topo_face_bound = dynamic_pointer_cast<IfcFaceBound>( topological_item );
        if( topo_face_bound ) {
            const auto loop = this->m_geometryConverter->ConvertLoop( topo_face_bound->m_Bound );
            auto rloop = loop;
            std::reverse( rloop.begin(), rloop.end() );
            return { this->CreatePolygons( { loop, rloop } ), {} };
        }

        const auto topo_loop = dynamic_pointer_cast<IfcLoop>( topological_item );
        if( topo_loop ) {
            return { {}, { this->m_adapter->CreatePolyline( this->m_geometryConverter->ConvertLoop( topo_loop ) ) } };
        }

        // TODO: Implement all
        return {};
    }

    std::tuple<std::vector<TPolygon>, std::vector<TPolyline>> ConvertMappedItem( const std::shared_ptr<IfcMappedItem>& mappedItem ) {
        if( !mappedItem->m_MappingSource || !mappedItem->m_MappingSource->m_MappedRepresentation ) {
            // TODO: Log error
            return {};
        }
        const auto transformation = this->m_primitivesConverter->ConvertTransformationOperator( mappedItem->m_MappingTarget );
        const auto origin = this->m_primitivesConverter->ConvertPlacement( mappedItem->m_MappingSource->m_MappingOrigin );
        const auto m = TMatrix::GetMultiplied( transformation, origin );

        auto [ polygons, polylines ] = this->ConvertRepresentation( mappedItem->m_MappingSource->m_MappedRepresentation );
        // TODO: Material

        this->m_adapter->Transform( &polygons, m );
        this->m_adapter->Transform( &polylines, m );

        return { polygons, polylines };
    }

    std::vector<TPolygon> ConvertRelatedOpening( const std::shared_ptr<IFC4X3::IfcObjectDefinition>& object ) {
        std::vector<TPolygon> resultPolygons;
        if( const auto element = std::dynamic_pointer_cast<IfcElement>( object ) ) {
            for( const auto& openingRef: element->m_HasOpenings_inverse ) {
                std::vector<TPolygon> polygons;
                const auto openingRel = openingRef.lock();
                if( openingRel ) {
                    const auto opening = openingRel->m_RelatedOpeningElement;

                    if( !opening->m_Representation ) {
                        continue;
                    }
                    for( const auto& representation: opening->m_Representation->m_Representations ) {
                        auto [ p, l ] = this->ConvertRepresentation( representation );
                        std::copy( p.begin(), p.end(), std::back_inserter( polygons ) );
                    }
                    auto m = this->m_primitivesConverter->ConvertPlacement( opening->m_ObjectPlacement );
                    TMatrix::Multiply( &m, TMatrix::GetScale( this->m_parameters->m_lengthFactor, this->m_parameters->m_lengthFactor, this->m_parameters->m_lengthFactor ) );
                    this->m_adapter->Transform( &polygons, m );
                    std::copy( polygons.begin(), polygons.end(), std::back_inserter( resultPolygons ) );
                }
            }
        }
        return resultPolygons;
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
            std::vector<TPolygon> polygons;
            std::vector<TPolyline> polylines;

            for( const auto& geom_select: geometricSet->m_Elements ) {
                // TYPE IfcGeometricSetSelect = SELECT (IfcPoint, IfcCurve, IfcSurface);
                if( !geom_select ) {
                    continue;
                }

                shared_ptr<IfcPoint> point = dynamic_pointer_cast<IfcPoint>( geom_select );
                if( point ) {
                    // TODO: Implement
                    continue;
                }

                shared_ptr<IfcCurve> select_curve = dynamic_pointer_cast<IfcCurve>( geom_select );
                if( select_curve ) {
                    const auto [ p, l ] = this->ConvertGeometryRepresentation( select_curve );
                    std::copy( p.begin(), p.end(), std::back_inserter( polygons ) );
                    std::copy( l.begin(), l.end(), std::back_inserter( polylines ) );
                }

                shared_ptr<IfcSurface> select_surface = dynamic_pointer_cast<IfcSurface>( geom_select );
                if( select_surface ) {
                    const auto [ p, l ] = this->ConvertGeometryRepresentation( select_surface );
                    std::copy( p.begin(), p.end(), std::back_inserter( polygons ) );
                    std::copy( l.begin(), l.end(), std::back_inserter( polylines ) );
                }
            }

            shared_ptr<IfcGeometricCurveSet> geometric_curve_set = dynamic_pointer_cast<IfcGeometricCurveSet>( geometricSet );
            if( geometric_curve_set ) {
                // no additional attributes
            }
            return { polygons, polylines };
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

    std::tuple<std::vector<TPolygon>, std::vector<TPolyline>> ConvertSurfaceModel( const std::shared_ptr<IfcFaceBasedSurfaceModel>& surfaceModel ) {
        std::vector<std::shared_ptr<IfcFace>> faces;
        for( const auto& face_set: surfaceModel->m_FbsmFaces ) {
            std::copy( face_set->m_CfsFaces.begin(), face_set->m_CfsFaces.end(), std::back_inserter( faces ) );
        }
        std::vector<std::vector<TVector>> loops = this->m_geometryConverter->ConvertFaces( faces );

        // TODO: Rework (try to fix points order)
        auto reversed = loops;
        for( auto& l: reversed ) {
            std::reverse( l.begin(), l.end() );
        }
        std::copy( std::begin( reversed ), std::end( reversed ), std::back_inserter( loops ) );

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

        // TODO: Rework (try to fix points order)
        auto reversed = loops;
        for( auto& l: reversed ) {
            std::reverse( l.begin(), l.end() );
        }
        std::copy( std::begin( reversed ), std::end( reversed ), std::back_inserter( loops ) );

        return { this->CreatePolygons( loops ), {} };
    }

    std::tuple<std::vector<TPolygon>, std::vector<TPolyline>> ConvertSurface( const std::shared_ptr<IfcSurface>& surface ) {
        auto loops = this->m_geometryConverter->ConvertSurface( surface );

        // TODO: Rework (try to fix points order)
        auto reversed = loops;
        for( auto& l: reversed ) {
            std::reverse( l.begin(), l.end() );
        }
        std::copy( std::begin( reversed ), std::end( reversed ), std::back_inserter( loops ) );

        return { this->CreatePolygons( loops ), {} };
    }

    std::vector<TPolygon> CreatePolygons( const std::vector<std::vector<TVector>>& loops ) {
        std::vector<TPolygon> result;
        for( const auto& l: loops ) {
            if( l.size() < 3 ) {
                // WTF????
                // TODO: Log error
                continue;
            }
            const auto indices = this->m_adapter->Triangulate( l );
            if( indices.size() < 3 ) {
                continue;
            }
            for( int i = 0; i < indices.size() - 2; i += 3 ) {
                result.push_back( this->m_adapter->CreatePolygon( l, { indices[ i ], indices[ i + 1 ], indices[ i + 2 ] } ) );
            }
        }
        return result;
    }
};

}
