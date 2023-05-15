#pragma once

#include <map>

#include "ifcpp/Model/OpenMPIncludes.h"

#include "ifcpp/Ifc/IfcAdvancedBrep.h"
#include "ifcpp/Ifc/IfcAdvancedBrepWithVoids.h"
#include "ifcpp/Ifc/IfcBlock.h"
#include "ifcpp/Ifc/IfcBooleanClippingResult.h"
#include "ifcpp/Ifc/IfcBooleanOperator.h"
#include "ifcpp/Ifc/IfcBooleanResult.h"
#include "ifcpp/Ifc/IfcBoundingBox.h"
#include "ifcpp/Ifc/IfcBoxedHalfSpace.h"
#include "ifcpp/Ifc/IfcClosedShell.h"
#include "ifcpp/Ifc/IfcCsgPrimitive3D.h"
#include "ifcpp/Ifc/IfcCsgSelect.h"
#include "ifcpp/Ifc/IfcCsgSolid.h"
#include "ifcpp/Ifc/IfcElementarySurface.h"
#include "ifcpp/Ifc/IfcExtrudedAreaSolid.h"
#include "ifcpp/Ifc/IfcFacetedBrep.h"
#include "ifcpp/Ifc/IfcFacetedBrepWithVoids.h"
#include "ifcpp/Ifc/IfcFixedReferenceSweptAreaSolid.h"
#include "ifcpp/Ifc/IfcHalfSpaceSolid.h"
#include "ifcpp/Ifc/IfcManifoldSolidBrep.h"
#include "ifcpp/Ifc/IfcPolygonalBoundedHalfSpace.h"
#include "ifcpp/Ifc/IfcRectangularPyramid.h"
#include "ifcpp/Ifc/IfcRevolvedAreaSolid.h"
#include "ifcpp/Ifc/IfcRightCircularCone.h"
#include "ifcpp/Ifc/IfcRightCircularCylinder.h"
#include "ifcpp/Ifc/IfcSolidModel.h"
#include "ifcpp/Ifc/IfcSphere.h"
#include "ifcpp/Ifc/IfcSurfaceCurveSweptAreaSolid.h"
#include "ifcpp/Ifc/IfcSweptAreaSolid.h"
#include "ifcpp/Ifc/IfcSweptDiskSolid.h"

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
#include "ifcpp/Geometry/StyleConverter.h"
#include "ifcpp/Geometry/VectorAdapter.h"
#include "ifcpp/Geometry/VisualObject.h"


namespace ifcpp {
using namespace IFC4X3;

template<CAdapter TAdapter>
class SolidConverter {
    using TVector = typename TAdapter::TVector;
    using AVector = VectorAdapter<TVector>;
    using TTriangle = typename TAdapter::TTriangle;
    using TMesh = typename TAdapter::TMesh;
    using TVisualObject = VisualObject<TAdapter>;

    std::shared_ptr<PrimitiveTypesConverter<TVector>> m_primitivesConverter;
    std::shared_ptr<CurveConverter<TVector>> m_curveConverter;
    std::shared_ptr<ProfileConverter<TVector>> m_profileConverter;
    std::shared_ptr<Extruder<TVector>> m_extruder;
    std::shared_ptr<GeometryConverter<TVector>> m_geometryConverter;
    std::shared_ptr<TAdapter> m_adapter;
    std::shared_ptr<GeomUtils<TVector>> m_geomUtils;
    std::shared_ptr<StyleConverter> m_styleConverter;
    std::shared_ptr<Parameters> m_parameters;

    std::map<std::shared_ptr<IfcBooleanResult>, std::vector<std::shared_ptr<TVisualObject>>> m_booleanResultToVisualObjectMap;
#ifdef ENABLE_OPENMP
    Mutex m_booleanResultToVisualObjectMapMutex;
#endif

public:
    SolidConverter( const std::shared_ptr<PrimitiveTypesConverter<TVector>>& primitivesConverter,
                    const std::shared_ptr<CurveConverter<TVector>>& curveConverter, const std::shared_ptr<ProfileConverter<TVector>>& profileConverter,
                    const std::shared_ptr<Extruder<TVector>>& extruder, const std::shared_ptr<GeometryConverter<TVector>> geometryConverter,
                    const std::shared_ptr<TAdapter>& adapter, const std::shared_ptr<GeomUtils<TVector>>& geomUtils,
                    const std::shared_ptr<StyleConverter>& styleConverter, const std::shared_ptr<Parameters>& parameters )
        : m_primitivesConverter( primitivesConverter )
        , m_curveConverter( curveConverter )
        , m_profileConverter( profileConverter )
        , m_extruder( extruder )
        , m_geometryConverter( geometryConverter )
        , m_adapter( adapter )
        , m_geomUtils( geomUtils )
        , m_styleConverter( styleConverter )
        , m_parameters( parameters ) {
    }

    void ResetCaches() {
#ifdef ENABLE_OPENMP
        ScopedLock lock( this->m_booleanResultToVisualObjectMapMutex );
#endif
        this->m_booleanResultToVisualObjectMap = {};
    }

    std::vector<std::shared_ptr<TVisualObject>> ConvertSolidModel( const shared_ptr<IfcSolidModel>& solidModel ) {
        // ENTITY IfcSolidModel ABSTRACT SUPERTYPE OF(ONEOF(IfcCsgSolid, IfcManifoldSolidBrep, IfcSweptAreaSolid, IfcSweptDiskSolid))

        std::vector<std::shared_ptr<TVisualObject>> result;

        const auto sweptAreaSolid = dynamic_pointer_cast<IfcSweptAreaSolid>( solidModel );
        if( sweptAreaSolid ) {
            result = { this->ConvertSweptAreaSolid( sweptAreaSolid ) };
        }

        const auto manifoldSolidBrep = dynamic_pointer_cast<IfcManifoldSolidBrep>( solidModel );
        if( manifoldSolidBrep ) {
            result = { this->ConvertManifoldSolidBrep( manifoldSolidBrep ) };
        }

        const auto csgSolid = dynamic_pointer_cast<IfcCsgSolid>( solidModel );
        if( csgSolid ) {
            result = this->ConvertCsgSolid( csgSolid );
        }

        const auto sweptDiskSolid = dynamic_pointer_cast<IfcSweptDiskSolid>( solidModel );
        if( sweptDiskSolid ) {
            result = { this->ConvertSweptDiskSolid( sweptDiskSolid ) };
        }

        const auto styles = this->m_styleConverter->GetStyles( solidModel );
        for( auto& r: result ) {
            r->AddStyles( styles );
        }

        return result;
    }

    std::vector<std::shared_ptr<TVisualObject>> ConvertBooleanResult( const shared_ptr<IfcBooleanResult>& booleanResult ) {
        {
#ifdef ENABLE_OPENMP
            ScopedLock lock( this->m_booleanResultToVisualObjectMapMutex );
#endif
            if( this->m_booleanResultToVisualObjectMap.contains( booleanResult ) ) {
                return TVisualObject::Copy( this->m_adapter, this->m_booleanResultToVisualObjectMap[ booleanResult ] );
            }
        }

        shared_ptr<IfcBooleanOperator>& ifc_boolean_operator = booleanResult->m_Operator;
        shared_ptr<IfcBooleanOperand> ifc_first_operand = booleanResult->m_FirstOperand;
        shared_ptr<IfcBooleanOperand> ifc_second_operand = booleanResult->m_SecondOperand;
        if( !ifc_boolean_operator || !ifc_first_operand || !ifc_second_operand ) {
            // TODO: Log error
            return {};
        }

        auto operand1 = this->ConvertBooleanOperand( ifc_first_operand );
        auto operand2 = this->ConvertBooleanOperand( ifc_second_operand );

        // TODO: Rework IfcBooleanClippingResult (and IfcHalfSpaceSolid)
        //                shared_ptr<IfcBooleanClippingResult> boolean_clipping_result = dynamic_pointer_cast<IfcBooleanClippingResult>( bool_result );
        //                if( boolean_clipping_result ) {
        //                    return operand2;
        //                }

        std::vector<TMesh> operand1meshes;
        std::vector<TMesh> operand2meshes;
        switch( ifc_boolean_operator->m_enum ) {
        case IfcBooleanOperator::ENUM_UNION:
            // TODO: Handle style
            for( const auto& o1: operand1 ) {
                std::copy( o1->m_meshes.begin(), o1->m_meshes.end(), std::back_inserter( operand2meshes ) );
            }
            for( const auto& o2: operand2 ) {
                std::copy( o2->m_meshes.begin(), o2->m_meshes.end(), std::back_inserter( operand2meshes ) );
            }
            operand1 = { TVisualObject::Create( this->m_adapter->ComputeUnion( operand1meshes, operand2meshes ) ) };
            break;
        case IfcBooleanOperator::ENUM_INTERSECTION:
            for( const auto& o2: operand2 ) {
                std::copy( o2->m_meshes.begin(), o2->m_meshes.end(), std::back_inserter( operand2meshes ) );
            }
            for( auto& o1: operand1 ) {
                o1->m_meshes = this->m_adapter->ComputeIntersection( o1->m_meshes, operand2meshes );
            }
            break;
        case IfcBooleanOperator::ENUM_DIFFERENCE:
            for( auto& o1: operand1 ) {
                for( auto& o2: operand2 ) {
                    o1->m_meshes = this->m_adapter->ComputeDifference( o1->m_meshes, o2->m_meshes );
                }
            }
            break;
        default:
            // TODO: Log error
            operand1 = {};
            break;
        }

        const auto styles = this->m_styleConverter->GetStyles( booleanResult );
        for( auto& o: operand1 ) {
            o->AddStyles( styles );
        }

        {
#ifdef ENABLE_OPENMP
            ScopedLock lock( this->m_booleanResultToVisualObjectMapMutex );
#endif
            this->m_booleanResultToVisualObjectMap[ booleanResult ] = operand1;
        }
        return operand1;
    }


private:
    std::shared_ptr<TVisualObject> ConvertSweptAreaSolid( const shared_ptr<IfcSweptAreaSolid>& sweptAreaSolid ) {
        // ENTITY IfcSweptAreaSolid
        //	ABSTRACT SUPERTYPE OF(ONEOF(IfcExtrudedAreaSolid, IfcFixedReferenceSweptAreaSolid, IfcRevolvedAreaSolid, IfcSurfaceCurveSweptAreaSolid))
        //	SUBTYPE OF IfcSolidModel;
        //	SweptArea	 :	IfcProfileDef;
        //	Position	 :	OPTIONAL IfcAxis2Placement3D;
        //	WHERE
        //	SweptAreaType	 :	SweptArea.ProfileType = IfcProfileTypeEnum.Area;
        // END_ENTITY;

        if( !sweptAreaSolid->m_SweptArea ) {
            // TODO: Log error
            return TVisualObject::CreateEmpty();
        }

        shared_ptr<IfcExtrudedAreaSolid> extrudedArea = dynamic_pointer_cast<IfcExtrudedAreaSolid>( sweptAreaSolid );
        if( extrudedArea ) {
            return this->ConvertExtrudedAreaSolid( extrudedArea );
        }

        const auto fixedReferenceSweptAreaSolid = dynamic_pointer_cast<IfcFixedReferenceSweptAreaSolid>( sweptAreaSolid );
        if( fixedReferenceSweptAreaSolid ) {
            return this->ConvertFixedReferenceSweptAreaSolid( fixedReferenceSweptAreaSolid );
        }

        const auto revolvedAreaSolid = dynamic_pointer_cast<IfcRevolvedAreaSolid>( sweptAreaSolid );
        if( revolvedAreaSolid ) {
            return this->ConvertRevolvedAreaSolid( revolvedAreaSolid );
        }

        shared_ptr<IfcSurfaceCurveSweptAreaSolid> surfaceCurveSweptAreaSolid = dynamic_pointer_cast<IfcSurfaceCurveSweptAreaSolid>( sweptAreaSolid );
        if( surfaceCurveSweptAreaSolid ) {
            // TODO: Implement
            return TVisualObject::CreateEmpty();
        }

        // TODO: Log error
        return TVisualObject::CreateEmpty();
    }

    std::shared_ptr<TVisualObject> ConvertExtrudedAreaSolid( const shared_ptr<IfcExtrudedAreaSolid>& extrudedArea ) {
        if( !extrudedArea->m_ExtrudedDirection || !extrudedArea->m_Depth || !extrudedArea->m_SweptArea ) {
            // TODO: Log error
            return TVisualObject::CreateEmpty();
        }
        const auto depth = extrudedArea->m_Depth->m_value;
        const auto extrusion = this->m_primitivesConverter->ConvertPoint( extrudedArea->m_ExtrudedDirection->m_DirectionRatios ) * depth;
        auto loops = this->m_extruder->Extrude( this->m_profileConverter->ConvertProfile( extrudedArea->m_SweptArea ), extrusion );
        const auto m = this->m_primitivesConverter->ConvertPlacement( extrudedArea->m_Position );
        m.TransformLoops( &loops );
        return TVisualObject::Create( { Helpers::CreateMesh( this->m_adapter, loops ) } );
    }

    std::shared_ptr<TVisualObject> ConvertFixedReferenceSweptAreaSolid( const shared_ptr<IfcFixedReferenceSweptAreaSolid>& fixedReferenceSweptAreaSolid ) {
        // Directrix	 : OPTIONAL IfcCurve;
        // StartParam	 : OPTIONAL IfcParameterValue;
        // EndParam	 : OPTIONAL IfcParameterValue;
        // FixedReference	 : IfcDirection;
        //  TODO: Trim curve with StartParam and EndParam, use FixedReference for swept area orientation

        const auto profile_paths = this->m_profileConverter->ConvertProfile( fixedReferenceSweptAreaSolid->m_SweptArea );
        auto loops = this->m_extruder->Sweep( profile_paths, this->m_curveConverter->ConvertCurve( fixedReferenceSweptAreaSolid->m_Directrix ) );
        const auto m = this->m_primitivesConverter->ConvertPlacement( fixedReferenceSweptAreaSolid->m_Position );
        m.TransformLoops( &loops );
        return TVisualObject::Create( { Helpers::CreateMesh( this->m_adapter, loops ) } );
    }

    std::shared_ptr<TVisualObject> ConvertRevolvedAreaSolid( const shared_ptr<IfcRevolvedAreaSolid>& revolvedArea ) {
        if( !revolvedArea || !revolvedArea->m_Angle || !revolvedArea->m_SweptArea || !revolvedArea->m_Axis ) {
            // TODO: Log error
            return TVisualObject::CreateEmpty();
        }
        double revolveAngle = revolvedArea->m_Angle->m_value * this->m_parameters->m_angleFactor;
        TVector axisLocation = AVector::New( 0, 0, 0 );
        TVector axisDirection = AVector::New( 1, 0, 0 );

        axisLocation = this->m_primitivesConverter->ConvertPoint( revolvedArea->m_Axis->m_Location );
        axisDirection = this->m_primitivesConverter->ConvertPoint( revolvedArea->m_Axis->m_Axis->m_DirectionRatios );

        auto loops =
            this->m_extruder->Revolve( this->m_profileConverter->ConvertProfile( revolvedArea->m_SweptArea ), axisLocation, axisDirection, revolveAngle );
        const auto m = this->m_primitivesConverter->ConvertPlacement( revolvedArea->m_Position );
        m.TransformLoops( &loops );
        return TVisualObject::Create( { Helpers::CreateMesh( this->m_adapter, loops ) } );
    }

    std::shared_ptr<TVisualObject> ConvertManifoldSolidBrep( const std::shared_ptr<IfcManifoldSolidBrep>& manifoldSolidBrep ) {
        if( !manifoldSolidBrep->m_Outer ) {
            // TODO: Log error
            return TVisualObject::CreateEmpty();
        }
        // ENTITY IfcManifoldSolidBrep
        //	ABSTRACT SUPERTYPE OF(ONEOF(IfcAdvancedBrep, IfcFacetedBrep))
        //	SUBTYPE OF IfcSolidModel;
        //		Outer	 :	IfcClosedShell;
        // END_ENTITY;

        auto loops = this->m_geometryConverter->ConvertFaces( manifoldSolidBrep->m_Outer->m_CfsFaces );

        std::vector<TMesh> resultMeshes = { Helpers::CreateMesh( this->m_adapter, loops ) };

        const auto facetedBrep = dynamic_pointer_cast<IfcFacetedBrep>( manifoldSolidBrep );
        if( facetedBrep ) {
            const auto withVoids = std::dynamic_pointer_cast<IfcFacetedBrepWithVoids>( facetedBrep );
            if( withVoids ) {
                std::vector<TMesh> voidMeshes;
                for( const auto& shell: withVoids->m_Voids ) {
                    auto voidLoops = this->m_geometryConverter->ConvertFaces( shell->m_CfsFaces );
                    voidMeshes.push_back( Helpers::CreateMesh( this->m_adapter, voidLoops ) );
                }
                resultMeshes = this->m_adapter->ComputeDifference( resultMeshes, voidMeshes );
            }
        }

        const auto advancedBrep = dynamic_pointer_cast<IfcAdvancedBrep>( manifoldSolidBrep );
        if( advancedBrep ) {
            const auto withVoids = dynamic_pointer_cast<IfcAdvancedBrepWithVoids>( advancedBrep );
            if( withVoids ) {
                std::vector<TMesh> voidMeshes;
                for( const auto& shell: withVoids->m_Voids ) {
                    auto voidLoops = this->m_geometryConverter->ConvertFaces( shell->m_CfsFaces );
                    voidMeshes.push_back( Helpers::CreateMesh( this->m_adapter, voidLoops ) );
                }
                resultMeshes = this->m_adapter->ComputeDifference( resultMeshes, voidMeshes );
            }
        }

        return TVisualObject::Create( resultMeshes );
    }

    std::vector<std::shared_ptr<TVisualObject>> ConvertCsgSolid( const std::shared_ptr<IfcCsgSolid>& csgSolid ) {
        const auto& csgSelect = csgSolid->m_TreeRootExpression;
        const auto booleanResult = dynamic_pointer_cast<IfcBooleanResult>( csgSelect );
        std::vector<std::shared_ptr<TVisualObject>> result;
        if( booleanResult ) {
            result = this->ConvertBooleanResult( booleanResult );
        }
        const auto csgPrimitive3d = dynamic_pointer_cast<IfcCsgPrimitive3D>( csgSelect );
        if( csgPrimitive3d ) {
            result = { this->ConvertCsgPrimitive3D( csgPrimitive3d ) };
        }
        for( auto& r: result ) {
            r->AddStyles( this->m_styleConverter->GetStyles( csgSolid ) );
        }
        return result;
    }

    std::shared_ptr<TVisualObject> ConvertSweptDiskSolid( const std::shared_ptr<IfcSweptDiskSolid>& sweptDiskSolid ) {
        // ENTITY IfcSweptDiskSolid;
        //	ENTITY IfcRepresentationItem;
        //	INVERSE
        //		LayerAssignments	 : 	SET OF IfcPresentationLayerAssignment FOR AssignedItems;
        //		StyledByItem	 : 	SET [0:1] OF IfcStyledItem FOR Item;
        //	ENTITY IfcGeometricRepresentationItem;
        //	ENTITY IfcSolidModel;
        //		DERIVE
        //		Dim	 : 	IfcDimensionCount :=  3;
        //	ENTITY IfcSweptDiskSolid;
        //		Directrix	 : 	IfcCurve;
        //		Radius	 : 	IfcPositiveLengthMeasure;
        //		InnerRadius	 : 	OPTIONAL IfcPositiveLengthMeasure;
        //		StartParam	 : 	OPTIONAL IfcParameterValue;
        //		EndParam	 : 	OPTIONAL IfcParameterValue;
        // END_ENTITY;

        if( !sweptDiskSolid->m_Radius ) {
            // TODO: Log error
            return TVisualObject::CreateEmpty();
        }

        auto radius = sweptDiskSolid->m_Radius->m_value;

        double innerRadius = 0.0;
        if( sweptDiskSolid->m_InnerRadius ) {
            innerRadius = sweptDiskSolid->m_InnerRadius->m_value;
        }

        // TODO: handle start param, end param

        const auto profile = this->m_curveConverter->ConvertCurve( sweptDiskSolid->m_Directrix );

        const auto outer = this->m_geomUtils->BuildCircle( radius, 0.0, ( M_PI * 2 ), this->m_parameters->m_numVerticesPerCircle );
        std::vector<TVector> inner;
        if( innerRadius > this->m_parameters->m_epsilon ) {
            inner = this->m_geomUtils->BuildCircle( innerRadius, 0.0, -( M_PI * 2 ), this->m_parameters->m_numVerticesPerCircle );
        }
        const auto loops = this->m_extruder->Sweep( this->m_geomUtils->CombineLoops( { outer, inner } ), profile );
        auto resultMesh = Helpers::CreateMesh( this->m_adapter, loops );
        return TVisualObject::Create( { resultMesh } );
    }

    std::vector<std::shared_ptr<TVisualObject>> ConvertBooleanOperand( const shared_ptr<IfcBooleanOperand>& operand ) {
        // TYPE IfcBooleanOperand = SELECT	(IfcBooleanResult	,IfcCsgPrimitive3D	,IfcHalfSpaceSolid	,IfcSolidModel);

        const auto solidModel = dynamic_pointer_cast<IfcSolidModel>( operand );
        if( solidModel ) {
            return this->ConvertSolidModel( solidModel );
        }

        const auto halfSpaceSolid = dynamic_pointer_cast<IfcHalfSpaceSolid>( operand );
        if( halfSpaceSolid ) {
            return { this->ConvertHalfSpaceSolid( halfSpaceSolid ) };
        }

        const auto booleanResult = dynamic_pointer_cast<IfcBooleanResult>( operand );
        if( booleanResult ) {
            return this->ConvertBooleanResult( booleanResult );
        }

        const auto csgPrimitive3d = dynamic_pointer_cast<IfcCsgPrimitive3D>( operand );
        if( csgPrimitive3d ) {
            return { this->ConvertCsgPrimitive3D( csgPrimitive3d ) };
        }

        // TODO: Log error
        return {};
    }

    std::shared_ptr<TVisualObject> ConvertHalfSpaceSolid( const shared_ptr<IfcHalfSpaceSolid>& halfSpaceSolid ) {
        // ENTITY IfcHalfSpaceSolid SUPERTYPE OF(ONEOF(IfcBoxedHalfSpace, IfcPolygonalBoundedHalfSpace))

        const auto elem_base_surface = dynamic_pointer_cast<IfcElementarySurface>( halfSpaceSolid->m_BaseSurface );
        if( !elem_base_surface ) {
            // TODO: Log error
            return TVisualObject::CreateEmpty();
        }

        auto planeMatrix = this->m_primitivesConverter->ConvertPlacement( elem_base_surface->m_Position );
        if( !halfSpaceSolid->m_AgreementFlag || halfSpaceSolid->m_AgreementFlag->m_value ) {
            planeMatrix = Matrix<TVector>::GetMultiplied( Matrix<TVector>::GetScale( 1, 1, -1 ), planeMatrix );
        }

        auto boxed = std::dynamic_pointer_cast<IfcBoxedHalfSpace>( halfSpaceSolid );
        if( boxed ) {
            // Convert as IfcHalfSpaceSolid
        }

        const auto polygonal = std::dynamic_pointer_cast<IfcPolygonalBoundedHalfSpace>( halfSpaceSolid );
        if( polygonal ) {
            auto boundaryProfile = this->m_geomUtils->SimplifyLoop( this->m_curveConverter->ConvertCurve( polygonal->m_PolygonalBoundary ) );
            const auto m = this->m_primitivesConverter->ConvertPlacement( polygonal->m_Position );

            for( auto& p: boundaryProfile ) {
                p.z = -this->m_parameters->m_modelMaxSize * 0.5;
            }

            auto boundaryLoops = this->m_extruder->Extrude( boundaryProfile, AVector::New( 0, 0, 1 ) * this->m_parameters->m_modelMaxSize );
            m.TransformLoops( &boundaryLoops );
            const auto boundaryMesh = Helpers::CreateMesh( this->m_adapter, boundaryLoops );

            // TODO: Just clip with plane
            std::vector<TVector> halfSpaceProfile = {
                AVector::New( -this->m_parameters->m_modelMaxSize, -this->m_parameters->m_modelMaxSize ),
                AVector::New( this->m_parameters->m_modelMaxSize, -this->m_parameters->m_modelMaxSize ),
                AVector::New( this->m_parameters->m_modelMaxSize, this->m_parameters->m_modelMaxSize ),
                AVector::New( -this->m_parameters->m_modelMaxSize, this->m_parameters->m_modelMaxSize ),
            };
            auto halfSpaceLoops = this->m_extruder->Extrude( halfSpaceProfile, AVector::New( 0, 0, 1 ) * this->m_parameters->m_modelMaxSize );
            planeMatrix.TransformLoops( &halfSpaceLoops );
            if( !halfSpaceSolid->m_AgreementFlag || halfSpaceSolid->m_AgreementFlag->m_value ) {
                for( auto& l: halfSpaceLoops ) {
                    std::reverse( l.begin(), l.end() );
                }
            }
            const auto halfSpaceMesh = Helpers::CreateMesh( this->m_adapter, halfSpaceLoops );

            return TVisualObject::Create( this->m_adapter->ComputeIntersection( { halfSpaceMesh }, { boundaryMesh } ) );
        }

        // TODO: Just clip with plane
        std::vector<TVector> halfSpaceProfile = {
            AVector::New( -this->m_parameters->m_modelMaxSize, -this->m_parameters->m_modelMaxSize ),
            AVector::New( this->m_parameters->m_modelMaxSize, -this->m_parameters->m_modelMaxSize ),
            AVector::New( this->m_parameters->m_modelMaxSize, this->m_parameters->m_modelMaxSize ),
            AVector::New( -this->m_parameters->m_modelMaxSize, this->m_parameters->m_modelMaxSize ),
        };
        auto halfSpaceLoops = this->m_extruder->Extrude( halfSpaceProfile, AVector::New( 0, 0, 1 ) * this->m_parameters->m_modelMaxSize );
        planeMatrix.TransformLoops( &halfSpaceLoops );
        if( !halfSpaceSolid->m_AgreementFlag || halfSpaceSolid->m_AgreementFlag->m_value ) {
            for( auto& l: halfSpaceLoops ) {
                std::reverse( l.begin(), l.end() );
            }
        }
        return TVisualObject::Create( { Helpers::CreateMesh( this->m_adapter, halfSpaceLoops ) } );
    }

    std::shared_ptr<TVisualObject> ConvertCsgPrimitive3D( const shared_ptr<IfcCsgPrimitive3D>& csgPrimitive ) {
        // ENTITY IfcCsgPrimitive3D  ABSTRACT SUPERTYPE OF(ONEOF(IfcBlock, IfcRectangularPyramid, IfcRightCircularCone, IfcRightCircularCylinder, IfcSphere

        std::vector<TMesh> resultMeshes;

        const auto block = dynamic_pointer_cast<IfcBlock>( csgPrimitive );
        if( block ) {
            resultMeshes = this->ConvertBlock( block );
        }

        const auto rectangular_pyramid = dynamic_pointer_cast<IfcRectangularPyramid>( csgPrimitive );
        if( rectangular_pyramid ) {
            resultMeshes = this->ConvertRectangularPyramid( rectangular_pyramid );
        }

        const auto right_circular_cone = dynamic_pointer_cast<IfcRightCircularCone>( csgPrimitive );
        if( right_circular_cone ) {
            resultMeshes = this->ConvertRightCircularCone( right_circular_cone );
        }

        shared_ptr<IfcRightCircularCylinder> right_circular_cylinder = dynamic_pointer_cast<IfcRightCircularCylinder>( csgPrimitive );
        if( right_circular_cylinder ) {
            resultMeshes = this->ConvertRightCircularCylinder( right_circular_cylinder );
        }

        shared_ptr<IfcSphere> sphere = dynamic_pointer_cast<IfcSphere>( csgPrimitive );
        if( sphere ) {
            resultMeshes = this->ConvertSphere( sphere );
        }

        const auto styles = this->m_styleConverter->GetStyles( csgPrimitive );
        auto result = TVisualObject::Create( resultMeshes, {}, styles );
        return result;
    }

    std::vector<TMesh> ConvertBlock( const std::shared_ptr<IfcBlock>& block ) {
        if( !block->m_XLength || !block->m_YLength || !block->m_ZLength ) {
            // TODO: Log error
            return {};
        }

        const auto m = this->m_primitivesConverter->ConvertPlacement( block->m_Position );

        const double xLen = block->m_XLength->m_value * 0.5;
        const double yLen = block->m_YLength->m_value * 0.5;
        const double zLen = block->m_ZLength->m_value * 0.5;

        std::vector<TVector> vertices;
        vertices.push_back( AVector::New( xLen, yLen, zLen ) );
        vertices.push_back( AVector::New( -xLen, yLen, zLen ) );
        vertices.push_back( AVector::New( -xLen, -yLen, zLen ) );
        vertices.push_back( AVector::New( xLen, -yLen, zLen ) );
        vertices.push_back( AVector::New( xLen, yLen, -zLen ) );
        vertices.push_back( AVector::New( -xLen, yLen, -zLen ) );
        vertices.push_back( AVector::New( -xLen, -yLen, -zLen ) );
        vertices.push_back( AVector::New( xLen, -yLen, -zLen ) );

        std::vector<TTriangle> polygons;
        polygons.push_back( this->m_adapter->CreateTriangle( vertices, { 0, 1, 2 } ) );
        polygons.push_back( this->m_adapter->CreateTriangle( vertices, { 2, 3, 0 } ) );

        polygons.push_back( this->m_adapter->CreateTriangle( vertices, { 7, 6, 5 } ) );
        polygons.push_back( this->m_adapter->CreateTriangle( vertices, { 5, 4, 7 } ) );

        polygons.push_back( this->m_adapter->CreateTriangle( vertices, { 0, 4, 5 } ) );
        polygons.push_back( this->m_adapter->CreateTriangle( vertices, { 5, 1, 0 } ) );

        polygons.push_back( this->m_adapter->CreateTriangle( vertices, { 1, 5, 6 } ) );
        polygons.push_back( this->m_adapter->CreateTriangle( vertices, { 6, 2, 1 } ) );

        polygons.push_back( this->m_adapter->CreateTriangle( vertices, { 2, 6, 7 } ) );
        polygons.push_back( this->m_adapter->CreateTriangle( vertices, { 7, 3, 2 } ) );

        polygons.push_back( this->m_adapter->CreateTriangle( vertices, { 3, 7, 4 } ) );
        polygons.push_back( this->m_adapter->CreateTriangle( vertices, { 4, 0, 3 } ) );

        std::vector<TMesh> result = { this->m_adapter->CreateMesh( polygons ) };
        this->m_adapter->Transform( &result, m );
        return result;
    }

    std::vector<TMesh> ConvertRectangularPyramid( const std::shared_ptr<IfcRectangularPyramid>& rectangularPyramid ) {
        if( !rectangularPyramid->m_XLength || !rectangularPyramid->m_YLength || !rectangularPyramid->m_Height ) {
            // TODO: Log error
            return {};
        }

        const auto m = this->m_primitivesConverter->ConvertPlacement( rectangularPyramid->m_Position );

        const double xLen = rectangularPyramid->m_XLength->m_value * 0.5;
        const double yLen = rectangularPyramid->m_YLength->m_value * 0.5;
        const double height = rectangularPyramid->m_Height->m_value * 0.5;

        std::vector<TVector> vertices;
        vertices.push_back( AVector::New( 0, 0, height ) );
        vertices.push_back( AVector::New( xLen, -yLen, 0.0 ) );
        vertices.push_back( AVector::New( -xLen, -yLen, 0.0 ) );
        vertices.push_back( AVector::New( -xLen, yLen, 0.0 ) );
        vertices.push_back( AVector::New( xLen, yLen, 0.0 ) );

        std::vector<TTriangle> polygons;
        polygons.push_back( this->m_adapter->CreateTriangle( vertices, { 1, 2, 3 } ) );
        polygons.push_back( this->m_adapter->CreateTriangle( vertices, { 3, 4, 1 } ) );
        polygons.push_back( this->m_adapter->CreateTriangle( vertices, { 0, 2, 1 } ) );
        polygons.push_back( this->m_adapter->CreateTriangle( vertices, { 0, 1, 4 } ) );
        polygons.push_back( this->m_adapter->CreateTriangle( vertices, { 0, 4, 3 } ) );
        polygons.push_back( this->m_adapter->CreateTriangle( vertices, { 0, 3, 2 } ) );

        std::vector<TMesh> result = { this->m_adapter->CreateMesh( polygons ) };
        this->m_adapter->Transform( &result, m );
        return result;
    }

    std::vector<TMesh> ConvertRightCircularCone( const std::shared_ptr<IfcRightCircularCone>& rightCircularCone ) {
        if( !rightCircularCone->m_Height || !rightCircularCone->m_BottomRadius ) {
            // TODO: Log error
            return {};
        }

        const auto m = this->m_primitivesConverter->ConvertPlacement( rightCircularCone->m_Position );

        const auto height = rightCircularCone->m_Height->m_value;
        const auto radius = rightCircularCone->m_BottomRadius->m_value;

        std::vector<TVector> vertices;
        vertices.push_back( AVector::New( 0.0, 0.0, height ) ); // top
        vertices.push_back( AVector::New( 0.0, 0.0, 0.0 ) ); // bottom center

        double angle = 0;
        auto delta = ( 2.0 * M_PI / double( this->m_parameters->m_numVerticesPerCircle - 1 ) ); // TODO: Use radius
        for( int i = 0; i < this->m_parameters->m_numVerticesPerCircle; ++i ) {
            vertices.push_back( AVector::New( sin( angle ) * radius, cos( angle ) * radius, 0.0 ) );
            angle += delta;
        }

        std::vector<TTriangle> polygons;
        // outer shape
        for( int i = 0; i < this->m_parameters->m_numVerticesPerCircle - 1; ++i ) {
            polygons.push_back( this->m_adapter->CreateTriangle( vertices, { 0, i + 3, i + 2 } ) );
        }
        polygons.push_back( this->m_adapter->CreateTriangle( vertices, { 0, 2, this->m_parameters->m_numVerticesPerCircle + 1 } ) );

        // bottom circle
        for( int i = 0; i < this->m_parameters->m_numVerticesPerCircle - 1; ++i ) {
            polygons.push_back( this->m_adapter->CreateTriangle( vertices, { 1, i + 2, i + 3 } ) );
        }
        polygons.push_back( this->m_adapter->CreateTriangle( vertices, { 1, this->m_parameters->m_numVerticesPerCircle + 1, 2 } ) );

        std::vector<TMesh> result = { this->m_adapter->CreateMesh( polygons ) };
        this->m_adapter->Transform( &result, m );
        return result;
    }

    std::vector<TMesh> ConvertRightCircularCylinder( const std::shared_ptr<IfcRightCircularCylinder>& rightCircularCylinder ) {
        if( !rightCircularCylinder->m_Height || !rightCircularCylinder->m_Radius ) {
            // TODO: Log error
            return {};
        }

        const auto m = this->m_primitivesConverter->ConvertPlacement( rightCircularCylinder->m_Position );

        auto height = rightCircularCylinder->m_Height->m_value;
        auto radius = rightCircularCylinder->m_Radius->m_value;

        // TODO: Use radius
        const auto circle = this->m_geomUtils->BuildCircle( radius, 0, ( M_PI * 2 ), this->m_parameters->m_numVerticesPerCircle );
        auto loops = this->m_extruder->Extrude( this->m_geomUtils->SimplifyLoop( circle ), AVector::New( 0, 0, height ), false );

        m.TransformLoops( &loops );
        return { Helpers::CreateMesh( this->m_adapter, loops ) };
    }

    std::vector<TMesh> ConvertSphere( const std::shared_ptr<IfcSphere>& sphere ) {
        if( !sphere->m_Radius ) {
            // TODO: Log error
            return {};
        }

        const auto m = this->m_primitivesConverter->ConvertPlacement( sphere->m_Position );

        double radius = sphere->m_Radius->m_value;

        //        \   |   /
        //         2- 1 -nvc
        //        / \ | / \
					//    ---3--- 0 ---7---
        //       \  / | \ /
        //         4- 5 -6
        //        /   |   \

        std::vector<TVector> vertices;
        vertices.push_back( AVector::New( 0.0, 0.0, radius ) ); // top

        const int nvc = this->m_parameters->m_numVerticesPerCircle; // TODO: Use radius
        const int num_vertical_edges = nvc * 0.5;
        double d_vertical_angle = M_PI / double( num_vertical_edges - 1 ); // TODO: adapt to model size and complexity
        double vertical_angle = d_vertical_angle;

        for( int vertical = 1; vertical < num_vertical_edges - 1; ++vertical ) {
            // for each vertical angle, add one horizontal circle
            double vertical_level = cos( vertical_angle ) * radius;
            double radius_at_level = sin( vertical_angle ) * radius;
            double horizontal_angle = 0;
            double d_horizontal_angle = 2.0 * M_PI / double( nvc );
            for( int i = 0; i < nvc; ++i ) {
                vertices.push_back( AVector::New( sin( horizontal_angle ) * radius_at_level, cos( horizontal_angle ) * radius_at_level, vertical_level ) );
                horizontal_angle += d_horizontal_angle;
            }
            vertical_angle += d_vertical_angle;
        }
        vertices.push_back( AVector::New( 0.0, 0.0, -radius ) ); // bottom

        std::vector<TTriangle> polygons;
        // upper triangle fan
        for( int i = 0; i < nvc - 1; ++i ) {
            polygons.push_back( this->m_adapter->CreateTriangle( vertices, { 0, i + 2, i + 1 } ) );
        }
        polygons.push_back( this->m_adapter->CreateTriangle( vertices, { 0, 1, nvc } ) );

        for( int vertical = 1; vertical < num_vertical_edges - 2; ++vertical ) {
            int offset_inner = nvc * ( vertical - 1 ) + 1;
            int offset_outer = nvc * vertical + 1;
            for( int i = 0; i < nvc - 1; ++i ) {
                polygons.push_back( this->m_adapter->CreateTriangle( vertices, { offset_inner + i, offset_inner + 1 + i, offset_outer + 1 + i } ) );
                polygons.push_back( this->m_adapter->CreateTriangle( vertices, { offset_outer + 1 + i, offset_outer + i, offset_inner + i } ) );
            }
            polygons.push_back( this->m_adapter->CreateTriangle( vertices, { offset_inner + nvc - 1, offset_inner, offset_outer } ) );
            polygons.push_back( this->m_adapter->CreateTriangle( vertices, { offset_outer, offset_outer + nvc - 1, offset_inner + nvc - 1 } ) );
        }

        // lower triangle fan
        int last_index = ( num_vertical_edges - 2 ) * nvc + 1;
        for( int i = 0; i < nvc - 1; ++i ) {
            polygons.push_back( this->m_adapter->CreateTriangle( vertices, { last_index, last_index - ( i + 2 ), last_index - ( i + 1 ) } ) );
        }
        polygons.push_back( this->m_adapter->CreateTriangle( vertices, { last_index, last_index - 1, last_index - nvc } ) );

        std::vector<TMesh> result = { this->m_adapter->CreateMesh( polygons ) };
        this->m_adapter->Transform( &result, m );
        return result;
    }
};
}