#pragma once

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
#include "ifcpp/Geometry/Matrix.h"
#include "ifcpp/Geometry/Parameters.h"
#include "ifcpp/Geometry/PrimitivesConverter.h"
#include "ifcpp/Geometry/ProfileConverter.h"
#include "ifcpp/Geometry/VectorAdapter.h"


namespace ifcpp {
using namespace IFC4X3;

template<CAdapter TAdapter>
class SolidConverter {
    using TVector = typename TAdapter::TVector;
    using AVector = VectorAdapter<TVector>;
    using TPolygon = typename TAdapter::TPolygon;

    std::shared_ptr<PrimitivesConverter<TVector>> m_primitivesConverter;
    std::shared_ptr<CurveConverter<TVector>> m_curveConverter;
    std::shared_ptr<ProfileConverter<TVector>> m_profileConverter;
    std::shared_ptr<Extruder<TVector>> m_extruder;
    std::shared_ptr<GeometryConverter<TVector>> m_geometryConverter;
    std::shared_ptr<TAdapter> m_adapter;
    std::shared_ptr<GeomUtils<TVector>> m_geomUtils;
    std::shared_ptr<Parameters> m_parameters;

public:
    SolidConverter( const std::shared_ptr<PrimitivesConverter<TVector>>& primitivesConverter, const std::shared_ptr<CurveConverter<TVector>>& curveConverter,
                    const std::shared_ptr<ProfileConverter<TVector>>& profileConverter, const std::shared_ptr<Extruder<TVector>>& extruder,
                    const std::shared_ptr<GeometryConverter<TVector>> geometryConverter, const std::shared_ptr<TAdapter>& adapter,
                    const std::shared_ptr<GeomUtils<TVector>>& geomUtils, const std::shared_ptr<Parameters>& parameters )
        : m_primitivesConverter( primitivesConverter )
        , m_curveConverter( curveConverter )
        , m_profileConverter( profileConverter )
        , m_extruder( extruder )
        , m_geometryConverter( geometryConverter )
        , m_adapter( adapter )
        , m_geomUtils( geomUtils )
        , m_parameters( parameters ) {
    }

    std::vector<TPolygon> ConvertSolidModel( const shared_ptr<IfcSolidModel>& solid_model ) {
        // ENTITY IfcSolidModel ABSTRACT SUPERTYPE OF(ONEOF(IfcCsgSolid, IfcManifoldSolidBrep, IfcSweptAreaSolid, IfcSweptDiskSolid))

        const auto swept_area_solid = dynamic_pointer_cast<IfcSweptAreaSolid>( solid_model );
        if( swept_area_solid ) {
            return this->ConvertSweptAreaSolid( swept_area_solid );
        }

        shared_ptr<IfcManifoldSolidBrep> manifold_solid_brep = dynamic_pointer_cast<IfcManifoldSolidBrep>( solid_model );
        if( manifold_solid_brep ) {
            return this->ConvertManifoldSolidBrep( manifold_solid_brep );
        }

        const auto csg_solid = dynamic_pointer_cast<IfcCsgSolid>( solid_model );
        if( csg_solid ) {
            return this->ConvertCsgSolid( csg_solid );
        }

        const auto swept_disk_solid = dynamic_pointer_cast<IfcSweptDiskSolid>( solid_model );
        if( swept_disk_solid ) {
            return this->ConvertSweptDiskSolid( swept_disk_solid );
        }

        // TODO: Log error
        return {};
    }

    std::vector<TPolygon> ConvertBooleanResult( const shared_ptr<IfcBooleanResult>& bool_result ) {
        shared_ptr<IfcBooleanOperator>& ifc_boolean_operator = bool_result->m_Operator;
        shared_ptr<IfcBooleanOperand> ifc_first_operand = bool_result->m_FirstOperand;
        shared_ptr<IfcBooleanOperand> ifc_second_operand = bool_result->m_SecondOperand;
        if( !ifc_boolean_operator || !ifc_first_operand || !ifc_second_operand ) {
            // TODO: Log error
            return {};
        }

        const auto operand1 = this->ConvertBooleanOperand( ifc_first_operand );
        const auto operand2 = this->ConvertBooleanOperand( ifc_second_operand );

        switch( ifc_boolean_operator->m_enum ) {
        case IfcBooleanOperator::ENUM_UNION:
            return this->m_adapter->ComputeUnion( operand1, operand2 );
        case IfcBooleanOperator::ENUM_INTERSECTION:
            return this->m_adapter->ComputeIntersection( operand1, operand2 );
        case IfcBooleanOperator::ENUM_DIFFERENCE:
            return this->m_adapter->ComputeDifference( operand1, operand2 );
        default:
            // TODO: Log error
            return {};
        }

        // shared_ptr<IfcBooleanClippingResult> boolean_clipping_result = dynamic_pointer_cast<IfcBooleanClippingResult>( bool_result );
        // if( boolean_clipping_result ) {
        //      no additional attributes, just the type of operands and operator is restricted:
        //      WHERE
        //      FirstOperand is IFCSWEPTAREASOLID or IFCSWEPTDISCSOLID or IFCBOOLEANCLIPPINGRESULT
        //      SecondOperand is IFCHALFSPACESOLID
        //      OperatorType	 :	Operator = DIFFERENCE;
        // }
    }


private:
    std::vector<TPolygon> ConvertSweptAreaSolid( const shared_ptr<IfcSweptAreaSolid>& swept_area_solid ) {
        // ENTITY IfcSweptAreaSolid
        //	ABSTRACT SUPERTYPE OF(ONEOF(IfcExtrudedAreaSolid, IfcFixedReferenceSweptAreaSolid, IfcRevolvedAreaSolid, IfcSurfaceCurveSweptAreaSolid))
        //	SUBTYPE OF IfcSolidModel;
        //	SweptArea	 :	IfcProfileDef;
        //	Position	 :	OPTIONAL IfcAxis2Placement3D;
        //	WHERE
        //	SweptAreaType	 :	SweptArea.ProfileType = IfcProfileTypeEnum.Area;
        // END_ENTITY;

        if( !swept_area_solid->m_SweptArea ) {
            // TODO: Log error
            return {};
        }

        shared_ptr<IfcExtrudedAreaSolid> extruded_area = dynamic_pointer_cast<IfcExtrudedAreaSolid>( swept_area_solid );
        if( extruded_area ) {
            return this->ConvertExtrudedAreaSolid( extruded_area );
        }

        const auto fixed_reference_swept_area_solid = dynamic_pointer_cast<IfcFixedReferenceSweptAreaSolid>( swept_area_solid );
        if( fixed_reference_swept_area_solid ) {
            return this->ConvertFixedReferenceSweptAreaSolid( fixed_reference_swept_area_solid );
        }

        const auto revolved_area_solid = dynamic_pointer_cast<IfcRevolvedAreaSolid>( swept_area_solid );
        if( revolved_area_solid ) {
            return this->ConvertRevolvedAreaSolid( revolved_area_solid );
        }

        shared_ptr<IfcSurfaceCurveSweptAreaSolid> surface_curve_swept_area_solid = dynamic_pointer_cast<IfcSurfaceCurveSweptAreaSolid>( swept_area_solid );
        if( surface_curve_swept_area_solid ) {
            // TODO: Implement
            return {};
        }

        // TODO: Log error
        return {};
    }

    std::vector<TPolygon> ConvertExtrudedAreaSolid( const shared_ptr<IfcExtrudedAreaSolid>& extruded_area ) {
        if( !extruded_area->m_ExtrudedDirection || !extruded_area->m_Depth || !extruded_area->m_SweptArea ) {
            // TODO: Log error
            return {};
        }
        const auto depth = (float)extruded_area->m_Depth->m_value;
        const auto extrusion = this->m_primitivesConverter->ConvertPoint( extruded_area->m_ExtrudedDirection->m_DirectionRatios ) * depth;
        auto loops = this->m_extruder->Extrude( this->m_profileConverter->ConvertProfile( extruded_area->m_SweptArea ), extrusion );
        const auto m = this->m_primitivesConverter->ConvertPlacement( extruded_area->m_Position );
        m.TransformLoops( &loops );
        return this->CreatePolygons( loops );
    }

    std::vector<TPolygon> ConvertFixedReferenceSweptAreaSolid( const shared_ptr<IfcFixedReferenceSweptAreaSolid>& fixed_reference_swept_area_solid ) {
        // Directrix	 : OPTIONAL IfcCurve;
        // StartParam	 : OPTIONAL IfcParameterValue;
        // EndParam	 : OPTIONAL IfcParameterValue;
        // FixedReference	 : IfcDirection;
        //  TODO: Trim curve with StartParam and EndParam, use FixedReference for swept area orientation

        const auto profile_paths = this->m_profileConverter->ConvertProfile( fixed_reference_swept_area_solid->m_SweptArea );
        auto loops = this->m_extruder->Sweep( profile_paths, this->m_curveConverter->ConvertCurve( fixed_reference_swept_area_solid->m_Directrix ) );
        const auto m = this->m_primitivesConverter->ConvertPlacement( fixed_reference_swept_area_solid->m_Position );
        m.TransformLoops( &loops );
        return this->CreatePolygons( loops );
    }

    std::vector<TPolygon> ConvertRevolvedAreaSolid( const shared_ptr<IfcRevolvedAreaSolid>& revolved_area ) {
        if( !revolved_area || !revolved_area->m_Angle || !revolved_area->m_SweptArea || !revolved_area->m_Axis ) {
            // TODO: Log error
            return {};
        }
        float revolveAngle = (float)revolved_area->m_Angle->m_value * this->m_parameters->m_angleFactor;
        TVector axisLocation = AVector::New( 0, 0, 0 );
        TVector axisDirection = AVector::New( 1, 0, 0 );

        axisLocation = this->m_primitivesConverter->ConvertPoint( revolved_area->m_Axis->m_Location );
        axisDirection = this->m_primitivesConverter->ConvertPoint( revolved_area->m_Axis->m_Axis->m_DirectionRatios );

        auto loops =
            this->m_extruder->Revolve( this->m_profileConverter->ConvertProfile( revolved_area->m_SweptArea ), axisLocation, axisDirection, revolveAngle );
        const auto m = this->m_primitivesConverter->ConvertPlacement( revolved_area->m_Position );
        m.TransformLoops( &loops );
        return this->CreatePolygons( loops );
    }

    std::vector<TPolygon> ConvertManifoldSolidBrep( const std::shared_ptr<IfcManifoldSolidBrep>& manifold_solid_brep ) {
        if( !manifold_solid_brep->m_Outer ) {
            // TODO: Log error
            return {};
        }
        // ENTITY IfcManifoldSolidBrep
        //	ABSTRACT SUPERTYPE OF(ONEOF(IfcAdvancedBrep, IfcFacetedBrep))
        //	SUBTYPE OF IfcSolidModel;
        //		Outer	 :	IfcClosedShell;
        // END_ENTITY;

        auto loops = this->m_geometryConverter->ConvertFaces( manifold_solid_brep->m_Outer->m_CfsFaces );

        // TODO: Rework (try to fix points order)
        auto reversed = loops;
        for( auto& l: reversed ) {
            std::reverse( l.begin(), l.end() );
        }
        std::copy( std::begin( reversed ), std::end( reversed ), std::back_inserter( loops ) );

        std::vector<TPolygon> result = this->CreatePolygons( loops );

        shared_ptr<IfcFacetedBrep> faceted_brep = dynamic_pointer_cast<IfcFacetedBrep>( manifold_solid_brep );
        if( faceted_brep ) {
            // no additional attributes
            return result;
        }

        shared_ptr<IfcAdvancedBrep> advanced_brep = dynamic_pointer_cast<IfcAdvancedBrep>( manifold_solid_brep );
        if( advanced_brep ) {
            // ENTITY IfcAdvancedBrep	SUPERTYPE OF(IfcAdvancedBrepWithVoids)
            shared_ptr<IfcAdvancedBrepWithVoids> brep_with_voids = dynamic_pointer_cast<IfcAdvancedBrepWithVoids>( manifold_solid_brep );
            if( brep_with_voids ) {
                // std::vector<shared_ptr<IfcClosedShell> >& vec_voids = advanced_brep_with_voids->m_Voids;
                // TODO: subtract voids from outer shell
            }
            return result;
        }

        // TODO: Log error
        return {};
    }

    std::vector<TPolygon> ConvertCsgSolid( const std::shared_ptr<IfcCsgSolid> csg_solid ) {
        const auto& csg_select = csg_solid->m_TreeRootExpression;
        const auto csg_select_boolean_result = dynamic_pointer_cast<IfcBooleanResult>( csg_select );
        if( csg_select_boolean_result ) {
            return this->ConvertBooleanResult( csg_select_boolean_result );
        }
        const auto csg_select_primitive_3d = dynamic_pointer_cast<IfcCsgPrimitive3D>( csg_select );
        if( csg_select_primitive_3d ) {
            return this->ConvertCsgPrimitive3D( csg_select_primitive_3d );
        }
        // TODO: Log error
        return {};
    }

    std::vector<TPolygon> ConvertSweptDiskSolid( const std::shared_ptr<IfcSweptDiskSolid>& swept_disk_solid ) {
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

        if( !swept_disk_solid->m_Radius ) {
            // TODO: Log error
            return {};
        }


        const auto& directrix_curve = swept_disk_solid->m_Directrix;
        auto radius = (float)swept_disk_solid->m_Radius->m_value;

        float radius_inner = 0.0f;
        if( swept_disk_solid->m_InnerRadius ) {
            radius_inner = (float)swept_disk_solid->m_InnerRadius->m_value;
        }

        // TODO: handle start param, end param

        const auto basis_curve_points = this->m_curveConverter->ConvertCurve( directrix_curve );

        const auto outer = this->m_geomUtils->BuildCircle( radius, 0.0f, (float)( M_PI * 2 ), this->m_parameters->m_numVerticesPerCircle );
        std::vector<TVector> inner;
        if( radius_inner > this->m_parameters->m_epsilon ) {
            inner = this->m_geomUtils->BuildCircle( radius_inner, 0.0f, -(float)( M_PI * 2 ), this->m_parameters->m_numVerticesPerCircle );
        }
        const auto loops = this->m_extruder->Sweep( this->m_geomUtils->CombineLoops( { outer, inner } ), basis_curve_points );
        return this->CreatePolygons( loops );
    }

    std::vector<TPolygon> ConvertBooleanOperand( const shared_ptr<IfcBooleanOperand>& operand_select ) {
        // TYPE IfcBooleanOperand = SELECT	(IfcBooleanResult	,IfcCsgPrimitive3D	,IfcHalfSpaceSolid	,IfcSolidModel);

        const auto solid_model = dynamic_pointer_cast<IfcSolidModel>( operand_select );
        if( solid_model ) {
            return this->ConvertSolidModel( solid_model );
        }

        const auto half_space_solid = dynamic_pointer_cast<IfcHalfSpaceSolid>( operand_select );
        if( half_space_solid ) {
            return this->ConvertHalfSpaceSolid( half_space_solid );
        }

        const auto boolean_result = dynamic_pointer_cast<IfcBooleanResult>( operand_select );
        if( boolean_result ) {
            return this->ConvertBooleanResult( boolean_result );
        }

        const auto csg_primitive3D = dynamic_pointer_cast<IfcCsgPrimitive3D>( operand_select );
        if( csg_primitive3D ) {
            return this->ConvertCsgPrimitive3D( csg_primitive3D );
        }

        // TODO: Log error
        return {};
    }

    std::vector<TPolygon> ConvertHalfSpaceSolid( const shared_ptr<IfcHalfSpaceSolid>& half_space_solid ) {
        // ENTITY IfcHalfSpaceSolid SUPERTYPE OF(ONEOF(IfcBoxedHalfSpace, IfcPolygonalBoundedHalfSpace))

        std::vector<std::vector<TVector>> resultLoops;

        const auto elem_base_surface = dynamic_pointer_cast<IfcElementarySurface>( half_space_solid->m_BaseSurface );
        if( !elem_base_surface ) {
            // TODO: Log error
            return {};
        }

        auto planeMatrix = this->m_primitivesConverter->ConvertPlacement( elem_base_surface->m_Position );
        if( half_space_solid->m_AgreementFlag && !half_space_solid->m_AgreementFlag->m_value ) {
            planeMatrix = Matrix<TVector>::GetMultiplied( Matrix<TVector>::GetScale( 1, 1, -1 ), planeMatrix );
        }

        auto boxed = std::dynamic_pointer_cast<IfcBoxedHalfSpace>( half_space_solid );
        if( boxed ) {
            auto x = (float)boxed->m_Enclosure->m_XDim->m_value;
            auto y = (float)boxed->m_Enclosure->m_YDim->m_value;
            auto z = (float)boxed->m_Enclosure->m_YDim->m_value;

            const auto p = this->m_primitivesConverter->ConvertPoint( boxed->m_Enclosure->m_Corner );

            std::vector<TVector> profile = {
                AVector::New( p.x + 0, p.y + 0, p.z ),
                AVector::New( p.x + x, p.y + 0, p.z ),
                AVector::New( p.x + x, p.y + y, p.z ),
                AVector::New( p.x + 0, p.y + y, p.z ),
            };

            auto loops = this->m_extruder->Extrude( profile, AVector::New( 0, 0, 1 ) * z );
            planeMatrix.TransformLoops( &loops );
            std::copy( loops.begin(), loops.end(), std::back_inserter( resultLoops ) );
        }

        const auto polygonal = std::dynamic_pointer_cast<IfcPolygonalBoundedHalfSpace>( half_space_solid );
        if( polygonal ) {
            // FIXME: Always loop?????
            auto profile = this->m_geomUtils->SimplifyLoop( this->m_curveConverter->ConvertCurve( polygonal->m_PolygonalBoundary ) );
            const auto m = this->m_primitivesConverter->ConvertPlacement( polygonal->m_Position );

            auto planeNormal = AVector::New( planeMatrix.data[ 0 ][ 2 ], planeMatrix.data[ 1 ][ 2 ], planeMatrix.data[ 2 ][ 2 ] );
            auto profileNormal = AVector::New( m.data[ 0 ][ 2 ], m.data[ 1 ][ 2 ], m.data[ 2 ][ 2 ] );
            m.TransformLoop( &profile );
            auto extrusion = profileNormal * this->m_parameters->m_modelMaxSize;
            if( AVector::Dot( extrusion, planeNormal ) < 0 ) {
                extrusion = -extrusion;
            }
            profile = this->GetProjection( planeMatrix, profile );
            auto loops = this->m_extruder->Extrude( profile, extrusion );
            planeMatrix.TransformLoops( &loops );
            std::copy( loops.begin(), loops.end(), std::back_inserter( resultLoops ) );
        }

        std::vector<TVector> profile = {
            AVector::New( -this->m_parameters->m_modelMaxSize, -this->m_parameters->m_modelMaxSize ),
            AVector::New( this->m_parameters->m_modelMaxSize, -this->m_parameters->m_modelMaxSize ),
            AVector::New( this->m_parameters->m_modelMaxSize, this->m_parameters->m_modelMaxSize ),
            AVector::New( -this->m_parameters->m_modelMaxSize, this->m_parameters->m_modelMaxSize ),
        };
        profile = this->GetProjection( planeMatrix, profile );
        auto loops = this->m_extruder->Extrude( profile, AVector::New( 0, 0, 1 ) * this->m_parameters->m_modelMaxSize );
        std::copy( loops.begin(), loops.end(), std::back_inserter( resultLoops ) );


        return this->CreatePolygons( resultLoops );
    }

    std::vector<TPolygon> ConvertCsgPrimitive3D( const shared_ptr<IfcCsgPrimitive3D>& csg_primitive ) {
        // ENTITY IfcCsgPrimitive3D  ABSTRACT SUPERTYPE OF(ONEOF(IfcBlock, IfcRectangularPyramid, IfcRightCircularCone, IfcRightCircularCylinder, IfcSphere
        const auto primitive_placement_matrix = this->m_primitivesConverter->ConvertPlacement( csg_primitive->m_Position );

        const auto block = dynamic_pointer_cast<IfcBlock>( csg_primitive );
        if( block ) {
            return this->ConvertBlock( block );
        }

        const auto rectangular_pyramid = dynamic_pointer_cast<IfcRectangularPyramid>( csg_primitive );
        if( rectangular_pyramid ) {
            return this->ConvertRectangularPyramid( rectangular_pyramid );
        }

        const auto right_circular_cone = dynamic_pointer_cast<IfcRightCircularCone>( csg_primitive );
        if( right_circular_cone ) {
            return this->ConvertRightCircularCone( right_circular_cone );
        }

        shared_ptr<IfcRightCircularCylinder> right_circular_cylinder = dynamic_pointer_cast<IfcRightCircularCylinder>( csg_primitive );
        if( right_circular_cylinder ) {
            return this->ConvertRightCircularCylinder( right_circular_cylinder );
        }

        shared_ptr<IfcSphere> sphere = dynamic_pointer_cast<IfcSphere>( csg_primitive );
        if( sphere ) {
            return this->ConvertSphere( sphere );
        }

        // TODO: Log error
        return {};
    }

    std::vector<TPolygon> ConvertBlock( const std::shared_ptr<IfcBlock>& block ) {
        if( !block->m_XLength || !block->m_YLength || !block->m_ZLength ) {
            // TODO: Log error
            return {};
        }

        const auto primitive_placement_matrix = this->m_primitivesConverter->ConvertPlacement( block->m_Position );

        const float x_length = (float)block->m_XLength->m_value * 0.5f;
        const float y_length = (float)block->m_YLength->m_value * 0.5f;
        const float z_length = (float)block->m_ZLength->m_value * 0.5f;

        std::vector<TVector> vertices;
        vertices.push_back( AVector::New( x_length, y_length, z_length ) );
        vertices.push_back( AVector::New( -x_length, y_length, z_length ) );
        vertices.push_back( AVector::New( -x_length, -y_length, z_length ) );
        vertices.push_back( AVector::New( x_length, -y_length, z_length ) );
        vertices.push_back( AVector::New( x_length, y_length, -z_length ) );
        vertices.push_back( AVector::New( -x_length, y_length, -z_length ) );
        vertices.push_back( AVector::New( -x_length, -y_length, -z_length ) );
        vertices.push_back( AVector::New( x_length, -y_length, -z_length ) );

        std::vector<TPolygon> polygons;

        polygons.push_back( this->m_adapter->CreatePolygon( vertices, { 0, 1, 2 } ) );
        polygons.push_back( this->m_adapter->CreatePolygon( vertices, { 2, 3, 0 } ) );

        polygons.push_back( this->m_adapter->CreatePolygon( vertices, { 7, 6, 5 } ) );
        polygons.push_back( this->m_adapter->CreatePolygon( vertices, { 5, 4, 7 } ) );

        polygons.push_back( this->m_adapter->CreatePolygon( vertices, { 0, 4, 5 } ) );
        polygons.push_back( this->m_adapter->CreatePolygon( vertices, { 5, 1, 0 } ) );

        polygons.push_back( this->m_adapter->CreatePolygon( vertices, { 1, 5, 6 } ) );
        polygons.push_back( this->m_adapter->CreatePolygon( vertices, { 6, 2, 1 } ) );

        polygons.push_back( this->m_adapter->CreatePolygon( vertices, { 2, 6, 7 } ) );
        polygons.push_back( this->m_adapter->CreatePolygon( vertices, { 7, 3, 2 } ) );

        polygons.push_back( this->m_adapter->CreatePolygon( vertices, { 3, 7, 4 } ) );
        polygons.push_back( this->m_adapter->CreatePolygon( vertices, { 4, 0, 3 } ) );

        this->m_adapter->Transform( &polygons, primitive_placement_matrix );

        return polygons;
    }

    std::vector<TPolygon> ConvertRectangularPyramid( const std::shared_ptr<IfcRectangularPyramid>& rectangular_pyramid ) {
        if( !rectangular_pyramid->m_XLength || !rectangular_pyramid->m_YLength || !rectangular_pyramid->m_Height ) {
            // TODO: Log error
            return {};
        }

        const auto primitive_placement_matrix = this->m_primitivesConverter->ConvertPlacement( rectangular_pyramid->m_Position );

        const float x_length = (float)rectangular_pyramid->m_XLength->m_value * 0.5f;
        const float y_length = (float)rectangular_pyramid->m_YLength->m_value * 0.5f;
        const float height = (float)rectangular_pyramid->m_Height->m_value * 0.5f;

        std::vector<TVector> vertices;
        vertices.push_back( AVector::New( 0, 0, height ) );
        vertices.push_back( AVector::New( x_length, -y_length, 0.0 ) );
        vertices.push_back( AVector::New( -x_length, -y_length, 0.0 ) );
        vertices.push_back( AVector::New( -x_length, y_length, 0.0 ) );
        vertices.push_back( AVector::New( x_length, y_length, 0.0 ) );

        std::vector<TPolygon> polygons;
        polygons.push_back( this->m_adapter->CreatePolygon( vertices, { 1, 2, 3 } ) );
        polygons.push_back( this->m_adapter->CreatePolygon( vertices, { 3, 4, 1 } ) );
        polygons.push_back( this->m_adapter->CreatePolygon( vertices, { 0, 2, 1 } ) );
        polygons.push_back( this->m_adapter->CreatePolygon( vertices, { 0, 1, 4 } ) );
        polygons.push_back( this->m_adapter->CreatePolygon( vertices, { 0, 4, 3 } ) );
        polygons.push_back( this->m_adapter->CreatePolygon( vertices, { 0, 3, 2 } ) );

        this->m_adapter->Transform( &polygons, primitive_placement_matrix );
        return polygons;
    }

    std::vector<TPolygon> ConvertRightCircularCone( const std::shared_ptr<IfcRightCircularCone>& right_circular_cone ) {
        const auto primitive_placement_matrix = this->m_primitivesConverter->ConvertPlacement( right_circular_cone->m_Position );

        if( !right_circular_cone->m_Height || !right_circular_cone->m_BottomRadius ) {
            // TODO: Log error
            return {};
        }

        const auto height = (float)right_circular_cone->m_Height->m_value;
        const auto radius = (float)right_circular_cone->m_BottomRadius->m_value;

        std::vector<TVector> vertices;
        vertices.push_back( AVector::New( 0.0f, 0.0f, height ) ); // top
        vertices.push_back( AVector::New( 0.0f, 0.0f, 0.0f ) ); // bottom center

        float angle = 0;
        auto d_angle = (float)( 2.0f * M_PI / float( this->m_parameters->m_numVerticesPerCircle - 1 ) ); // TODO: Use radius
        for( int i = 0; i < this->m_parameters->m_numVerticesPerCircle; ++i ) {
            vertices.push_back( AVector::New( sinf( angle ) * radius, cosf( angle ) * radius, 0.0f ) );
            angle += d_angle;
        }

        std::vector<TPolygon> polygons;
        // outer shape
        for( int i = 0; i < this->m_parameters->m_numVerticesPerCircle - 1; ++i ) {
            polygons.push_back( this->m_adapter->CreatePolygon( vertices, { 0, i + 3, i + 2 } ) );
        }
        polygons.push_back( this->m_adapter->CreatePolygon( vertices, { 0, 2, this->m_parameters->m_numVerticesPerCircle + 1 } ) );

        // bottom circle
        for( int i = 0; i < this->m_parameters->m_numVerticesPerCircle - 1; ++i ) {
            polygons.push_back( this->m_adapter->CreatePolygon( vertices, { 1, i + 2, i + 3 } ) );
        }
        polygons.push_back( this->m_adapter->CreatePolygon( vertices, { 1, this->m_parameters->m_numVerticesPerCircle + 1, 2 } ) );
        this->m_adapter->Transform( &polygons, primitive_placement_matrix );
        return polygons;
    }

    std::vector<TPolygon> ConvertRightCircularCylinder( const std::shared_ptr<IfcRightCircularCylinder>& right_circular_cylinder ) {
        if( !right_circular_cylinder->m_Height || !right_circular_cylinder->m_Radius ) {
            // TODO: Log error
            return {};
        }

        const auto primitive_placement_matrix = this->m_primitivesConverter->ConvertPlacement( right_circular_cylinder->m_Position );
        auto height = (float)right_circular_cylinder->m_Height->m_value;
        auto radius = (float)right_circular_cylinder->m_Radius->m_value;

        // TODO: Use radius
        const auto circle = this->m_geomUtils->BuildCircle( radius, 0, (float)( M_PI * 2 ), this->m_parameters->m_numVerticesPerCircle );
        auto loops = this->m_extruder->Extrude( this->m_geomUtils->SimplifyLoop( circle ), AVector::New( 0, 0, height ) );
        primitive_placement_matrix.TransformLoops( &loops );
        return this->CreatePolygons( loops );
    }

    std::vector<TPolygon> ConvertSphere( const std::shared_ptr<IfcSphere>& sphere ) {
        if( !sphere->m_Radius ) {
            // TODO: Log error
            return {};
        }

        const auto primitive_placement_matrix = this->m_primitivesConverter->ConvertPlacement( sphere->m_Position );

        double radius = sphere->m_Radius->m_value;

        //        \   |   /
        //         2- 1 -nvc
        //        / \ | / \
					//    ---3--- 0 ---7---
        //       \  / | \ /
        //         4- 5 -6
        //        /   |   \

        std::vector<TVector> vertices;
        vertices.push_back( AVector::New( 0.0f, 0.0f, radius ) ); // top

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

        std::vector<TPolygon> polygons;
        // upper triangle fan
        for( int i = 0; i < nvc - 1; ++i ) {
            polygons.push_back( this->m_adapter->CreatePolygon( vertices, { 0, i + 2, i + 1 } ) );
        }
        polygons.push_back( this->m_adapter->CreatePolygon( vertices, { 0, 1, nvc } ) );

        for( int vertical = 1; vertical < num_vertical_edges - 2; ++vertical ) {
            int offset_inner = nvc * ( vertical - 1 ) + 1;
            int offset_outer = nvc * vertical + 1;
            for( int i = 0; i < nvc - 1; ++i ) {
                polygons.push_back( this->m_adapter->CreatePolygon( vertices, { offset_inner + i, offset_inner + 1 + i, offset_outer + 1 + i } ) );
                polygons.push_back( this->m_adapter->CreatePolygon( vertices, { offset_outer + 1 + i, offset_outer + i, offset_inner + i } ) );
            }
            polygons.push_back( this->m_adapter->CreatePolygon( vertices, { offset_inner + nvc - 1, offset_inner, offset_outer } ) );
            polygons.push_back( this->m_adapter->CreatePolygon( vertices, { offset_outer, offset_outer + nvc - 1, offset_inner + nvc - 1 } ) );
        }

        // lower triangle fan
        int last_index = ( num_vertical_edges - 2 ) * nvc + 1;
        for( int i = 0; i < nvc - 1; ++i ) {
            polygons.push_back( this->m_adapter->CreatePolygon( vertices, { last_index, last_index - ( i + 2 ), last_index - ( i + 1 ) } ) );
        }
        polygons.push_back( this->m_adapter->CreatePolygon( vertices, { last_index, last_index - 1, last_index - nvc } ) );
        this->m_adapter->Transform( &polygons, primitive_placement_matrix );
        return polygons;
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
            if( indices.size() < 3) {
                continue;
            }
            for( int i = 0; i < indices.size() - 2; i += 3 ) {
                result.push_back( this->m_adapter->CreatePolygon( l, { indices[ i ], indices[ i + 1 ], indices[ i + 2 ] } ) );
            }
        }
        return result;
    }

    std::vector<TVector> GetProjection( Matrix<TVector> planeMatrix, std::vector<TVector> loop ) {
        auto right = AVector::New( planeMatrix.data[ 0 ][ 0 ], planeMatrix.data[ 1 ][ 0 ], planeMatrix.data[ 2 ][ 0 ] );
        auto up = AVector::New( planeMatrix.data[ 0 ][ 1 ], planeMatrix.data[ 1 ][ 1 ], planeMatrix.data[ 2 ][ 1 ] );
        auto planeNormal = AVector::New( planeMatrix.data[ 0 ][ 2 ], planeMatrix.data[ 1 ][ 2 ], planeMatrix.data[ 2 ][ 2 ] );
        auto planePosition = AVector::New( planeMatrix.data[ 0 ][ 3 ], planeMatrix.data[ 1 ][ 3 ], planeMatrix.data[ 2 ][ 3 ] );

        std::vector<TVector> result = loop;

        for( auto& p: result ) {
            float x = AVector::Dot( right, p - planePosition );
            float y = AVector::Dot( up, p - planePosition );
            p = planePosition + right * x + up * y;
        }

        return result;
    }
};
}