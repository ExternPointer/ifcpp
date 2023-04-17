#pragma once

#include "ifcpp/Ifc/IfcAdvancedBrep.h"
#include "ifcpp/Ifc/IfcAdvancedBrepWithVoids.h"
#include "ifcpp/Ifc/IfcBooleanResult.h"
#include "ifcpp/Ifc/IfcClosedShell.h"
#include "ifcpp/Ifc/IfcCsgPrimitive3D.h"
#include "ifcpp/Ifc/IfcCsgSelect.h"
#include "ifcpp/Ifc/IfcCsgSolid.h"
#include "ifcpp/Ifc/IfcExtrudedAreaSolid.h"
#include "ifcpp/Ifc/IfcFacetedBrep.h"
#include "ifcpp/Ifc/IfcFixedReferenceSweptAreaSolid.h"
#include "ifcpp/Ifc/IfcManifoldSolidBrep.h"
#include "ifcpp/Ifc/IfcRevolvedAreaSolid.h"
#include "ifcpp/Ifc/IfcSolidModel.h"
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
    using TVector = TAdapter::TVector;
    using AVector = VectorAdapter<TVector>;
    using TLoop = std::vector<TVector>;

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

    std::vector<TLoop> ConvertSolidModel( const shared_ptr<IfcSolidModel>& solid_model ) {
        // ENTITY IfcSolidModel ABSTRACT SUPERTYPE OF(ONEOF(IfcCsgSolid, IfcManifoldSolidBrep, IfcSweptAreaSolid, IfcSweptDiskSolid))
        const auto swept_area_solid = dynamic_pointer_cast<IfcSweptAreaSolid>( solid_model );
        if( swept_area_solid && swept_area_solid->m_SweptArea ) {
            // ENTITY IfcSweptAreaSolid
            //	ABSTRACT SUPERTYPE OF(ONEOF(IfcExtrudedAreaSolid, IfcFixedReferenceSweptAreaSolid, IfcRevolvedAreaSolid, IfcSurfaceCurveSweptAreaSolid))
            //	SUBTYPE OF IfcSolidModel;
            //	SweptArea	 :	IfcProfileDef;
            //	Position	 :	OPTIONAL IfcAxis2Placement3D;
            //	WHERE
            //	SweptAreaType	 :	SweptArea.ProfileType = IfcProfileTypeEnum.Area;
            // END_ENTITY;

            const auto swept_area_pos = this->m_primitivesConverter->ConvertPlacement( swept_area_solid->m_Position );

            shared_ptr<IfcExtrudedAreaSolid> extruded_area = dynamic_pointer_cast<IfcExtrudedAreaSolid>( swept_area_solid );
            if( extruded_area ) {
                return swept_area_pos.GetTransformedLoops( this->ConvertExtrudedAreaSolid( extruded_area ) );
            }

            const auto profile_paths = this->m_profileConverter->ConvertProfile( swept_area_solid->m_SweptArea );
            const auto fixed_reference_swept_area_solid = dynamic_pointer_cast<IfcFixedReferenceSweptAreaSolid>( swept_area_solid );
            if( fixed_reference_swept_area_solid ) {
                // Directrix	 : OPTIONAL IfcCurve;
                // StartParam	 : OPTIONAL IfcParameterValue;
                // EndParam	 : OPTIONAL IfcParameterValue;
                // FixedReference	 : IfcDirection;
                //  TODO: Trim curve with StartParam and EndParam, use FixedReference for swept area orientation
                return swept_area_pos.GetTransformedLoops(
                    this->m_extruder->Sweep( profile_paths, this->m_curveConverter->ConvertCurve( fixed_reference_swept_area_solid->m_Directrix ) ) );
            }

            const auto revolved_area_solid = dynamic_pointer_cast<IfcRevolvedAreaSolid>( swept_area_solid );
            if( revolved_area_solid ) {
                return swept_area_pos.GetTransformedLoops( this->ConvertRevolvedAreaSolid( revolved_area_solid ) );
            }

            shared_ptr<IfcSurfaceCurveSweptAreaSolid> surface_curve_swept_area_solid = dynamic_pointer_cast<IfcSurfaceCurveSweptAreaSolid>( swept_area_solid );
            if( surface_curve_swept_area_solid ) {
                // TODO: Implement
                return {};
            }

            // TODO: Log error
        }

        shared_ptr<IfcManifoldSolidBrep> manifold_solid_brep = dynamic_pointer_cast<IfcManifoldSolidBrep>( solid_model );
        if( manifold_solid_brep && manifold_solid_brep->m_Outer ) {
            // ENTITY IfcManifoldSolidBrep
            //	ABSTRACT SUPERTYPE OF(ONEOF(IfcAdvancedBrep, IfcFacetedBrep))
            //	SUBTYPE OF IfcSolidModel;
            //		Outer	 :	IfcClosedShell;
            // END_ENTITY;

            auto result = this->m_geometryConverter->ConvertFaces( manifold_solid_brep->m_Outer->m_CfsFaces );

            shared_ptr<IfcFacetedBrep> faceted_brep = dynamic_pointer_cast<IfcFacetedBrep>( manifold_solid_brep );
            if( faceted_brep ) {
                // no additional attributes
                return result;
            }

            shared_ptr<IfcAdvancedBrep> advanced_brep = dynamic_pointer_cast<IfcAdvancedBrep>( manifold_solid_brep );
            if( advanced_brep ) {
                // ENTITY IfcAdvancedBrep	SUPERTYPE OF(IfcAdvancedBrepWithVoids)
                shared_ptr<IfcAdvancedBrepWithVoids> brep_with_voids = dynamic_pointer_cast<IfcAdvancedBrepWithVoids>( solid_model );
                if( brep_with_voids ) {
                    // std::vector<shared_ptr<IfcClosedShell> >& vec_voids = advanced_brep_with_voids->m_Voids;
                    // TODO: subtract voids from outer shell
                }
                return result;
            }

            // TODO: Log error
        }

        const auto csg_solid = dynamic_pointer_cast<IfcCsgSolid>( solid_model );
        if( csg_solid ) {
            const auto& csg_select = csg_solid->m_TreeRootExpression;
            const auto csg_select_boolean_result = dynamic_pointer_cast<IfcBooleanResult>( csg_select );
            if( csg_select_boolean_result ) {
                return this->ConvertBooleanResult( csg_select_boolean_result );
            }
            const auto csg_select_primitive_3d = dynamic_pointer_cast<IfcCsgPrimitive3D>( csg_select );
            if( csg_select_primitive_3d ) {
                return this->ConvertCsgPrimitive3D( csg_select_primitive_3d );
            }
        }

        const auto swept_disp_solid = dynamic_pointer_cast<IfcSweptDiskSolid>( solid_model );
        if( swept_disp_solid ) {
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

            const auto& directrix_curve = swept_disp_solid->m_Directrix;
            float radius = 0.0f;
            if( swept_disp_solid->m_Radius ) {
                radius = (float)swept_disp_solid->m_Radius->m_value;
            }

            float radius_inner = 0.0f;
            if( swept_disp_solid->m_InnerRadius ) {
                radius_inner = (float)swept_disp_solid->m_InnerRadius->m_value;
            }

            // TODO: handle start param, end param

            const auto basis_curve_points = this->m_curveConverter->ConvertCurve( directrix_curve );

            const auto outer = this->m_geomUtils->BuildCircle( radius, 0.0f, (float)( M_PI * 2 ), this->m_parameters->m_NumVerticesPerCircle );
            const auto inner = this->m_geomUtils->BuildCircle( radius_inner, 0.0f, -(float)( M_PI * 2 ), this->m_parameters->m_NumVerticesPerCircle );

            return this->m_extruder->Sweep( this->m_geomUtils->CombineLoops( { outer, inner } ), basis_curve_points );
        }

        // TODO: Log error
        return {};
    }

private:
    std::vector<TLoop> ConvertExtrudedAreaSolid( const shared_ptr<IfcExtrudedAreaSolid>& extruded_area ) {
        if( !extruded_area->m_ExtrudedDirection || !extruded_area->m_Depth || extruded_area->m_SweptArea ) {
            // TODO: Log error
            return {};
        }
        const auto extrusion =
            this->m_primitivesConverter->ConvertPoint( extruded_area->m_ExtrudedDirection->m_DirectionRatios ) * (float)extruded_area->m_Depth->m_value;
        return this->m_extruder->Extrude( this->m_profileConverter->ConvertProfile( extruded_area->m_SweptArea ), extrusion );
    }

    std::vector<TLoop> ConvertRevolvedAreaSolid( const shared_ptr<IfcRevolvedAreaSolid>& revolved_area ) {
        if( !revolved_area || !revolved_area->m_Angle || !revolved_area->m_SweptArea || !revolved_area->m_Axis ) {
            // TODO: Log error
            return {};
        }
        float revolveAngle = revolved_area->m_Angle * this->m_parameters->m_angleFactor;
        TVector axisLocation = AVector::New( 0, 0, 0 );
        TVector axisDirection = AVector::New( 1, 0, 0 );

        axisLocation = this->m_primitivesConverter->ConvertPoint( revolved_area->m_Axis->m_Location );
        axisDirection = this->m_primitivesConverter->ConvertPoint( revolved_area->m_Axis->m_Axis->m_DirectionRatios );

        return this->m_extruder->Revolve( this->m_profileConverter->ConvertProfile( revolved_area->m_SweptArea, axisLocation, axisDirection, revolveAngle ) );
    }


};

}