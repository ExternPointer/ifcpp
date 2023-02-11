#include "ifcpp/Geometry/Solid.h"
#include "ifcpp/Geometry/Common.h"
#include "ifcpp/Geometry/Matrix.h"
#include "ifcpp/Geometry/csgjs.h"
#include "ifcpp/Geometry/Curve.h"

#include "ifcpp/Ifc/IfcExtrudedAreaSolid.h"
#include "ifcpp/Ifc/IfcFixedReferenceSweptAreaSolid.h"
#include "ifcpp/Ifc/IfcProfileDef.h"
#include "ifcpp/Ifc/IfcSolidModel.h"
#include "ifcpp/Ifc/IfcSweptAreaSolid.h"
#include "ifcpp/Ifc/IfcRevolvedAreaSolid.h"
#include "ifcpp/Ifc/IfcSurfaceCurveSweptAreaSolid.h"
#include "ifcpp/Ifc/IfcSurface.h"

using namespace IFC4X3;

namespace ifcpp {

csgjscpp::Model convertIfcSolidModel( const shared_ptr<IfcSolidModel>& solid_model, float angleFactor ) {
    const auto swept_area_solid = dynamic_pointer_cast<IfcSweptAreaSolid>( solid_model );
    if( swept_area_solid ) {
        // ENTITY IfcSweptAreaSolid
        //	ABSTRACT SUPERTYPE OF(ONEOF(IfcExtrudedAreaSolid, IfcFixedReferenceSweptAreaSolid, IfcRevolvedAreaSolid, IfcSurfaceCurveSweptAreaSolid))
        //	SUBTYPE OF IfcSolidModel;
        //	SweptArea	 :	IfcProfileDef;
        //	Position	 :	OPTIONAL IfcAxis2Placement3D;
        //	WHERE
        //	SweptAreaType	 :	SweptArea.ProfileType = IfcProfileTypeEnum.Area;
        // END_ENTITY;

        const auto& swept_area = swept_area_solid->m_SweptArea;
        if( !swept_area ) {
            // TODO: Log error
            return {};
        }

        // check if local coordinate system is specified for extrusion
        auto swept_area_pos = Matrix::GetIdentity();
        if( swept_area_solid->m_Position ) {
            shared_ptr<IfcAxis2Placement3D> swept_area_position = swept_area_solid->m_Position;
            swept_area_pos = ConvertAxis2Placement3D( swept_area_position );
        }

        const auto extruded_area = dynamic_pointer_cast<IfcExtrudedAreaSolid>( swept_area_solid );
        if( extruded_area ) {
            convertIfcExtrudedAreaSolid( extruded_area, item_data_solid );
            item_data->addItemData( item_data_solid );
            item_data->applyTransformToItem( swept_area_pos );
            return;
        }
        shared_ptr<ProfileConverter> profile_converter = m_profile_cache->getProfileConverter( swept_area );
        const std::vector<std::vector<vec2>>& profile_paths = profile_converter->getCoordinates();

        const auto fixed_reference_swept_area_solid = dynamic_pointer_cast<IfcFixedReferenceSweptAreaSolid>( swept_area_solid );
        if( fixed_reference_swept_area_solid ) {
            /*
            // Directrix	 : OPTIONAL IfcCurve;
            // StartParam	 : OPTIONAL IfcParameterValue;
            // EndParam	 : OPTIONAL IfcParameterValue;
            // FixedReference	 : IfcDirection;

            shared_ptr<IfcCurve>& ifc_directrix_curve = fixed_reference_swept_area_solid->m_Directrix;
            // shared_ptr<IfcParameterValue>& ifc_start_param = fixed_reference_swept_area_solid->m_StartParam;				//optional
            // shared_ptr<IfcParameterValue>& ifc_end_param = fixed_reference_swept_area_solid->m_EndParam;					//optional
            // shared_ptr<IfcDirection>& ifc_fixed_reference = fixed_reference_swept_area_solid->m_FixedReference;				// TODO: apply fixed
            // reference
            messageCallback( "IfcFixedReferenceSweptAreaSolid: Fixed reference not implemented", StatusCallback::MESSAGE_TYPE_WARNING, __FUNC__,
                             fixed_reference_swept_area_solid.get() );

            std::vector<vec3> segment_start_points;
            std::vector<vec3> basis_curve_points;
            m_curve_converter->convertIfcCurve( ifc_directrix_curve, basis_curve_points, segment_start_points, true );

            m_sweeper->sweepArea( basis_curve_points, profile_paths, fixed_reference_swept_area_solid.get(), item_data_solid );
            item_data->addItemData( item_data_solid );
            item_data->applyTransformToItem( swept_area_pos );

            return;
             */
            // TODO: Implement
            return {};
        }

        const auto revolved_area_solid = dynamic_pointer_cast<IfcRevolvedAreaSolid>( swept_area_solid );
        if( revolved_area_solid ) {
            convertIfcRevolvedAreaSolid( revolved_area_solid, item_data_solid );
            item_data->addItemData( item_data_solid );
            item_data->applyTransformToItem( swept_area_pos );
            return;
        }

        shared_ptr<IfcSurfaceCurveSweptAreaSolid> surface_curve_swept_area_solid = dynamic_pointer_cast<IfcSurfaceCurveSweptAreaSolid>( swept_area_solid );
        if( surface_curve_swept_area_solid ) {
            shared_ptr<IfcCurve>& ifc_directrix_curve = surface_curve_swept_area_solid->m_Directrix;
            // shared_ptr<IfcParameterValue>& ifc_start_param = surface_curve_swept_area_solid->m_StartParam;				//optional
            // shared_ptr<IfcParameterValue>& ifc_end_param = surface_curve_swept_area_solid->m_EndParam;					//optional
            shared_ptr<IfcSurface>& ifc_reference_surface = surface_curve_swept_area_solid->m_ReferenceSurface; // TODO: apply start_param, end_param
            //messageCallback( "IfcSurfaceCurveSweptAreaSolid: StartParam and EndParam not implemented", StatusCallback::MESSAGE_TYPE_WARNING, __FUNC__,
            //                 surface_curve_swept_area_solid.get() );
            // TODO: Log warning

            std::vector<csgjscpp::Vector> segment_start_points;
            std::vector<csgjscpp::Vector> directrix_curve_points;
            ConvertCurveInternal( ifc_directrix_curve, directrix_curve_points, segment_start_points, true, angleFactor );

            // apply reference curve
            // shared_ptr<carve::input::PolylineSetData> reference_surface_data( new carve::input::PolylineSetData() );
            std::shared_ptr<SurfaceProxy> surface_proxy;
            m_face_converter->convertIfcSurface( ifc_reference_surface, item_data_solid, surface_proxy );

            if( surface_proxy ) {
                for( size_t ii = 0; ii < directrix_curve_points.size(); ++ii ) {
                    // vec3& point_3d = directrix_curve_points[ii];
                    // vec2 point_2d( carve::geom::VECTOR( point_3d.x, point_3d.y ) );
                    // surface_proxy->computePointOnSurface( point_3d, point_3d );
                    //  TODO: implement
                }
            }

            m_sweeper->sweepArea( directrix_curve_points, profile_paths, surface_curve_swept_area_solid.get(), item_data_solid );
            item_data->addItemData( item_data_solid );
            item_data->applyTransformToItem( swept_area_pos );

            return;
        }

        messageCallback( "Unhandled IFC Representation", StatusCallback::MESSAGE_TYPE_WARNING, __FUNC__, solid_model.get() );
    }

    shared_ptr<IfcManifoldSolidBrep> manifold_solid_brep = dynamic_pointer_cast<IfcManifoldSolidBrep>( solid_model );
    if( manifold_solid_brep ) {
        // ENTITY IfcManifoldSolidBrep
        //	ABSTRACT SUPERTYPE OF(ONEOF(IfcAdvancedBrep, IfcFacetedBrep))
        //	SUBTYPE OF IfcSolidModel;
        //		Outer	 :	IfcClosedShell;
        // END_ENTITY;

        shared_ptr<IfcClosedShell>& outer_shell = manifold_solid_brep->m_Outer;
        if( outer_shell ) {
            // first convert outer shell
            std::vector<shared_ptr<IfcFace>>& vec_faces_outer_shell = outer_shell->m_CfsFaces;
            m_face_converter->convertIfcFaceList( vec_faces_outer_shell, item_data, FaceConverter::CLOSED_SHELL );
        }

        shared_ptr<IfcFacetedBrep> faceted_brep = dynamic_pointer_cast<IfcFacetedBrep>( manifold_solid_brep );
        if( faceted_brep ) {
            // no additional attributes
            return;
        }

        shared_ptr<IfcAdvancedBrep> advanced_brep = dynamic_pointer_cast<IfcAdvancedBrep>( manifold_solid_brep );
        if( advanced_brep ) {
            // ENTITY IfcAdvancedBrep	SUPERTYPE OF(IfcAdvancedBrepWithVoids)
            shared_ptr<IfcAdvancedBrepWithVoids> brep_with_voids = dynamic_pointer_cast<IfcAdvancedBrepWithVoids>( solid_model );
            if( brep_with_voids ) {

                // std::vector<shared_ptr<IfcClosedShell> >& vec_voids = advanced_brep_with_voids->m_Voids;

                // TODO: subtract voids from outer shell
#ifdef _DEBUG
                std::cout << "IfcAdvancedBrep not implemented" << std::endl;
#endif
            }
            return;
        }

        messageCallback( "Unhandled IFC Representation", StatusCallback::MESSAGE_TYPE_WARNING, __FUNC__, solid_model.get() );
    }

    shared_ptr<IfcCsgSolid> csg_solid = dynamic_pointer_cast<IfcCsgSolid>( solid_model );
    if( csg_solid ) {
        shared_ptr<IfcCsgSelect> csg_select = csg_solid->m_TreeRootExpression;

        shared_ptr<IfcBooleanResult> csg_select_boolean_result = dynamic_pointer_cast<IfcBooleanResult>( csg_select );
        if( csg_select_boolean_result ) {
            convertIfcBooleanResult( csg_select_boolean_result, item_data );
        } else {
            shared_ptr<IfcCsgPrimitive3D> csg_select_primitive_3d = dynamic_pointer_cast<IfcCsgPrimitive3D>( csg_select );
            if( csg_select_primitive_3d ) {
                convertIfcCsgPrimitive3D( csg_select_primitive_3d, item_data );
            }
        }
        return;
    }

    // shared_ptr<IfcReferencedSectionedSpine> spine = dynamic_pointer_cast<IfcReferencedSectionedSpine>(solid_model);
    // if( spine )
    //{
    //	convertIfcReferencedSectionedSpine( spine, pos, item_data );
    //	return;
    // }

    shared_ptr<IfcSweptDiskSolid> swept_disp_solid = dynamic_pointer_cast<IfcSweptDiskSolid>( solid_model );
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

        shared_ptr<IfcCurve>& directrix_curve = swept_disp_solid->m_Directrix;
        double radius = 0.0;
        const double length_in_meter = m_curve_converter->getPointConverter()->getUnitConverter()->getLengthInMeterFactor();
        if( swept_disp_solid->m_Radius ) {
            radius = swept_disp_solid->m_Radius->m_value * length_in_meter;
        }

        double radius_inner = -1.0;
        if( swept_disp_solid->m_InnerRadius ) {
            radius_inner = swept_disp_solid->m_InnerRadius->m_value * length_in_meter;
        }

        // TODO: handle start param, end param

        std::vector<vec3> segment_start_points;
        std::vector<vec3> basis_curve_points;
        m_curve_converter->convertIfcCurve( directrix_curve, basis_curve_points, segment_start_points, true );
        GeomUtils::removeDuplicates( basis_curve_points );

        shared_ptr<ItemShapeData> item_data_solid( new ItemShapeData() );
        const int nvc = m_geom_settings->getNumVerticesPerCircleWithRadius( radius );
        int nvc_disk = nvc;
        if( radius < 0.1 ) {
            nvc_disk = std::min( 12, nvc );
            if( radius < 0.05 ) {
                nvc_disk = std::min( 8, nvc );
            }
        }

        m_sweeper->sweepDisk( basis_curve_points, swept_disp_solid.get(), item_data_solid, nvc_disk, radius, radius_inner );
        item_data->addItemData( item_data_solid );

        return;
    }

    messageCallback( "Unhandled IFC Representation", StatusCallback::MESSAGE_TYPE_WARNING, __FUNC__, solid_model.get() );
}

}