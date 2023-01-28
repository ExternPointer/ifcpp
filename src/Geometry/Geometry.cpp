#include "ifcpp/Geometry/Geometry.h"
#include "ifcpp/Geometry/Utils.h"

#include "ifcpp/Ifc/Factories/EntityFactory.h"
#include "ifcpp/Ifc/IfcBoolean.h"
#include "ifcpp/Ifc/IfcBooleanResult.h"
#include "ifcpp/Ifc/IfcClosedShell.h"
#include "ifcpp/Ifc/IfcConnectedFaceSet.h"
#include "ifcpp/Ifc/IfcFace.h"
#include "ifcpp/Ifc/IfcFaceBasedSurfaceModel.h"
#include "ifcpp/Ifc/IfcFaceBound.h"
#include "ifcpp/Ifc/IfcFeatureElementSubtraction.h"
#include "ifcpp/Ifc/IfcGeometricRepresentationItem.h"
#include "ifcpp/Ifc/IfcGeometricSet.h"
#include "ifcpp/Ifc/IfcGloballyUniqueId.h"
#include "ifcpp/Ifc/IfcLoop.h"
#include "ifcpp/Ifc/IfcObjectDefinition.h"
#include "ifcpp/Ifc/IfcOpenShell.h"
#include "ifcpp/Ifc/IfcProductRepresentation.h"
#include "ifcpp/Ifc/IfcRepresentation.h"
#include "ifcpp/Ifc/IfcShellBasedSurfaceModel.h"
#include "ifcpp/Ifc/IfcSolidModel.h"
#include "ifcpp/Ifc/IfcSurface.h"
#include "ifcpp/Model/BuildingModel.h"


using namespace IFC4X3;

namespace ifcpp {

std::vector<std::shared_ptr<Geometry>> GenerateGeometry( std::shared_ptr<BuildingModel> ifcModel ) {
    std::vector<std::shared_ptr<Geometry>> geometry;

    std::vector<std::shared_ptr<IfcObjectDefinition>> vec_object_definitions;

    const auto& map_entities = ifcModel->getMapIfcEntities();
    for( const auto& idEntityPair: map_entities ) {
        auto obj = idEntityPair.second;
        // if( obj )
        auto object_def = dynamic_pointer_cast<IfcObjectDefinition>( obj );
        // if( object_def )
        vec_object_definitions.push_back( object_def );
        /*
        if( object_def->classID() == IFC4X3::IFCSITE )
        {
            std::shared_ptr<IfcSite> ifc_site = dynamic_pointer_cast<IfcSite>(object_def);
            if( ifc_site )
            {
                setIfcSiteToOrigin(ifc_site);
            }
        }
         */
    }

    for( const auto& object_def: vec_object_definitions ) {
        // const int tag = object_def->m_tag;
        // std::string guid;
        // if( object_def->m_GlobalId ) {
        //     guid = object_def->m_GlobalId->m_value;
        // }

        if( !dynamic_pointer_cast<IfcFeatureElementSubtraction>( object_def ) ) {
            continue;
        } /*else if( object_def->classID() == IFCPROJECT ) {
            ifc_project_data = product_geom_input_data;
        }*/

        // try catch
        geometry.push_back( details::GenerateGeometryFromObject( object_def ) );

        // progress calculation
    }

    // subtract openings???

    // resolve spatial structure???

    return geometry;
}

namespace details {
    std::shared_ptr<Geometry> GenerateGeometryFromObject( std::shared_ptr<IFC4X3::IfcObjectDefinition> ifc_object_def ) {
        auto geometry = std::make_shared<Geometry>();
        geometry->m_object = ifc_object_def;
        const auto ifc_product = dynamic_pointer_cast<IfcProduct>( ifc_object_def );
        if( !ifc_product ) {
            return geometry;
        }
        const auto& product_representation = ifc_product->m_Representation;
        if( !product_representation ) {
            return geometry;
        }
        const auto& vec_representations = product_representation->m_Representations;
        for( const auto& representation: vec_representations ) {
            // if( !representation ) {
            //     continue;
            // }
            for( const auto& representation_item: representation->m_Items ) {
                // ENTITY IfcRepresentationItem  ABSTRACT SUPERTYPE OF(ONEOF(IfcGeometricRepresentationItem, IfcMappedItem, IfcStyledItem,
                // IfcTopologicalRepresentationItem));
                const auto geom_item = dynamic_pointer_cast<IfcGeometricRepresentationItem>( representation_item );
                if( geom_item ) {
                    geometry->m_meshes.push_back( ConvertGeometryRepresentation( geom_item ) );
                }
            }
        }
        return geometry;
    }

    csgjscpp::Model ConvertGeometryRepresentation( std::shared_ptr<IFC4X3::IfcGeometricRepresentationItem> geom_item ) {
        // ENTITY IfcGeometricRepresentationItem
        // ABSTRACT SUPERTYPE OF(ONEOF(IfcAnnotationFillArea, IfcBooleanResult, IfcBoundingBox, IfcCartesianPointList, IfcCartesianTransformationOperator,
        // IfcCompositeCurveSegment, IfcCsgPrimitive3D, IfcCurve, IfcDirection, IfcFaceBasedSurfaceModel, IfcFillAreaStyleHatching, IfcFillAreaStyleTiles,
        // IfcGeometricSet, IfcHalfSpaceSolid, IfcLightSource, IfcPlacement, IfcPlanarExtent, IfcPoint, IfcSectionedSpine, IfcShellBasedSurfaceModel,
        // IfcSolidModel, IfcSurface, IfcTessellatedItem, IfcTextLiteral, IfcVector))
        /*
                if( m_geom_settings->handleStyledItems() )
                {
                    std::vector<std::shared_ptr<AppearanceData> > vec_appearance_data;
                    convertRepresentationStyle( geom_item, vec_appearance_data );
                    std::copy( vec_appearance_data.begin(), vec_appearance_data.end(), std::back_inserter( item_data->m_vec_item_appearances ) );
                }
                */

        const auto surface_model = dynamic_pointer_cast<IfcFaceBasedSurfaceModel>( geom_item );
        if( surface_model ) {
            std::vector<std::shared_ptr<IfcFace>> faces;
            for( const auto& face_set: surface_model->m_FbsmFaces ) {
                std::copy( face_set->m_CfsFaces.begin(), face_set->m_CfsFaces.end(), std::back_inserter( faces ) );
            }
            return ConvertFaceList( faces );
        }

        const auto boolean_result = dynamic_pointer_cast<IfcBooleanResult>( geom_item );
        if( boolean_result ) {
            return ConvertBooleanResult( boolean_result );
        }

        const auto solid_model = dynamic_pointer_cast<IfcSolidModel>( geom_item );
        if( solid_model ) {
            return ConvertSolidModel( solid_model );
        }

        const auto shell_based_surface_model = dynamic_pointer_cast<IfcShellBasedSurfaceModel>( geom_item );
        if( shell_based_surface_model ) {
            std::vector<std::shared_ptr<IfcFace>> faces;
            for( const auto& shell_select: shell_based_surface_model->m_SbsmBoundary ) {
                const auto closed_shell = dynamic_pointer_cast<IfcClosedShell>( shell_select );
                if( closed_shell ) {
                    std::copy( closed_shell->m_CfsFaces.begin(), closed_shell->m_CfsFaces.end(), std::back_inserter( faces ) );
                    continue;
                }
                const auto open_shell = dynamic_pointer_cast<IfcOpenShell>( shell_select );
                if( open_shell ) {
                    std::copy( open_shell->m_CfsFaces.begin(), open_shell->m_CfsFaces.end(), std::back_inserter( faces ) );
                    continue;
                }
            }
            return ConvertFaceList( faces );
        }

        const auto ifc_surface = dynamic_pointer_cast<IfcSurface>( geom_item );
        if( ifc_surface ) {
            return ConvertSurface( ifc_surface );
        }

        return {};
    }

    csgjscpp::Model ConvertFaceList( const std::vector<std::shared_ptr<IFC4X3::IfcFace>>& vec_faces /*, style data*/ ) {
        std::vector<csgjscpp::Polygon> polygons;
        for( const std::shared_ptr<IfcFace>& ifc_face: vec_faces ) {
            // if( !ifc_face ) {
            //     continue;
            // }
            const auto& vec_bounds = ifc_face->m_Bounds;
            std::vector<std::vector<csgjscpp::Vector>> face_loops;
            bool mergeAlignedEdges = true;
            for( auto it_bounds = vec_bounds.begin(); it_bounds != vec_bounds.end(); ++it_bounds ) {
                const auto& face_bound = *it_bounds;
                // if( !face_bound ) {
                //     continue;
                // }
                //  ENTITY IfcLoop SUPERTYPE OF(ONEOF(IfcEdgeLoop, IfcPolyLoop, IfcVertexLoop))
                const auto loop = face_bound->m_Bound;
                // if( !loop ) {
                //     if( it_bounds == vec_bounds.begin() ) {
                //         break;
                //     } else {
                //         continue;
                //     }
                // }
                auto loop_points = ConvertLoop( loop );
                if( loop_points.size() < 3 ) {
                    if( it_bounds == vec_bounds.begin() ) {
                        break;
                    } else {
                        continue;
                    }
                }
                bool orientation = true;
                if( face_bound->m_Orientation ) {
                    orientation = face_bound->m_Orientation->m_value;
                }
                if( !orientation ) {
                    std::reverse( loop_points.begin(), loop_points.end() );
                }
                face_loops.push_back( loop_points );
            }
            for( size_t iiLoop = 0; iiLoop < face_loops.size(); ++iiLoop ) {
                ifcpp::UnCloseLoop( face_loops[ iiLoop ] );
                polygons.push_back( ifcpp::CreatePolygon( face_loops[ iiLoop ] /* style data */ ) );
            }
        }
        return ifcpp::CreateModel( polygons );
    }
    csgjscpp::Model ConvertBooleanResult( const std::shared_ptr<IFC4X3::IfcBooleanResult>& bool_result /*, style data*/ );
    csgjscpp::Model ConvertSolidModel( const std::shared_ptr<IFC4X3::IfcSolidModel>& bool_result /*, style data*/ );
    csgjscpp::Model ConvertSurface( const std::shared_ptr<IFC4X3::IfcSurface>& surface /*, style data*/ );
}

}