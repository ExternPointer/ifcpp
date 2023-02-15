#include "ifcpp/Geometry/Geometry.h"

#include "ifcpp/Geometry/Matrix.h"
#include "ifcpp/Geometry/Solid.h"
#include "ifcpp/Geometry/Style.h"
#include "ifcpp/Geometry/Utils.h"
#include <memory>

#include "ifcpp/Geometry/Common.h"
#include "ifcpp/Geometry/Curve.h"
#include "ifcpp/Ifc/Factories/EntityFactory.h"
#include "ifcpp/Ifc/IfcBoolean.h"
#include "ifcpp/Ifc/IfcBooleanResult.h"
#include "ifcpp/Ifc/IfcCartesianPoint.h"
#include "ifcpp/Ifc/IfcCartesianPointList2D.h"
#include "ifcpp/Ifc/IfcClosedShell.h"
#include "ifcpp/Ifc/IfcConic.h"
#include "ifcpp/Ifc/IfcEdge.h"
#include "ifcpp/Ifc/IfcEdgeCurve.h"
#include "ifcpp/Ifc/IfcEdgeLoop.h"
#include "ifcpp/Ifc/IfcFace.h"
#include "ifcpp/Ifc/IfcFaceBasedSurfaceModel.h"
#include "ifcpp/Ifc/IfcFaceBound.h"
#include "ifcpp/Ifc/IfcSpace.h"
#include "ifcpp/Ifc/IfcFeatureElementSubtraction.h"
#include "ifcpp/Ifc/IfcGeometricRepresentationItem.h"
#include "ifcpp/Ifc/IfcGloballyUniqueId.h"
#include "ifcpp/Ifc/IfcLengthMeasure.h"
#include "ifcpp/Ifc/IfcLoop.h"
#include "ifcpp/Ifc/IfcObjectDefinition.h"
#include "ifcpp/Ifc/IfcObjectPlacement.h"
#include "ifcpp/Ifc/IfcOpenShell.h"
#include "ifcpp/Ifc/IfcOrientedEdge.h"
#include "ifcpp/Ifc/IfcPoint.h"
#include "ifcpp/Ifc/IfcPolyLoop.h"
#include "ifcpp/Ifc/IfcProductRepresentation.h"
#include "ifcpp/Ifc/IfcReal.h"
#include "ifcpp/Ifc/IfcRelVoidsElement.h"
#include "ifcpp/Ifc/IfcRepresentation.h"
#include "ifcpp/Ifc/IfcShellBasedSurfaceModel.h"
#include "ifcpp/Ifc/IfcSolidModel.h"
#include "ifcpp/Ifc/IfcStyledItem.h"
#include "ifcpp/Ifc/IfcSubedge.h"
#include "ifcpp/Ifc/IfcSurface.h"
#include "ifcpp/Ifc/IfcTrimmedCurve.h"
#include "ifcpp/Ifc/IfcVertex.h"
#include "ifcpp/Ifc/IfcVertexPoint.h"
#include "ifcpp/Model/BuildingModel.h"
#include "ifcpp/Model/UnitConverter.h"


using namespace IFC4X3;

namespace ifcpp {

std::vector<std::shared_ptr<Geometry>> GenerateGeometry( const std::shared_ptr<BuildingModel>& ifcModel ) {
    std::vector<std::shared_ptr<Geometry>> geometry;
    auto lengthFactor = (float)ifcModel->getUnitConverter()->getLengthInMeterFactor();
    auto angleFactor = (float)ifcModel->getUnitConverter()->getAngleInRadiantFactor();
    for( const auto& idEntityPair: ifcModel->getMapIfcEntities() ) {
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
        auto g = GenerateGeometryFromObject( object, lengthFactor, angleFactor );
        if( !g->m_meshes.empty() ) {
            SubtractOpenings( g, lengthFactor, angleFactor );
            geometry.push_back( g );
        }
    }

    // resolve spatial structure???

    return geometry;
}

void SubtractOpenings( const std::shared_ptr<Geometry>& geometry, float lengthFactor, float angleFactor ) {
    const shared_ptr<IfcElement> ifc_element = dynamic_pointer_cast<IfcElement>( geometry->m_object );
    if( !ifc_element ) {
        return;
    }
    std::vector<weak_ptr<IfcRelVoidsElement>> vec_rel_voids( ifc_element->m_HasOpenings_inverse );
    if( vec_rel_voids.empty() ) {
        return;
    }
    for( auto& rel_voids_weak: vec_rel_voids ) {
        shared_ptr<IfcRelVoidsElement> rel_voids( rel_voids_weak );
        if( !rel_voids ) {
            continue;
        }
        shared_ptr<IfcFeatureElementSubtraction> opening = rel_voids->m_RelatedOpeningElement;
        if( !opening ) {
            continue;
        }
        if( !opening->m_Representation ) {
            continue;
        }
        auto openingGeometry = GenerateGeometryFromObject( opening, lengthFactor, angleFactor );
        for( auto& model: geometry->m_meshes ) {
            for( auto& openingModel: openingGeometry->m_meshes ) {
                auto color = model.color;
                model = csgjscpp::csgsubtract( model, openingModel );
                model.color = color;
            }
        }
    }
}

std::shared_ptr<Geometry> GenerateGeometryFromObject( const std::shared_ptr<IFC4X3::IfcObjectDefinition>& object, float lengthFactor, float angleFactor ) {
    auto geometry = std::make_shared<Geometry>();
    geometry->m_object = object;
    const auto product = dynamic_pointer_cast<IfcProduct>( object );
    if( !product ) {
        return geometry;
    }
    const auto& productRepresentation = product->m_Representation;
    if( !productRepresentation ) {
        return geometry;
    }

    auto matrix = Matrix::GetScale( lengthFactor, lengthFactor, lengthFactor );
    if( product->m_ObjectPlacement ) {
        Matrix::Multiply( &matrix, ConvertObjectPlacement( product->m_ObjectPlacement ) );
    }

    for( const auto& representation: productRepresentation->m_Representations ) {
        for( const auto& item: representation->m_Items ) {
            // ENTITY IfcRepresentationItem  ABSTRACT SUPERTYPE OF(ONEOF(IfcGeometricRepresentationItem,
            // IfcMappedItem, IfcStyledItem, IfcTopologicalRepresentationItem));
            const auto geometric = dynamic_pointer_cast<IfcGeometricRepresentationItem>( item );
            if( geometric ) {
                auto model = ConvertGeometryRepresentation( geometric, angleFactor );
                if( !model.vertices.empty() ) {
                    matrix.Transform( &model );
                    geometry->m_meshes.push_back( model );
                }
            }
        }
    }

    // subtract openings


    // fetch properties

    return geometry;
}

unsigned int GetColor( const std::shared_ptr<IFC4X3::IfcGeometricRepresentationItem>& geometric ) {
    auto color = convertRepresentationStyle( geometric );
    return color != 0 ? color : 4278190335;
}

csgjscpp::Model ConvertGeometryRepresentation( const std::shared_ptr<IFC4X3::IfcGeometricRepresentationItem>& geometric, float angleFactor ) {
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
    auto color = GetColor( geometric );
    csgjscpp::Model result;

    const auto surfaceModel = dynamic_pointer_cast<IfcFaceBasedSurfaceModel>( geometric );
    if( surfaceModel ) {
        result = ConvertFaceBasedSurfaceModel( surfaceModel, angleFactor );
    }

    const auto boolean_result = dynamic_pointer_cast<IfcBooleanResult>( geometric );
    if( boolean_result ) {
        result = ConvertBooleanResult( boolean_result, angleFactor );
    }

    const auto solid_model = dynamic_pointer_cast<IfcSolidModel>( geometric );
    if( solid_model ) {
        result = ConvertSolidModel( solid_model, angleFactor );
    }

    const auto shellModel = dynamic_pointer_cast<IfcShellBasedSurfaceModel>( geometric );
    if( shellModel ) {
        result = ConvertShellBasedSurfaceModel( shellModel, angleFactor );
    }

    const auto ifc_surface = dynamic_pointer_cast<IfcSurface>( geometric );
    if( ifc_surface ) {
        result = ConvertSurface( ifc_surface );
    }

    result.color = color;
    return result;
}


csgjscpp::Model ConvertFaceBasedSurfaceModel( const std::shared_ptr<IFC4X3::IfcFaceBasedSurfaceModel>& surfaceModel, float angleFactor /* style data */ ) {
    std::vector<std::shared_ptr<IfcFace>> faces;
    for( const auto& face_set: surfaceModel->m_FbsmFaces ) {
        std::copy( face_set->m_CfsFaces.begin(), face_set->m_CfsFaces.end(), std::back_inserter( faces ) );
    }
    return ConvertFaceList( faces, angleFactor );
}

csgjscpp::Model ConvertShellBasedSurfaceModel( const std::shared_ptr<IFC4X3::IfcShellBasedSurfaceModel>& shellModel, float angleFactor /* style data */ ) {
    std::vector<std::shared_ptr<IfcFace>> faces;
    for( const auto& shell_select: shellModel->m_SbsmBoundary ) {
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
    return ConvertFaceList( faces, angleFactor );
}


csgjscpp::Model ConvertBooleanResult( const std::shared_ptr<IFC4X3::IfcBooleanResult>& bool_result, float angleFactor ) {
    return convertIfcBooleanResult( bool_result, angleFactor );
}
csgjscpp::Model ConvertSolidModel( const std::shared_ptr<IFC4X3::IfcSolidModel>& bool_result, float angleFactor ) {
    return convertIfcSolidModel( bool_result, angleFactor );
}
csgjscpp::Model ConvertSurface( const std::shared_ptr<IFC4X3::IfcSurface>& surface /*, style data*/ ) {
    return {};
}
}