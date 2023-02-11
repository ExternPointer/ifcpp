#include "ifcpp/Geometry/Geometry.h"

#include "ifcpp/Geometry/Matrix.h"
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
#include "ifcpp/Ifc/IfcRepresentation.h"
#include "ifcpp/Ifc/IfcShellBasedSurfaceModel.h"
#include "ifcpp/Ifc/IfcSolidModel.h"
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
        geometry.push_back( GenerateGeometryFromObject( object, lengthFactor, angleFactor ) );
    }

    // resolve spatial structure???

    return geometry;
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

    for( const auto& representation: productRepresentation->m_Representations ) {
        for( const auto& item: representation->m_Items ) {
            // ENTITY IfcRepresentationItem  ABSTRACT SUPERTYPE OF(ONEOF(IfcGeometricRepresentationItem,
            // IfcMappedItem, IfcStyledItem, IfcTopologicalRepresentationItem));
            const auto geometric = dynamic_pointer_cast<IfcGeometricRepresentationItem>( item );
            if( geometric ) {
                geometry->m_meshes.push_back( ConvertGeometryRepresentation( geometric, angleFactor ) );
            }
        }
    }

    // subtract openings

    auto matrix = Matrix::GetScale( lengthFactor, lengthFactor, lengthFactor );
    if( product->m_ObjectPlacement ) {
        Matrix::Multiply( &matrix, ConvertObjectPlacement( product->m_ObjectPlacement ) );
    }
    // apply matrix

    // fetch properties


    return geometry;
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

    const auto surfaceModel = dynamic_pointer_cast<IfcFaceBasedSurfaceModel>( geometric );
    if( surfaceModel ) {
        return ConvertFaceBasedSurfaceModel( surfaceModel, angleFactor );
    }

    const auto boolean_result = dynamic_pointer_cast<IfcBooleanResult>( geometric );
    if( boolean_result ) {
        return ConvertBooleanResult( boolean_result );
    }

    const auto solid_model = dynamic_pointer_cast<IfcSolidModel>( geometric );
    if( solid_model ) {
        return ConvertSolidModel( solid_model );
    }

    const auto shellModel = dynamic_pointer_cast<IfcShellBasedSurfaceModel>( geometric );
    if( shellModel ) {
        return ConvertShellBasedSurfaceModel( shellModel, angleFactor );
    }

    const auto ifc_surface = dynamic_pointer_cast<IfcSurface>( geometric );
    if( ifc_surface ) {
        return ConvertSurface( ifc_surface );
    }

    return {};
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

csgjscpp::Model ConvertFaceList( const std::vector<std::shared_ptr<IFC4X3::IfcFace>>& faces, float angleFactor /*, style data*/ ) {
    std::vector<csgjscpp::Polygon> polygons;
    for( const std::shared_ptr<IfcFace>& face: faces ) {
        const auto& bounds = face->m_Bounds;
        std::vector<std::vector<csgjscpp::Vector>> loops;
        for( const auto& bound: bounds ) {
            //  ENTITY IfcLoop SUPERTYPE OF(ONEOF(IfcEdgeLoop, IfcPolyLoop, IfcVertexLoop))
            const auto loop = bound->m_Bound;
            auto points = ConvertLoop( loop, angleFactor );
            if( bound->m_Orientation && !bound->m_Orientation->m_value ) {
                std::reverse( points.begin(), points.end() );
            }
            loops.push_back( points );
        }
        for( auto& loop: loops ) {
            ifcpp::UnCloseLoop( loop );
            polygons.push_back( ifcpp::CreatePolygon( loop /* style data */ ) );
        }
    }
    return ifcpp::CreateModel( polygons );
}

std::vector<csgjscpp::Vector> ConvertLoop( const std::shared_ptr<IFC4X3::IfcLoop>& loop, float angleFactor ) {
    std::vector<csgjscpp::Vector> loop_points;
    const auto poly_loop = dynamic_pointer_cast<IfcPolyLoop>( loop );
    if( poly_loop ) {
        const auto& ifc_points = poly_loop->m_Polygon;
        loop_points = ConvertCartesianPoints( ifc_points );
        UnCloseLoop( loop_points );
    }
    const auto edge_loop = dynamic_pointer_cast<IfcEdgeLoop>( loop );
    if( edge_loop ) {
        for( const shared_ptr<IfcOrientedEdge>& oriented_edge: edge_loop->m_EdgeList ) {
            const auto& edge = oriented_edge->m_EdgeElement;
            auto edge_points = ConvertEdge( edge, angleFactor );
        }
    }
    return loop_points;
}


std::vector<csgjscpp::Vector> ConvertEdge( const std::shared_ptr<IFC4X3::IfcEdge>& edge, float angleFactor ) {
    // ENTITY IfcEdge SUPERTYPE OF	(ONEOF(IfcOrientedEdge, IfcEdgeCurve, IfcSubedge))

    const auto& edge_start = edge->m_EdgeStart;
    const auto& edge_end = edge->m_EdgeEnd;

    const auto orientedEdge = dynamic_pointer_cast<IfcOrientedEdge>( edge );
    if( orientedEdge ) {
        bool orientedEdgeOrientation = orientedEdge->m_Orientation->m_value;
        auto loopPointsEdgeElement = ConvertEdge( orientedEdge->m_EdgeElement, angleFactor );
        if( !orientedEdgeOrientation ) {
            std::reverse( loopPointsEdgeElement.begin(), loopPointsEdgeElement.end() );
        }
        return loopPointsEdgeElement;
    }

    const auto subEdge = dynamic_pointer_cast<IfcSubedge>( edge );
    if( subEdge ) {
        if( subEdge->m_ParentEdge ) {
            return ConvertEdge( subEdge->m_ParentEdge, angleFactor );
        }
    }

    const auto edgeCurve = dynamic_pointer_cast<IfcEdgeCurve>( edge );
    if( edgeCurve ) {
        bool edgeSameSense = !edgeCurve->m_SameSense || edgeCurve->m_SameSense->m_value;

        auto p0 = ConvertVertex( edge_start );
        auto p1 = ConvertVertex( edge_end );

        if( !edgeSameSense ) {
            std::swap( p0, p1 );
        }

        std::vector<csgjscpp::Vector> curvePoints;
        std::vector<csgjscpp::Vector> segmentStartPoints;
        const shared_ptr<IfcCurve> edgeCurveCurve = edgeCurve->m_EdgeGeometry;
        bool senseAgreement = true;

        if( edgeCurveCurve ) {
            const auto trimmedCurve = dynamic_pointer_cast<IfcTrimmedCurve>( edgeCurveCurve );
            if( trimmedCurve ) {
                const std::shared_ptr<IfcCurve> basisCurve = trimmedCurve->m_BasisCurve;
                if( basisCurve ) {
                    std::vector<shared_ptr<IfcTrimmingSelect>> curve_trim1_vec;
                    std::vector<shared_ptr<IfcTrimmingSelect>> curve_trim2_vec;

                    std::shared_ptr<IfcCartesianPoint> trim1( new IfcCartesianPoint() );
                    trim1->m_Coordinates.push_back( std::make_shared<IfcLengthMeasure>( p0.x ) );
                    trim1->m_Coordinates.push_back( std::make_shared<IfcLengthMeasure>( p0.y ) );
                    trim1->m_Coordinates.push_back( std::make_shared<IfcLengthMeasure>( p0.z ) );
                    curve_trim1_vec.push_back( trim1 );

                    shared_ptr<IfcCartesianPoint> trim2( new IfcCartesianPoint() );
                    trim2->m_Coordinates.push_back( std::make_shared<IfcLengthMeasure>( p1.x ) );
                    trim2->m_Coordinates.push_back( std::make_shared<IfcLengthMeasure>( p1.y ) );
                    trim2->m_Coordinates.push_back( std::make_shared<IfcLengthMeasure>( p1.z ) );
                    curve_trim2_vec.push_back( trim2 );
                    return ConvertCurve( basisCurve, curve_trim1_vec, curve_trim2_vec, senseAgreement, angleFactor );
                }
            } else {
                std::vector<shared_ptr<IfcTrimmingSelect>> curve_trim1_vec;
                std::vector<shared_ptr<IfcTrimmingSelect>> curve_trim2_vec;
                return ConvertCurve( edgeCurveCurve, curve_trim1_vec, curve_trim2_vec, senseAgreement, angleFactor );
            }
        } else {
            curvePoints.push_back( p0 );
            curvePoints.push_back( p1 );
        }

        return curvePoints;
    }
    return {};
}


csgjscpp::Model ConvertBooleanResult( const std::shared_ptr<IFC4X3::IfcBooleanResult>& bool_result /*, style data*/ ) {
    return {};
}
csgjscpp::Model ConvertSolidModel( const std::shared_ptr<IFC4X3::IfcSolidModel>& bool_result /*, style data*/ ) {
    return {};
}
csgjscpp::Model ConvertSurface( const std::shared_ptr<IFC4X3::IfcSurface>& surface /*, style data*/ ) {
    return {};
}
}