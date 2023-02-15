#include "ifcpp/Geometry/Common.h"
#include "ifcpp/Geometry/Matrix.h"
#include "ifcpp/Geometry/Utils.h"
#include "ifcpp/Geometry/csgjs.h"

#include "ifcpp/Ifc/IfcAxis2Placement3D.h"
#include "ifcpp/Ifc/IfcBoolean.h"
#include "ifcpp/Ifc/IfcCartesianPoint.h"
#include "ifcpp/Ifc/IfcDirection.h"
#include "ifcpp/Ifc/IfcFace.h"
#include "ifcpp/Ifc/IfcFaceBound.h"
#include "ifcpp/Ifc/IfcLengthMeasure.h"
#include "ifcpp/Ifc/IfcEdgeCurve.h"
#include "ifcpp/Ifc/IfcLocalPlacement.h"
#include "ifcpp/Ifc/IfcTrimmedCurve.h"
#include "ifcpp/Ifc/IfcPolyLoop.h"
#include "ifcpp/Ifc/IfcEdgeLoop.h"
#include "ifcpp/Ifc/IfcOrientedEdge.h"
#include "ifcpp/Ifc/IfcSubedge.h"
#include "ifcpp/Ifc/IfcReal.h"
#include "ifcpp/Ifc/IfcVertexPoint.h"


using namespace IFC4X3;

namespace ifcpp {

csgjscpp::Vector ConvertPoint( const std::vector<std::shared_ptr<IfcLengthMeasure>>& coords ) {
    float x = 0.0f, y = 0.0f, z = 0.0f;
    if( coords.size() > 2 ) {
        x = (float)coords[ 0 ]->m_value;
        y = (float)coords[ 1 ]->m_value;
        z = (float)coords[ 2 ]->m_value;
    } else if( coords.size() > 1 ) {
        x = (float)coords[ 0 ]->m_value;
        y = (float)coords[ 1 ]->m_value;
    } else {
        // TODO: Log error
    }

    return { x, y, z };
}

csgjscpp::Vector ConvertVertex( const std::shared_ptr<IfcVertex>& vertex ) {
    const auto vertexPoint = dynamic_pointer_cast<IfcVertexPoint>( vertex );
    if( vertexPoint ) {
        if( vertexPoint->m_VertexGeometry ) {
            const auto& vertexGeometry = vertexPoint->m_VertexGeometry;
            const auto cartesianPoint = dynamic_pointer_cast<IfcCartesianPoint>( vertexGeometry );
            return ConvertCartesianPoint( cartesianPoint );
        }
    }
    // TODO: Log error
    return {};
}

csgjscpp::Vector ConvertCartesianPoint( const std::shared_ptr<IfcCartesianPoint>& point ) {
    float x = 0.0f, y = 0.0f, z = 0.0f;
    const auto& coords = point->m_Coordinates;

    return ConvertPoint( point->m_Coordinates );
}

std::vector<csgjscpp::Vector> ConvertCartesianPoints( const std::vector<std::shared_ptr<IfcCartesianPoint>>& points ) {
    std::vector<csgjscpp::Vector> result;
    for( const auto& point: points ) {
        result.emplace_back( ConvertCartesianPoint( point ) );
    }
    return result;
}


Matrix ConvertObjectPlacement( const std::shared_ptr<IFC4X3::IfcObjectPlacement>& objectPlacement ) {
    // TODO: Recursion
    shared_ptr<IfcLocalPlacement> localPlacement = dynamic_pointer_cast<IfcLocalPlacement>( objectPlacement );
    auto relativeTo = Matrix::GetIdentity();
    auto result = Matrix::GetIdentity();
    if( localPlacement ) {
        if( localPlacement->m_PlacementRelTo ) {
            shared_ptr<IfcObjectPlacement> relativeToPlacement = localPlacement->m_PlacementRelTo;
            relativeTo = ConvertObjectPlacement( relativeToPlacement );
        }
        const auto& axis2placement = localPlacement->m_RelativePlacement;
        if( axis2placement ) {
            shared_ptr<IfcPlacement> placement = dynamic_pointer_cast<IfcPlacement>( axis2placement );
            if( placement ) {
                result = ConvertPlacement( placement );
                Matrix::Multiply( &result, relativeTo );
            } else {
                // TODO: Log error
            }
        } else {
            // TODO: Log error
        }
    }
    return result;
}

Matrix ConvertAxis2Placement3D( const std::shared_ptr<IFC4X3::IfcAxis2Placement3D>& axis2placement3d ) {
    csgjscpp::Vector t( 0.0f, 0.0f, 0.0f );
    csgjscpp::Vector x( 1.0f, 0.0f, 0.0f );
    csgjscpp::Vector y( 0.0f, 1.0f, 0.0f );
    csgjscpp::Vector z( 0.0f, 0.0f, 1.0f );

    if( axis2placement3d->m_Location ) {
        const auto cartesianPoint = dynamic_pointer_cast<IfcCartesianPoint>( axis2placement3d->m_Location );
        if( cartesianPoint ) {
            const auto& coords = cartesianPoint->m_Coordinates;
            if( coords.size() > 2 ) {
                t = csgjscpp::Vector( (float)coords[ 0 ]->m_value, (float)coords[ 1 ]->m_value, (float)coords[ 2 ]->m_value );
            }
        }
    }
    if( axis2placement3d->m_Axis ) {
        const auto& axis = axis2placement3d->m_Axis->m_DirectionRatios;
        if( axis.size() > 2 ) {
            z = csgjscpp::Vector( (float)axis[ 0 ]->m_value, (float)axis[ 1 ]->m_value, (float)axis[ 2 ]->m_value );
        }
    }
    if( axis2placement3d->m_RefDirection ) {
        const auto& ref = axis2placement3d->m_RefDirection->m_DirectionRatios;
        if( ref.size() > 2 ) {
            x = csgjscpp::Vector( (float)ref[ 0 ]->m_value, (float)ref[ 1 ]->m_value, (float)ref[ 2 ]->m_value );
        }
    }

    y = csgjscpp::cross( z, x );
    x = csgjscpp::cross( y, z );

    x = csgjscpp::unit( x );
    y = csgjscpp::unit( y );
    z = csgjscpp::unit( z );

    return Matrix::CreateFromAxis( x, y, z, t );
}

Matrix ConvertPlacement( const std::shared_ptr<IFC4X3::IfcPlacement>& placement ) {
    // TODO: Implement another placements converts
    const auto axis2placement3d = dynamic_pointer_cast<IfcAxis2Placement3D>( placement );
    if( !axis2placement3d ) {
        return Matrix::GetIdentity();
    }
    return ConvertAxis2Placement3D( axis2placement3d );
}

std::vector<csgjscpp::Vector> ConvertPoints( const std::vector<std::vector<shared_ptr<IfcLengthMeasure>>>& pointList ) {
    std::vector<csgjscpp::Vector> result;
    for( const auto& coords: pointList ) {
        result.push_back( ConvertPoint( coords ) );
    }
    return result;
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
}