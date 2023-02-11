#include "ifcpp/Geometry/Common.h"
#include "ifcpp/Geometry/Matrix.h"
#include "ifcpp/Geometry/csgjs.h"

#include "ifcpp/Ifc/IfcAxis2Placement3D.h"
#include "ifcpp/Ifc/IfcCartesianPoint.h"
#include "ifcpp/Ifc/IfcDirection.h"
#include "ifcpp/Ifc/IfcLengthMeasure.h"
#include "ifcpp/Ifc/IfcLocalPlacement.h"
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

}