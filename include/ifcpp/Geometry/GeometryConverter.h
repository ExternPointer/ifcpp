#pragma once

#include "ifcpp/Geometry/CAdapter.h"
#include "ifcpp/Geometry/CurveConverter.h"
#include "ifcpp/Geometry/GeomUtils.h"
#include "ifcpp/Geometry/Matrix.h"
#include "ifcpp/Geometry/Parameters.h"
#include "ifcpp/Geometry/PrimitivesConverter.h"

#include "ifcpp/Ifc/IfcBoolean.h"
#include "ifcpp/Ifc/IfcEdgeCurve.h"
#include "ifcpp/Ifc/IfcEdgeLoop.h"
#include "ifcpp/Ifc/IfcFace.h"
#include "ifcpp/Ifc/IfcFaceBound.h"
#include "ifcpp/Ifc/IfcLoop.h"
#include "ifcpp/Ifc/IfcOrientedEdge.h"
#include "ifcpp/Ifc/IfcPolyLoop.h"
#include "ifcpp/Ifc/IfcSubedge.h"
#include "ifcpp/Ifc/IfcVertexLoop.h"


namespace ifcpp {
using namespace IFC4X3;

template<CVector TVector>
class GeometryConverter {
    using TLoop = std::vector<TVector>;
    using TEdge = std::vector<TVector>;
    using TFaceBounds = std::vector<TLoop>;

    std::shared_ptr<PrimitivesConverter<TVector>> m_primitivesConverter;
    std::shared_ptr<CurveConverter<TVector>> m_curveConverter;
    std::shared_ptr<GeomUtils<TVector>> m_geomUtils;
    std::shared_ptr<Parameters> m_parameters;

public:
    explicit GeometryConverter( const std::shared_ptr<CurveConverter<TVector>>& curveConverter,
                                const std::shared_ptr<PrimitivesConverter<TVector>>& primitivesConverter, const std::shared_ptr<GeomUtils<TVector>> geomUtils,
                                const std::shared_ptr<Parameters>& parameters )
        : m_parameters( parameters )
        , m_curveConverter( curveConverter )
        , m_geomUtils( geomUtils )
        , m_primitivesConverter( primitivesConverter ) {
    }

    TEdge ConvertEdge( const std::shared_ptr<IfcEdge>& edge ) {
        // ENTITY IfcEdge SUPERTYPE OF	(ONEOF(IfcOrientedEdge, IfcEdgeCurve, IfcSubedge))

        if( !edge ) {
            return {};
        }

        const auto start = this->m_primitivesConverter->ConvertPoint( edge->m_EdgeStart );
        const auto end = this->m_primitivesConverter->ConvertPoint( edge->m_EdgeEnd );

        const auto orientedEdge = dynamic_pointer_cast<IfcOrientedEdge>( edge );
        if( orientedEdge ) {
            TEdge points = this->ConvertEdge( orientedEdge->m_EdgeElement );
            if( orientedEdge->m_Orientation && !orientedEdge->m_Orientation->m_value ) {
                std::reverse( points.begin(), points.end() );
            }
            return this->m_curveConverter->TrimCurve( points, start, end );
        }

        const auto subEdge = dynamic_pointer_cast<IfcSubedge>( edge );
        if( subEdge ) {
             return this->m_curveConverter->TrimCurve( this->ConvertEdge( subEdge->m_ParentEdge ), start, end );
        }

        const auto edgeCurve = dynamic_pointer_cast<IfcEdgeCurve>( edge );
        if( edgeCurve ) {
            const auto points = this->m_curveConverter->ConvertCurve( edgeCurve->m_EdgeGeometry );
            if( edgeCurve->m_SameSense && !edgeCurve->m_SameSense->m_value ) {
                std::reverse( std::begin( points ), std::end( points ) );
            }
            return this->m_curveConverter->TrimCurve( points, start, end );
        }

        return { start, end };
    }

    TLoop ConvertLoop( const std::shared_ptr<IfcLoop>& loop ) {
        // ENTITY IfcLoop SUPERTYPE OF(ONEOF(IfcEdgeLoop, IfcPolyLoop, IfcVertexLoop))
        const auto polyLoop = dynamic_pointer_cast<IfcPolyLoop>( loop );
        if( polyLoop ) {
            auto points = this->m_primitivesConverter->ConvertPoints( polyLoop->m_Polygon );
            return this->m_geomUtils->SimplifyLoop( points );
        }

        const auto edgeLoop = dynamic_pointer_cast<IfcEdgeLoop>( loop );
        if( edgeLoop ) {
            TLoop points;
            for( const auto& orientedEdge: edgeLoop->m_EdgeList ) {
                const auto edgePoints = this->ConvertEdge( orientedEdge->m_EdgeElement );
                this->m_geomUtils->AppendToLoop( &points, edgePoints );
            }
            return this->m_geomUtils->SimplifyLoop( points );
        }

        const auto vertexLoop = dynamic_pointer_cast<IfcVertexLoop>( loop );
        if( vertexLoop ) {
            // Not implemented
            // TODO: Implement
        }
        // TODO: Log error
        return {};
    }

    TFaceBounds ConvertFace( const std::shared_ptr<IfcFace>& face ) {
        const auto& ifcBounds = face->m_Bounds;
        TFaceBounds bounds;
        for( const auto& bound: ifcBounds ) {
            auto loop = this->ConvertLoop( bound->m_Bound );
            if( bound->m_Orientation && !bound->m_Orientation->m_value ) {
                std::reverse( loop.begin(), loop.end() );
            }
            bounds.push_back( loop );
        }
        return bounds;
    }

    std::vector<TFaceBounds> ConvertFaces( const std::vector<std::shared_ptr<IfcFace>>& loops ) {
        std::vector<TFaceBounds> result;
        for( const auto& face: loops ) {
            result.push_back( this->ConvertFace( face ) );
        }
        return result;
    }
};

}