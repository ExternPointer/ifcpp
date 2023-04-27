#pragma once

#include "ifcpp/Geometry/CAdapter.h"
#include "ifcpp/Geometry/CurveConverter.h"
#include "ifcpp/Geometry/Extruder.h"
#include "ifcpp/Geometry/GeomUtils.h"
#include "ifcpp/Geometry/Matrix.h"
#include "ifcpp/Geometry/Parameters.h"
#include "ifcpp/Geometry/PrimitiveTypesConverter.h"
#include "ifcpp/Geometry/ProfileConverter.h"
#include "ifcpp/Geometry/SplineConverter.h"
#include "ifcpp/Geometry/VectorAdapter.h"

#include "ifcpp/Ifc/IfcBoolean.h"
#include "ifcpp/Ifc/IfcCurveBoundedPlane.h"
#include "ifcpp/Ifc/IfcCurveBoundedSurface.h"
#include "ifcpp/Ifc/IfcCylindricalSurface.h"
#include "ifcpp/Ifc/IfcEdgeCurve.h"
#include "ifcpp/Ifc/IfcEdgeLoop.h"
#include "ifcpp/Ifc/IfcFace.h"
#include "ifcpp/Ifc/IfcFaceBound.h"
#include "ifcpp/Ifc/IfcFaceOuterBound.h"
#include "ifcpp/Ifc/IfcLoop.h"
#include "ifcpp/Ifc/IfcOrientedEdge.h"
#include "ifcpp/Ifc/IfcPlane.h"
#include "ifcpp/Ifc/IfcPolyLoop.h"
#include "ifcpp/Ifc/IfcRectangularTrimmedSurface.h"
#include "ifcpp/Ifc/IfcSubedge.h"
#include "ifcpp/Ifc/IfcSurfaceOfLinearExtrusion.h"
#include "ifcpp/Ifc/IfcSurfaceOfRevolution.h"
#include "ifcpp/Ifc/IfcSweptSurface.h"
#include "ifcpp/Ifc/IfcVertexLoop.h"


namespace ifcpp {
using namespace IFC4X3;

template<CVector TVector>
class GeometryConverter {
    using TLoop = std::vector<TVector>;
    using TEdge = std::vector<TVector>;
    using AVector = VectorAdapter<TVector>;

    std::shared_ptr<PrimitiveTypesConverter<TVector>> m_primitivesConverter;
    std::shared_ptr<CurveConverter<TVector>> m_curveConverter;
    std::shared_ptr<SplineConverter<TVector>> m_splineConverter;
    std::shared_ptr<GeomUtils<TVector>> m_geomUtils;
    std::shared_ptr<Extruder<TVector>> m_extruder;
    std::shared_ptr<ProfileConverter<TVector>> m_profileConverter;
    std::shared_ptr<Parameters> m_parameters;

public:
    explicit GeometryConverter( const std::shared_ptr<CurveConverter<TVector>>& curveConverter,
                                const std::shared_ptr<PrimitiveTypesConverter<TVector>>& primitivesConverter,
                                const std::shared_ptr<SplineConverter<TVector>>& splineConverter, const std::shared_ptr<GeomUtils<TVector>> geomUtils,
                                const std::shared_ptr<Extruder<TVector>>& extruder, const std::shared_ptr<ProfileConverter<TVector>>& profileConverter,
                                const std::shared_ptr<Parameters>& parameters )
        : m_parameters( parameters )
        , m_curveConverter( curveConverter )
        , m_splineConverter( splineConverter )
        , m_geomUtils( geomUtils )
        , m_extruder( extruder )
        , m_profileConverter( profileConverter )
        , m_primitivesConverter( primitivesConverter ) {
    }

    TEdge ConvertEdge( const std::shared_ptr<IfcEdge>& edge ) {
        // ENTITY IfcEdge SUPERTYPE OF	(ONEOF(IfcOrientedEdge, IfcEdgeCurve, IfcSubedge))

        if( !edge ) {
            return {};
        }

        const auto start = this->m_primitivesConverter->ConvertVertex( edge->m_EdgeStart );
        const auto end = this->m_primitivesConverter->ConvertVertex( edge->m_EdgeEnd );

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
            auto points = this->m_curveConverter->ConvertCurve( edgeCurve->m_EdgeGeometry );
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
            return { this->m_primitivesConverter->ConvertVertex( vertexLoop->m_LoopVertex ) };
        }
        // TODO: Log error
        return {};
    }

    TLoop ConvertFace( const std::shared_ptr<IfcFace>& face ) {
        TLoop outer;
        std::vector<TLoop> inners;

        std::vector<TLoop> loops;
        for( const auto& bound: face->m_Bounds ) {
            auto loop = this->ConvertLoop( bound->m_Bound );
            if( bound->m_Orientation && !bound->m_Orientation->m_value ) {
                std::reverse( loop.begin(), loop.end() );
            }
            loops.push_back( loop );
        }
        loops = this->m_geomUtils->BringToFrontOuterLoop( loops );
        if( loops.empty() ) {
            return {};
        }

        outer = loops[0];
        std::copy( std::begin( loops ) + 1, std::end( loops ), std::back_inserter( inners ) );

        auto inner = this->m_geomUtils->CombineLoops( inners );
        // TODO: Check order
        return this->m_geomUtils->CombineLoops( { outer, inner } );
    }

    std::vector<TLoop> ConvertFaces( const std::vector<std::shared_ptr<IfcFace>>& loops ) {
        std::vector<TLoop> result;
        for( const auto& face: loops ) {
            result.push_back( this->ConvertFace( face ) );
        }
        return result;
    }

    std::vector<TLoop> ConvertSurface( const shared_ptr<IfcSurface>& surface ) {
        // ENTITY IfcSurface ABSTRACT SUPERTYPE OF(ONEOF(IfcBoundedSurface, IfcElementarySurface, IfcSweptSurface))

        const auto bounded_surface = dynamic_pointer_cast<IfcBoundedSurface>( surface );
        if( bounded_surface ) {
            // ENTITY IfcBoundedSurface ABSTRACT SUPERTYPE OF(ONEOF(IfcBSplineSurface, IfcCurveBoundedPlane, IfcCurveBoundedSurface,
            // IfcRectangularTrimmedSurface))
            if( const auto bspline_surface = dynamic_pointer_cast<IfcBSplineSurface>( bounded_surface ) ) {
                return this->m_splineConverter->ConvertBSplineSurface( bspline_surface );
            } else if( const auto curve_bounded_plane = dynamic_pointer_cast<IfcCurveBoundedPlane>( bounded_surface ) ) {
                // ENTITY IfcCurveBoundedPlane SUBTYPE OF IfcBoundedSurface;

                const auto curve_bounded_plane_matrix = this->m_primitivesConverter->ConvertPlacement( curve_bounded_plane->m_BasisSurface->m_Position );

                const auto outer = this->m_geomUtils->SimplifyLoop( this->m_curveConverter->ConvertCurve( curve_bounded_plane->m_OuterBoundary ) );
                std::vector<TLoop> inners;
                for( auto& inner: curve_bounded_plane->m_InnerBoundaries ) {
                    inners.push_back( this->m_geomUtils->SimplifyLoop( this->m_curveConverter->ConvertCurve( inner ) ) );
                }

                auto inner = this->m_geomUtils->CombineLoops( inners );
                // TODO: Check order
                auto result = this->m_geomUtils->CombineLoops( { outer, inner } );
                curve_bounded_plane_matrix.TransformLoop( &result );
                return { result };
            } else if( const auto curve_bounded_surface = dynamic_pointer_cast<IfcCurveBoundedSurface>( bounded_surface ) ) {
                // TODO: Implement
                return {};
            } else if( dynamic_pointer_cast<IfcRectangularTrimmedSurface>( bounded_surface ) ) {
                // TODO: Implement
                return {};
            }
            // TODO: Log error
            return {};
        }

        shared_ptr<IfcElementarySurface> elementary_surface = dynamic_pointer_cast<IfcElementarySurface>( surface );
        if( elementary_surface ) {
            // ENTITY IfcElementarySurface	ABSTRACT SUPERTYPE OF(ONEOF(IfcCylindricalSurface, IfcPlane))

            const auto planeMatrix = this->m_primitivesConverter->ConvertPlacement( elementary_surface->m_Position );

            shared_ptr<IfcPlane> elementary_surface_plane = dynamic_pointer_cast<IfcPlane>( elementary_surface );
            if( elementary_surface_plane ) {
                TLoop result = {
                    AVector::New( -this->m_parameters->m_modelMaxSize, -this->m_parameters->m_modelMaxSize ),
                    AVector::New( this->m_parameters->m_modelMaxSize, -this->m_parameters->m_modelMaxSize ),
                    AVector::New( this->m_parameters->m_modelMaxSize, this->m_parameters->m_modelMaxSize ),
                    AVector::New( -this->m_parameters->m_modelMaxSize, this->m_parameters->m_modelMaxSize ),
                };
                planeMatrix.TransformLoop( &result );
                return { result };
            }

            shared_ptr<IfcCylindricalSurface> cylindrical_surface = dynamic_pointer_cast<IfcCylindricalSurface>( elementary_surface );
            if( cylindrical_surface ) {
                auto radius = (float)cylindrical_surface->m_Radius->m_value;
                int nvc = this->m_parameters->m_numVerticesPerCircle; // TODO: Use radius
                auto circle = this->m_geomUtils->BuildCircle( radius, 0, (float)M_PI * 2.0f, nvc );
                for( auto& p: circle ) {
                    p.z = -this->m_parameters->m_modelMaxSize;
                }
                auto result = this->m_extruder->Extrude( circle, AVector::New( 0, 0, 2.0f * this->m_parameters->m_modelMaxSize ), false );
                planeMatrix.TransformLoops( &result );
                return result;
            }

            // TODO: Log error
            return {};
        }

        shared_ptr<IfcSweptSurface> swept_surface = dynamic_pointer_cast<IfcSweptSurface>( surface );
        if( swept_surface ) {
            // ENTITY IfcSweptSurface	ABSTRACT SUPERTYPE OF(ONEOF(IfcSurfaceOfLinearExtrusion, IfcSurfaceOfRevolution))
            // shared_ptr<IfcProfileDef>& swept_surface_profile = swept_surface->m_SweptCurve;
            const auto m = this->m_primitivesConverter->ConvertPlacement( swept_surface->m_Position );
            const auto curve = this->m_profileConverter->ConvertProfile( swept_surface->m_SweptCurve );

            shared_ptr<IfcSurfaceOfLinearExtrusion> linear_extrusion = dynamic_pointer_cast<IfcSurfaceOfLinearExtrusion>( swept_surface );
            if( linear_extrusion ) {
                const auto extrusion = this->m_primitivesConverter->ConvertPoint( linear_extrusion->m_ExtrudedDirection->m_DirectionRatios ) *
                    (float)linear_extrusion->m_Depth->m_value;
                auto result = this->m_extruder->Extrude( curve, extrusion, false );
                m.TransformLoops( &result );
                return result;
            }

            const auto surface_of_revolution = dynamic_pointer_cast<IfcSurfaceOfRevolution>( swept_surface );
            if( surface_of_revolution ) {
                const auto axisDirection = this->m_primitivesConverter->ConvertPoint( surface_of_revolution->m_AxisPosition->m_Axis->m_DirectionRatios );
                const auto axisLocation = this->m_primitivesConverter->ConvertPoint( surface_of_revolution->m_AxisPosition->m_Location );
                auto result = this->m_extruder->Revolve( curve, axisLocation, axisDirection, (float)M_PI * 2.0f, false );
                m.TransformLoops( &result );
                return result;
            }

            // TODO: Log error
            return {};
        }

        // TODO: Log error
        return {};
    }
};

}