/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcAxis2Placement3D.h"
#include "ifcpp/Ifc/IfcDirection.h"
#include "ifcpp/Ifc/IfcLengthMeasure.h"
#include "ifcpp/Ifc/IfcPresentationLayerAssignment.h"
#include "ifcpp/Ifc/IfcProfileDef.h"
#include "ifcpp/Ifc/IfcStyledItem.h"
#include "ifcpp/Ifc/IfcSurfaceOfLinearExtrusion.h"

// ENTITY IfcSurfaceOfLinearExtrusion 
IFC4X3::IfcSurfaceOfLinearExtrusion::IfcSurfaceOfLinearExtrusion( int tag ) { m_tag = tag; }
void IFC4X3::IfcSurfaceOfLinearExtrusion::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCSURFACEOFLINEAREXTRUSION" << "(";
	if( m_SweptCurve ) { stream << "#" << m_SweptCurve->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_Position ) { stream << "#" << m_Position->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_ExtrudedDirection ) { stream << "#" << m_ExtrudedDirection->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_Depth ) { m_Depth->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcSurfaceOfLinearExtrusion::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcSurfaceOfLinearExtrusion::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 4 ){ std::stringstream err; err << "Wrong parameter count for entity IfcSurfaceOfLinearExtrusion, expecting 4, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	readEntityReference( args[0], m_SweptCurve, map, errorStream );
	readEntityReference( args[1], m_Position, map, errorStream );
	readEntityReference( args[2], m_ExtrudedDirection, map, errorStream );
	m_Depth = IfcLengthMeasure::createObjectFromSTEP( args[3], map, errorStream );
}
void IFC4X3::IfcSurfaceOfLinearExtrusion::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcSweptSurface::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "ExtrudedDirection", m_ExtrudedDirection ) );
	vec_attributes.emplace_back( std::make_pair( "Depth", m_Depth ) );
}
void IFC4X3::IfcSurfaceOfLinearExtrusion::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcSweptSurface::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcSurfaceOfLinearExtrusion::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcSweptSurface::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcSurfaceOfLinearExtrusion::unlinkFromInverseCounterparts()
{
	IfcSweptSurface::unlinkFromInverseCounterparts();
}
