/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcBoolean.h"
#include "ifcpp/Ifc/IfcBoundaryCurve.h"
#include "ifcpp/Ifc/IfcCurveBoundedSurface.h"
#include "ifcpp/Ifc/IfcPresentationLayerAssignment.h"
#include "ifcpp/Ifc/IfcStyledItem.h"
#include "ifcpp/Ifc/IfcSurface.h"

// ENTITY IfcCurveBoundedSurface 
IFC4X3::IfcCurveBoundedSurface::IfcCurveBoundedSurface( int tag ) { m_tag = tag; }
void IFC4X3::IfcCurveBoundedSurface::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCCURVEBOUNDEDSURFACE" << "(";
	if( m_BasisSurface ) { stream << "#" << m_BasisSurface->m_tag; } else { stream << "$"; }
	stream << ",";
	writeEntityList( stream, m_Boundaries );
	stream << ",";
	if( m_ImplicitOuter ) { m_ImplicitOuter->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcCurveBoundedSurface::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcCurveBoundedSurface::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 3 ){ std::stringstream err; err << "Wrong parameter count for entity IfcCurveBoundedSurface, expecting 3, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	readEntityReference( args[0], m_BasisSurface, map, errorStream );
	readEntityReferenceList( args[1], m_Boundaries, map, errorStream );
	m_ImplicitOuter = IfcBoolean::createObjectFromSTEP( args[2], map, errorStream );
}
void IFC4X3::IfcCurveBoundedSurface::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcBoundedSurface::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "BasisSurface", m_BasisSurface ) );
	shared_ptr<AttributeObjectVector> Boundaries_vec_object( new AttributeObjectVector() );
	std::copy( m_Boundaries.begin(), m_Boundaries.end(), std::back_inserter( Boundaries_vec_object->m_vec ) );
	vec_attributes.emplace_back( std::make_pair( "Boundaries", Boundaries_vec_object ) );
	vec_attributes.emplace_back( std::make_pair( "ImplicitOuter", m_ImplicitOuter ) );
}
void IFC4X3::IfcCurveBoundedSurface::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcBoundedSurface::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcCurveBoundedSurface::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcBoundedSurface::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcCurveBoundedSurface::unlinkFromInverseCounterparts()
{
	IfcBoundedSurface::unlinkFromInverseCounterparts();
}
