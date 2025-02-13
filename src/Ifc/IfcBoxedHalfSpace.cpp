/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcBoolean.h"
#include "ifcpp/Ifc/IfcBoundingBox.h"
#include "ifcpp/Ifc/IfcBoxedHalfSpace.h"
#include "ifcpp/Ifc/IfcPresentationLayerAssignment.h"
#include "ifcpp/Ifc/IfcStyledItem.h"
#include "ifcpp/Ifc/IfcSurface.h"

// ENTITY IfcBoxedHalfSpace 
IFC4X3::IfcBoxedHalfSpace::IfcBoxedHalfSpace( int tag ) { m_tag = tag; }
void IFC4X3::IfcBoxedHalfSpace::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCBOXEDHALFSPACE" << "(";
	if( m_BaseSurface ) { stream << "#" << m_BaseSurface->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_AgreementFlag ) { m_AgreementFlag->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Enclosure ) { stream << "#" << m_Enclosure->m_tag; } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcBoxedHalfSpace::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcBoxedHalfSpace::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 3 ){ std::stringstream err; err << "Wrong parameter count for entity IfcBoxedHalfSpace, expecting 3, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	readEntityReference( args[0], m_BaseSurface, map, errorStream );
	m_AgreementFlag = IfcBoolean::createObjectFromSTEP( args[1], map, errorStream );
	readEntityReference( args[2], m_Enclosure, map, errorStream );
}
void IFC4X3::IfcBoxedHalfSpace::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcHalfSpaceSolid::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "Enclosure", m_Enclosure ) );
}
void IFC4X3::IfcBoxedHalfSpace::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcHalfSpaceSolid::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcBoxedHalfSpace::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcHalfSpaceSolid::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcBoxedHalfSpace::unlinkFromInverseCounterparts()
{
	IfcHalfSpaceSolid::unlinkFromInverseCounterparts();
}
