/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcColour.h"
#include "ifcpp/Ifc/IfcTextStyleForDefinedFont.h"

// ENTITY IfcTextStyleForDefinedFont 
IFC4X3::IfcTextStyleForDefinedFont::IfcTextStyleForDefinedFont( int tag ) { m_tag = tag; }
void IFC4X3::IfcTextStyleForDefinedFont::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCTEXTSTYLEFORDEFINEDFONT" << "(";
	if( m_Colour ) { m_Colour->getStepParameter( stream, true ); } else { stream << "$" ; }
	stream << ",";
	if( m_BackgroundColour ) { m_BackgroundColour->getStepParameter( stream, true ); } else { stream << "$" ; }
	stream << ");";
}
void IFC4X3::IfcTextStyleForDefinedFont::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcTextStyleForDefinedFont::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 2 ){ std::stringstream err; err << "Wrong parameter count for entity IfcTextStyleForDefinedFont, expecting 2, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_Colour = IfcColour::createObjectFromSTEP( args[0], map, errorStream );
	m_BackgroundColour = IfcColour::createObjectFromSTEP( args[1], map, errorStream );
}
void IFC4X3::IfcTextStyleForDefinedFont::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcPresentationItem::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "Colour", m_Colour ) );
	vec_attributes.emplace_back( std::make_pair( "BackgroundColour", m_BackgroundColour ) );
}
void IFC4X3::IfcTextStyleForDefinedFont::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcPresentationItem::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcTextStyleForDefinedFont::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcPresentationItem::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcTextStyleForDefinedFont::unlinkFromInverseCounterparts()
{
	IfcPresentationItem::unlinkFromInverseCounterparts();
}