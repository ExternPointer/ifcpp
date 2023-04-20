/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcAxis2Placement.h"
#include "ifcpp/Ifc/IfcPresentableText.h"
#include "ifcpp/Ifc/IfcPresentationLayerAssignment.h"
#include "ifcpp/Ifc/IfcStyledItem.h"
#include "ifcpp/Ifc/IfcTextLiteral.h"
#include "ifcpp/Ifc/IfcTextPath.h"

// ENTITY IfcTextLiteral 
IFC4X3::IfcTextLiteral::IfcTextLiteral( int tag ) { m_tag = tag; }
void IFC4X3::IfcTextLiteral::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCTEXTLITERAL" << "(";
	if( m_Literal ) { m_Literal->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Placement ) { m_Placement->getStepParameter( stream, true ); } else { stream << "$" ; }
	stream << ",";
	if( m_Path ) { m_Path->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcTextLiteral::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcTextLiteral::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 3 ){ std::stringstream err; err << "Wrong parameter count for entity IfcTextLiteral, expecting 3, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_Literal = IfcPresentableText::createObjectFromSTEP( args[0], map, errorStream );
	m_Placement = IfcAxis2Placement::createObjectFromSTEP( args[1], map, errorStream );
	m_Path = IfcTextPath::createObjectFromSTEP( args[2], map, errorStream );
}
void IFC4X3::IfcTextLiteral::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcGeometricRepresentationItem::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "Literal", m_Literal ) );
	vec_attributes.emplace_back( std::make_pair( "Placement", m_Placement ) );
	vec_attributes.emplace_back( std::make_pair( "Path", m_Path ) );
}
void IFC4X3::IfcTextLiteral::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcGeometricRepresentationItem::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcTextLiteral::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcGeometricRepresentationItem::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcTextLiteral::unlinkFromInverseCounterparts()
{
	IfcGeometricRepresentationItem::unlinkFromInverseCounterparts();
}