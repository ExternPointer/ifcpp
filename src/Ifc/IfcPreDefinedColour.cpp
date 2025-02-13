/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcPreDefinedColour.h"

// ENTITY IfcPreDefinedColour 
IFC4X3::IfcPreDefinedColour::IfcPreDefinedColour( int tag ) { m_tag = tag; }
void IFC4X3::IfcPreDefinedColour::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCPREDEFINEDCOLOUR" << "(";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcPreDefinedColour::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcPreDefinedColour::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 1 ){ std::stringstream err; err << "Wrong parameter count for entity IfcPreDefinedColour, expecting 1, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_Name = IfcLabel::createObjectFromSTEP( args[0], map, errorStream );
}
void IFC4X3::IfcPreDefinedColour::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcPreDefinedItem::getAttributes( vec_attributes );
}
void IFC4X3::IfcPreDefinedColour::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcPreDefinedItem::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcPreDefinedColour::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcPreDefinedItem::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcPreDefinedColour::unlinkFromInverseCounterparts()
{
	IfcPreDefinedItem::unlinkFromInverseCounterparts();
}
