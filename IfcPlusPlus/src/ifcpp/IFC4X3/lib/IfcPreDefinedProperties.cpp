/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/model/AttributeObject.h"
#include "ifcpp/model/BuildingException.h"
#include "ifcpp/model/BuildingGuid.h"
#include "ifcpp/reader/ReaderUtil.h"
#include "ifcpp/writer/WriterUtil.h"
#include "ifcpp/IFC4X3/include/IfcExternalReferenceRelationship.h"
#include "ifcpp/IFC4X3/include/IfcPreDefinedProperties.h"

// ENTITY IfcPreDefinedProperties 
IFC4X3::IfcPreDefinedProperties::IfcPreDefinedProperties( int tag ) { m_tag = tag; }
void IFC4X3::IfcPreDefinedProperties::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCPREDEFINEDPROPERTIES" << "(";
	stream << ");";
}
void IFC4X3::IfcPreDefinedProperties::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcPreDefinedProperties::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
}
void IFC4X3::IfcPreDefinedProperties::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcPropertyAbstraction::getAttributes( vec_attributes );
}
void IFC4X3::IfcPreDefinedProperties::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcPropertyAbstraction::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcPreDefinedProperties::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcPropertyAbstraction::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcPreDefinedProperties::unlinkFromInverseCounterparts()
{
	IfcPropertyAbstraction::unlinkFromInverseCounterparts();
}
