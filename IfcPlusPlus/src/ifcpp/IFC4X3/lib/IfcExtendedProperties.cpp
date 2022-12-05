/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/model/AttributeObject.h"
#include "ifcpp/model/BuildingException.h"
#include "ifcpp/model/BuildingGuid.h"
#include "ifcpp/reader/ReaderUtil.h"
#include "ifcpp/writer/WriterUtil.h"
#include "ifcpp/IFC4X3/include/IfcExtendedProperties.h"
#include "ifcpp/IFC4X3/include/IfcExternalReferenceRelationship.h"
#include "ifcpp/IFC4X3/include/IfcIdentifier.h"
#include "ifcpp/IFC4X3/include/IfcProperty.h"
#include "ifcpp/IFC4X3/include/IfcText.h"

// ENTITY IfcExtendedProperties 
IFC4X3::IfcExtendedProperties::IfcExtendedProperties( int tag ) { m_tag = tag; }
void IFC4X3::IfcExtendedProperties::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCEXTENDEDPROPERTIES" << "(";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Description ) { m_Description->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	writeEntityList( stream, m_Properties );
	stream << ");";
}
void IFC4X3::IfcExtendedProperties::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcExtendedProperties::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 3 ){ std::stringstream err; err << "Wrong parameter count for entity IfcExtendedProperties, expecting 3, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_Name = IfcIdentifier::createObjectFromSTEP( args[0], map, errorStream );
	m_Description = IfcText::createObjectFromSTEP( args[1], map, errorStream );
	readEntityReferenceList( args[2], m_Properties, map, errorStream );
}
void IFC4X3::IfcExtendedProperties::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcPropertyAbstraction::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "Name", m_Name ) );
	vec_attributes.emplace_back( std::make_pair( "Description", m_Description ) );
	if( !m_Properties.empty() )
	{
		shared_ptr<AttributeObjectVector> Properties_vec_object( new AttributeObjectVector() );
		std::copy( m_Properties.begin(), m_Properties.end(), std::back_inserter( Properties_vec_object->m_vec ) );
		vec_attributes.emplace_back( std::make_pair( "Properties", Properties_vec_object ) );
	}
}
void IFC4X3::IfcExtendedProperties::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcPropertyAbstraction::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcExtendedProperties::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcPropertyAbstraction::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcExtendedProperties::unlinkFromInverseCounterparts()
{
	IfcPropertyAbstraction::unlinkFromInverseCounterparts();
}
