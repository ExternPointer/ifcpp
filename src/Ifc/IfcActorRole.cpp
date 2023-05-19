/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcActorRole.h"
#include "ifcpp/Ifc/IfcExternalReferenceRelationship.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcRoleEnum.h"
#include "ifcpp/Ifc/IfcText.h"

// ENTITY IfcActorRole 
IFC4X3::IfcActorRole::IfcActorRole( int tag ) { m_tag = tag; }
void IFC4X3::IfcActorRole::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCACTORROLE" << "(";
	if( m_Role ) { m_Role->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_UserDefinedRole ) { m_UserDefinedRole->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Description ) { m_Description->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcActorRole::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcActorRole::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 3 ){ std::stringstream err; err << "Wrong parameter count for entity IfcActorRole, expecting 3, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_Role = IfcRoleEnum::createObjectFromSTEP( args[0], map, errorStream );
	m_UserDefinedRole = IfcLabel::createObjectFromSTEP( args[1], map, errorStream );
	m_Description = IfcText::createObjectFromSTEP( args[2], map, errorStream );
}
void IFC4X3::IfcActorRole::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	vec_attributes.emplace_back( std::make_pair( "Role", m_Role ) );
	vec_attributes.emplace_back( std::make_pair( "UserDefinedRole", m_UserDefinedRole ) );
	vec_attributes.emplace_back( std::make_pair( "Description", m_Description ) );
}
void IFC4X3::IfcActorRole::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	shared_ptr<AttributeObjectVector> HasExternalReference_inverse_vec_obj( new AttributeObjectVector() );
	for( size_t i=0; i<m_HasExternalReference_inverse.size(); ++i )
	{
		if( !m_HasExternalReference_inverse[i].expired() )
		{
			HasExternalReference_inverse_vec_obj->m_vec.emplace_back( shared_ptr<IfcExternalReferenceRelationship>( m_HasExternalReference_inverse[i] ) );
		}
	}
	vec_attributes_inverse.emplace_back( std::make_pair( "HasExternalReference_inverse", HasExternalReference_inverse_vec_obj ) );
}
void IFC4X3::IfcActorRole::setInverseCounterparts( shared_ptr<BuildingEntity> )
{
}
void IFC4X3::IfcActorRole::unlinkFromInverseCounterparts()
{
}
