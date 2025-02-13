/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcExternalReferenceRelationship.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcProfileDef.h"
#include "ifcpp/Ifc/IfcProfileProperties.h"
#include "ifcpp/Ifc/IfcProfileTypeEnum.h"

// ENTITY IfcProfileDef 
IFC4X3::IfcProfileDef::IfcProfileDef( int tag ) { m_tag = tag; }
void IFC4X3::IfcProfileDef::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCPROFILEDEF" << "(";
	if( m_ProfileType ) { m_ProfileType->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ProfileName ) { m_ProfileName->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcProfileDef::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcProfileDef::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 2 ){ std::stringstream err; err << "Wrong parameter count for entity IfcProfileDef, expecting 2, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_ProfileType = IfcProfileTypeEnum::createObjectFromSTEP( args[0], map, errorStream );
	m_ProfileName = IfcLabel::createObjectFromSTEP( args[1], map, errorStream );
}
void IFC4X3::IfcProfileDef::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	vec_attributes.emplace_back( std::make_pair( "ProfileType", m_ProfileType ) );
	vec_attributes.emplace_back( std::make_pair( "ProfileName", m_ProfileName ) );
}
void IFC4X3::IfcProfileDef::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
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
	shared_ptr<AttributeObjectVector> HasProperties_inverse_vec_obj( new AttributeObjectVector() );
	for( size_t i=0; i<m_HasProperties_inverse.size(); ++i )
	{
		if( !m_HasProperties_inverse[i].expired() )
		{
			HasProperties_inverse_vec_obj->m_vec.emplace_back( shared_ptr<IfcProfileProperties>( m_HasProperties_inverse[i] ) );
		}
	}
	vec_attributes_inverse.emplace_back( std::make_pair( "HasProperties_inverse", HasProperties_inverse_vec_obj ) );
}
void IFC4X3::IfcProfileDef::setInverseCounterparts( shared_ptr<BuildingEntity> )
{
}
void IFC4X3::IfcProfileDef::unlinkFromInverseCounterparts()
{
}
