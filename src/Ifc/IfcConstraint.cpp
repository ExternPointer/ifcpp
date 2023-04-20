/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcActorSelect.h"
#include "ifcpp/Ifc/IfcConstraint.h"
#include "ifcpp/Ifc/IfcConstraintEnum.h"
#include "ifcpp/Ifc/IfcDateTime.h"
#include "ifcpp/Ifc/IfcExternalReferenceRelationship.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcResourceConstraintRelationship.h"
#include "ifcpp/Ifc/IfcText.h"

// ENTITY IfcConstraint 
IFC4X3::IfcConstraint::IfcConstraint( int tag ) { m_tag = tag; }
void IFC4X3::IfcConstraint::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCCONSTRAINT" << "(";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Description ) { m_Description->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ConstraintGrade ) { m_ConstraintGrade->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ConstraintSource ) { m_ConstraintSource->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_CreatingActor ) { m_CreatingActor->getStepParameter( stream, true ); } else { stream << "$" ; }
	stream << ",";
	if( m_CreationTime ) { m_CreationTime->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_UserDefinedGrade ) { m_UserDefinedGrade->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcConstraint::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcConstraint::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 7 ){ std::stringstream err; err << "Wrong parameter count for entity IfcConstraint, expecting 7, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_Name = IfcLabel::createObjectFromSTEP( args[0], map, errorStream );
	m_Description = IfcText::createObjectFromSTEP( args[1], map, errorStream );
	m_ConstraintGrade = IfcConstraintEnum::createObjectFromSTEP( args[2], map, errorStream );
	m_ConstraintSource = IfcLabel::createObjectFromSTEP( args[3], map, errorStream );
	m_CreatingActor = IfcActorSelect::createObjectFromSTEP( args[4], map, errorStream );
	m_CreationTime = IfcDateTime::createObjectFromSTEP( args[5], map, errorStream );
	m_UserDefinedGrade = IfcLabel::createObjectFromSTEP( args[6], map, errorStream );
}
void IFC4X3::IfcConstraint::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	vec_attributes.emplace_back( std::make_pair( "Name", m_Name ) );
	vec_attributes.emplace_back( std::make_pair( "Description", m_Description ) );
	vec_attributes.emplace_back( std::make_pair( "ConstraintGrade", m_ConstraintGrade ) );
	vec_attributes.emplace_back( std::make_pair( "ConstraintSource", m_ConstraintSource ) );
	vec_attributes.emplace_back( std::make_pair( "CreatingActor", m_CreatingActor ) );
	vec_attributes.emplace_back( std::make_pair( "CreationTime", m_CreationTime ) );
	vec_attributes.emplace_back( std::make_pair( "UserDefinedGrade", m_UserDefinedGrade ) );
}
void IFC4X3::IfcConstraint::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	if( !m_HasExternalReferences_inverse.empty() )
	{
		shared_ptr<AttributeObjectVector> HasExternalReferences_inverse_vec_obj( new AttributeObjectVector() );
		for( size_t i=0; i<m_HasExternalReferences_inverse.size(); ++i )
		{
			if( !m_HasExternalReferences_inverse[i].expired() )
			{
				HasExternalReferences_inverse_vec_obj->m_vec.emplace_back( shared_ptr<IfcExternalReferenceRelationship>( m_HasExternalReferences_inverse[i] ) );
			}
		}
		vec_attributes_inverse.emplace_back( std::make_pair( "HasExternalReferences_inverse", HasExternalReferences_inverse_vec_obj ) );
	}
	if( !m_PropertiesForConstraint_inverse.empty() )
	{
		shared_ptr<AttributeObjectVector> PropertiesForConstraint_inverse_vec_obj( new AttributeObjectVector() );
		for( size_t i=0; i<m_PropertiesForConstraint_inverse.size(); ++i )
		{
			if( !m_PropertiesForConstraint_inverse[i].expired() )
			{
				PropertiesForConstraint_inverse_vec_obj->m_vec.emplace_back( shared_ptr<IfcResourceConstraintRelationship>( m_PropertiesForConstraint_inverse[i] ) );
			}
		}
		vec_attributes_inverse.emplace_back( std::make_pair( "PropertiesForConstraint_inverse", PropertiesForConstraint_inverse_vec_obj ) );
	}
}
void IFC4X3::IfcConstraint::setInverseCounterparts( shared_ptr<BuildingEntity> )
{
}
void IFC4X3::IfcConstraint::unlinkFromInverseCounterparts()
{
}