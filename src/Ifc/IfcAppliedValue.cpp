/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcAppliedValue.h"
#include "ifcpp/Ifc/IfcAppliedValueSelect.h"
#include "ifcpp/Ifc/IfcArithmeticOperatorEnum.h"
#include "ifcpp/Ifc/IfcDate.h"
#include "ifcpp/Ifc/IfcExternalReferenceRelationship.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcMeasureWithUnit.h"
#include "ifcpp/Ifc/IfcText.h"

// ENTITY IfcAppliedValue 
IFC4X3::IfcAppliedValue::IfcAppliedValue( int tag ) { m_tag = tag; }
void IFC4X3::IfcAppliedValue::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCAPPLIEDVALUE" << "(";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Description ) { m_Description->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_AppliedValue ) { m_AppliedValue->getStepParameter( stream, true ); } else { stream << "$" ; }
	stream << ",";
	if( m_UnitBasis ) { stream << "#" << m_UnitBasis->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_ApplicableDate ) { m_ApplicableDate->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_FixedUntilDate ) { m_FixedUntilDate->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Category ) { m_Category->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Condition ) { m_Condition->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ArithmeticOperator ) { m_ArithmeticOperator->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	writeEntityList( stream, m_Components );
	stream << ");";
}
void IFC4X3::IfcAppliedValue::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcAppliedValue::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 10 ){ std::stringstream err; err << "Wrong parameter count for entity IfcAppliedValue, expecting 10, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_Name = IfcLabel::createObjectFromSTEP( args[0], map, errorStream );
	m_Description = IfcText::createObjectFromSTEP( args[1], map, errorStream );
	m_AppliedValue = IfcAppliedValueSelect::createObjectFromSTEP( args[2], map, errorStream );
	readEntityReference( args[3], m_UnitBasis, map, errorStream );
	m_ApplicableDate = IfcDate::createObjectFromSTEP( args[4], map, errorStream );
	m_FixedUntilDate = IfcDate::createObjectFromSTEP( args[5], map, errorStream );
	m_Category = IfcLabel::createObjectFromSTEP( args[6], map, errorStream );
	m_Condition = IfcLabel::createObjectFromSTEP( args[7], map, errorStream );
	m_ArithmeticOperator = IfcArithmeticOperatorEnum::createObjectFromSTEP( args[8], map, errorStream );
	readEntityReferenceList( args[9], m_Components, map, errorStream );
}
void IFC4X3::IfcAppliedValue::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	vec_attributes.emplace_back( std::make_pair( "Name", m_Name ) );
	vec_attributes.emplace_back( std::make_pair( "Description", m_Description ) );
	vec_attributes.emplace_back( std::make_pair( "AppliedValue", m_AppliedValue ) );
	vec_attributes.emplace_back( std::make_pair( "UnitBasis", m_UnitBasis ) );
	vec_attributes.emplace_back( std::make_pair( "ApplicableDate", m_ApplicableDate ) );
	vec_attributes.emplace_back( std::make_pair( "FixedUntilDate", m_FixedUntilDate ) );
	vec_attributes.emplace_back( std::make_pair( "Category", m_Category ) );
	vec_attributes.emplace_back( std::make_pair( "Condition", m_Condition ) );
	vec_attributes.emplace_back( std::make_pair( "ArithmeticOperator", m_ArithmeticOperator ) );
	if( !m_Components.empty() )
	{
		shared_ptr<AttributeObjectVector> Components_vec_object( new AttributeObjectVector() );
		std::copy( m_Components.begin(), m_Components.end(), std::back_inserter( Components_vec_object->m_vec ) );
		vec_attributes.emplace_back( std::make_pair( "Components", Components_vec_object ) );
	}
}
void IFC4X3::IfcAppliedValue::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	if( !m_HasExternalReference_inverse.empty() )
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
}
void IFC4X3::IfcAppliedValue::setInverseCounterparts( shared_ptr<BuildingEntity> )
{
}
void IFC4X3::IfcAppliedValue::unlinkFromInverseCounterparts()
{
}