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
#include "ifcpp/Ifc/IfcCostValue.h"
#include "ifcpp/Ifc/IfcDate.h"
#include "ifcpp/Ifc/IfcExternalReferenceRelationship.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcMeasureWithUnit.h"
#include "ifcpp/Ifc/IfcText.h"

// ENTITY IfcCostValue 
IFC4X3::IfcCostValue::IfcCostValue( int tag ) { m_tag = tag; }
void IFC4X3::IfcCostValue::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCCOSTVALUE" << "(";
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
void IFC4X3::IfcCostValue::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcCostValue::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 10 ){ std::stringstream err; err << "Wrong parameter count for entity IfcCostValue, expecting 10, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
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
void IFC4X3::IfcCostValue::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcAppliedValue::getAttributes( vec_attributes );
}
void IFC4X3::IfcCostValue::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcAppliedValue::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcCostValue::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcAppliedValue::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcCostValue::unlinkFromInverseCounterparts()
{
	IfcAppliedValue::unlinkFromInverseCounterparts();
}