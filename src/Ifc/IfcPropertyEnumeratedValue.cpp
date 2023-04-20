/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcComplexProperty.h"
#include "ifcpp/Ifc/IfcExternalReferenceRelationship.h"
#include "ifcpp/Ifc/IfcIdentifier.h"
#include "ifcpp/Ifc/IfcPropertyDependencyRelationship.h"
#include "ifcpp/Ifc/IfcPropertyEnumeratedValue.h"
#include "ifcpp/Ifc/IfcPropertyEnumeration.h"
#include "ifcpp/Ifc/IfcPropertySet.h"
#include "ifcpp/Ifc/IfcResourceApprovalRelationship.h"
#include "ifcpp/Ifc/IfcResourceConstraintRelationship.h"
#include "ifcpp/Ifc/IfcText.h"
#include "ifcpp/Ifc/IfcValue.h"

// ENTITY IfcPropertyEnumeratedValue 
IFC4X3::IfcPropertyEnumeratedValue::IfcPropertyEnumeratedValue( int tag ) { m_tag = tag; }
void IFC4X3::IfcPropertyEnumeratedValue::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCPROPERTYENUMERATEDVALUE" << "(";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Specification ) { m_Specification->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_EnumerationValues.size() > 0 )
	{
		stream << "(";
		for( size_t ii = 0; ii < m_EnumerationValues.size(); ++ii )
		{
			if( ii > 0 )
			{
				stream << ",";
			}
			const shared_ptr<IfcValue>& type_object = m_EnumerationValues[ii];
			if( type_object )
			{
				type_object->getStepParameter( stream, true );
			}
			else
			{
				stream << "$";
			}
		}
		stream << ")";
	}
	else { stream << "$"; }
	stream << ",";
	if( m_EnumerationReference ) { stream << "#" << m_EnumerationReference->m_tag; } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcPropertyEnumeratedValue::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcPropertyEnumeratedValue::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 4 ){ std::stringstream err; err << "Wrong parameter count for entity IfcPropertyEnumeratedValue, expecting 4, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_Name = IfcIdentifier::createObjectFromSTEP( args[0], map, errorStream );
	m_Specification = IfcText::createObjectFromSTEP( args[1], map, errorStream );
	readSelectList( args[2], m_EnumerationValues, map, errorStream );
	readEntityReference( args[3], m_EnumerationReference, map, errorStream );
}
void IFC4X3::IfcPropertyEnumeratedValue::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcSimpleProperty::getAttributes( vec_attributes );
	if( !m_EnumerationValues.empty() )
	{
		shared_ptr<AttributeObjectVector> EnumerationValues_vec_object( new AttributeObjectVector() );
		std::copy( m_EnumerationValues.begin(), m_EnumerationValues.end(), std::back_inserter( EnumerationValues_vec_object->m_vec ) );
		vec_attributes.emplace_back( std::make_pair( "EnumerationValues", EnumerationValues_vec_object ) );
	}
	vec_attributes.emplace_back( std::make_pair( "EnumerationReference", m_EnumerationReference ) );
}
void IFC4X3::IfcPropertyEnumeratedValue::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcSimpleProperty::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcPropertyEnumeratedValue::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcSimpleProperty::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcPropertyEnumeratedValue::unlinkFromInverseCounterparts()
{
	IfcSimpleProperty::unlinkFromInverseCounterparts();
}