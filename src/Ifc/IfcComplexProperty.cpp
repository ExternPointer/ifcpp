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
#include "ifcpp/Ifc/IfcProperty.h"
#include "ifcpp/Ifc/IfcPropertyDependencyRelationship.h"
#include "ifcpp/Ifc/IfcPropertySet.h"
#include "ifcpp/Ifc/IfcResourceApprovalRelationship.h"
#include "ifcpp/Ifc/IfcResourceConstraintRelationship.h"
#include "ifcpp/Ifc/IfcText.h"

// ENTITY IfcComplexProperty 
IFC4X3::IfcComplexProperty::IfcComplexProperty( int tag ) { m_tag = tag; }
void IFC4X3::IfcComplexProperty::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCCOMPLEXPROPERTY" << "(";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Specification ) { m_Specification->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_UsageName ) { m_UsageName->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	writeEntityList( stream, m_HasProperties );
	stream << ");";
}
void IFC4X3::IfcComplexProperty::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcComplexProperty::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 4 ){ std::stringstream err; err << "Wrong parameter count for entity IfcComplexProperty, expecting 4, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_Name = IfcIdentifier::createObjectFromSTEP( args[0], map, errorStream );
	m_Specification = IfcText::createObjectFromSTEP( args[1], map, errorStream );
	m_UsageName = IfcIdentifier::createObjectFromSTEP( args[2], map, errorStream );
	readEntityReferenceList( args[3], m_HasProperties, map, errorStream );
}
void IFC4X3::IfcComplexProperty::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcProperty::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "UsageName", m_UsageName ) );
	shared_ptr<AttributeObjectVector> HasProperties_vec_object( new AttributeObjectVector() );
	std::copy( m_HasProperties.begin(), m_HasProperties.end(), std::back_inserter( HasProperties_vec_object->m_vec ) );
	vec_attributes.emplace_back( std::make_pair( "HasProperties", HasProperties_vec_object ) );
}
void IFC4X3::IfcComplexProperty::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcProperty::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcComplexProperty::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcProperty::setInverseCounterparts( ptr_self_entity );
	shared_ptr<IfcComplexProperty> ptr_self = dynamic_pointer_cast<IfcComplexProperty>( ptr_self_entity );
	if( !ptr_self ) { throw BuildingException( "IfcComplexProperty::setInverseCounterparts: type mismatch" ); }
	for( size_t i=0; i<m_HasProperties.size(); ++i )
	{
		if( m_HasProperties[i] )
		{
			m_HasProperties[i]->m_PartOfComplex_inverse.emplace_back( ptr_self );
		}
	}
}
void IFC4X3::IfcComplexProperty::unlinkFromInverseCounterparts()
{
	IfcProperty::unlinkFromInverseCounterparts();
	for( size_t i=0; i<m_HasProperties.size(); ++i )
	{
		if( m_HasProperties[i] )
		{
			std::vector<weak_ptr<IfcComplexProperty> >& PartOfComplex_inverse = m_HasProperties[i]->m_PartOfComplex_inverse;
			for( auto it_PartOfComplex_inverse = PartOfComplex_inverse.begin(); it_PartOfComplex_inverse != PartOfComplex_inverse.end(); )
			{
				weak_ptr<IfcComplexProperty> self_candidate_weak = *it_PartOfComplex_inverse;
				if( self_candidate_weak.expired() )
				{
					++it_PartOfComplex_inverse;
					continue;
				}
				shared_ptr<IfcComplexProperty> self_candidate( *it_PartOfComplex_inverse );
				if( self_candidate.get() == this )
				{
					it_PartOfComplex_inverse= PartOfComplex_inverse.erase( it_PartOfComplex_inverse );
				}
				else
				{
					++it_PartOfComplex_inverse;
				}
			}
		}
	}
}
