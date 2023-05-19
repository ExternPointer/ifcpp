/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcDefinitionSelect.h"
#include "ifcpp/Ifc/IfcGloballyUniqueId.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcObjectDefinition.h"
#include "ifcpp/Ifc/IfcOwnerHistory.h"
#include "ifcpp/Ifc/IfcPropertyDefinition.h"
#include "ifcpp/Ifc/IfcRelAssociates.h"
#include "ifcpp/Ifc/IfcText.h"

// ENTITY IfcRelAssociates 
IFC4X3::IfcRelAssociates::IfcRelAssociates( int tag ) { m_tag = tag; }
void IFC4X3::IfcRelAssociates::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCRELASSOCIATES" << "(";
	if( m_GlobalId ) { m_GlobalId->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_OwnerHistory ) { stream << "#" << m_OwnerHistory->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Description ) { m_Description->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	stream << "(";
	for( size_t ii = 0; ii < m_RelatedObjects.size(); ++ii )
	{
		if( ii > 0 )
		{
			stream << ",";
		}
		const shared_ptr<IfcDefinitionSelect>& type_object = m_RelatedObjects[ii];
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
	stream << ");";
}
void IFC4X3::IfcRelAssociates::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcRelAssociates::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 5 ){ std::stringstream err; err << "Wrong parameter count for entity IfcRelAssociates, expecting 5, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_GlobalId = IfcGloballyUniqueId::createObjectFromSTEP( args[0], map, errorStream );
	readEntityReference( args[1], m_OwnerHistory, map, errorStream );
	m_Name = IfcLabel::createObjectFromSTEP( args[2], map, errorStream );
	m_Description = IfcText::createObjectFromSTEP( args[3], map, errorStream );
	readSelectList( args[4], m_RelatedObjects, map, errorStream );
}
void IFC4X3::IfcRelAssociates::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcRelationship::getAttributes( vec_attributes );
	shared_ptr<AttributeObjectVector> RelatedObjects_vec_object( new AttributeObjectVector() );
	std::copy( m_RelatedObjects.begin(), m_RelatedObjects.end(), std::back_inserter( RelatedObjects_vec_object->m_vec ) );
	vec_attributes.emplace_back( std::make_pair( "RelatedObjects", RelatedObjects_vec_object ) );
}
void IFC4X3::IfcRelAssociates::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcRelationship::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcRelAssociates::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcRelationship::setInverseCounterparts( ptr_self_entity );
	shared_ptr<IfcRelAssociates> ptr_self = dynamic_pointer_cast<IfcRelAssociates>( ptr_self_entity );
	if( !ptr_self ) { throw BuildingException( "IfcRelAssociates::setInverseCounterparts: type mismatch" ); }
	for( size_t i=0; i<m_RelatedObjects.size(); ++i )
	{
		shared_ptr<IfcObjectDefinition>  RelatedObjects_IfcObjectDefinition = dynamic_pointer_cast<IfcObjectDefinition>( m_RelatedObjects[i] );
		if( RelatedObjects_IfcObjectDefinition )
		{
			RelatedObjects_IfcObjectDefinition->m_HasAssociations_inverse.emplace_back( ptr_self );
		}
		shared_ptr<IfcPropertyDefinition>  RelatedObjects_IfcPropertyDefinition = dynamic_pointer_cast<IfcPropertyDefinition>( m_RelatedObjects[i] );
		if( RelatedObjects_IfcPropertyDefinition )
		{
			RelatedObjects_IfcPropertyDefinition->m_HasAssociations_inverse.emplace_back( ptr_self );
		}
	}
}
void IFC4X3::IfcRelAssociates::unlinkFromInverseCounterparts()
{
	IfcRelationship::unlinkFromInverseCounterparts();
	for( size_t i=0; i<m_RelatedObjects.size(); ++i )
	{
		shared_ptr<IfcObjectDefinition>  RelatedObjects_IfcObjectDefinition = dynamic_pointer_cast<IfcObjectDefinition>( m_RelatedObjects[i] );
		if( RelatedObjects_IfcObjectDefinition )
		{
			std::vector<weak_ptr<IfcRelAssociates> >& HasAssociations_inverse = RelatedObjects_IfcObjectDefinition->m_HasAssociations_inverse;
			for( auto it_HasAssociations_inverse = HasAssociations_inverse.begin(); it_HasAssociations_inverse != HasAssociations_inverse.end(); )
			{
				weak_ptr<IfcRelAssociates> self_candidate_weak = *it_HasAssociations_inverse;
				if( self_candidate_weak.expired() )
				{
					++it_HasAssociations_inverse;
					continue;
				}
				shared_ptr<IfcRelAssociates> self_candidate( *it_HasAssociations_inverse );
				if( self_candidate.get() == this )
				{
					it_HasAssociations_inverse= HasAssociations_inverse.erase( it_HasAssociations_inverse );
				}
				else
				{
					++it_HasAssociations_inverse;
				}
			}
		}
		shared_ptr<IfcPropertyDefinition>  RelatedObjects_IfcPropertyDefinition = dynamic_pointer_cast<IfcPropertyDefinition>( m_RelatedObjects[i] );
		if( RelatedObjects_IfcPropertyDefinition )
		{
			std::vector<weak_ptr<IfcRelAssociates> >& HasAssociations_inverse = RelatedObjects_IfcPropertyDefinition->m_HasAssociations_inverse;
			for( auto it_HasAssociations_inverse = HasAssociations_inverse.begin(); it_HasAssociations_inverse != HasAssociations_inverse.end(); )
			{
				weak_ptr<IfcRelAssociates> self_candidate_weak = *it_HasAssociations_inverse;
				if( self_candidate_weak.expired() )
				{
					++it_HasAssociations_inverse;
					continue;
				}
				shared_ptr<IfcRelAssociates> self_candidate( *it_HasAssociations_inverse );
				if( self_candidate.get() == this )
				{
					it_HasAssociations_inverse= HasAssociations_inverse.erase( it_HasAssociations_inverse );
				}
				else
				{
					++it_HasAssociations_inverse;
				}
			}
		}
	}
}
