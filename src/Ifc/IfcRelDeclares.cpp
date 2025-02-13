/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcContext.h"
#include "ifcpp/Ifc/IfcDefinitionSelect.h"
#include "ifcpp/Ifc/IfcGloballyUniqueId.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcObjectDefinition.h"
#include "ifcpp/Ifc/IfcOwnerHistory.h"
#include "ifcpp/Ifc/IfcPropertyDefinition.h"
#include "ifcpp/Ifc/IfcRelDeclares.h"
#include "ifcpp/Ifc/IfcText.h"

// ENTITY IfcRelDeclares 
IFC4X3::IfcRelDeclares::IfcRelDeclares( int tag ) { m_tag = tag; }
void IFC4X3::IfcRelDeclares::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCRELDECLARES" << "(";
	if( m_GlobalId ) { m_GlobalId->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_OwnerHistory ) { stream << "#" << m_OwnerHistory->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Description ) { m_Description->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_RelatingContext ) { stream << "#" << m_RelatingContext->m_tag; } else { stream << "$"; }
	stream << ",";
	stream << "(";
	for( size_t ii = 0; ii < m_RelatedDefinitions.size(); ++ii )
	{
		if( ii > 0 )
		{
			stream << ",";
		}
		const shared_ptr<IfcDefinitionSelect>& type_object = m_RelatedDefinitions[ii];
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
void IFC4X3::IfcRelDeclares::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcRelDeclares::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 6 ){ std::stringstream err; err << "Wrong parameter count for entity IfcRelDeclares, expecting 6, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_GlobalId = IfcGloballyUniqueId::createObjectFromSTEP( args[0], map, errorStream );
	readEntityReference( args[1], m_OwnerHistory, map, errorStream );
	m_Name = IfcLabel::createObjectFromSTEP( args[2], map, errorStream );
	m_Description = IfcText::createObjectFromSTEP( args[3], map, errorStream );
	readEntityReference( args[4], m_RelatingContext, map, errorStream );
	readSelectList( args[5], m_RelatedDefinitions, map, errorStream );
}
void IFC4X3::IfcRelDeclares::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcRelationship::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "RelatingContext", m_RelatingContext ) );
	shared_ptr<AttributeObjectVector> RelatedDefinitions_vec_object( new AttributeObjectVector() );
	std::copy( m_RelatedDefinitions.begin(), m_RelatedDefinitions.end(), std::back_inserter( RelatedDefinitions_vec_object->m_vec ) );
	vec_attributes.emplace_back( std::make_pair( "RelatedDefinitions", RelatedDefinitions_vec_object ) );
}
void IFC4X3::IfcRelDeclares::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcRelationship::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcRelDeclares::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcRelationship::setInverseCounterparts( ptr_self_entity );
	shared_ptr<IfcRelDeclares> ptr_self = dynamic_pointer_cast<IfcRelDeclares>( ptr_self_entity );
	if( !ptr_self ) { throw BuildingException( "IfcRelDeclares::setInverseCounterparts: type mismatch" ); }
	for( size_t i=0; i<m_RelatedDefinitions.size(); ++i )
	{
		shared_ptr<IfcObjectDefinition>  RelatedDefinitions_IfcObjectDefinition = dynamic_pointer_cast<IfcObjectDefinition>( m_RelatedDefinitions[i] );
		if( RelatedDefinitions_IfcObjectDefinition )
		{
			RelatedDefinitions_IfcObjectDefinition->m_HasContext_inverse.emplace_back( ptr_self );
		}
		shared_ptr<IfcPropertyDefinition>  RelatedDefinitions_IfcPropertyDefinition = dynamic_pointer_cast<IfcPropertyDefinition>( m_RelatedDefinitions[i] );
		if( RelatedDefinitions_IfcPropertyDefinition )
		{
			RelatedDefinitions_IfcPropertyDefinition->m_HasContext_inverse.emplace_back( ptr_self );
		}
	}
	if( m_RelatingContext )
	{
		m_RelatingContext->m_Declares_inverse.emplace_back( ptr_self );
	}
}
void IFC4X3::IfcRelDeclares::unlinkFromInverseCounterparts()
{
	IfcRelationship::unlinkFromInverseCounterparts();
	for( size_t i=0; i<m_RelatedDefinitions.size(); ++i )
	{
		shared_ptr<IfcObjectDefinition>  RelatedDefinitions_IfcObjectDefinition = dynamic_pointer_cast<IfcObjectDefinition>( m_RelatedDefinitions[i] );
		if( RelatedDefinitions_IfcObjectDefinition )
		{
			std::vector<weak_ptr<IfcRelDeclares> >& HasContext_inverse = RelatedDefinitions_IfcObjectDefinition->m_HasContext_inverse;
			for( auto it_HasContext_inverse = HasContext_inverse.begin(); it_HasContext_inverse != HasContext_inverse.end(); )
			{
				weak_ptr<IfcRelDeclares> self_candidate_weak = *it_HasContext_inverse;
				if( self_candidate_weak.expired() )
				{
					++it_HasContext_inverse;
					continue;
				}
				shared_ptr<IfcRelDeclares> self_candidate( *it_HasContext_inverse );
				if( self_candidate.get() == this )
				{
					it_HasContext_inverse= HasContext_inverse.erase( it_HasContext_inverse );
				}
				else
				{
					++it_HasContext_inverse;
				}
			}
		}
		shared_ptr<IfcPropertyDefinition>  RelatedDefinitions_IfcPropertyDefinition = dynamic_pointer_cast<IfcPropertyDefinition>( m_RelatedDefinitions[i] );
		if( RelatedDefinitions_IfcPropertyDefinition )
		{
			std::vector<weak_ptr<IfcRelDeclares> >& HasContext_inverse = RelatedDefinitions_IfcPropertyDefinition->m_HasContext_inverse;
			for( auto it_HasContext_inverse = HasContext_inverse.begin(); it_HasContext_inverse != HasContext_inverse.end(); )
			{
				weak_ptr<IfcRelDeclares> self_candidate_weak = *it_HasContext_inverse;
				if( self_candidate_weak.expired() )
				{
					++it_HasContext_inverse;
					continue;
				}
				shared_ptr<IfcRelDeclares> self_candidate( *it_HasContext_inverse );
				if( self_candidate.get() == this )
				{
					it_HasContext_inverse= HasContext_inverse.erase( it_HasContext_inverse );
				}
				else
				{
					++it_HasContext_inverse;
				}
			}
		}
	}
	if( m_RelatingContext )
	{
		std::vector<weak_ptr<IfcRelDeclares> >& Declares_inverse = m_RelatingContext->m_Declares_inverse;
		for( auto it_Declares_inverse = Declares_inverse.begin(); it_Declares_inverse != Declares_inverse.end(); )
		{
			weak_ptr<IfcRelDeclares> self_candidate_weak = *it_Declares_inverse;
			if( self_candidate_weak.expired() )
			{
				++it_Declares_inverse;
				continue;
			}
			shared_ptr<IfcRelDeclares> self_candidate( *it_Declares_inverse );
			if( self_candidate.get() == this )
			{
				it_Declares_inverse= Declares_inverse.erase( it_Declares_inverse );
			}
			else
			{
				++it_Declares_inverse;
			}
		}
	}
}
