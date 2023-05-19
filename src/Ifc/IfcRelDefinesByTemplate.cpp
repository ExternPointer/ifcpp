/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcGloballyUniqueId.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcOwnerHistory.h"
#include "ifcpp/Ifc/IfcPropertySetDefinition.h"
#include "ifcpp/Ifc/IfcPropertySetTemplate.h"
#include "ifcpp/Ifc/IfcRelDefinesByTemplate.h"
#include "ifcpp/Ifc/IfcText.h"

// ENTITY IfcRelDefinesByTemplate 
IFC4X3::IfcRelDefinesByTemplate::IfcRelDefinesByTemplate( int tag ) { m_tag = tag; }
void IFC4X3::IfcRelDefinesByTemplate::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCRELDEFINESBYTEMPLATE" << "(";
	if( m_GlobalId ) { m_GlobalId->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_OwnerHistory ) { stream << "#" << m_OwnerHistory->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Description ) { m_Description->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	writeEntityList( stream, m_RelatedPropertySets );
	stream << ",";
	if( m_RelatingTemplate ) { stream << "#" << m_RelatingTemplate->m_tag; } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcRelDefinesByTemplate::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcRelDefinesByTemplate::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 6 ){ std::stringstream err; err << "Wrong parameter count for entity IfcRelDefinesByTemplate, expecting 6, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_GlobalId = IfcGloballyUniqueId::createObjectFromSTEP( args[0], map, errorStream );
	readEntityReference( args[1], m_OwnerHistory, map, errorStream );
	m_Name = IfcLabel::createObjectFromSTEP( args[2], map, errorStream );
	m_Description = IfcText::createObjectFromSTEP( args[3], map, errorStream );
	readEntityReferenceList( args[4], m_RelatedPropertySets, map, errorStream );
	readEntityReference( args[5], m_RelatingTemplate, map, errorStream );
}
void IFC4X3::IfcRelDefinesByTemplate::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcRelDefines::getAttributes( vec_attributes );
	shared_ptr<AttributeObjectVector> RelatedPropertySets_vec_object( new AttributeObjectVector() );
	std::copy( m_RelatedPropertySets.begin(), m_RelatedPropertySets.end(), std::back_inserter( RelatedPropertySets_vec_object->m_vec ) );
	vec_attributes.emplace_back( std::make_pair( "RelatedPropertySets", RelatedPropertySets_vec_object ) );
	vec_attributes.emplace_back( std::make_pair( "RelatingTemplate", m_RelatingTemplate ) );
}
void IFC4X3::IfcRelDefinesByTemplate::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcRelDefines::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcRelDefinesByTemplate::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcRelDefines::setInverseCounterparts( ptr_self_entity );
	shared_ptr<IfcRelDefinesByTemplate> ptr_self = dynamic_pointer_cast<IfcRelDefinesByTemplate>( ptr_self_entity );
	if( !ptr_self ) { throw BuildingException( "IfcRelDefinesByTemplate::setInverseCounterparts: type mismatch" ); }
	for( size_t i=0; i<m_RelatedPropertySets.size(); ++i )
	{
		if( m_RelatedPropertySets[i] )
		{
			m_RelatedPropertySets[i]->m_IsDefinedBy_inverse.emplace_back( ptr_self );
		}
	}
	if( m_RelatingTemplate )
	{
		m_RelatingTemplate->m_Defines_inverse.emplace_back( ptr_self );
	}
}
void IFC4X3::IfcRelDefinesByTemplate::unlinkFromInverseCounterparts()
{
	IfcRelDefines::unlinkFromInverseCounterparts();
	for( size_t i=0; i<m_RelatedPropertySets.size(); ++i )
	{
		if( m_RelatedPropertySets[i] )
		{
			std::vector<weak_ptr<IfcRelDefinesByTemplate> >& IsDefinedBy_inverse = m_RelatedPropertySets[i]->m_IsDefinedBy_inverse;
			for( auto it_IsDefinedBy_inverse = IsDefinedBy_inverse.begin(); it_IsDefinedBy_inverse != IsDefinedBy_inverse.end(); )
			{
				weak_ptr<IfcRelDefinesByTemplate> self_candidate_weak = *it_IsDefinedBy_inverse;
				if( self_candidate_weak.expired() )
				{
					++it_IsDefinedBy_inverse;
					continue;
				}
				shared_ptr<IfcRelDefinesByTemplate> self_candidate( *it_IsDefinedBy_inverse );
				if( self_candidate.get() == this )
				{
					it_IsDefinedBy_inverse= IsDefinedBy_inverse.erase( it_IsDefinedBy_inverse );
				}
				else
				{
					++it_IsDefinedBy_inverse;
				}
			}
		}
	}
	if( m_RelatingTemplate )
	{
		std::vector<weak_ptr<IfcRelDefinesByTemplate> >& Defines_inverse = m_RelatingTemplate->m_Defines_inverse;
		for( auto it_Defines_inverse = Defines_inverse.begin(); it_Defines_inverse != Defines_inverse.end(); )
		{
			weak_ptr<IfcRelDefinesByTemplate> self_candidate_weak = *it_Defines_inverse;
			if( self_candidate_weak.expired() )
			{
				++it_Defines_inverse;
				continue;
			}
			shared_ptr<IfcRelDefinesByTemplate> self_candidate( *it_Defines_inverse );
			if( self_candidate.get() == this )
			{
				it_Defines_inverse= Defines_inverse.erase( it_Defines_inverse );
			}
			else
			{
				++it_Defines_inverse;
			}
		}
	}
}
