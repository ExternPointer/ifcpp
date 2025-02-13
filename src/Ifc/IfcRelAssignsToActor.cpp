/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcActor.h"
#include "ifcpp/Ifc/IfcActorRole.h"
#include "ifcpp/Ifc/IfcGloballyUniqueId.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcObjectDefinition.h"
#include "ifcpp/Ifc/IfcObjectTypeEnum.h"
#include "ifcpp/Ifc/IfcOwnerHistory.h"
#include "ifcpp/Ifc/IfcRelAssignsToActor.h"
#include "ifcpp/Ifc/IfcText.h"

// ENTITY IfcRelAssignsToActor 
IFC4X3::IfcRelAssignsToActor::IfcRelAssignsToActor( int tag ) { m_tag = tag; }
void IFC4X3::IfcRelAssignsToActor::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCRELASSIGNSTOACTOR" << "(";
	if( m_GlobalId ) { m_GlobalId->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_OwnerHistory ) { stream << "#" << m_OwnerHistory->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Description ) { m_Description->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	writeEntityList( stream, m_RelatedObjects );
	stream << ",";
	if( m_RelatedObjectsType ) { m_RelatedObjectsType->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_RelatingActor ) { stream << "#" << m_RelatingActor->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_ActingRole ) { stream << "#" << m_ActingRole->m_tag; } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcRelAssignsToActor::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcRelAssignsToActor::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 8 ){ std::stringstream err; err << "Wrong parameter count for entity IfcRelAssignsToActor, expecting 8, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_GlobalId = IfcGloballyUniqueId::createObjectFromSTEP( args[0], map, errorStream );
	readEntityReference( args[1], m_OwnerHistory, map, errorStream );
	m_Name = IfcLabel::createObjectFromSTEP( args[2], map, errorStream );
	m_Description = IfcText::createObjectFromSTEP( args[3], map, errorStream );
	readEntityReferenceList( args[4], m_RelatedObjects, map, errorStream );
	m_RelatedObjectsType = IfcObjectTypeEnum::createObjectFromSTEP( args[5], map, errorStream );
	readEntityReference( args[6], m_RelatingActor, map, errorStream );
	readEntityReference( args[7], m_ActingRole, map, errorStream );
}
void IFC4X3::IfcRelAssignsToActor::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcRelAssigns::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "RelatingActor", m_RelatingActor ) );
	vec_attributes.emplace_back( std::make_pair( "ActingRole", m_ActingRole ) );
}
void IFC4X3::IfcRelAssignsToActor::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcRelAssigns::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcRelAssignsToActor::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcRelAssigns::setInverseCounterparts( ptr_self_entity );
	shared_ptr<IfcRelAssignsToActor> ptr_self = dynamic_pointer_cast<IfcRelAssignsToActor>( ptr_self_entity );
	if( !ptr_self ) { throw BuildingException( "IfcRelAssignsToActor::setInverseCounterparts: type mismatch" ); }
	if( m_RelatingActor )
	{
		m_RelatingActor->m_IsActingUpon_inverse.emplace_back( ptr_self );
	}
}
void IFC4X3::IfcRelAssignsToActor::unlinkFromInverseCounterparts()
{
	IfcRelAssigns::unlinkFromInverseCounterparts();
	if( m_RelatingActor )
	{
		std::vector<weak_ptr<IfcRelAssignsToActor> >& IsActingUpon_inverse = m_RelatingActor->m_IsActingUpon_inverse;
		for( auto it_IsActingUpon_inverse = IsActingUpon_inverse.begin(); it_IsActingUpon_inverse != IsActingUpon_inverse.end(); )
		{
			weak_ptr<IfcRelAssignsToActor> self_candidate_weak = *it_IsActingUpon_inverse;
			if( self_candidate_weak.expired() )
			{
				++it_IsActingUpon_inverse;
				continue;
			}
			shared_ptr<IfcRelAssignsToActor> self_candidate( *it_IsActingUpon_inverse );
			if( self_candidate.get() == this )
			{
				it_IsActingUpon_inverse= IsActingUpon_inverse.erase( it_IsActingUpon_inverse );
			}
			else
			{
				++it_IsActingUpon_inverse;
			}
		}
	}
}
